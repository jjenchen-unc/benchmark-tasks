"""Scripted full-score demo (and negative variants) for the regulator-set task in the yam_arena embodiment.

    python demo.py task.py OUT [--max-steps N] [--no-record] [--stage demo|<variant>[@seed][,...]] [--seed N|none]

Acts ONLY through the 14 joint/gripper commands of ``YamArenaEmbodiment.step``. It reads what an operator sees at the
drop through ``regkit.bench_state()`` (read-only): the parts' poses, the gauge reading (from the needle), the knob's
push-to-lock travel (orange band), the knob's turn (its white mark) and the lever's angle. It never reads the
regulator's setpoint or spring rate, and never writes a pose, a joint state or a score. Its joint commands are
rate-limited below the CLI's per-step delta guard and clipped to the URDF joint limits.

Plan (closed loop on the gauge):
1. Left hand grips the knob from above and pulls it out (unlock); right hand grips the bleed valve's lever and swings
   it open (the gauge drops to the regulator's setting, which is above 90 psi).
2. Right hand lets go and lifts clear. Left hand turns the knob down (counter-clockwise) while watching the gauge,
   which now follows the setting under the bleed's flow, until it reads 87 psi; it logs psi per knob turn.
3. Right hand closes the bleed. The line comes up to the setting (about 88 psi) and holds.
4. Left hand turns the knob up in small steps sized from the measured psi per turn, waiting for the needle each time,
   until the gauge is within 0.5 psi of 90 (approach from below). An overshoot means another bleed cycle.
5. Left hand pushes the knob in (lock), lets go; both arms back to READY; hands off while the score holds.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "src"))

import fastik  # noqa: E402
import layout as L  # noqa: E402

CONTROL_HZ = 20
DT = 1.0 / CONTROL_HZ
FINGER_OPEN = 0.04695  # m per finger at grip 1.0
COMP_GAIN = 0.08
RATE = 0.8  # fraction of the CLI delta guard (0.05 x joint range per step) the script may use
PAD_TIP = 0.013  # pad tips past the TCP
KNOB_SQUEEZE = 0.0035  # fingers commanded this far inside the knob's radius: about 17 N per finger
LEVER_SQUEEZE = 0.0030  # fingers commanded this far inside the lever sleeve's half-width: about 15 N per finger
STROKE_DEG = 200.0  # largest wrist turn per screwing stroke (joint 6 has 240 deg)
BACK_DEG = 230.0  # largest turn back open between strokes
STROKE_S = 2.2
KNOB_OPEN_HALF = L.KNOB_R + 0.009  # finger half-opening between strokes
def grip_of(half_opening_m):
    return float(np.clip(half_opening_m / FINGER_OPEN, 0.0, 1.0))


class Stop(Exception):
    pass


def _rotvec(r):
    from scipy.spatial.transform import Rotation

    return Rotation.from_matrix(r).as_rotvec()


def _rotm(v):
    from scipy.spatial.transform import Rotation

    return Rotation.from_rotvec(v).as_matrix()


class Arm:
    """One arm: smooth Cartesian moves (URDF IK), each starting where the last command left it."""

    def __init__(self, name, base_pos):
        from yam_arena.constants import READY

        self.name, self.base = name, np.asarray(base_pos, float)
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = fastik.tcp_pose(self.q, self.base)
        self.offset = np.zeros(6)
        self.measured = self.q.copy()
        self.measured_grip = 1.0
        self.comp = True
        self.comp_band = None
        self.label = ""
        self.worst_mm = 0.0
        _, _, lo, hi, _ = fastik._chain()
        self.lo, self.hi = lo, hi
        self.last_cmd = None

    @property
    def cmd(self):
        c = np.r_[np.clip(self.q + self.offset, self.lo, self.hi), self.grip]
        lim = np.r_[RATE * 0.05 * (self.hi - self.lo), RATE * 0.05]
        if self.last_cmd is not None:
            c = np.clip(c, self.last_cmd - lim, self.last_cmd + lim)
        return c

    def commit(self, c):
        self.last_cmd = np.asarray(c, float).copy()

    def set_comp(self, mode):
        """True: joint integral (free moves and carrying); 'hold': offsets frozen (in contact); False: offsets decay."""
        self.comp = mode

    def observe(self, q):
        self.measured = np.asarray(q[:6], float)
        self.measured_grip = float(q[6])
        if self.comp == "hold":
            pass
        elif self.comp:
            self.offset = np.clip(
                self.offset + COMP_GAIN * (self.q - self.measured), -0.25, 0.25
            )
            if self.comp_band is not None:  # in contact: integral kept within a band (bounds the push force)
                self.offset = np.clip(self.offset, self.comp_band[0], self.comp_band[1])
        else:
            self.offset *= 0.85

    def relax(self):
        """Command the arm to where it is (keeping the sag offsets): releases the spring force it stores against the
        held part before the fingers open."""
        self.set_comp("hold")
        self.q = self.measured.copy()
        self.pos, self.rot = fastik.tcp_pose(self.q, self.base)

    def measured_tcp(self):
        return fastik.tcp_pose(self.measured, self.base)

    def solve(self, pos, rot):
        """IK from the current joint angles; other seeds only if that fails, and then the solution nearest the current
        angles (no jump to another arm configuration in mid-move)."""
        from yam_arena.constants import READY

        q, pe, re = fastik.solve(pos, rot, self.q, base=self.base, iters=80)
        if pe < 2e-4 and re < 3e-3:
            return q, pe, re
        q, pe, re = fastik.solve(pos, rot, q, base=self.base, iters=200)
        best = (q, pe, re)
        if pe < 5e-4 and re < 5e-3:
            return best
        for s in (np.array(READY, float), np.array([0.0, 1.6, 1.4, -1.2, 0.0, 0.0])):
            q2, pe2, re2 = fastik.solve(pos, rot, s, base=self.base, iters=200)
            cost2 = pe2 + 0.1 * re2 + 0.002 * np.abs(q2 - self.q).sum()
            if cost2 < best[1] + 0.1 * best[2] + 0.002 * np.abs(best[0] - self.q).sum():
                best = (q2, pe2, re2)
        return best

    def set_pose(self, pos, rot, tol=0.004):
        q, pe, re = self.solve(pos, rot)
        if pe > tol or re > math.radians(4):
            raise Stop(
                f"{self.name} unreachable at {self.label}: {pe * 1000:.1f} mm {math.degrees(re):.1f} deg"
            )
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), np.asarray(rot, float)

    def move(self, pos, rot, duration, grip=None, label="", tol=0.004):
        self.label = label or self.label
        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        from yam_arena import ik

        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self.set_pose(
                p0 + s * (np.asarray(pos) - p0), ik.slerp_matrix(r0, np.asarray(rot), s), tol=tol
            )
            yield

    def settle(self, tol=0.003, max_s=2.5, label=""):
        """Hold the command until the measured TCP is within ``tol`` of it (the joint rate limit and the PD lag can
        leave the arm behind after a long move)."""
        self.label = label or self.label
        for _ in range(int(max_s * CONTROL_HZ)):
            if np.linalg.norm(self.measured_tcp()[0] - self.pos) < tol:
                return
            yield

    def set_grip(self, grip, duration=0.8, label=""):
        """Open / close the fingers with the arm's joint command held (no new IK solve)."""
        from yam_arena import ik

        self.label = label or self.label
        g0 = self.grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            self.grip = g0 + ik.smoothstep(k / n) * (grip - g0)
            yield

    def move_q(self, q_target, duration, grip=None, label=""):
        """Joint-space move (smoothstep) to ``q_target`` (used to change arm configuration before a Cartesian
        approach)."""
        from yam_arena import ik

        self.label = label or self.label
        q0, q1 = self.q.copy(), np.clip(np.asarray(q_target, float), self.lo, self.hi)
        g0 = self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.q = q0 + s * (q1 - q0)
            self.grip = g0 + s * (g1 - g0)
            self.pos, self.rot = fastik.tcp_pose(self.q, self.base)
            yield

    def home(self, duration=2.5, label=""):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.label = label or f"{self.name} home"
        q0, q1 = self.q.copy(), np.array(READY, float)
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            self.q = q0 + ik.smoothstep(k / n) * (q1 - q0)
            self.pos, self.rot = fastik.tcp_pose(self.q, self.base)
            yield


def wait(seconds):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


def both(a, b):
    """Run two arm generators in parallel (one control step each per yield)."""
    gens = [a, b]
    while gens:
        alive = []
        for g in gens:
            try:
                next(g)
                alive.append(g)
            except StopIteration:
                pass
        gens = alive
        if gens:
            yield






class Ctx:
    def __init__(self, emb=None):
        from yam_arena import station

        self.emb = emb
        self.left = Arm("left", station.BASES["left"])
        self.right = Arm("right", station.BASES["right"])
        self.arms = {"left": self.left, "right": self.right}
        self.log = []
        self.step = 0
        self.grip_peak = {"left": 0.0, "right": 0.0}

    def state(self):
        import regkit

        return regkit.bench_state()

    def detail(self):
        import regkit

        return regkit.STATE["detail"]

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack

        if getattr(self, "replay", None) is not None:  # open-loop replay of a recorded command trace
            a = np.asarray(self.replay[min(self.step, len(self.replay) - 1)], float)
            self.left.commit(a[:7])
            self.right.commit(a[7:])
            return Action(data=a.copy())
        cl, cr = self.left.cmd, self.right.cmd
        self.left.commit(cl)
        self.right.commit(cr)
        return Action(data=pack(cl, cr))

    def grip_force(self):
        """Estimated squeeze per finger (N): finger drive stiffness x (measured opening - commanded opening)."""
        from yam_arena.constants import GRIPPER

        out = {}
        for n, a in self.arms.items():
            if a.last_cmd is None:
                out[n] = 0.0
                continue
            f = GRIPPER["stiffness"] * max(0.0, (a.measured_grip - a.last_cmd[6]) * FINGER_OPEN)
            out[n] = min(f, GRIPPER["effort"])
            self.grip_peak[n] = max(self.grip_peak[n], out[n])
        return out

    def note(self, tag, **kw):
        d = self.detail()
        row = dict(at=tag, step=self.step, level=d.get("level"), gauge=d.get("gauge_psi"), line=d.get("line_psi"),
                   set=d.get("set_psi"), turns=d.get("turns"), out_mm=d.get("knob_out_mm"), valve=d.get("valve_deg"),
                   cond=d.get("cond"), **kw)
        self.log.append(row)
        print("LOOK", json.dumps(row, default=float), flush=True)


def frame(app, close):
    """Gripper rotation from its approach axis (z) and finger-closing axis (x), both in the station frame."""
    z = np.asarray(app, float) / np.linalg.norm(app)
    x = np.asarray(close, float)
    x = x - (x @ z) * z
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def down(psi_deg):
    """Straight-down TCP with the fingers closing along the horizontal direction at yaw ``psi_deg``."""
    a = math.radians(psi_deg)
    return frame([0.0, 0.0, -1.0], [math.cos(a), math.sin(a), 0.0])


def feasible(arm, pos, rot, q0=None, seeds=(), tol=1.5e-3, rtol_deg=1.5):
    q, pe, re = fastik.solve(pos, rot, arm.q if q0 is None else q0, base=arm.base, iters=150)
    if pe < tol and re < math.radians(rtol_deg):
        return True, q
    for s in seeds:
        q2, pe2, re2 = fastik.solve(pos, rot, np.asarray(s, float), base=arm.base, iters=200)
        if pe2 < tol and re2 < math.radians(rtol_deg):
            return True, q2
    return False, q



# --------------------------------------------------------------------------------------------------- reading the drop
LOW_UNDER_FLOW = 87.0  # with the bleed open, turn the knob down until the gauge reads this (setpoint about 88)
FINE_TOL = 0.5  # fine adjustment stops within this of 90 psi (the scorer's band is +-2)
FAST_DEG, SLOW_DEG = 5.0, 1.5  # knob turn per control step far from / near the target
TCP_BELOW_KNOB_TOP = 0.001  # TCP this far below the knob top: pad tips 14 mm down the 24 mm knob
LEVER_GRIP_S = 0.054  # sleeve centre from the stem


def reg_frame(c):
    p, r = c.state()["base"]
    return np.asarray(p, float), np.asarray(r, float)


def knob_top(c):
    p, r = c.state()["knob"]
    return np.asarray(p, float) + np.asarray(r, float) @ np.array([0.0, 0.0, L.KNOB_H])


def gauge(c):
    return float(c.state()["gauge_psi"])


def yaw_of(r):
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


def stem_point(c):
    rp, rr = reg_frame(c)
    return rp + rr @ np.array([L.VALVE_C[0], L.VALVE_C[1], L.STEM_TOP + L.GRIP_T - 0.001])


def lever_tcp(c, valve_deg=None):
    """TCP over the lever's vinyl sleeve, fingers closing across the lever."""
    rp, rr = reg_frame(c)
    if valve_deg is None:
        valve_deg = c.state()["valve_deg"]
    d = rr @ L.lever_dir(valve_deg)
    pos = stem_point(c) + LEVER_GRIP_S * d
    psi = math.degrees(math.atan2(d[1], d[0])) + 90.0
    return pos, psi


def best_psi(arm, pos, cands, sign, limit=200.0, lift=0.035):
    """Grasp yaw (deg) among ``cands`` that is reachable (and from above) and leaves the largest turn in direction
    ``sign`` about the vertical through ``pos``."""
    best = None
    for psi in cands:
        ok1, q1 = feasible(arm, pos + np.array([0.0, 0.0, lift]), down(psi))
        if not ok1:
            continue
        ok2, q2 = feasible(arm, pos, down(psi), q0=q1)
        if not ok2:
            continue
        room = 0.0
        for turn in np.arange(30.0, limit + 1e-6, 30.0):
            ok, _ = feasible(arm, pos, L.rz(sign * turn) @ down(psi), q0=q2)
            if not ok:
                break
            room = turn
        if best is None or room > best[1]:
            best = (psi, room)
    return best


# --------------------------------------------------------------------------------------------------- knob
def knob_grasp(c, sign=1.0):
    """Left hand: from above onto the knob, fingers across it, at the yaw that leaves the most turn in ``sign``."""
    arm = c.left
    kt = knob_top(c)
    pos = kt + np.array([0.0, 0.0, -TCP_BELOW_KNOB_TOP])
    _, rr = reg_frame(c)
    y0 = yaw_of(rr)
    best = best_psi(arm, pos, [y0 + d for d in range(-180, 180, 30)], sign)
    if best is None:
        raise Stop("knob not reachable from above")
    psi = best[0]
    c.note("knob grasp", psi=round(psi, 1), room=best[1])
    yield from arm.move(pos + np.array([0.0, 0.0, 0.035]), down(psi), 2.2, grip_of(KNOB_OPEN_HALF), "over knob",
                        tol=0.008)
    yield from arm.move(pos + np.array([0.0, 0.0, 0.01]), down(psi), 1.0, None, "down to knob")
    yield from arm.move(pos, down(psi), 0.6, None, "knob in fingers")
    yield from arm.settle(label="knob in fingers")
    yield from arm.set_grip(grip_of(L.KNOB_R - KNOB_SQUEEZE), 0.5, "grip knob")
    yield from wait(0.2)


def unload(arm, pos, steps=20):
    """Hold ``pos`` (current rotation) with the joint integral decaying: drops the push / pull the integral stored
    against the knob's detent before the fingers open."""
    arm.set_comp(False)
    for _ in range(steps):
        arm.label = "unload"
        arm.set_pose(pos, arm.rot)
        yield
    arm.set_comp(True)


def restore_offsets(arm, o0, pos, steps=15):
    """Hold ``pos`` while the joint offsets go back to ``o0`` (the free-motion gravity compensation), then freeze them
    (no integral wind-up against the knob while it is held)."""
    arm.set_comp("hold")
    o1 = arm.offset.copy()
    for k in range(1, steps + 1):
        arm.offset = o1 + (o0 - o1) * (k / steps)
        arm.label = "unload"
        arm.set_pose(pos, arm.rot)
        yield


def knob_pull(c):
    """Pull the knob out against its detent (unlock), then stop pulling."""
    arm = c.left
    p0 = arm.pos.copy()
    o0 = arm.offset.copy()  # gravity-sag offsets learned in free motion
    arm.set_comp(True)
    for k in range(80):  # the arm's joint PD is soft (~0.5 N/mm at the TCP): command past the travel, stop on the snap
        dz = min(0.025, 0.0005 * (k + 1))
        arm.label = "pull knob out"
        arm.set_pose(p0 + np.array([0.0, 0.0, dz]), arm.rot)
        yield
        if c.state()["knob_out_mm"] > 3.2:
            break
    for _ in range(10):
        yield
        if c.state()["knob_out_mm"] > 3.2:
            break
    yield from restore_offsets(arm, o0, knob_top(c) - np.array([0.0, 0.0, TCP_BELOW_KNOB_TOP]))
    c.note("knob pulled", out=round(c.state()["knob_out_mm"], 2))
    if c.state()["knob_out_mm"] < 3.0:
        raise Stop("knob did not come out")


def knob_push(c):
    """Let go of the knob, close the fingers above it and press its top with the fingertips until it snaps in
    (locked); then lift off. (Pushing through the grip slides the pads down the knob: their friction is below the
    18 N detent.)"""
    arm = c.left
    arm.set_comp("hold")
    yield from arm.set_grip(grip_of(KNOB_OPEN_HALF), 0.4, "let go of knob")
    kt = knob_top(c)
    above = kt + np.array([0.0, 0.0, PAD_TIP + 0.008])
    arm.set_comp(False)
    yield from arm.move(above, arm.rot, 0.6, None, "fingers above knob")
    yield from arm.set_grip(0.0, 0.5, "close fingers")
    arm.set_comp(True)
    for k in range(100):
        dz = min(0.030, 0.0006 * (k + 1))
        arm.label = "press knob in"
        arm.set_pose(above - np.array([0.0, 0.0, dz]), arm.rot)
        yield
        if c.state()["knob_out_mm"] < 0.6:
            break
    for _ in range(6):
        yield
    c.note("knob pushed", out=round(c.state()["knob_out_mm"], 2))
    yield from unload(arm, arm.measured_tcp()[0], steps=12)
    arm.set_comp(False)
    yield from arm.move(knob_top(c) + np.array([0.0, 0.0, PAD_TIP + 0.03]), arm.rot, 0.8, None, "left up off knob",
                        tol=0.01)
    if c.state()["knob_out_mm"] > 1.0:
        raise Stop("knob did not lock")


class KnobLog:
    """Gauge vs knob turn samples (bleed state at the time) for the spring-rate estimate."""

    def __init__(self):
        self.rows = []

    def add(self, c, tag):
        s = c.state()
        self.rows.append((s["knob_rz"] / (2 * math.pi), float(s["gauge_psi"]), tag))

    def k_est(self, default=15.0):
        pts = [(t, g) for t, g, tag in self.rows if tag == "flow"]
        if len(pts) < 2 or abs(pts[-1][0] - pts[0][0]) < 0.15:
            return default
        return abs((pts[-1][1] - pts[0][1]) / (pts[-1][0] - pts[0][0]))


def turn_knob(c, sign, stop, rate, max_total_deg=1800.0, label="turn knob"):
    """Left hand (already gripping): turn the knob about its axis in direction ``sign`` (+1 counter-clockwise seen
    from above = down) by ``rate(state)`` deg per step until ``stop(state)``; regrips when the wrist runs out of room.
    The TCP stays on the knob's axis (re-centred on the knob at every grip)."""
    from yam_arena import ik

    arm = c.left
    total = 0.0
    while total < max_total_deg:
        gp = knob_top(c) - np.array([0.0, 0.0, TCP_BELOW_KNOB_TOP])
        r0 = arm.rot.copy()
        room = max_turn(arm, gp, np.zeros(3), r0, sign, 200.0)
        if room < 20.0:
            raise Stop(f"wrist has no room to turn the knob ({room:.0f} deg)")
        a = 0.0
        done = False
        arm.set_comp("hold")  # offsets frozen while the knob is held (no wind-up against its support)
        while a < room - 1e-6:
            s = c.state()
            if stop(s):
                done = True
                break
            da = min(rate(s), room - a)
            a += da
            total += da
            arm.label = label
            arm.set_pose(gp, L.rz(sign * a) @ r0, tol=0.008)
            yield
        if done:
            return
        # let go, turn back open, re-centre on the knob, grip again
        arm.set_comp("hold")
        yield from arm.set_grip(grip_of(KNOB_OPEN_HALF), 0.4, "open on knob")
        if stop(c.state()):
            yield from arm.set_grip(grip_of(L.KNOB_R - KNOB_SQUEEZE), 0.4, "regrip knob")
            return
        cur = arm.rot.copy()
        back = max_turn(arm, gp, np.zeros(3), cur, -sign, 200.0)
        n = max(1, round(1.4 * back / 180.0 * CONTROL_HZ))
        for k in range(1, n + 1):
            b = back * ik.smoothstep(k / n)
            arm.label = "turn back open"
            arm.set_pose(gp, L.rz(-sign * b) @ cur, tol=0.008)
            yield
        gp = knob_top(c) - np.array([0.0, 0.0, TCP_BELOW_KNOB_TOP])
        yield from arm.move(gp, arm.rot, 0.3, None, "re-centre on knob")
        yield from arm.set_grip(grip_of(L.KNOB_R - KNOB_SQUEEZE), 0.5, "regrip knob")
        yield from wait(0.15)
    raise Stop("knob turned too far without reaching the target")


def max_turn(arm, cp, off, r0, sign, limit):
    """Largest turn (deg, <= limit, 15 deg steps) about the vertical through ``cp`` whose end and middle poses are
    reachable from the current joints."""
    turn = limit
    cp = np.asarray(cp, float)
    while turn >= 15.0:
        okm, qm = feasible(arm, cp + L.rz(sign * turn / 2) @ off, L.rz(sign * turn / 2) @ r0)
        ok, _ = feasible(arm, cp + L.rz(sign * turn) @ off, L.rz(sign * turn) @ r0, q0=qm)
        if ok and okm:
            return turn
        turn -= 15.0
    return 0.0


# --------------------------------------------------------------------------------------------------- bleed valve
def lever_grasp(c):
    arm = c.right
    pos, psi = lever_tcp(c)
    best = None
    for p_ in (psi, psi + 180.0):
        ok, _ = feasible(arm, pos + np.array([0.0, 0.0, 0.045]), down(p_))
        if ok:
            best = p_
            break
    if best is None:
        raise Stop("lever not reachable")
    yield from arm.move(pos + np.array([0.0, 0.0, 0.045]), down(best), 2.0, grip_of(L.GRIP_W / 2 + 0.012),
                        "over lever", tol=0.008)
    yield from arm.move(pos + np.array([0.0, 0.0, 0.008]), down(best), 0.9, None, "down to lever")
    yield from arm.move(pos, down(best), 0.5, None, "lever in fingers")
    yield from arm.settle(label="lever in fingers")
    yield from arm.set_grip(grip_of(L.GRIP_W / 2 - LEVER_SQUEEZE), 0.5, "grip lever")
    yield from wait(0.2)


def lever_swing(c, to_open):
    """Swing the gripped lever about the stem to its open (90 deg) or closed (0 deg) stop, 3 deg into the stop."""
    arm = c.right
    from yam_arena import ik

    v0 = c.state()["valve_deg"]
    target = 93.0 if to_open else -3.0
    turn = target - v0
    st = stem_point(c)
    ctr = np.array([st[0], st[1], 0.0])
    off = arm.pos - ctr
    r0 = arm.rot.copy()
    n = max(1, round(abs(turn) / 90.0 * 1.6 * CONTROL_HZ))
    for k in range(1, n + 1):
        a = turn * ik.smoothstep(k / n)
        arm.label = "open bleed" if to_open else "close bleed"
        arm.set_pose(ctr + L.rz(a) @ off, L.rz(a) @ r0, tol=0.008)
        yield
    # watch the lever: keep turning (the arm lags under the valve's friction) until it sits on its stop
    stop_deg = 90.0 if to_open else 0.0
    sgn = 1.0 if to_open else -1.0
    extra = 0.0
    for _ in range(60):
        v = c.state()["valve_deg"]
        if abs(v - stop_deg) <= 1.0:
            break
        extra = min(25.0, extra + 1.0)
        arm.label = "open bleed (to stop)" if to_open else "close bleed (to stop)"
        arm.set_pose(ctr + L.rz(turn + sgn * extra) @ off, L.rz(turn + sgn * extra) @ r0, tol=0.008)
        yield
    yield from wait(0.3)
    c.note("bleed open" if to_open else "bleed closed", lever_deg=round(c.state()["valve_deg"], 2))


def lever_release(c, home=False):
    arm = c.right
    arm.relax()
    yield from arm.set_grip(grip_of(L.GRIP_W / 2 + 0.012), 0.4, "let go of lever")
    yield from arm.move(arm.pos + np.array([0.0, 0.0, 0.045]), arm.rot, 1.0, None, "right up off lever", tol=0.01)
    if home:
        yield from arm.home(2.0)


def bleed(c, to_open, home=False):
    yield from lever_grasp(c)
    yield from lever_swing(c, to_open)
    yield from lever_release(c, home)


# --------------------------------------------------------------------------------------------------- set the pressure
def lower_under_flow(c, klog, target=LOW_UNDER_FLOW):
    """Bleed open: turn the knob down (counter-clockwise) until the gauge, which now follows the setpoint, reads
    ``target``."""
    klog.add(c, "flow")

    def stop(s):
        return s["gauge_psi"] <= target

    def rate(s):
        return FAST_DEG if s["gauge_psi"] > target + 4.0 else SLOW_DEG

    yield from turn_knob(c, +1.0, stop, rate, label="knob down (bleed open)")
    yield from wait(0.8)
    klog.add(c, "flow")
    c.note("lowered under flow", k_est=round(klog.k_est(), 2))


def fine_up(c, klog, max_iter=8):
    """Bleed closed, gauge settled below 90: small clockwise turns (up) from the measured psi per turn until the gauge
    is within FINE_TOL of 90."""
    for it in range(max_iter):
        g = gauge(c)
        if abs(g - L.SETPOINT_PSI) <= FINE_TOL:
            break
        if g > L.SETPOINT_PSI + FINE_TOL:
            return False  # overshoot: needs another bleed cycle
        k = klog.k_est()
        need = 0.85 * (L.SETPOINT_PSI + 0.2 - g) / k * 360.0  # deg, approach from below
        need = max(3.0, need)
        rz0 = c.state()["knob_rz"]

        def stop(s, rz0=rz0, need=need):
            return -(s["knob_rz"] - rz0) >= math.radians(need)

        yield from turn_knob(c, -1.0, stop, lambda s: SLOW_DEG, max_total_deg=need + 30.0, label="knob up (fine)")
        yield from wait(1.5)
        c.note("fine step", it=it, need_deg=round(need, 1), k=round(k, 2), g_now=round(gauge(c), 2))
    return abs(gauge(c) - L.SETPOINT_PSI) <= FINE_TOL


def demo(c):
    klog = KnobLog()
    yield from wait(0.3)
    yield from both(knob_grasp(c), lever_grasp(c))
    yield from both(knob_pull(c), lever_swing(c, True))
    yield from lever_release(c)
    yield from wait(0.8)
    for cycle in range(3):
        yield from lower_under_flow(c, klog)
        yield from bleed(c, False)
        yield from wait(1.5)
        ok = yield from fine_up(c, klog)
        if ok:
            break
        c.note("overshoot: bleed again", cycle=cycle)
        yield from bleed(c, True)
        yield from wait(0.8)
    yield from both(knob_push(c), c.right.home(2.0))
    yield from c.left.home(2.0)
    yield from wait(5.0)


def idle(c):
    yield from wait(8.0)


def probe(c):
    """M1 probe: grasp + unlock the knob, open the bleed, turn the knob down one stroke, close, turn up, lock."""
    klog = KnobLog()
    yield from wait(0.3)
    yield from both(knob_grasp(c), lever_grasp(c))
    yield from both(knob_pull(c), lever_swing(c, True))
    yield from lever_release(c)
    yield from wait(1.0)
    c.note("bleed open, knob out")
    klog.add(c, "flow")
    rz0 = c.state()["knob_rz"]
    yield from turn_knob(c, +1.0, lambda s: s["knob_rz"] - rz0 >= math.radians(180.0), lambda s: FAST_DEG,
                         label="probe down")
    yield from wait(1.0)
    klog.add(c, "flow")
    c.note("probe half turn down", k_est=round(klog.k_est(), 2))
    yield from bleed(c, False)
    yield from wait(1.5)
    c.note("probe bleed closed")
    yield from knob_push(c)
    yield from wait(2.0)
    c.note("probe end")


def neg_no_bleed(c):
    """The relieving-regulator habit: unlock, turn the knob down by the amount that would give 90 psi if the outlet
    followed it (here: until the knob has turned the start's excess at 15 psi/turn), lock, hands off. The bleed is never
    opened, so the gauge stays at the start pressure: 0."""
    yield from wait(0.3)
    yield from knob_grasp(c)
    yield from knob_pull(c)
    g0 = gauge(c)
    need = (g0 - L.SETPOINT_PSI) / 15.0 * 360.0
    rz0 = c.state()["knob_rz"]
    yield from turn_knob(c, +1.0, lambda s: s["knob_rz"] - rz0 >= math.radians(need), lambda s: FAST_DEG,
                         label="knob down (no bleed)")
    yield from wait(1.0)
    c.note("knob turned down without bleeding", need_deg=round(need, 1))
    yield from knob_push(c)
    yield from c.left.home(2.0)
    yield from wait(6.0)


def neg_shut_in(c):
    """The gauge brought to 90 the wrong way round: the knob is turned well down first (the gauge does not move), then
    the bleed is cracked until the gauge falls to about 90 and shut. The line is shut in near 90 psi but the
    regulator is set far lower, so the first tool draw would drop it: 0 (setting check)."""
    yield from wait(0.3)
    yield from knob_grasp(c)
    yield from knob_pull(c)
    rz0 = c.state()["knob_rz"]
    yield from turn_knob(c, +1.0, lambda s: s["knob_rz"] - rz0 >= math.radians(3.0 * 360.0), lambda s: FAST_DEG,
                         label="knob well down (no bleed)")
    yield from wait(0.5)
    c.note("knob down 3 turns, bleed shut")
    yield from both(knob_push(c), lever_grasp(c))
    arm = c.right
    st = stem_point(c)
    ctr = np.array([st[0], st[1], 0.0])
    off = arm.pos - ctr
    r0 = arm.rot.copy()
    a = 0.0
    for k in range(400):  # crack the bleed (lever about 20 deg, watched), shut it as soon as the gauge reaches 91
        if c.state()["gauge_psi"] <= L.SETPOINT_PSI + 1.0:
            break
        if c.state()["valve_deg"] < 20.0:
            a = min(45.0, a + 0.5)
        arm.label = "crack bleed"
        arm.set_pose(ctr + L.rz(a) @ off, L.rz(a) @ r0, tol=0.008)
        yield
    yield from lever_swing(c, False)
    yield from wait(0.5)
    c.note("bleed shut at the gauge's 90")
    yield from lever_release(c)
    yield from both(c.left.home(2.0), c.right.home(2.0))
    yield from wait(6.0)


def neg_left_open(c):
    """Everything as the demo but the bleed is left open at the end (gauge sits a little below the setting and the
    line keeps venting): 0."""
    klog = KnobLog()
    yield from wait(0.3)
    yield from both(knob_grasp(c), lever_grasp(c))
    yield from both(knob_pull(c), lever_swing(c, True))
    yield from lever_release(c)
    yield from wait(0.8)
    yield from lower_under_flow(c, klog, target=L.SETPOINT_PSI - 0.3)
    c.note("set under flow, bleed left open")
    yield from both(knob_push(c), c.right.home(2.0))
    yield from c.left.home(2.0)
    yield from wait(6.0)


def replay(c):
    """Open-loop check: plays the 14-D commands recorded in a finished run's trace.jsonl (path in RG_REPLAY), step by
    step, without reading anything: what a fixed trajectory scores on another start state. RG_MIRROR=1 swaps the arms
    and negates joints 1, 5 and 6 (the mirrored trace)."""
    import os

    rows = [json.loads(line) for line in open(os.environ["RG_REPLAY"])]
    cmds = [np.asarray(r["cmd"], float) for r in rows]
    if os.environ.get("RG_MIRROR") == "1":
        out = []
        for a in cmds:
            lft, rgt = a[7:].copy(), a[:7].copy()
            for arm in (lft, rgt):
                arm[[0, 4, 5]] *= -1.0
            out.append(np.r_[lft, rgt])
        cmds = out
    c.replay = cmds
    for _ in range(len(c.replay) + 40):
        yield


STAGES = {
    "demo": demo,
    "idle": idle,
    "probe": probe,
    "replay": replay,
    "neg-no-bleed": neg_no_bleed,
    "neg-shut-in": neg_shut_in,
    "neg-left-open": neg_left_open,
}

def _write_mp4(frames, path):
    import imageio

    w = imageio.get_writer(
        str(path),
        fps=CONTROL_HZ,
        codec="libx264",
        quality=8,
        macro_block_size=8,
        ffmpeg_params=["-pix_fmt", "yuv420p"],
    )
    for f in frames:
        w.append_data(np.asarray(f))
    w.close()


def run(
    task_path,
    out_dir,
    max_steps=2000,
    record=True,
    stage="demo",
    cam_width=320,
    cam_height=176,
    device="cpu",
    seed=None,
):
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    t0 = time.time()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(
        task=str(Path(task_path).resolve()),
        object_jitter=0,
        control_hz=CONTROL_HZ,
        cam_width=cam_width,
        cam_height=cam_height,
        device=device,
    )
    stages = stage.split(",")
    results, resets = [], []
    for i, st in enumerate(stages):
        tdir = out if len(stages) == 1 else out / f"trial{i}_{st.replace('@', '_seed')}"
        st_seed = seed
        if "@" in st:  # "<stage>@<seed>": this trial's reset seed
            st, sd = st.split("@")
            st_seed = int(sd)
        res, reset_state = run_trial(emb, tdir, max_steps, record, st, t0, seed=st_seed)
        results.append(res)
        resets.append(reset_state)
    if len(stages) > 1:
        rep = reset_report([x.split("@")[0] for x in stages], resets, results)
        (out / "reset_check.json").write_text(json.dumps(rep, indent=2, default=float))
        print("RESETCHECK", json.dumps(rep["summary"]), flush=True)
    return results


def reset_state_now():
    import regkit

    rt = regkit.runtime()
    poses = rt.read()
    q, _ = rt.dofs()
    return dict(
        score=regkit.STATE["score"],
        setup=rt.setup,
        joints={k: round(v, 5) for k, v in q.items()},
        line_psi=round(rt.plant.p, 3),
        bodies={n: np.round(p * 1000, 4).tolist() for n, (p, r) in poses.items()},
    )


def reset_report(stages, resets, results):
    import layout as L

    rows = []
    for i, r in enumerate(resets):
        setup = tuple(r["setup"]) if isinstance(r["setup"], list) else r["setup"]
        spec = L.setup_spec(setup)
        jdev = max(abs(r["joints"][k]) for k in ("slide", "turn", "valve"))
        pdev = abs(r["joints"]["pointer"] - L.gauge_angle(spec["p0"]))
        rows.append(dict(trial=i, stage=stages[i], setup=r["setup"], score_after_reset=r["score"],
                         max_joint_abs=round(jdev, 5), pointer_dev=round(pdev, 5),
                         line_dev=round(abs(r["line_psi"] - spec["p0"]), 4), trial_score=results[i]["task_score"],
                         trial_stop=results[i]["stop"]))
    ok = all(x["score_after_reset"] == 0.0 and x["max_joint_abs"] < 1e-3 and x["line_dev"] < 1e-6 for x in rows)
    return dict(summary=dict(passed=ok, trials=len(rows)), rows=rows, reset_states=resets)


def run_trial(emb, out, max_steps, record, stage, t0, seed=None):
    import regkit
    from inspect_robots import Scene
    from inspect_robots_yam.packing import STATE_KEY

    out.mkdir(parents=True, exist_ok=True)
    t1 = time.time()
    obs = emb.reset(Scene(id="demo", instruction="demo"), seed=seed)
    print(f"BENCH reset {time.time() - t1:.0f} s", flush=True)
    reset_state = reset_state_now()
    c = Ctx(emb)
    gen = STAGES[stage](c)
    frames = {}

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    snap(obs)
    steps, ended, info, stop, success, res = 0, None, {}, None, False, None
    tracef = open(out / "trace.jsonl", "w")
    while steps < max_steps:
        if ended is None:
            try:
                next(gen)
            except StopIteration:
                ended = steps
            except Stop as e:
                stop, ended = str(e), steps
                print("STOP", stop, flush=True)
        res = emb.step(c.action())
        jp = np.asarray(res.observation.state[STATE_KEY], float).ravel()
        c.left.observe(jp[:7])
        c.right.observe(jp[7:14])
        steps += 1
        c.step = steps
        info = res.info
        snap(res.observation)
        d = regkit.STATE["detail"]
        row = dict(
            step=steps,
            score=info.get("score"),
            level=regkit.STATE["level"],
            detail=d,
            labels=[c.left.label, c.right.label],
            nonfinite=regkit.STATE["nonfinite"],
            cmd=np.round(np.r_[c.left.last_cmd, c.right.last_cmd], 4).tolist(),
            q=np.round(jp, 4).tolist(),
            grip_n={k: round(v, 2) for k, v in c.grip_force().items()},
            tcp_mm={
                n: np.round(a.measured_tcp()[0] * 1000, 2).tolist()
                for n, a in c.arms.items()
            },
            tcp_cmd_mm={
                n: np.round(a.pos * 1000, 2).tolist() for n, a in c.arms.items()
            },
            offs={n: np.round(a.offset, 3).tolist() for n, a in c.arms.items()},
        )
        tracef.write(json.dumps(row, default=float) + "\n")
        if steps % 20 == 0:
            tracef.flush()
        if steps % 25 == 0:
            print(
                f"step {steps} {c.left.label} | {c.right.label} score {info.get('score')} gauge {d.get('gauge_psi')} "
                f"line {d.get('line_psi')} set {d.get('set_psi')} turns {d.get('turns')} out {d.get('knob_out_mm')} "
                f"valve {d.get('valve_deg')} contact {d.get('contact_n')} ok {d.get('ok_s')} "
                f"wall {time.time() - t1:.0f}s",
                flush=True,
            )
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= 2 * CONTROL_HZ:
            break
    score = float(info.get("score", 0.0) or 0.0)
    result = dict(
        stage=stage,
        task_score=score,
        success=success,
        steps=steps,
        stop=stop,
        termination=getattr(res, "termination_reason", None),
        setup=reset_state["setup"],
        scorer=dict(score=regkit.STATE["score"], level=regkit.STATE["level"]),
        final=regkit.STATE["detail"],
        nonfinite=regkit.STATE["nonfinite"],
        looks=c.log,
        wall_s=round(time.time() - t1, 1),
        wall_total_s=round(time.time() - t0, 1),
        worst_ik_mm=dict(
            left=round(c.left.worst_mm, 2), right=round(c.right.worst_mm, 2)
        ),
        grip_peak_n={k: round(v, 2) for k, v in c.grip_peak.items()},
        physics_dt=regkit.PHYSICS_DT,
        control_hz=CONTROL_HZ,
    )
    tracef.close()
    (out / "result.json").write_text(json.dumps(result, indent=2, default=float))
    if record and frames:
        import imageio

        for cam, fs in frames.items():
            _write_mp4(fs, out / f"demo_{cam}.mp4")
        top = next((k for k in frames if "top" in k), next(iter(frames), None))
        if top:
            imageio.imwrite(out / "setup.png", frames[top][0])
            imageio.imwrite(out / "final.png", frames[top][-1])
        for cam, fs in frames.items():
            imageio.imwrite(out / f"final_{cam}.png", fs[-1])
    print(
        "RESULT",
        json.dumps(
            {
                k: result[k]
                for k in ("stage", "task_score", "success", "steps", "stop", "setup")
            }
        ),
        flush=True,
    )
    return result, reset_state


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2000)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--stage", default="demo")
    ap.add_argument(
        "--cam-size",
        default="320x176",
        help="camera WxH (evidence runs use the default)",
    )
    ap.add_argument(
        "--device",
        default="cpu",
        help="PhysX device (cpu, or cuda:0); rendering is on the GPU either way",
    )
    ap.add_argument(
        "--seed",
        default=str(L.OFFICIAL_SEED),
        help="reset seed (seeded start state, as in inspect-robots runs; default: the official eval-seed-0 trial seed, "
        "'none': the named setups)",
    )
    a = ap.parse_args()
    cw, ch = (int(v) for v in a.cam_size.split("x"))
    sd = None if a.seed in ("none", "") else int(a.seed)
    run(a.task, a.out, a.max_steps, not a.no_record, a.stage, cw, ch, a.device, sd)
