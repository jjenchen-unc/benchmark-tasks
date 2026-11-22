"""Scripted full-score demo (and negative variants) for the grill-lid task in the yam_arena embodiment.

    python demo.py task.py OUT [--max-steps N] [--no-record] [--stage demo|<variant>[,<variant>...]] [--seed N]

Acts ONLY through the 14 joint/gripper commands of ``YamArenaEmbodiment.step``. It reads the grill's pose and joint
state through ``grillkit.bench_state()`` (read-only, like an operator watching the grill: where it stands, how far the
lid is open, where the knob points, whether the flame shows); it never writes a pose, a joint state or a score. Its
joint commands are rate-limited below the CLI's per-step delta guard and clipped to the URDF joint limits.

Plan:
1. Left hand: cage the lid's front handle bar (fingers either side of the 25 mm bar with 8 mm play, not squeezing)
   and lift it along the hinge arc, the hand leading the measured lid angle by a few degrees, until the lid is past its
   balance point (63 deg) to 68 deg; open the fingers part way and draw the hand back: the lid swings onto its rear stop (95 deg).
2. Right hand (configuration chosen by a seeded IK search so the whole knob turn and the igniter press need no
   configuration change): pinch the knob's grip fin from the front-right, push it in 4.5 mm (past the 3 mm unlock),
   turn it 92 deg counter-clockwise to START/HI, ease the push, let go.
3. Same hand: press the igniter 7 cm to the left (3.5 mm, past the 2 mm switch point) and hold it until the flame
   shows; gas has been on about 2.5 s by then (limit 5 s).
4. Both arms back to READY (the knob stays at START/HI by its valve friction).
Both arms add the remaining joint error back to their commands (slow integral on their own joint readings), because
the joint PD sags under load (pattern reused from earlier demos).
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
BAR_Y = 0.13  # grill-frame y where the left hand cages the handle bar
CAGE_HALF = 0.021  # finger half-opening around the 25 mm bar (8.5 mm play each side)
LID_LEAD_DEG = 3.0  # the hand's arc target leads the measured lid angle by this much
LID_BAND = 0.15  # rad: joint-integral band while lifting
LID_RATE_DEG = 0.8  # deg per control step of the virtual lid target (16 deg/s)
LID_RELEASE_DEG = 68.0  # past the 63 deg balance point (a lid that falls back is lifted again)
FIN_SQUEEZE = 0.0035  # fingers commanded 3.5 mm inside the fin faces: about 17.5 N per finger
PUSH_DEPTH = 0.0045  # knob first commanded 4.5 mm in (unlock at 3 mm, stop at 5 mm), then closed loop on the travel
PUSH_HOLD_MM = 4.3  # measured push held at 4.3 mm while turning (about 9.8 N)
KNOB_TURN_DEG = 92.0  # to START/HI (90) and a little past
IGN_DEPTH = 0.0030  # igniter first commanded 3 mm in (switch at 2 mm, stop at 4 mm), then closed loop on the travel
IGN_HOLD_MM = 2.8
KNOB_ALPHA = 45.0  # right hand's approach to the knob, from the front-right, this far off the knob axis (deg)
KNOB_PINCH_IN = 0.006  # pad-tip centre this far in front of the knob skirt (on the 16 mm fin)
KNOB_START_S = 1.5  # first 12 deg of the turn
KNOB_TURN_S = 1.5  # the rest
CONTACT_BAND = 0.10  # rad: joint-integral band while pressing / turning


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
        import grillkit

        return grillkit.bench_state()

    def joints(self):
        return self.state()["joints"]

    def grill(self):
        """Measured grill frame (position, rotation) in the station frame."""
        return self.state()["body"]

    def to_st(self, p_g):
        gp, gr = self.grill()
        return gp + gr @ np.asarray(p_g, float)

    def rot_st(self, r_g):
        return self.grill()[1] @ np.asarray(r_g, float)

    def detail(self):
        import grillkit

        return grillkit.STATE["detail"]

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
        row = dict(at=tag, step=self.step, level=d.get("level"), lid=d.get("lid_deg"), knob=d.get("knob_deg"),
                   push=d.get("push_mm"), ign=d.get("igniter_mm"), lit=d.get("lit"), veto=d.get("veto"), **kw)
        self.log.append(row)
        print("LOOK", json.dumps(row, default=float), flush=True)


def frame(app, close):
    """Gripper rotation from its approach axis (z) and finger-closing axis (x), both in the station frame."""
    z = np.asarray(app, float) / np.linalg.norm(app)
    x = np.asarray(close, float)
    x = x - (x @ z) * z
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


# --------------------------------------------------------------------------------------------------- lid
# caging hand's approach along the lid arc (IK-feasible schedule from the URDF reach maps, grill frame):
# (lid deg, pitch deg [positive = pointing down], yaw deg [negative = turned toward the grill's right])
LID_SCHEDULE = ((0.0, 30.0, -60.0), (16.0, 10.0, -60.0), (40.0, 10.0, -60.0), (48.0, 0.0, -60.0),
                (56.0, -10.0, -60.0), (64.0, -20.0, -45.0), (72.0, -35.0, -30.0), (78.0, -40.0, -30.0))


def lid_angles(deg):
    d, p, y = (np.array(v) for v in zip(*LID_SCHEDULE))
    return float(np.interp(deg, d, p)), float(np.interp(deg, d, y))


def lid_hand_best(c, arm, deg):
    """lid_hand at ``deg`` with the approach searched +-12 deg in pitch and +-15 deg in yaw around the schedule: the
    reachable pose nearest the arm's current joints (the cage leaves the orientation free)."""
    pitch0, yaw0 = lid_angles(deg)
    best = None
    for dp in (0.0, -12.0, 12.0, -24.0, 24.0):
        for dy in (0.0, 15.0, -15.0, 30.0):
            p, r = lid_hand(c, deg, pitch=pitch0 + dp, yaw=yaw0 + dy)
            q, pe, re = fastik.solve(p, r, arm.q, base=arm.base, iters=60)
            cost = pe * 1e3 + math.degrees(re) + 0.2 * (abs(dp) + abs(dy)) / 12.0
            if best is None or cost < best[0]:
                best = (cost, p, r)
            if pe < 2e-4 and dp == 0.0 and dy == 0.0:
                return p, r
    return best[1], best[2]


def lid_hand(c, deg, back=0.0, pitch=None, yaw=None):
    """TCP pose (station) that cages the handle bar at lid angle ``deg`` (``back`` m drawn back along -approach)."""
    p0, y0 = lid_angles(deg)
    pitch = p0 if pitch is None else pitch
    yaw = y0 if yaw is None else yaw
    rg = L.rz(yaw) @ L.ry(pitch)
    app = c.rot_st(rg @ np.array([1.0, 0.0, 0.0]))
    close = c.rot_st(rg @ np.array([0.0, 0.0, 1.0]))
    p = c.to_st(L.handle_point(deg, BAR_Y)) - back * app
    return p, frame(app, close)


def open_lid(c, release_deg=LID_RELEASE_DEG, max_s=20.0):
    arm = c.left
    arm.set_comp(True)
    p, r = lid_hand(c, 0.0, back=0.07)
    yield from arm.move(p, r, 3.0, 1.0, "left to handle", tol=0.008)
    p, r = lid_hand(c, 0.0)
    yield from arm.move(p, r, 1.5, None, "left onto bar")
    yield from arm.set_grip(grip_of(CAGE_HALF), 0.6, "left cage bar")
    c.note("caged")
    # lifting: joint integral kept within +-LID_BAND rad of the free-space sag offsets, so the arm lifts the lid's
    # weight but cannot build up a large force on the handle
    arm.comp_band = (arm.offset - LID_BAND, arm.offset + LID_BAND)
    virt = 0.0
    hist = []
    for k in range(int(max_s * CONTROL_HZ)):
        th = c.joints()["lid_deg"]
        hist.append(th)
        if arm.comp_band is not None and len(hist) > 20 and hist[-1] - hist[-21] < 1.0:
            arm.comp_band = None  # stalled: full integral authority
            c.note("lid lift stalled, band off", lid_now=round(th, 2))
        virt = min(virt + LID_RATE_DEG, th + LID_LEAD_DEG, release_deg + 2.0)
        arm.label = "lift lid"
        p, r = lid_hand_best(c, arm, virt)
        try:
            arm.set_pose(p, r, tol=0.010)  # caged bar: 8.5 mm play each side
        except Stop:
            virt = max(th, virt - LID_RATE_DEG)  # at the edge of the arm's reach: hold the last pose
        yield
        if th >= release_deg:
            break
    else:
        raise Stop(f"lid only reached {c.joints()['lid_deg']:.1f} deg")
    c.note("lid past balance")
    # let go: open the fingers, then draw the hand back out of the bar's path
    # open only part way: a finger opened fully would press on the lid's front face and push it back shut
    yield from arm.set_grip(grip_of(0.040), 0.3, "left open")
    yield from wait(0.3)
    app = arm.rot[:, 2]  # straight back along the approach: the bar leaves between the open fingertips
    yield from arm.move(arm.pos - 0.07 * app, arm.rot, 1.0, None, "left back off bar", tol=0.012)
    arm.comp_band = None
    yield from wait(1.0)
    c.note("lid open")
    if c.joints()["lid_deg"] < L.LID_OPEN_STOP_DEG - 5.0:
        raise LidFell(f"lid did not stay open ({c.joints()['lid_deg']:.1f} deg)")


class LidFell(Exception):
    pass


def open_lid_retry(c, tries=2):
    """open_lid; if the lid fell back shut after the hand let go, wait for it to settle and lift it again."""
    for k in range(tries):
        try:
            yield from open_lid(c)
            return
        except LidFell as e:
            c.note("lid fell back", attempt=k, why=str(e))
            if k == tries - 1:
                raise Stop(str(e))
            yield from both(c.left.home(2.0), wait(0.1))
            yield from wait(1.0)
            if c.joints()["lid_deg"] >= L.LID_OPEN_STOP_DEG - 5.0:  # the hand had been holding it: open now
                c.note("lid open after the hand moved away")
                return


# --------------------------------------------------------------------------------------------------- knob, igniter
def knob_pose(c, turn_deg, depth=0.0, back=0.0, sign=None):
    """Right hand pinching the knob's grip fin (the fingers close across it), approaching from the front-right at
    KNOB_ALPHA to the knob axis, the knob turned ``turn_deg`` counter-clockwise
    (seen from the front) and pushed in ``depth``; ``back`` m drawn back along the approach. Station frame."""
    sign = c.knob_sign if sign is None else sign
    rt = L.rx(-(turn_deg + L.FIN_OFF_DEG))  # counter-clockwise seen from the front = about the grill's -x
    a0 = L.rz(KNOB_ALPHA) @ np.array([1.0, 0.0, 0.0])
    c0 = np.array([0.0, 0.0, sign])
    skirt = L.KNOB_C[0] - L.KNOB_GAP - L.KNOB_T
    centre = np.array([skirt - KNOB_PINCH_IN + depth, L.KNOB_C[1], L.KNOB_C[2]])  # pad-tip centre, on the knob axis
    app = rt @ a0
    p = centre - (0.013 + back) * app
    return c.to_st(p), frame(c.rot_st(app), c.rot_st(rt @ c0))


def ign_pose(c, depth=-0.005, back=0.0, yaw=None, pitch=None):
    """Right hand, closed pad tips ``depth`` past the igniter's rest face (negative: in front of it), approach turned
    ``yaw`` about the vertical and pitched ``pitch`` down (default: the planned ones) from the button axis."""
    yaw = getattr(c, "ign_yaw", 0.0) if yaw is None else yaw
    pitch = getattr(c, "ign_pitch", 0.0) if pitch is None else pitch
    app = L.rz(yaw) @ L.ry(pitch) @ np.array([1.0, 0.0, 0.0])
    face = L.IGN_C + np.array([-L.IGN_PROUD + depth, 0.0, 0.0])
    p = face - (0.013 + back) * app
    return c.to_st(p), frame(c.rot_st(app), c.rot_st(L.rz(yaw) @ L.ry(pitch) @ np.array([0.0, 0.0, 1.0])))


IGN_APPROACHES = ((0.0, 0.0), (-10.0, 0.0), (0.0, 20.0), (-10.0, 20.0), (0.0, 35.0), (-20.0, 20.0), (10.0, 20.0),
                  (-10.0, 35.0))


KNOB_PRE = ((0.03, 0.0), (0.02, 0.0), (0.02, 0.02), (0.02, -0.02), (0.012, 0.0), (0.03, 0.02), (0.03, -0.02))


def knob_pre(c, sign=None, back=0.03, side=0.0):
    """Pre-grasp: ``back`` m back along the approach and ``side`` m along the fin's length from the fin pinch."""
    p, r = knob_pose(c, 0.0, back=back, sign=sign)
    return p + side * r[:, 1], r


def plan_right(c):
    """Seeded IK search for a right-arm configuration that reaches the whole knob turn and then the igniter press
    without a configuration change (the knob is low and close in front of the right arm's base). Returns the joint
    angles at the pre-grasp pose."""
    arm = c.right
    rng = np.random.default_rng(7)
    seeds = [arm.q.copy(), np.array([-0.9, 0.73, 0.31, -0.54, -1.31, 0.24]), np.array([-0.7, 1.0, 0.5, -0.8, -1.2, 0.3]),
             np.array([-1.1, 0.6, 0.2, -0.4, -1.4, 0.2])]
    seeds += list(rng.uniform(arm.lo, arm.hi, size=(80, 6)))
    best = None
    for sign in (1.0, -1.0):
        poses = [knob_pose(c, 0.0, sign=sign)]
        poses += [knob_pose(c, t, depth=PUSH_DEPTH, sign=sign) for t in (0.0, 30.0, 60.0, KNOB_TURN_DEG)]
        poses += [knob_pose(c, KNOB_TURN_DEG, back=0.025, sign=sign)]
        for s in seeds:
            q = s.copy()
            worst, qs = 0.0, []
            for p, r in poses:
                q, pe, re = fastik.solve(p, r, q, base=arm.base, iters=120)
                worst = max(worst, pe * 1e3 + math.degrees(re))
                qs.append(q.copy())
            cost = worst + 0.3 * float(np.abs(qs[0] - arm.q).sum())  # prefer configurations near the current one
            if best is None or cost < best[0]:
                best = (cost, worst, sign, qs)
            if worst < 0.5 and best[1] < 0.5 and len(seeds) > 0:
                break
        if best[1] < 1.0:
            break
    _, kworst, sign, qs = best
    c.knob_sign = sign
    pbest = None
    for back, side in KNOB_PRE:  # pre-grasp reached from the grasp configuration (no configuration change)
        q, pe, re = fastik.solve(*knob_pre(c, sign, back, side), qs[0], base=arm.base, iters=150)
        err = pe * 1e3 + math.degrees(re)
        if pbest is None or err < pbest[0]:
            pbest = (err, q, back, side)
        if err < 1.0:
            break
    c.knob_pre = (pbest[2], pbest[3])
    ibest = None
    for yaw, pitch in IGN_APPROACHES:
        q = qs[-1].copy()
        worst = 0.0
        for p, r in (ign_pose(c, -0.006, yaw=yaw, pitch=pitch), ign_pose(c, IGN_DEPTH, yaw=yaw, pitch=pitch)):
            q, pe, re = fastik.solve(p, r, q, base=arm.base, iters=150)
            worst = max(worst, pe * 1e3 + math.degrees(re))
        if ibest is None or worst < ibest[0]:
            ibest = (worst, yaw, pitch)
        if worst < 1.0:
            break
    c.ign_yaw, c.ign_pitch = ibest[1], ibest[2]
    c.note("right plan", sign=sign, knob_worst=round(kworst, 2), pre=[pbest[2], pbest[3], round(pbest[0], 2)], ign_approach=[ibest[1], ibest[2]], ign_worst=round(ibest[0], 2),
           q_pre=np.round(pbest[1], 3).tolist())
    if kworst > 4.0 or ibest[0] > 4.0 or pbest[0] > 4.0:
        raise Stop(f"knob / igniter not reachable without a configuration change ({kworst:.1f}, {ibest[0]:.1f}, "
                   f"{pbest[0]:.1f})")
    return pbest[1]


def press_depth(cmd, measured_mm, target_mm, gain=0.25, hi=0.020):
    """Commanded press depth (m) nudged so the measured travel settles at ``target_mm``: the arm's joint PD is soft,
    so the hand stops short of a commanded depth by however much the spring pushes it back."""
    return float(np.clip(cmd + gain * (target_mm - measured_mm) / 1000.0, 0.0, hi))


def knob_to_hi(c, push=True, turn_to=KNOB_TURN_DEG):
    """Right hand: pinch the fin, push in, turn to START/HI. Generator; yields each control step."""
    from yam_arena import ik

    arm = c.right
    arm.set_comp(True)
    q_pre = plan_right(c)
    half = L.FIN[2] / 2
    # via a point 12 cm above the pre-grasp (same arm configuration), so the arm swings in above the grill's front
    p_pre, r_pre = knob_pose(c, 0.0, back=c.knob_pre[0])
    p_pre = p_pre + c.knob_pre[1] * r_pre[:, 1]
    for up in (0.12, 0.09, 0.06, 0.03):
        q_high, pe, re = fastik.solve(p_pre + np.array([0.0, 0.0, up]), r_pre, q_pre, base=arm.base, iters=150)
        if pe < 0.002 and re < math.radians(2):
            break
    c.note("knob via", up=up, err_mm=round(pe * 1e3, 2))
    c.right_via = q_high
    yield from arm.move_q(q_high, 2.5, grip_of(half + 0.012), "right over knob")
    yield from arm.move_q(q_pre, 1.0, None, "right down to knob")
    yield from arm.move(*knob_pose(c, 0.0), 1.0, None, "right onto fin")
    yield from arm.set_grip(grip_of(half - FIN_SQUEEZE), 0.6, "right pinch fin")
    yield from wait(0.2)
    c.note("fin pinched", grip_n=c.grip_force())
    # in contact: joint integral on (the PD alone sags), kept within a band so it cannot build up a large force
    arm.comp_band = (arm.offset - CONTACT_BAND, arm.offset + CONTACT_BAND)
    d = PUSH_DEPTH if push else -0.002  # without the push: held 2 mm out, the hand only twists
    target = PUSH_HOLD_MM if push else 0.0
    yield from arm.move(*knob_pose(c, 0.0, depth=d), 0.6, None, "push knob in")
    for _ in range(int(2.5 * CONTROL_HZ)):  # until the knob is in past its unlock depth
        if push:
            d = press_depth(d, c.joints()["push_mm"], target)
        arm.label = "push knob in"
        arm.set_pose(*knob_pose(c, 0.0, depth=d))
        yield
        if not push or c.joints()["push_mm"] >= PUSH_HOLD_MM - 0.4:
            break
    c.note("knob pushed", cmd_mm=round(d * 1e3, 2))
    # turn: the first 12 deg slowly (the OFF stop only lets go while the knob is held in), then on to START/HI
    prof = [12.0 * ik.smoothstep(k / (KNOB_START_S * CONTROL_HZ)) for k in range(1, int(KNOB_START_S * CONTROL_HZ) + 1)]
    n = int(KNOB_TURN_S * CONTROL_HZ)
    prof += [12.0 + (turn_to - 12.0) * ik.smoothstep(k / n) for k in range(1, n + 1)]
    for t in prof:
        if push:
            d = press_depth(d, c.joints()["push_mm"], target)
        arm.label = "turn knob"
        arm.set_pose(*knob_pose(c, t, depth=d))
        yield
    c.note("knob turned", cmd_mm=round(d * 1e3, 2))
    c.knob_cmd_t = prof[-1]
    c.push_cmd = d


def knob_let_go(c):
    """Ease the push, open, draw back (the knob stays where it is: valve friction)."""
    arm = c.right
    t = getattr(c, "knob_cmd_t", KNOB_TURN_DEG)
    yield from arm.move(*knob_pose(c, t, depth=0.0), 0.25, None, "ease push", tol=0.008)
    yield from arm.set_grip(grip_of(L.FIN[2] / 2 + 0.012), 0.25, "right open")
    arm.comp_band = None
    yield from arm.move(*knob_pose(c, t, back=0.02), 0.25, None, "right off knob", tol=0.012)


def igniter_until_lit(c, hold_max_s=3.0, pause_s=0.0):
    arm = c.right
    yield from arm.move(*ign_pose(c, depth=-0.006), 0.6, 0.0, "to igniter", tol=0.008)
    if pause_s > 0:
        c.note("pausing at the igniter", s=pause_s)
        yield from wait(pause_s)
    d = IGN_DEPTH
    arm.comp_band = (arm.offset - CONTACT_BAND, arm.offset + CONTACT_BAND)
    yield from arm.move(*ign_pose(c, depth=d), 0.3, None, "press igniter", tol=0.008)
    for _ in range(int(hold_max_s * CONTROL_HZ)):
        d = press_depth(d, c.joints()["igniter_mm"], IGN_HOLD_MM, hi=0.015)
        arm.label = "hold igniter"
        arm.set_pose(*ign_pose(c, depth=d), tol=0.008)
        yield
        if c.detail().get("lit") and c.detail().get("lit_s", 0) >= 0.2:
            break
    c.note("igniter held", cmd_mm=round(d * 1e3, 2), grip_n=c.grip_force())
    yield from arm.move(*ign_pose(c, depth=-0.010), 0.3, None, "off igniter", tol=0.012)
    arm.comp_band = None
    # straight out from the panel (clear of the knob), then back up through the approach's via point; home from there
    back = c.rot_st(np.array([-1.0, 0.0, 0.0]))
    yield from arm.move(arm.pos + 0.07 * back, arm.rot, 0.8, None, "right back", tol=0.03)
    yield from arm.move_q(getattr(c, "right_via", arm.q), 1.2, None, "right up")


def demo(c):
    yield from wait(0.5)
    yield from open_lid_retry(c)
    yield from both(c.left.home(2.5), knob_to_hi(c))
    yield from knob_let_go(c)
    yield from igniter_until_lit(c)
    c.note("lit?")
    yield from both(c.left.home(1.0), c.right.home(2.5))
    yield from wait(3.0)


def idle(c):
    yield from wait(8.0)


def lid_only(c):
    """Development: open the lid and stop."""
    yield from wait(0.5)
    yield from open_lid(c)
    yield from both(c.left.home(2.5), wait(0.1))
    yield from wait(2.0)


def knob_only(c):
    """Development: knob and igniter with the lid closed (scores 0, for the mechanism)."""
    yield from wait(0.5)
    yield from knob_to_hi(c)
    yield from knob_let_go(c)
    yield from igniter_until_lit(c)
    yield from both(c.left.home(1.0), c.right.home(2.5))
    yield from wait(2.0)


def neg_lid_closed(c):
    """Lights the grill without opening the lid (gas on under the closed lid, igniter pressed): 0."""
    yield from wait(0.5)
    yield from knob_to_hi(c)
    yield from knob_let_go(c)
    yield from igniter_until_lit(c)
    c.note("lit with the lid closed")
    yield from both(c.left.home(1.0), c.right.home(2.5))
    yield from wait(3.0)


def neg_slow_spark(c):
    """Opens the lid and turns the gas on, but waits 4 s at the igniter before pressing it: gas flows unlit for more
    than 5 s: 0."""
    yield from wait(0.5)
    yield from open_lid_retry(c)
    yield from both(c.left.home(2.5), knob_to_hi(c))
    yield from knob_let_go(c)
    c.note("gas on")
    yield from igniter_until_lit(c, pause_s=4.0)
    c.note("late spark")
    yield from both(c.left.home(1.0), c.right.home(2.5))
    yield from wait(3.0)


def neg_no_push(c):
    """Opens the lid, pinches the knob and tries to turn it 30 deg WITHOUT pushing it in: the OFF stop holds, the knob
    does not turn; the hand lets go, draws straight back and goes home. No gas: 0."""
    yield from wait(0.5)
    yield from open_lid_retry(c)
    yield from both(c.left.home(2.5), knob_to_hi(c, push=False, turn_to=30.0))
    c.note("turned without pushing")
    yield from knob_let_go(c)
    arm = c.right
    back = c.rot_st(np.array([-1.0, 0.0, 0.0]))
    yield from arm.move(arm.pos + 0.07 * back, arm.rot, 0.8, None, "right back", tol=0.03)
    yield from arm.move_q(getattr(c, "right_via", arm.q), 1.2, None, "right up")
    yield from both(c.left.home(1.0), c.right.home(2.5))
    yield from wait(3.0)


def replay(c):
    """Open-loop check: plays the 14-D commands recorded in a finished run's trace.jsonl (path in GRILL_REPLAY), step
    by step, without reading anything: what a fixed trajectory scores on another start state."""
    import os

    rows = [json.loads(line) for line in open(os.environ["GRILL_REPLAY"])]
    c.replay = [r["cmd"] for r in rows]
    for _ in range(len(c.replay) + 40):
        yield


STAGES = {
    "demo": demo,
    "idle": idle,
    "lid-only": lid_only,
    "knob-only": knob_only,
    "replay": replay,
    "neg-lid-closed": neg_lid_closed,
    "neg-slow-spark": neg_slow_spark,
    "neg-no-push": neg_no_push,
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
    import grillkit

    rt = grillkit.runtime()
    poses = rt.read()
    return dict(
        score=grillkit.STATE["score"],
        setup=rt.setup,
        joints={k: round(v, 4) for k, v in rt.joint_state().items()},
        bodies={n: np.round(p * 1000, 4).tolist() for n, (p, r) in poses.items()},
        rots={n: np.round(r, 6).tolist() for n, (p, r) in poses.items()},
    )


def reset_report(stages, resets, results):
    import layout as L

    rows = []
    for i, r in enumerate(resets):
        setup = tuple(r["setup"]) if isinstance(r["setup"], list) else r["setup"]
        gp, gr = L.grill_pose(L.setup_spec(setup))
        dev = float(np.linalg.norm(np.subtract(r["bodies"]["body"], gp * 1000)))
        rdev = float(np.abs(np.subtract(r["rots"]["body"], gr)).max())
        jdev = max(abs(v) for v in r["joints"].values())
        rows.append(dict(trial=i, stage=stages[i], setup=r["setup"], score_after_reset=r["score"],
                         body_dev_from_authored_mm=round(dev, 4), max_rot_dev=round(rdev, 6),
                         max_joint_abs=round(jdev, 4), trial_score=results[i]["task_score"],
                         trial_stop=results[i]["stop"]))
    ok = all(x["score_after_reset"] == 0.0 and x["body_dev_from_authored_mm"] < 2.0 and x["max_joint_abs"] < 0.5
             for x in rows)
    return dict(summary=dict(passed=ok, trials=len(rows), max_body_dev_mm=max(x["body_dev_from_authored_mm"] for x in rows)),
                rows=rows, reset_states=resets)


def run_trial(emb, out, max_steps, record, stage, t0, seed=None):
    import grillkit
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
        d = grillkit.STATE["detail"]
        row = dict(
            step=steps,
            score=info.get("score"),
            level=grillkit.STATE["level"],
            detail=d,
            labels=[c.left.label, c.right.label],
            nonfinite=grillkit.STATE["nonfinite"],
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
                f"step {steps} {c.left.label} | {c.right.label} score {info.get('score')} lid {d.get('lid_deg')} "
                f"knob {d.get('knob_deg')} push {d.get('push_mm')} ign {d.get('igniter_mm')} lit {d.get('lit')} "
                f"unlit {d.get('unlit_s')} contact {d.get('contact_n')} veto {d.get('veto')} "
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
        scorer=dict(score=grillkit.STATE["score"], level=grillkit.STATE["level"]),
        final=grillkit.STATE["detail"],
        nonfinite=grillkit.STATE["nonfinite"],
        looks=c.log,
        wall_s=round(time.time() - t1, 1),
        wall_total_s=round(time.time() - t0, 1),
        worst_ik_mm=dict(
            left=round(c.left.worst_mm, 2), right=round(c.right.worst_mm, 2)
        ),
        grip_peak_n={k: round(v, 2) for k, v in c.grip_peak.items()},
        physics_dt=grillkit.PHYSICS_DT,
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
        type=int,
        default=None,
        help="reset seed (seeded start state, as in inspect-robots runs)",
    )
    a = ap.parse_args()
    cw, ch = (int(v) for v in a.cam_size.split("x"))
    run(a.task, a.out, a.max_steps, not a.no_record, a.stage, cw, ch, a.device, a.seed)
