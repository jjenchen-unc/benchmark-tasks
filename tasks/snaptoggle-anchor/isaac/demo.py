"""Scripted full-score demo (and negative variants) for the snaptoggle-anchor task in the yam_arena embodiment.

    python demo.py task.py OUT [--max-steps N] [--no-record] [--stage demo|<variant>[,<variant>...]] [--seed S]

Acts ONLY through the 14 joint/gripper commands of ``YamArenaEmbodiment.step``. It reads the anchor's poses, the
channel angle, the cap position and the channel-on-board contact force through ``snapkit.bench_state()`` (read-only,
like an installer watching the hole and feeling the straps go taut); it never writes a pose, a joint state or a score.
Its joint commands are rate-limited below the CLI's per-step delta guard and clipped to the URDF joint bounds.

Plan (left hand = strap hand, right hand = channel / cap hand; both grippers pitched forward-down):
1. Right pinches the folded channel by its rear half, left pinches the straps' pull tab; lift the anchor off its block.
2. Carry it in line with the hole (the hole's place is read from the wall's pose), align the channel's end with the
   hole (closed loop on the measured channel pose) and push the leading half of the channel through.
3. Right lets go and moves clear; left pushes the straps on until the channel is through and swings across behind
   the board (measured channel angle).
4. Left pulls the straps until the channel bears on the back of the board (channel-on-board contact force), and keeps
   them taut (force regulated) while right slides the cap along the straps with its open fingertips into the hole
   until its flange is on the board.
5. Left lets go; right bends the strap tail sideways at the cap face until the straps snap off, then both go home.
Both arms add the remaining joint error back to their commands (slow integral on their own joint readings) in free
moves; in contact the offsets are frozen and the strap tension is regulated from the contact force instead.
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
PITCH = 50.0  # deg: approach axis pitched forward-down from horizontal
TIP = 0.013  # pad tips past the TCP
CH_SQUEEZE = 0.0025  # channel pinch: fingers commanded 2.5 mm inside its faces (~12.5 N per finger)
TAB_SQUEEZE = 0.0016  # pull tab (3.2 mm): fingers commanded fully closed (~8 N per finger)
TENSION_N = 3.0  # strap tension held while the cap slides (channel-on-board force)
ALIGN_TOL_M = 0.0004


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


def pitched(theta_deg=PITCH, yaw_deg=0.0, flip=1.0):
    """TCP rotation: approach axis forward-down at theta from horizontal (heading yaw), fingers closing along the
    horizontal perpendicular."""
    t, p = math.radians(theta_deg), math.radians(yaw_deg)
    z = np.array([math.cos(t) * math.cos(p), math.cos(t) * math.sin(p), -math.sin(t)])
    x = flip * np.array([-math.sin(p), math.cos(p), 0.0])
    return np.column_stack([x, np.cross(z, x), z])


def tcp_for_tip(tip_point, rot):
    """TCP position that puts the pad tips' centre at tip_point."""
    return np.asarray(tip_point, float) - TIP * rot[:, 2]


PUSH_PITCH = 60.0  # cap push: the pad tips' forward corners lead (the rest of the finger edge trails)
PUSH_Z, PUSH_W = 0.013, 0.0077  # forward corner of the pad tips (TCP frame: along approach, along pad width)


def edge_local(rot):
    """TCP-frame offset of the fingers' forward edge point (the pad-width end with +x in the world)."""
    s = 1.0 if rot[0, 1] >= 0 else -1.0
    return np.array([0.0, s * PUSH_W, PUSH_Z])


def tcp_for_edge(point, rot):
    """TCP position that puts the fingers' forward edge point at ``point``."""
    return np.asarray(point, float) - rot @ edge_local(rot)


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

    def set_pose(self, pos, rot):
        q, pe, re = self.solve(pos, rot)
        if pe > 0.004 or re > math.radians(4):
            raise Stop(
                f"{self.name} unreachable at {self.label}: {pe * 1000:.1f} mm {math.degrees(re):.1f} deg"
            )
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), np.asarray(rot, float)

    def move(self, pos, rot, duration, grip=None, label=""):
        self.label = label or self.label
        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        from yam_arena import ik

        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self.set_pose(
                p0 + s * (np.asarray(pos) - p0), ik.slerp_matrix(r0, np.asarray(rot), s)
            )
            yield

    def set_grip(self, grip, duration=0.8, label=""):
        yield from self.move(self.pos, self.rot, duration, grip, label)

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
    """Run two generators in parallel (one control step each per yield)."""
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
        self.G = {}
        self.corr_p, self.corr_r = np.zeros(3), np.zeros(3)
        self.pull_x = None

    def state(self):
        import snapkit

        return snapkit.bench_state()

    def pose(self, name="channel"):
        return self.state()[name]

    def detail(self):
        import snapkit

        return snapkit.STATE["detail"]

    def hole(self):
        wp = self.pose("wall")[0]
        return np.array([float(wp[0]), float(wp[1]), L.HOLE_Z0])

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

    def note(self, tag, **kw):
        d = self.detail()
        st = self.state() or {}
        row = dict(at=tag, step=self.step, level=d.get("level"), phi_deg=d.get("phi_deg"), q_mm=d.get("q_mm"),
                   contacts=st.get("contacts"), measure=d.get("measure"), **kw)
        self.log.append(row)
        print("LOOK", json.dumps(row, default=float), flush=True)


def best_rot(arm, pos, theta=PITCH, yaw=0.0):
    """Pitched TCP rotation with the finger order (flip) that needs the least wrist-roll travel."""
    best = None
    for flip in (1.0, -1.0):
        r = pitched(theta, yaw, flip)
        q, pe, re = arm.solve(pos, r)
        cost = abs(q[5]) / 2.094 + pe * 10 + re
        if best is None or cost < best[0]:
            best = (cost, r)
    return best[1]


def capture(c, body, names):
    """Freeze the body -> TCP transform of each named hand (measured, with the anchor held still)."""
    pb, rb = c.pose(body)
    for n in names:
        pt, rt = c.arms[n].measured_tcp()
        c.G[n] = (rb.T @ (pt - pb), rb.T @ rt)
    c.corr_p, c.corr_r = np.zeros(3), np.zeros(3)


def track(c, body, names, target_fn, duration, label, gain=0.05, rot_gain=None, until=None, max_s=None,
          deadband=0.0, axes=(1.0, 1.0, 1.0)):
    """Closed loop on a held body: each step command the named hands to the TCP poses that put the body at
    target_fn(s) (through the frozen grasp transforms) plus slow integral corrections of the measured position /
    orientation error (arm sag, grasp slip). ``axes`` weights the position error per world axis."""
    from yam_arena import ik

    n = max(1, round(duration * CONTROL_HZ))
    nmax = n if max_s is None else max(n, round(max_s * CONTROL_HZ))
    for k in range(1, nmax + 1):
        pt, rt = target_fn(ik.smoothstep(min(1.0, k / n)))
        pm, rm = c.pose(body)
        err = (pt - pm) * np.asarray(axes)
        if deadband > 0:
            err = np.where(np.abs(err) < deadband, 0.0, err)
        c.corr_p = np.clip(c.corr_p + gain * err, -0.012, 0.012)
        e_r = _rotvec(rt @ rm.T)
        c.corr_r = np.clip(c.corr_r + (gain if rot_gain is None else rot_gain) * e_r, -0.15, 0.15)
        rc = _rotm(c.corr_r)
        for name in names:
            gp, gr = c.G[name]
            arm = c.arms[name]
            arm.label = label
            arm.set_pose(pt + c.corr_p + rc @ rt @ gp, rc @ rt @ gr)
        yield
        if until is not None and k >= n and until(c):
            break


# --------------------------------------------------------------------------------------------------- the script
def tab_center(c):
    tp, tr = c.pose("tail")
    sl = L.STRAP_LEN - L.X_BRK - L.TAB[0]
    return tp + tr @ np.array([sl + L.TAB[0] / 2, 0.0, 0.0])


def pick(c):
    pc, rc = c.pose("channel")
    for arm in c.arms.values():
        arm.set_comp(True)
    pad_top = pc[2] - L.CH_H / 2
    # right: pads' tips 1 mm above the block, centred on the channel's rear half (+Ch.x points to the robot)
    rp = pc + rc @ np.array([L.CH_L / 4, 0.0, 0.0])
    r_tip = np.array([rp[0], rp[1], pad_top + 0.001])
    rr = best_rot(c.right, tcp_for_tip(r_tip, pitched()))
    tc = tab_center(c)
    l_tip = np.array([tc[0], tc[1], tc[2] - L.TAB[2] / 2 + 0.002])
    lr = best_rot(c.left, tcp_for_tip(l_tip, pitched()))
    tg = {"right": (tcp_for_tip(r_tip, rr), rr, L.CH_W / 2 + 0.008), "left": (tcp_for_tip(l_tip, lr), lr, 0.009)}
    yield from both(*(c.arms[n].move(tg[n][0] + [0, 0, 0.05], tg[n][1], 3.0, grip_of(tg[n][2]), f"{n} over grip")
                      for n in c.arms))
    yield from wait(0.4)
    yield from both(*(c.arms[n].move(tg[n][0], tg[n][1], 2.0, None, f"{n} descend") for n in c.arms))
    yield from wait(0.4)
    yield from both(c.right.set_grip(grip_of(L.CH_W / 2 - CH_SQUEEZE), 1.0, "right close"),
                    c.left.set_grip(grip_of(L.TAB[1] / 2 - TAB_SQUEEZE), 1.0, "left close"))
    yield from wait(0.6)
    c.note("gripped")
    yield from both(*(c.arms[n].move(c.arms[n].pos + [0, 0, 0.030], c.arms[n].rot, 1.5, None, f"{n} lift")
                      for n in c.arms))
    yield from wait(0.6)
    capture(c, "channel", ("left", "right"))
    c.note("lifted")


def carry(c, dy=0.0):
    """In line with the hole, channel end 11 mm short of the board face (``dy`` offsets the target for negatives)."""
    hole = c.hole() + np.array([0.0, dy, 0.0])
    p0, r0 = c.pose("channel")
    p1 = np.array([hole[0] - L.CH_L / 2 - 0.011, hole[1], hole[2]])
    yield from track(c, "channel", ("left", "right"), lambda s: (p0 + s * (p1 - p0), L.S_SET), 4.0, "carry")
    c.note("in line")


def lead_end(c):
    """Leading end centre of the folded channel (the -Ch.x end points into the wall)."""
    pc, rc = c.pose("channel")
    return pc + rc @ np.array([-L.CH_L / 2, 0.0, 0.0]), rc


def lateral_err(c, dy=0.0):
    """(y, z) error (m) of the channel's leading end from the hole axis, and the channel axis angle to the hole axis."""
    tip, rc = lead_end(c)
    hole = c.hole()
    ax = math.degrees(math.acos(min(1.0, abs(float(rc[0, 0])))))
    return np.array([tip[1] - hole[1] - dy, tip[2] - hole[2]]), ax


INSERT_TOL_M = 0.00050  # leading end within 0.5 mm of the hole axis before (and while) it advances
INSERT_STEP_M = 0.0004  # 8 mm/s


def insert(c, dy=0.0):
    """Align the folded channel's leading end with the hole (closed loop on its measured pose), then push it in,
    advancing only while the end stays within INSERT_TOL_M of the hole axis (the channel's corners clear the 1/2 in
    hole by 0.8 mm); once the end is 3 mm inside, the hole guides it and only the push continues."""
    hole = c.hole() + np.array([0.0, dy, 0.0])
    xa = hole[0] - L.CH_L / 2 - 0.004
    yield from track(c, "channel", ("left", "right"), lambda s: (np.array([xa, hole[1], hole[2]]), L.S_SET), 1.5,
                     "align with hole", gain=0.08,
                     until=lambda c: np.abs(lateral_err(c, dy)[0]).max() < 0.8 * INSERT_TOL_M and lateral_err(c, dy)[1] < 1.0,
                     max_s=5.0)
    e, ax = lateral_err(c, dy)
    c.note("aligned", lateral_mm=(e * 1e3).round(3).tolist(), axis_deg=ax)
    x1 = hole[0] + 0.004  # channel centre 4 mm past the face: the right fingers stop short of the board
    x = xa
    from yam_arena import ik  # noqa: F401

    for k in range(600):
        tip, rm = lead_end(c)
        e, ax = lateral_err(c, dy)
        inside = tip[0] > hole[0] + 0.003
        if inside or np.abs(e).max() < INSERT_TOL_M:
            x = min(x1, x + INSERT_STEP_M)
        if not inside:
            err = np.array([0.0, -e[0], -e[1]])
            c.corr_p = np.clip(c.corr_p + 0.10 * err, -0.012, 0.012)
        pc, _ = c.pose("channel")
        ex = x - pc[0]
        c.corr_p[0] = float(np.clip(c.corr_p[0] + 0.03 * ex, -0.006, 0.006))
        e_r = _rotvec(L.S_SET @ rm.T)
        c.corr_r = np.clip(c.corr_r + 0.05 * e_r, -0.15, 0.15)
        rc = _rotm(c.corr_r)
        pt = np.array([x, hole[1], hole[2]])
        for name in ("left", "right"):
            gp, gr = c.G[name]
            arm = c.arms[name]
            arm.label = "push channel into hole"
            arm.set_pose(pt + c.corr_p + rc @ L.S_SET @ gp, rc @ L.S_SET @ gr)
        yield
        f = float((c.state()["contacts"] or {}).get("channel_n", 0.0))
        if x >= x1 and abs(ex) < 0.0015:
            break
        if not inside and f > 3.0:  # end on the board face: back off 1 mm and re-align
            x -= 0.001
    tip, _ = lead_end(c)
    c.note("channel half in", lateral_mm=(lateral_err(c, dy)[0] * 1e3).round(3).tolist(),
           end_in_mm=round((tip[0] - hole[0]) * 1e3, 2))
    if tip[0] < hole[0] + L.BOARD_T + 0.005:
        raise Stop(f"channel not through the hole: leading end {(tip[0] - hole[0]) * 1e3:.1f} mm past the face")


def handoff(c):
    """Right lets go of the channel and moves clear; left pushes the straps on until the channel is through and has
    swung across behind the board."""
    arm = c.right
    arm.set_comp("hold")
    yield from arm.set_grip(grip_of(L.CH_W / 2 + 0.008), 0.8, "right open")
    yield from arm.move(arm.pos + np.array([-0.030, 0.0, 0.050]), arm.rot, 1.5, None, "right clear")
    arm.set_comp(True)
    capture(c, "stub", ("left",))
    hole = c.hole()
    p0, _ = c.pose("stub")
    p1 = np.array([hole[0] + L.BOARD_T + L.CH_L / 2 + 0.014, hole[1], hole[2]])
    yield from track(c, "stub", ("left",), lambda s: (p0 + s * (p1 - p0), L.S_SET), 3.5, "push straps in",
                     gain=0.04, deadband=0.0003)
    yield from track(c, "stub", ("left",), lambda s: (p1, L.S_SET), 0.5, "wait for the swing", gain=0.0,
                     until=lambda c: abs(c.state()["phi"]) < 12.0, max_s=4.0)
    c.note("swung", phi=c.state()["phi"])


def tension_gen(c, target_n, duration, label="hold straps taut", until=None, steer="pivot"):
    """Left hand: regulate the strap tension (the channel's contact force on the board) by moving its commanded
    position along the hole axis; joint offsets frozen (no integral wind-up against the wall)."""
    arm = c.left
    arm.set_comp("hold")
    if c.pull_x is None:
        c.pull_x = arm.pos.copy()
    n = max(1, round(duration * CONTROL_HZ))
    for k in range(n):
        f = float((c.state()["contacts"] or {}).get("channel_n", 0.0))
        dx = float(np.clip(0.0005 * (target_n - f), -0.0006, 0.0006))
        c.pull_x = c.pull_x + np.array([-dx, 0.0, 0.0])
        # steer the tab sideways. While pulling (channel free in the cavity, the pivot follows the tab): bring the
        # channel's pivot onto the hole axis, so the channel lands centred on the hole. Once it bears: keep the tab in
        # line with the pivot (straps parallel to the hole axis), so the cap's collar is not cocked in the hole.
        piv = c.pose("stub")[0]
        tc = tab_center(c)
        err = (c.hole()[1:] - piv[1:]) if (steer == "hole" and f < 0.5) else (piv[1:] - tc[1:])
        c.pull_x[1:] += np.clip(0.3 * err, -0.0004, 0.0004)
        arm.label = label
        arm.set_pose(c.pull_x, arm.rot)
        yield
        if until is not None and until(c, f):
            break


def pull(c):
    """Pull the straps until the channel bears on the back of the board, then hold the tension."""
    c.pull_x = None
    bear = [0]

    def bearing(c, f):
        g = c.detail().get("measure", {}).get("gap_mm") or [9, 9]
        bear[0] = bear[0] + 1 if (max(g) < 0.5 and abs(f - TENSION_N) < 1.0) else 0
        return bear[0] >= 8

    yield from tension_gen(c, TENSION_N, 6.0, "pull straps", until=bearing, steer="hole")
    c.note("bearing")


def cap_push(c, tension=True):
    """Right: open fingertips straddling the straps behind the cap's flange, slide it along the straps into the
    hole until the flange is on the board; left keeps the straps taut meanwhile (``tension=False``: left has let go)."""
    arm = c.right
    arm.set_comp(True)
    kp, _ = c.pose("cap")
    sp, _ = c.pose("stub")
    hole = c.hole()
    rot = best_rot(arm, tcp_for_edge(kp, pitched(PUSH_PITCH)), theta=PUSH_PITCH)
    # aim at the straps' axis (taut straps: it runs through the hole); with slack straps (``tension=False``) they
    # droop, so aim at the cap itself
    aim = (lambda: c.pose("stub")[0]) if tension else (lambda: c.pose("cap")[0])
    tip0 = np.array([kp[0] - 0.0030, aim()[1], aim()[2]])
    open_half = L.STRAP_Y + L.STRAP_T / 2 + 0.003

    def right_moves():
        yield from arm.move(tcp_for_edge(tip0 + [-0.012, 0, 0.030], rot), rot, 2.5, grip_of(open_half), "right over cap")
        yield from arm.move(tcp_for_edge(tip0 + [-0.004, 0, 0.0], rot), rot, 1.5, None, "right behind cap")
        arm.set_comp("hold")  # in contact from here: offsets frozen; the push is closed on the measured cap position
        x = tip0[0] - 0.004
        goal = hole[0] - L.CAP_FLANGE_T + 0.0005  # flange outer face 0.5 mm past seated: presses it on the board
        for _ in range(300):
            kp_, _ = c.pose("cap")
            m = c.detail().get("measure", {})
            seated = m.get("proud_mm", 99.0) < 0.5
            if not seated and x - kp_[0] < 0.030:  # the hand may run ahead of the cap by its lag, at most 30 mm
                x += float(np.clip(0.4 * (goal - kp_[0]), 0.0, 0.0006))
            sp_ = aim()
            arm.set_pose(tcp_for_edge(np.array([x, sp_[1], sp_[2]]), rot), rot)
            arm.label = "slide cap"
            yield
            if seated:
                break
        arm.set_comp("hold")
        yield from wait(0.5)

    gen = right_moves()
    if tension:
        reg = tension_gen(c, TENSION_N, 120.0)  # the left regulates the tension while the right moves
        for _ in gen:
            next(reg)
            yield
    else:
        yield from gen
    c.note("cap seated")


def snap_off(c, side=1.0):
    """Left lets go; right bends the strap tail sideways at ~25 mm from the cap face until the straps snap."""
    left = c.left
    left.set_comp("hold")
    yield from left.set_grip(grip_of(0.009), 0.8, "left open")
    left.set_comp(True)
    yield from both(left.move(left.pos + np.array([-0.04, 0.0, 0.05]), left.rot, 1.5, None, "left clear"),
                    c.right.move(c.right.pos + np.array([-0.020, 0.0, 0.030]), c.right.rot, 1.5, 0.0, "right back"))
    yield from left.home(2.0)
    arm = c.right
    kp, _ = c.pose("cap")
    sp, sr = c.pose("stub")
    lever = 0.020
    y0 = sp[1] + side * (L.STRAP_Y + L.STRAP_T / 2 + 0.006)
    tip = np.array([kp[0] - lever, y0, sp[2] - 0.008])  # tips 8 mm below the straps: the fingers cover them
    rot = best_rot(arm, tcp_for_tip(tip, pitched()))
    yield from arm.move(tcp_for_tip(tip + [0, 0, 0.03], rot), rot, 2.0, 0.0, "right over tail")
    yield from arm.move(tcp_for_tip(tip, rot), rot, 1.5, None, "right beside tail")
    arm.set_comp("hold")
    for k in range(100):
        tip = tip + np.array([0.0, -side * 0.0008, 0.0])
        arm.set_pose(tcp_for_tip(tip, rot), rot)
        arm.label = "bend tail"
        yield
        if c.state()["broken"]:
            break
    c.note("snapped", broken=c.state()["broken"])
    arm.set_comp(True)
    yield from arm.move(arm.pos + np.array([-0.04, 0.0, 0.06]), rot, 1.5, None, "right away")
    yield from arm.home(2.5)


def demo(c):
    yield from wait(0.5)
    yield from pick(c)
    yield from carry(c)
    yield from insert(c)
    yield from handoff(c)
    yield from pull(c)
    yield from cap_push(c)
    yield from snap_off(c)
    yield from wait(4.0)


def idle(c):
    yield from wait(8.0)


def upto_pull(c):
    """Development: pick, insert, swing, pull and hold."""
    yield from wait(0.5)
    yield from pick(c)
    yield from carry(c)
    yield from insert(c)
    yield from handoff(c)
    yield from pull(c)
    yield from tension_gen(c, TENSION_N, 2.0)


def neg_no_tension(c):
    """The strap hand lets go after the pull, before the cap slides (one hand at a time): the cap's ratchet drags
    the straps in with it, the channel leaves the back face; then the straps are bent at the cap. Expected 0."""
    yield from wait(0.5)
    yield from pick(c)
    yield from carry(c)
    yield from insert(c)
    yield from handoff(c)
    yield from pull(c)
    left = c.left
    left.set_comp("hold")
    yield from left.set_grip(grip_of(0.009), 0.8, "left lets go")
    yield from left.move(left.pos + np.array([-0.04, 0.0, 0.05]), left.rot, 1.5, None, "left clear")
    c.note("straps let go")
    yield from cap_push(c, tension=False)
    yield from snap_off_right_only(c)
    yield from wait(4.0)


def snap_off_right_only(c, side=1.0):
    arm = c.right
    yield from arm.move(arm.pos + np.array([-0.020, 0.0, 0.030]), arm.rot, 1.5, 0.0, "right back")
    kp, _ = c.pose("cap")
    sp, _ = c.pose("stub")
    tip = np.array([kp[0] - 0.020, sp[1] + side * (L.STRAP_Y + L.STRAP_T / 2 + 0.006), sp[2] - 0.008])
    rot = best_rot(arm, tcp_for_tip(tip, pitched()))
    yield from arm.move(tcp_for_tip(tip + [0, 0, 0.03], rot), rot, 2.0, 0.0, "right over tail")
    yield from arm.move(tcp_for_tip(tip, rot), rot, 1.5, None, "right beside tail")
    arm.set_comp("hold")
    for k in range(60):
        tip = tip + np.array([0.0, -side * 0.0008, 0.0])
        arm.set_pose(tcp_for_tip(tip, rot), rot)
        arm.label = "bend tail"
        yield
    c.note("bent", broken=c.state()["broken"])
    arm.set_comp(True)
    yield from arm.move(arm.pos + np.array([-0.04, 0.0, 0.06]), rot, 1.5, None, "right away")
    yield from arm.home(2.5)


def neg_no_snap(c):
    """Set correctly, straps left on: 0.6."""
    yield from wait(0.5)
    yield from pick(c)
    yield from carry(c)
    yield from insert(c)
    yield from handoff(c)
    yield from pull(c)
    yield from cap_push(c)
    left = c.left
    left.set_comp("hold")
    yield from left.set_grip(grip_of(0.009), 0.8, "left open")
    left.set_comp(True)
    yield from both(left.move(left.pos + np.array([-0.04, 0.0, 0.05]), left.rot, 1.5, None, "left clear"),
                    c.right.move(c.right.pos + np.array([-0.04, 0.0, 0.05]), c.right.rot, 1.5, 0.0, "right clear"))
    yield from both(left.home(2.5), c.right.home(2.5))
    yield from wait(4.0)


def neg_early_pull(c):
    """Pulls the straps back right after the channel is pushed in, before it has swung across: the folded channel
    comes back out of the hole. Expected 0."""
    yield from wait(0.5)
    yield from pick(c)
    yield from carry(c)
    yield from insert(c)
    arm = c.right
    arm.set_comp("hold")
    yield from arm.set_grip(grip_of(L.CH_W / 2 + 0.008), 0.8, "right open")
    yield from arm.move(arm.pos + np.array([-0.030, 0.0, 0.050]), arm.rot, 1.5, None, "right clear")
    capture(c, "stub", ("left",))
    p0, _ = c.pose("stub")
    p1 = p0 + np.array([-0.06, 0.0, 0.0])
    yield from track(c, "stub", ("left",), lambda s: (p0 + s * (p1 - p0), L.S_SET), 3.0, "pull back", gain=0.02)
    c.note("pulled early")
    yield from wait(1.0)
    yield from cap_push(c, tension=False)
    yield from wait(3.0)


def pinch_bend(c):
    """Alternative snap-off (the maker's "bend side to side" with the hand already holding the straps): set as in
    the demo with the straps taut, the right hand moves clear, then the LEFT hand, still pinching the pull tab, rocks it
    sideways until the straps snap at the cap; it then lets go and both arms go home. Expected 1.0."""
    yield from wait(0.5)
    yield from pick(c)
    yield from carry(c)
    yield from insert(c)
    yield from handoff(c)
    yield from pull(c)
    yield from cap_push(c)
    right = c.right
    right.set_comp(True)
    yield from right.move(right.pos + np.array([-0.040, 0.0, 0.050]), right.rot, 1.5, None, "right clear")
    yield from right.home(2.0)
    left = c.left
    left.set_comp("hold")
    for side in (1.0, -1.0, 1.0, -1.0):
        for _ in range(40):
            left.set_pose(left.pos + np.array([0.0, side * 0.0006, 0.0]), left.rot)
            left.label = "rock tab"
            yield
            if c.state()["broken"]:
                break
        if c.state()["broken"]:
            break
        for _ in range(40):
            left.set_pose(left.pos + np.array([0.0, -side * 0.0006, 0.0]), left.rot)
            left.label = "rock tab"
            yield
            if c.state()["broken"]:
                break
        if c.state()["broken"]:
            break
    c.note("pinch-bent", broken=c.state()["broken"])
    yield from left.set_grip(grip_of(0.009), 0.8, "left open")
    left.set_comp(True)
    yield from left.move(left.pos + np.array([-0.04, 0.0, 0.05]), left.rot, 1.5, None, "left clear")
    yield from left.home(2.0)
    yield from wait(4.0)


def replay(c):
    """Open-loop check: plays the 14-D commands recorded in a finished run's trace.jsonl (path in SNAP_REPLAY), step
    by step, without reading anything: what a fixed trajectory scores on another start state."""
    import os

    rows = [json.loads(line) for line in open(os.environ["SNAP_REPLAY"])]
    c.replay = [r["cmd"] for r in rows]
    for _ in range(len(c.replay) + 40):
        yield


STAGES = {
    "demo": demo,
    "idle": idle,
    "upto-pull": upto_pull,
    "replay": replay,
    "neg-no-tension": neg_no_tension,
    "neg-no-snap": neg_no_snap,
    "neg-early-pull": neg_early_pull,
    "pinch-bend": pinch_bend,
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
    max_steps=3000,
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
    # one seed for all trials, or a comma-separated list matched to the stages ("none" = unseeded named setup)
    seeds = [None if v in (None, "none") else int(v) for v in (str(seed).split(",") if seed is not None else [None])]
    results, resets = [], []
    for i, st in enumerate(stages):
        tdir = out if len(stages) == 1 else out / f"trial{i}_{st}"
        res, reset_state = run_trial(emb, tdir, max_steps, record, st, t0, seed=seeds[min(i, len(seeds) - 1)])
        results.append(res)
        resets.append(reset_state)
    if len(stages) > 1:
        rep = reset_report(stages, resets, results)
        (out / "reset_check.json").write_text(json.dumps(rep, indent=2, default=float))
        print("RESETCHECK", json.dumps(rep["summary"]), flush=True)
    return results


def reset_state_now():
    import snapkit

    rt = snapkit.runtime()
    poses = rt.read()
    return dict(
        score=snapkit.STATE["score"],
        setup=rt.setup,
        dofs={k: round(v, 5) for k, v in rt.dofs().items()},
        broken=rt.broken,
        bodies={n: np.round(p * 1000, 4).tolist() for n, (p, r) in poses.items()},
        rots={n: np.round(r, 6).tolist() for n, (p, r) in poses.items()},
    )


def reset_report(stages, resets, results):
    import snapkit

    rows = []
    for i, r in enumerate(resets):
        setup = tuple(r["setup"]) if isinstance(r["setup"], list) else r["setup"]
        want = snapkit.authored(setup)
        dev = max(float(np.linalg.norm(np.subtract(r["bodies"][n], want[n][0] * 1000)))
                  for n in ("stub", "channel", "cap", "tail", "wall", "block"))
        rows.append(dict(trial=i, stage=stages[i], setup=r["setup"], score_after_reset=r["score"],
                         broken_after_reset=r["broken"], max_body_dev_from_authored_mm=round(dev, 4),
                         trial_score=results[i]["task_score"], trial_stop=results[i]["stop"]))
    ok = all(x["score_after_reset"] == 0.0 and x["max_body_dev_from_authored_mm"] < 1.0 and not x["broken_after_reset"]
             for x in rows)
    return dict(summary=dict(passed=ok, trials=len(rows),
                             max_body_dev_mm=max(x["max_body_dev_from_authored_mm"] for x in rows)),
                rows=rows, reset_states=resets)


def run_trial(emb, out, max_steps, record, stage, t0, seed=None):
    import snapkit
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
        d = snapkit.STATE["detail"]
        row = dict(
            step=steps,
            score=info.get("score"),
            level=snapkit.STATE["level"],
            detail=d,
            labels=[c.left.label, c.right.label],
            nonfinite=snapkit.STATE["nonfinite"],
            cmd=np.round(np.r_[c.left.last_cmd, c.right.last_cmd], 4).tolist(),
            q=np.round(jp, 4).tolist(),
            corr_mm=np.round(c.corr_p * 1000, 3).tolist(),
            corr_r=np.round(c.corr_r, 4).tolist(),
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
            m = d.get("measure", {})
            print(
                f"step {steps} {c.left.label} | {c.right.label} score {info.get('score')} phi {d.get('phi_deg')} "
                f"q {d.get('q_mm')} gap {m.get('gap_mm')} proud {m.get('proud_mm')} play {m.get('play_mm')} "
                f"broken {d.get('broken')} c {d.get('contacts')} wall {time.time() - t1:.0f}s",
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
        scorer=dict(score=snapkit.STATE["score"], level=snapkit.STATE["level"]),
        final=snapkit.STATE["detail"],
        nonfinite=snapkit.STATE["nonfinite"],
        looks=c.log,
        wall_s=round(time.time() - t1, 1),
        wall_total_s=round(time.time() - t0, 1),
        worst_ik_mm=dict(
            left=round(c.left.worst_mm, 2), right=round(c.right.worst_mm, 2)
        ),
        physics_dt=snapkit.PHYSICS_DT,
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
    ap.add_argument("--max-steps", type=int, default=3000)
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
        default=None,
        help="reset seed (seeded start state, as in inspect-robots runs); comma-separated: one per stage",
    )
    a = ap.parse_args()
    cw, ch = (int(v) for v in a.cam_size.split("x"))
    run(a.task, a.out, a.max_steps, not a.no_record, a.stage, cw, ch, a.device, a.seed)
