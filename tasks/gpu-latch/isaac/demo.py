"""Scripted full-score demo (and negative variants) for the gpu-latch task in the yam_arena embodiment.

    python demo.py task.py OUT [--max-steps N] [--no-record] [--stage demo|<variant>[,<variant>...]] [--seed S]

Acts ONLY through the 14 joint/gripper commands of ``YamArenaEmbodiment.step``. It reads the card's, the benchtable's
and the latch's poses through ``gpukit.rig_state()`` (read-only, like a person watching the card and the latch); it
never writes a pose, a joint state or a score. Its joint commands are rate-limited below the CLI's per-step delta
guard and clipped to the URDF joint bounds.

Plan (left hand on the latch, right hand on the card):
1. Read where the benchtable (slot frame) stands.
2. In parallel: the right hand opens over the card's top edge and closes on it near its centre of mass (pads on the
   cooler shroud and the back cover); the left hand, closed, comes in pitched from the robot's side beside the card's
   back and lowers its fingertips onto the latch's thumb pad.
3. The left hand presses the pad down until the latch is seen tipped open (closed loop on the latch angle) and keeps
   pressing.
4. The right hand draws the card straight up (closed loop on the measured card height; a slow integral winds up the
   pull against the contacts' friction). Once the card's lip is above the hook, the left hand lets go and backs away.
5. The right hand lifts the card clear of the case rim, carries it over the mat and keeps its gripped edge raised
   until the opposite edge rests on the mat. The gripper opens slowly and both arms park.
The arms add the remaining joint error back to their commands (slow integral on their own joint readings), because
the joint PD sags under load (pattern reused from earlier demos).
"""

from __future__ import annotations

import argparse
import json
import math
import os
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
SQUEEZE = 0.006  # fingers commanded 6 mm inside the card faces: about 30 N per finger (gripper 5 kN/m)
OPEN_CLEAR = 0.016  # clearance a side before closing
CARD_HALF = (2.3 + 15.3) / 2 * L.MM  # half the card's thickness across the shroud and back faces
CARD_MID_N = (2.3 - 15.3) / 2 * L.MM
GRIP_U = 0.112  # grip point along the card (m, K frame u)
LIFT_TILT = 45.0  # deg: right-hand approach tilt in the card's plane
PRESS_PITCH = 35.0  # deg from vertical, leaning back toward the robot
PRESS_N = 0.0080  # fingertip target across the card (K n, m): beside the back cover (n 2.3 mm)
PRESS_U = 0.1495
PRESS_OVER = 0.004  # press target below the pad's top
PRESS_OPEN_DEG = 17.0  # latch angle seen as "open"
PULL_RATE = 0.010  # m/s command rise while drawing the card
PULL_LEAD = 0.012
LIP_CLEAR = 0.008  # card rise after which the hook can no longer catch the lip
CLEAR_RISE = L.CASE_RIM * L.MM + 0.018  # card rise before carrying (finger edge 25 mm above the bracket bar)
LAY_DEG = -50.0  # card turned this far from upright before it is set down
LAY_YAW = 0.0  # and turned about the vertical so it lies along x on the mat
VIEW_TARGET = (130.0, 75.0)  # mm (S frame u, z): the point the setup view looks at
VIEW_EYE = (0.18, 0.03, 0.10)  # m: view eye offset from it (toward the robot, along the slot, up)
PARK = {"left": (0.150, 0.330, 0.990), "right": (0.150, -0.330, 0.990)}


class Stop(Exception):
    pass


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
        self.offset_clip = 0.25

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
        """True: joint integral (free moves and carrying); 'hold': offsets frozen; False: offsets decay."""
        self.comp = mode

    def observe(self, q):
        self.measured = np.asarray(q[:6], float)
        self.measured_grip = float(q[6])
        if self.comp == "hold":
            pass
        elif self.comp:
            self.offset = np.clip(self.offset + COMP_GAIN * (self.q - self.measured), -self.offset_clip,
                                  self.offset_clip)
        else:
            self.offset *= 0.85

    def relax(self):
        self.set_comp("hold")
        self.q = self.measured.copy()
        self.pos, self.rot = fastik.tcp_pose(self.q, self.base)

    def measured_tcp(self):
        return fastik.tcp_pose(self.measured, self.base)

    def solve(self, pos, rot):
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
            raise Stop(f"{self.name} unreachable at {self.label}: {pe * 1000:.1f} mm {math.degrees(re):.1f} deg")
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), np.asarray(rot, float)

    def move(self, pos, rot, duration, grip=None, label=""):
        from yam_arena import ik

        self.label = label or self.label
        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self.set_pose(p0 + s * (np.asarray(pos) - p0), ik.slerp_matrix(r0, np.asarray(rot), s))
            yield

    def set_grip(self, grip, duration=0.8, label=""):
        yield from self.move(self.pos, self.rot, duration, grip, label)


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
        self.corr = {}

    def pose(self, name="card"):
        import gpukit

        return gpukit.rig_state()[name]

    def latch_deg(self):
        import gpukit

        return gpukit.rig_state()["latch_deg"]

    def detail(self):
        import gpukit

        return gpukit.STATE["detail"]

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
        row = dict(at=tag, step=self.step, measure=d.get("measure"), level=d.get("level"), **kw)
        self.log.append(row)
        print("LOOK", json.dumps(row, default=float), flush=True)


def grip_of(half_opening_m):
    return float(np.clip(half_opening_m / FINGER_OPEN, 0.0, 1.0))


def slot(c):
    return c.pose("bench")


def card_point(c, k_point):
    """A card-frame point (m) in the station frame, from the measured card pose."""
    pk, rk = c.pose("card")
    return pk + rk @ np.asarray(k_point, float)


def rise(c):
    pk, rk = c.pose("card")
    ps, rs = slot(c)
    return L.finger_bottom_rise(pk, rk, ps, rs)


def lift_rot(rs, tilt_deg=None):
    """Right-hand TCP rotation over the card: approach tilted LIFT_TILT from straight down toward the latch end (in the
    card's plane, the hand coming from the bracket side), fingers closing across the card (along n)."""
    a = math.radians(LIFT_TILT if tilt_deg is None else tilt_deg)
    u = rs[:, 0].copy()
    u[2] = 0.0
    u /= np.linalg.norm(u)
    x = -rs[:, 1].copy()
    x[2] = 0.0
    x /= np.linalg.norm(x)
    z = np.array([0.0, 0.0, -math.cos(a)]) + math.sin(a) * u
    return np.column_stack([x, np.cross(z, x), z])


def press_rot(rs, pitch_deg=PRESS_PITCH):
    """Left-hand TCP rotation for the latch press: approach pitched from straight down, leaning back toward the
    robot (+n), the closed fingers side by side along the slot (u)."""
    a = math.radians(pitch_deg)
    n = rs[:, 1].copy()
    n[2] = 0.0
    n /= np.linalg.norm(n)
    z = np.array([0.0, 0.0, -math.cos(a)]) - math.sin(a) * n
    x = -rs[:, 0].copy()
    x -= (x @ z) * z
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def capture(c, hand, ref="card"):
    pr, rr = c.pose(ref)
    pt, rt = c.arms[hand].measured_tcp()
    c.G[(hand, ref)] = (rr.T @ (pt - pr), rr.T @ rt)


def tcp_for_card(c, hand, pk, rk):
    """TCP pose that puts the card at (pk, rk) through the hand's frozen grasp transform."""
    gp, gr = c.G[(hand, "card")]
    return pk + rk @ gp, rk @ gr


def track(c, hand, target_fn, duration, label, gain=0.05, until=None, max_s=None, clip=0.015):
    """Closed loop on the held card: each step command the TCP pose that puts the card at target_fn(s) = (pos, rot)
    through the frozen grasp transform, plus a slow integral correction of the measured card position error."""
    from yam_arena import ik

    n = max(1, round(duration * CONTROL_HZ))
    nmax = n if max_s is None else max(n, round(max_s * CONTROL_HZ))
    cp = c.corr.get(hand, np.zeros(3))
    for k in range(1, nmax + 1):
        pt, rt = target_fn(ik.smoothstep(min(1.0, k / n)))
        pm, _ = c.pose("card")
        cp = np.clip(cp + gain * (pt - pm), -clip, clip)
        tp, tr = tcp_for_card(c, hand, pt + cp, rt)
        arm = c.arms[hand]
        arm.label = label
        arm.set_pose(tp, tr)
        c.corr[hand] = cp
        yield
        if until is not None and k >= n and until(c):
            break


# ------------------------------------------------------------------------------------------------ phases
def grasp_card(c, hand="right"):
    arm = c.arms[hand]
    arm.set_comp(True)
    ps, rs = slot(c)
    rot = lift_rot(rs)
    g = np.array([GRIP_U, CARD_MID_N, L.GRIP_Z * L.MM])
    p = card_point(c, g)
    yield from arm.move(p + [0, 0, 0.070], rot, 3.0, grip_of(CARD_HALF + OPEN_CLEAR), f"{hand} over the card")
    yield from wait(0.2)
    p = card_point(c, g)
    yield from arm.move(p, rot, 1.8, None, f"{hand} down onto the card")
    yield from wait(0.3)
    yield from arm.set_grip(grip_of(CARD_HALF - SQUEEZE), 0.8, f"{hand} close")
    yield from wait(0.6)
    capture(c, hand)
    c.note("card gripped", hand=hand)


def reach_latch(c, hand="left"):
    arm = c.arms[hand]
    arm.set_comp(True)
    ps, rs = slot(c)
    rot = press_rot(rs)
    tip = ps + rs @ np.array([PRESS_U, PRESS_N, L.PAD_CENTER[2] * L.MM])
    tcp = tip - rot[:, 2] * L_TIP
    yield from arm.move(tcp - rot[:, 2] * 0.060 + [0, 0, 0.02], rot, 3.0, 0.0, f"{hand} to the latch")
    yield from wait(0.2)
    yield from arm.move(tcp - rot[:, 2] * 0.006, rot, 1.5, 0.0, f"{hand} over the pad")
    c.note("over the pad", hand=hand, latch_deg=c.latch_deg())


L_TIP = 0.0135  # TCP to the closed fingertips' end along the approach axis (yam_arena PAD_TIP_PAST_TCP + 0.5 mm)


def press_latch(c, hand="left", max_s=6.0):
    """Lower the fingertips onto the pad (straight down) until the latch is seen tipped open, then press 2 mm further
    and keep pressing (the command stays there)."""
    arm = c.arms[hand]
    arm.set_comp("hold")
    rot = arm.rot.copy()
    z0 = arm.pos[2]
    cnt = 0
    for k in range(round(max_s * CONTROL_HZ)):
        th = c.latch_deg()
        cnt = cnt + 1 if th >= PRESS_OPEN_DEG else 0
        if cnt >= 4:
            break
        dz = min(0.0006 * (k + 1), 0.030)
        arm.label = "press the latch"
        arm.set_pose(np.array([arm.pos[0], arm.pos[1], z0 - dz]), rot)
        yield
    for k in range(10):
        arm.set_pose(np.array([arm.pos[0], arm.pos[1], arm.pos[2] - 0.0002]), rot)
        yield
    c.note("latch pressed", hand=hand, latch_deg=c.latch_deg())


def draw_card(c, hand="right", presser="left", until_rise=LIP_CLEAR, max_s=12.0):
    """Pull the card straight up by its grasp: the commanded card height stays at most PULL_LEAD above the measured
    one and rises at PULL_RATE; the joint integral winds up the pull against the contacts' friction. The presser keeps
    its press until the lip is above the hook."""
    arm = c.arms[hand]
    arm.offset_clip = 0.45
    arm.set_comp(True)
    pk0, rk0 = c.pose("card")
    z_cmd = pk0[2]
    for k in range(round(max_s * CONTROL_HZ)):
        pk, _ = c.pose("card")
        z_cmd = min(z_cmd + PULL_RATE / CONTROL_HZ, pk[2] + PULL_LEAD)
        tp, tr = tcp_for_card(c, hand, np.array([pk0[0], pk0[1], z_cmd]), rk0)
        arm.label = "draw the card"
        arm.set_pose(tp, tr)
        yield
        if rise(c) >= until_rise:
            break
    c.note("card drawn", hand=hand, rise_mm=rise(c) * 1e3, latch_deg=c.latch_deg())


def let_go_latch(c, hand="left"):
    arm = c.arms[hand]
    arm.set_comp(True)
    yield from arm.move(arm.pos - arm.rot[:, 2] * 0.050 + [0, 0, 0.030], arm.rot, 1.2, 0.0, f"{hand} off the latch")
    yield from park(c, hand)


def lift_clear(c, hand="right"):
    """Lift the card straight up 60 mm (out of the slot and clear of the board's parts), then on up and 40 mm back
    toward the robot (easier reach) until its gold fingers are well above the bracket bar."""
    pk0, rk0 = c.pose("card")
    ps, rs = slot(c)
    mid = pk0 + np.array([0.0, 0.0, 0.060])
    top = np.array([pk0[0] - 0.040, pk0[1], ps[2] + CLEAR_RISE + 0.004])
    c.corr[hand] = np.zeros(3)
    yield from track(c, hand, lambda s: (pk0 + s * (mid - pk0), rk0), 1.5, f"{hand} lift clear", gain=0.03)
    yield from track(c, hand, lambda s: (mid + s * (top - mid), rk0), 1.8, f"{hand} lift clear", gain=0.03)
    c.note("lifted clear", rise_mm=rise(c) * 1e3)


def lay_on_mat(c, hand="right"):
    """Keep the gripped edge raised while lowering the far edge onto the mat."""
    from yam_arena import ik

    pk0, rk0 = c.pose("card")
    u = rk0[:, 0].copy()
    u[2] = 0.0
    u /= np.linalg.norm(u)
    lo, hi = L.card_box()
    ctr_k = (lo + hi) / 2
    # The upper edge swings toward the robot as the card tips onto the mat.
    # Contact the inner half first, leaving space for that swing and gripper withdrawal.
    mat = np.array([L.MAT_CENTER[0], L.MAT_CENTER[1] + 0.025, 0.0])

    def rot_about(axis, deg):
        from scipy.spatial.transform import Rotation

        return Rotation.from_rotvec(np.asarray(axis, float) * math.radians(deg)).as_matrix()

    r_lay = rot_about([0.0, 0.0, 1.0], LAY_YAW) @ rot_about(-u, LAY_DEG) @ rk0
    # the card's body centre over the mat centre, its lowest corner 30 mm above the mat
    def pose_at(height):
        c_w = r_lay @ ctr_k
        corners = L.card_vertices(np.zeros(3), r_lay)
        pk = np.array([mat[0], mat[1], L.MAT_TOP + height - corners[:, 2].min()]) - np.r_[c_w[:2], 0.0]
        return pk

    # Move out over the mat before rolling the card. This keeps the wrist
    # on the free side of the tower wall throughout the descent.
    outside = pk0.copy()
    outside[1] = mat[1] - (rk0 @ ctr_k)[1]
    yield from track(c, hand, lambda s: (pk0 + s * (outside - pk0), rk0), 2.0,
                     f"{hand} clear the chassis", gain=0.025)
    capture(c, hand)
    pk0, rk0 = c.pose("card")
    over = pose_at(L.BOARD_Z - L.TOP + L.CASE_RIM * L.MM + 0.018)
    yield from track(c, hand, lambda s: (pk0 + s * (over - pk0), ik.slerp_matrix(rk0, r_lay, s)), 4.0,
                     f"{hand} carry and turn the card", gain=0.0)
    # the card may have turned a little in the hand: re-take the grasp transform from the measured poses, then square
    # the card's length along x over the mat's centre (closed loop on the measured card pose)
    capture(c, hand)
    pk1, rk1 = c.pose("card")
    err = math.atan2(rk1[1, 0], rk1[0, 0])
    if abs(err) > math.pi / 2:  # the card's length along -x is as good as along +x
        err -= math.copysign(math.pi, err)
    r_lay = rot_about([0.0, 0.0, 1.0], -math.degrees(err)) @ rk1
    over2 = pose_at(0.110)
    yield from track(c, hand, lambda s: (pk1 + s * (over2 - pk1), ik.slerp_matrix(rk1, r_lay, s)), 1.5,
                     f"{hand} square the card over the mat", gain=0.0)
    capture(c, hand)
    over, _ = c.pose("card")
    low = pose_at(0.0)
    cnt = [0]

    def touched(cc):
        pk, rk = cc.pose("card")
        z = L.card_vertices(pk, rk)[:, 2].min()
        cnt[0] = cnt[0] + 1 if z < L.MAT_TOP + 0.0015 else 0
        return cnt[0] >= 2

    yield from track(c, hand, lambda s: (over + s * (low + [0, 0, -0.002] - over), r_lay), 3.0,
                     f"{hand} lower to the mat", gain=0.0, until=touched, max_s=4.0)
    arm = c.arms[hand]
    for _ in range(int(4 * CONTROL_HZ)):
        pk, rk = c.pose("card")
        gap = L.card_vertices(pk, rk)[:, 2].min() - L.MAT_TOP
        if gap < 0.0025 and c.detail()["measure"]["mat_load_n"] > 0.05 and abs(rk[2, 1]) > math.cos(math.radians(48)):
            break
        if gap < 0.0025 and abs(rk[2, 1]) <= math.cos(math.radians(48)):
            raise Stop("card reached mat at too steep an angle")
        arm.set_pose(arm.pos - [0, 0, min(max(gap, 0.0001), 0.0008)], arm.rot)
        yield
    else:
        raise Stop("no measured mat support; card retained")
    c.note("card supported on mat", clearance_mm=round(gap * 1000, 2), mat_load_n=c.detail()["measure"]["mat_load_n"])
    arm = c.arms[hand]
    arm.relax()
    yield from wait(0.2)
    yield from arm.set_grip(grip_of(CARD_HALF + 0.012), 1.2, f"{hand} open")
    yield from wait(0.5)
    arm.set_comp(True)
    yield from arm.move(arm.pos - arm.rot[:, 2] * 0.06 + [0, 0, 0.05], arm.rot, 1.5, None, f"{hand} back off")
    yield from park(c, hand)


def park(c, hand, duration=2.5):
    """Joint-space move to a parked pose near the arm's base (TCP at PARK, the READY orientation), clear of the bench
    and the mat in the overhead view."""
    from yam_arena import ik
    from yam_arena.constants import READY

    arm = c.arms[hand]
    arm.set_comp(True)
    arm.label = f"{hand} park"
    _, r_ready = fastik.tcp_pose(np.array(READY, float), arm.base)
    q1, pe, re = fastik.solve(np.array(PARK[hand]), r_ready, np.array(READY, float), base=arm.base, iters=200)
    q0 = arm.q.copy()
    g0 = arm.grip
    n = max(1, round(duration * CONTROL_HZ))
    for k in range(1, n + 1):
        s = ik.smoothstep(k / n)
        arm.q = q0 + s * (q1 - q0)
        arm.grip = g0 + s * (0.3 - g0)
        arm.pos, arm.rot = fastik.tcp_pose(arm.q, arm.base)
        yield


def demo(c):
    yield from wait(0.3)
    yield from both(grasp_card(c), reach_latch(c))
    yield from press_latch(c)
    yield from draw_card(c)
    yield from both(let_go_latch(c), lift_clear(c))
    yield from lay_on_mat(c)
    yield from wait(1.5)


def idle(c):
    yield from wait(8.0)


def view(c):
    """Setup picture (media only): the left hand, gripper open, moves its wrist camera to an oblique view from the
    robot's side and above, looking at the card's back, the slot and the latch; nothing is touched."""
    ps, rs = slot(c)
    target = ps + rs @ (np.array([VIEW_TARGET[0], 0.0, VIEW_TARGET[1]]) * L.MM)
    n = rs[:, 1].copy()
    n[2] = 0.0
    n /= np.linalg.norm(n)
    u = rs[:, 0].copy()
    u[2] = 0.0
    u /= np.linalg.norm(u)
    eye = target + n * VIEW_EYE[0] + u * VIEW_EYE[1] + np.array([0.0, 0.0, VIEW_EYE[2]])
    z = target - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, [0.0, 0.0, 1.0])
    x /= np.linalg.norm(x)
    rot = np.column_stack([x, np.cross(z, x), z])
    c.left.set_comp(True)
    yield from c.left.move(eye, rot, 3.0, 1.0, "left to the view pose")
    yield from wait(1.5)


def grip_test(c):
    """(dev) grasp the card, open the latch with the left hand, pull 30 mm; logs the grasp contacts."""
    yield from wait(0.3)
    yield from both(grasp_card(c), reach_latch(c))
    c.note("grip links", links=c.detail().get("grip_links"), fric=c.detail().get("grip_fric"))
    yield from press_latch(c)
    yield from draw_card(c, until_rise=0.030, max_s=6.0)
    c.note("grip links after", links=c.detail().get("grip_links"), fric=c.detail().get("grip_fric"))


def neg_nopress(c):
    """The card drawn up without pressing the latch (the left hand stays at READY): the hook holds; 0."""
    yield from wait(0.3)
    yield from grasp_card(c)
    yield from draw_card(c, max_s=8.0)
    yield from wait(2.0)


def neg_early(c):
    """The latch pressed open and then let go before the card is drawn (the hook springs back): 0."""
    yield from wait(0.3)
    yield from both(grasp_card(c), reach_latch(c))
    yield from press_latch(c)
    yield from let_go_latch(c)
    yield from draw_card(c, max_s=8.0)
    yield from wait(2.0)


def neg_held(c):
    """Card drawn out and lifted clear, but held in the air to the end: 0.5."""
    yield from wait(0.3)
    yield from both(grasp_card(c), reach_latch(c))
    yield from press_latch(c)
    yield from draw_card(c)
    yield from both(let_go_latch(c), lift_clear(c))
    yield from wait(4.0)


def replay(c):
    """Open-loop check: plays the 14-D commands recorded in a finished run's trace.jsonl (path in GPU_REPLAY), step by
    step, without reading anything. GPU_REPLAY_MIRROR=1 plays its left-right mirror image instead."""
    rows = [json.loads(line) for line in open(os.environ["GPU_REPLAY"])]
    c.replay = [r["cmd"] for r in rows]
    if os.environ.get("GPU_REPLAY_MIRROR") == "1":  # arms swapped, joints 1, 5 and 6 negated

        def mirror(a):
            a = np.asarray(a, float)
            m = np.r_[a[7:14], a[0:7]]
            m[[0, 4, 5, 7, 11, 12]] *= -1.0
            return m.tolist()

        c.replay = [mirror(a) for a in c.replay]
    for _ in range(len(c.replay) + 40):
        yield


OVERRIDES: dict = {}  # development reset overrides per stage (none for the shipped stages)

STAGES = {
    "demo": demo,
    "idle": idle,
    "view": view,
    "replay": replay,
    "neg-nopress": neg_nopress,
    "grip-test": grip_test,
    "neg-early": neg_early,
    "neg-held": neg_held,
}


def _write_mp4(frames, path):
    import imageio

    w = imageio.get_writer(str(path), fps=CONTROL_HZ, codec="libx264", quality=8, macro_block_size=8,
                           ffmpeg_params=["-pix_fmt", "yuv420p"])
    for f in frames:
        w.append_data(np.asarray(f))
    w.close()


def run(task_path, out_dir, max_steps=3000, record=True, stage="demo", cam_width=320, cam_height=176, device="cpu",
        seed=None, seeds=None):
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    t0 = time.time()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=str(Path(task_path).resolve()), object_jitter=0, control_hz=CONTROL_HZ,
                               cam_width=cam_width, cam_height=cam_height, device=device)
    stages = stage.split(",")
    trial_seeds = [None if x in ("none", "") else int(x) for x in seeds.split(",")] if seeds else [seed] * len(stages)
    results, resets = [], []
    for i, st in enumerate(stages):
        tdir = out if len(stages) == 1 else out / f"trial{i}_{st}"
        res, reset_state = run_trial(emb, tdir, max_steps, record, st, t0, seed=trial_seeds[i])
        results.append(res)
        resets.append(reset_state)
    if len(stages) > 1:
        rep = reset_report(stages, resets, results)
        (out / "reset_check.json").write_text(json.dumps(rep, indent=2, default=float))
        print("RESETCHECK", json.dumps(rep["summary"]), flush=True)
    return results


def reset_state_now():
    import gpukit

    rt = gpukit.runtime()
    poses = rt.read()
    return dict(score=gpukit.STATE["score"], setup=rt.setup,
                bodies={n: np.round(p * 1000, 4).tolist() for n, (p, r) in poses.items()},
                rots={n: np.round(r, 6).tolist() for n, (p, r) in poses.items()})


def reset_report(stages, resets, results):
    import gpukit

    rows = []
    for i, r in enumerate(resets):
        want = gpukit.authored(tuple(r["setup"]) if isinstance(r["setup"], list) else r["setup"])
        dev = max(float(np.linalg.norm(np.subtract(r["bodies"][n], want[n][0] * 1000))) for n in gpukit.BODIES)
        rdev = float(np.abs(np.subtract(r["rots"]["card"], want["card"][1])).max())
        rows.append(dict(trial=i, stage=stages[i], setup=r["setup"], score_after_reset=r["score"],
                         max_body_dev_from_authored_mm=round(dev, 4), max_rot_dev=round(rdev, 6),
                         trial_score=results[i]["task_score"], trial_stop=results[i]["stop"]))
    ok = all(x["score_after_reset"] == 0.0 and x["max_body_dev_from_authored_mm"] < 1.0 for x in rows)
    return dict(summary=dict(passed=ok, trials=len(rows),
                             max_body_dev_mm=max(x["max_body_dev_from_authored_mm"] for x in rows)),
                rows=rows, reset_states=resets)


_SLOW = [0]


def _watchdog(step_s):
    """Optional (GPU_WATCHDOG_S set): end the process if five control steps in a row each take longer than that many
    seconds of wall time (a stalled renderer)."""
    lim = float(os.environ.get("GPU_WATCHDOG_S", "0") or 0)
    if lim <= 0:
        return
    _SLOW[0] = _SLOW[0] + 1 if step_s > lim else 0
    if _SLOW[0] >= 5:
        print(f"WATCHDOG: 5 control steps over {lim} s each; ending the run", flush=True)
        os._exit(3)


def run_trial(emb, out, max_steps, record, stage, t0, seed=None):
    import gpukit
    from inspect_robots import Scene
    from inspect_robots_yam.packing import STATE_KEY

    out.mkdir(parents=True, exist_ok=True)
    t1 = time.time()
    gpukit._RT["override"] = OVERRIDES.get(stage)
    wd = None
    lim = float(os.environ.get("GPU_RESET_WATCHDOG_S", "0") or 0)
    if lim > 0:  # optional: end the process if the reset (Kit start-up / renderer) stalls longer than this
        import threading

        wd = threading.Timer(lim, lambda: (print(f"WATCHDOG: reset over {lim} s; ending the run", flush=True),
                                           os._exit(3)))
        wd.daemon = True
        wd.start()
    obs = emb.reset(Scene(id="demo", instruction="demo"), seed=seed)
    if wd is not None:
        wd.cancel()
    print(f"RIG reset {time.time() - t1:.0f} s", flush=True)
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
        t_step = time.time()
        res = emb.step(c.action())
        _watchdog(time.time() - t_step)
        jp = np.asarray(res.observation.state[STATE_KEY], float).ravel()
        c.left.observe(jp[:7])
        c.right.observe(jp[7:14])
        steps += 1
        c.step = steps
        info = res.info
        snap(res.observation)
        d = gpukit.STATE["detail"]
        row = dict(step=steps, score=info.get("score"), level=gpukit.STATE["level"], detail=d,
                   labels=[c.left.label, c.right.label], nonfinite=gpukit.STATE["nonfinite"],
                   cmd=np.round(np.r_[c.left.last_cmd, c.right.last_cmd], 4).tolist(), q=np.round(jp, 4).tolist(),
                   corr_mm={k: np.round(v * 1000, 3).tolist() for k, v in c.corr.items()},
                   tcp_mm={n: np.round(a.measured_tcp()[0] * 1000, 2).tolist() for n, a in c.arms.items()},
                   tcp_cmd_mm={n: np.round(a.pos * 1000, 2).tolist() for n, a in c.arms.items()},
                   offs={n: np.round(a.offset, 3).tolist() for n, a in c.arms.items()})
        tracef.write(json.dumps(row, default=float) + "\n")
        if steps % 20 == 0:
            tracef.flush()
        if steps % 25 == 0:
            m = d.get("measure", {}) or {}
            print(f"step {steps} {c.left.label} | {c.right.label} score {info.get('score')} rise {m.get('rise_mm')} "
                  f"latch {m.get('latch_deg')} rel {m.get('rel_mm')} tip {m.get('tip_mm')} hook {d.get('hook_n')} "
                  f"grip {d.get('grip_peak_n')} press {d.get('press_now_n')} wall {time.time() - t1:.0f}s", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= 2 * CONTROL_HZ:
            break
    score = float(info.get("score", 0.0) or 0.0)
    result = dict(stage=stage, task_score=score, success=success, steps=steps, stop=stop,
                  termination=getattr(res, "termination_reason", None), setup=reset_state["setup"],
                  scorer=dict(score=gpukit.STATE["score"], level=gpukit.STATE["level"]),
                  final=gpukit.STATE["detail"], nonfinite=gpukit.STATE["nonfinite"], looks=c.log,
                  wall_s=round(time.time() - t1, 1), wall_total_s=round(time.time() - t0, 1),
                  worst_ik_mm=dict(left=round(c.left.worst_mm, 2), right=round(c.right.worst_mm, 2)),
                  physics_dt=gpukit.PHYSICS_DT, control_hz=CONTROL_HZ)
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
    print("RESULT", json.dumps({k: result[k] for k in ("stage", "task_score", "success", "steps", "stop", "setup")}),
          flush=True)
    return result, reset_state


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=3000)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--stage", default="demo")
    ap.add_argument("--cam-size", default="320x176", help="camera WxH (evidence runs use the default)")
    ap.add_argument("--device", default="cpu", help="PhysX device (cpu, or cuda:0); rendering is on the GPU either way")
    ap.add_argument("--seed", type=int, default=None, help="reset seed (seeded start state, as in inspect-robots runs)")
    ap.add_argument("--seeds", default=None, help="comma-separated reset seed per trial ('none' = unseeded named setup)")
    a = ap.parse_args()
    cw, ch = (int(v) for v in a.cam_size.split("x"))
    run(a.task, a.out, a.max_steps, not a.no_record, a.stage, cw, ch, a.device, a.seed, a.seeds)
