"""Scripted full-score demo (and negative variants) for the choke-tube task in the yam_arena embodiment.

    python demo.py task.py OUT [--max-steps N] [--no-record] [--stage demo|<variant>[,<variant>...]] [--seed N]

Acts ONLY through the 14 joint/gripper commands of ``YamArenaEmbodiment.step``. It reads the tube and toy poses
through ``toykit.bench_state()`` (read-only, like a person watching the tube); it never writes a pose, a joint state
or a score. Its joint commands are rate-limited below the CLI's per-step delta guard and clipped to the URDF bounds.

Plan:
1. The left hand takes hold of the testing tube where it stands (side grip, from above and behind) and steadies it.
2. The right hand tests the toys one at a time, nearest to the tube first: it picks the toy up, holds it centred over
   the tube's mouth and lets go. The result is read from the measured poses (what a person sees): the toy dropped
   fully below the rim (it fits: a choking hazard), or it stays on the mouth, partly above the rim (it does not fit).
3. A toy that fitted: the left hand lifts the tube, tips it out into the toy storage box and puts it back; next toy.
4. The first toy that did not fit: the right hand lifts it off the tube's mouth, sets it on the toddler's placemat
   and takes the hand away; the left hand lets go of the tube; both arms go back to READY.
The decision is closed-loop on the test result; nothing in the script knows which toys fit.
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
import scoring  # noqa: E402

CONTROL_HZ = 20
DT = 1.0 / CONTROL_HZ
FINGER_OPEN = 0.04695  # m per finger at grip 1.0
COMP_GAIN = 0.08
RATE = 0.8  # fraction of the CLI delta guard (0.05 x joint range per step) the script may use
TOY_SQUEEZE = 0.0015  # fingers commanded 1.5 mm inside the toy's faces: about 7.5 N per finger (5000 N/m drive)
TUBE_SQUEEZE = 0.0025  # about 12.5 N per finger on the tube
TCP_TO_TIP = 0.013  # pad tips 13 mm past the TCP
TUBE_GRIP_Z = 0.030  # left grip height on the tube (above its bottom)
TUBE_PITCH_DEG = 35.0  # left approach: from above and behind the tube, 35 deg below horizontal
DROP_GAP = 0.008  # toy's lowest point above the rim when let go
POUR_TILT_DEG = 125.0
POUR_RIM_Z = 0.17  # tube mouth above the table while pouring


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
        object before the fingers open."""
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
        self.G = {}
        self.corr_p, self.corr_r = np.zeros(3), np.zeros(3)

    def pose(self, name):
        import toykit

        return toykit.bench_state()[name]

    def active(self):
        import toykit

        return toykit.active_toys()

    def detail(self):
        import toykit

        return toykit.STATE["detail"]

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
        row = dict(at=tag, step=self.step, level=d.get("level"), veto=d.get("veto"), **kw)
        self.log.append(row)
        print("LOOK", json.dumps(row, default=float), flush=True)


def rot_from(approach, close):
    """Gripper rotation: approach (gripper z) and finger closing direction (gripper x)."""
    z = np.asarray(approach, float)
    z = z / np.linalg.norm(z)
    x = np.asarray(close, float)
    x = x - z * (x @ z)
    x = x / np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def down_rot(close_xy, arm):
    """Straight-down TCP, fingers closing along the given horizontal direction (or its opposite, or for square toys
    the perpendicular ones): the choice with the least joint-6 travel from the current angles."""
    best = None
    for k in range(4):
        a = math.atan2(close_xy[1], close_xy[0]) + k * math.pi / 2
        r = rot_from([0, 0, -1], [math.cos(a), math.sin(a), 0])
        q, pe, re = arm.solve(arm.pos, r)
        cost = abs(q[5] - arm.q[5]) + abs(q[5]) * 0.3 + pe * 10
        if best is None or cost < best[0]:
            best = (cost, r)
    return best[1]


def best_yaw(arm, pos, rot):
    """The rotation Rz(d) @ rot (d within +-90 deg: the held toy turns with the hand, which does not matter for the
    test or on the placemat) that the arm reaches with the least wrist-roll travel."""
    best = None
    for d in range(-90, 91, 15):
        r = L.rz(d) @ rot
        q, pe, re = arm.solve(pos, r)
        cost = (pe > 1e-3) * 10 + (re > 0.01) * 10 + abs(q[5] - arm.q[5]) + abs(d) * 0.002
        if best is None or cost < best[0]:
            best = (cost, r)
    return best[1]


def toy_grip(c, name, arm, tip_z=None):
    """Top-down grasp of a toy as it lies: TCP position, rotation and the half opening that touches its faces. For a
    block, the fingers close across its narrowest horizontal body axis (whatever face it lies on)."""
    p, r = c.pose(name)
    t = L.TOYS[name]
    lo = min(((r @ L.toy_corners(name).T).T + p)[:, 2])
    if t["kind"] == "ball":
        close = np.array([1.0, 0.0])
        half_w = t["size"][0] / 2
    elif t["kind"] == "stick":  # lying hexagonal stick: close across it, square to its length
        perp = np.cross(r[:, 0], [0.0, 0.0, 1.0])
        close = perp[:2] / np.linalg.norm(perp[:2])
        half_w = t["size"][2] / math.sqrt(3)  # half its width across the corners
    elif t["kind"] in ("cyl", "doll"):
        ax = r[:, 2]
        if abs(ax[2]) > 0.7:  # standing (or a disc lying flat): close across its diameter
            close = np.array([1.0, 0.0])
        else:  # lying on its side: close across it, square to its axis
            perp = np.cross(ax, [0.0, 0.0, 1.0])
            close = perp[:2] / np.linalg.norm(perp[:2])
        half_w = t["size"][0] / 2
    else:
        cands = [(t["size"][i], r[:, i]) for i in range(3) if abs(r[2, i]) < 0.5]
        size, ax = min(cands, key=lambda x: x[0])
        close = ax[:2] / np.linalg.norm(ax[:2])
        half_w = size / 2
    if tip_z is None:
        tip_z = max(lo + 0.002, p[2] - 0.008)
    rot = down_rot(close, arm)
    return np.array([p[0], p[1], tip_z + TCP_TO_TIP]), rot, half_w


def toy_height(name):
    t = L.TOYS[name]
    return t["size"][2] + (t["studs"][3] if t["kind"] == "brick" else 0.0)


def pick_toy(c, name, tip_z=None, label="pick", lift=0.06):
    arm = c.right
    arm.set_comp(True)
    tcp, rot, half_w = toy_grip(c, name, arm, tip_z)
    yield from arm.move(tcp + [0, 0, 0.06], rot, 2.0, grip_of(half_w + 0.012), f"{label}: over {name}")
    tcp, rot, half_w = toy_grip(c, name, arm, tip_z)
    yield from arm.move(tcp, rot, 1.2, None, f"{label}: down to {name}")
    yield from arm.set_grip(grip_of(half_w - TOY_SQUEEZE), 0.6, f"{label}: close on {name}")
    yield from wait(0.3)
    yield from arm.move(arm.pos + [0, 0, lift], arm.rot, 1.0 + 10 * (lift - 0.06), None, f"{label}: lift {name}")
    c.note("picked", toy=name)


def tube_rot(c):
    """Left grip rotation on the tube: approach from behind-left, pitched down, fingers closing horizontally."""
    tp, _ = c.pose("tube")
    base = c.left.base
    az = math.atan2(tp[1] - base[1], tp[0] - base[0])
    a = math.radians(TUBE_PITCH_DEG)
    app = np.array([math.cos(a) * math.cos(az), math.cos(a) * math.sin(az), -math.sin(a)])
    close = np.array([math.sin(az), -math.cos(az), 0.0])
    return rot_from(app, close), app


def grip_tube(c):
    arm = c.left
    arm.set_comp(True)
    tp, tr = c.pose("tube")
    rot, app = tube_rot(c)
    tcp = tp + np.array([0.0, 0.0, TUBE_GRIP_Z])
    yield from arm.move(tcp - 0.07 * app, rot, 2.5, grip_of(L.TUBE_OR + 0.012), "left: behind the tube")
    tp, tr = c.pose("tube")
    tcp = tp + np.array([0.0, 0.0, TUBE_GRIP_Z])
    yield from arm.move(tcp, rot, 1.5, None, "left: onto the tube")
    yield from arm.set_grip(grip_of(L.TUBE_OR - TUBE_SQUEEZE), 0.6, "left: grip the tube")
    yield from wait(0.2)
    c.tube_home = (tcp.copy(), rot.copy())
    c.note("tube gripped")


def drop_test(c, name):
    """Hold the toy centred over the tube's mouth, let go, look."""
    arm = c.right
    tp, tr = c.pose("tube")
    rim = L.tube_rim_center(tp, tr)
    pt, rt = c.pose(name)
    tcp_m, _ = arm.measured_tcp()
    lowest = min(((rt @ L.toy_corners(name).T).T + pt)[:, 2])
    off = tcp_m - np.array([pt[0], pt[1], lowest])  # TCP relative to the toy's lowest point, below its centre
    gap = DROP_GAP if toy_height(name) > 0.012 else 0.003  # a thin disc is let go just above the rim
    target = np.array([rim[0], rim[1], rim[2] + gap]) + off
    rot = best_yaw(arm, target + [0, 0, 0.05], arm.rot)
    yield from arm.move(target + [0, 0, 0.05], rot, 2.0, None, f"over the tube with {name}")
    # correct for the measured toy position (grasp offset, arm sag)
    for k in range(2):
        pt, rt = c.pose(name)
        err = np.array([rim[0] - pt[0], rim[1] - pt[1], 0.0])
        yield from arm.move(arm.pos + err, arm.rot, 0.5, None, f"centre {name} over the mouth")
    yield from arm.move(arm.pos + [0, 0, -0.05], arm.rot, 1.0, None, f"down to the mouth with {name}")
    yield from wait(0.3)
    half_w = L.TOYS[name]["size"][0] / 2
    yield from arm.set_grip(grip_of(min(half_w + 0.018, FINGER_OPEN)), 0.3, f"let {name} go")
    yield from wait(0.4)
    yield from arm.move(arm.pos + [0, 0, 0.05], arm.rot, 1.5, None, "hand up")
    yield from wait(0.4)
    res = scoring.tube_test(name, c.pose(name), c.pose("tube"))
    c.note("tested", toy=name, result=res, top_below_rim_mm=round(top_below_rim(c, name) * 1e3, 1))
    return res


def top_below_rim(c, name):
    """How far the toy's highest point is below the tube's rim plane (m; negative: above it)."""
    tp, tr = c.pose("tube")
    p, r = c.pose(name)
    pts = (r @ L.toy_corners(name).T).T + p
    return float(L.TUBE_H - max((tr.T @ (pts - tp).T).T[:, 2]))


def pour_plan(c, rim_target=None):
    """A reachable left-hand pose that holds the tube tilted POUR_TILT_DEG with its mouth over the storage box."""
    tp, tr = c.pose("tube")
    pm, rm = c.left.pos, c.left.rot  # the commanded grip (the held tube follows it, less the arm's sag)
    g_p = tr.T @ (pm - tp)  # grip in the tube frame
    g_r = tr.T @ rm
    best = None
    for haz in range(0, 360, 15):
        h = np.array([math.cos(math.radians(haz)), math.sin(math.radians(haz)), 0.0])
        rt = _rotm(h * math.radians(POUR_TILT_DEG)) @ tr
        rim_local = np.array([0.0, 0.0, L.TUBE_H])
        if rim_target is None:
            rim_target = np.array([L.BOX[0], L.BOX[1], L.TOP + POUR_RIM_Z])
        t_pos = rim_target - rt @ rim_local
        tcp = t_pos + rt @ g_p
        rot = rt @ g_r
        q, pe, re = c.left.solve(tcp, rot)
        cost = pe * 1e3 + math.degrees(re) + 0.2 * abs(q[5] - c.left.q[5])
        if pe < 1e-3 and re < math.radians(1.5) and (best is None or cost < best[0]):
            best = (cost, tcp, rot, haz)
    if best is None:
        raise Stop("no reachable pour pose")
    return best[1], best[2], best[3]


def pour_out(c, name, rim_target=None):
    arm = c.left
    tcp0, rot0 = c.tube_home
    yield from arm.move(tcp0 + [0, 0, 0.10], rot0, 1.5, None, "left: lift the tube")
    tcp, rot, haz = pour_plan(c, rim_target)
    up = tcp.copy()
    up[2] = max(up[2], tcp0[2] + 0.10)
    yield from arm.move(np.array([tcp[0], tcp[1], up[2] + 0.03]), rot0, 2.0, None, "left: tube over the box")
    yield from arm.move(tcp, rot, 2.0, None, "left: tip it out")
    yield from wait(0.8)
    for k in range(3):  # if it is still in, a little shake
        if scoring.tube_test(name, c.pose(name), c.pose("tube")) is None and not _in_tube_frame(c, name):
            break
        yield from arm.move(tcp + [0, 0, 0.015], rot, 0.25, None, "left: shake")
        yield from arm.move(tcp, rot, 0.25, None, "left: shake")
        yield from wait(0.4)
    c.note("poured", toy=name, haz=haz, toy_mm=np.round(c.pose(name)[0] * 1000, 1).tolist())
    yield from arm.move(np.array([tcp[0], tcp[1], up[2] + 0.03]), rot0, 2.0, None, "left: tube upright")
    yield from arm.move(tcp0 + [0, 0, 0.10], rot0, 2.0, None, "left: tube back")
    yield from arm.move(tcp0 + [0, 0, 0.003], rot0, 1.2, None, "left: tube down")
    yield from wait(0.3)


def _in_tube_frame(c, name):
    tp, tr = c.pose("tube")
    p, _ = c.pose(name)
    loc = tr.T @ (p - tp)
    return math.hypot(loc[0], loc[1]) < L.TUBE_R + 0.002 and -0.005 < loc[2] < L.TUBE_H + 0.005


def place_on_mat(c, name):
    """Carry the held toy to her placemat, set it down, let go, hand up."""
    arm = c.right
    pt, rt = c.pose(name)
    tcp_m, _ = arm.measured_tcp()
    lowest = min(((rt @ L.toy_corners(name).T).T + pt)[:, 2])
    off = tcp_m - np.array([pt[0], pt[1], lowest])
    target = np.array([L.MAT[0] - 0.02, L.MAT[1] + 0.03, L.MAT_TOP + 0.004]) + off
    rot = best_yaw(arm, target, arm.rot)
    yield from arm.move(target + [0, 0, 0.06], rot, 3.0, None, f"{name} over the placemat")
    yield from arm.move(target, rot, 1.5, None, f"{name} onto the placemat")
    half_w = max(L.TOYS[name]["size"][:2]) / 2
    yield from arm.set_grip(grip_of(min(half_w + 0.018, FINGER_OPEN)), 0.5, f"let go of {name}")
    yield from wait(0.3)
    yield from arm.move(arm.pos + [0, 0, 0.06], arm.rot, 1.2, None, "hand up")
    c.note("given", toy=name)


def give(c, name):
    """Lift the toy off the tube's mouth and set it on the placemat; hand away."""
    tp, tr = c.pose("tube")
    rim = L.tube_rim_center(tp, tr)
    pt, rt = c.pose(name)
    lowest = min(((rt @ L.toy_corners(name).T).T + pt)[:, 2])
    top = max(((rt @ L.toy_corners(name).T).T + pt)[:, 2])
    # fingertips just above the rim, at most 1 cm below the toy's middle (a big ball sitting in the mouth is gripped
    # near its equator); a peg standing out of the tube is gripped by its top 22 mm
    tip_z = max(max(lowest, rim[2]) + 0.0025, pt[2] - 0.010)
    if L.TOYS[name]["kind"] == "cyl":
        tip_z = max(tip_z, top - 0.022)
    lift = max(0.06, rim[2] - lowest + 0.012)  # its bottom clears the rim before it moves sideways
    yield from pick_toy(c, name, tip_z=tip_z, label="give", lift=lift)
    yield from place_on_mat(c, name)


REST_RIGHT = [1.2, 1.0, 1.0, -0.6, 0.0, 0.0]  # right arm's end pose: READY turned 1.2 rad toward the middle (hand
# over x 0.12, y 0.0: out of her reach, and the arm clear of the overhead camera's view of the placemat)


def park_right(c, duration=2.5):
    from yam_arena import ik

    arm = c.right
    arm.label = "right: rest"
    q0, q1 = arm.q.copy(), np.array(REST_RIGHT, float)
    n = max(1, round(duration * CONTROL_HZ))
    for k in range(1, n + 1):
        arm.q = q0 + ik.smoothstep(k / n) * (q1 - q0)
        arm.pos, arm.rot = fastik.tcp_pose(arm.q, arm.base)
        yield


def release_tube(c):
    arm = c.left
    yield from arm.set_grip(grip_of(L.TUBE_OR + 0.012), 0.5, "left: let go of the tube")
    _, app = tube_rot(c)
    yield from arm.move(arm.pos - 0.06 * app, arm.rot, 1.0, None, "left: back off")


def test_toy(c, name, picked=False):
    """Pick (unless already held) and test one toy; if it ended neither inside the tube nor on its mouth (knocked off),
    pick it up where it lies and test it again (twice at most). Returns 'inside', 'on_rim' or None."""
    res = None
    for attempt in range(3):
        if not picked or attempt > 0:
            yield from pick_toy(c, name, label="pick" if attempt == 0 else "pick again")
        res = yield from drop_test(c, name)
        if res is not None:
            break
        yield from c.right.move(c.right.pos + [0.0, -0.08, 0.04], c.right.rot, 1.0, None, "hand aside")
    return res


def test_order(c):
    tp, _ = c.pose("tube")
    names = c.active()
    return sorted(names, key=lambda n: float(np.linalg.norm(c.pose(n)[0][:2] - tp[:2])))


def demo(c):
    yield from wait(0.3)
    order = test_order(c)
    c.note("order", toys=order)
    first = order[0]
    yield from both(grip_tube(c), pick_toy(c, first))
    for i, name in enumerate(order):
        res = yield from test_toy(c, name, picked=(i == 0))
        if res == "inside":
            yield from both(pour_out(c, name), c.right.home(2.0))
            continue
        if res is None:
            c.note("untestable", toy=name)
            continue
        yield from give(c, name)
        yield from both(park_right(c), _then(release_tube(c), c.left.home(2.0)))
        yield from wait(3.0)
        return
    raise Stop("every toy fitted the tube: nothing safe to give")


def _then(*gens):
    for g in gens:
        yield from g


def idle(c):
    yield from wait(4.0)


def probe(c):
    """Development only (with CHOKE_PROBE): the toy was placed just above the tube's mouth at reset; let it settle and
    report where it ended up."""
    import toykit

    name = toykit.PROBE[toykit.runtime().trial % len(toykit.PROBE)]
    for k in range(6):
        yield from wait(0.5)
        p, r = c.pose(name)
        c.note("probe", toy=name, fits=L.TOYS[name]["fits"], result=scoring.tube_test(name, (p, r), c.pose("tube")),
               top_below_rim_mm=round(top_below_rim(c, name) * 1e3, 1),
               pos_mm=np.round(p * 1000, 2).tolist(), tube_mm=np.round(c.pose("tube")[0] * 1000, 2).tolist())


def neg_untested(c):
    """Gives a toy that fits the tube without testing it (the nearest such toy; picked by its known size, which a
    policy cannot rely on): a small part on her placemat, 0."""
    yield from wait(0.3)
    hz = [n for n in test_order(c) if L.TOYS[n]["hazard"]]
    name = hz[0]
    c.note("untested choice", toy=name)
    yield from pick_toy(c, name)
    yield from place_on_mat(c, name)
    yield from park_right(c)
    yield from wait(3.0)


def neg_ignore_test(c):
    """Runs the test like the demo, but gives the first toy tested whatever the result: on the default seed that toy
    drops fully into the tube (it fits); the left hand tips it out onto the table and the right hand gives it to her
    anyway: 0."""
    yield from wait(0.3)
    order = test_order(c)
    name = order[0]
    yield from both(grip_tube(c), pick_toy(c, name))
    res = yield from drop_test(c, name)
    c.note("ignored result", toy=name, result=res)
    if res == "inside":
        yield from both(pour_out(c, name, np.array([0.36, 0.0, L.TOP + 0.075])), c.right.home(2.0))
        yield from wait(0.5)
        yield from pick_toy(c, name, label="give anyway")
        yield from place_on_mat(c, name)
    else:
        yield from give(c, name)
    yield from both(c.right.home(2.0), _then(release_tube(c), c.left.home(2.0)))
    yield from wait(3.0)


def neg_hand_stays(c):
    """The demo, but the right hand stays over the placemat after letting go: 0.5."""
    yield from wait(0.3)
    order = test_order(c)
    yield from both(grip_tube(c), pick_toy(c, order[0]))
    for i, name in enumerate(order):
        res = yield from test_toy(c, name, picked=(i == 0))
        if res == "inside":
            yield from both(pour_out(c, name), c.right.home(2.0))
            continue
        if res is None:
            continue
        yield from give(c, name)
        yield from _then(release_tube(c), c.left.home(2.0))
        yield from wait(4.0)
        return


def replay(c):
    """Open-loop check: plays the 14-D commands recorded in a finished run's trace.jsonl (path in CHOKE_REPLAY), step
    by step, without reading anything: what a fixed trajectory scores on another start state."""
    import os

    rows = [json.loads(line) for line in open(os.environ["CHOKE_REPLAY"])]
    c.replay = [r["cmd"] for r in rows]
    for _ in range(len(c.replay) + 40):
        yield


STAGES = {
    "demo": demo,
    "idle": idle,
    "probe": probe,
    "replay": replay,
    "neg-untested": neg_untested,
    "neg-ignore-test": neg_ignore_test,
    "neg-hand-stays": neg_hand_stays,
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
    results, resets = [], []
    for i, st in enumerate(stages):
        tdir = out if len(stages) == 1 else out / f"trial{i}_{st}"
        res, reset_state = run_trial(emb, tdir, max_steps, record, st, t0, seed=seed)
        results.append(res)
        resets.append(reset_state)
    if len(stages) > 1:
        rep = reset_report(stages, resets, results)
        (out / "reset_check.json").write_text(json.dumps(rep, indent=2, default=float))
        print("RESETCHECK", json.dumps(rep["summary"]), flush=True)
    return results


def reset_state_now():
    import toykit

    rt = toykit.runtime()
    poses = rt.read()
    return dict(
        score=toykit.STATE["score"],
        setup=rt.setup,
        bodies={n: np.round(p * 1000, 4).tolist() for n, (p, r) in poses.items()},
        rots={n: np.round(r, 6).tolist() for n, (p, r) in poses.items()},
    )


def reset_report(stages, resets, results):
    import toykit

    rows = []
    for i, r in enumerate(resets):
        setup = tuple(r["setup"]) if isinstance(r["setup"], list) else r["setup"]
        want = toykit.authored(setup, i)
        names = ["tube"] + L.active_toys(setup)
        dev = max(float(np.linalg.norm(np.subtract(r["bodies"][n], want[n][0] * 1000))) for n in names)
        rdev = max(float(np.abs(np.subtract(r["rots"][n], want[n][1])).max()) for n in names)
        rows.append(
            dict(
                trial=i,
                stage=stages[i],
                setup=r["setup"],
                score_after_reset=r["score"],
                max_body_dev_from_authored_mm=round(dev, 4),
                max_rot_dev=round(rdev, 6),
                trial_score=results[i]["task_score"],
                trial_stop=results[i]["stop"],
            )
        )
    ok = all(x["score_after_reset"] == 0.0 and x["max_body_dev_from_authored_mm"] < 1.0 for x in rows)
    return dict(
        summary=dict(passed=ok, trials=len(rows), max_body_dev_mm=max(x["max_body_dev_from_authored_mm"] for x in rows)),
        rows=rows,
        reset_states=resets,
    )


def run_trial(emb, out, max_steps, record, stage, t0, seed=None):
    import toykit
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
        d = toykit.STATE["detail"]
        row = dict(
            step=steps,
            score=info.get("score"),
            level=toykit.STATE["level"],
            detail=d,
            labels=[c.left.label, c.right.label],
            nonfinite=toykit.STATE["nonfinite"],
            cmd=np.round(np.r_[c.left.last_cmd, c.right.last_cmd], 4).tolist(),
            q=np.round(jp, 4).tolist(),
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
            toys = {k: (v.get("where"), v.get("test")) for k, v in (d.get("toys") or {}).items()}
            print(
                f"step {steps} {c.left.label} | {c.right.label} score {info.get('score')} toys {toys} "
                f"tube_tilt {d.get('tube_tilt_deg')} veto {d.get('veto')} hands_out {d.get('hands_out')} "
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
        scorer=dict(score=toykit.STATE["score"], level=toykit.STATE["level"]),
        final=toykit.STATE["detail"],
        nonfinite=toykit.STATE["nonfinite"],
        looks=c.log,
        wall_s=round(time.time() - t1, 1),
        wall_total_s=round(time.time() - t0, 1),
        worst_ik_mm=dict(
            left=round(c.left.worst_mm, 2), right=round(c.right.worst_mm, 2)
        ),
        physics_dt=toykit.PHYSICS_DT,
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
        type=int,
        default=None,
        help="reset seed (seeded start state, as in inspect-robots runs)",
    )
    a = ap.parse_args()
    cw, ch = (int(v) for v in a.cam_size.split("x"))
    run(a.task, a.out, a.max_steps, not a.no_record, a.stage, cw, ch, a.device, a.seed)
