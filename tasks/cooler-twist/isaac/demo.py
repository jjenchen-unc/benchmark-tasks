"""Scripted full-score demo (and negative variants) for the cooler-twist task in the yam_arena embodiment.

    python demo.py task.py OUT [--max-steps N] [--no-record] [--stage demo|<variant>[,<variant>...]] [--seed N]

Acts ONLY through the 14 joint/gripper commands of ``YamArenaEmbodiment.step``. It reads the poses of the test bench,
processor, cooler and lever and the joint readings an operator can see (lever angle, how far the cooler has turned or
lifted on the processor, whether the processor is rising) through ``coolerkit.bench_state()`` (read-only); it never
writes a pose, a joint state or a score, and never reads the paste's break torque or state. Its joint commands are
rate-limited below the CLI's per-step delta guard and clipped to the URDF joint limits.

Plan:
1. Left hand: pinch the clip lever's finger tab (fingers on its two faces), swing the lever up about its pivot until it
   passes over the cam's high point and the clip goes slack (the lever then rests on its open stop); let go, back off,
   go home.
2. Right hand (meanwhile): pinch the fan shroud's rim on the side facing it (one finger on the shroud's top, one under
   its overhang). Once the clip is slack, twist the cooler about its axis, one way until it is seen to turn on the
   processor (the dried paste starts to shear), then back the other way, then to the middle.
3. Right hand: lift the cooler straight up, watching the processor; if the processor starts to rise with the cooler, set
   back down and twist again. Carry the cooler to the mat, set it down upright, let go, back off and go home.
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
PAD_TIP = 0.013  # pad tips past the TCP
TAB_PINCH_PITCH = 10.0  # deg: lever-paddle approach pitched down from the lever's length
TAB_TCP_BACK = 0.004  # TCP this far short of the paddle's centre (pads cover the 16 mm paddle)
TAB_ROLL = 1.0  # finger-closing axis along +lever z (1) or -z (-1)
TAB_SQUEEZE = 0.003  # fingers commanded past the 4 mm paddle's faces (clipped at closed: about 10 N per finger)
SHROUD_SQUEEZE = 0.006  # fingers commanded this far inside the shroud's height: about 30 N per finger
SHROUD_DEPTH = 0.0085  # pad tips this far inside the shroud's rim (the overhang is 11 mm)
SHROUD_PITCH = 10.0  # deg: shroud pinch approach pitched down
SHROUD_AZ = {"right": -135.0, "left": 135.0}  # deg (station): pinch points on the shroud's rim (front-right, front-left; clear of the open lever at 78-102 deg)
LEVER_DONE_DEG = 62.0  # seen past this: the lever lies open (the cam's high point is passed)
TWIST_SEEN_DEG = 4.0  # the cooler is seen to have turned this far on the processor
TWIST_CMD_MAX = 40.0  # deg: largest commanded hand turn (the arm sags under the paste's torque)
LIFT = 0.045
CPU_RISE_MM = 0.4  # the processor seen rising with the cooler: stop lifting
MAT_SPOT = (0.37, -0.18)  # cooler set down here (station x, y), on the mat
SET_GAP = 0.002  # cooler base this far above the mat when the fingers open


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


def axis_rot(axis, deg):
    a = np.asarray(axis, float)
    return _rotm(a / np.linalg.norm(a) * math.radians(deg))


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

    def set_pose(self, pos, rot, tol=0.004, rtol=4.0):
        q, pe, re = self.solve(pos, rot)
        if pe > tol or re > math.radians(rtol):
            raise Stop(
                f"{self.name} unreachable at {self.label}: {pe * 1000:.1f} mm {math.degrees(re):.1f} deg"
            )
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), np.asarray(rot, float)

    def move(self, pos, rot, duration, grip=None, label="", tol=0.004, rtol=4.0):
        self.label = label or self.label
        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        from yam_arena import ik

        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self.set_pose(
                p0 + s * (np.asarray(pos) - p0), ik.slerp_matrix(r0, np.asarray(rot), s), tol=tol, rtol=rtol
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
        import coolerkit

        return coolerkit.bench_state()

    def detail(self):
        import coolerkit

        return coolerkit.STATE["detail"]

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
        row = dict(at=tag, step=self.step, level=d.get("level"), clip=d.get("clip"), cpu=d.get("cpu"),
                   bond=d.get("bond"), slip=d.get("slip_deg"), lever=d.get("lever_deg"), twist=d.get("twist_deg"),
                   bz=d.get("bond_z_mm"), cpu_lift=d.get("cpu_lift_mm"), veto=d.get("veto"))
        row.update({f"{k}_now" if k in row else k: v for k, v in kw.items()})
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


# --------------------------------------------------------------------------------------------------- lever
def tab_pose(c, back=0.0, up=0.0):
    """Left-hand TCP pinching the lever's flat finger paddle (one finger above it, one below; approach along the lever
    toward its pivot, pitched down TAB_PINCH_PITCH), from the lever's measured pose; ``back`` m back along the
    approach, ``up`` m up."""
    s = c.state()
    lp, lr = s["lever"]
    tl = L.LEV_TAB[0]
    pinch = np.array([-L.LEV_LEN + tl / 2, 0.0, 0.0])
    a = math.radians(TAB_PINCH_PITCH)
    app_l = np.array([math.cos(a), 0.0, -math.sin(a)])
    tcp_l = pinch - TAB_TCP_BACK * app_l
    app = lr @ app_l
    p = lp + lr @ tcp_l - back * app + np.array([0.0, 0.0, up])
    return p, lr @ frame(app_l, [0.0, 0.0, TAB_ROLL])


def lever_open(c, max_deg=88.0, then_pre=True):
    """Left hand: pinch the tab, swing the lever up about its pivot until it is seen past LEVER_DONE_DEG, let go and
    back off."""
    arm = c.left
    arm.set_comp(True)
    p, r = tab_pose(c, back=0.04)
    yield from arm.move(p, r, 2.5, grip_of(0.012), "left to lever tab", tol=0.006)
    yield from arm.settle(0.004, 2.5, "left settle at tab")
    p, r = tab_pose(c)
    yield from arm.move(p, r, 1.2, None, "left onto tab")
    yield from arm.settle(0.002, 1.5, "left settle on tab")
    yield from arm.set_grip(grip_of(L.LEV_TAB[2] / 2 - TAB_SQUEEZE), 0.6, "left pinch tab")
    yield from wait(0.2)
    c.note("tab pinched")
    s = c.state()
    lp, lr = s["lever"]
    axis = lr[:, 1].copy()
    p0, r0 = arm.pos.copy(), arm.rot.copy()
    th = 0.0
    rate = 30.0 * DT  # deg per step
    for k in range(int(9.0 * CONTROL_HZ)):
        seen = c.state()["lever_deg"]
        if seen >= LEVER_DONE_DEG:
            break
        if th < max_deg:
            th = min(max_deg, th + rate)
        R = axis_rot(axis, th)  # positive lever angle lifts the tip (about the lever axis +y, see layout)
        arm.label = f"left swing lever {th:.0f} (seen {seen:.0f})"
        try:
            arm.set_pose(lp + R @ (p0 - lp), R @ r0, tol=0.006)
        except Stop:  # out of reach: hold the last reachable pose (the arm keeps pushing through its integral)
            th = max_deg = th - rate
        yield
    else:
        raise Stop(f"lever not open: seen {c.state()['lever_deg']:.1f} deg")
    c.note("lever open", lever=round(c.state()["lever_deg"], 1))
    yield from wait(0.3)
    arm.relax()
    yield from arm.set_grip(grip_of(0.012), 0.5, "left let go of tab")
    app = arm.rot[:, 2]
    yield from arm.move(arm.pos - 0.04 * app + np.array([0.0, 0.0, 0.03]), arm.rot, 1.2, None, "left back off lever")
    arm.set_comp(True)
    yield from arm.home(2.0)
    if then_pre:
        yield from shroud_grasp(c, "left", pre_only=True)


# --------------------------------------------------------------------------------------------------- cooler
def shroud_pose(c, back=0.0, up=0.0, az=None):
    """Right-hand TCP pinching the shroud's rim (one finger on top, one under the overhang) at station azimuth ``az``
    on the rim, approach toward the cooler's axis pitched down SHROUD_PITCH."""
    s = c.state()
    cp, cr = s["cooler"]
    az = SHROUD_AZ["right"] if az is None else az
    u = L.rz(az) @ np.array([1.0, 0.0, 0.0])
    a = math.radians(SHROUD_PITCH)
    app = -math.cos(a) * u + np.array([0.0, 0.0, -math.sin(a)])
    zc = cp[2] + (L.SHROUD_Z[0] + L.SHROUD_Z[1]) / 2
    tip = np.array([cp[0], cp[1], zc]) + (L.SHROUD_R - SHROUD_DEPTH) * u
    p = tip - PAD_TIP * app - back * app + np.array([0.0, 0.0, up])
    return p, frame(app, [0.0, 0.0, 1.0])


def shroud_grasp(c, side="right", pre_only=False, from_pre=False):
    """Pinch the shroud's rim at the arm's azimuth (pre-pose 5 cm back first, unless ``from_pre``)."""
    arm = c.arms[side]
    az = SHROUD_AZ[side]
    arm.set_comp(True)
    half_open = (L.SHROUD_Z[1] - L.SHROUD_Z[0]) / 2 + 0.010
    if not from_pre:
        p, r = shroud_pose(c, back=0.05, up=0.01, az=az)
        yield from arm.move(p, r, 2.5, grip_of(half_open), f"{side} to shroud", tol=0.006)
        yield from arm.settle(0.004, 2.5, f"{side} settle at shroud")
    if pre_only:
        return
    p, r = shroud_pose(c, az=az)
    yield from arm.move(p, r, 1.2, None, f"{side} onto shroud")
    yield from arm.settle(0.002, 2.5, f"{side} settle on shroud")
    yield from arm.set_grip(grip_of((L.SHROUD_Z[1] - L.SHROUD_Z[0]) / 2 - SHROUD_SQUEEZE), 0.8, f"{side} pinch shroud")
    yield from wait(0.3)
    arm.relax()  # command where the hand IS (offsets frozen): no stored push on the cooler once it is held
    c.note(f"shroud pinched ({side})")


def wait_clip(c, max_s=20.0):
    for _ in range(int(max_s * CONTROL_HZ)):
        if c.state()["lever_deg"] >= LEVER_DONE_DEG:
            return
        yield
    raise Stop("lever never seen open")


def twist_to(c, target_seen, cmd_lim, label, max_s=6.0):
    """Turn the pinched cooler about its axis (the hands move on arcs round the axis, together) until the cooler is SEEN
    turned to ``target_seen`` deg on the processor (sign gives the direction); the commanded turn may run ahead of the
    seen one up to ``cmd_lim`` deg (the arms sag against the paste's torque)."""
    sgn = 1.0 if target_seen >= c.state()["twist_deg"] else -1.0
    rate = 15.0 * DT
    for _ in range(int(max_s * CONTROL_HZ)):
        seen = c.state()["twist_deg"]
        if sgn * (seen - target_seen) >= 0.0:
            return True
        c.alpha = float(np.clip(c.alpha + sgn * rate, -cmd_lim, cmd_lim))
        R = axis_rot(c.twist_axis, c.alpha)
        for side, (p0, r0) in c.twist_start.items():
            arm = c.arms[side]
            arm.label = f"{label} cmd {c.alpha:.1f} seen {seen:.1f}"
            arm.set_pose(c.twist_c + R @ (p0 - c.twist_c), R @ r0, tol=0.006)
        yield
    return False


def twist_setup(c, hands=("right", "left")):
    s = c.state()
    cp, cr = s["cooler"]
    c.twist_axis = cr[:, 2].copy()
    c.twist_c = cp.copy()
    c.twist_start = {h: (c.arms[h].pos.copy(), c.arms[h].rot.copy()) for h in hands}
    for h in hands:
        c.arms[h].set_comp(True)
    c.alpha = 0.0


def twist_free(c, amp=TWIST_SEEN_DEG, hands=("right", "left")):
    """Side to side: seen +amp, then seen -amp, then back to the middle."""
    twist_setup(c, hands)
    ok = yield from twist_to(c, amp, TWIST_CMD_MAX, "twist +")
    c.note("twisted +", ok=ok, twist=round(c.state()["twist_deg"], 2), cmd=round(c.alpha, 1))
    ok2 = yield from twist_to(c, -amp, TWIST_CMD_MAX, "twist -")
    c.note("twisted -", ok=ok2, twist=round(c.state()["twist_deg"], 2), cmd=round(c.alpha, 1))
    yield from twist_to(c, 0.0, TWIST_CMD_MAX, "twist back")
    c.note("twist done", twist=round(c.state()["twist_deg"], 2))
    if not (ok and ok2):
        raise Stop("cooler never seen to turn")


def lift_cooler(c, retries=2, twist=True, hands=("right", "left")):
    """Lift straight up (both hands together); if the processor is seen rising with the cooler, set back down, the
    left hand lets go, the right hand twists again and the left hand takes hold again."""
    from yam_arena import ik

    for attempt in range(retries + 1):
        arms = [c.arms[h] for h in hands]
        start = [(a.pos.copy(), a.rot.copy()) for a in arms]
        n = int(2.0 * CONTROL_HZ)
        rose = False
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            for a, (p0, r0) in zip(arms, start):
                a.label = f"{a.name} lift cooler {attempt}"
                a.set_pose(p0 + np.array([0.0, 0.0, LIFT * s]), r0)
            yield
            if c.state()["cpu_lift_mm"] > CPU_RISE_MM:
                rose = True
                break
        if not rose:
            for _ in range(int(1.5 * CONTROL_HZ)):
                if all(np.linalg.norm(a.measured_tcp()[0] - a.pos) < 0.004 for a in arms):
                    break
                yield
            if c.state()["cpu_lift_mm"] < CPU_RISE_MM and c.state()["bond_z_mm"] > 10.0:
                c.note("cooler lifted", bz=round(c.state()["bond_z_mm"], 1))
                return
        c.note("processor rising: set back down", attempt=attempt)
        n = int(0.8 * CONTROL_HZ)
        now = [(a.pos.copy(), a.rot.copy()) for a in arms]
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            for a, (p0, r0), (p1, _) in zip(arms, start, now):
                a.set_pose(p1 + s * (p0 - p1), r0)
            yield
        if not twist or attempt == retries:
            break
        yield from twist_free(c, amp=TWIST_SEEN_DEG + 2.0 * (attempt + 1), hands=hands)
    raise Stop("cooler would not come off")


def let_go(c, side, back=0.05):
    arm = c.arms[side]
    arm.relax()
    half_open = (L.SHROUD_Z[1] - L.SHROUD_Z[0]) / 2 + 0.010
    yield from arm.set_grip(grip_of(half_open), 0.6, f"{side} let go of cooler")
    app = arm.rot[:, 2]
    yield from arm.move(arm.pos - back * app + np.array([0.0, 0.0, 0.02]), arm.rot, 1.2, None, f"{side} back off",
                        tol=0.01, rtol=25.0)
    arm.set_comp(True)


def move_together(c, delta, duration, label, hands=("right", "left")):
    from yam_arena import ik

    arms = [c.arms[h] for h in hands]
    start = [(a.pos.copy(), a.rot.copy()) for a in arms]
    n = max(1, round(duration * CONTROL_HZ))
    for k in range(1, n + 1):
        s = ik.smoothstep(k / n)
        for a, (p0, r0) in zip(arms, start):
            a.label = f"{a.name} {label}"
            a.set_pose(p0 + s * np.asarray(delta), r0, tol=0.006)
        yield


def place_on_mat(c, hands=("right", "left")):
    s = c.state()
    cp, cr = s["cooler"]
    target = np.array([MAT_SPOT[0], MAT_SPOT[1], L.TOP + L.MAT[2] + SET_GAP])
    up = np.array([0.0, 0.0, 0.03])
    d = target - cp
    yield from move_together(c, up, 0.8, "raise cooler", hands)
    yield from move_together(c, np.array([d[0], d[1], 0.0]), 2.5, "carry cooler", hands)
    s = c.state()
    cp, _ = s["cooler"]
    yield from move_together(c, np.array([0.0, 0.0, target[2] - cp[2]]), 1.8, "lower cooler", hands)
    for _ in range(int(1.0 * CONTROL_HZ)):
        yield
    s = c.state()
    cp, _ = s["cooler"]
    if cp[2] - target[2] > 0.003:  # arms sag less unloaded: lower the rest of the way once
        yield from move_together(c, np.array([0.0, 0.0, target[2] - cp[2]]), 0.8, "lower cooler more", hands)
    c.note("cooler down", base=round(c.state()["cooler"][0][2] - L.TOP - L.MAT[2], 4),
           tilt=round(c.state()["cooler_tilt"], 1))
    yield from both(*[let_go(c, h) for h in hands]) if len(hands) == 2 else let_go(c, hands[0])
    yield from both(*[c.arms[h].home(2.5) for h in hands]) if len(hands) == 2 else c.arms[hands[0]].home(2.5)


def release_hook(c, side, name, sign):
    arm = c.arms[side]
    pos, rot = c.state()[name]
    ex, ey, ez = L.HOOK_EAR
    ear = pos + rot @ np.array([sign * ex, ey, ez])
    outward = sign * rot[:, 0]
    up = rot[:, 2]
    approach = -0.87 * outward - 0.493 * up
    hand_rot = frame(approach, up if sign < 0 else -up)
    tcp = ear - 0.002 * approach
    arm.set_comp(True)
    yield from arm.move(tcp - 0.045 * approach + 0.025 * up, hand_rot, 2.0, grip_of(0.009), "approach " + name)
    yield from arm.move(tcp, hand_rot, 1.4, None, "reach " + name)
    yield from arm.settle(0.0025, 2, "settle " + name)
    yield from arm.set_grip(0, 0.7, "pinch " + name)
    c.note("hook pinched", hook=name, contact=c.detail().get("contact_n", {}).get(name),
           tcp_error_mm=np.round((arm.measured_tcp()[0] - tcp) * 1000, 2).tolist())
    yield from arm.move(tcp + 0.022 * outward, hand_rot, 1.3, None, "unhook " + name)
    yield from wait(0.3)
    travel = sign * c.state()["dofs"][name]
    c.note("clip end displacement", hook=name, mm=round(travel * 1000, 2))
    if travel < L.HOOK_RELEASE:
        raise Stop(f"{name} still on lug ({travel * 1000:.1f} mm)")
    yield from arm.set_grip(grip_of(0.014), 0.4, "release " + name)
    yield from arm.move(arm.pos + 0.04 * outward + 0.03 * up, hand_rot, 1.0, None, "back off " + name)
    yield from arm.home(1.8)


def release_and_grasp(c):
    yield from lever_open(c, then_pre=False)
    yield from wait_clip(c)
    yield from both(release_hook(c, "left", *L.HOOKS[0]), release_hook(c, "right", *L.HOOKS[1]))
    yield from both(shroud_grasp(c, "left"), shroud_grasp(c, "right"))


def demo(c):
    yield from wait(0.3)
    yield from release_and_grasp(c)
    yield from twist_free(c)
    yield from lift_cooler(c)
    yield from place_on_mat(c)
    yield from wait(4.0)


def idle(c):
    yield from wait(8.0)


def neg_straight_pull(c):
    """The clip released and the cooler lifted straight up with both hands, no twist: the dried paste holds and the
    processor comes out of the socket with the cooler: 0."""
    yield from wait(0.3)
    yield from release_and_grasp(c)
    c.left.set_comp(True)
    c.right.set_comp(True)
    yield from move_together(c, np.array([0.0, 0.0, LIFT]), 2.0, "lift straight up")
    yield from wait(1.0)
    c.note("pulled straight", cpu_lift=round(c.state()["cpu_lift_mm"], 2))
    yield from move_together(c, np.array([0.0, 0.0, -LIFT * 0.8]), 1.5, "set back down")
    yield from both(let_go(c, "right"), let_go(c, "left"))
    yield from both(c.right.home(2.5), c.left.home(2.5))
    yield from wait(3.0)


def neg_no_hooks(c):
    yield from both(lever_open(c), shroud_grasp(c))
    yield from shroud_grasp(c, "left", from_pre=True)
    yield from move_together(c, np.array([0, 0, LIFT]), 2.0, "lift with clip ends retained")
    yield from wait(1.0)


def neg_no_lever(c):
    """The clip left locked: the twist and the lift are tried with the lever down; nothing comes off: 0."""
    yield from wait(0.3)
    yield from shroud_grasp(c)
    twist_setup(c, ("right",))
    ok = yield from twist_to(c, TWIST_SEEN_DEG, TWIST_CMD_MAX, "right twist + (clip locked)", max_s=5.0)
    c.note("twist with clip locked", ok=ok, twist=round(c.state()["twist_deg"], 2), cmd=round(c.alpha, 1))
    yield from twist_to(c, 0.0, TWIST_CMD_MAX, "right twist back", max_s=4.0)
    arm = c.right
    p0, r0 = arm.pos.copy(), arm.rot.copy()
    yield from arm.move(p0 + np.array([0.0, 0.0, LIFT]), r0, 2.0, None, "right pull up (clip locked)")
    yield from wait(1.0)
    c.note("pulled with clip locked", bz=round(c.state()["bond_z_mm"], 2), cpu_lift=round(c.state()["cpu_lift_mm"], 2))
    yield from arm.move(p0, r0, 1.0, None, "right ease back")
    arm.relax()
    half_open = (L.SHROUD_Z[1] - L.SHROUD_Z[0]) / 2 + 0.010
    yield from arm.set_grip(grip_of(half_open), 0.6, "right let go")
    yield from arm.move(arm.pos - 0.05 * arm.rot[:, 2], arm.rot, 1.2, None, "right back off")
    arm.set_comp(True)
    yield from arm.home(2.5)
    yield from wait(3.0)


def neg_short_twist(c):
    """The clip released, a token twist (seen 1 deg) and a straight two-handed lift: the paste has not sheared, the
    processor comes out: 0."""
    yield from wait(0.3)
    yield from release_and_grasp(c)
    twist_setup(c)
    yield from twist_to(c, 1.0, TWIST_CMD_MAX, "token twist")
    yield from twist_to(c, 0.0, TWIST_CMD_MAX, "twist back")
    c.note("token twist", twist=round(c.state()["twist_deg"], 2))
    yield from move_together(c, np.array([0.0, 0.0, LIFT]), 2.0, "lift straight up")
    yield from wait(1.0)
    c.note("lifted after token twist", cpu_lift=round(c.state()["cpu_lift_mm"], 2))
    yield from move_together(c, np.array([0.0, 0.0, -LIFT * 0.8]), 1.5, "set back down")
    yield from both(let_go(c, "right"), let_go(c, "left"))
    yield from both(c.right.home(2.5), c.left.home(2.5))
    yield from wait(3.0)


def replay(c):
    """Open-loop check: plays the 14-D commands recorded in a finished run's trace.jsonl (path in CT_REPLAY), step by
    step, without reading anything: what a fixed trajectory scores on another start state. CT_REPLAY_MIRROR=1 plays
    it mirrored (arms swapped, joints 1, 5 and 6 negated)."""
    import os

    rows = [json.loads(line) for line in open(os.environ["CT_REPLAY"])]
    cmds = [np.asarray(r["cmd"], float) for r in rows]
    if os.environ.get("CT_REPLAY_MIRROR") == "1":
        out = []
        for a in cmds:
            lft, rgt = a[:7].copy(), a[7:].copy()
            for v in (lft, rgt):
                v[[0, 4, 5]] *= -1.0
            out.append(np.r_[rgt, lft])
        cmds = out
    c.replay = [list(a) for a in cmds]
    for _ in range(len(c.replay) + 40):
        yield


STAGES = {
    "demo": demo,
    "idle": idle,
    "replay": replay,
    "neg-straight-pull": neg_straight_pull,
    "neg-no-hooks": neg_no_hooks,
    "neg-no-lever": neg_no_lever,
    "neg-short-twist": neg_short_twist,
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
    import coolerkit

    rt = coolerkit.runtime()
    poses = rt.read()
    return dict(
        score=coolerkit.STATE["score"],
        setup=rt.setup,
        joints={k: round(v, 4) for k, v in rt.dofs().items()},
        bodies={n: np.round(p * 1000, 4).tolist() for n, (p, r) in poses.items()},
        rots={n: np.round(r, 6).tolist() for n, (p, r) in poses.items()},
    )


def reset_report(stages, resets, results):
    import layout as L

    rows = []
    for i, r in enumerate(resets):
        setup = tuple(r["setup"]) if isinstance(r["setup"], list) else r["setup"]
        spec = L.setup_spec(setup)
        gp, gr = L.bench_pose(spec)
        dev = float(np.linalg.norm(np.subtract(r["bodies"]["tbench"], gp * 1000)))
        dev = max(dev, float(np.linalg.norm(np.subtract(r["bodies"]["cooler"], L.ihs_station(spec) * 1000))))
        rdev = float(np.abs(np.subtract(r["rots"]["tbench"], gr)).max())
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
    import coolerkit
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
        d = coolerkit.STATE["detail"]
        row = dict(
            step=steps,
            score=info.get("score"),
            level=coolerkit.STATE["level"],
            detail=d,
            labels=[c.left.label, c.right.label],
            nonfinite=coolerkit.STATE["nonfinite"],
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
                f"step {steps} {c.left.label} | {c.right.label} score {info.get('score')} clip {d.get('clip')} "
                f"bond {d.get('bond')} slip {d.get('slip_deg')} lever {d.get('lever_deg')} twist {d.get('twist_deg')} "
                f"bz {d.get('bond_z_mm')} cpu {d.get('cpu_lift_mm')} mat {d.get('cooler_on_mat')} contact {d.get('contact_n')} veto {d.get('veto')} "
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
        scorer=dict(score=coolerkit.STATE["score"], level=coolerkit.STATE["level"]),
        final=coolerkit.STATE["detail"],
        nonfinite=coolerkit.STATE["nonfinite"],
        looks=c.log,
        wall_s=round(time.time() - t1, 1),
        wall_total_s=round(time.time() - t0, 1),
        worst_ik_mm=dict(
            left=round(c.left.worst_mm, 2), right=round(c.right.worst_mm, 2)
        ),
        grip_peak_n={k: round(v, 2) for k, v in c.grip_peak.items()},
        physics_dt=coolerkit.PHYSICS_DT,
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
