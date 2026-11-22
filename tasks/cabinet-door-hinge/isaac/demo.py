"""Scripted full-score demo (and negative variants) for the cabinet-door-hinge task in the yam_arena embodiment.

    python demo.py task.py OUT [--max-steps N] [--no-record] [--stage demo|<variant>[,<variant>...]] [--seed S]

Acts ONLY through the 14 joint/gripper commands of ``YamArenaEmbodiment.step``. It reads the door's and the hinge
arms' poses, the knuckle angles and the hinges' plate states through ``hingekit.hinge_state()`` (read-only, like a
carpenter watching each arm's lug over its plate and hearing the click); it never writes a pose, a joint state or a
score. Its joint commands are rate-limited below the CLI's per-step delta guard and clipped to the URDF joint bounds.

Plan:
1. Both hands take the door's free edge (fingers across the 19 mm thickness): left high, right low.
2. Lift it out of the groove, carry it to the opening square to the cabinet front, turned 5 deg so both arms' rears
   stand off their plates (the lugs must go on first), lugs 5 mm above the plates' noses; slide the arms in beside
   the side panel; lower 6.5 mm so both lugs drop onto the noses (hooked). Closed loop on the measured door pose.
3. Right lets go; left holds the door. Right presses the lower arm's rear onto its plate (about 30 N, past the
   latch) while left turns the door square; then the upper arm's rear. Each click is read from the hinge state.
4. Right backs off; left lets go of the edge and pushes the door's outer face round to about 25 deg; the hinges'
   self-closing springs shut it. Both arms go home.
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
COMP_CLIP = 0.45
RATE = 0.8  # fraction of the CLI delta guard (0.05 x joint range per step) the script may use
JOINT_K = np.array([80.0, 80.0, 80.0, 10.0, 10.0, 10.0])  # arm joint PD stiffness (yam_arena constants)
# door grasp (both hands at the free edge, fingers across the thickness)
EDGE_HALF = L.DOOR_T / 2
EDGE_SQUEEZE = 0.004  # fingers 4 mm inside the faces: about 20 N per finger
EDGE_OPEN_HALF = EDGE_HALF + 0.012
EDGE_INSERT = 0.012  # TCP this far inside the free edge (pad tips 25 mm in)
EDGE_APPROACH = 0.035  # TCP this far outside the edge before moving in
GRASP_C = {"left": 0.120, "right": 0.340}  # grasp heights above the door's bottom edge
GRASP_TILT = {"left": 30.0, "right": 0.0}  # approach tilted down in the door plane (deg)
# hanging
PSI_DEG = 5.0  # door turned about the hook line so the rears stand ~3.7 mm off their plates
HANG_HI = 0.005  # lugs above their seats before dropping on
HANG_LO = -0.0015
SLIDE_X = 0.045  # arms start beside the panel this far in front of their plates
# pressing (right fingertips on each arm's rear)
PRESS_DIR = np.array([1.0, 1.0, 0.4]) / np.linalg.norm([1.0, 1.0, 0.4])  # fingertip approach (push force is along +y)
PRESS_FY = 40.0  # N toward the panel (latch needs 22 N; the joint PD and the hook spring take part of it)
PRESS_OFF = 0.030
# closing (left fingertips on the door's outer face)
CLOSE_R = 0.200
CLOSE_C = 0.200
CLOSE_END_DEG = 24.0


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
        self.ff = np.zeros(6)
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
        c = np.r_[np.clip(self.q + self.offset + self.ff, self.lo, self.hi), self.grip]
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
            self.offset = np.clip(self.offset + COMP_GAIN * (self.q - self.measured), -COMP_CLIP, COMP_CLIP)
        else:
            self.offset *= 0.85

    def feedforward(self, force_world, at_world=None):
        """Joint offsets that make the PD push with ``force_world`` (N) at the TCP (or ``at_world``): tau = J^T F,
        offset = tau / K."""
        p, _, axes = fastik.fk(self.q, self.base)
        at = p if at_world is None else np.asarray(at_world, float)
        tau = np.array([np.cross(a, at - o) @ np.asarray(force_world, float) for a, o in axes])
        return tau / JOINT_K

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
        if pe > 0.030 or re > math.radians(15):
            raise Stop(f"{self.name} unreachable at {self.label}: {pe * 1000:.1f} mm {math.degrees(re):.1f} deg")
        if pe > 0.004 or re > math.radians(4):  # best effort (closed-loop corrections near the reach limit)
            self.unreach = getattr(self, "unreach", 0) + 1
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

    def home(self, duration=2.5, label=""):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.label = label or f"{self.name} home"
        q0, q1 = self.q.copy(), np.array(READY, float)
        f0 = self.ff.copy()
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.q = q0 + s * (q1 - q0)
            self.ff = f0 * (1 - s)
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
        self.tries = 0
        self.dp_corr = np.zeros(3)
        self.dr_corr = np.zeros(3)
        self.door_target = None
        self.psi = PSI_DEG
        self.hang_off = np.zeros(3)
        self.press_roll = 0.0
        self.ff_scale = 0.0

    def state(self):
        import hingekit

        return hingekit.hinge_state()

    def detail(self):
        import hingekit

        return hingekit.STATE["detail"]

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


# --------------------------------------------------------------------------------------------- geometry helpers
def door_tcp(name, sign):
    """TCP pose in the door frame for a free-edge grasp: approach into the door (-a) tilted down in the door plane,
    fingers across the thickness (+-b)."""
    t = math.radians(GRASP_TILT[name])
    z = np.array([-math.cos(t), 0.0, -math.sin(t)])
    x = np.array([0.0, sign, 0.0])
    r = np.column_stack([x, np.cross(z, x), z])
    p = np.array([L.DOOR_W - EDGE_INSERT, L.DOOR_T / 2, GRASP_C[name]])
    return p, r


def prehang(spec, dz, psi_deg, dx=0.0):
    """Door pose at the opening: hung pose turned psi about the vertical line through the hook seats (rears off their
    plates), lugs dz above their seats, moved dx along x."""
    dp, dr = L.door_pose_hung(spec)
    hook = L.plate_origin(spec, 0) + L.HOOK_A
    r = L.rz(-psi_deg)
    p = hook + r @ (dp - hook)
    p = p + np.array([dx, 0.0, dz])
    return p, r @ dr


def compose(fp, fr, gp, gr):
    return fp + fr @ gp, fr @ gr


def door_cmd(c, tp, tr):
    """Closed loop on the measured door pose: target plus an integral correction (the hands sag and slip)."""
    st = c.state()
    mp, mr = st["door"]
    c.dp_corr = np.clip(c.dp_corr + 0.06 * (tp - mp), -0.03, 0.03)
    e = _rotvec(tr @ mr.T)
    c.dr_corr = np.clip(c.dr_corr + 0.05 * e, -0.10, 0.10)
    return tp + c.dp_corr, _rotm(c.dr_corr) @ tr


def load_ff(c, n, scale):
    """Joint feed-forward for hand n's share of the door's weight: equal vertical shares, plus the horizontal couple
    (along the door's a axis) that carries the door's moment about the grip line (hands one above the other)."""
    st = c.state()
    dp, dr = st["door"]
    w = (L.M_DOOR + 2 * L.M_ARM) * 9.81
    ca = L.DOOR_W / 2
    ga = {k: c.G[k][1][0] for k in ("left", "right")}
    gc = {k: c.G[k][1][2] for k in ("left", "right")}
    d = np.mean(list(ga.values())) - ca  # grip line this far outboard of the COM (along a)
    dc = gc["right"] - gc["left"]
    couple = w * d / dc  # upper hand pulls +a, lower pushes -a
    sgn = 1.0 if gc[n] > np.mean(list(gc.values())) else -1.0
    f = np.array([0.0, 0.0, w / 2]) + sgn * couple * dr[:, 0]
    return c.arms[n].feedforward(scale * f)


def hold_door(c, tp, tr, label, hands=("left", "right")):
    """One control step: every holding hand at its captured grasp on the commanded door pose (with the load
    feed-forward when both hands carry)."""
    cp, cr = door_cmd(c, tp, tr)
    for n in hands:
        _, gp, gr = c.G[n]
        c.arms[n].label = label
        c.arms[n].set_pose(*compose(cp, cr, gp, gr))
        if c.ff_scale > 0 and len(hands) == 2:
            c.arms[n].ff = load_ff(c, n, c.ff_scale)


def carry(c, target, duration, label, hands=("left", "right")):
    """Move the door from the current commanded target to ``target`` (smooth), closed loop."""
    from yam_arena import ik

    p0, r0 = c.door_target
    p1, r1 = target
    n = max(1, round(duration * CONTROL_HZ))
    for k in range(1, n + 1):
        s = ik.smoothstep(k / n)
        tp, tr = p0 + s * (p1 - p0), ik.slerp_matrix(r0, r1, s)
        c.door_target = (tp, tr)
        hold_door(c, tp, tr, label, hands)
        yield


def settle(c, seconds, label, hands=("left", "right"), until=None):
    for _ in range(round(seconds * CONTROL_HZ)):
        hold_door(c, *c.door_target, label, hands)
        yield
        if until is not None and until(c.state()):
            return


def capture(c, name):
    """Freeze the door -> TCP transform of a hand (measured)."""
    st = c.state()
    dp, dr = st["door"]
    pt, rt = c.arms[name].measured_tcp()
    c.G[name] = ("door", dr.T @ (pt - dp), dr.T @ rt)


def choose_sign(arm, frame, tcp_fn):
    best = None
    for s in (1.0, -1.0):
        gp, gr = tcp_fn(s)
        q, pe, re = arm.solve(*compose(*frame, gp, gr))
        cost = abs(q[5] - arm.q[5]) + pe * 50 + re
        if best is None or cost < best[0]:
            best = (cost, s)
    return best[1]


# ------------------------------------------------------------------------------------------------ the script
def grasp_door(c):
    signs = {}

    def targets():
        st = c.state()
        frame = st["door"]
        out = {}
        for n in ("left", "right"):
            if n not in signs:
                signs[n] = choose_sign(c.arms[n], frame, lambda s, n=n: door_tcp(n, s))
            gp, gr = door_tcp(n, signs[n])
            p_in, rot = compose(*frame, gp, gr)
            out[n] = (p_in, rot, p_in - rot[:, 2] * (EDGE_APPROACH + EDGE_INSERT))
        return out

    t0 = targets()
    yield from both(*[c.arms[n].move(t0[n][2] - t0[n][1][:, 2] * 0.01, t0[n][1], 3.0, grip_of(EDGE_OPEN_HALF),
                                     f"{n} to door edge") for n in ("left", "right")])
    t1 = targets()  # re-read the door (closed loop) before moving in
    yield from both(*[c.arms[n].move(t1[n][2], t1[n][1], 0.8, None, f"{n} at door edge") for n in ("left", "right")])
    t2 = targets()
    yield from both(*[c.arms[n].move(t2[n][0], t2[n][1], 1.6, None, f"{n} onto door edge") for n in ("left", "right")])
    yield from both(c.left.set_grip(grip_of(EDGE_HALF - EDGE_SQUEEZE), 0.8, "left grips door"),
                    c.right.set_grip(grip_of(EDGE_HALF - EDGE_SQUEEZE), 0.8, "right grips door"))
    yield from wait(0.5)
    capture(c, "left")
    capture(c, "right")
    st = c.state()
    c.door_target = (st["door"][0].copy(), st["door"][1].copy())
    c.left.set_comp("hold")  # two hands on one rigid door: no per-arm joint integral (they would fight)
    c.right.set_comp("hold")
    for k in range(1, 21):  # take the door's weight (feed-forward ramp) before lifting
        c.ff_scale = k / 20
        hold_door(c, *c.door_target, "take the door's weight")
        yield
    c.note("grasped")


def lugs(c):
    """Per hinge: lug offset from its seat (mm, world xyz) and rear gap (mm)."""
    st = c.state()
    return [(np.round(g[2] * 1e3, 2).tolist(), round(g[3] * 1e3, 2)) for g in st["geom"]]


def hang(c, tries=3):
    spec = c.state()["spec"]
    dp, dr = c.door_target
    yield from carry(c, (dp + [0, 0, 0.022], dr), 1.5, "lift out of groove")
    p, r = prehang(spec, HANG_HI, PSI_DEG, -SLIDE_X)
    yield from carry(c, (np.array([p[0], dp[1], p[2]]), r), 2.0, "carry clear of the carcass")
    yield from carry(c, (p, r), 2.0, "carry to opening")
    off = np.zeros(3)
    for attempt in range(tries):
        c.tries = attempt + 1
        p, r = prehang(spec, HANG_HI, PSI_DEG)
        yield from carry(c, (p + off, r), 1.6, "slide arms in")
        yield from settle(c, 1.0, "line up lugs")
        c.note("lined up", lugs=lugs(c))
        # correct by the measured lug offset (mean of the two, x and y only: z comes from the drop)
        d = np.mean([np.asarray(x[0]) for x in lugs(c)], axis=0) / 1e3
        off = off - np.array([d[0], d[1], 0.0])
        p, r = prehang(spec, HANG_HI, PSI_DEG)
        yield from carry(c, (p + off, r), 0.8, "line up lugs (corrected)")
        yield from settle(c, 0.6, "line up lugs (corrected)")
        p, r = prehang(spec, HANG_LO, PSI_DEG)
        yield from carry(c, (p + off, r), 1.5, "drop lugs onto noses")
        yield from settle(c, 0.8, "drop lugs onto noses")
        st = c.state()
        c.note("dropped", hinges=st["hinges"], lugs=lugs(c))
        if all(h == "hooked" for h in st["hinges"]):
            return True
        p, r = prehang(spec, HANG_HI, PSI_DEG)
        yield from carry(c, (p + off, r), 1.0, "lift off (retry)")
    return False


def let_go_right(c):
    arm = c.right
    f0 = {n: c.arms[n].ff.copy() for n in ("left", "right")}
    c.ff_scale = 0.0
    for k in range(1, 21):  # the hooks take the door: both hands unload
        for n in ("left", "right"):
            c.arms[n].ff = f0[n] * (1 - k / 20)
        hold_door(c, *c.door_target, "unload onto hooks", hands=("left", "right"))
        yield
    yield from arm.set_grip(grip_of(EDGE_OPEN_HALF), 0.6, "right lets go")
    p, r = arm.pos, arm.rot
    hold = [True]

    def left():
        while hold[0]:
            hold_door(c, *c.door_target, "left holds door", hands=("left",))
            yield

    def right():
        arm.set_comp(True)
        yield from arm.move(p - r[:, 2] * 0.07, r, 1.2, None, "right off door")
        hold[0] = False

    yield from both(right(), left())


def press_pose(c, i, back, roll=None):
    """Right TCP for pressing arm i's rear: fingertips on the arm's top face over its rear latch, approach along
    PRESS_DIR; fingers closed, finger axis near vertical (turned by ``roll`` deg about the approach)."""
    st = c.state()
    ap, ar = st[f"arm{i}"]
    p_r = ap + ar @ (L.REAR_A + np.array([-0.004, L.ARM_Y[0], 0.0]))
    z = PRESS_DIR
    x = np.array([0.0, 0.0, 1.0])
    x = x - (x @ z) * z
    x /= np.linalg.norm(x)
    roll = c.press_roll if roll is None else roll
    x = math.cos(math.radians(roll)) * x + math.sin(math.radians(roll)) * np.cross(z, x)
    r = np.column_stack([x, np.cross(z, x), z])
    from yam_arena.constants import PAD_TIP_PAST_TCP

    return p_r - z * (PAD_TIP_PAST_TCP + back), r


VIA_Y = -0.080  # right hand's lane in front of the cabinet opening, inside the door's plane


def press_vias(c, p_pre, r):
    """Waypoints for the right hand from beside the door's free edge to in front of the arm: sideways (-y) clear of
    the door, then forward to 12 cm in front of the press pose, turning to the press orientation."""
    p0, r0 = c.right.pos.copy(), c.right.rot.copy()
    v1 = np.array([p0[0], VIA_Y, p0[2]])
    v2 = np.array([p_pre[0] - 0.07, VIA_Y, p_pre[2] + 0.04])
    return [(v1, r0, 1.2), (v2, r, 2.0)]


def press(c, i, square_door=True):
    """Right presses arm i's rear past the latch; left holds the door and (first press) turns it square."""
    arm = c.right
    spec = c.state()["spec"]
    p_pre, r = press_pose(c, i, PRESS_OFF)
    done = [False]
    psi0 = c.psi

    def left():
        while not done[0]:
            p, rr = prehang(spec, HANG_LO, c.psi)
            c.door_target = (p + c.hang_off, rr)
            hold_door(c, *c.door_target, "left holds door", hands=("left",))
            yield

    def right():
        if i == 0:  # from the door's free edge: sideways into the cabinet's open front (inside the door), then in
            for p_via, r_via, dur in press_vias(c, p_pre, r):
                yield from arm.move(p_via, r_via, dur, 0.0, f"right to arm {i}")
        yield from arm.move(p_pre + [0, 0, 0.04], r, 2.0, 0.0, f"right to arm {i}")
        yield from arm.move(p_pre, r, 0.8, None, f"right at arm {i} rear")
        p_c, _ = press_pose(c, i, -0.012)
        yield from arm.move(p_c, r, 1.2, None, f"right on arm {i} rear")
        f0 = arm.ff.copy()
        f1 = arm.feedforward(np.array([0.0, PRESS_FY, 0.0]))
        n = round(2.0 * CONTROL_HZ)
        clicked = False
        for k in range(1, n + 1):
            arm.ff = f0 + min(1.0, k / (0.7 * n)) * (f1 - f0)
            if square_door:
                c.psi = psi0 * max(0.0, 1.0 - k / (0.7 * n))
            arm.label = f"right presses arm {i}"
            yield
            if c.state()["hinges"][i] == "clipped":
                clicked = True
                break
        c.note("press", hinge=i, clicked=clicked, hinges=c.state()["hinges"], lugs=lugs(c))
        for k in range(1, 11):
            arm.ff = f1 + (k / 10) * (f0 - f1)
            yield
        yield from arm.move(p_pre - PRESS_DIR * 0.03, r, 1.0, None, f"right off arm {i}")
        done[0] = True

    yield from both(right(), left())


def let_go_left(c):
    arm = c.left
    yield from arm.set_grip(grip_of(EDGE_OPEN_HALF), 0.6, "left lets go")
    arm.set_comp(True)
    p, r = arm.pos, arm.rot
    yield from arm.move(p - r[:, 2] * 0.08, r, 1.2, None, "left off door")
    yield from arm.home(2.5)


def right_out(c):
    """Right hand backs out of the cabinet along its lane, then goes home."""
    arm = c.right
    p, r = arm.pos.copy(), arm.rot.copy()
    xf, _ = L.cabinet_origin(c.state()["spec"])
    yield from arm.move(np.array([p[0] - 0.02, VIA_Y, p[2]]), r, 1.0, None, "right out (lane)")
    yield from arm.move(np.array([xf - 0.10, VIA_Y, p[2]]), r, 1.5, None, "right out of the cabinet")
    yield from arm.home(2.5)


def close_door(c):
    """Left lets go of the edge, then pushes the door's outer face round to CLOSE_END_DEG; the springs shut it."""
    arm = c.left
    st = c.state()
    yield from arm.set_grip(grip_of(EDGE_OPEN_HALF), 0.6, "left lets go")
    arm.set_comp(True)
    p, r = arm.pos, arm.rot
    yield from arm.move(p - r[:, 2] * 0.08, r, 1.2, None, "left off door")
    kp = L.knuckle_hung(st["spec"])
    from yam_arena.constants import PAD_TIP_PAST_TCP

    def pose(alpha, gap):
        al = math.radians(alpha)
        a_dir = np.array([-math.sin(al), -math.cos(al), 0.0])
        n_out = np.array([-math.cos(al), math.sin(al), 0.0])  # outer face normal
        at = kp + a_dir * CLOSE_R + np.array([0.0, 0.0, CLOSE_C])
        z = -n_out
        x = np.array([0.0, 0.0, 1.0])
        rr = np.column_stack([x, np.cross(z, x), z])
        return at + n_out * (PAD_TIP_PAST_TCP + gap), rr

    al0 = min(c.state()["alpha"])
    yield from arm.move(*pose(al0, 0.04), 2.5, 0.0, "left to door face")
    yield from arm.move(*pose(al0, 0.002), 1.0, None, "left on door face")
    al_cmd = al0
    for _ in range(round(8.0 * CONTROL_HZ)):
        al_m = min(c.state()["alpha"])
        al_cmd = max(CLOSE_END_DEG, min(al_cmd - 1.2, al_m + 2.0))
        arm.label = "left pushes door shut"
        arm.set_pose(*pose(al_cmd, -0.004))
        yield
        if al_m <= CLOSE_END_DEG + 1.0:
            break
    c.note("pushed", alpha=c.state()["alpha"])
    yield from arm.move(*pose(al_cmd + 8.0, 0.05), 1.2, None, "left off door face")


def demo(c, stop_after=None):
    yield from wait(0.5)
    yield from grasp_door(c)
    ok = yield from hang(c)
    c.hang_off = c.door_target[0] - prehang(c.state()["spec"], HANG_LO, PSI_DEG)[0]
    c.psi = PSI_DEG
    if not ok:
        c.note("not hooked")
    if stop_after == "hook":
        return
    yield from let_go_right(c)
    yield from press(c, 0, square_door=True)
    if stop_after == "one":  # leave the upper arm only hooked; let go of the door, do not close it
        yield from right_out(c)
        yield from let_go_left(c)
        yield from wait(3.0)
        return
    yield from press(c, 1, square_door=False)
    yield from right_out(c)
    yield from close_door(c)
    yield from both(c.left.home(2.5), wait(0.1))
    yield from wait(3.0)


def look(c):
    """Media only: the right hand parks its wrist camera in the cabinet's open front, looking at the two mounting
    plates on the left side panel's inside (and the door with its hinges beside the cabinet); ends there."""
    yield from wait(0.5)
    st = c.state()
    xf, yl = L.cabinet_origin(st["spec"])
    zc = 0.5 * (st["seats"][0][2] + st["seats"][1][2])
    target = np.array([xf + L.PLATE_LINE, yl - L.PANEL_T, zc])
    eye = np.array([xf - 0.10, yl - 0.20, zc + 0.06])
    z = target - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, [0.0, 0.0, 1.0])
    x /= np.linalg.norm(x)
    r = np.column_stack([x, np.cross(z, x), z])
    yield from c.right.move(eye + [0, 0, 0.05], r, 2.5, 0.0, "right to look")
    yield from c.right.move(eye, r, 1.0, None, "right looks at the plates")
    yield from wait(1.5)


def neg_one_clip(c):
    """Negative: hooks both, clips only the lower hinge, lets go of the door (it hangs on the clipped hinge): 0.25."""
    yield from demo(c, stop_after="one")


def neg_high(c):
    """Negative: the lugs are never dropped onto the noses (door held 5 mm high, square); the right presses both
    rears anyway: without the hook nothing clicks (0)."""
    yield from wait(0.5)
    yield from grasp_door(c)
    spec = c.state()["spec"]
    dp, dr = c.door_target
    yield from carry(c, (dp + [0, 0, 0.022], dr), 1.5, "lift out of groove")
    p, r = prehang(spec, HANG_HI, PSI_DEG, -SLIDE_X)
    yield from carry(c, (np.array([p[0], dp[1], p[2]]), r), 2.0, "carry clear of the carcass")
    yield from carry(c, (p, r), 2.0, "carry to opening")
    yield from carry(c, prehang(spec, HANG_HI, PSI_DEG), 1.6, "slide arms in (lugs high)")
    yield from settle(c, 1.0, "hold high")
    c.hang_off = c.door_target[0] - prehang(spec, HANG_LO, PSI_DEG)[0]
    c.psi = PSI_DEG
    c.note("held high", hinges=c.state()["hinges"], lugs=lugs(c))
    yield from let_go_right(c)
    yield from press(c, 0, square_door=True)
    yield from press(c, 1, square_door=False)
    yield from right_out(c)
    yield from wait(1.0)


def neg_twist(c):
    """Negative: both lugs hooked, nobody presses the rears; the left turns the door square (and 3 deg past) to force
    the arms on through the hinges: the hinge pivots slip, nothing clicks (0)."""
    yield from wait(0.5)
    yield from grasp_door(c)
    ok = yield from hang(c)
    spec = c.state()["spec"]
    c.note("hooked" if ok else "not hooked", hinges=c.state()["hinges"])
    off = c.door_target[0] - prehang(spec, HANG_LO, PSI_DEG)[0]
    for psi in (0.0, -3.0):
        p, r = prehang(spec, HANG_LO, psi)
        yield from carry(c, (p + off, r), 2.5, f"twist door to {psi} deg")
        yield from settle(c, 1.0, "twist hold")
    c.note("twisted", hinges=c.state()["hinges"], lugs=lugs(c))
    yield from wait(1.0)


def neg_drop(c):
    """Negative: right lets go of the door before it is hooked; left alone cannot hold its moment (0)."""
    yield from wait(0.5)
    yield from grasp_door(c)
    dp, dr = c.door_target
    yield from carry(c, (dp + [0, 0, 0.022], dr), 1.5, "lift out of groove")
    yield from let_go_right(c)
    yield from settle(c, 3.0, "left alone", hands=("left",))
    c.note("left alone", door=np.round(c.state()["door"][0] * 1e3, 1).tolist())
    yield from c.right.home(2.0)
    yield from wait(2.0)


def idle(c):
    for _ in range(100000):
        yield


def hold_still(c):
    """Development probes: both arms at READY for 13 s (+2 s after the end)."""
    for _ in range(260):
        yield


def replay(c):
    """Open-loop check: plays the 14-D commands recorded in a finished run's trace.jsonl (path in CDH_REPLAY), step
    by step, without reading anything: what a fixed trajectory scores on another start state."""
    import os

    rows = [json.loads(line) for line in open(os.environ["CDH_REPLAY"])]
    c.replay = [r["cmd"] for r in rows]
    for _ in range(len(c.replay) + 40):
        yield


STAGES = {
    "demo": demo,
    "idle": idle,
    "hold-still": hold_still,
    "replay": replay,
    "neg-one-clip": neg_one_clip,
    "look": look,
    "neg-drop": neg_drop,
    "neg-high": neg_high,
    "neg-twist": neg_twist,
}


def _write_mp4(frames, path):
    import imageio

    w = imageio.get_writer(str(path), fps=CONTROL_HZ, codec="libx264", quality=8, macro_block_size=8,
                           ffmpeg_params=["-pix_fmt", "yuv420p"])
    for f in frames:
        w.append_data(np.asarray(f))
    w.close()


def run(task_path, out_dir, max_steps=2400, record=True, stage="demo", cam_width=320, cam_height=176, device="cpu",
        seed=None):
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    t0 = time.time()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=str(Path(task_path).resolve()), object_jitter=0, control_hz=CONTROL_HZ,
                               cam_width=cam_width, cam_height=cam_height, device=device)
    stages = stage.split(",")
    results, resets = [], []
    seeds = [None] if seed is None else [int(x) for x in str(seed).split(",")]
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
    import hingekit

    rt = hingekit.runtime()
    poses = rt.read()
    al = [L.door_alpha(q) for q in rt.dofs()]
    return dict(score=hingekit.STATE["score"], setup=rt.setup, alpha=al, hinges=[h.state for h in rt.hinges],
                bodies={n: np.round(p * 1000, 4).tolist() for n, (p, r) in poses.items()},
                rots={n: np.round(r, 6).tolist() for n, (p, r) in poses.items()})


def reset_report(stages, resets, results):
    import hingekit

    rows = []
    for i, r in enumerate(resets):
        want = hingekit.authored(tuple(r["setup"]) if isinstance(r["setup"], list) else r["setup"])
        dev = max(float(np.linalg.norm(np.subtract(r["bodies"][n], want[n][0] * 1000))) for n in ("door", "arm0", "arm1"))
        rows.append(dict(trial=i, stage=stages[i], setup=r["setup"], score_after_reset=r["score"],
                         max_body_dev_from_authored_mm=round(dev, 4), alpha_deg=r["alpha"], hinges=r["hinges"],
                         trial_score=results[i]["task_score"],
                         trial_stop=results[i]["stop"]))
    ok = all(x["score_after_reset"] == 0.0 and x["max_body_dev_from_authored_mm"] < 1.0
             and x["hinges"] == ["free", "free"] for x in rows)
    return dict(summary=dict(passed=ok, trials=len(rows),
                             max_body_dev_mm=max(x["max_body_dev_from_authored_mm"] for x in rows)),
                rows=rows, reset_states=resets)


def run_trial(emb, out, max_steps, record, stage, t0, seed=None):
    import hingekit
    from inspect_robots import Scene
    from inspect_robots_yam.packing import STATE_KEY

    out.mkdir(parents=True, exist_ok=True)
    t1 = time.time()
    obs = emb.reset(Scene(id="demo", instruction="demo"), seed=seed)
    print(f"BENCH reset {time.time() - t1:.0f} s", flush=True)
    reset_state = reset_state_now()
    print("START", json.dumps(hingekit.runtime().spec, default=float), flush=True)
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
        d = hingekit.STATE["detail"]
        st = c.state()
        row = dict(step=steps, score=info.get("score"), level=hingekit.STATE["level"], detail=d,
                   labels=[c.left.label, c.right.label], nonfinite=hingekit.STATE["nonfinite"],
                   cmd=np.round(np.r_[c.left.last_cmd, c.right.last_cmd], 4).tolist(), q=np.round(jp, 4).tolist(),
                   alpha=[round(a, 3) for a in st["alpha"]], hinges=st["hinges"],
                   door_mm=np.round(st["door"][0] * 1000, 2).tolist(),
                   tcp_mm={n: np.round(a.measured_tcp()[0] * 1000, 2).tolist() for n, a in c.arms.items()},
                   tcp_cmd_mm={n: np.round(a.pos * 1000, 2).tolist() for n, a in c.arms.items()},
                   offs={n: np.round(a.offset, 3).tolist() for n, a in c.arms.items()})
        tracef.write(json.dumps(row, default=float) + "\n")
        if steps % 20 == 0:
            tracef.flush()
        if steps % 25 == 0:
            m = d.get("measure", {})
            print(f"step {steps} {c.left.label} | {c.right.label} score {info.get('score')} "
                  f"hinges {d.get('hinges')} alpha {m.get('alpha_deg')} closed {m.get('closed')} "
                  f"clear {m.get('clear_mm')} door {np.round(st['door'][0] * 1000, 1).tolist()} "
                  f"grip {d.get('grip_n')} push {d.get('peak_rear_push_n')} wall {time.time() - t1:.0f}s", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= 2 * CONTROL_HZ:
            break
    score = float(info.get("score", 0.0) or 0.0)
    result = dict(stage=stage, task_score=score, success=success, steps=steps, stop=stop,
                  termination=getattr(res, "termination_reason", None), setup=reset_state["setup"],
                  start=hingekit.runtime().spec, scorer=dict(score=hingekit.STATE["score"], level=hingekit.STATE["level"]),
                  final=hingekit.STATE["detail"], nonfinite=hingekit.STATE["nonfinite"], looks=c.log, tries=getattr(c, "tries", 0),
                  wall_s=round(time.time() - t1, 1), wall_total_s=round(time.time() - t0, 1),
                  worst_ik_mm=dict(left=round(c.left.worst_mm, 2), right=round(c.right.worst_mm, 2)),
                  physics_dt=hingekit.PHYSICS_DT, control_hz=CONTROL_HZ)
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
            imageio.imwrite(out / f"setup_{cam}.png", fs[0])
    print("RESULT", json.dumps({k: result[k] for k in ("stage", "task_score", "success", "steps", "stop", "setup")}),
          flush=True)
    return result, reset_state


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--stage", default="demo")
    ap.add_argument("--cam-size", default="320x176", help="camera WxH (evidence runs use the default)")
    ap.add_argument("--device", default="cpu", help="PhysX device (cpu, or cuda:0); rendering is on the GPU either way")
    ap.add_argument("--seed", default=None,
                    help="reset seed (seeded start state, as in inspect-robots runs); a comma list gives one per trial")
    a = ap.parse_args()
    cw, ch = (int(v) for v in a.cam_size.split("x"))
    run(a.task, a.out, a.max_steps, not a.no_record, a.stage, cw, ch, a.device, a.seed)
