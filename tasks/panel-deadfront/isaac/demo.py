"""Scripted full-score demo (and negative variants) for the panel-deadfront task in the yam_arena embodiment.

    python demo.py task.py OUT [--max-steps N] [--no-record] [--stage demo|<variant>[,<variant>...]] [--seed N]

Acts ONLY through the 14 joint/gripper commands of ``YamArenaEmbodiment.step``. It reads the cover's, the driver's
and the panel's poses and the screws' advance through ``panelkit.bench_state()`` (read-only, like an electrician
watching the shrouds in the windows and the screw heads); it never writes a pose, a joint state or a score. Its joint
commands are rate-limited below the CLI's per-step delta guard and clipped to the URDF joint bounds.

Plan:
1. Both hands pinch the cover's hemmed side edges (fingers closing through the plate, hands coming in from above and
   outside) where it stands in its bench stand, and lift it out of the groove.
2. Carry it to the panel, top leaning out 5 deg, its lower part just touching the tub's bottom flange, and lower it
   until both open-bottom slots sit on the started screws' shanks.
3. Swing it shut about the screws while sliding it sideways (the slots allow 1.6 mm) until the windows are centred on
   the breaker shrouds, which must be done before the shrouds enter the windows; press it flat.
4. The left hand keeps it flat; the right hand lets go, takes the ratcheting nut driver from its rest, sets the
   socket on the right screw's hex and turns it in strokes until the head is down on the cover, then the left screw.
5. Both hands let go and move away.
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
PHI = 65.0  # cover grasp: approach tilted 65 deg from straight down toward the cover's centre line
HEM_HALF = L.COVER_T  # the hem doubles the plate: 3.0 mm, TCP at the cover frame's x = 0
LEAN = 6.5  # top out while the slots go onto the screws (handles 16 mm proud need > 5 deg)
DROP_H = 0.006  # slot tops this far above the shanks before the final drop
PRONG_CLEAR = 0.034  # slot tops this far above the shanks while coming in: the prongs' lower ends clear the heads
FLANGE_TOUCH = 0.0003  # lower part of the cover held 0.3 mm into the bottom flange plane (light contact)
PRESS_FLAT = 0.0005  # cover commanded 0.5 mm into the tub face once shut (a harder push at the outboard hem levers
# the far side of the cover off the tub)
DRV_GRIP_BELOW_TOP = 0.058  # driver: pad tips 58 mm below the handle's top (the top stays clear of the finger carriages)
DRV_PRESS = 0.0030  # socket commanded 3 mm past the washer face (the washer stops it in the mouth chamfer)
STROKE_DEG = 165.0
MAX_STROKES = 14


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


def frame(z, x):
    z = np.asarray(z, float)
    z = z / np.linalg.norm(z)
    x = np.asarray(x, float)
    x = x - z * (x @ z)
    x = x / np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


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
        c[:6] = np.clip(c[:6], self.lo, self.hi)
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
            self.offset = np.clip(self.offset + COMP_GAIN * (self.q - self.measured), -0.25, 0.25)
        else:
            self.offset *= 0.85

    def measured_tcp(self):
        return fastik.tcp_pose(self.measured, self.base)

    def solve(self, pos, rot):
        """IK from the current joint angles (fast DLS, then bounded least squares); other seeds only if both fail, and
        then the solution nearest the current angles (no jump to another arm configuration in mid-move)."""
        from yam_arena.constants import READY

        q, pe, re = fastik.solve(pos, rot, self.q, base=self.base, iters=80)
        if pe < 2e-4 and re < 3e-3:
            return q, pe, re
        q, pe, re = fastik.solve_ls(pos, rot, self.q, base=self.base)
        best = (q, pe, re)
        if pe < 3e-4 and re < 4e-3:
            return best
        for s in (np.array(READY, float), np.array([0.0, 1.6, 1.4, -1.2, 0.0, 0.0])):
            q2, pe2, re2 = fastik.solve_ls(pos, rot, s, base=self.base)
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
        self.label = label or self.label
        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        from yam_arena import ik

        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self.set_pose(p0 + s * (np.asarray(pos) - p0), ik.slerp_matrix(r0, np.asarray(rot), s))
            yield

    def set_grip(self, grip, duration=0.8, label=""):
        yield from self.move(self.pos, self.rot, duration, grip, label)

    def goto(self, pos, rot, duration, grip=None, label=""):
        """Joint-space move to an IK solution of (pos, rot) found from many seeds (for large re-orientations where a
        Cartesian path leaves the reachable set)."""
        from yam_arena import ik

        self.label = label or self.label
        rng = np.random.default_rng(0)
        seeds = [self.q] + [rng.uniform(self.lo, self.hi) for _ in range(60)]
        best = None
        for s in seeds[:25]:
            q, pe, re = fastik.solve_ls(pos, rot, s, base=self.base)
            cost = pe * 1e3 + math.degrees(re) + 0.05 * float(np.abs(q - self.q).sum())
            if pe < 5e-4 and re < 5e-3 and (best is None or cost < best[0]):
                best = (cost, q)
        if best is None:
            raise Stop(f"{self.name} no IK solution for {self.label}")
        q0, q1, g0 = self.q.copy(), best[1], self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.q = q0 + s * (q1 - q0)
            self.grip = g0 + s * (g1 - g0)
            self.pos, self.rot = fastik.tcp_pose(self.q, self.base)
            yield
        self.pos, self.rot = np.asarray(pos, float), np.asarray(rot, float)

    def move_max(self, delta, duration, label=""):
        """Cartesian move by up to ``delta`` (same orientation): as far along it as stays reachable (checked first)."""
        delta = np.asarray(delta, float)
        for f in (1.0, 0.75, 0.5, 0.3, 0.15):
            ok = True
            for s in np.linspace(0.2, 1.0, 5):
                _, pe, re = self.solve(self.pos + f * s * delta, self.rot)
                if pe > 0.001 or re > math.radians(2):
                    ok = False
                    break
            if ok:
                yield from self.move(self.pos + f * delta, self.rot, duration * max(f, 0.5), None, label)
                return
        self.label = label

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
        self.GD = None
        self.corr_p, self.corr_r = np.zeros(3), np.zeros(3)
        self.dcorr = np.zeros(3)
        self.opts = {}

    def bench(self):
        import panelkit

        return panelkit.bench_state()

    def pose(self, name="cover"):
        return self.bench()[name]

    def panel(self):
        return np.asarray(self.bench()["panel_origin"], float)

    def detail(self):
        import panelkit

        return panelkit.STATE["detail"]

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
        row = dict(at=tag, step=self.step, measure=d.get("measure"), level=d.get("level"),
                   screws=d.get("screws"), **kw)
        self.log.append(row)
        print("LOOK", json.dumps(row, default=float), flush=True)


# --------------------------------------------------------------------------------------------- cover handling
def grasp_v(name):
    """TCP pose in the cover frame for each hand: pinching the hem, coming in from above and outside."""
    s = 1.0 if name == "left" else -1.0
    f = math.radians(PHI)
    p = np.array([0.0, s * (L.COVER_W / 2 - L.GRIP_IN), L.GRIP_Z])
    z = np.array([0.0, -s * math.sin(f), -math.cos(f)])
    return p, z


def grasp_rot_v(c, name, pc, rc):
    """Finger direction (+-x of the cover): the one that puts the wrist camera (on the gripper's -y side) outboard of
    the cover's edge (the other puts it against the plate)."""
    _, z = grasp_v(name)
    s = 1.0 if name == "left" else -1.0
    for sg in (1.0, -1.0):
        r = frame(z, [sg, 0.0, 0.0])
        if -r[1, 1] * s > 0.0:  # camera offset direction has an outboard y component
            return r
    raise Stop("no camera-outboard grasp")


def capture_grasp(c):
    """Freeze the cover -> TCP transform of each hand (measured, cover held still)."""
    pc, rc = c.pose()
    for n, arm in c.arms.items():
        pt, rt = arm.measured_tcp()
        c.G[n] = (rc.T @ (pt - pc), rc.T @ rt)
    c.corr_p, c.corr_r = np.zeros(3), np.zeros(3)


def track(c, target_fn, duration, label, gain=0.05, rot_gain=None, until=None, max_s=None, arms=("left", "right"),
          mask=(1.0, 1.0, 1.0)):
    """Closed loop on the cover: each step command the holding hands to the TCP poses that put the cover at
    target_fn(s) through the frozen grasp transforms, plus slow integral corrections of the measured cover pose error
    (arm sag, grasp slip). ``mask`` weights the position error per axis (0 = that axis is left to contact)."""
    from yam_arena import ik

    n = max(1, round(duration * CONTROL_HZ))
    nmax = n if max_s is None else max(n, round(max_s * CONTROL_HZ))
    for k in range(1, nmax + 1):
        pt, rt = target_fn(ik.smoothstep(min(1.0, k / n)))
        pm, rm = c.pose()
        err = (pt - pm) * np.asarray(mask)
        c.corr_p = np.clip(c.corr_p + gain * err, -0.012, 0.012)
        e_r = _rotvec(rt @ rm.T)
        c.corr_r = np.clip(c.corr_r + (gain if rot_gain is None else rot_gain) * e_r, -0.10, 0.10)
        rc = _rotm(c.corr_r)
        for name in arms:
            gp, gr = c.G[name]
            arm = c.arms[name]
            arm.label = label
            arm.set_pose(pt + c.corr_p + rc @ rt @ gp, rc @ rt @ gr)
        yield
        if until is not None and k >= n and until(c):
            break


def cover_target(theta_deg, slot_xz, y):
    """Cover pose with the top leaning out theta, the slot-top centre point (V: (-T/2, 0, SLOT_TOP)) at panel-frame
    (x, z) = slot_xz and the cover centred at panel y; returned in the station frame."""
    r = L.ry(-theta_deg)
    v = np.array([-L.COVER_T / 2, 0.0, L.SLOT_TOP])
    p = np.array([slot_xz[0], y, slot_xz[1]])
    return p - r @ v, r


def hang_slot_x(theta_deg):
    """Panel-frame x of the slot-top point (mid-plate) that puts the cover's lower back face 0.3 mm into the bottom
    flange plane at the flange's lowest edge (z = -TUB_H/2) for a lean of theta."""
    t = math.radians(theta_deg)
    dz = L.SLOT_TOP - (-L.TUB_H / 2)  # slot top above the flange bottom (in the cover's plane)
    # back face at the flange bottom: x = x_slot + T/2 + dz * sin(t)
    return FLANGE_TOUCH - L.COVER_T / 2 - dz * math.sin(t)


def pick_cover(c):
    pc, rc = c.pose()
    for arm in c.arms.values():
        arm.set_comp(True)
    tg = {}
    for n in c.arms:
        p, z = grasp_v(n)
        r = grasp_rot_v(c, n, pc, rc)
        tg[n] = (pc + rc @ p, rc @ r, rc @ z)
    c.note("cover at stand", cover_mm=np.round(pc * 1e3, 2).tolist())
    yield from both(*(arm.move(tg[n][0] - 0.06 * tg[n][2], tg[n][1], 3.0, grip_of(HEM_HALF + 0.010), f"{n} over hem")
                      for n, arm in c.arms.items()))
    yield from wait(0.3)
    pc, rc = c.pose()
    for n in c.arms:
        p, z = grasp_v(n)
        tg[n] = (pc + rc @ p, tg[n][1], rc @ z)
    yield from both(*(arm.move(tg[n][0], tg[n][1], 2.0, None, f"{n} onto hem") for n, arm in c.arms.items()))
    yield from wait(0.3)
    yield from both(*(arm.set_grip(0.0, 1.0, f"{n} pinch") for n, arm in c.arms.items()))
    yield from wait(0.5)
    c.note("pinched")
    yield from both(*(arm.move(arm.pos + [0.0, 0.0, 0.025], arm.rot, 1.5, None, f"{n} lift") for n, arm in
                      c.arms.items()))
    yield from wait(0.6)
    capture_grasp(c)
    c.note("lifted")


def carry_and_hang(c):
    """Carry the cover to the panel leaning LEAN, come in above the screws (the slot prongs clear the heads), then
    lower it straight down so the prongs pass between the heads and the tub flange and the slots sit on the shanks."""
    from yam_arena import ik

    pc, rc = c.pose()
    po = c.panel()
    y = po[1]
    zs = po[2] + L.SCREW_Z + L.SHANK_R  # shank tops (station z)
    x_hang = po[0] + hang_slot_x(LEAN)
    p1, r1 = cover_target(LEAN, (x_hang - 0.030, zs + 0.045), y)
    p0, r0 = pc.copy(), rc.copy()
    yield from track(c, lambda s: (p0 + s * (p1 - p0), ik.slerp_matrix(r0, r1, s)), 4.0, "carry to panel")
    p2, r2 = cover_target(LEAN, (x_hang, zs + PRONG_CLEAR), y)
    yield from track(c, lambda s: (p1 + s * (p2 - p1), r2), 2.0, "in over the screws", gain=0.06)
    yield from track(c, lambda s: (p2, r2), 1.0, "settle over the screws", gain=0.06)
    c.note("over screws")
    p3, _ = cover_target(LEAN, (x_hang, zs + DROP_H), y)
    yield from track(c, lambda s: (p2 + s * (p3 - p2), r2), 2.0, "down past the heads", gain=0.06)
    # lower until both slots sit on the shanks (the cover stops going down)
    p4, _ = cover_target(LEAN, (x_hang, zs - 0.002), y)
    for arm in c.arms.values():
        arm.set_comp("hold")
    yield from track(c, lambda s: (p3 + s * (p4 - p3), r2), 2.0, "drop onto the screws", gain=0.04,
                     mask=(1.0, 1.0, 0.0))
    yield from wait(0.5)
    c.note("hung")


def swing_shut(c):
    po = c.panel()
    y = po[1]
    zs = po[2] + L.SCREW_Z + L.SHANK_R
    x_hang = po[0] + hang_slot_x(LEAN)
    x_flat = po[0] - L.COVER_T / 2 + PRESS_FLAT
    for arm in c.arms.values():
        arm.set_comp(True)
    # first centre the windows on the shrouds sideways while still leaning (the shrouds enter below ~3.5 deg)
    yield from track(c, lambda s: cover_target(LEAN, (x_hang, zs), y), 2.0, "centre sideways", gain=0.08,
                     mask=(1.0, 1.0, 0.3), until=lambda c: abs(c.pose()[0][1] - y) < 0.0003, max_s=4.0)
    c.note("centred", dy_mm=(c.pose()[0][1] - y) * 1e3)
    yield from track(c, lambda s: cover_target(LEAN * (1 - s), (x_hang + s * (x_flat - x_hang), zs), y), 4.0,
                     "swing shut", gain=0.05, mask=(1.0, 1.0, 0.3))
    for arm in c.arms.values():
        arm.set_comp("hold")  # in contact with the tub: no joint integral wind-up
    yield from track(c, lambda s: cover_target(0.0, (x_flat, zs), y), 1.5, "press flat", gain=0.05,
                     mask=(0.0, 1.0, 0.3))
    c.note("shut")


# --------------------------------------------------------------------------------------------- driver handling
def driver_rot(phi):
    """Driver frame pointing at the wall (socket toward +x), roll phi (driver x axis angle in the y-z plane)."""
    return L.driver_rot_x(phi)


def driver_roll(rd):
    return math.atan2(rd[2, 0], rd[1, 0])


def capture_driver(c):
    pd, rd = c.pose("driver")
    pt, rt = c.right.measured_tcp()
    c.GD = (rt.T @ (pd - pt), rt.T @ rd)  # driver pose in the TCP frame
    c.dcorr = np.zeros(3)
    c.note("driver grasp", mouth_in_tcp_mm=np.round(c.GD[0] * 1e3, 2).tolist(),
           axis_in_tcp=np.round(-c.GD[1][:, 2], 4).tolist())


def tcp_for_driver(c, pd, rd):
    gp, gr = c.GD
    rt = rd @ gr.T
    return pd - rt @ gp, rt


def _driver_of_q(c, q):
    """Driver mouth position, socket direction and roll for right-arm joint angles q (frozen in-hand transform)."""
    gp, gr = c.GD
    pt, rt = fastik.tcp_pose(q, c.right.base)
    rd = rt @ gr
    return pt + rt @ gp, -rd[:, 2], driver_roll(rd)


J6_PARK = -1.55  # wrist roll kept near here while the socket goes on (room for a +165 deg stroke, margin to the limit)


def driver_ik(c, mouth, axis=(1.0, 0.0, 0.0), roll=None, q0=None, j6_pref=None):
    """Right-arm IK on the driver itself: socket mouth at ``mouth``, socket pointing along ``axis``, roll free unless
    given (the roll is free for the approach; the strokes set it). Bounded least squares from q0."""
    from scipy.optimize import least_squares

    arm = c.right
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    mouth = np.asarray(mouth, float)

    def resid(q):
        m, ax, ro = _driver_of_q(c, q)
        r = [(m - mouth) * 1000.0, np.cross(ax, axis) * 300.0]
        if roll is not None:
            r.append([((ro - roll + math.pi) % (2 * math.pi) - math.pi) * 30.0])
        if j6_pref is not None:
            r.append([(q[5] - j6_pref) * 3.0])  # weak: keeps the free roll away from the wrist limits
        return np.concatenate(r)

    x0 = np.clip(np.asarray(arm.q if q0 is None else q0, float), arm.lo + 1e-6, arm.hi - 1e-6)
    res = least_squares(resid, x0, bounds=(arm.lo, arm.hi), max_nfev=150, xtol=1e-10, ftol=1e-10)
    m, ax, ro = _driver_of_q(c, res.x)
    return res.x, float(np.linalg.norm(m - mouth)), math.degrees(math.acos(float(np.clip(ax @ axis, -1, 1)))), ro


def set_driver(c, mouth, axis=(1.0, 0.0, 0.0), roll=None, label=""):
    q, pe, ae, ro = driver_ik(c, mouth, axis, roll, j6_pref=J6_PARK if roll is None else None)
    if pe > 0.0015 or ae > 3.0:
        raise Stop(f"right: driver unreachable at {label}: {pe * 1e3:.1f} mm {ae:.1f} deg")
    arm = c.right
    arm.worst_mm = max(arm.worst_mm, pe * 1000)
    arm.q = q
    arm.pos, arm.rot = fastik.tcp_pose(q, arm.base)
    arm.label = label
    return ro


def stroke_room(c, mouth, phi):
    """Largest roll increment (deg, up to STROKE_DEG) the arm can follow from its current joints, in 15 deg steps."""
    q = c.right.q.copy()
    room = 0.0
    for d_ in np.arange(15.0, STROKE_DEG + 0.1, 15.0):
        q, pe, ae, _ = driver_ik(c, mouth, roll=phi + math.radians(d_), q0=q)
        if pe > 0.0008 or ae > 2.0:
            break
        room = float(d_)
    return room


def goto_driver(c, mouth, duration, label, axis=(1.0, 0.0, 0.0), j6_pref=J6_PARK):
    """Joint-space move to a driver pose (roll free) found from many seeds; prefers wrist roll (j6) near ``j6_pref`` so
    the ratchet strokes (positive roll) have room."""
    from yam_arena import ik

    arm = c.right
    rng = np.random.default_rng(0)
    seeds = [arm.q] + [rng.uniform(arm.lo, arm.hi) for _ in range(40)]
    best = None
    for s in seeds:
        q, pe, ae, ro = driver_ik(c, mouth, axis, None, q0=s)
        if pe < 0.0005 and ae < 1.0:
            # roll is free: put j6 near the preferred value by turning the wrist (same pose otherwise)
            cost = abs(q[5] - j6_pref) + 0.05 * float(np.abs(q - arm.q).sum())
            if best is None or cost < best[0]:
                best = (cost, q)
    if best is None:
        raise Stop(f"right: no driver IK solution for {label}")
    q1 = best[1].copy()
    # re-solve with the roll that puts j6 at the preferred value (if within limits)
    _, _, ro = _driver_of_q(c, q1)
    target_roll = ro + (j6_pref - q1[5])
    q2, pe2, ae2, _ = driver_ik(c, mouth, axis, target_roll, q0=np.r_[q1[:5], j6_pref])
    if pe2 < 0.0005 and ae2 < 1.0:
        q1 = q2
    q0 = arm.q.copy()
    n = max(1, round(duration * CONTROL_HZ))
    arm.label = label
    for k in range(1, n + 1):
        s = ik.smoothstep(k / n)
        arm.q = q0 + s * (q1 - q0)
        arm.pos, arm.rot = fastik.tcp_pose(arm.q, arm.base)
        yield


def track_driver(c, target_fn, duration, label, gain=0.06, until=None, max_s=None, correct=True):
    """target_fn(s) -> (mouth position, roll or None): driver IK every step, socket along +x, with a slow integral
    correction of the measured mouth position across the axis (the axial direction is left to contact)."""
    from yam_arena import ik

    n = max(1, round(duration * CONTROL_HZ))
    nmax = n if max_s is None else max(n, round(max_s * CONTROL_HZ))
    for k in range(1, nmax + 1):
        p_t, roll = target_fn(ik.smoothstep(min(1.0, k / n)))
        if correct:
            pd_m, _ = c.pose("driver")
            e = p_t - pd_m
            e[0] = 0.0
            c.dcorr = np.clip(c.dcorr + gain * e, -0.008, 0.008)
        set_driver(c, p_t + c.dcorr, roll=roll, label=label)
        yield
        if until is not None and k >= n and until(c):
            break


def take_driver(c):
    """Right hand: straight down over the driver standing in its holster, pads on the handle 58 mm below its top, lift
    it out (the shaft clears the holster)."""
    from yam_arena.constants import PAD_TIP_PAST_TCP

    arm = c.right
    arm.set_comp(True)
    pd, rd = c.pose("driver")
    top = pd + rd @ np.array([0.0, 0.0, L.DRV_HANDLE_Z0 + L.DRV_HANDLE_L])
    r = frame([0.0, 0.0, -1.0], [1.0, 0.0, 0.0])
    p = top - np.array([0.0, 0.0, DRV_GRIP_BELOW_TOP - PAD_TIP_PAST_TCP])
    yield from arm.goto(p + [0.0, 0.0, 0.05], r, 3.0, grip_of(L.DRV_HANDLE_R + 0.012), "right over driver")
    yield from wait(0.3)
    pd, rd = c.pose("driver")  # re-read (closed loop on the driver's place)
    top = pd + rd @ np.array([0.0, 0.0, L.DRV_HANDLE_Z0 + L.DRV_HANDLE_L])
    p = top - np.array([0.0, 0.0, DRV_GRIP_BELOW_TOP - PAD_TIP_PAST_TCP])
    yield from arm.move(p, r, 1.5, None, "right onto handle")
    yield from wait(0.6)  # the joint integral takes out the arm's sag before the fingers close
    arm.set_comp("hold")  # in contact from here: no integral wind-up
    yield from arm.set_grip(grip_of(L.DRV_HANDLE_R - 0.005), 0.8, "right grip driver")
    yield from wait(0.4)
    yield from arm.move(p + [0.0, 0.0, 0.065], r, 2.0, None, "driver out of holster")
    arm.set_comp(True)
    yield from arm.move(np.array([0.16, -0.12, p[2] + 0.065]), r, 1.5, None, "driver toward the robot")
    yield from wait(0.3)
    capture_driver(c)
    c.note("driver in hand")


def screw_axis(c, i):
    return np.asarray(c.bench()["screws"]["axis"][i], float)


def washer_x(c, i):
    adv = c.bench()["screws"]["adv"][i]
    return screw_axis(c, i)[0] + L.head_under_x(adv) - L.HEAD_WASHER_T


def drive_screw(c, i):
    """Set the socket on screw i and turn it in ratchet strokes until the head is down on the cover."""
    a = screw_axis(c, i)
    stand = a + np.array([washer_x(c, i) - a[0] - 0.030, 0.0, 0.0])
    far = stand + np.array([-0.012, 0.0, 0.0])
    yield from goto_driver(c, far, 3.0 if not c.opts.get("driver_at_screws") else 2.0, f"driver to screw {i}")
    c.opts["driver_at_screws"] = True
    yield from wait(0.5)
    capture_driver(c)  # the driver turns a little in the fingers when the hand turns it horizontal: measure it again
    yield from track_driver(c, lambda s: (far + s * (stand - far), None), 1.0, f"driver in {i}", correct=False)
    yield from wait(0.3)
    capture_driver(c)
    yield from track_driver(c, lambda s: (stand, None), 1.2, f"driver line-up {i}", gain=0.08)
    c.right.set_comp("hold")
    eng = a + np.array([washer_x(c, i) - a[0] + DRV_PRESS, 0.0, 0.0])
    yield from track_driver(c, lambda s: (stand + s * (eng - stand), None), 1.5, f"socket on {i}", gain=0.06)
    _, _, phi = _driver_of_q(c, c.right.q)  # roll at the start of the strokes
    yield from wait(0.3)
    c.note("socket on", screw=i, eng=c.detail().get("screws", {}).get("eng"))
    # strokes: wrist roll +STROKE_DEG (tightening, ratchet engaged), back (ratchet slips)
    for k in range(MAX_STROKES):
        if c.bench()["screws"]["seated"][i]:
            break
        eng = a + np.array([washer_x(c, i) - a[0] + DRV_PRESS, 0.0, 0.0])
        sweep = stroke_room(c, eng + c.dcorr, phi)
        if sweep < 30.0:
            raise Stop(f"screw {i}: no room for a ratchet stroke ({sweep:.0f} deg)")
        dur = 1.05 * sweep / STROKE_DEG
        yield from track_driver(c, lambda s, e=eng, w=sweep: (e, phi + s * math.radians(w)), dur,
                                f"stroke {i}.{k}", gain=0.04)
        eng = a + np.array([washer_x(c, i) - a[0] + DRV_PRESS, 0.0, 0.0])
        yield from track_driver(c, lambda s, e=eng, w=sweep: (e, phi + (1 - s) * math.radians(w)), 0.85 * dur,
                                f"return {i}.{k}", gain=0.04)
        c.note("stroke", screw=i, k=k, sweep_deg=sweep)
    if not c.bench()["screws"]["seated"][i]:
        raise Stop(f"screw {i} not seated after {MAX_STROKES} strokes")
    c.right.set_comp(True)
    back = a + np.array([washer_x(c, i) - a[0] - 0.040, 0.0, 0.0])
    pd, _ = c.pose("driver")
    yield from track_driver(c, lambda s: (pd + s * (back - pd), phi), 1.2, f"off screw {i}", correct=False)
    c.note("seated", screw=i)


def release_right(c):
    """Right hand lets go of the hem and backs away along its approach, then straight toward the robot (clear of the
    cover's edge) before going home."""
    arm = c.right
    arm.set_comp("hold")
    _, z = grasp_v("right")
    pc, rc = c.pose()
    yield from arm.set_grip(grip_of(HEM_HALF + 0.012), 0.8, "right lets go")
    yield from arm.move_max(-0.03 * (rc @ z), 0.8, "right off the hem")
    yield from arm.move_max(np.array([0.0, -0.018, 0.0]), 0.8, "right clear of the edge")
    arm.set_comp(True)
    yield from arm.home(2.0)
    c.note("right released")


def release_left(c):
    arm = c.left
    arm.set_comp("hold")
    _, z = grasp_v("left")
    pc, rc = c.pose()
    yield from arm.set_grip(grip_of(HEM_HALF + 0.012), 0.8, "left lets go")
    yield from arm.move_max(-0.03 * (rc @ z), 0.8, "left off the hem")
    yield from arm.move_max(np.array([0.0, 0.018, 0.0]), 0.8, "left clear of the edge")
    arm.set_comp(True)
    yield from arm.home(2.5)
    c.note("left released")


def park_right(c):
    """The driver stays where 'off screw' left it, 40 mm back from the screw head, the hand far from the cover."""
    c.right.label = "driver held back"
    yield from wait(0.3)


def hold_left(c, seconds, label="left holds the cover flat"):
    """Keep the left hand where it is (cover flat) for the given time."""
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        c.left.label = label
        yield


def left_hold_during(c, gen):
    """Run a right-arm generator while the left hand keeps its pose."""
    yield from gen


# --------------------------------------------------------------------------------------------- stages
def demo(c):
    yield from wait(0.5)
    yield from pick_cover(c)
    yield from carry_and_hang(c)
    yield from swing_shut(c)
    c.left.set_comp("hold")
    yield from release_right(c)
    yield from take_driver(c)
    yield from drive_screw(c, 1)
    yield from drive_screw(c, 0)
    yield from park_right(c)
    yield from release_left(c)
    yield from wait(4.0)


def idle(c):
    yield from wait(8.0)


def hang_only(c):
    """Development: pick, hang and swing shut, then hold."""
    yield from wait(0.5)
    yield from pick_cover(c)
    yield from carry_and_hang(c)
    yield from swing_shut(c)
    yield from wait(3.0)


def neg_no_centre(c):
    """Swings the cover shut without the sideways correction, with the cover deliberately 1.5 mm off to the robot's
    left on the screws (inside the slots' play): a shroud meets the window edge, the cover cannot close flat."""
    yield from wait(0.5)
    yield from pick_cover(c)
    c.opts["dy"] = 0.0015
    yield from carry_and_hang(c)
    po = c.panel()
    y = po[1] + 0.0015
    zs = po[2] + L.SCREW_Z + L.SHANK_R
    x_hang = po[0] + hang_slot_x(LEAN)
    x_flat = po[0] - L.COVER_T / 2 + PRESS_FLAT
    for arm in c.arms.values():
        arm.set_comp(True)
    yield from track(c, lambda s: cover_target(LEAN, (x_hang, zs), y), 2.0, "offset sideways", gain=0.08,
                     mask=(1.0, 1.0, 0.3))
    c.note("offset", dy_mm=(c.pose()[0][1] - po[1]) * 1e3)
    yield from track(c, lambda s: cover_target(LEAN * (1 - s), (x_hang + s * (x_flat - x_hang), zs), y), 4.0,
                     "swing shut (no centring)", gain=0.05, mask=(1.0, 0.0, 0.3))
    yield from track(c, lambda s: cover_target(0.0, (x_flat, zs), y), 1.5, "press flat", gain=0.05,
                     mask=(1.0, 0.0, 0.3))
    c.note("shut attempt")
    c.left.set_comp("hold")
    yield from release_right(c)
    yield from take_driver(c)
    try:
        yield from drive_screw(c, 1)
        yield from drive_screw(c, 0)
    except Stop as e:
        c.note("stop", why=str(e))
    yield from park_right(c)
    yield from release_left(c)
    yield from wait(4.0)


def neg_no_hold(c):
    """Shuts the cover correctly but lets go with both hands before the screws are driven: the cover tips open on
    the screws (nothing holds its top), so the screws clamp it crooked or not at all."""
    yield from wait(0.5)
    yield from pick_cover(c)
    yield from carry_and_hang(c)
    yield from swing_shut(c)
    c.left.set_comp("hold")
    yield from release_right(c)
    yield from release_left(c)
    yield from wait(1.5)
    pc, rc = c.pose()
    c.note("let go before screws", lean_deg=math.degrees(math.atan2(-rc[0, 2], rc[2, 2])))
    yield from take_driver(c)
    try:
        yield from drive_screw(c, 1)
        yield from drive_screw(c, 0)
    except Stop as e:
        c.note("stop", why=str(e))
    yield from park_right(c)
    yield from wait(4.0)


def neg_one_screw(c):
    """Drives only the right screw (0.5)."""
    yield from wait(0.5)
    yield from pick_cover(c)
    yield from carry_and_hang(c)
    yield from swing_shut(c)
    c.left.set_comp("hold")
    yield from release_right(c)
    yield from take_driver(c)
    yield from drive_screw(c, 1)
    yield from park_right(c)
    yield from release_left(c)
    yield from wait(4.0)


def replay(c):
    """Open-loop check: plays the 14-D commands recorded in a finished run's trace.jsonl (path in PANEL_REPLAY), step
    by step, without reading anything: what a fixed trajectory scores on another start state."""
    import os

    rows = [json.loads(line) for line in open(os.environ["PANEL_REPLAY"])]
    c.replay = [r["cmd"] for r in rows]
    for _ in range(len(c.replay) + 40):
        yield


STAGES = {
    "demo": demo,
    "idle": idle,
    "hang-only": hang_only,
    "replay": replay,
    "neg-no-centre": neg_no_centre,
    "neg-no-hold": neg_no_hold,
    "neg-one-screw": neg_one_screw,
}


def _write_mp4(frames, path):
    import imageio

    w = imageio.get_writer(str(path), fps=CONTROL_HZ, codec="libx264", quality=8, macro_block_size=8,
                           ffmpeg_params=["-pix_fmt", "yuv420p"])
    for f in frames:
        w.append_data(np.asarray(f))
    w.close()


def run(task_path, out_dir, max_steps=3600, record=True, stage="demo", cam_width=320, cam_height=176, device="cpu",
        seed=None):
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    t0 = time.time()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=str(Path(task_path).resolve()), object_jitter=0, control_hz=CONTROL_HZ,
                               cam_width=cam_width, cam_height=cam_height, device=device)
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
    import panelkit

    rt = panelkit.runtime()
    poses = rt.read()
    return dict(score=panelkit.STATE["score"], setup=rt.setup, adv_mm=[v * 1e3 for v in rt.adv],
                bodies={n: np.round(p * 1000, 4).tolist() for n, (p, r) in poses.items()},
                rots={n: np.round(r, 6).tolist() for n, (p, r) in poses.items()})


def reset_report(stages, resets, results):
    import panelkit

    rows = []
    for i, r in enumerate(resets):
        want = panelkit.authored(tuple(r["setup"]) if isinstance(r["setup"], list) else r["setup"])
        dev = max(float(np.linalg.norm(np.subtract(r["bodies"][n], want[n][0] * 1000))) for n in ("cover", "driver"))
        rows.append(dict(trial=i, stage=stages[i], setup=r["setup"], score_after_reset=r["score"],
                         screws_adv_mm=r["adv_mm"], max_body_dev_from_authored_mm=round(dev, 4),
                         trial_score=results[i]["task_score"], trial_stop=results[i]["stop"]))
    ok = all(x["score_after_reset"] == 0.0 and x["max_body_dev_from_authored_mm"] < 1.0
             and max(x["screws_adv_mm"]) == 0.0 for x in rows)
    return dict(summary=dict(passed=ok, trials=len(rows),
                             max_body_dev_mm=max(x["max_body_dev_from_authored_mm"] for x in rows)),
                rows=rows, reset_states=resets)


def run_trial(emb, out, max_steps, record, stage, t0, seed=None):
    import panelkit
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
    grip_peak = [0.0, 0.0]
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
        d = panelkit.STATE["detail"]
        row = dict(step=steps, score=info.get("score"), level=panelkit.STATE["level"], detail=d,
                   labels=[c.left.label, c.right.label], nonfinite=panelkit.STATE["nonfinite"],
                   cmd=np.round(np.r_[c.left.last_cmd, c.right.last_cmd], 4).tolist(), q=np.round(jp, 4).tolist(),
                   corr_mm=np.round(c.corr_p * 1000, 3).tolist(), corr_r=np.round(c.corr_r, 4).tolist(),
                   dcorr_mm=np.round(c.dcorr * 1000, 3).tolist(),
                   tcp_mm={n: np.round(a.measured_tcp()[0] * 1000, 2).tolist() for n, a in c.arms.items()},
                   tcp_cmd_mm={n: np.round(a.pos * 1000, 2).tolist() for n, a in c.arms.items()})
        tracef.write(json.dumps(row, default=float) + "\n")
        if steps % 20 == 0:
            tracef.flush()
        if steps % 25 == 0:
            m = d.get("measure", {})
            sc = d.get("screws", {})
            print(f"step {steps} {c.left.label} | {c.right.label} score {info.get('score')} gaps {m.get('gaps_mm')} "
                  f"hung {m.get('hung')} slot {m.get('slot_dy_mm')} {m.get('slot_z_mm')} adv {sc.get('adv_mm')} "
                  f"eng {sc.get('engaged')} seat {sc.get('seated')} cont {d.get('contacts', {}).get('by')} "
                  f"wall {time.time() - t1:.0f}s", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= 3 * CONTROL_HZ:
            break
    score = float(info.get("score", 0.0) or 0.0)
    result = dict(stage=stage, task_score=score, success=success, steps=steps, stop=stop,
                  termination=getattr(res, "termination_reason", None), setup=reset_state["setup"],
                  scorer=dict(score=panelkit.STATE["score"], level=panelkit.STATE["level"]),
                  final=panelkit.STATE["detail"], nonfinite=panelkit.STATE["nonfinite"], looks=c.log,
                  wall_s=round(time.time() - t1, 1), wall_total_s=round(time.time() - t0, 1),
                  worst_ik_mm=dict(left=round(c.left.worst_mm, 2), right=round(c.right.worst_mm, 2)),
                  physics_dt=panelkit.PHYSICS_DT, control_hz=CONTROL_HZ)
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
    ap.add_argument("--max-steps", type=int, default=3600)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--stage", default="demo")
    ap.add_argument("--cam-size", default="320x176", help="camera WxH (evidence runs use the default)")
    ap.add_argument("--device", default="cpu", help="PhysX device (cpu, or cuda:0); rendering is on the GPU either way")
    ap.add_argument("--seed", type=int, default=None, help="reset seed (seeded start state, as in inspect-robots runs)")
    a = ap.parse_args()
    cw, ch = (int(v) for v in a.cam_size.split("x"))
    run(a.task, a.out, a.max_steps, not a.no_record, a.stage, cw, ch, a.device, a.seed)
    import os

    sys.stdout.flush()
    os._exit(0)
