"""Scripted full-score demo (and negative variants) for the edge-ring task in the yam_arena embodiment.

    python demo.py task.py OUT [--max-steps N] [--no-record] [--stage demo|<variant>[,<variant>...]]

Acts ONLY through the 14 joint/gripper commands of ``YamArenaEmbodiment.step``. It reads the ring's pose through
``ringkit.bench_state()`` (read-only, like an operator watching the slots and pins); it never writes a pose, a joint
state or a score. Its joint commands are rate-limited below the CLI's per-step delta guard.

Plan (both arms, top-down, each hand pinching the ring across its 30.5 mm width, one finger in the bore and one
outside the rim, at ring angles +125 / -125 deg):
1. Grip the ring on its stand at those two points (where they are depends on the setup's stand angle).
2. Lift it, carry it over the chamber wall while turning it to the fitted clocking, and lower it into the bowl.
3. Hold it 6 mm above the raised lift-pin tips and correct its position and clocking from the measured ring pose
   (the slots over the locating pins), then lower it at 4 mm/s until it rests on the pin tips.
4. Relax the arms, open both hands, lift them straight up and out of the chamber.
5. Press RING DOWN on the pendant with the right hand's closed fingertips; both arms go back to READY while the lift
   pins lower the ring onto its locating pins.
Both arms add the remaining joint error back to their commands (slow integral on their own joint readings), because
the joint PD sags under load (pattern reused from earlier demos).
"""

from __future__ import annotations

import argparse
import json
from functools import lru_cache
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
RING_W = L.RING_R[1] - L.RING_R[0]
R_MID = (L.RING_R[0] + L.RING_R[1]) / 2
SQUEEZE = (
    0.0025  # fingers commanded 2.5 mm inside the ring faces: about 12.5 N per finger
)
OPEN_HALF = (
    RING_W / 2 + 0.010
)  # 10 mm clearance a side before closing / after letting go
TIP_BELOW = 0.0025  # pad tips 2.5 mm below the ring's bottom face
TCP_ABOVE_BOTTOM = 0.013 - TIP_BELOW
PSI = {"left": 125.0, "right": -125.0}  # grip angles in the ring frame (deg)
COMP_GAIN = 0.08
RATE = 0.8  # fraction of the CLI delta guard (0.05 x joint range per step) the script may use
CARRY_Z = L.BOWL_TOP + 0.012  # ring bottom clears the complete rim before traversing it
HOVER = 0.006  # alignment height above the lift-pin tips
ALIGN_TOL_M = 0.00015
ALIGN_DEADBAND_M = 0.00008  # re-align: no integral action inside 0.08 mm (stops hunting against stiction)
# ring level in the hands before it is lowered (0.6 deg keeps its low side off the pins 2 mm below; a two-point
# grip lets the ring pivot about the grip line, so finer is slow)
TILT_TOL_DEG = 0.6
ROT_CORR_MAX = 0.12  # rad: authority of the ring-orientation correction (arm sag tilts the held ring ~1.5 deg)


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
        ring before the fingers open."""
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

    def pose(self, name="ring"):
        import ringkit

        return ringkit.bench_state()[name]

    def detail(self):
        import ringkit

        return ringkit.STATE["detail"]

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack

        if (
            getattr(self, "replay", None) is not None
        ):  # open-loop replay of a recorded command trace
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
        row = dict(
            at=tag, step=self.step, measure=d.get("measure"), level=d.get("level"), **kw
        )
        self.log.append(row)
        print("LOOK", json.dumps(row, default=float), flush=True)


def down_rot(radial_xy, arm):
    """Straight-down TCP, fingers closing along the given horizontal (radial) direction; sign = least joint-6 travel."""
    best = None
    for sgn in (1.0, -1.0):
        x = sgn * np.array([radial_xy[0], radial_xy[1], 0.0])
        x /= np.linalg.norm(x)
        z = np.array([0.0, 0.0, -1.0])
        r = np.column_stack([x, np.cross(z, x), z])
        q, pe, re = arm.solve(arm.pos, r)
        cost = abs(q[5]) / 2.094 + pe * 10
        if best is None or cost < best[0]:
            best = (cost, r)
    return best[1]


def grip_point(pr, rr, psi_deg, dz=0.0):
    a = math.radians(psi_deg)
    radial = rr @ np.array([math.cos(a), math.sin(a), 0.0])
    return pr + R_MID * radial + np.array([0.0, 0.0, TCP_ABOVE_BOTTOM + dz]), radial


def ring_yaw(rr):
    return math.degrees(math.atan2(rr[1, 0], rr[0, 0]))


def capture_grasp(c):
    """Freeze the ring -> TCP transform of each hand (measured once, with the ring held still)."""
    pr, rr = c.pose()
    for n, arm in c.arms.items():
        pt, rt = arm.measured_tcp()
        c.G[n] = (rr.T @ (pt - pr), rr.T @ rt)
    c.corr_p, c.corr_r = np.zeros(3), np.zeros(3)


def ring_target(xy, z_bottom, yaw_deg):
    return np.array([xy[0], xy[1], z_bottom]), L.rz(yaw_deg)


def track(
    c,
    target_fn,
    duration,
    label,
    gain=0.05,
    rot_gain=None,
    free_z=False,
    until=None,
    max_s=None,
    deadband=0.0,
):
    """Closed loop on the ring: each step command both hands to the TCP poses that put the ring at target_fn(s)
    through the frozen grasp transforms, plus slow integral corrections of the measured ring position / orientation
    error (arm sag, grasp slip). ``until(c)`` ends the move early (checked every step after the nominal duration
    too, up to ``max_s``). ``deadband`` (m): position errors smaller than this are not integrated (and rotation
    errors smaller than deadband / 0.16 m), so the loop stops pushing once close instead of hunting against the arms'
    joint stiction."""
    from yam_arena import ik

    n = max(1, round(duration * CONTROL_HZ))
    nmax = n if max_s is None else max(n, round(max_s * CONTROL_HZ))
    for k in range(1, nmax + 1):
        pt, rt = target_fn(ik.smoothstep(min(1.0, k / n)))
        pm, rm = c.pose()
        err = pt - pm
        if free_z:
            err[2] = 0.0
        if deadband > 0:
            err = np.where(np.abs(err) < deadband, 0.0, err)
        c.corr_p = np.clip(c.corr_p + gain * err, -0.012, 0.012)
        e_r = _rotvec(rt @ rm.T)
        if deadband > 0:
            e_r = np.where(np.abs(e_r) < deadband / L.LOC_PITCH_R, 0.0, e_r)
        c.corr_r = np.clip(
            c.corr_r + (gain if rot_gain is None else rot_gain) * e_r,
            -ROT_CORR_MAX,
            ROT_CORR_MAX,
        )
        rc = _rotm(c.corr_r)
        for name, arm in c.arms.items():
            gp, gr = c.G[name]
            arm.label = label
            arm.set_pose(pt + c.corr_p + rc @ rt @ gp, rc @ rt @ gr)
        yield
        if until is not None and k >= n and until(c):
            break


def settled(n=6):
    """until-condition: the ring has been aligned (both offsets < ALIGN_TOL_M) and level for ``n`` consecutive control
    steps, i.e. not just passing through alignment in the middle of a stick-slip swing of the arms."""
    count = [0]

    def ok(c):
        ea, eb = slot_error(c)
        count[0] = (
            count[0] + 1 if (ea < ALIGN_TOL_M and eb < ALIGN_TOL_M and level(c)) else 0
        )
        return count[0] >= n

    return ok


def level(c):
    """The held ring is level within TILT_TOL_DEG (measured pose)."""
    rr = c.pose()[1]
    return math.degrees(math.acos(float(np.clip(rr[2, 2], -1.0, 1.0)))) < TILT_TOL_DEG


def slot_error(c):
    """Largest locating-pin offset from its slot centre (m), across / along the slot."""
    import scoring

    pr, rr = c.pose()
    off = scoring.pin_offsets(pr, rr)
    return max(abs(o[0]) for o in off), max(abs(o[1]) for o in off)


# --------------------------------------------------------------------------------------------------- the script
def carry_samples(pr, rr, yaw_end=0.0):
    """Ring poses (centre-bottom position, yaw) sampled along the planned carry (as carry() moves it), ending at
    ``yaw_end`` (0 = fitted; the no-turn negatives keep the stand angle)."""
    y0 = ring_yaw(rr)
    p0 = pr + np.array([0.0, 0.0, 0.016])
    xy1 = np.array([L.C[0] - 0.060, L.C[1]])
    z1 = max(L.C[2] + CARRY_Z + 0.010, L.STAND[2] + 0.015, p0[2] - 0.02)
    out = [
        (
            np.r_[p0[:2] + s * (xy1 - p0[:2]), p0[2] + s * (z1 - p0[2])],
            y0 + s * (yaw_end - y0),
        )
        for s in np.linspace(0, 1, 13)
    ]
    out += [
        (np.r_[xy1 + s * (L.C[:2] - xy1), z1], yaw_end) for s in np.linspace(0, 1, 5)
    ]
    out += [
        (np.r_[L.C[:2], z1 + s * (L.C[2] + L.LIFT_UP + HOVER - z1)], yaw_end)
        for s in np.linspace(0, 1, 5)
    ]
    return out


@lru_cache(maxsize=1)
def arm_surface_points():
    """Sample the official arm collision hulls on a 3 mm grid for rim clearance checks."""
    from yam_arena.analysis import link_hulls

    clouds = {}
    for name, hulls in link_hulls().items():
        points = np.concatenate([hull.vertices for hull in hulls])
        clouds[name] = np.unique(np.round(points / 0.003), axis=0) * 0.003
    return clouds


def rim_clearance(q, base, opening=RING_W / 2):
    from yam_arena.analysis import fk_link_poses_world
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    joints = dict(zip(ARM_JOINTS, q))
    joints.update(dict.fromkeys(FINGER_JOINTS, -opening))
    transforms = fk_link_poses_world(joints, base)
    clearance = math.inf
    for name, local in arm_surface_points().items():
        tf = transforms[name]
        points = local @ tf[:3, :3].T + tf[:3, 3] - L.C
        radius = np.hypot(points[:, 0], points[:, 1])
        # Negative inside the annulus. Treat its six-degree inner chords conservatively.
        gap = np.maximum.reduce((L.BOWL_R[0] * math.cos(math.radians(3)) - radius,
                                 radius - L.BOWL_R[1], points[:, 2] - L.BOWL_TOP))
        clearance = min(clearance, float(gap.min()))
    return clearance


def choose_grips(c, pr, rr):
    """Two grip points on opposite sides of the ring (on a diameter through its centre of mass) and finger orientations, chosen so that both arms reach the whole
    pick and carry (URDF IK only) with the least wrist-roll travel and no finger over a stand arm; the ring's stand
    angle is read from its pose."""
    from yam_arena.constants import READY

    from yam_arena import ik

    samples = carry_samples(pr, rr, getattr(c, "plan_yaw", 0.0))
    options = {"left": [], "right": []}
    for offset in (0, -5, 5, -10, 10, -15, 15, -20, 20, -25, 25, -30, 30, -35, 35):
        for n, centre in (("left", 90), ("right", -90)):
            psi = centre + offset
            arm = c.arms[n]
            world = psi + ring_yaw(rr)
            if (
                min(
                    abs((world - a + 180.0) % 360.0 - 180.0) for a in L.STAND_ARM_ANGLES
                )
                < 10.0
            ):
                continue  # a finger would come down on a stand arm
            choice = None
            # Lean the wrist into the chamber and along its tangent, keeping the pads
            # across the ring. Mirror the tangential lean for the two arm bases.
            tilts = ((12, 20), (24, 20), (24, 40), (12, 40), (0, 20),
                     (36, 40), (0, 0), (12, 0), (24, 0), (-12, 0), (-24, 0))
            for tilt, twist in tilts:
                for flip in (1.0, -1.0):
                    p, radial = grip_point(pr, rr, psi)
                    a = math.radians(tilt)
                    up = np.array([0.0, 0.0, 1.0])
                    x = flip * (math.cos(a) * radial + math.sin(a) * up)
                    b = math.radians(twist if n == "right" else -twist)
                    z = math.cos(b) * (math.sin(a) * radial - math.cos(a) * up) + math.sin(b) * np.cross(up, radial)
                    rot = np.column_stack([x, np.cross(z, x), z])
                    gp, gr = rr.T @ (p - pr), rr.T @ rot
                    q = np.array(READY, float)
                    worst, j6 = 0.0, 0.0
                    above = p + [0, 0, 0.021]
                    approach = [(arm.pos + s * (above - arm.pos), ik.slerp_matrix(arm.rot, rot, s))
                                for s in np.linspace(0, 1, 17)[1:]]
                    for pp, rot_ in approach + [(p, rot)] + [
                        (pc + L.rz(y) @ gp, L.rz(y) @ gr) for pc, y in samples
                    ]:
                        q, pe, re = fastik.solve(pp, rot_, q, base=arm.base, iters=80)
                        # A grasp at a joint stop leaves no travel for measured grasp offsets
                        # or the closed-loop levelling corrections during transport.
                        if np.min(np.minimum(q - arm.lo, arm.hi - q)) < math.radians(5):
                            worst = math.inf
                            break
                        worst = max(worst, pe * 1e3 + math.degrees(re))
                        j6 = max(j6, abs(q[5]))
                        if worst >= 0.5:
                            break
                    if worst < 0.5:
                        # Include final contact height below the nominal hover point.
                        for height in (L.BOWL_TOP + .010, .080, L.LIFT_UP - .001):
                            pp = L.C + [0, 0, height]
                            landing_rot = L.rz(samples[-1][1])
                            q, pe, re = fastik.solve(pp + landing_rot @ gp, landing_rot @ gr, q, base=arm.base)
                            if pe > 0.0005 or re > math.radians(.5) or rim_clearance(q, arm.base) < .006:
                                worst = math.inf
                                break
                    if worst < 0.5 and (choice is None or j6 < choice[0]):
                        choice = (j6, psi, rot)
                if choice is not None:
                    break
            if choice is not None:
                options[n].append(choice)
        pairs = [(left, right) for left in options["left"] for right in options["right"]
                 if left[1] - right[1] == 180.0]
        if pairs:
            left, right = min(pairs, key=lambda pair: (
                abs(pair[0][1] - pair[1][1] - 180.0), max(pair[0][0], pair[1][0])))
            plan = {"left": left, "right": right}
            break
    else:
        raise Stop("no compatible left and right grips for this stand angle")
    c.note(
        "grips",
        psi={n: plan[n][1] for n in plan},
        j6={n: round(plan[n][0], 2) for n in plan},
    )
    return plan


def pick(c):
    pr, rr = c.pose()
    for arm in c.arms.values():
        arm.set_comp(True)
    plan = choose_grips(c, pr, rr)
    c.psi = {n: plan[n][1] for n in plan}
    tg = {}
    for n in c.arms:
        p, _ = grip_point(pr, rr, c.psi[n])
        tg[n] = (p, plan[n][2])
    yield from both(
        *(
            arm.move(
                tg[n][0] + [0, 0, 0.021],
                tg[n][1],
                3.0,
                grip_of(OPEN_HALF),
                f"{n} over grip",
            )
            for n, arm in c.arms.items()
        )
    )
    yield from wait(0.5)
    pr, rr = c.pose()
    for n in c.arms:
        tg[n] = (grip_point(pr, rr, c.psi[n])[0], tg[n][1])
    yield from both(
        *(
            arm.move(tg[n][0], tg[n][1], 2.0, None, f"{n} descend")
            for n, arm in c.arms.items()
        )
    )
    yield from wait(0.5)
    yield from both(
        *(
            arm.set_grip(grip_of(RING_W / 2 - SQUEEZE), 1.0, f"{n} close")
            for n, arm in c.arms.items()
        )
    )
    yield from wait(0.6)
    c.note("gripped")
    yield from both(
        *(
            arm.move(arm.pos + [0, 0, 0.016], arm.rot, 2.0, None, f"{n} lift")
            for n, arm in c.arms.items()
        )
    )
    yield from wait(0.8)
    capture_grasp(c)
    c.note("lifted", yaw=ring_yaw(c.pose()[1]))


def carry(c):
    pr, rr = c.pose()
    y0 = ring_yaw(rr)
    p0 = pr.copy()
    xy1 = np.array([L.C[0] - 0.060, L.C[1]])
    z1 = max(L.C[2] + CARRY_Z + 0.010, L.STAND[2] + 0.015, p0[2] - 0.02)
    yield from track(
        c,
        lambda s: ring_target(
            p0[:2] + s * (xy1 - p0[:2]), p0[2] + s * (z1 - p0[2]), y0 * (1 - s)
        ),
        6.0,
        "carry + turn",
    )
    yield from track(
        c,
        lambda s: ring_target(xy1 + s * (L.C[:2] - xy1), z1, 0.0),
        3.0,
        "over the cathode",
    )
    zh = L.C[2] + L.LIFT_UP + HOVER
    yield from track(
        c, lambda s: ring_target(L.C[:2], z1 + s * (zh - z1), 0.0), 4.0, "into the bowl"
    )
    c.note("hover")


def align(c):
    zh = L.C[2] + L.LIFT_UP + HOVER
    yield from track(
        c,
        lambda s: ring_target(L.C[:2], zh, 0.0),
        1.0,
        "align",
        gain=0.08,
        until=lambda c: slot_error(c)[0] < ALIGN_TOL_M
        and slot_error(c)[1] < 0.0005
        and level(c),
        max_s=8.0,
    )
    ea, eb = slot_error(c)
    c.note("aligned", across_mm=ea * 1e3, along_mm=eb * 1e3)
    if max(ea, eb) > L.LOC_PIN_R:
        raise Stop("ring has not reached the lift-pin alignment window")


# accept a landing when every dowel is within 0.22 mm of its slot centre (inside the scorer's 0.35 mm on-pins
# criterion; the seat probe seated the ring from pure offsets up to 0.34 mm, but a mixed offset jammed from 0.29 mm)
PLACE_TOL_M = 0.00022
PLACE_TRIES = 14
LANDED_SUPPORT = (
    0.8  # a landing counts when the pins carry >= 80 % of the ring's weight
)


def lower_onto_pins(c, z_end_above_tips=-0.0008, speed=0.005, tol=PLACE_TOL_M):
    """Set the ring on the lift-pin tips: re-align 2 mm above them (closed loop on the measured slot offsets), lower at
    ``speed`` until the ring rests on them, then look: if a dowel is more than ``tol`` off its slot centre (the arms'
    joint friction makes them stick and slip by up to about 1 mm), lift it back 2 mm and try again (up to
    PLACE_TRIES times)."""
    z0 = L.C[2] + L.LIFT_UP + HOVER
    z2 = L.C[2] + L.LIFT_UP + 0.002
    z1 = L.C[2] + L.LIFT_UP + z_end_above_tips
    yield from track(
        c,
        lambda s: ring_target(L.C[:2], z0 + s * (z2 - z0), 0.0),
        1.5,
        "down to 2 mm",
        gain=0.04,
    )

    def landed(c):
        """The ring rests on the pin tips and they carry most of its weight (lift-pin contact force), so the hands
        only steady it: the pose measured now is the pose it keeps when the fingers open."""
        import ringkit

        pr, _ = c.pose()
        sup = (
            (ringkit.STATE["detail"].get("contacts") or {}).get("support_by") or [0.0]
        )[0]
        ok = (
            pr[2] - (L.C[2] + L.LIFT_UP) < 0.00015
            and sup >= LANDED_SUPPORT * L.RING_MASS * 9.81
        )
        landed_count[0] = landed_count[0] + 1 if ok else 0
        return landed_count[0] >= 3

    landed_count = [0]
    for attempt in range(PLACE_TRIES):
        landed_count[0] = 0
        for arm in c.arms.values():
            arm.set_comp(
                "hold"
            )  # only the ring-level loop acts in the fine alignment (no joint integral hunting)
        yield from track(
            c,
            lambda s: ring_target(L.C[:2], z2, 0.0),
            1.0,
            f"re-align at 2 mm {attempt}",
            gain=0.06,
            until=settled(),
            max_s=4.0,
            deadband=ALIGN_DEADBAND_M,
        )
        dur = (z2 - z1) / speed
        for arm in c.arms.values():
            arm.set_comp(
                "hold"
            )  # joint offsets frozen while lowering: no integral wind-up against joint stiction
        yield from track(
            c,
            lambda s: ring_target(L.C[:2], z2 + s * (z1 - z2), 0.0),
            dur,
            f"lower onto lift pins {attempt}",
            gain=0.03,
            free_z=True,
            until=landed,
            max_s=dur + 2.0,
        )
        yield from wait(0.3)
        ea, eb = slot_error(c)
        c.note("landed", attempt=attempt, across_mm=ea * 1e3, along_mm=eb * 1e3)
        if max(ea, eb) <= tol:
            break
        if attempt == PLACE_TRIES - 1:
            raise Stop(
                f"not placed within {tol * 1e3:.2f} mm in {PLACE_TRIES} tries (holding the ring)"
            )
        zl = c.pose()[0][2]
        for arm in c.arms.values():
            arm.set_comp(True)
        yield from track(
            c,
            lambda s, zl=zl: ring_target(L.C[:2], zl + s * (z2 - zl), 0.0),
            0.8,
            "lift 2 mm",
            gain=0.03,
        )
    c.note("on lift pins")


def let_go(c):
    """Open the hands where they are, with the ring resting on the pins (no relaxing of the arms first: commanding them
    to where they are releases the landing's downward press, they rise and lift the ring off the pins, and a ring let go
    hanging shifts sideways; any further arm move with the ring on the pins drags it through the joints' stick-slip)."""
    for arm in c.arms.values():
        arm.set_comp("hold")
    yield from both(
        *(
            arm.set_grip(grip_of(OPEN_HALF), 1.0, f"{n} open")
            for n, arm in c.arms.items()
        )
    )
    yield from wait(0.4)
    ea, eb = slot_error(c)
    c.note("opened", across_mm=ea * 1e3, along_mm=eb * 1e3)
    yield from both(
        *(
            arm.move(arm.pos + [0, 0, L.BOWL_TOP - L.LIFT_UP + 0.021], arm.rot, 2.5, None, f"{n} withdraw vertically above rim")
            for n, arm in c.arms.items()
        )
    )
    yield from both(
        *(
            arm.move(
                arm.pos + [-0.10, 0, 0.045], arm.rot, 2.0, None, f"{n} out of the bowl"
            )
            for n, arm in c.arms.items()
        )
    )
    for arm in c.arms.values():
        arm.set_comp(True)
    c.note("released")


def press_key(c, key="down", depth=0.008):
    """Right hand: closed fingertips straight down on the pendant key, about ``depth`` below its top (the arm's own
    stiffness gives the press force), held 0.8 s."""
    arm = c.right
    kc = L.key_center(key)
    top = kc[2] + L.KEY_SIZE[2] / 2
    r = down_rot(np.array([1.0, 0.0]), arm)
    above = np.array([kc[0], kc[1], top + 0.013 + 0.040])
    yield from both(arm.move(above, r, 3.0, 0.0, "to pendant"), c.left.home(3.0))
    yield from arm.move(
        np.array([kc[0], kc[1], top + 0.013 + 0.003]), r, 1.5, None, "onto key"
    )
    yield from arm.move(
        np.array([kc[0], kc[1], top + 0.013 - depth]), r, 1.0, None, "press key"
    )
    yield from wait(0.8)
    c.note("pressed", key=key)
    yield from arm.move(above, r, 1.2, None, "off key")
    yield from arm.home(2.5)


def demo(c):
    yield from wait(0.5)
    yield from pick(c)
    yield from carry(c)
    yield from align(c)
    yield from lower_onto_pins(c)
    yield from let_go(c)
    yield from press_key(c, "down")
    yield from wait(10.0)


def idle(c):
    yield from wait(8.0)


def carry_only(c):
    """Development: pick, carry, align and hold (no placement)."""
    yield from wait(0.5)
    yield from pick(c)
    yield from carry(c)
    yield from align(c)
    yield from wait(2.0)


def carry_to(c, yaw_end=0.0, dxy=(0.0, 0.0)):
    """carry() variant for the negatives: ends over the cathode at yaw ``yaw_end`` and offset ``dxy`` (m)."""
    pr, rr = c.pose()
    y0 = ring_yaw(rr)
    p0 = pr.copy()
    xy1 = np.array([L.C[0] - 0.060, L.C[1]])
    xyc = L.C[:2] + np.asarray(dxy)
    z1 = max(L.C[2] + CARRY_Z + 0.010, L.STAND[2] + 0.015, p0[2] - 0.02)
    yield from track(
        c,
        lambda s: ring_target(
            p0[:2] + s * (xy1 - p0[:2]),
            p0[2] + s * (z1 - p0[2]),
            y0 + s * (yaw_end - y0),
        ),
        6.0,
        "carry",
    )
    yield from track(
        c,
        lambda s: ring_target(xy1 + s * (xyc - xy1), z1, yaw_end),
        3.0,
        "over the cathode",
    )
    zh = L.C[2] + L.LIFT_UP + HOVER
    yield from track(
        c, lambda s: ring_target(xyc, z1 + s * (zh - z1), yaw_end), 4.0, "into the bowl"
    )
    yield from track(c, lambda s: ring_target(xyc, zh, yaw_end), 2.0, "hold", gain=0.08)


def lower_at(c, yaw_end, dxy, z_end_above_tips=-0.0008, speed=0.004):
    z0 = L.C[2] + L.LIFT_UP + HOVER
    z1 = L.C[2] + L.LIFT_UP + z_end_above_tips
    xyc = L.C[:2] + np.asarray(dxy)
    dur = (z0 - z1) / speed

    def landed(c):
        pr, _ = c.pose()
        return pr[2] - (L.C[2] + L.LIFT_UP) < 0.0003

    yield from track(
        c,
        lambda s: ring_target(xyc, z0 + s * (z1 - z0), yaw_end),
        dur,
        "lower onto lift pins",
        gain=0.05,
        free_z=True,
        until=landed,
        max_s=dur + 3.0,
    )
    for arm in c.arms.values():
        arm.set_comp("hold")
    yield from wait(0.5)
    c.note("on lift pins", yaw=yaw_end, dxy=list(dxy))


def neg_unclocked(c):
    """Does not turn the ring to the fitted angle: sets it on the lift pins at its stand angle (or, if the arms cannot
    reach that over the chamber, at 3/4 or 1/2 of it, still >= 5 deg off) and presses RING DOWN. Expected: the slots
    miss the dowels, the ring comes down on the dowel tips (0)."""
    yield from wait(0.5)
    pr, rr = c.pose()
    y0 = ring_yaw(rr)
    for frac in (1.0, 0.75, 0.5):
        c.plan_yaw = (
            frac * y0
        )  # grips planned for a carry that ends at this (unfitted) angle
        try:
            choose_grips(c, pr, rr)
            break
        except Stop:
            continue
    c.note("no-turn target", stand_deg=y0, kept_deg=c.plan_yaw)
    yield from pick(c)
    y = c.plan_yaw
    yield from carry_to(c, yaw_end=y)
    yield from lower_at(c, y, (0.0, 0.0))
    yield from let_go(c)
    yield from press_key(c, "down")
    yield from wait(10.0)


def neg_offset(c):
    """Clocked but 1.5 mm off toward the robot (beyond the dowels' 0.9 mm lead-in): on RING DOWN the ring comes down
    on the dowel tips, not seated (0)."""
    yield from wait(0.5)
    yield from pick(c)
    yield from carry_to(c, 0.0, (-0.0015, 0.0))
    yield from lower_at(c, 0.0, (-0.0015, 0.0))
    yield from let_go(c)
    yield from press_key(c, "down")
    yield from wait(10.0)


def neg_drop(c):
    """Lets go of the ring 15 mm above the lift-pin tips (0: dropped)."""
    yield from wait(0.5)
    yield from pick(c)
    yield from carry(c)
    yield from align(c)
    zr = L.C[2] + L.LIFT_UP + 0.015
    yield from track(
        c,
        lambda s: ring_target(
            L.C[:2],
            L.C[2] + L.LIFT_UP + HOVER + s * (zr - L.C[2] - L.LIFT_UP - HOVER),
            0.0,
        ),
        1.5,
        "raise to 15 mm",
        gain=0.05,
    )
    c.note("drop height")
    yield from let_go(c)
    yield from both(c.left.home(2.5), c.right.home(2.5))
    yield from wait(4.0)


def neg_press(c):
    """Lowered onto the raised lift pins, then pushed 25 mm further down (the hands press the ring onto the pin
    tips): 0 (pressed)."""
    yield from wait(0.5)
    yield from pick(c)
    yield from carry_to(c, 0.0)
    yield from lower_at(c, 0.0, (0.0, 0.0))
    for arm in c.arms.values():
        arm.set_comp("hold")
    for arm in c.arms.values():
        arm.label = "press down"
    yield from both(
        *(
            arm.move(arm.pos + [0, 0, -0.025], arm.rot, 2.0, None, f"{n} press")
            for n, arm in c.arms.items()
        )
    )
    yield from wait(1.0)
    c.note("pressed ring")
    yield from let_go(c)
    yield from both(c.left.home(2.5), c.right.home(2.5))
    yield from wait(3.0)


def neg_not_lowered(c):
    """Placement on the raised lift pins as in the demo, hands out, RING DOWN never pressed: not installed, 0."""
    yield from wait(0.5)
    yield from pick(c)
    yield from carry(c)
    yield from align(c)
    yield from lower_onto_pins(c)
    yield from let_go(c)
    yield from both(c.left.home(2.5), c.right.home(2.5))
    yield from wait(4.0)


def neg_hand_in(c):
    """Full fit, but the left hand is left inside the chamber bowl next to the ring: 0.5."""
    yield from wait(0.5)
    yield from pick(c)
    yield from carry(c)
    yield from align(c)
    yield from lower_onto_pins(c)
    yield from let_go(c)
    arm = c.left
    yield from arm.move(
        arm.pos + [0, 0, -0.045], arm.rot, 2.0, None, "left hand back into the bowl"
    )
    yield from press_key_right_only(c, "down")
    yield from wait(10.0)


def press_key_right_only(c, key="down", depth=0.008):
    arm = c.right
    kc = L.key_center(key)
    top = kc[2] + L.KEY_SIZE[2] / 2
    r = down_rot(np.array([1.0, 0.0]), arm)
    above = np.array([kc[0], kc[1], top + 0.013 + 0.040])
    yield from arm.move(above, r, 3.0, 0.0, "to pendant")
    yield from arm.move(
        np.array([kc[0], kc[1], top + 0.013 + 0.003]), r, 1.5, None, "onto key"
    )
    yield from arm.move(
        np.array([kc[0], kc[1], top + 0.013 - depth]), r, 1.0, None, "press key"
    )
    yield from wait(0.8)
    c.note("pressed", key=key)
    yield from arm.move(above, r, 1.2, None, "off key")
    yield from arm.home(2.5)


def neg_touch_chuck(c):
    """Correct placement, then the left hand's closed fingertips touch the chuck face inside the ring bore: 0."""
    yield from wait(0.5)
    yield from pick(c)
    yield from carry(c)
    yield from align(c)
    yield from lower_onto_pins(c)
    yield from let_go(c)
    arm = c.left
    pr, rr = c.pose()
    tgt = L.C + np.array([-0.10, 0.05, L.PUCK_TOP + 0.013 - 0.002])
    rot = down_rot(np.array([1.0, 0.0]), arm)
    yield from arm.move(tgt + [0, 0, 0.06], rot, 2.5, 0.0, "over the chuck")
    yield from arm.move(tgt, rot, 2.0, None, "touch the chuck face")
    yield from wait(0.5)
    c.note("touched")
    yield from arm.move(tgt + [0, 0, 0.08], rot, 1.5, None, "up")
    yield from press_key_right_only(c, "down")
    yield from wait(10.0)


def replay(c):
    """Open-loop check: plays the 14-D commands recorded in a finished run's trace.jsonl (path in EDGE_REPLAY), step
    by step, without reading anything: what a fixed trajectory scores on another start state."""
    import os

    rows = [json.loads(line) for line in open(os.environ["EDGE_REPLAY"])]
    c.replay = [r["cmd"] for r in rows]
    for _ in range(len(c.replay) + 40):
        yield


STAGES = {
    "demo": demo,
    "idle": idle,
    "carry-only": carry_only,
    "replay": replay,
    "neg-unclocked": neg_unclocked,
    "neg-offset": neg_offset,
    "neg-drop": neg_drop,
    "neg-press": neg_press,
    "neg-not-lowered": neg_not_lowered,
    "neg-hand-in": neg_hand_in,
    "neg-touch-chuck": neg_touch_chuck,
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
    import ringkit

    rt = ringkit.runtime()
    poses = rt.read()
    return dict(
        score=ringkit.STATE["score"],
        setup=rt.setup,
        lift_mm=round(rt.lift_z * 1e3, 4),
        bodies={n: np.round(p * 1000, 4).tolist() for n, (p, r) in poses.items()},
        rots={n: np.round(r, 6).tolist() for n, (p, r) in poses.items()},
    )


def reset_report(stages, resets, results):
    import ringkit

    rows = []
    for i, r in enumerate(resets):
        want = ringkit.authored(
            tuple(r["setup"]) if isinstance(r["setup"], list) else r["setup"]
        )
        dev = max(
            float(np.linalg.norm(np.subtract(r["bodies"][n], want[n][0] * 1000)))
            for n in ("ring", "lifter")
        )
        rdev = float(np.abs(np.subtract(r["rots"]["ring"], want["ring"][1])).max())
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
    ok = all(
        x["score_after_reset"] == 0.0 and x["max_body_dev_from_authored_mm"] < 0.5
        for x in rows
    )
    return dict(
        summary=dict(
            passed=ok,
            trials=len(rows),
            max_body_dev_mm=max(x["max_body_dev_from_authored_mm"] for x in rows),
        ),
        rows=rows,
        reset_states=resets,
    )


def run_trial(emb, out, max_steps, record, stage, t0, seed=None):
    import ringkit
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
        d = ringkit.STATE["detail"]
        row = dict(
            step=steps,
            score=info.get("score"),
            level=ringkit.STATE["level"],
            detail=d,
            labels=[c.left.label, c.right.label],
            nonfinite=ringkit.STATE["nonfinite"],
            cmd=np.round(np.r_[c.left.last_cmd, c.right.last_cmd], 4).tolist(),
            q=np.round(jp, 4).tolist(),
            corr_mm=np.round(c.corr_p * 1000, 3).tolist(),
            corr_r=np.round(c.corr_r, 4).tolist(),
            ring_rv=np.round(_rotvec(c.pose()[1]), 5).tolist(),
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
                f"step {steps} {c.left.label} | {c.right.label} score {info.get('score')} dz {m.get('dz_mm')} "
                f"tilt {m.get('tilt_deg')} clock {m.get('clock_deg')} across {m.get('across_mm')} "
                f"lift {m.get('lift_mm')} tip {m.get('tip_mm')} veto {d.get('veto')} "
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
        scorer=dict(score=ringkit.STATE["score"], level=ringkit.STATE["level"]),
        final=ringkit.STATE["detail"],
        nonfinite=ringkit.STATE["nonfinite"],
        looks=c.log,
        wall_s=round(time.time() - t1, 1),
        wall_total_s=round(time.time() - t0, 1),
        worst_ik_mm=dict(
            left=round(c.left.worst_mm, 2), right=round(c.right.worst_mm, 2)
        ),
        physics_dt=ringkit.PHYSICS_DT,
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
