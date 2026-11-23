"""Scripted full-score demo (and negative variants) for the h4-bulb-clip task in the yam_arena embodiment.

    python demo.py task.py OUT [--max-steps N] [--no-record] [--stage demo|<variant>[,<variant>...]] [--seed N]

Acts ONLY through the 14 joint/gripper commands of ``YamArenaEmbodiment.step``. It reads the bulb's and the clip's
poses through ``h4kit.bench_state()`` (read-only, like a mechanic watching the tabs, the notches and the clip); it
never writes a pose, a joint state or a score. Its joint commands are rate-limited below the CLI's per-step delta guard
and clipped to the URDF joint bounds.

Plan:
1. Right hand: read the bulb's roll in its holder (tab pattern) and grip its cap from above so that, once turned, the
   fingers close vertically and the tabs face their notches; lift it out (glass never touched).
2. Turn it to point into the lamp and bring it behind the bulb boss; closed loop on the measured bulb pose: centre and
   roll it 3.5 mm behind the seat, then keep a light axial load while finding the slots.
3. Right hand releases the bulb and withdraws. Left hand swings the wire over the flange.
4. Flex the wire sideways, press its operating end behind the fixed bracket, then release and withdraw.
Both arms add the remaining joint error back to their commands (slow integral on their own joint readings) in free
moves, because the joint PD sags under load (pattern reused from earlier demos).
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
SQUEEZE = 0.0020  # fingers commanded 2 mm inside the cap's faces: about 10 N per finger (5000 N/m finger drive)
COMP_GAIN = 0.08
RATE = 0.8  # fraction of the CLI delta guard (0.05 x joint range per step) the script may use


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
        self.joint_step = np.zeros(6)
        self.comp = True
        self.label = ""
        self.worst_mm = 0.0
        _, _, lo, hi, _ = fastik._chain()
        self.lo, self.hi = lo, hi
        self.last_cmd = None

    @property
    def cmd(self):
        drive = self.q + self.offset
        if self.comp == "insertion":
            # The YAM drives have a Coulomb dead band: friction/stiffness is
            # 0.00375 rad at the shoulder and 0.006 rad at the wrist. Overcome
            # it gradually, braking on observed motion. Only position commands change.
            error = self.q - self.measured
            friction = np.array([0.00375]*3 + [0.006]*3)
            assist = 0.4 * error + friction * np.tanh(error / 0.0025) - 0.5 * self.joint_step
            drive += np.clip(assist, -0.006, 0.006)
        c = np.r_[np.clip(drive, self.lo, self.hi), self.grip]
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
        new_q = np.asarray(q[:6], float)
        self.joint_step = new_q - self.measured
        self.measured = new_q
        self.measured_grip = float(q[6])
        if self.comp in ("hold", "insertion"):
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

    def best_ik(self, pos, rot, tol_mm=0.5, tol_deg=0.3):
        """IK from many seeds; the solution nearest the current joint angles among those that reach the pose."""
        from yam_arena.constants import READY

        seeds = [self.q, np.array(READY, float)] + [
            np.array(v, float)
            for v in (
                [0.0, 1.6, 1.4, -1.2, 0.0, 0.0],
                [0.0, 2.0, 2.0, -0.5, 0.0, 0.0],
                [0.0, 1.2, 0.6, 0.3, 0.0, 0.0],
                [0.0, 2.4, 1.6, 0.6, 0.0, 0.0],
                [0.5, 1.5, 1.2, 0.0, 1.0, 1.5],
                [-0.5, 1.5, 1.2, 0.0, -1.0, -1.5],
                [0.3, 1.8, 1.6, -0.4, 0.8, -1.5],
                [-0.3, 1.8, 1.6, -0.4, -0.8, 1.5],
            )
        ]
        best = None
        for sd in seeds:
            q, pe, re = fastik.solve(pos, rot, sd, base=self.base, iters=300)
            if pe * 1e3 > tol_mm or math.degrees(re) > tol_deg:
                continue
            cost = float(np.abs(q - self.q).sum())
            if best is None or cost < best[0]:
                best = (cost, q)
        if best is None:
            raise Stop(f"{self.name}: no IK solution for {self.label}")
        return best[1]

    def move_joint(self, q1, duration, grip=None, label=""):
        """Joint-space move (smoothstep) to q1; the Cartesian path in between is not controlled."""
        from yam_arena import ik

        self.label = label or self.label
        q0, g0 = self.q.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.q = q0 + s * (np.asarray(q1) - q0)
            self.grip = g0 + s * (g1 - g0)
            self.pos, self.rot = fastik.tcp_pose(self.q, self.base)
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

    def pose(self, name="bulb"):
        import h4kit

        return h4kit.bench_state()[name]

    def clip(self):
        import h4kit

        return h4kit.runtime().clip_state()

    def detail(self):
        import h4kit

        return h4kit.STATE["detail"]

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




# --------------------------------------------------------------------------------------------------- geometry helpers
def frame(z_axis, x_axis):
    """Rotation with the given approach (z) and closing (x) directions (x is orthogonalised against z)."""
    z = np.asarray(z_axis, float)
    z /= np.linalg.norm(z)
    x = np.asarray(x_axis, float) - (np.asarray(x_axis, float) @ z) * z
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def capture_grasp(c, arm="right"):
    """Freeze the bulb -> TCP transform of the holding hand (measured with the bulb held still)."""
    pb, rb = c.pose()
    pt, rt = c.arms[arm].measured_tcp()
    c.G[arm] = (rb.T @ (pt - pb), rb.T @ rt)
    c.corr_p, c.corr_r = np.zeros(3), np.zeros(3)


def tcp_for_bulb(c, pb, rb, arm="right"):
    gp, gr = c.G[arm]
    return pb + rb @ gp, rb @ gr


def bulb_err(c):
    """Measured bulb pose error from the fitted seat: (radial m, roll deg, tilt deg, axial m)."""
    import scoring

    pb, rb = c.pose()
    ax, rad, tilt, roll = scoring.bulb_in_h(pb, rb)
    return rad, roll, tilt, ax


def track_bulb(c, target_fn, duration, label, gain=0.06, rot_gain=None, free_x=False, until=None, max_s=None,
               deadband=0.0, feedback=0.0):
    """Closed loop on the bulb (right hand): each step command the TCP that puts the bulb at target_fn(s) through the
    frozen grasp transform, plus slow integral corrections of the measured bulb position / orientation error."""
    from yam_arena import ik

    arm = c.right
    n = max(1, round(duration * CONTROL_HZ))
    nmax = n if max_s is None else max(n, round(max_s * CONTROL_HZ))
    for k in range(1, nmax + 1):
        pt, rt = target_fn(ik.smoothstep(min(1.0, k / n)))
        pm, rm = c.pose()
        err = pt - pm
        if free_x:
            err[0] = 0.0
        if deadband > 0:
            err = np.where(np.abs(err) < deadband, 0.0, err)
        c.corr_p = np.clip(c.corr_p + gain * err, -0.010, 0.010)
        e_r = _rotvec(rt @ rm.T)
        if deadband > 0:
            e_r = np.where(np.abs(e_r) < deadband / 0.02, 0.0, e_r)
        c.corr_r = np.clip(c.corr_r + (gain if rot_gain is None else rot_gain) * e_r, -0.15, 0.15)
        rc = _rotm(c.corr_r + feedback * e_r)
        gp, gr = c.G["right"]
        arm.label = label
        arm.set_pose(pt + c.corr_p + feedback * err + rc @ rt @ gp, rc @ rt @ gr)
        yield
        if until is not None and k >= n and until(c):
            return True
    return until is None


# --------------------------------------------------------------------------------------------------- right hand: bulb
GRIP_TIP_ABOVE = 0.0040  # pad tips 4 mm above the flange's back face (cap body grip)
OPEN_HALF = L.BASE_R + 0.012
PRE_X = -0.060  # pre-insertion: bulb reference plane 60 mm behind the seat (glass tip clear of the boss)
ALIGN_X = -0.0035  # alignment stop: flange 3.5 mm behind the seat
SEAT_PUSH_M = 0.0010  # bulb target 1.0 mm past the seat while seating (arm stiffness gives the press)
FINAL_ROLL_SIGNS = (1.0, -1.0)
PRESENT_PITCH_DEG = 45.0


def pick_closing_dir(c):
    """Closing direction for the top-down grasp, read from the bulb's measured roll in its holder: the cap is gripped so
    that, once the bulb points into the lamp, the fingers close along the vertical (the YAM's comfortable wrist roll),
    and the tab pattern then matches the notches. Two choices (fingers either way round); the one with the least
    wrist-roll travel at the pick wins."""
    pb, rb = c.pose()
    if getattr(c, "fixed_grasp", False):
        c.note("pick dir", sign="fixed (world x)")
        return frame((0.0, 0.0, -1.0), (1.0, 0.0, 0.0)), 1.0
    best = None
    for s in FINAL_ROLL_SIGNS:
        cdir = rb @ np.array([0.0, 0.0, s])  # bulb z axis (horizontal while it hangs glass down)
        cdir[2] = 0.0
        cdir /= np.linalg.norm(cdir)
        rot = frame((0.0, 0.0, -1.0), cdir)
        q, pe, re = c.right.solve(pb + np.array([0, 0, 0.05]), rot)
        # final pose check (approach +x, closing s*z) from the same seed
        fin = frame((1.0, 0.0, 0.0), (0.0, 0.0, s))
        q2, pe2, re2 = fastik.solve(L.H + np.array([PRE_X - 0.016, 0.0, 0.0]), fin, q, base=c.right.base, iters=200)
        cost = abs(q[5]) + 3.0 * (pe2 * 1e3 + math.degrees(re2) > 1.0) + abs(q2[5] - q[5]) * 0.2
        if best is None or cost < best[0]:
            best = (cost, rot, s)
    c.note("pick dir", sign=best[2])
    return best[1], best[2]


def pick_bulb(c):
    arm = c.right
    arm.set_comp(True)
    rot, sgn = pick_closing_dir(c)
    c.final_sign = sgn
    pb, rb = c.pose()
    top = pb + np.array([0.0, 0.0, L.FLANGE_T + GRIP_TIP_ABOVE + 0.013])  # TCP over the cap
    yield from arm.move(top + [0, 0, 0.06], rot, 3.0, grip_of(OPEN_HALF), "right over bulb")
    yield from wait(0.3)
    pb, rb = c.pose()
    top = pb + np.array([0.0, 0.0, L.FLANGE_T + GRIP_TIP_ABOVE + 0.013])
    yield from arm.move(top, rot, 2.0, None, "right descend")
    yield from wait(0.3)
    yield from arm.set_grip(grip_of(L.BASE_R - SQUEEZE), 1.0, "right close")
    yield from wait(0.5)
    c.note("gripped")
    yield from arm.move(arm.pos + [0, 0, 0.075], arm.rot, 2.0, None, "right lift")
    yield from wait(0.5)
    capture_grasp(c)
    c.note("lifted", G=np.round(c.G["right"][0] * 1e3, 2).tolist())


def present_bulb(c):
    """Turn the bulb from glass-down to pointing into the lamp, roll fitted, 40 mm short of the pre-insertion pose
    (a joint-space move: the wrist takes its own way round), then slide it to the pre-insertion pose behind the boss."""
    p1 = L.H + np.array([PRE_X, 0.0, 0.0])
    p0 = p1 + np.array([-0.040, 0.0, 0.030])
    r1 = np.eye(3)
    if getattr(c, "fixed_grasp", False):
        # no look at the tabs: the hand goes to its comfortable pose (fingers closing vertically); the bulb's roll is
        # whatever the fixed grasp gives
        gp, gr = c.G["right"]
        r1 = frame((1.0, 0.0, 0.0), (0.0, 0.0, 1.0)) @ gr.T
    c.target_rot = r1
    arm = c.right
    arm.label = "turn bulb"
    q1 = arm.best_ik(*tcp_for_bulb(c, p0, r1))
    yield from arm.move_joint(q1, 4.0, label="turn bulb")
    yield from wait(0.4)
    yield from track_bulb(c, lambda s: (p0 + s * (p1 - p0), r1), 2.0, "to the lamp", gain=0.03)
    c.note("presented", err=list(bulb_err(c)))


def insert_bulb(c, tries=4):
    """Centre the cap before entering the narrow slot; guide insertion from the measured bulb pose."""
    import scoring

    p0 = L.H + np.array([PRE_X, 0.0, 0.0])
    pa = L.H + np.array([ALIGN_X, 0.0, 0.0])
    r0 = c.target_rot
    fixed_grasp = getattr(c, "fixed_grasp", False)
    if fixed_grasp:
        tries = 1
    yield from track_bulb(c, lambda s: (p0 + s * (pa - p0), r0), 3.0, "slide in", gain=0.04)
    # Use one slow bulb-pose integral during precision work. A simultaneous joint
    # integral stores extra force when the pilot contacts the bore.
    c.right.set_comp("insertion")

    def at_entry(c):
        radial, roll, tilt, axial = bulb_err(c)
        target_roll = scoring.bulb_in_h(L.H, r0)[3]
        return (radial <= 0.00045 and tilt <= 0.7 and abs(scoring.wrap180(roll-target_roll)) <= 0.8
                and abs(axial-ALIGN_X) <= 0.00065)

    for attempt in range(tries):
        ready = yield from track_bulb(
            c, lambda _: (pa, r0), 0.5, f"align {attempt}", gain=0.032, feedback=0.40,
            until=at_entry, max_s=7.0, deadband=0.000010)
        c.note("alignment", attempt=attempt, ready=ready, err=list(bulb_err(c)))
        if not ready:
            continue

        feed_x = bulb_err(c)[3]
        loaded_steps = 0
        lateral_scale = 0.0005

        def advance(_):
            nonlocal feed_x, loaded_steps
            pb, _ = c.pose()
            axial = pb[0] - L.H[0]
            radial = np.linalg.norm((pb-L.H)[1:])
            sample = c.detail().get("contacts") or {}
            load = sample.get("housing_on_bulb_n", 0)
            # Keep the bulb against the entry face even when dry friction briefly
            # moves the arm off centre. Only excessive force or a gross positioning
            # error calls for unloading. This lets the physical rim guide the index.
            if load > 3 or radial > 0.0025:
                feed_x = max(ALIGN_X, min(feed_x, axial+0.0004)-0.0012*DT)
            elif load < 1.4:
                feed_x = min(SEAT_PUSH_M, axial + 0.0028, feed_x + 0.0017 * DT)
            # A small lateral search under light pressure releases a caught lug.
            # Once the flange is inside the locating wall, complete the axial stroke.
            if load > 0.12 and axial < L.RIM_X[0]+0.00025:
                loaded_steps += 1
            target = L.H + [feed_x, 0.0, 0.0]
            if loaded_steps and axial < L.RIM_X[0]+0.00025:
                phase = loaded_steps * DT * 2 * math.pi * 0.48
                amplitude = min(1.0, loaded_steps/18) * lateral_scale
                target[1] += amplitude * math.sin(phase)
                target[2] += amplitude * math.sin(phase*0.73)
            return target, r0

        def seated(c):
            rad, roll, tilt, ax = bulb_err(c)
            return abs(ax) < 0.00035 and tilt < 0.9 and rad <= scoring.SEAT_RADIAL_M and abs(roll) <= scoring.SEAT_ROLL_DEG

        arrived = yield from track_bulb(
            c, advance, 0.25, f"seat {attempt}", gain=0.032, feedback=0.40,
            free_x=True, until=seated, max_s=17.0, deadband=0.000010)
        c.note("seat try", attempt=attempt, err=list(bulb_err(c)))
        if arrived:
            c.note("seated")
            return
        if fixed_grasp:
            return
        pb, _ = c.pose()
        yield from track_bulb(c, lambda s: (pb + s * (pa-pb), r0), 1.5, "back off", gain=0.02)
    raise Stop("bulb not aligned and seated within insertion retries")


def release_bulb(c):
    """Let go of the seated bulb without disturbing it: take the push off (bulb target back on the seat), relax the arm
    where it is, open the fingers slowly by a few millimetres, draw straight back off the cap, then out and away."""
    arm = c.right
    # Keep the friction compensation through the last loaded motion. Freeze the
    # last joint correction before opening the fingers to avoid a release jump.
    ps = L.H.copy()
    r0 = getattr(c, "target_rot", np.eye(3))
    yield from track_bulb(c, lambda s: (ps, r0), 0.6, "right ease push", gain=0.0)
    yield from wait(0.3)
    if arm.comp == "insertion":
        arm.offset = arm.cmd[:6] - arm.q
    arm.set_comp("hold")
    yield from arm.set_grip(grip_of(L.BASE_R + 0.003), 1.5, "right open")
    yield from wait(0.3)
    c.note("right opened", err=list(bulb_err(c)))
    yield from arm.move(arm.pos + [-0.012, 0.0, 0.0], arm.rot, 1.0, None, "right back")
    yield from arm.set_grip(grip_of(L.BASE_R + 0.010), 0.5, "right open more")
    # off the cap's end to the robot's right first (the open clip lies on the left), then back and away
    yield from arm.move(arm.pos + [-0.030, -0.030, 0.0], arm.rot, 1.2, None, "right back right")
    arm.set_comp(True)
    yield from arm.move(arm.pos + [-0.05, -0.06, 0.04], arm.rot, 1.5, None, "right away")
    yield from arm.home(2.0, "right home")


# --------------------------------------------------------------------------------------------------- left hand: clip
PUSH_PITCH_DEG = 10.0  # approach tilted 10 deg down from the push direction (one hand pose reaches 100..0 deg)
PUSH_PITCH_OPEN_DEG = 25.0  # ... and 25 deg while the bail is wider open than 100 deg (reach)


def push_pitch(phi, sign=1.0):
    if sign < 0:
        return 25.0  # the other hand way round (final part of the swing and the press): 25 deg
    s = float(np.clip((phi - 90.0) / 15.0, 0.0, 1.0))
    return PUSH_PITCH_DEG + s * (PUSH_PITCH_OPEN_DEG - PUSH_PITCH_DEG)
# fingertips centred 4 mm outside the bight's radius (a 12 mm offset lost contact with the bight in d19 / d21)
PUSH_RADIAL_OPEN, PUSH_RADIAL_CLOSE = 0.004, 0.004


def push_radial(phi):
    s = float(np.clip((phi - 12.0) / 13.0, 0.0, 1.0))
    return PUSH_RADIAL_CLOSE + s * (PUSH_RADIAL_OPEN - PUSH_RADIAL_CLOSE)
# Target the short upper-right operating end. The two fingertips straddle the fixed lip.
PUSH_Z_OPEN, PUSH_Z_CLOSE, PUSH_Z_PHI = L.BIGHT_Z, L.BIGHT_Z, (12.0, 25.0)
PUSH_HALF_GAP = 0.0032


def push_tip_z(phi):
    lo, hi = PUSH_Z_PHI
    s = float(np.clip((phi - lo) / (hi - lo), 0.0, 1.0))
    return PUSH_Z_CLOSE + s * (PUSH_Z_OPEN - PUSH_Z_CLOSE)


def clip_push_pose(phi, c, gap=0.0, back=0.0):
    """TCP pose that puts the left hand's closed fingertips (end face) against the trailing side of the bail at angle
    phi, pushing along the bight's direction of motion; ``back`` (m) withdraws the hand along its approach axis."""
    a = math.radians(phi)
    t = np.array([math.cos(a), -math.sin(a), 0.0])  # bight motion for decreasing phi
    u = np.array([-math.sin(a), -math.cos(a), 0.0])  # along the bail, hinge -> bight
    b = math.radians(push_pitch(phi, c.clip_sign))
    app = t * math.cos(b) - np.array([0.0, 0.0, 1.0]) * math.sin(b)
    hinge = L.H + L.hinge_point()
    tip = hinge + (L.CLIP_LEN + push_radial(phi)) * u + np.array([0.0, 0.0, push_tip_z(phi)]) - t * (L.WIRE_R + gap)
    yax = u * c.clip_sign
    xax = np.cross(yax, app)
    rot = np.column_stack([xax, yax, app])
    tcp = tip - app * (0.013 + back)
    return tcp, rot


def clip_approach(c):
    """Left hand, fingers closed, down beside the open bail (trailing side)."""
    phi0, _ = c.clip()
    c.clip_sign = 1.0  # first half of the swing: finger width axis along the bail (reach of the left arm)
    arm = c.left
    arm.set_comp(True)
    p, r = clip_push_pose(min(phi0 + 2.0, 107.0), c, gap=0.004, back=0.03)
    arm.label = "left over clip"
    q1 = arm.best_ik(p + [0, 0, 0.01], r)
    yield from arm.move_joint(q1, 3.0, grip_of(PUSH_HALF_GAP), "left over clip")
    p, r = clip_push_pose(min(phi0 + 2.0, 107.0), c, gap=0.004)
    yield from arm.move(p, r, 1.5, None, "left down by clip")
    c.note("left at clip", phi=phi0)


SWING_LEAD_DEG = 3.0


def regrip_clip(c, sign):
    """Lift the left hand off the bail along its approach axis and set it down again on the trailing side with the
    hand turned the other way round (the second half of the swing is out of reach otherwise)."""
    arm = c.left
    phi, _ = c.clip()
    p, r = clip_push_pose(phi, c, gap=0.004, back=0.02)
    yield from arm.move(p + [0, 0, 0.01], r, 0.8, None, "left off clip")
    arm.label = "left turn hand"
    q1 = None
    for s in (sign, -sign):  # the requested way round, else the other one if that is out of reach here
        c.clip_sign = s
        p, r = clip_push_pose(phi + 4.0, c, gap=0.004, back=0.02)
        try:
            q1 = arm.best_ik(p, r)
            break
        except Stop:
            continue
    if q1 is None:
        raise Stop(f"no hand pose to set down on the bail at {phi:.1f} deg")
    yield from arm.move_joint(q1, 2.0, label="left turn hand")
    p, r = clip_push_pose(phi + 4.0, c, gap=0.003)
    yield from arm.move(p, r, 1.0, None, "left back on clip")


def swing_clip(c, phi_to, speed_dps=20.0, label="swing clip", max_s=14.0):
    """Push the bail round its hinge to phi_to: the hand's target angle sweeps at speed_dps but never leads the bail's
    measured angle by more than SWING_LEAD_DEG (the hand waits for the bail if the bail stops). If the bail springs
    back open by more than 4 deg (the hand slipped off the bight), stop: the caller sets the hand down again."""
    arm = c.left
    phi, _ = c.clip()
    target, low = phi, phi
    for _ in range(int(max_s * CONTROL_HZ)):
        phi, _ = c.clip()
        low = min(low, phi)
        if phi > low + 4.0:
            c.note("bail slipped", phi=phi, low=low)
            break
        target = max(phi_to - 1.0, phi - SWING_LEAD_DEG, target - speed_dps / CONTROL_HZ)
        p, r = clip_push_pose(target, c, gap=-0.0005)
        arm.label = label
        try:
            arm.set_pose(p, r)
        except Stop as e:
            c.note("swing pose out of reach", phi=phi, err=str(e))
            break
        yield
        if phi <= phi_to + 0.5:
            break
    c.note("clip at", phi=c.clip()[0])


def clip_over(c):
    """Hand off while the bail is clear of the flange. The outer edge of the pads reaches the rear shroud below this arc."""
    yield from swing_clip(c, 40.0)
    yield from regrip_clip(c, -1.0)
    yield from swing_clip(c, 22.0)
    phi, _ = c.clip()
    if not 8.0 <= phi <= 24.0:
        raise Stop(f"bail outside the transfer arc at {phi:.2f} degrees")
    # Hold the observed transfer pose briefly before drawing the finger pads back.
    yield from wait(0.5)
    arm = c.left
    yield from arm.move(arm.pos - arm.rot[:, 2]*0.025, arm.rot, 0.65, None, "withdraw after sweep")
    c.note("transfer to operating end", contacts=c.detail().get("contacts"))


def wire_pose(phi, lateral, grip, sign, depth):
    """TCP on the desired distal wire segment. Each target lies on the authored polyline."""
    path = L.clip_wire_path()
    if grip == "axial":
        # Straight central section, immediately below the turn.
        point = path[5] + (path[6] - path[5]) * (0.0005 / 0.015)
        roll = 0.0
    else:
        # Angled segment leading out to the vertical operating end.
        point = path[4] + 0.2 * (path[5] - path[4])
        roll = math.radians(20)
    rot = L.clip_rot(phi)
    pitch = math.radians(24)
    approach = rot[:, 0]*math.cos(pitch) - [0., 0., math.sin(pitch)]
    closing = sign * (rot[:, 1]*math.cos(roll) + [0., 0., math.sin(roll)])
    world = L.H + L.hinge_point() + rot @ point + [0., lateral, 0.]
    return world + approach*(depth - 0.013), frame(approach, closing)


def take_wire(c, grip):
    arm = c.left
    phi, displacement = c.clip()
    arm.set_comp(True)
    arm.label = "approach distal wire"
    for sign in (-1, 1):
        p, r = wire_pose(phi, -displacement, grip, sign, -0.020)
        try:
            q = arm.best_ik(p, r)
            break
        except Stop:
            continue
    else:
        raise Stop("no reachable approach to the operating end")
    yield from arm.move_joint(q, 1.5, grip_of(0.004), arm.label)
    for step in range(35):
        phi, displacement = c.clip()
        depth = -0.020 + 0.025 * min((step + 1) / 30, 1)
        arm.label = "straddle distal wire"
        arm.set_pose(*wire_pose(phi, -displacement, grip, sign, depth))
        yield
    for step in range(24):
        phi, displacement = c.clip()
        arm.label = "grip distal wire"
        arm.grip = grip_of(0.004 * max(0, 1 - (step + 1) / 20))
        arm.set_pose(*wire_pose(phi, -displacement, grip, sign, 0.005))
        yield
    return sign


def shoe_pose(phi,side,axial=0.0,roll=0.0):
    import retainer_tool as tool
    path=L.clip_wire_path()
    r=L.clip_rot(phi)
    center=L.H+L.hinge_point()+r@(path[5]*.50+path[6]*.50)+np.array([0.,side,0.])
    u,v=L.clip_rot(0.)[:,:2].T
    orientation=np.column_stack([-v,-u,np.cross(v,u)])
    tcp=center+axial*u+orientation@(np.array([0.,0.,.134])-tool.TIP)
    return tcp,orientation

def latch_clip(c, axial_only=False):
    import retainer_tool as tool
    arm=c.left
    phi,cy=c.clip()
    # Give the stock YAM pads room to rotate away from the opening.
    pos,rot=arm.measured_tcp()
    yield from arm.move(pos+[-.045,0,.028],rot,1.1,grip_of(.002),'bring stock pads clear')
    phi,cy=c.clip()
    pos,rot=shoe_pose(phi,-cy-.001,-.030)
    q=arm.best_ik(pos,rot)
    yield from arm.move_joint(q,2.,grip_of(.002),'bring collar shoe behind wire')
    phi,cy=c.clip()
    yield from arm.move(*shoe_pose(phi,-cy-.0003,-.0025),1.0,None,'align steel shoe')
    yield from wait(.3)
    bias=np.clip(arm.last_cmd[:6]-arm.measured,-.07,.07)
    arm.set_comp('hold')
    motor_k=np.array([80.,80.,80.,10.,10.,10.])

    phase='swing with shoe'
    a=max(4.1,phi)
    goal_side=0.
    force=np.zeros(3);trim=np.zeros(2);stable=0
    for i in range(460):
        phi,disp=c.clip();side=-disp
        f=np.asarray(c.detail()['contacts']['robot_on_clip_force_n'])
        if phase=='swing with shoe':
            a=max(3.9,a-.20)
            f_req=[1.5,0.]
            if a<=4 and phi<5.4:phase='side-load shoe'
        elif phase=='side-load shoe' and axial_only:
            # Use the same shoe and the same approach. Only the straight station-x stroke is applied here.
            p,r=arm.pos.copy(),arm.rot.copy()
            c.note('shoe axial baseline',contacts=c.detail()['contacts'])
            yield from arm.move(p+[.003,0.,0.],r,1.25,None,'axial press only')
            yield from wait(.60)
            c.note('shoe axial stroke complete',contacts=c.detail()['contacts'])
            break
        elif phase=='side-load shoe':
            a=max(3.75,a-.025)
            goal_side=min(.0032,goal_side+.000045)
            f_req=[4.,8.]
            if side>.00195 and phi<5.:phase='press behind retainer'
        elif phase=='press behind retainer':
            a=max(-.35,a-.04)
            f_req=[18.,8.]
            if phi<.45:phase='return wire under retainer'
        else:
            a=-.3;goal_side=max(-.0003,goal_side-.00004)
            f_req=[18.,0.]
            stable=stable+1 if side<.00045 and abs(phi)<.41 else 0
            if stable>=4:break
        force+=np.clip(np.asarray(f_req+[0.])-force,-.18,.18)
        if phase=='return wire under retainer':
            trim[1]=max(-.0005,trim[1]-.00004)
        else:
            trim[1]=np.clip(trim[1]+.000005*(f_req[1]-f[1]),-.0007,.0015)
        trim[0]=np.clip(trim[0]+.000004*(f_req[0]-f[0]),-.0008,.0015)
        pos,rot=shoe_pose(a,goal_side+trim[1],trim[0])
        arm.label=phase;arm.set_pose(pos,rot)
        p,r,axes=fastik.fk(arm.measured,arm.base);tip=p+r@(tool.TIP-[0.,0.,.134])
        jac=np.stack([np.cross(ax,tip-o) for ax,o in axes],axis=1)
        arm.offset=bias+np.clip((jac.T@force)*1./motor_k,-.24,.24)
        yield
    c.note('shoe before unload',contacts=c.detail()['contacts'])
    pos,rot=arm.measured_tcp();q=arm.measured.copy();warm=arm.last_cmd[:6]-q
    arm.q=q;arm.pos,arm.rot=pos,rot
    for k in range(35):
        s=(k+1)/35
        arm.offset=warm*(1-s)+bias*s
        arm.label='unload and retract shoe'
        arm.set_pose(pos+[-.026*s,-.0015*s,0.],rot);yield
    # Continue until the external shoe and all unmodified stock pads have cleared both objects.
    for reach in (.065,.080,.095,.110,.125):
        yield from arm.move(pos+[-reach,-.005,.018],rot,.55,None,'retract shoe clear of bulb')
        m=c.detail().get('measure') or {}
        if min(m.get('tip_bulb_mm',0),m.get('tip_clip_mm',0))>36:
            break
    yield from wait(2.)


def demo(c):
    yield from wait(0.5)
    yield from pick_bulb(c)
    yield from present_bulb(c)
    yield from insert_bulb(c)
    yield from release_bulb(c)
    yield from clip_approach(c)
    yield from clip_over(c)
    yield from latch_clip(c)
    yield from wait(4.0)


def idle(c):
    yield from wait(8.0)


def hold(c):
    """Do nothing (both arms at READY) for the whole step limit: the full-length no-op."""
    yield from wait(10000.0)


def neg_noroll(c):
    """Ignores the tab pattern: grips the cap with the fingers closing along world x (a fixed grasp), presents and
    pushes the bulb in exactly as the demo, then clips. Expected: unless the start roll happens to fit, a tab lands on
    the rim, the bulb is not seated, the clip cannot be hooked: 0."""
    c.fixed_grasp = True
    yield from demo(c)


def neg_unhooked(c):
    """Hold the operating wire against the face of the fixed tab, with the bulb seated."""
    yield from wait(0.5)
    yield from pick_bulb(c)
    yield from present_bulb(c)
    yield from insert_bulb(c)
    yield from release_bulb(c)
    yield from clip_approach(c)
    yield from clip_over(c)
    sign = yield from take_wire(c, "axial")
    arm = c.left
    for _ in range(70):
        phi, _ = c.clip()
        arm.label = "place axial pads"
        arm.set_pose(*wire_pose(max(4.3, phi - 1.0), 0., "axial", sign, 0.005))
        yield
    arm.set_comp("hold")
    # Start and end in the same command frame. The loaded TCP can lag its target by millimetres.
    p, r = arm.pos.copy(), arm.rot.copy()
    c.note("before axial press", contacts=c.detail().get("contacts"))
    yield from arm.move(p + [0.003, 0., 0.], r, 1.25, None, "axial press only")
    yield from wait(0.6)
    c.note("axial press complete", contacts=c.detail().get("contacts"))


def neg_axial_press(c):
    yield from wait(.5)
    yield from pick_bulb(c)
    yield from present_bulb(c)
    yield from insert_bulb(c)
    yield from release_bulb(c)
    yield from clip_approach(c)
    yield from clip_over(c)
    yield from latch_clip(c, axial_only=True)
    yield from wait(2.)

def neg_noclip(c):
    """Bulb seated, then let go and both hands home without the clip: 0.25 if it stays seated, else 0."""
    yield from wait(0.5)
    yield from pick_bulb(c)
    yield from present_bulb(c)
    yield from insert_bulb(c)
    yield from release_bulb(c)
    yield from both(c.left.home(2.5), c.right.home(2.5))
    yield from wait(5.0)


def neg_glass(c):
    """Demo, but while the bulb is presented behind the lamp the left hand's closed fingertips touch its glass: 0."""
    yield from wait(0.5)
    yield from pick_bulb(c)
    yield from present_bulb(c)
    pb, rb = c.pose()
    tip = pb + rb @ np.array([0.030, 0.0, L.GLASS_R + 0.0005])
    arm = c.left
    arm.label = "left over glass"
    choice = None
    for pitch in (90.0, 60.0, 45.0, 30.0):  # approach from above, tilted toward -y (the left arm reaches across)
        a = math.radians(pitch)
        app = np.array([0.0, -math.cos(a), -math.sin(a)])
        rot = frame(app, (1.0, 0.0, 0.0))
        try:
            q1 = arm.best_ik(tip - app * (0.013 + 0.05), rot)
            arm.best_ik(tip - app * (0.013 - 0.002), rot)
        except Stop:
            continue
        choice = (app, rot, q1)
        break
    if choice is None:
        raise Stop("no reachable pose to touch the glass")
    app, rot, q1 = choice
    yield from arm.move_joint(q1, 3.0, 0.0, "left over glass")
    yield from arm.move(tip - app * (0.013 - 0.002), rot, 2.0, None, "touch glass")
    yield from wait(0.5)
    c.note("touched glass")
    yield from arm.move(tip - app * 0.08, rot, 1.0, None, "left up")
    yield from insert_bulb(c)
    yield from release_bulb(c)
    yield from clip_approach(c)
    yield from clip_over(c)
    yield from latch_clip(c)
    yield from wait(4.0)


def insert_only(c):
    yield from wait(0.5)
    yield from pick_bulb(c)
    yield from present_bulb(c)
    yield from insert_bulb(c)
    yield from wait(2.0)
    yield from release_bulb(c)
    yield from wait(4.0)


def replay(c):
    """Open-loop check: plays the 14-D commands recorded in a finished run's trace.jsonl (path in H4_REPLAY), step by
    step, without reading anything: what a fixed trajectory scores on another start state."""
    import os

    rows = [json.loads(line) for line in open(os.environ["H4_REPLAY"])]
    c.replay = [r["cmd"] for r in rows]
    for _ in range(len(c.replay) + 40):
        yield


STAGES = {
    "demo": demo,
    "idle": idle,
    "hold": hold,
    "insert-only": insert_only,
    "neg-noroll": neg_noroll,
    "neg-unhooked": neg_unhooked,
    "neg-axial-press": neg_axial_press,
    "neg-noclip": neg_noclip,
    "neg-glass": neg_glass,
    "replay": replay,
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
    max_steps=2400,
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
    import h4kit

    rt = h4kit.runtime()
    poses = rt.read()
    return dict(
        score=h4kit.STATE["score"],
        setup=rt.setup,
        clip_deg_catch_m=list(rt.clip_state()),
        bodies={n: np.round(p * 1000, 4).tolist() for n, (p, r) in poses.items()},
        rots={n: np.round(r, 6).tolist() for n, (p, r) in poses.items()},
    )


def reset_report(stages, resets, results):
    import h4kit

    rows = []
    for i, r in enumerate(resets):
        want = h4kit.authored(
            tuple(r["setup"]) if isinstance(r["setup"], list) else r["setup"]
        )
        dev = max(
            float(np.linalg.norm(np.subtract(r["bodies"][n], want[n][0] * 1000)))
            for n in ("bulb", "glass", "clip", "catch", "holder")
        )
        rdev = max(
            float(np.abs(np.subtract(r["rots"][n], want[n][1])).max())
            for n in ("bulb", "clip")
        )
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
    import h4kit
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
        d = h4kit.STATE["detail"]
        row = dict(
            step=steps,
            score=info.get("score"),
            level=h4kit.STATE["level"],
            detail=d,
            labels=[c.left.label, c.right.label],
            nonfinite=h4kit.STATE["nonfinite"],
            cmd=np.round(np.r_[c.left.last_cmd, c.right.last_cmd], 4).tolist(),
            q=np.round(jp, 4).tolist(),
            corr_mm=np.round(c.corr_p * 1000, 3).tolist(),
            corr_r=np.round(c.corr_r, 4).tolist(),
            bulb_rv=np.round(_rotvec(c.pose()[1]), 5).tolist(),
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
                f"step {steps} {c.left.label} | {c.right.label} score {info.get('score')} ax {m.get('axial_mm')} "
                f"rad {m.get('radial_mm')} tilt {m.get('tilt_deg')} roll {m.get('roll_deg')} clip {m.get('clip_deg')} "
                f"catch {m.get('catch_mm')} tips {m.get('tip_bulb_mm')}/{m.get('tip_clip_mm')} veto {d.get('veto')} "
                f"c {d.get('contacts')} "
                f"wall {time.time() - t1:.0f}s",
                flush=True,
            )
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if steps % 20 == 0 and ((out / "STOP").exists() or (out.parent / "STOP").exists()):
            stop = "STOP file"
            print("STOP file found: ending the trial", flush=True)
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
        scorer=dict(score=h4kit.STATE["score"], level=h4kit.STATE["level"]),
        final=h4kit.STATE["detail"],
        nonfinite=h4kit.STATE["nonfinite"],
        looks=c.log,
        wall_s=round(time.time() - t1, 1),
        wall_total_s=round(time.time() - t0, 1),
        worst_ik_mm=dict(
            left=round(c.left.worst_mm, 2), right=round(c.right.worst_mm, 2)
        ),
        physics_dt=h4kit.PHYSICS_DT,
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
    ap.add_argument("--max-steps", type=int, default=2400)
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
