"""Scripted full-score demo (and negative variants) for the flask-clamp task in the yam_arena embodiment.

    python demo.py task.py OUT [--max-steps N] [--no-record] [--stage demo|<variant>[,<variant>...]] [--seed S]

Acts ONLY through the 14 joint/gripper commands of ``YamArenaEmbodiment.step``. It reads the flask's, the free
clamp's and the fingers' poses through ``flaskkit.bench_state()`` (read-only, like a person watching the flask and the
clamp); it never writes a pose, a joint state or a score. Its joint commands are rate-limited below the CLI's
per-step delta guard and clipped to the URDF joint bounds.

Plan (the hand on the free clamp's side does the work; the other arm stays at READY):
1. Read which near-row clamp is free (the one without a flask) and where the flask stands; choose a neck grasp
   (approach pitched 35-55 deg from vertical, fingers closing horizontally) that keeps every waypoint in reach.
2. Grip the flask's neck and lift it.
3. Carry it over the platform's edge, bring it in from the open side of the free clamp (not over the loaded flasks),
   centre it over the clamp axis (closed loop on the measured flask pose) with its heel just above the flared tips.
4. Push it straight down: the heel spreads the five fingers, passes their pads and the bottom lands on the base
   plate; the fingers spring back onto the flask's cone. Stop pushing once the bottom has been on the plate 0.3 s.
5. Let go, back off and park while the scorer runs the shaker for its short test.
The arm adds the remaining joint error back to their commands (slow integral on their own joint readings), because
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
SQUEEZE = 0.006  # fingers commanded 6 mm inside the neck: about 30 N per finger (gripper 5 kN/m)
OPEN_CLEAR = 0.012  # 12 mm clearance a side before closing / after letting go
CARRY_DZ = 0.090  # flask bottom this far above the platform while carrying
STAGE_DY = 0.170  # staging point: flask this far outboard (y) of the free clamp's axis
PERCH_DZ = 0.080  # centring height: flask bottom above the clamp origin (heel just above the flared tips)
REST_DZ = 0.064  # set-down height: heel on the flared tips
CENTRE_TOL = 0.0015
PUSH_OVER = 0.006  # push target: 6 mm below the plate
PUSH_LEAD = 0.020  # commanded flask height at most 20 mm below the measured one
PUSH_RATE = 0.030  # m/s descent of the command when the flask follows
PUSH_XY_GAIN = 0.06
GIVE_DZ = 0.003  # a drop of more than 3 mm in one control step ...
GIVE_BELOW = 0.040  # ... ending with the bottom less than 40 mm above the plate: the heel has passed the pads
GIVE_OPEN = 0.004  # on giving way, the fingers open to 4 mm off the neck
PUSH_OFFSET_CLIP = 0.50  # joint-integral limit while pushing (rad; wrist torque stays below its 7 N m limit)
PUSH_MAX_S = 12.0
APPROACH = 0.070  # stand-off along the approach axis before closing on the neck
GRASP_PITCHES = (35.0, 45.0, 55.0)  # grasp pitch candidates (deg from straight down)
PARK = (0.160, 0.260, 0.950)  # parked TCP (left; right mirrored): clear of the platform in the overhead view
ROT_CORR_MAX = 0.10

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
        """True: joint integral (free moves and carrying); 'hold': offsets frozen (in contact); False: offsets decay."""
        self.comp = mode

    def observe(self, q):
        self.measured = np.asarray(q[:6], float)
        self.measured_grip = float(q[6])
        if self.comp == "hold":
            pass
        elif self.comp:
            self.offset = np.clip(
                self.offset + COMP_GAIN * (self.q - self.measured), -self.offset_clip, self.offset_clip
            )
        else:
            self.offset *= 0.85

    def relax(self):
        """Command the arm to where it is (keeping the sag offsets): releases the spring force it stores against the
        flask before the fingers open."""
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
        self.corr = {}

    def pose(self, name="flask"):
        import flaskkit

        return flaskkit.bench_state()[name]

    def spec(self):
        import flaskkit

        return flaskkit.bench_state()["spec"]

    def detail(self):
        import flaskkit

        return flaskkit.STATE["detail"]

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



def grip_of(half_opening_m):
    return float(np.clip(half_opening_m / FINGER_OPEN, 0.0, 1.0))


def grasp_rot(pitch_deg, fwd):
    """TCP rotation for a neck grasp: approach axis pitched ``pitch_deg`` from straight down toward the horizontal
    direction ``fwd``, fingers closing horizontally (perpendicular to ``fwd``)."""
    a = math.radians(pitch_deg)
    f = np.array([fwd[0], fwd[1], 0.0], float)
    f /= np.linalg.norm(f)
    z = np.array([0.0, 0.0, -math.cos(a)]) + math.sin(a) * f
    x = np.cross(z, f)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def carrier_of(c):
    """The hand on the free clamp's side carries the flask (the flask stands on that side)."""
    return "left" if L.socket_side(c.spec()["socket"]) > 0 else "right"


def clamp_frame(c):
    return c.pose("clamp")


def flask_bottom(c):
    return c.pose("flask")


def flask_rel(c):
    pf, rf = c.pose("flask")
    pc, rc = c.pose("clamp")
    return L.flask_in_clamp(pf, rf, pc, rc)


def capture(c, hand, ref="flask"):
    pr, rr = c.pose(ref)
    pt, rt = c.arms[hand].measured_tcp()
    c.G[(hand, ref)] = (rr.T @ (pt - pr), rr.T @ rt)


def track(c, hands, target_fn, duration, label, gain=0.05, until=None, max_s=None, clip=0.015):
    """Closed loop on the flask held by ``hands`` (one hand name or a tuple): each step command every holding hand's
    TCP pose that puts the flask (upright) at target_fn(s) through its frozen grasp transform, plus a slow integral
    correction of the measured flask position error (arm sag, grasp slip; against a resistance it winds up the push).
    ``until(c)`` ends the move early (checked after the nominal duration, up to ``max_s``)."""
    from yam_arena import ik

    hands = (hands,) if isinstance(hands, str) else tuple(hands)
    key = "+".join(hands)
    n = max(1, round(duration * CONTROL_HZ))
    nmax = n if max_s is None else max(n, round(max_s * CONTROL_HZ))
    cp = c.corr.get(key, np.zeros(3))
    for k in range(1, nmax + 1):
        pt = target_fn(ik.smoothstep(min(1.0, k / n)))
        pm, _ = c.pose("flask")
        cp = np.clip(cp + gain * (pt - pm), -clip, clip)
        for hand in hands:
            gp, gr = c.G[(hand, "flask")]
            arm = c.arms[hand]
            arm.label = label
            arm.set_pose(pt + cp + gp, gr)
        c.corr[key] = cp
        yield
        if until is not None and k >= n and until(c):
            break


def plan_grasp(c, carrier):
    """Choose the grasp pitch and heading that keep every waypoint of the carry and the push in reach (IK only)."""
    from yam_arena.constants import READY

    arm = c.arms[carrier]
    pf, _ = flask_bottom(c)
    pc, _ = clamp_frame(c)
    side = L.socket_side(c.spec()["socket"])
    zc = L.PLATFORM_Z + CARRY_DZ
    tcp_z = L.NECK_GRIP_Z
    pts = [pf + [0, 0, tcp_z], np.array([pf[0], pf[1], zc + tcp_z]),
           np.array([pc[0], pc[1] + side * STAGE_DY, zc + tcp_z]), np.array([pc[0], pc[1], zc + tcp_z]),
           np.array([pc[0], pc[1], pc[2] + PERCH_DZ + tcp_z]), np.array([pc[0], pc[1], pc[2] + L.SEAT_Z + tcp_z])]
    base = arm.base
    best = None
    for pitch in GRASP_PITCHES:
        for fwd in (pc[:2] - base[:2], pf[:2] - base[:2], (pc[:2] + pf[:2]) / 2 - base[:2]):
            r = grasp_rot(pitch, fwd)
            worst = 0.0
            for p in pts:
                e = min(sum(fastik.solve(p, r, s, base=base, iters=150)[1:]) for s in (np.array(READY, float),
                        np.array([0.0, 1.6, 1.4, -1.2, 0.0, 0.0])))
                worst = max(worst, e)
            if best is None or worst < best[0] - 1e-4:
                best = (worst, pitch, r)
    c.note("grasp plan", hand=carrier, pitch=best[1], worst=best[0])
    return best[2]


def pick(c, carrier, rot):
    arm = c.arms[carrier]
    arm.set_comp(True)
    pf, rf = flask_bottom(c)
    p = pf + rf @ np.array([0.0, 0.0, L.NECK_GRIP_Z])
    back = -rot[:, 2] * APPROACH
    yield from arm.move(p + back, rot, 3.0, grip_of(L.F_NECK_R + OPEN_CLEAR), f"{carrier} to neck")
    yield from wait(0.3)
    pf, rf = flask_bottom(c)  # re-read (the flask does not move, but the plan uses the measured pose)
    p = pf + rf @ np.array([0.0, 0.0, L.NECK_GRIP_Z])
    yield from arm.move(p, rot, 1.8, None, f"{carrier} onto neck")
    yield from wait(0.3)
    yield from arm.set_grip(grip_of(L.F_NECK_R - SQUEEZE), 0.8, f"{carrier} close")
    yield from wait(0.5)
    c.note("gripped", hand=carrier)
    capture(c, carrier)
    pf, _ = flask_bottom(c)
    yield from track(c, carrier, lambda s, pf=pf: pf + [0, 0, 0.03 * s], 1.0, f"{carrier} lift")
    capture(c, carrier)
    c.note("lifted", hand=carrier)


def carry_to_clamp(c, carrier):
    """Lift the flask over the platform's edge, bring it in from the open side of the free clamp (not over the loaded
    flasks) and centre it over the clamp axis with its heel just above the fingers' flared tips."""
    pc, rc = clamp_frame(c)
    p0, _ = flask_bottom(c)
    z_carry = L.PLATFORM_Z + CARRY_DZ
    side = L.socket_side(c.spec()["socket"])
    stage = np.array([pc[0], pc[1] + side * STAGE_DY, z_carry])
    up = np.array([p0[0], p0[1], z_carry])
    yield from track(c, carrier, lambda s: p0 + s * (up - p0), 1.6, f"{carrier} raise")
    yield from track(c, carrier, lambda s: up + s * (stage - up), 2.2, f"{carrier} to clamp side")
    over = np.array([pc[0], pc[1], z_carry])
    yield from track(c, carrier, lambda s: stage + s * (over - stage), 2.0, f"{carrier} over clamp", gain=0.06)
    pc, rc = clamp_frame(c)
    above = pc + rc @ np.array([0.0, 0.0, PERCH_DZ])
    cnt = [0]

    def centred(cc):
        pm, _ = cc.pose("flask")
        cnt[0] = cnt[0] + 1 if np.hypot(*(pm[:2] - above[:2])) < CENTRE_TOL else 0
        return cnt[0] >= 4

    yield from track(c, carrier, lambda s: over + s * (above - over), 1.5, f"{carrier} lower to the tips",
                     gain=0.07, until=centred, max_s=5.0)
    c.note("over the tips", hand=carrier, rel=flask_rel(c))


def push_in(c, hands):
    """Push the flask straight down by its neck: the heel spreads the fingers on their flared tips, passes the pads
    and lands on the base plate. The command runs at most 20 mm ahead of the flask (down to a few mm below the plate),
    and the joint integral winds up the push as the fingers resist; it ends once the bottom has been on the plate for
    0.3 s."""
    hands = (hands,) if isinstance(hands, str) else tuple(hands)
    c.saved_offset = {}
    for h in hands:
        c.saved_offset[h] = c.arms[h].offset.copy()
        c.arms[h].offset_clip = PUSH_OFFSET_CLIP
        c.arms[h].set_comp(True)
    pc, rc = clamp_frame(c)
    p0, _ = flask_bottom(c)
    goal = pc + rc @ np.array([0.0, 0.0, L.SEAT_Z - PUSH_OVER])
    cnt = [0]

    def seated(cc):
        cnt[0] = cnt[0] + 1 if abs(flask_rel(cc)["dz"] - L.SEAT_Z) < 0.001 else 0
        return cnt[0] >= 6

    # The arm is compliant (it sags sideways under the fingers' resistance), so the push is paced by the flask: the
    # commanded flask height stays at most PUSH_LEAD below the measured one, and a slow integral keeps the measured
    # neck over the clamp axis (the flask square) while the joint integral takes out the sag.
    # When the heel slips past the pads the flask gives way at once: the hand then eases off (pre-push sag offsets,
    # command at the flask, fingers loosened off the neck) and the clamp's springs finish drawing it down.
    cxy = c.corr.get("+".join(hands), np.zeros(3)) * np.array([1.0, 1.0, 0.0])
    z_cmd, z_prev, gave = p0[2], p0[2], False
    for k in range(round(PUSH_MAX_S * CONTROL_HZ)):
        pf, rf = flask_bottom(c)
        neck = pf + rf @ np.array([0.0, 0.0, L.NECK_GRIP_Z])
        e = np.array([pc[0] - neck[0], pc[1] - neck[1], 0.0])
        cxy = np.clip(cxy + PUSH_XY_GAIN * e, -0.020, 0.020)
        if not gave and z_prev - pf[2] > GIVE_DZ and pf[2] < pc[2] + GIVE_BELOW:
            gave = True
            for h in hands:
                c.arms[h].offset = c.saved_offset[h].copy()
                c.arms[h].set_comp("hold")
                c.arms[h].grip = grip_of(L.F_NECK_R + GIVE_OPEN)  # loosen the fingers: let the flask drop in freely
            c.note("gave way", rel=flask_rel(c))
        z_prev = pf[2]
        if gave:
            z_cmd = max(goal[2], min(z_cmd, pf[2]) - PUSH_RATE / CONTROL_HZ)
        else:
            z_cmd = max(goal[2], z_cmd - PUSH_RATE / CONTROL_HZ, pf[2] - PUSH_LEAD)
        target = np.array([pc[0], pc[1], z_cmd]) + cxy
        for h in hands:
            gp, gr = c.G[(h, "flask")]
            c.arms[h].label = "push in"
            c.arms[h].set_pose(target + gp, gr)
        yield
        if seated(c):
            break
    c.note("pushed", hands=list(hands), rel=flask_rel(c), seated=bool(c.detail().get("seated")))


def let_go(c, carrier, rot):
    """Stop pushing (the arm back to its pre-push sag offsets, commanded where it is), open, back off along the
    approach axis and up, then park clear of the platform."""
    arm = c.arms[carrier]
    arm.offset_clip = 0.25
    saved = getattr(c, "saved_offset", None) or {}
    if carrier in saved:
        arm.offset = saved[carrier]
    arm.relax()
    yield from wait(0.3)
    yield from arm.set_grip(grip_of(L.F_NECK_R + OPEN_CLEAR), 0.8, f"{carrier} open")
    yield from wait(0.3)
    arm.set_comp(True)
    yield from arm.move(arm.pos - rot[:, 2] * APPROACH + [0, 0, 0.03], arm.rot, 1.5, None, f"{carrier} back off")
    yield from park(c, carrier)
    c.note("released", hand=carrier, rel=flask_rel(c))


def set_on_tips(c, carrier):
    """(negative) Lower the heel onto the flared tips only."""
    pc, rc = clamp_frame(c)
    pf, _ = flask_bottom(c)
    rest = np.array([pf[0], pf[1], pc[2] + REST_DZ])
    yield from track(c, carrier, lambda s: pf + s * (rest - pf), 1.2, f"{carrier} onto the tips", gain=0.0)


def park(c, hand, duration=2.2):
    arm = c.arms[hand]
    arm.set_comp(True)
    p = np.array([PARK[0], PARK[1] * (1 if hand == "left" else -1), PARK[2]])
    yield from arm.move(p, arm.rot, duration, 0.0, f"{hand} park")


def demo(c):
    carrier = carrier_of(c)
    rot = plan_grasp(c, carrier)
    yield from wait(0.5)
    yield from pick(c, carrier, rot)
    yield from carry_to_clamp(c, carrier)
    yield from push_in(c, carrier)
    yield from let_go(c, carrier, rot)
    yield from wait(5.0)


def idle(c):
    yield from wait(8.0)


def neg_perched(c):
    """Flask set centred on the clamp's flared finger tips and let go, never pushed down past the pads: 0."""
    carrier = carrier_of(c)
    rot = plan_grasp(c, carrier)
    yield from wait(0.5)
    yield from pick(c, carrier, rot)
    yield from carry_to_clamp(c, carrier)
    yield from set_on_tips(c, carrier)
    yield from let_go(c, carrier, rot)
    yield from wait(5.0)


def neg_held(c):
    """Flask pushed home, but the hand keeps holding its neck to the end: 0.5 (not released, no shake test)."""
    carrier = carrier_of(c)
    rot = plan_grasp(c, carrier)
    yield from wait(0.5)
    yield from pick(c, carrier, rot)
    yield from carry_to_clamp(c, carrier)
    yield from push_in(c, carrier)
    arm = c.arms[carrier]
    arm.offset_clip = 0.25
    arm.offset = c.saved_offset[carrier]
    arm.relax()
    yield from wait(6.0)


def replay(c):
    """Open-loop check: plays the 14-D commands recorded in a finished run's trace.jsonl (path in FLASK_REPLAY), step
    by step, without reading anything: what a fixed trajectory scores on another start state. FLASK_REPLAY_MIRROR=1
    plays its left-right mirror image instead (for starts whose free clamp is on the other side)."""
    import os

    rows = [json.loads(line) for line in open(os.environ["FLASK_REPLAY"])]
    c.replay = [r["cmd"] for r in rows]
    if os.environ.get("FLASK_REPLAY_MIRROR") == "1":  # the trace's left-right mirror image: arms swapped, joints 1, 5
        # and 6 negated (the YAM pair is mirror-symmetric about the x-z plane)
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
    "replay": replay,
    "neg-perched": neg_perched,
    "neg-held": neg_held,
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
    seeds=None,
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
    # per-trial seeds: --seeds a,b,... ('none' = unseeded named setup); otherwise --seed for every trial
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
    import flaskkit

    rt = flaskkit.runtime()
    poses = rt.read()
    return dict(
        score=flaskkit.STATE["score"],
        setup=rt.setup,
        bodies={n: np.round(p * 1000, 4).tolist() for n, (p, r) in poses.items()},
        rots={n: np.round(r, 6).tolist() for n, (p, r) in poses.items()},
    )


def reset_report(stages, resets, results):
    import flaskkit

    rows = []
    for i, r in enumerate(resets):
        want = flaskkit.authored(
            tuple(r["setup"]) if isinstance(r["setup"], list) else r["setup"]
        )
        dev = max(
            float(np.linalg.norm(np.subtract(r["bodies"][n], want[n][0] * 1000)))
            for n in ("flask", "clamp", "platform", "finger0", "finger1", "finger2", "finger3", "finger4")
        )
        rdev = float(np.abs(np.subtract(r["rots"]["flask"], want["flask"][1])).max())
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
        x["score_after_reset"] == 0.0 and x["max_body_dev_from_authored_mm"] < 1.0
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


_SLOW = [0]


def _watchdog(step_s):
    """Optional (FLASK_WATCHDOG_S set): end the process if five control steps in a row each take longer than that many
    seconds of wall time (a stalled renderer), instead of holding a compute slot until an outside timeout."""
    import os

    lim = float(os.environ.get("FLASK_WATCHDOG_S", "0") or 0)
    if lim <= 0:
        return
    _SLOW[0] = _SLOW[0] + 1 if step_s > lim else 0
    if _SLOW[0] >= 5:
        print(f"WATCHDOG: 5 control steps over {lim} s each; ending the run", flush=True)
        os._exit(3)


def run_trial(emb, out, max_steps, record, stage, t0, seed=None):
    import flaskkit
    from inspect_robots import Scene
    from inspect_robots_yam.packing import STATE_KEY

    out.mkdir(parents=True, exist_ok=True)
    t1 = time.time()
    flaskkit._RT["override"] = OVERRIDES.get(stage)  # development probes only (None for every evidence stage)
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
        d = flaskkit.STATE["detail"]
        row = dict(
            step=steps,
            score=info.get("score"),
            level=flaskkit.STATE["level"],
            detail=d,
            labels=[c.left.label, c.right.label],
            nonfinite=flaskkit.STATE["nonfinite"],
            cmd=np.round(np.r_[c.left.last_cmd, c.right.last_cmd], 4).tolist(),
            q=np.round(jp, 4).tolist(),
            corr_mm={f"{k[0]}:{'+'.join(k[1])}": np.round(v[0] * 1000, 3).tolist() for k, v in c.corr.items()},
            extra=getattr(c, "extra_now", None),
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
            m = d.get("measure", {}) or {}
            print(
                f"step {steps} {c.left.label} | {c.right.label} score {info.get('score')} rel {m.get('rel')} "
                f"theta {m.get('theta_deg')} fN {m.get('finger_n')} tip {m.get('tip_mm')} shake {d.get('shaking')} "
                f"grip {d.get('grip_peak_n')} press {d.get('press_peak_n')} wall {time.time() - t1:.0f}s",
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
        scorer=dict(score=flaskkit.STATE["score"], level=flaskkit.STATE["level"]),
        final=flaskkit.STATE["detail"],
        nonfinite=flaskkit.STATE["nonfinite"],
        looks=c.log,
        wall_s=round(time.time() - t1, 1),
        wall_total_s=round(time.time() - t0, 1),
        worst_ik_mm=dict(
            left=round(c.left.worst_mm, 2), right=round(c.right.worst_mm, 2)
        ),
        physics_dt=flaskkit.PHYSICS_DT,
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
    ap.add_argument(
        "--seeds",
        default=None,
        help="comma-separated reset seed per trial of a multi-stage run ('none' = unseeded named setup)",
    )
    a = ap.parse_args()
    cw, ch = (int(v) for v in a.cam_size.split("x"))
    run(a.task, a.out, a.max_steps, not a.no_record, a.stage, cw, ch, a.device, a.seed, a.seeds)
