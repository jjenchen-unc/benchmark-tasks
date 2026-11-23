"""Closed-loop robot-command policies for the guided Leviton deadfront assembly."""
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
GRASP_HALF = L.GRIP_HALF[0]
LIFT = .075  # clear the 60 mm stand backrest before translating
PRESS_FLAT = .00025
DRV_GRIP_BELOW_TOP = 0.058  # driver: pad tips 58 mm below the handle's top (the top stays clear of the finger carriages)
DRV_PRESS = .0004  # contact preload at the working face of the magnetic driver
TOOL_CARRY_JOINT_RATE = math.radians(25)  # maximum joint reference speed during the free-tool handoff
DRIVER_FAR = .024
DRIVER_STANDOFF = .016
STROKE_DEG = 90.0
MAX_STROKES = 85


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
        self.payload_offset = np.zeros(6)
        self.measured = self.q.copy()
        self.measured_grip = 1.0
        self.comp = True
        self.label = ""
        self.worst_mm = 0.0
        _, _, lo, hi, _ = fastik._chain()
        self.lo, self.hi = lo, hi
        self.last_cmd = None
        self.driver_tracking = None
        self.reference_velocity = np.zeros(6)

    @property
    def cmd(self):
        target = self.q + self.offset + self.payload_offset
        if self.driver_tracking is not None:
            from driver_dynamics import joint_torque

            grasp, bias, stiffness, damping = self.driver_tracking
            vel = (self.q-self.previous_reference)/DT
            accel = (vel-self.reference_velocity)/DT
            tau = joint_torque(self.q, vel, accel, grasp, self.measured_grip*FINGER_OPEN)
            error = self.previous_reference-self.measured
            # A position drive needs a lead equal to its damping/stiffness time constant during rotation.
            tracking_gain = np.clip(damping/(stiffness*DT)-.5, .25, 2.0)
            target = self.q + bias + self.tracking_trim + (tau+damping*vel)/stiffness - .5*DT*vel + tracking_gain*error
        c = np.r_[np.clip(target, self.lo, self.hi), self.grip]
        lim = np.r_[RATE * 0.05 * (self.hi - self.lo), RATE * 0.05]
        if self.last_cmd is not None:
            c = np.clip(c, self.last_cmd - lim, self.last_cmd + lim)
        c[:6] = np.clip(c[:6], self.lo, self.hi)
        return c

    def enable_driver_tracking(self, grasp):
        from driver_dynamics import joint_torque
        from yam_arena.constants import SHOULDER, WRIST

        stiffness = np.array([SHOULDER["stiffness"]]*3+[WRIST["stiffness"]]*3)
        damping = np.array([SHOULDER["damping"]]*3+[WRIST["damping"]]*3)
        zero = np.zeros(6)
        gravity = joint_torque(self.measured, zero, zero, grasp, self.measured_grip*FINGER_OPEN)
        self.previous_reference = self.q.copy()
        self.tracking_trim = zero.copy()
        self.reference_velocity = zero.copy()
        self.driver_tracking = (grasp, self.offset-gravity/stiffness, stiffness, damping)

    def commit(self, c):
        if self.driver_tracking is not None:
            self.reference_velocity = (self.q-self.previous_reference)/DT
            self.previous_reference = self.q.copy()
        self.last_cmd = np.asarray(c, float).copy()

    def set_comp(self, mode):
        """True: joint integral (free moves and carrying); 'hold': offsets frozen (in contact); False: offsets decay."""
        self.comp = mode

    def support(self, point, force_n):
        """Use the station joint position drives to support a share of the plate's measured weight."""
        from yam_arena.constants import SHOULDER, WRIST

        _, _, axes = fastik.fk(self.q, self.base)
        stiffness = [SHOULDER["stiffness"]]*3 + [WRIST["stiffness"]]*3
        upward = np.array([0., 0., force_n])
        torques = [axis @ np.cross(point-joint, upward) for axis, joint in axes]
        self.payload_offset = np.array(torques)/stiffness

    def observe(self, q):
        self.measured = np.asarray(q[:6], float)
        self.measured_grip = float(q[6])
        if self.driver_tracking is not None:
            self.tracking_trim = np.clip(self.tracking_trim + .06*(self.previous_reference-self.measured), -.07, .07)
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
        self.driver_trace = []
        self.driver_contact_view = None
        self.tool_fixtures = None
        self.corr_p, self.corr_r = np.zeros(3), np.zeros(3)
        self.dcorr = np.zeros(3)
        self.opts = {}
        self.cover_mass, self.cover_com, _ = L.cover_mass_props()
        self.support_fraction = 0.

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

    def tool_arm_clearance(self, q):
        from tool_clearance import arm_fixture_clearance, fixture_boxes

        if self.tool_fixtures is None:
            self.tool_fixtures = fixture_boxes(*self.pose("stand"))
        return arm_fixture_clearance(q, self.right.base, self.right.measured_grip*FINGER_OPEN, self.tool_fixtures)

    def check_tool_route(self, q):
        gap, link, fixture = self.tool_arm_clearance(q)
        if gap < .004:
            raise Stop(f"tool route clearance: {link} to {fixture} {gap*1000:.1f} mm")

    def tool_contacts(self):
        """Read the forces on the carried driver and between the stand and arm links."""
        import panelkit

        runtime = panelkit.runtime()
        if runtime is None:
            return {}, {}
        if self.driver_contact_view is None:
            root = "/World/envs/env_0/panel/"
            fixtures = ["holster", "stand", "tub", "cover", "screw_r"]
            links = runtime.robot_links
            paths = [root+name for name in fixtures] + links
            self.driver_contact_names = fixtures + [path.split("/env_0/", 1)[-1] for path in links]
            self.driver_contact_view = runtime.simview.create_rigid_contact_view(
                [root+"driver"], filter_patterns=[paths], max_contact_data_count=2048)
            self.stand_contact_names = self.driver_contact_names[len(fixtures):]
            self.stand_contact_view = runtime.simview.create_rigid_contact_view(
                [root+"stand"], filter_patterns=[links], max_contact_data_count=2048)

        def magnitudes(view, names):
            forces = view.get_contact_force_matrix(dt=panelkit.PHYSICS_DT)[0].detach().cpu().numpy()
            return {name: round(float(force), 3) for name, force in zip(names, np.linalg.norm(forces, axis=-1))
                    if force > .05}

        return (magnitudes(self.driver_contact_view, self.driver_contact_names),
                magnitudes(self.stand_contact_view, self.stand_contact_names))

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
                   screws=d.get("screws"), contacts=d.get("contacts"),
                   tcp_mm={n: np.round(a.measured_tcp()[0]*1000, 2).tolist() for n, a in self.arms.items()}, **kw)
        self.log.append(row)
        print("LOOK", json.dumps(row, default=float), flush=True)


# --------------------------------------------------------------------------------------------- cover handling
def grasp_v(name):
    sign = 1. if name == "left" else -1.
    return L.grip_center(sign), L.grip_frame(sign)[:, 2]


def grasp_rot_v(c, name, pc, rc):
    return L.grip_frame(1. if name == "left" else -1.)


def capture_grasp(c):
    """Freeze the cover -> TCP transform of each hand (measured, cover held still)."""
    pc, rc = c.pose()
    for n, arm in c.arms.items():
        pt, rt = arm.measured_tcp()
        c.G[n] = (rc.T @ (pt - pc), rc.T @ rt)
    c.corr_p, c.corr_r = np.zeros(3), np.zeros(3)


def update_support(c, pc, rc):
    for side, arm in c.arms.items():
        sign = 1. if side == "left" else -1.
        com = c.cover_com.copy()
        com[1] = L.GRIP_Y*sign
        arm.support(pc+rc@com, c.cover_mass*9.81*.5*c.support_fraction)


def track(c, target_fn, duration, label, gain=0.05, rot_gain=None, until=None, max_s=None, arms=("left", "right"),
          mask=(1.0, 1.0, 1.0), guide=False):
    """Closed loop on the cover: each step command the holding hands to the TCP poses that put the cover at
    target_fn(s) through the frozen grasp transforms, plus slow integral corrections of the measured cover pose error
    (arm sag, grasp slip). ``mask`` weights the position error per axis (0 = that axis is left to contact)."""
    from yam_arena import ik

    n = max(1, round(duration * CONTROL_HZ))
    nmax = n if max_s is None else max(n, round(max_s * CONTROL_HZ))
    for k in range(1, nmax + 1):
        pt, rt = target_fn(ik.smoothstep(min(1.0, k / n)))
        pm, rm = c.pose()
        anchor = np.array([0., 0., L.GUIDE_Z]) if guide else np.zeros(3)
        err = ((pt + rt@anchor) - (pm + rm@anchor)) * np.asarray(mask)
        limit = .024 if guide else .012
        c.corr_p = np.clip(c.corr_p + gain * err, -limit, limit)
        update_support(c, pm, rm)
        e_r = _rotvec(rt @ rm.T)
        angle_limit = .18 if guide else .10
        c.corr_r = np.clip(c.corr_r + (gain if rot_gain is None else rot_gain)*e_r, -angle_limit, angle_limit)
        rc = _rotm(c.corr_r)
        for name in arms:
            gp, gr = c.G[name]
            arm = c.arms[name]
            arm.label = label
            arm.set_pose(pt + rt@anchor + c.corr_p + rc@rt@(gp-anchor), rc@rt@gr)
        yield
        if until is not None and k >= n and until(c):
            break


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
    yield from both(*(arm.goto(tg[n][0] - 0.06 * tg[n][2], tg[n][1], 3.0, grip_of(GRASP_HALF + 0.010), f"{n} over handle")
                      for n, arm in c.arms.items()))
    yield from wait(0.3)
    pc, rc = c.pose()
    for n in c.arms:
        p, z = grasp_v(n)
        tg[n] = (pc + rc @ p, tg[n][1], rc @ z)
    yield from both(*(arm.move(tg[n][0], tg[n][1], 2.0, None, f"{n} onto handle") for n, arm in c.arms.items()))
    yield from wait(0.3)
    yield from both(*(arm.set_grip(grip_of(GRASP_HALF-.006), 1.0, f"{n} pinch") for n, arm in c.arms.items()))
    yield from wait(0.5)
    c.note("pinched")
    pc, rc = c.pose()
    capture_grasp(c)
    # Build the static load before lifting. On the old pickup the wrists stayed at their starting
    # height through the 25 mm command and the stand still carried nearly the entire weight.
    for k in range(CONTROL_HZ):
        c.support_fraction = (k+1)/CONTROL_HZ
        update_support(c, *c.pose())
        yield
    up = rc[:, 2]
    def clears_stand(ctx):
        p, r = ctx.pose()
        bottom = p - r[:,2]*(L.COVER_H/2)
        initial_bottom = pc - rc[:,2]*(L.COVER_H/2)
        return (bottom-initial_bottom) @ up >= LIFT-.006

    yield from track(c, lambda s: (pc+up*(LIFT*s), rc), 3.0, "lift from stand",
                     until=clears_stand, max_s=6.)
    if not clears_stand(c):
        raise Stop("plate bottom has not cleared the 60 mm stand backrest")
    capture_grasp(c)
    c.note("lifted")


def insert_upper_edge(c):
    from yam_arena import ik

    pc, rc = c.pose()
    po = c.panel()
    first, middle, last = np.asarray(L.ENTRY_WAYPOINTS)
    def pose(offset):
        return L.guide_pose(L.INSERT_ANGLE, po, dx=offset[0], dz=offset[1])

    def register(offset, angle_tol=2.0, distance_tol=.003):
        p, r = c.pose()
        anchor = np.array([0., 0., L.GUIDE_Z])
        goal, rg = pose(offset)
        angle = math.degrees(math.atan2(r[0,2], r[2,2]))
        return (abs(angle-L.INSERT_ANGLE) < angle_tol
                and max(abs(p + r@anchor - goal - rg@anchor)) < distance_tol)

    p1, r1 = pose(first)
    yield from track(c, lambda t: (pc+t*(p1-pc), ik.slerp_matrix(rc, r1, t)), 4.5,
                     "angle plate clear of top return")
    # Control the guide midpoint. An angle error of 3.5 degrees at the plate center used to put
    # the leading edge about 15 mm above the opening even when the center reached its target.
    c.corr_p[:], c.corr_r[:] = 0., 0.
    yield from track(c, lambda t: pose(first), 1., "align while clear of return", guide=True,
                     until=lambda ctx: register(first), max_s=4.)
    c.note("before top return")
    for attempt in range(2):
        if attempt:
            yield from track(c, lambda t: pose(last+t*(first-last)), 1.8,
                             "withdraw from top return", guide=True)
            yield from track(c, lambda t: pose(first), 1., "recover insertion angle", guide=True,
                             until=lambda ctx: register(first), max_s=4.)
        yield from track(c, lambda t: pose(first+t*(middle-first)), 2.0,
                         "slide top under return", guide=True)
        yield from track(c, lambda t: pose(middle), .8, "register beneath return", guide=True,
                         until=lambda ctx: register(middle), max_s=2.)
        c.note("below top return", attempt=attempt+1)
        yield from track(c, lambda t: pose(middle+t*(last-middle)), 2.0,
                         "slide above side guides", guide=True)
        yield from track(c, lambda t: pose(last), .8, "register both side guides", guide=True,
                         until=lambda ctx: ctx.detail().get("guided"), max_s=3.)
        c.note("measured guide entry", attempt=attempt+1)
        if c.detail().get("guided"):
            break
    else:
        raise Stop("upper edge has not entered both guides at 45 degrees")


def seating_adjustment(gaps_mm, dx, angle_deg):
    """Bias the next two-hand pose using the signed normal gaps at the top and bottom edges."""
    top, bottom = np.asarray(gaps_mm).reshape(2, 2).mean(axis=1) * .001
    top_error, bottom_error = .00010-top, .00010-bottom
    # Change pitch to seat the bottom independently. For +pitch the bottom moves toward the robot.
    pitch_error = math.degrees(math.atan2(top_error-bottom_error, L.COVER_H))
    # The guide pivot lies 20 mm below the top edge. Interpolate the corresponding axial correction.
    top_weight = .5 + L.GUIDE_Z/L.COVER_H
    pivot_error = top_weight*top_error + (1.-top_weight)*bottom_error
    return (float(np.clip(dx+.16*pivot_error, -.002, .002)),
            float(np.clip(angle_deg+.16*pitch_error, -.65, .65)))


def swing_shut(c):
    po = c.panel()
    yield from track(c, lambda s: L.guide_pose(L.INSERT_ANGLE*(1-s), po), 5., "bring bottom toward panel", guide=True)
    for arm in c.arms.values():
        arm.set_comp("hold")
    # Lower the plate onto the rest before handing control of the driver to the right arm.
    # The two-part handoff keeps one arm from having to carry the entire assembly.
    mass = L.cover_mass_props()[0]
    def supported(ctx):
        detail = ctx.detail()
        return (detail.get("guided") and detail.get("measure", {}).get("flat")
                and detail.get("contacts", {}).get("tub_up_n", 0.) >= mass*9.81/2)
    def lower_onto_rest(s):
        c.support_fraction = 1.-s
        return L.guide_pose(0., po+[0., 0., -.0006*s], dx=PRESS_FLAT)
    yield from track(c, lower_onto_rest, 1.5, "transfer weight to lower rest",
                     until=supported, max_s=2.5, guide=True)
    c.note("load on lower rest")
    if not supported(c):
        # The tub can already carry the panel while its bottom still stands proud. Correct the
        # upper and lower measured gaps independently with translation and pitch about the guides.
        dx, angle = PRESS_FLAT, 0.
        def square_on_land(s):
            nonlocal dx, angle
            gaps = c.detail().get("measure", {}).get("gaps_mm", [])
            if len(gaps) == 4:
                dx, angle = seating_adjustment(gaps, dx, angle)
            return L.guide_pose(angle, po, dx=dx, dz=-.0006)
        yield from track(c, square_on_land, .8, "square bottom against mounting land",
                         until=supported, max_s=5., guide=True)
        c.note("squared on mounting land", pitch_bias_deg=angle, guide_bias_mm=dx*1000)
    if not supported(c):
        m = c.detail()["measure"]
        upward = c.detail()["contacts"].get("tub_up_n", 0.)
        raise Stop(f"handoff incomplete: gaps {m['gaps_mm']} mm, tub upward {upward:.2f} N")

    c.note("lower edge seated")


# --------------------------------------------------------------------------------------------- driver handling
def driver_rot(phi):
    """Driver frame pointing at the wall (socket toward +x), roll phi (driver x axis angle in the y-z plane)."""
    return L.driver_rot_x(phi)


def driver_roll(rd):
    return math.atan2(rd[2, 0], rd[1, 0])


def driver_grasp_fit(mouth_in_tcp, handle_axis_in_tcp):
    """Distance of the pad-tip midpoint from the handle axis and its station along that axis."""
    from yam_arena.constants import PAD_TIP_PAST_TCP

    delta = np.array([0., 0., PAD_TIP_PAST_TCP]) - mouth_in_tcp
    axial = float(delta @ handle_axis_in_tcp)
    radial = float(np.linalg.norm(delta - axial*handle_axis_in_tcp))
    return radial, axial


def checked_driver_grasp(c):
    pd, rd = c.pose("driver")
    pt, rt = c.right.measured_tcp()
    gp, gr = rt.T @ (pd-pt), rt.T @ rd
    radial, axial = driver_grasp_fit(gp, gr[:, 2])
    half_gap = c.right.measured_grip * FINGER_OPEN
    lost = (radial > L.DRV_HANDLE_R+.002 or half_gap > L.DRV_HANDLE_R+.006 or
            not L.DRV_HANDLE_Z0 <= axial <= L.DRV_HANDLE_Z0+L.DRV_HANDLE_L)
    if lost or c.step % 4 == 0 or c.right.label.startswith(("driver to screw", "stroke ", "return ", "hold socket")):
        detail = c.detail()
        contact = detail.get("contacts", {})
        screw = detail.get("screws", {})
        tool_forces, stand_forces = c.tool_contacts()
        gap, link, fixture = c.tool_arm_clearance(c.right.measured)
        c.driver_trace.append(dict(step=c.step, phase=c.right.label,
                                   radial_mm=round(radial*1000,2), axial_mm=round(axial*1000,2),
                                   driver_roll_deg=round(math.degrees(driver_roll(rd)),3),
                                   engagement=screw.get("eng"), started=screw.get("started"), advance_mm=screw.get("adv_mm"),
                                   half_gap_mm=round(half_gap*1000,2), mouth_mm=np.round(pd*1000,2).tolist(),
                                   tcp_mm=np.round(pt*1000,2).tolist(), tool_axis=np.round(-rd[:,2],4).tolist(),
                                   q=np.round(c.right.measured,4).tolist(), target_q=np.round(c.right.q,4).tolist(),
                                   screw_contact_n=contact.get("driver_on_screw_n"),
                                   driver_contacts_n=tool_forces, stand_on_arm_n=stand_forces,
                                   arm_gap_mm=round(gap*1000,2), arm_gap_pair=[link, fixture]))
    if lost:
        raise Stop(f"driver out of grasp: pad radial {radial*1000:.1f} mm, "
                   f"axial {axial*1000:.1f} mm, half-gap {half_gap*1000:.1f} mm")
    return gp, gr


def capture_driver(c):
    c.GD = checked_driver_grasp(c)
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


J6_PARK = .40  # rotate the wrist housing clear of the cover stand before the driver becomes horizontal


def driver_ik(c, mouth, axis=(1.0, 0.0, 0.0), roll=None, q0=None, j6_pref=None):
    """Right-arm IK on the driver itself: socket mouth at ``mouth``, socket pointing along ``axis``, roll free unless
    given (the roll is free for the approach; the strokes set it). Bounded least squares from q0."""
    from scipy.optimize import least_squares

    arm = c.right
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    mouth = np.asarray(mouth, float)

    def resid(q):
        m, ax, ro = _driver_of_q(c, q)
        r = [(m - mouth) * 1000.0, (ax - axis) * 300.0]
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
    c.check_tool_route(q)
    arm.worst_mm = max(arm.worst_mm, pe * 1000)
    arm.q = q
    arm.pos, arm.rot = fastik.tcp_pose(q, arm.base)
    arm.label = label
    return ro


def stroke_room(c, mouth, phi):
    """Largest roll increment (deg, up to STROKE_DEG) the arm can follow from its current joints, in 15 deg steps."""
    q = c.right.q.copy()
    room = 0.0
    for d_ in np.arange(15.0, min(STROKE_DEG, 60.) + 0.1, 15.0):
        q, pe, ae, _ = driver_ik(c, mouth, roll=phi + math.radians(d_), q0=q)
        margin = np.minimum(q-c.right.lo, c.right.hi-q)
        if pe > .0008 or ae > 2.0 or min(margin) < .15 or c.tool_arm_clearance(q)[0] < .010:
            break
        room = float(d_)
    return room


def goto_driver(c, mouth, duration, label, observe_grip=True):
    """Carry the held driver toward the pilot on an arc above and inboard of its holster."""
    from yam_arena import ik

    arm = c.right
    start, start_axis, _ = _driver_of_q(c, arm.measured)
    end_axis = np.array([1., 0., 0.])
    theta = math.acos(float(np.clip(start_axis @ end_axis, -1, 1)))
    wrist_start = arm.measured[5]
    n = max(1, round(duration * CONTROL_HZ))
    arm.label = label
    for k in range(1, n+1):
        t = ik.smoothstep(k/n)
        tilt = t**1.3
        direction = (math.sin((1-tilt)*theta)*start_axis + math.sin(tilt*theta)*end_axis
                     if theta > .001 else end_axis)
        target = (1-t)*start + t*mouth + np.array([0., -.045, .035])*math.sin(math.pi*t)
        q, pe, ae, _ = driver_ik(c, target, direction, j6_pref=(1-t)*wrist_start + t*J6_PARK)
        if pe > .0005 or ae > 1.:
            # Wrist roll is free in transit. At a joint bound, keep the tool on the clearance path.
            q, pe, ae, _ = driver_ik(c, target, direction, q0=q)
        if pe > .0015 or ae > 3.:
            raise Stop(f"driver transit unreachable at {t:.2f}: {pe*1000:.1f} mm, {ae:.1f} deg")
        # This off-axis grasp can turn three wrist joints rapidly while the socket itself moves slowly.
        # Walk the actual IK branch at a bounded joint speed and give the position drives time to follow.
        previous = arm.q.copy()
        segments = max(1, math.ceil(np.max(abs(q-previous))/(TOOL_CARRY_JOINT_RATE*DT)))
        for i in range(1, segments+1):
            arm.q = previous + (q-previous)*i/segments
            arm.pos, arm.rot = fastik.tcp_pose(arm.q, arm.base)
            c.check_tool_route(arm.q)
            if observe_grip:
                checked_driver_grasp(c)
            yield
            if observe_grip:
                for _ in range(CONTROL_HZ*3):
                    if max(abs(arm.q-arm.measured)) <= math.radians(2.5):
                        break
                    checked_driver_grasp(c)
                    yield
                else:
                    raise Stop(f"driver transit stalled at {t:.2f}: joint lag "
                               f"{math.degrees(max(abs(arm.q-arm.measured))):.1f} deg")


def track_driver(c, target_fn, duration, label, gain=0.06, until=None, max_s=None, correct=True):
    """target_fn(s) -> (mouth position, roll or None): driver IK every step, socket along +x, with a slow integral
    correction of the measured mouth position across the axis (the axial direction is left to contact)."""
    from yam_arena import ik

    n = max(1, round(duration * CONTROL_HZ))
    nmax = n if max_s is None else max(n, round(max_s * CONTROL_HZ))
    for k in range(1, nmax + 1):
        checked_driver_grasp(c)
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


def driver_pick_pose(pd, rd):
    from yam_arena.constants import PAD_TIP_PAST_TCP

    # A 30-degree side approach leaves the wrist above the table at the low fixing screw.
    r = rd @ frame([0., .5, -math.sqrt(.75)], [-1., 0., 0.])
    grip_z = L.DRV_HANDLE_Z0 + L.DRV_HANDLE_L - DRV_GRIP_BELOW_TOP
    return pd + rd[:, 2]*grip_z - r[:, 2]*PAD_TIP_PAST_TCP, r


def driver_clear_point(p):
    return np.array([.20, -.17, p[2]+.065])


def take_driver(c):
    """Grasp the driver at an angle, then lift it vertically out of the holster."""
    arm = c.right
    arm.set_comp(True)
    p, r = driver_pick_pose(*c.pose("driver"))
    yield from arm.goto(p - .05*r[:, 2], r, 3., grip_of(L.DRV_HANDLE_R+.012), "right over driver")
    yield from wait(.3)
    p, r = driver_pick_pose(*c.pose("driver"))
    yield from arm.move(p, r, 1.5, None, "right onto handle")
    yield from wait(0.6)  # the joint integral takes out the arm's sag before the fingers close
    arm.set_comp("hold")  # in contact from here: no integral wind-up
    yield from arm.set_grip(grip_of(L.DRV_HANDLE_R - 0.005), 0.8, "right grip driver")
    yield from wait(0.4)
    yield from arm.move(p + [0.0, 0.0, 0.065], r, 2.0, None, "driver out of holster")
    arm.set_comp(True)
    yield from arm.move(driver_clear_point(p), r, 1.5, None, "driver toward the robot")
    yield from wait(0.3)
    capture_driver(c)
    c.note("driver in hand")


def screw_axis(c, i):
    return np.asarray(c.bench()["screws"]["axis"][i], float)


def washer_x(c, i):
    adv = c.bench()["screws"]["adv"][i]
    return screw_axis(c, i)[0] + L.head_under_x(adv) - L.HEAD_T


def drive_screw(c, i):
    """Set the socket on screw i and turn it in ratchet strokes until the head is down on the cover."""
    a = screw_axis(c, i)
    contact = a + [washer_x(c, i)-a[0], 0., 0.]
    stand = contact - [DRIVER_STANDOFF, 0., 0.]
    far = contact - [DRIVER_FAR, 0., 0.]
    yield from goto_driver(c, far, 4.5, f"driver to screw {i}")
    yield from wait(0.5)
    capture_driver(c)  # the driver turns a little in the fingers when the hand turns it horizontal: measure it again
    yield from track_driver(c, lambda s: (far + s * (stand - far), None), 1.0, f"driver in {i}", correct=False)
    yield from wait(0.3)
    capture_driver(c)
    yield from track_driver(c, lambda s: (stand, None), 1.2, f"driver line-up {i}", gain=0.08)
    c.right.set_comp("hold")
    c.right.enable_driver_tracking(c.GD)
    eng = a + np.array([washer_x(c, i) - a[0] + DRV_PRESS, 0.0, 0.0])
    yield from track_driver(c, lambda s: (stand + s * (eng - stand), None), 1.5, f"socket on {i}", gain=0.06)
    yield from track_driver(c, lambda s: (eng, None), 1.0, f"hold socket {i}", gain=.12)
    _, _, phi = _driver_of_q(c, c.right.q)  # roll at the start of the strokes
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
        yield from track_driver(c, lambda s, w=sweep: (a + [washer_x(c, i)-a[0]+DRV_PRESS, 0., 0.], phi + s * math.radians(w)), dur,
                                f"stroke {i}.{k}", gain=0.04)
        eng = a + np.array([washer_x(c, i) - a[0] + DRV_PRESS, 0.0, 0.0])
        yield from track_driver(c, lambda s, e=eng, w=sweep: (e, phi + (1 - s) * math.radians(w)), 0.85 * dur,
                                f"return {i}.{k}", gain=0.04)
        eng = a + np.array([washer_x(c, i)-a[0]+DRV_PRESS, 0., 0.])
        yield from track_driver(c, lambda s: (eng, phi), .2, f"recenter {i}.{k}", gain=.12)
        c.note("stroke", screw=i, k=k, sweep_deg=sweep)
        if k >= 2 and c.bench()["screws"]["adv"][i] < .00005:
            raise Stop(f"screw {i}: negligible advance after {k+1} strokes")
    if not c.bench()["screws"]["seated"][i]:
        raise Stop(f"screw {i} not seated after {MAX_STROKES} strokes")
    c.right.set_comp(True)
    back = a + np.array([washer_x(c, i) - a[0] - 0.040, 0.0, 0.0])
    pd, _ = c.pose("driver")
    yield from track_driver(c, lambda s: (pd + s * (back - pd), phi), 1.2, f"off screw {i}", correct=False)
    c.note("seated", screw=i)


def release_right(c):
    """Right hand lets go of the handle and backs away along its approach, then straight toward the robot (clear of the
    cover's edge) before going home."""
    arm = c.right
    arm.set_comp("hold")
    _, z = grasp_v("right")
    pc, rc = c.pose()
    yield from arm.set_grip(grip_of(GRASP_HALF + 0.012), 0.8, "right lets go")
    yield from arm.move_max(-0.03 * (rc @ z), 0.8, "right off the handle")
    yield from arm.move_max(np.array([0.0, -0.018, 0.0]), 0.8, "right clear of the edge")
    arm.set_comp(True)
    yield from arm.home(2.0)
    c.note("right released")


def release_left(c):
    arm = c.left
    arm.set_comp("hold")
    _, z = grasp_v("left")
    pc, rc = c.pose()
    yield from arm.set_grip(grip_of(GRASP_HALF + 0.012), 0.8, "left lets go")
    yield from arm.move_max(-0.03 * (rc @ z), 0.8, "left off the handle")
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
    yield from insert_upper_edge(c)
    yield from swing_shut(c)
    c.left.set_comp("hold")
    yield from release_right(c)
    yield from take_driver(c)
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
    yield from insert_upper_edge(c)
    yield from swing_shut(c)
    yield from wait(3.0)


def neg_straight(c):
    """Try a normal-to-panel insertion. This never passes the upper guides at the installation angle."""
    from yam_arena import ik

    yield from wait(.5)
    yield from pick_cover(c)
    pc, rc = c.pose()
    po = c.panel()
    stage = po + [-.050, 0., 0.]
    yield from track(c, lambda s: (pc+s*(stage-pc), ik.slerp_matrix(rc, np.eye(3), s)), 4., "approach flat")
    yield from track(c, lambda s: (stage+s*(po-stage), np.eye(3)), 3., "straight insertion")
    yield from wait(2.)


def neg_held(c):
    """Install the screw but keep the left hand on the cover."""
    yield from wait(.5)
    yield from pick_cover(c)
    yield from insert_upper_edge(c)
    yield from swing_shut(c)
    yield from release_right(c)
    yield from take_driver(c)
    yield from drive_screw(c, 0)
    yield from wait(2.)


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
    "neg-straight": neg_straight,
    "neg-no-screw": hang_only,
    "neg-held": neg_held,
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
                  f"guides {m.get('in_guides')} angle {m.get('guide_angle')} adv {sc.get('adv_mm')} "
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
                  driver_trace=c.driver_trace,
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
