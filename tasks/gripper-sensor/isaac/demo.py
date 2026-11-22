"""Scripted full-score demo (and negative variants) for the gripper-sensor task in the yam_arena embodiment.

    python demo.py task.py OUT [--max-steps N] [--no-record] [--stage demo|<variant>[,<variant>...]] [--seed N]

Acts ONLY through the 14 joint/gripper commands of ``YamArenaEmbodiment.step``. It reads what an operator sees
through ``grippkit.bench_state()`` (read-only): the switch's LED, whether the jaws are closed, where the switch sits in
its groove, the screwdriver's pose and whether its blade sits in the screw. It never writes a pose, a joint state or a
score. Its joint commands are rate-limited below the CLI's per-step delta guard and clipped to the URDF joint bounds.

Plan:
1. Left hand: closed fingertips straight down on the valve's manual override and hold it (the jaws close on the part).
2. Right hand: closed fingertips down on the switch's top, then drag it along the groove: first to the open-jaw end of
   the range, then slowly toward the jaws, watching the LED: note where it comes on and where it goes off again, and
   bring the switch back to the middle of that window. Lift the fingertips.
3. Left hand lets go of the override (the jaws open; the LED must go out) and goes back to READY.
4. Right hand: take the screwdriver out of its rack, set its blade on the switch's set screw, push in and turn it
   about 110 deg clockwise (the screw locks at 90 deg), lift the screwdriver clear and hold it.
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
TIP = 0.013  # pad tips past the TCP
OVR_DEPTH = 0.012  # override press: fingertips commanded this far below the button top
DRAG_DEPTH = 0.016  # switch drag: fingertips commanded this far below the switch top
SCAN_V = 0.0018  # m/s: LED search speed (the switch's own position is read every step)
DRAG_V = 0.008  # m/s: repositioning speed
SCAN_START = (
    0.0012  # start the search just past the open-jaw switching point (1 mm + margin)
)
DRV_GRIP_ABOVE_BOTTOM = 0.030  # TCP (pad centre) this far above the handle's bottom end (reach: the arms get
# top-down only to about 0.91 m over the switch)
DRV_LIFT = 0.055  # lift out of the rack (blade 40 mm in the rack hole)
DRV_SQUEEZE = (
    0.0025  # fingers commanded 2.5 mm inside the handle surface (about 12 N per finger)
)
TURN_DEG = 100.0  # screw turn the demo aims for (90 deg locks)
TURN_MAX_DEG = 140.0  # wrist roll available for the turn
TURN_RATE_DEG = 20.0  # deg/s
PUSH_DEPTH = 0.010  # blade push: commanded this far below the screw top
LEAD_MAX = 0.025  # fingertips may lead the switch by up to 25 mm while dragging (the arm's compliance sets the push)
LEAD_GAIN = 4.0  # fingertip lead per metre of remaining switch error
LEAD_KI = (
    10.0  # 1/s: lead integral gain (the push builds up until the switch breaks away)
)
FINGER_V = 0.025  # m/s: fingertip speed limit while dragging
Y_GAIN = 0.10  # per step: sideways correction of the dragging fingertips onto the groove line
PRESS_FROM_FRONT = 0.0085  # fingertip contact on the switch top, 8.5 mm from its front end (behind the screw)
LAT_GAIN = (
    0.05  # blade-tip lateral correction gain (per step) while the blade is on the screw
)
ALIGN_GAIN = (
    0.04  # blade-tip lateral correction gain (per step) in the air over the screw
)
WATCHDOG_S = float(
    os.environ.get("GS_WATCHDOG", "0")
)  # dev runs: abort when a control step takes longer (s)
TURN_GAIN = 0.10  # blade-tip lateral correction gain (per step) while turning
TURN_STROKES = 3  # screwdriver strokes at most (lift, wrist back, turn again)


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
        self.scan = {}
        self.corr = np.zeros(3)

    def state(self):
        import grippkit

        return grippkit.bench_state()

    def detail(self):
        import grippkit

        return grippkit.STATE["detail"]

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
            at=tag,
            step=self.step,
            level=d.get("level"),
            u_mm=d.get("u_switch_mm"),
            travel_mm=d.get("travel_mm"),
            led=d.get("led"),
            jaw_mm=d.get("jaw_travel_mm"),
            screw_deg=d.get("screw_deg"),
        )
        row.update(kw)
        self.log.append(row)
        print("LOOK", json.dumps(row, default=float), flush=True)


def hdown(closing_deg):
    """Straight-down TCP with the fingers closing along the horizontal direction at ``closing_deg`` (from +x)."""
    a = math.radians(closing_deg)
    x = np.array([math.cos(a), math.sin(a), 0.0])
    z = np.array([0.0, 0.0, -1.0])
    return np.column_stack([x, np.cross(z, x), z])


def best_closing(arm, points, candidates):
    """Closing direction (deg) from ``candidates`` for which the arm reaches every (point, extra_yaw) top-down with the
    least wrist-roll excursion."""
    best = None
    for phi in candidates:
        cost, ok = 0.0, True
        for p, dyaw in points:
            q, pe, re = fastik.solve(
                p, hdown(phi + dyaw), arm.q, base=arm.base, iters=200
            )
            if pe > 5e-4 or re > 5e-3 or abs(q[5]) > 2.0:
                ok = False
                break
            cost = max(cost, abs(q[5]))
        if ok and (best is None or cost < best[0]):
            best = (cost, phi)
    if best is None:
        raise Stop(
            f"{arm.name}: no top-down closing direction reaches {len(points)} poses"
        )
    return best[1]


# --------------------------------------------------------------------------------------------- left hand: valve
def press_override(c, depth=OVR_DEPTH):
    arm = c.left
    arm.set_comp(True)
    oc = L.ovr_center()
    above = oc + np.array([0.0, 0.0, TIP + 0.040])
    closing = best_closing(
        arm,
        [(above, 0.0), (oc + np.array([0.0, 0.0, TIP - depth - 0.006]), 0.0)],
        [-90, -75, -105, -60, -120, -45, -135, -30, -150],
    )
    r = hdown(closing)
    yield from arm.move(above, r, 3.0, 0.0, "left over override")
    yield from arm.move(
        oc + np.array([0.0, 0.0, TIP + 0.003]), r, 1.2, None, "left onto override"
    )
    arm.set_comp("hold")
    yield from arm.move(
        oc + np.array([0.0, 0.0, TIP - depth]), r, 1.0, None, "left press override"
    )
    for _attempt in range(3):
        for _ in range(int(1.5 * CONTROL_HZ)):
            yield
            if c.state()["valve_on"]:
                break
        if c.state()["valve_on"]:
            break
        depth += 0.002
        yield from arm.move(
            oc + np.array([0.0, 0.0, TIP - depth]), r, 0.5, None, "left press harder"
        )
    c.note("override held", valve=c.state()["valve_on"], depth_mm=depth * 1e3)


def release_override(c):
    arm = c.left
    yield from arm.move(
        arm.pos + np.array([0.0, 0.0, 0.030]), arm.rot, 1.0, None, "left off override"
    )
    arm.set_comp(True)
    yield from wait(0.5)
    st = c.state()
    c.note("override released", valve=st["valve_on"], led_open=st["led"])
    yield from arm.home(2.5)


# --------------------------------------------------------------------------------------------- right hand: switch
def switch_top_center(c):
    """Where the fingertips press: on the switch's top near its front (screw) end, so the LED at its back end stays
    in view of the right wrist camera."""
    p, _ = c.state()["switch"]
    return np.array(
        [p[0] + L.SW_L / 2 - PRESS_FROM_FRONT, p[1], L.BODY_TOP + L.SW_PROUD]
    )


def onto_switch(c):
    """Closed fingertips down on the switch's top, fingers closing along the groove (the wrist camera then looks at
    the switch from the side, past the fingers, at its LED)."""
    arm = c.right
    arm.set_comp(True)
    top = switch_top_center(c)
    pts = [(top + np.array([0.0, 0.0, TIP + 0.030]), 0.0)] + [
        (np.array([L.switch_center_x(u), L.GROOVE_Y, top[2] + TIP - DRAG_DEPTH]), 0.0)
        for u in (L.U_MIN + 0.004, L.STROKE)
    ]
    closing = best_closing(arm, pts, [0, -15, 15, -30, 30])
    r = hdown(closing)
    yield from arm.move(
        top + np.array([0.0, 0.0, TIP + 0.030]), r, 3.0, 0.0, "right over switch"
    )
    yield from wait(0.3)
    top = switch_top_center(c)
    yield from arm.move(
        top + np.array([0.0, 0.0, TIP + 0.002]), r, 1.2, None, "right onto switch"
    )
    arm.set_comp("hold")
    yield from arm.move(
        top + np.array([0.0, 0.0, TIP - DRAG_DEPTH]), r, 1.0, None, "right press switch"
    )
    yield from wait(0.3)
    c.drag_x = arm.pos[0]


def drag_to(c, u_target, v=DRAG_V, tol=0.0002, max_s=12.0, watch=None, label="drag"):
    """Drag the pressed fingertips along x so that the switch reaches u_target (a number, or a function of the elapsed
    time in s for a moving target; the switch's position is read every step). The arm is compliant along x, so the
    push comes from how far the fingertips lead the switch: lead = LEAD_GAIN x remaining error + its integral
    (LEAD_KI), capped at LEAD_MAX; the push builds up until the switch breaks away and eases off as it arrives. ``v`` limits how fast
    the target itself moves. ``watch(c)`` may end the drag early (returns True)."""
    arm = c.right
    settled = 0
    goal = u_target if callable(u_target) else None
    tgt = c.state()["u_switch"]
    for k in range(int(max_s * CONTROL_HZ)):
        u = c.state()["u_switch"]
        want = goal(k * DT) if goal is not None else u_target
        tgt += float(np.clip(want - tgt, -v * DT, v * DT))
        err = tgt - u
        if goal is None and abs(want - u) < tol and abs(tgt - want) < 1e-9:
            settled += 1
            if settled >= 6:
                return True
        else:
            settled = 0
        if (
            err * c.lead_i < 0.0
        ):  # passed the target: drop the wound-up push at once (no hunting)
            c.lead_i = 0.0
        if abs(err) > tol:
            c.lead_i = float(
                np.clip(c.lead_i + LEAD_KI * err * DT, -LEAD_MAX, LEAD_MAX)
            )
        lead = float(np.clip(LEAD_GAIN * err + c.lead_i, -LEAD_MAX, LEAD_MAX))
        desired = c.x_ref + (u - c.u_ref) + lead
        c.drag_x += float(np.clip(desired - c.drag_x, -FINGER_V * DT, FINGER_V * DT))
        arm.label = label
        # keep the fingertips over the 3 mm switch: the compliant arm drifts sideways under a long push
        y_err = L.GROOVE_Y - float(arm.measured_tcp()[0][1])
        c.drag_y = float(
            np.clip(getattr(c, "drag_y", 0.0) + Y_GAIN * y_err, -0.010, 0.010)
        )
        arm.set_pose(np.array([c.drag_x, L.GROOVE_Y + c.drag_y, arm.pos[2]]), arm.rot)
        yield
        if watch is not None and watch(c):
            return True
    return False


def find_window(c):
    """With the jaws held closed on the part: the switching window lies somewhere between the open-jaw and the
    empty-jaw switching points. Bring the switch to the nearer end of that range, scan across it at SCAN_V while
    watching the LED, and set the switch at the middle of where the LED was on."""
    c.u_ref, c.x_ref, c.lead_i = c.state()["u_switch"], c.drag_x, 0.0
    u0 = c.state()["u_switch"]
    lo, hi = SCAN_START, L.STROKE - SCAN_START
    up = abs(u0 - lo) <= abs(
        u0 - hi
    )  # scan toward the jaws from the open end, or back from the empty end
    yield from drag_to(
        c, lo if up else hi, DRAG_V, max_s=20.0, label="right drag to scan start"
    )
    c.note("scan start", upward=up)
    seen = {"on": None, "off": None}

    def watch(c):
        st = c.state()
        if st["led"] and seen["on"] is None:
            seen["on"] = st["u_switch"]
        if (not st["led"]) and seen["on"] is not None and seen["off"] is None:
            seen["off"] = st["u_switch"]
            return True
        return False

    u_s = c.state()["u_switch"]
    end = hi if up else lo
    yield from drag_to(
        c,
        lambda ts: min(end, u_s + SCAN_V * ts) if up else max(end, u_s - SCAN_V * ts),
        SCAN_V,
        max_s=9.0,
        watch=watch,
        label="right scan for LED",
    )
    c.scan = dict(seen)
    c.note(
        "scan done",
        on_mm=None if seen["on"] is None else seen["on"] * 1e3,
        off_mm=None if seen["off"] is None else seen["off"] * 1e3,
    )
    if seen["on"] is None:
        raise Stop("LED never came on during the scan")
    off = seen["off"] if seen["off"] is not None else c.state()["u_switch"]
    mid = 0.5 * (seen["on"] + off)
    yield from drag_to(
        c, mid, 0.001, tol=0.00025, max_s=8.0, label="right to window middle"
    )
    c.note("set", led_set=c.state()["led"])


def off_switch(c):
    """Ease the push off first (command the fingertips to where they actually are, still pressing: the push stored
    in the compliant arm goes to zero), then lift: lifting off with the push still stored drags the switch."""
    arm = c.right
    x_meas = float(arm.measured_tcp()[0][0])
    x0 = c.drag_x
    for k in range(1, 11):
        c.drag_x = x0 + (x_meas - x0) * k / 10
        arm.label = "right ease off"
        arm.set_pose(np.array([c.drag_x, arm.pos[1], arm.pos[2]]), arm.rot)
        yield
    yield from arm.move(
        arm.pos + np.array([0.0, 0.0, 0.030]), arm.rot, 1.0, None, "right off switch"
    )
    arm.set_comp(True)
    c.note("switch released")


# --------------------------------------------------------------------------------------------- right hand: screwdriver
def driver_tip(c):
    p, r = c.state()["driver"]
    return p, r


def screw_top(c):
    sxy = L.screw_xy(c.state()["u_switch"])
    return np.array([sxy[0], sxy[1], L.BODY_TOP + L.SW_PROUD + L.CUP_PROUD])


def take_driver(c):
    arm = c.right
    arm.set_comp(True)
    p, r = driver_tip(c)
    grasp = p + r[:, 2] * (L.DRV_BLADE_L + DRV_GRIP_ABOVE_BOTTOM)
    st = screw_top(c)
    tcp_over_screw = st + np.array(
        [0.0, 0.0, L.DRV_BLADE_L + DRV_GRIP_ABOVE_BOTTOM + 0.008]
    )
    c.phi = best_closing(
        arm,
        [
            (grasp, 0.0),
            (grasp + [0, 0, DRV_LIFT], 0.0),
            (tcp_over_screw, 0.0),
            (tcp_over_screw, -TURN_MAX_DEG),
        ],
        list(range(-180, 180, 15)),
    )
    r0 = hdown(c.phi)
    open_half = L.DRV_HANDLE_R + 0.012
    yield from arm.move(
        grasp + np.array([0.0, 0.0, 0.050]),
        r0,
        3.0,
        grip_of(open_half),
        "right over driver",
    )
    yield from wait(0.3)
    p, r = driver_tip(c)
    grasp = p + r[:, 2] * (L.DRV_BLADE_L + DRV_GRIP_ABOVE_BOTTOM)
    yield from arm.move(grasp, r0, 1.5, None, "right down to handle")
    yield from wait(0.3)
    yield from arm.set_grip(
        grip_of(L.DRV_HANDLE_R - DRV_SQUEEZE), 1.0, "right grip driver"
    )
    yield from wait(0.5)
    yield from arm.move(
        grasp + np.array([0.0, 0.0, DRV_LIFT]), r0, 2.5, None, "right lift driver"
    )
    yield from wait(0.5)
    p, r = driver_tip(c)
    pt, rt = arm.measured_tcp()
    c.tip_in_tcp = rt.T @ (p - pt)
    c.axis_in_tcp = (
        rt.T @ r[:, 2]
    )  # the screwdriver's axis (toward the handle) in the TCP frame
    c.note(
        "driver taken",
        tilt_deg=math.degrees(math.acos(min(1.0, r[2, 2]))),
        tip_in_tcp_mm=(c.tip_in_tcp * 1e3).tolist(),
    )


def upright(r0, axis_in_tcp):
    """TCP rotation nearest r0 that holds the gripped screwdriver's axis vertical (corrects a tilted grasp)."""
    a = r0 @ np.asarray(axis_in_tcp, float)
    a /= np.linalg.norm(a)
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(a, z)
    s = float(np.linalg.norm(v))
    if s < 1e-9:
        return r0
    return _rotm(v / s * math.atan2(s, float(a @ z))) @ r0


def tool_track(
    c,
    tip_fn,
    yaw_fn,
    duration,
    label,
    gain=0.06,
    until=None,
    max_s=None,
    lateral_only=True,
):
    """Each step: TCP pose that puts the blade tip at tip_fn(s) with the TCP closing direction at yaw_fn(s), plus an
    integral correction of the measured tip error (x, y; and z unless lateral_only)."""
    from yam_arena import ik

    arm = c.right
    n = max(1, round(duration * CONTROL_HZ))
    nmax = n if max_s is None else max(n, round(max_s * CONTROL_HZ))
    for k in range(1, nmax + 1):
        s = ik.smoothstep(min(1.0, k / n))
        tgt = tip_fn(s)
        rot = upright(hdown(yaw_fn(s)), c.axis_in_tcp)
        p, _ = driver_tip(c)
        err = tgt - p
        if lateral_only:
            err[2] = 0.0
        c.corr = np.clip(c.corr + gain * err, -0.008, 0.008)
        arm.label = label
        arm.set_pose(tgt + c.corr - rot @ c.tip_in_tcp, rot)
        yield
        if until is not None and k >= n and until(c):
            break


def steady_over(p, tol=0.0003, n=8):
    """until-condition: the blade tip has stayed within ``tol`` of p (horizontally) for ``n`` steps."""
    count = [0]

    def ok(c):
        count[0] = (
            count[0] + 1 if float(np.hypot(*(driver_tip(c)[0] - p)[:2])) < tol else 0
        )
        return count[0] >= n

    return ok


def engaged_for(n):
    """until-condition: the blade has been engaged in the screw for ``n`` consecutive steps."""
    count = [0]

    def ok(c):
        count[0] = count[0] + 1 if c.state()["engaged"] else 0
        return count[0] >= n

    return ok


def engage(c, above, low, st):
    """Blade down onto the screw with the lateral correction running (it keeps pulling the tip onto the screw axis while
    the arm's compliance shifts it under the contact force); hold the push until engaged; retry from above."""
    yield from wait(
        0.5
    )  # let the arm settle with its joint offsets frozen (the joint integral hunts at standstill)
    yield from tool_track(
        c,
        lambda s: above,
        lambda s: c.phi,
        1.0,
        "right align blade",
        gain=ALIGN_GAIN,
        until=steady_over(above),
        max_s=6.0,
    )
    c.note(
        "blade over screw",
        lat_mm=float(np.hypot(*(driver_tip(c)[0] - above)[:2])) * 1e3,
    )
    touch = st + np.array([0.0, 0.0, 0.0015])
    for attempt in range(4):
        # slow approach to just above the cone (the arm's joint friction shifts the tip when it starts moving; the
        # lateral loop takes that out before contact), then down into the cone and push
        yield from tool_track(
            c,
            lambda s: above + s * (touch - above),
            lambda s: c.phi,
            3.0,
            "right blade to screw",
            gain=ALIGN_GAIN * 2,
        )
        yield from tool_track(
            c,
            lambda s: touch + s * (low - touch),
            lambda s: c.phi,
            1.5,
            "right blade into screw",
            gain=LAT_GAIN,
            until=engaged_for(5),
            max_s=4.0,
        )
        if c.state()["engaged"]:
            break
        c.note("not engaged, retry", attempt=attempt, engage=c.detail().get("engage"))
        yield from tool_track(
            c,
            lambda s: low + s * (above - low),
            lambda s: c.phi,
            1.0,
            "right blade up",
            gain=LAT_GAIN,
        )
        yield from tool_track(
            c,
            lambda s: above,
            lambda s: c.phi,
            1.0,
            "right realign blade",
            gain=ALIGN_GAIN,
            until=steady_over(above),
            max_s=5.0,
        )
    yield from wait(0.3)
    c.note("engaged", engaged=c.state()["engaged"], engage=c.detail().get("engage"))


def turn_stroke(c, low):
    """Turn clockwise (seen from above) until the screw has turned TURN_DEG (the slot's turn is visible), within the
    wrist's roll range."""
    c.turned = 0.0

    def turn_step(s):
        if c.state()["screw_deg"] < TURN_DEG and c.turned < TURN_MAX_DEG:
            c.turned += TURN_RATE_DEG / CONTROL_HZ
        return c.phi - c.turned

    yield from tool_track(
        c,
        lambda s: low,
        turn_step,
        1.0,
        "right turn screw",
        gain=TURN_GAIN,
        until=lambda c: c.state()["screw_deg"] >= TURN_DEG or c.turned >= TURN_MAX_DEG,
        max_s=12.0,
    )
    yield from wait(0.3)
    c.note("turned", screw_now=c.state()["screw_deg"], wrist_turn_deg=c.turned)


def tighten(c):
    arm = c.right
    c.corr = np.zeros(3)
    st = screw_top(c)
    p0, _ = driver_tip(c)
    above = st + np.array([0.0, 0.0, 0.007])
    high = st + np.array([0.0, 0.0, 0.012])
    p0h = np.array([p0[0], p0[1], max(p0[2], high[2])])
    yield from tool_track(
        c, lambda s: p0 + s * (p0h - p0), lambda s: c.phi, 1.0, "right driver up"
    )
    yield from tool_track(
        c,
        lambda s: p0h + s * (high - p0h),
        lambda s: c.phi,
        3.0,
        "right driver over screw",
    )
    yield from tool_track(
        c,
        lambda s: high + s * (above - high),
        lambda s: c.phi,
        1.5,
        "right driver down",
    )
    arm.set_comp("hold")
    low = st - np.array([0.0, 0.0, PUSH_DEPTH])
    for stroke in range(TURN_STROKES):
        # a screwdriver is turned in strokes: if the wrist's roll runs out before the screw is tight, lift the blade,
        # turn the wrist back, set the blade in again and turn again
        yield from engage(c, above, low, st)
        yield from turn_stroke(c, low)
        if c.state()["screw_deg"] >= TURN_DEG:
            break
        c.note("next stroke", stroke=stroke)
        yield from tool_track(
            c,
            lambda s: low + s * (above - low),
            lambda s: c.phi - c.turned,
            1.0,
            "right blade up",
            gain=LAT_GAIN,
        )
        t0 = c.turned
        yield from tool_track(
            c,
            lambda s: above,
            lambda s, t0=t0: c.phi - t0 * (1 - s),
            2.0,
            "right wrist back",
            gain=ALIGN_GAIN,
        )
    top = arm.pos + np.array(
        [0.0, 0.0, 0.015]
    )  # blade 15 mm clear of the screw (the arm's reach ends ~0.91 m)
    yield from arm.move(top, arm.rot, 1.0, None, "right lift driver off")
    yield from arm.move(
        top + np.array([-0.040, 0.0, 0.0]), arm.rot, 1.5, None, "right driver back"
    )
    arm.set_comp(True)
    c.note("driver off screw")
    # take the arm (still holding the screwdriver) back out of the way, so the gripper is in plain view
    yield from arm.home(3.0, "right home with driver")


# --------------------------------------------------------------------------------------------- stages
def set_switch(c):
    yield from wait(0.3)
    yield from both(press_override(c), onto_switch(c))
    yield from find_window(c)
    yield from off_switch(c)
    yield from release_override(c)


def driver_only(c):
    """Dev stage: the screwdriver part alone (switch left where it starts)."""
    yield from wait(0.3)
    yield from take_driver(c)
    yield from tighten(c)
    yield from wait(2.0)


def demo(c):
    yield from set_switch(c)
    yield from take_driver(c)
    yield from tighten(c)
    yield from wait(2.0)


def probe(c):
    """M1 probe: hold the override, press the switch and drag it +4 mm then -4 mm, release."""
    yield from wait(0.3)
    yield from both(press_override(c), onto_switch(c))
    c.u_ref, c.x_ref, c.lead_i = c.state()["u_switch"], c.drag_x, 0.0
    u0 = c.state()["u_switch"]
    yield from drag_to(c, u0 + 0.004, DRAG_V, label="probe +4")
    c.note("probe +4")
    yield from drag_to(c, u0 - 0.004, DRAG_V, label="probe -4")
    c.note("probe -4")
    yield from off_switch(c)
    yield from release_override(c)


def idle(c):
    yield from wait(2.0)


def neg_no_lock(c):
    """Set correctly but never locked: 0.75."""
    yield from set_switch(c)
    yield from wait(2.0)


def neg_end_of_stroke(c):
    """The common mistake: the switch set where it lights with the jaws closed EMPTY (no part) -- here at the
    end-of-stroke position, found by the same LED scan with the part's switching point skipped -- then locked: 0."""
    yield from wait(0.3)
    yield from both(press_override(c), onto_switch(c))
    c.u_ref, c.x_ref, c.lead_i = c.state()["u_switch"], c.drag_x, 0.0
    yield from drag_to(c, L.STROKE, DRAG_V, label="right drag to end of stroke")
    c.note("at end-of-stroke point")
    yield from off_switch(c)
    yield from release_override(c)
    yield from take_driver(c)
    yield from tighten(c)
    yield from wait(2.0)


def neg_no_override(c):
    """Never works the override: the LED cannot be seen on the part; the switch is dragged by a fixed amount (+6 mm)
    and locked: 0 unless the seed happens to put the window there."""
    yield from wait(0.3)
    yield from onto_switch(c)
    c.u_ref, c.x_ref, c.lead_i = c.state()["u_switch"], c.drag_x, 0.0
    u0 = c.state()["u_switch"]
    yield from drag_to(c, u0 + 0.006, DRAG_V, label="right blind drag")
    c.note("blind drag done")
    yield from off_switch(c)
    yield from take_driver(c)
    yield from tighten(c)
    yield from wait(2.0)


def neg_light_touch(c):
    """Set correctly, then the screwdriver only rests on the screw (no push) while turning: not locked, 0.75."""
    global PUSH_DEPTH
    yield from set_switch(c)
    yield from take_driver(c)
    old = PUSH_DEPTH
    PUSH_DEPTH = 0.0005  # tip commanded only 0.5 mm into the screw top: a light touch (well under 1.5 N)
    try:
        yield from tighten(c)
    finally:
        PUSH_DEPTH = old
    yield from wait(2.0)


def replay(c):
    """Open-loop check: plays the 14-D commands recorded in a finished run's trace.jsonl (path in GS_REPLAY), step by
    step, without reading anything: what a fixed trajectory scores on another start state."""
    import os

    rows = [json.loads(line) for line in open(os.environ["GS_REPLAY"])]
    c.replay = [r["cmd"] for r in rows]
    for _ in range(len(c.replay) + 40):
        yield


STAGES = {
    "demo": demo,
    "probe": probe,
    "driver-only": driver_only,
    "idle": idle,
    "replay": replay,
    "neg-no-lock": neg_no_lock,
    "neg-end-of-stroke": neg_end_of_stroke,
    "neg-no-override": neg_no_override,
    "neg-light-touch": neg_light_touch,
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
    import grippkit

    rt = grippkit.runtime()
    poses = rt.read()
    return dict(
        score=grippkit.STATE["score"],
        setup=rt.setup,
        u_switch_mm=round(rt.u_sw * 1e3, 4),
        jaw_mm=round(rt.travel * 1e3, 4),
        screw_deg=rt.screw_deg,
        valve_on=rt.valve_on,
        bodies={n: np.round(p * 1000, 4).tolist() for n, (p, r) in poses.items()},
        rots={n: np.round(r, 6).tolist() for n, (p, r) in poses.items()},
    )


def reset_report(stages, resets, results):
    import grippkit

    rows = []
    for i, r in enumerate(resets):
        setup = tuple(r["setup"]) if isinstance(r["setup"], list) else r["setup"]
        want = grippkit.authored(setup)
        dev = max(
            float(np.linalg.norm(np.subtract(r["bodies"][n], want[n][0] * 1000)))
            for n in ("switch", "screw", "jaw_p", "jaw_n", "driver")
        )
        rdev = float(np.abs(np.subtract(r["rots"]["driver"], want["driver"][1])).max())
        rows.append(
            dict(
                trial=i,
                stage=stages[i],
                setup=r["setup"],
                score_after_reset=r["score"],
                screw_deg_after_reset=r["screw_deg"],
                valve_after_reset=r["valve_on"],
                max_body_dev_from_authored_mm=round(dev, 4),
                max_rot_dev=round(rdev, 6),
                trial_score=results[i]["task_score"],
                trial_stop=results[i]["stop"],
            )
        )
    ok = all(
        x["score_after_reset"] == 0.0
        and x["max_body_dev_from_authored_mm"] < 0.5
        and x["screw_deg_after_reset"] == 0.0
        and not x["valve_after_reset"]
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
    import grippkit
    from inspect_robots import Scene
    from inspect_robots_yam.packing import STATE_KEY

    out.mkdir(parents=True, exist_ok=True)
    t1 = time.time()
    obs = emb.reset(Scene(id="demo", instruction="demo"), seed=seed)
    print(f"BENCH reset {time.time() - t1:.0f} s", flush=True)
    t2 = time.time()
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
            except Exception as e:  # noqa: BLE001  (a script error ends the script, not the trial record)
                import traceback

                traceback.print_exc()
                stop, ended = f"script error: {type(e).__name__}: {e}", steps
                print("STOP", stop, flush=True)
        res = emb.step(c.action())
        jp = np.asarray(res.observation.state[STATE_KEY], float).ravel()
        c.left.observe(jp[:7])
        c.right.observe(jp[7:14])
        steps += 1
        c.step = steps
        info = res.info
        snap(res.observation)
        d = grippkit.STATE["detail"]
        row = dict(
            step=steps,
            score=info.get("score"),
            level=grippkit.STATE["level"],
            detail=d,
            labels=[c.left.label, c.right.label],
            nonfinite=grippkit.STATE["nonfinite"],
            cmd=np.round(np.r_[c.left.last_cmd, c.right.last_cmd], 4).tolist(),
            q=np.round(jp, 4).tolist(),
            corr_mm=np.round(c.corr * 1000, 3).tolist(),
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
                f"step {steps} {c.left.label} | {c.right.label} score {info.get('score')} u {d.get('u_switch_mm')} "
                f"part {d.get('travel_mm')} jaw {d.get('jaw_travel_mm')} led {d.get('led')} ovr {d.get('ovr_n')} "
                f"fsw {d.get('sw_force_n')} eng {d.get('engaged')} screw {d.get('screw_deg')} "
                f"wall {time.time() - t1:.0f}s",
                flush=True,
            )
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if steps % 20 == 0 and (out / "STOP").exists():
            stop = "STOP file"
            print("STOP", stop, flush=True)
            break
        if WATCHDOG_S and 3 <= steps <= 40 and (time.time() - t2) / steps > WATCHDOG_S:
            # dev runs only (GS_WATCHDOG): the simulator runs pathologically slowly on this host now; give up early
            stop = f"watchdog: {(time.time() - t2) / steps:.1f} s per control step"
            print("STOP", stop, flush=True)
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
        scorer=dict(score=grippkit.STATE["score"], level=grippkit.STATE["level"]),
        final=grippkit.STATE["detail"],
        nonfinite=grippkit.STATE["nonfinite"],
        looks=c.log,
        wall_s=round(time.time() - t1, 1),
        wall_total_s=round(time.time() - t0, 1),
        worst_ik_mm=dict(
            left=round(c.left.worst_mm, 2), right=round(c.right.worst_mm, 2)
        ),
        physics_dt=grippkit.PHYSICS_DT,
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
