"""Scripted full-score demo (and negative variants) for the bearing-press task in the yam_arena embodiment.

    python demo.py task.py OUT [--max-steps N] [--no-record] [--stage demo|<variant>[,<variant>...]] [--seed N]

Acts ONLY through the 14 joint/gripper commands of ``YamArenaEmbodiment.step``. It reads poses and the press's joint
angles through ``presskit.bench_state()`` (read-only, like an operator watching the parts); it never writes a pose, a
joint state or a score. Its joint commands are rate-limited below the CLI's per-step delta guard and clipped to the
URDF joint limits.

Plan:
1. Left hand: pick the bearing from the tool tray (pinch on its outer ring), set it square in the bore mouth of the
   housing where the housing lies on the press table (closed loop on the bearing and housing poses), let go.
2. Left hand: pick the 41 mm outer-ring driver (the disc that matches the outer ring; the 32 mm disc would press the
   inner ring), set it centred on the bearing, let go.
   Meanwhile the right hand takes the press handle (fingers either side of the bar, approaching from the right).
3. Left hand: fingertips against the housing's side, push the housing across the press table until the bore is under
   the ram (closed loop on the housing pose), and keep the fingertips there.
4. Right hand: swing the handle down along its arc; once the ram bears on the driver the left hand comes away; keep
   pulling until the bearing is on its shoulder (the ram stops: the handle stalls), stop, let go of the handle.
5. Both arms back to READY.
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
SQUEEZE = 0.0025  # fingers commanded 2.5 mm inside the part: about 12.5 N per finger (5000 N/m finger drive)
COMP_GAIN = 0.08
RATE = 0.8  # fraction of the CLI delta guard (0.05 x joint range per step) the script may use
TIP_ABOVE_TCP = 0.013  # pad tips are 13 mm past the TCP


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

    def pose(self, name="bearing"):
        import presskit

        return presskit.bench_state()[name]

    def dofs(self):
        import presskit

        return presskit.bench_state()["dofs"]

    def detail(self):
        import presskit

        return presskit.STATE["detail"]

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


def rot_from(approach, close):
    z = np.asarray(approach, float)
    z = z / np.linalg.norm(z)
    x = np.asarray(close, float)
    x = x - (x @ z) * z
    x = x / np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def down_rot(arm, pos, tilt_deg=0.0, tilt_dir=(1.0, 0.0), yaws=range(0, 180, 15), also=()):
    """Gripper pointing down (optionally tipped by tilt_deg so the fingertips lead along tilt_dir), with the closing
    direction yaw chosen for the least joint-6 travel among IK solutions that reach ``pos`` (and every point in
    ``also`` with the same orientation)."""
    best = None
    td = np.array([tilt_dir[0], tilt_dir[1], 0.0])
    td /= np.linalg.norm(td)
    t = math.radians(tilt_deg)
    ap = math.sin(t) * td + np.array([0.0, 0.0, -math.cos(t)])
    for yaw in yaws:
        for sgn in (1.0, -1.0):
            c = math.radians(yaw)
            r = rot_from(ap, sgn * np.array([math.cos(c), math.sin(c), 0.0]))
            q, pe, re = arm.solve(pos, r)
            if pe > 5e-4 or re > 0.01:
                continue
            if any(max(e[1] / 5e-4, e[2] / 0.01) > 1.0 for e in (fastik.solve(p2, r, q, base=arm.base, iters=150) for p2 in also)):
                continue
            cost = abs(q[5]) / 2.094 + 0.2 * np.abs(q - arm.q).sum()
            if best is None or cost < best[0]:
                best = (cost, r)
    if best is None:
        raise Stop(f"{arm.name}: no down orientation reaches {np.round(pos, 3)}")
    return best[1]


def part_grip_z(bottom_z, tip_above_bottom):
    """TCP height for a pinch whose pad tips are ``tip_above_bottom`` above a part's bottom face."""
    return bottom_z + tip_above_bottom + TIP_ABOVE_TCP


def pick(c, arm, name, radius, tip_above_bottom, label, place_tcp=()):
    """Top-down pinch of a round part lying flat (its pose read from the bench), lift it 40 mm. The closing direction
    is chosen so that the same hand orientation also reaches the TCP points in ``place_tcp`` (where it is put down)."""
    p, _ = c.pose(name)
    over = np.array([p[0], p[1], part_grip_z(p[2], tip_above_bottom) + 0.06])
    try:
        r = down_rot(arm, over - [0, 0, 0.06], also=place_tcp)
    except Stop:
        r = down_rot(arm, over - [0, 0, 0.06])  # place_on() turns the hand about the part's axis on the way
    arm.set_comp(True)
    yield from arm.move(over, r, 2.5, grip_of(radius + 0.012), f"{label}: over")
    p, _ = c.pose(name)
    at = np.array([p[0], p[1], part_grip_z(p[2], tip_above_bottom)])
    yield from arm.move(at + [0, 0, 0.02], r, 1.0, None, f"{label}: down")
    yield from arm.move(at, r, 1.0, None, f"{label}: down")
    yield from arm.set_grip(grip_of(radius - SQUEEZE), 0.8, f"{label}: close")
    yield from wait(0.3)
    yield from arm.move(at + [0, 0, 0.040], r, 1.2, None, f"{label}: lift")
    c.note(f"picked {name}")


def place_on(c, arm, name, target_xy_fn, bottom_z_fn, seated_fn, label, tol=0.00025, speed=0.004):
    """Carry the held part over target_xy_fn(), align it there (closed loop on the part's measured position), lower it
    at ``speed`` with the same xy loop until seated_fn() says it rests on its seat (or 1 mm below the nominal contact),
    then open the hand and lift it away."""
    p, _ = c.pose(name)
    hold = arm.pos - p  # TCP minus part origin (grasp offset, measured)
    z_hi = bottom_z_fn() + 0.030
    tgt = np.r_[target_xy_fn(), z_hi]
    rot = arm.rot
    q, pe, re = arm.solve(tgt + hold, rot)
    if pe > 5e-4 or re > 0.01:  # turn the hand about the vertical (the part is round; TCP on its axis)
        rot = down_rot(arm, tgt + hold, also=[tgt + hold - [0, 0, 0.03]])
    yield from arm.move(tgt + hold, rot, 2.5, None, f"{label}: carry")
    corr = np.zeros(2)
    good = 0
    for k in range(int(4.0 * CONTROL_HZ)):  # align xy over the target
        p, _ = c.pose(name)
        e = target_xy_fn() - p[:2]
        good = good + 1 if np.linalg.norm(e) < tol else 0
        if good >= 5:
            break
        corr = np.clip(corr + 0.3 * e, -0.01, 0.01)
        arm.label = f"{label}: align"
        arm.set_pose(np.r_[target_xy_fn() + corr, z_hi] + hold, arm.rot)
        yield
    z = arm.pos[2]
    z_min = bottom_z_fn() - 0.001 + hold[2]
    for k in range(int(12.0 * CONTROL_HZ)):
        if seated_fn():
            break
        p, _ = c.pose(name)
        e = target_xy_fn() - p[:2]
        corr = np.clip(corr + 0.15 * e, -0.01, 0.01)
        gap = p[2] - bottom_z_fn()
        z = max(z - (speed if gap < 0.004 else 0.015) * DT, z_min)
        arm.label = f"{label}: lower"
        arm.set_pose(np.r_[target_xy_fn() + corr + hold[:2], z], arm.rot)
        yield
    p, _ = c.pose(name)
    c.note(f"{name} at release", err_mm=(np.r_[target_xy_fn() - p[:2], p[2] - bottom_z_fn()] * 1e3).round(3).tolist(),
           seated=bool(seated_fn()))
    arm.set_comp("hold")
    yield from arm.set_grip(grip_of(0.03), 0.6, f"{label}: open")
    yield from wait(0.3)
    arm.set_comp(True)
    yield from arm.move(arm.pos + [0.0, 0.0, 0.02], arm.rot, 0.8, None, f"{label}: up")  # straight up first
    yield from arm.move(arm.pos + [-0.02, 0.0, 0.01], arm.rot, 0.6, None, f"{label}: back")


def housing_xy(c):
    return c.pose("housing")[0][:2]


def housing_top(c):
    return c.pose("housing")[0][2] + L.HOUSING_H


def place_bearing(c):
    arm = c.left
    h = c.pose("housing")[0]
    z = housing_top(c) + 0.0004 + 0.002 + TIP_ABOVE_TCP
    yield from pick(c, arm, "bearing", L.BRG_OD / 2, 0.002, "bearing", [np.r_[h[:2], z], np.r_[h[:2], z + 0.03]])
    def in_mouth():
        m = c.detail().get("measure") or {}
        return m.get("depth_mm", -9.0) >= 0.35 and m.get("off_mm", 9.0) < 0.6

    yield from place_on(c, arm, "bearing", lambda: housing_xy(c), lambda: housing_top(c), in_mouth, "bearing")
    c.note("bearing set")


def place_driver(c):
    arm = c.left
    b = c.pose("bearing")[0]
    z = b[2] + L.BEARING_B + 0.0004 + 0.003 + TIP_ABOVE_TCP
    yield from pick(c, arm, "disc_out", L.DISC_OUT_R, 0.003, "driver", [np.r_[b[:2], z], np.r_[b[:2], z + 0.03]])
    bear_top = lambda: c.pose("bearing")[0][2] + L.BEARING_B  # noqa: E731
    yield from place_on(c, arm, "disc_out", lambda: c.pose("bearing")[0][:2], bear_top,
                        lambda: c.pose("disc_out")[0][2] - bear_top() < 0.0003, "driver")


PUSH_Z = 0.011  # fingertip height above the press table while pushing
PUSH_TILT = 35.0
PUSH_STANDOFF = 0.0015  # aim point of the pad-tip centre behind the housing wall (less than the pad's half thickness)
PUSH_LEAD = 0.004  # the aim point runs this far ahead of the measured housing (the arm's lag otherwise slows the push)


def push_axis(c, u, tol=0.0004, stop_short=0.0, keep=False, label="push", aim=(0.0, 0.0), over=0.05, back_extra=0.02):
    """Closed fingertips against the housing's side opposite u, push it along u (fixed direction) until the housing
    axis reaches the ram axis along u (minus stop_short). The pusher stays on the housing's centre line (closed loop on
    the measured housing). keep=True: stay at the housing afterwards (eased off 1.5 mm), else lift away."""
    arm = c.left
    arm.set_comp(True)
    u = np.asarray(u, float) / np.linalg.norm(u)
    h = housing_xy(c)
    back = L.HOUSING_R + 0.009
    tip_z = L.PLATE_Z + PUSH_Z
    # fingers closing along the push direction: the leading finger's flat outer face meets the housing square on, so
    # the housing moves along u (an oblique finger face would steer it sideways)
    yaw_u = math.degrees(math.atan2(u[1], u[0])) % 180.0
    r = down_rot(arm, np.r_[h - u * (back + back_extra), tip_z + TIP_ABOVE_TCP], PUSH_TILT, u,
                 yaws=[yaw_u], also=[np.r_[L.RAM_XY - u * (L.HOUSING_R + 0.002), tip_z + TIP_ABOVE_TCP]])
    tcp_off = r @ np.array([0, 0, -TIP_ABOVE_TCP])  # TCP relative to the pad tips
    start = np.r_[h - u * (back + back_extra), tip_z] + tcp_off
    yield from arm.move(start + [0, 0, over], r, 2.0, 0.0, f"{label}: over")
    yield from arm.move(start, r, 1.2, None, f"{label}: down")
    speed = 0.010  # m/s, slowing to 2 mm/s over the last 5 mm
    target = L.RAM_XY + np.asarray(aim, float) - u * stop_short
    for k in range(int(25 * CONTROL_HZ)):
        h = housing_xy(c)
        e = target - h
        along = float(e @ u)  # remaining travel along the push direction (the pusher can only push, not pull)
        if along < tol:
            break
        lat = e - along * u
        # steer: push through the housing's centre along a direction tilted toward the lateral error
        ua = u * max(along, 0.004) + 1.5 * lat
        ua = ua / np.linalg.norm(ua)
        want = h - ua * (L.HOUSING_R + PUSH_STANDOFF) + ua * min(PUSH_LEAD, along)
        cur = arm.pos - tcp_off
        step = want - cur[:2]
        nrm = np.linalg.norm(step)
        v = min(speed, max(0.002, 2.0 * along))
        if nrm > v * DT:
            step = step / nrm * v * DT
        # turn the hand about the vertical so the leading finger's face normal points along ua (max 0.5 deg per step)
        ang = math.atan2(float(np.cross(u, ua)), float(u @ ua))
        yaw_now = math.atan2(float(np.cross(u, (arm.rot @ r.T @ np.r_[u, 0.0])[:2])), 1.0)
        dyaw = float(np.clip(ang - yaw_now, -math.radians(0.5), math.radians(0.5)))
        rot = L.rz(math.degrees(dyaw)) @ arm.rot
        tcp_off = rot @ np.array([0, 0, -TIP_ABOVE_TCP])
        arm.label = f"{label}: push"
        arm.set_pose(np.r_[cur[:2] + step, tip_z] + tcp_off, rot)
        yield
    e = L.RAM_XY - housing_xy(c)
    c.note(f"{label} done", err_mm=(e * 1e3).round(3).tolist())
    yield from arm.move(arm.pos - np.r_[u * 0.0015, 0.0], arm.rot, 0.4, None, f"{label}: ease off")
    if keep:
        arm.set_comp("hold")
    else:
        yield from arm.move(arm.pos + [0, 0, 0.04], arm.rot, 0.8, None, f"{label}: up")


SLIDE_TILT = 40.0  # deg: the hand leans back toward +y (away from the ram) while it holds the housing
SLIDE_TIP_Z = 0.008  # pad tips 8 mm above the press table (the housing is 22 mm tall)


def push_housing(c, tol=0.0003, stop_short=0.0):
    """Take the housing by its sides (fingers closing fore-aft, the hand leaning back to the left so it stays clear of
    the ram) and slide it across the press table until its bore is under the ram (closed loop on the measured housing
    pose); keep holding it there until the ram bears on the driver (pull_handle then sends the hand away).
    stop_short: stop that far to the left of the ram axis (negative test)."""
    arm = c.left
    arm.set_comp(True)
    u = np.array([0.0, -1.0])
    h = housing_xy(c)
    tip_z = L.PLATE_Z + SLIDE_TIP_Z
    r = down_rot(arm, np.r_[h, tip_z + TIP_ABOVE_TCP], SLIDE_TILT, u, yaws=[0.0],
                 also=[np.r_[L.RAM_XY, tip_z + TIP_ABOVE_TCP]])
    tcp_off = r @ np.array([0, 0, -TIP_ABOVE_TCP])  # TCP relative to the pad-tip centre
    at = np.r_[h, tip_z] + tcp_off
    yield from arm.move(at + [0, 0.02, 0.05], r, 2.0, grip_of(L.HOUSING_R + 0.012), "slide: over")
    h = housing_xy(c)
    at = np.r_[h, tip_z] + tcp_off
    yield from arm.move(at + [0, 0, 0.02], r, 1.0, None, "slide: down")
    yield from arm.move(at, r, 0.8, None, "slide: down")
    yield from arm.set_grip(grip_of(L.HOUSING_R - SQUEEZE), 0.8, "slide: close")
    yield from wait(0.3)
    p0 = housing_xy(c)
    hold = arm.pos[:2] - p0  # TCP minus housing centre (measured grasp)
    target = L.RAM_XY + np.array([0.0, stop_short])
    corr = np.zeros(2)
    good = 0
    cmd = p0.copy()
    for k in range(int(25 * CONTROL_HZ)):
        h = housing_xy(c)
        e = target - h
        good = good + 1 if np.linalg.norm(e) < tol else 0
        if good >= 6:
            break
        dist = float(np.linalg.norm(target - cmd))
        v = min(0.010, max(0.002, 2.0 * dist))
        if dist > 1e-9:
            cmd = cmd + (target - cmd) * min(1.0, v * DT / dist)
        if dist < 0.003:  # near the end: integrate the measured error (arm sag, slip in the grasp)
            corr = np.clip(corr + 0.15 * e, -0.006, 0.006)
        arm.label = "slide: slide"
        arm.set_pose(np.r_[cmd + corr + hold, arm.pos[2]], arm.rot)
        yield
    arm.set_comp("hold")
    c.note("pushed", centre_mm=float(np.linalg.norm(L.RAM_XY - housing_xy(c)) * 1e3),
           err_mm=((L.RAM_XY - housing_xy(c)) * 1e3).round(3).tolist())


def handle_grasp_pose(theta, r=L.GRIP_R):
    a = math.radians(L.HANDLE_ALPHA0_DEG) - theta
    t = np.array([-math.sin(a), 0.0, -math.cos(a)])  # direction the grip moves as the handle goes down
    return L.handle_point(theta, r), rot_from([0.0, 1.0, 0.0], t)


BAR_SQUEEZE = 0.0015
LEAD_MAX = 0.015  # rad (3.7 mm at the grip): the arm never leads the handle by more, so the finger force stays bounded


def take_handle(c):
    arm = c.right
    arm.set_comp(True)
    th = c.dofs()["theta"]
    p, r = handle_grasp_pose(th)
    yield from arm.move(p + [0, -0.05, 0.01], r, 3.0, grip_of(L.HANDLE_R + 0.012), "handle: approach")
    th = c.dofs()["theta"]
    p, r = handle_grasp_pose(th)
    yield from arm.move(p + [0, -0.05, 0], r, 0.8, None, "handle: approach")
    yield from arm.move(p, r, 2.0, None, "handle: in")
    yield from arm.set_grip(grip_of(L.HANDLE_R - BAR_SQUEEZE), 0.8, "handle: close")
    yield from wait(0.3)
    c.note("handle taken", theta=th)


def pull_handle(c, until="seated", rate=0.25, theta_max=0.95, release_left=True):
    """Swing the handle down at ``rate`` rad/s along its arc (commanded ahead of the measured handle angle by at most
    LEAD_MAX, so the arm stops pressing when the handle stalls). until='seated' stops once the bearing is on its
    shoulder (from the bench state; a person sees the ram stop); a number = stop at that handle angle."""
    arm = c.right
    arm.set_comp(True)
    th_cmd = c.dofs()["theta"]
    left_away = not release_left
    stall = 0
    hist = []
    for k in range(int(30 * CONTROL_HZ)):
        d = c.dofs()
        mech = c.detail().get("mech") or {}
        meas = c.detail().get("measure") or {}
        rb = mech.get("ram_by_n") or {}
        if not left_away and max(rb.get("disc_out", 0.0), rb.get("disc_in", 0.0), rb.get("disc_big", 0.0)) > 10.0:
            left_away = True
            c.gens.append(left_away_gen(c))
            c.note("ram bears", theta=d["theta"], ram_mm=mech.get("ram_mm"))
        if until == "seated" and meas.get("seated"):
            c.note("seated", theta=d["theta"], ram_n=mech.get("ram_drive_n"))
            break
        if isinstance(until, float) and d["theta"] >= until:
            break
        if until == "depth4" and meas.get("depth_mm", 0.0) >= 4.5:
            c.note("stopped early", depth_mm=meas.get("depth_mm"))
            break
        near = meas.get("depth_mm", 0.0) > 6.0  # last mm before the shoulder: slower, so the arm stops at the stall
        lead_max = LEAD_MAX
        lead = th_cmd - d["theta"]
        if lead < lead_max:
            th_cmd += (0.1 if near else rate) * DT
        stall = stall + 1 if lead >= lead_max else 0
        hist.append(d["theta"])
        if len(hist) > 80 and hist[-1] - hist[-81] < 0.003:  # the handle moved < 0.17 deg in 4 s: the ram is stalled
            # (jammed, or on the shoulder)
            c.note("stalled", theta=d["theta"], ram_n=mech.get("ram_drive_n"))
            break
        if theta_max is not None:
            th_cmd = min(th_cmd, theta_max)
        p, r = handle_grasp_pose(th_cmd)
        arm.label = "handle: pull"
        arm.set_pose(p, r)
        yield
    c.note("pull done", dofs=c.dofs(), mech=c.detail().get("mech"))


def left_away_gen(c):
    arm = c.left
    arm.set_comp("hold")
    yield from arm.set_grip(grip_of(L.HOUSING_R + 0.012), 0.6, "left: let go")
    arm.set_comp(True)
    yield from arm.move(arm.pos + [0.0, 0.03, 0.04], arm.rot, 1.2, None, "left: away")
    yield from arm.home(2.5, "left: home")


def let_go_handle(c):
    arm = c.right
    arm.set_comp("hold")
    yield from wait(0.3)
    yield from arm.set_grip(grip_of(L.HANDLE_R + 0.012), 0.8, "handle: open")
    arm.set_comp(True)
    yield from arm.move(arm.pos + [0, -0.05, 0.0], arm.rot, 1.5, None, "handle: out")
    yield from arm.home(2.5, "right: home")


def run_parallel(c, *gens):
    """Run several generators one control step at a time; generators appended to c.gens while running join in."""
    c.gens = list(gens)
    while c.gens:
        done = set()
        for g in list(c.gens):
            try:
                next(g)
            except StopIteration:
                done.add(id(g))
        c.gens = [g for g in c.gens if id(g) not in done]
        if c.gens:
            yield


def demo(c):
    yield from wait(0.3)

    def left_part():
        yield from place_bearing(c)
        yield from place_driver(c)
        yield from push_housing(c)

    yield from run_parallel(c, left_part(), take_handle(c))
    yield from run_parallel(c, pull_handle(c))
    yield from run_parallel(c, let_go_handle(c))
    yield from wait(4.0)


def neg_inner_disc(c):
    """Uses the wrong disc that is in the tray this trial instead of the outer-ring driver (the 32 mm disc bears on the
    inner ring only, the 46 mm plain disc on both rings); otherwise as the demo. Expected: press load through the inner
    ring: brinelled, 0."""
    import presskit

    wrong = presskit.STATE["start"]["wrong"]
    r_wrong = L.DISC_IN_R if wrong == "disc_in" else L.DISC_BIG_R
    yield from wait(0.3)

    def left_part():
        yield from place_bearing(c)
        arm = c.left
        b = c.pose("bearing")[0]
        z = b[2] + L.BEARING_B + 0.0004 + 0.003 + TIP_ABOVE_TCP
        yield from pick(c, arm, wrong, r_wrong, 0.003, "wrong disc", [np.r_[b[:2], z], np.r_[b[:2], z + 0.03]])
        bear_top = lambda: c.pose("bearing")[0][2] + L.BEARING_B  # noqa: E731
        yield from place_on(c, arm, wrong, lambda: c.pose("bearing")[0][:2], bear_top,
                            lambda: c.pose(wrong)[0][2] - bear_top() < 0.0003, "wrong disc")
        yield from push_housing(c)

    yield from run_parallel(c, left_part(), take_handle(c))
    yield from run_parallel(c, pull_handle(c, until="seated", release_left=True))
    yield from run_parallel(c, let_go_handle(c))
    yield from wait(4.0)


def neg_off_centre(c):
    """Pushes the housing only part of the way: the bore stays 4 mm short of the ram axis; then presses as the demo.
    Expected: off-centre press, jammed, 0."""
    yield from wait(0.3)

    def left_part():
        yield from place_bearing(c)
        yield from place_driver(c)
        yield from push_housing(c, stop_short=0.004)

    yield from run_parallel(c, left_part(), take_handle(c))
    yield from run_parallel(c, pull_handle(c, until="seated"))
    yield from run_parallel(c, let_go_handle(c))
    yield from wait(4.0)


def neg_not_home(c):
    """As the demo but stops pulling 2.5 mm before the shoulder (handle let go early): started, not seated, 0.25."""
    yield from wait(0.3)

    def left_part():
        yield from place_bearing(c)
        yield from place_driver(c)
        yield from push_housing(c)

    yield from run_parallel(c, left_part(), take_handle(c))
    yield from run_parallel(c, pull_handle(c, until="depth4"))
    yield from run_parallel(c, let_go_handle(c))
    yield from wait(4.0)


def idle(c):
    yield from wait(8.0)


def probe_press(c):
    """Development (BP_DEV_PRESET=stack): no robot motion; a handle torque ramp applied by the kit's dev hook shows the
    rack and pinion, the fit resistance and the seat."""
    import presskit

    rt = presskit.runtime()
    for k in range(int(30 * CONTROL_HZ)):
        t = k * DT
        rt.dev_torque = min(6.0, 0.4 * t)  # N m on the handle (theta direction)
        if k % 2 == 0:
            ps = {n: c.pose(n) for n in ("housing", "bearing", "disc_out", "plunger")}
            c.note("probe", torque=rt.dev_torque, dofs=c.dofs(), mech=c.detail().get("mech"),
                   veto=c.detail().get("veto"),
                   pos_mm={n: np.round(p_ * 1e3, 3).tolist() for n, (p_, r_) in ps.items()},
                   tilt_w_deg={n: round(math.degrees(math.acos(min(1.0, r_[2, 2]))), 3) for n, (p_, r_) in ps.items()})
        yield
    rt.dev_torque = 0.0
    yield from wait(2.0)


def replay(c):
    """Open-loop check: plays the 14-D commands recorded in a finished run's trace.jsonl (path in BP_REPLAY), step by
    step, without reading anything: what a fixed trajectory scores on another start state."""
    import os

    rows = [json.loads(line) for line in open(os.environ["BP_REPLAY"])]
    c.replay = [r["cmd"] for r in rows]
    for _ in range(len(c.replay) + 40):
        yield


STAGES = {
    "demo": demo,
    "idle": idle,
    "probe-press": probe_press,
    "neg-inner-disc": neg_inner_disc,
    "neg-off-centre": neg_off_centre,
    "neg-not-home": neg_not_home,
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
    import presskit

    rt = presskit.runtime()
    poses = rt.read()
    return dict(
        score=presskit.STATE["score"],
        setup=rt.setup,
        dofs=rt.dofs(),
        bodies={n: np.round(p * 1000, 4).tolist() for n, (p, r) in poses.items()},
        rots={n: np.round(r, 6).tolist() for n, (p, r) in poses.items()},
    )


def reset_report(stages, resets, results):
    import presskit

    rows = []
    for i, r in enumerate(resets):
        want = presskit.authored(
            tuple(r["setup"]) if isinstance(r["setup"], list) else r["setup"]
        )
        dev = max(
            float(np.linalg.norm(np.subtract(r["bodies"][n], want[n][0] * 1000)))
            for n in ("bearing", "housing", "disc_out", "disc_in", "disc_big")
        )
        rdev = float(np.abs(np.subtract(r["rots"]["bearing"], want["bearing"][1])).max())
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
    import presskit
    from inspect_robots import Scene
    from inspect_robots_yam.packing import STATE_KEY

    out.mkdir(parents=True, exist_ok=True)
    t1 = time.time()
    obs = emb.reset(Scene(id="demo", instruction="demo"), seed=seed)
    print(f"BENCH reset {time.time() - t1:.0f} s", flush=True)
    import os

    if os.environ.get("BP_STREAM_WAIT"):  # development runs only: cap the renderer's per-frame RTX streaming wait
        from isaaclab_physx.renderers import isaac_rtx_renderer_utils as rtx_utils

        rtx_utils._STREAMING_WAIT_TIMEOUT_S = float(os.environ["BP_STREAM_WAIT"])
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
    stop_files = [out / "STOP", out.parent / "STOP"]
    gen_s = 0.0
    while steps < max_steps:
        if steps % 10 == 0 and any(f.exists() for f in stop_files):
            stop, ended = "STOP file", steps
            print("STOP file found, ending the trial", flush=True)
            break
        if ended is None:
            tg0 = time.time()
            try:
                next(gen)
            except StopIteration:
                ended = steps
            except Stop as e:
                stop, ended = str(e), steps
                print("STOP", stop, flush=True)
            gen_s += time.time() - tg0
        res = emb.step(c.action())
        jp = np.asarray(res.observation.state[STATE_KEY], float).ravel()
        c.left.observe(jp[:7])
        c.right.observe(jp[7:14])
        steps += 1
        c.step = steps
        info = res.info
        snap(res.observation)
        d = presskit.STATE["detail"]
        row = dict(
            step=steps,
            score=info.get("score"),
            level=presskit.STATE["level"],
            detail=d,
            labels=[c.left.label, c.right.label],
            nonfinite=presskit.STATE["nonfinite"],
            cmd=np.round(np.r_[c.left.last_cmd, c.right.last_cmd], 4).tolist(),
            q=np.round(jp, 4).tolist(),
            dofs={k: round(float(v), 6) for k, v in c.dofs().items()},
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
                f"step {steps} {c.left.label} | {c.right.label} score {info.get('score')} depth {m.get('depth_mm')} "
                f"tilt {m.get('tilt_deg')} off {m.get('off_mm')} centre {m.get('centre_mm')} th {m.get('theta_deg')} "
                f"mech {d.get('mech')} veto {d.get('veto')} "
                f"wall {time.time() - t1:.0f}s gen {gen_s:.1f}s",
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
        scorer=dict(score=presskit.STATE["score"], level=presskit.STATE["level"]),
        final=presskit.STATE["detail"],
        nonfinite=presskit.STATE["nonfinite"],
        looks=c.log,
        wall_s=round(time.time() - t1, 1),
        wall_total_s=round(time.time() - t0, 1),
        worst_ik_mm=dict(
            left=round(c.left.worst_mm, 2), right=round(c.right.worst_mm, 2)
        ),
        physics_dt=presskit.PHYSICS_DT,
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
