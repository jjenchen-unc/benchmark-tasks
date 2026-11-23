"""Scripted full-score demo (and negative variants) for the nm-cable-box task in the yam_arena embodiment.

    python demo.py task.py OUT [--max-steps N] [--no-record] [--stage demo|<variant>] [--seed S]

Acts ONLY through the 14 joint/gripper commands of ``YamArenaEmbodiment.step``. It reads the cable link poses, the box
pose and the clamp tab angle through ``cablekit.bench_state()`` (read-only, like an electrician watching the cable
end, the entry and the length inside); it never writes a pose, a joint state or a score. Its joint commands are
rate-limited below the CLI's per-step delta guard and clipped to the URDF joint bounds.

Plan:
1. Right hand: top-down pinch across the flat cable 40 mm behind its cut end; lift it, turn the end square to the
   box's entry (cable flat, as the 20 x 10 mm opening needs) and bring it level with the opening, outside the box.
2. Align the end with the opening (closed loop on the measured end pose) and push it through the clamp tab (about
   15 N) until about 20 mm is in; open, slide the grip 40 mm back along the cable (the tab's bite holds the cable),
   close and push again until about 55 mm is in.
3. Left hand: reach into the box from above and pinch the cable 12 mm from its cut end; the right hand lets go and
   moves away. Draw the cable up out of the box face, watching the length past the clamp, until it is 150 mm.
4. Open, lift clear, park both arms out of the overhead view.
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
SQUEEZE = float(__import__("os").environ.get("NMB_SQUEEZE", "0.006"))  # fingers commanded 6 mm inside the sheath edges: about 30 N per finger (gripper 5 kN/m)
OPEN_CLEAR = 0.010
GRIP_S = 0.032  # right hand's grip, from the cut end
R_TILT = 35.0  # right hand's approach leans this far toward the cut end (its gripper body stays clear of the box)
REGRIP_BACK = 0.035
LEFT_S = 0.010
LEFT_OPEN = 0.003  # left fingers open only 3 mm past the sheath: their wider upper part must clear the box sides  # left hand's grip, from the cut end
PAD_ABOVE = 0.0012  # pad tips this far above the surface the cable lies on
PRE_GAP = 0.025  # cut end this far outside the entry wall's outer face for the final alignment
AIM_DZ = 0.0045  # cable centre at mid-opening height when presented (bottom 2.1 mm above the lower edge)  # cable centre this far above the opening's lower edge
PUSH1_IN = 0.015
PUSH_PITCH_DEG = 0.0
EDGE_CLEAR = 0.0015  # cable centre line held this far above the line of the opening's lower edge while pushing
PUSH_S = 3.5
PUSH_DZ = L.CABLE_T / 2 + 0.0003  # push target: the cable just above the opening's lower edge; the tab presses it on
PUSH2_IN = 0.050
DRAW_TO = 0.150
OVERSHOOT_TO = 0.188  # neg-overshoot (past the 130-170 mm window)
GRIP_FAR = 0.150  # neg-push-far
DRAW_Y_END = 0.060
GUIDE_LEAD = 0.015
LEFT_AWAY_YAW = 0.8  # left arm ends at READY turned 0.8 rad toward +y: clear of the box in the overhead view  # left guide leads the measured length by this much (it draws while the right feeds)  # left grip path end: this far past the entry wall's inner face
DRAW_Z_END = L.BOX_RIM_Z + 0.022
PARK = (0.170, 0.230, 0.880)  # parked TCP (left; right mirrored), top-down
ROT_CORR_MAX = 0.20
_FF = __import__("os").environ.get("NMB_FLIP")
FORCE_FLIP = None if _FF is None else bool(int(_FF))  # development knob


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
        rail before the fingers open."""
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
        if pe > 0.004 or re > math.radians(8):
            raise Stop(
                f"{self.name} unreachable at {self.label}: {pe * 1000:.1f} mm {math.degrees(re):.1f} deg"
            )
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.last_re_deg = math.degrees(re)
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

    def home(self, duration=2.5, label="", yaw=0.0):
        """Joint-space move to READY (base yaw offset by ``yaw`` rad: turned away from the box, clear of the overhead
        view)."""
        from yam_arena import ik
        from yam_arena.constants import READY

        self.label = label or f"{self.name} home"
        q0, q1 = self.q.copy(), np.array(READY, float) + np.array([yaw, 0, 0, 0, 0, 0])
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

    def bench(self):
        import cablekit

        return cablekit.bench_state()

    def detail(self):
        import cablekit

        return cablekit.STATE["detail"]

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


# --------------------------------------------------------------------------------------------- cable geometry
def cable(c):
    b = c.bench()
    spec = b["spec"]
    if getattr(c, "blind", False):  # neg-blind-aim: aims at the box's nominal place, not where the box is seen
        spec = dict(spec, box_dx=0.0, box_dy=0.0)
    return b["links"], spec


def point_at(c, s):
    """Centre-line point at arc length s from the cut end, the cable direction there (toward the cut end) and the
    width axis of the link there."""
    links, _ = cable(c)
    pts = L.cable_points(links)
    acc = 0.0
    for i in range(len(pts) - 1):
        seg = float(np.linalg.norm(pts[i + 1] - pts[i]))
        if acc + seg >= s or i == len(pts) - 2:
            t = (s - acc) / max(seg, 1e-9)
            p = pts[i] + t * (pts[i + 1] - pts[i])
            r = links[i][1]
            return p, r[:, 0], r[:, 1]
        acc += seg


def end_frame(c):
    """Cut-end frame: origin at the cut end's centre, rotation = the lead link's (x along the cable toward the end)."""
    links, _ = cable(c)
    p, r = links[0]
    return p + r[:, 0] * L.LEAD_L / 2, r


def inserted(c):
    links, spec = cable(c)
    return L.inserted_length(links, spec)


def wall_height(c, y):
    """Height of the cable's centre line where it crosses the plane at y (None if it does not)."""
    links, _ = cable(c)
    pts = L.cable_points(links)
    for i in range(len(pts) - 1):
        a, b = pts[i], pts[i + 1]
        if (a[1] - y) * (b[1] - y) <= 0 and abs(a[1] - b[1]) > 1e-9:
            t = (a[1] - y) / (a[1] - b[1])
            return float(a[2] + t * (b[2] - a[2]))
    return None


def topdown_rot(width_axis, flip=False, lean=None, tilt_deg=0.0):
    """Downward TCP with the fingers closing along the (horizontal part of the) given axis; ``lean`` (horizontal
    direction, perpendicular to the closing axis) tilts the approach that way by tilt_deg, so the gripper body leans
    back the other way."""
    x = np.array([width_axis[0], width_axis[1], 0.0]) * (-1.0 if flip else 1.0)
    x /= np.linalg.norm(x)
    z = np.array([0.0, 0.0, -1.0])
    if lean is not None and tilt_deg:
        d = np.array([lean[0], lean[1], 0.0])
        d -= (d @ x) * x
        d /= np.linalg.norm(d)
        z = z + math.tan(math.radians(tilt_deg)) * d
        z /= np.linalg.norm(z)
    return np.column_stack([x, np.cross(z, x), z])


def best_rot(arm, p, width_axis):
    best = None
    for flip in (False, True):
        r = topdown_rot(width_axis, flip)
        q, pe, re = arm.solve(p, r)
        cost = (abs(q[5]) > 1.95) + 10.0 * (pe > 0.002 or re > 0.03) + 0.01 * flip + 0.05 * abs(q[5])
        if best is None or cost < best[0]:
            best = (cost, r)
    return best[1]


def tcp_for_tips(p_tips, rot=None):
    """TCP position that puts the pad tips at p_tips (the tips are PAD_TIP_PAST_TCP along the approach axis)."""
    from yam_arena.constants import PAD_TIP_PAST_TCP

    z = np.array([0.0, 0.0, -1.0]) if rot is None else np.asarray(rot)[:, 2]
    return np.asarray(p_tips, float) - z * PAD_TIP_PAST_TCP


def capture(c, hand):
    pe, re = end_frame(c)
    pt, rt = c.arms[hand].measured_tcp()
    c.G[(hand, "end")] = (re.T @ (pt - pe), re.T @ rt)


def entry_rot():
    """Cable end square to the entry: along +y, width along -x, flat face up."""
    return np.column_stack([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])


def track(c, hand, target_fn, duration, label, gain=0.05, rot_gain=0.03, free=(False, False, False), until=None,
          max_s=None):
    """Closed loop on the cable's cut end, held by ``hand``: each step command the TCP that puts the end at
    target_fn(s) through the frozen grasp transform, plus slow integral corrections of the measured end error."""
    from yam_arena import ik

    n = max(1, round(duration * CONTROL_HZ))
    nmax = n if max_s is None else max(n, round(max_s * CONTROL_HZ))
    key = ("end", (hand,))
    cp, cr = c.corr.get(key, (np.zeros(3), np.zeros(3)))
    for k in range(1, nmax + 1):
        pt, rt = target_fn(ik.smoothstep(min(1.0, k / n)))
        pm, rm = end_frame(c)
        err = np.where(np.asarray(free), 0.0, pt - pm)
        cp = np.clip(cp + gain * err, -0.025, 0.025)
        e_r = _rotvec(rt @ rm.T)
        cr = np.clip(cr + rot_gain * e_r, -ROT_CORR_MAX, ROT_CORR_MAX)
        rc = _rotm(cr)
        gp, gr = c.G[(hand, "end")]
        arm = c.arms[hand]
        arm.label = label
        arm.set_pose(pt + cp + rc @ rt @ gp, rc @ rt @ gr)
        c.corr[key] = (cp, cr)
        yield
        if until is not None and k >= n and until(c):
            break


# --------------------------------------------------------------------------------------------------- the script
def pick_right(c):
    arm = c.right
    arm.set_comp(True)
    p, ax, wy = point_at(c, GRIP_S)
    tips = np.array([p[0], p[1], L.TOP + PAD_ABOVE])
    # grasp: approach leaning toward the cut end (gripper body back, clear of the box wall when pushing); flip
    # feasible both here and at the presentation pose in front of the entry (end along +y)
    _, spec = cable(c)
    ec = L.entry_centre(spec)
    pres_tips = np.array([ec[0], ec[1] - PRE_GAP - GRIP_S, L.ENTRY_Z0 + AIM_DZ - L.CABLE_T / 2 + PAD_ABOVE])
    best = None
    for flip in (False, True):
        r_pick = topdown_rot(wy, flip, ax, R_TILT)
        sx = 1.0 if (wy @ r_pick[:, 0]) > 0 else -1.0
        r_pres = topdown_rot(np.array([-1.0, 0.0, 0.0]) * sx, False, np.array([0.0, 1.0, 0.0]), R_TILT)
        cost = 0.0
        push_tips = pres_tips + [0, PRE_GAP + L.BOX_WALL + PUSH1_IN, 0]
        for pp, rr in ((tcp_for_tips(tips, r_pick), r_pick), (tcp_for_tips(pres_tips, r_pres), r_pres),
                       (tcp_for_tips(pres_tips, r_pres) + [0, 0, 0.012], r_pres), (tcp_for_tips(push_tips, r_pres), r_pres)):
            q, pe, re = arm.solve(pp, rr)
            cost += 1000.0 * pe + 10.0 * re + 0.05 * abs(q[5]) + 20.0 * max(0.0, abs(q[5]) - 1.6)  # wrist roll margin
        if FORCE_FLIP is not None and flip != FORCE_FLIP:
            cost += 1e6
        c.note("flip option", flip=flip, cost=round(cost, 3))
        if best is None or cost < best[0]:
            best = (cost, r_pick)
    rot = best[1]
    half = L.CABLE_W / 2
    yield from arm.move(tcp_for_tips(tips, rot) + [0, 0, 0.07], rot, 3.0, grip_of(half + OPEN_CLEAR), "right over grip")
    yield from wait(0.3)
    p, ax, wy = point_at(c, GRIP_S)
    tips = np.array([p[0], p[1], L.TOP + PAD_ABOVE])
    yield from arm.move(tcp_for_tips(tips, rot), rot, 2.0, None, "right descend")
    yield from wait(0.3)
    yield from arm.set_grip(grip_of(half - SQUEEZE), 1.0, "right close")
    yield from wait(0.5)
    c.note("right gripped")
    c.grip_s = GRIP_S
    yield from arm.move(arm.pos + [0, 0, 0.035], arm.rot, 1.5, None, "right lift")
    yield from wait(0.4)
    capture(c, "right")


def present(c):
    """Turn the end square to the entry and bring it in front of the opening, PRE_GAP outside the wall."""
    _, spec = cable(c)
    ec = L.entry_centre(spec)
    p0, r0 = end_frame(c)
    re = entry_rot()
    pa = np.array([ec[0], ec[1] - PRE_GAP - 0.020, ec[2] + 0.010])
    from yam_arena import ik

    yield from track(c, "right", lambda s: (p0 + s * (pa - p0), ik.slerp_matrix(r0, re, s)), 4.0, "present carry",
                     gain=0.04, rot_gain=0.0)
    pb = np.array([ec[0], ec[1] - PRE_GAP, L.ENTRY_Z0 + AIM_DZ])
    yield from track(c, "right", lambda s: (pa + s * (pb - pa), re), 2.0, "present lower", gain=0.05, rot_gain=0.0)

    def ok(c, cnt=[0]):
        pm, rm = end_frame(c)
        e = pb - pm
        good = abs(e[0]) < 0.0015 and abs(e[2]) < 0.0015
        cnt[0] = cnt[0] + 1 if good else 0
        return cnt[0] >= 4

    yield from track(c, "right", lambda s: (pb, re), 1.0, "present align", gain=0.06, rot_gain=0.0, until=ok, max_s=4.0)
    pm, rm = end_frame(c)
    c.note("aligned", err_mm=np.round((pb - pm) * 1e3, 2).tolist())


def push_in(c, depth, label):
    """Push the end along +y until ``depth`` of cable is past the wall's inner face (closed loop on x/z only)."""
    _, spec = cable(c)
    ec = L.entry_centre(spec)
    y_in = L.entry_wall_y(spec)
    re = L.rx(math.radians(-PUSH_PITCH_DEG)) @ entry_rot()  # end pitched slightly down: it rides under the tab
    p0, _ = end_frame(c)
    pt = np.array([ec[0], y_in + depth + 0.010, L.ENTRY_Z0 + PUSH_DZ])  # cable pressed onto the opening's lower edge
    c.right.set_comp(True)
    saved = c.right.offset.copy()
    c.right.offset_clip = 0.60

    def deep(c):
        v = inserted(c)
        return v is not None and v >= depth

    pa = np.array([ec[0], y_in + 0.004, L.ENTRY_Z0 + AIM_DZ])

    def past(c):
        return end_frame(c)[0][1] >= y_in + 0.003

    # phase A: into the opening at mid height, closed loop on x and z
    if not past(c):
        yield from track(c, "right", lambda s: (p0 + s * (pa - p0), re), 1.5, label + " in", gain=0.05,
                         rot_gain=0.0, free=(False, True, False), until=past, max_s=4.0)
    # phase B: the pads pinch the sheath's narrow edges, so the grip is a hinge about the closing axis: the lead
    # section's angle is set by the grip height against the opening's lower edge. Hold the grip point level with
    # the edge (closed loop on the end's x), advance along +y, let the end ride under the tab.
    arm = c.right
    s_grip = getattr(c, "grip_s", GRIP_S)
    gp0, _, _ = point_at(c, s_grip)
    tcp0, _ = arm.measured_tcp()
    d = tcp0 - gp0  # TCP relative to the grip point
    rot = arm.rot.copy()
    z_grip = L.ENTRY_Z0 + L.CABLE_T / 2 + EDGE_CLEAR
    cx = 0.0
    hz = 0.0
    adv = max(0.004, depth + 0.010 - (inserted(c) or 0.0))
    links_, _ = cable(c)
    ax0 = links_[1][1][:, 0]
    tan_p = float(np.clip(ax0[2] / max(np.hypot(ax0[0], ax0[1]), 1e-6), -0.25, 0.25))  # slope frozen at the start
    n = round(PUSH_S * CONTROL_HZ)
    for k in range(1, round(14.0 * CONTROL_HZ) + 1):
        from yam_arena import ik

        sfr = ik.smoothstep(min(1.0, k / n))
        pe_, _ = end_frame(c)
        cx = float(np.clip(cx + 0.05 * (ec[0] - pe_[0]), -0.01, 0.01))
        # the cable runs straight through the pads (pitched by its slope down to the bench behind the hand): hold the
        # grip so that the cable's centre line crosses the wall just above the opening's lower edge, which then
        # carries the tab's downward push (otherwise the lead is a cantilever from the pads and kinks)
        yg = gp0[1] + sfr * adv
        y_wall = y_in - L.BOX_WALL / 2
        z_line = z_grip - (y_wall - yg) * tan_p
        # closed loop on the measured cable centre height where it crosses the wall (next to the edge fulcrum)
        zw = wall_height(c, y_wall)
        if zw is not None:
            hz = float(np.clip(hz + 0.25 * (z_grip - zw), -0.012, 0.012))
        g = np.array([ec[0] + cx, yg, gp0[2] + min(1.0, k / 10.0) * (z_line - gp0[2]) + hz])
        arm.label = label
        arm.set_pose(g + d, rot)
        if getattr(c, "follow", None) is not None:
            c.follow(c)
        yield
        if k >= n // 2 and deep(c):
            break
    c.right.offset_clip = 0.25
    c.right.offset = saved  # stop pushing: back to the pre-push sag offsets, commanded where the arm is
    c.right.relax()
    yield from wait(0.3)
    c.note(label, inserted_mm=None if inserted(c) is None else round(inserted(c) * 1e3, 2))


def regrip_right(c):
    arm = c.right
    half = L.CABLE_W / 2
    arm.relax()
    yield from arm.set_grip(grip_of(half + OPEN_CLEAR), 0.8, "right open")
    arm.set_comp(True)
    # once let go, the outside part drops (it pivots on the opening's edge): find the sheath again REGRIP_BACK further
    # back along it, from its measured pose, and take it from above as at the first pick
    rot = arm.rot.copy()
    s_new = getattr(c, "grip_s", GRIP_S) + REGRIP_BACK
    yield from arm.move(arm.pos + [0, 0, 0.015], rot, 0.5, None, "right up")
    p, _, _ = point_at(c, s_new)
    tips = np.array([p[0], p[1], p[2] - L.CABLE_T / 2 + PAD_ABOVE])
    yield from arm.move(tcp_for_tips(tips, rot) + [0, 0, 0.020], rot, 1.1, None, "right over cable")
    p, _, _ = point_at(c, s_new)
    tips = np.array([p[0], p[1], p[2] - L.CABLE_T / 2 + PAD_ABOVE])
    yield from arm.move(tcp_for_tips(tips, rot), rot, 0.7, None, "right down")
    yield from arm.set_grip(grip_of(half - SQUEEZE), 0.8, "right close 2")
    yield from wait(0.3)
    c.corr.pop(("end", ("right",)), None)
    c.grip_s = s_new
    capture(c, "right")


def take_left(c):
    """Left hand pinches the cable LEFT_S from its cut end, inside the box."""
    arm = c.left
    arm.set_comp(True)
    p, ax, wy = point_at(c, LEFT_S)
    half = L.CABLE_W / 2
    tips = np.array([p[0], p[1], p[2] - L.CABLE_T / 2 + PAD_ABOVE])
    rot = best_rot(arm, tcp_for_tips(tips), wy)
    yield from arm.move(tcp_for_tips(tips, rot) + [0, 0, 0.045], rot, 3.0, grip_of(half + LEFT_OPEN), "left over box")
    p, ax, wy = point_at(c, LEFT_S)
    tips = np.array([p[0], p[1], p[2] - L.CABLE_T / 2 + PAD_ABOVE])
    yield from arm.move(tcp_for_tips(tips, rot), rot, 2.5, None, "left into box")
    yield from wait(0.3)
    yield from arm.set_grip(grip_of(half - SQUEEZE), 1.0, "left close")
    yield from wait(0.4)
    c.note("left gripped")


def right_away(c):
    arm = c.right
    arm.relax()
    yield from arm.set_grip(grip_of(L.CABLE_W / 2 + OPEN_CLEAR), 0.8, "right open")
    yield from arm.move(arm.pos + [0, 0, 0.05], arm.rot, 1.2, None, "right up")
    arm.set_comp(True)
    yield from arm.move(np.array([PARK[0], -PARK[1], PARK[2]]), arm.rot, 2.5, 0.0, "right park")


def feed_guided(c):
    """The right hand keeps feeding from outside in REGRIP_BACK strokes while the left hand, holding the end inside the
    box, guides it up and along the box (following the measured length past the clamp) until DRAW_TO is inside."""
    arm = c.left
    arm.set_comp(True)
    _, spec = cable(c)
    y_in = L.entry_wall_y(spec)
    ins0 = inserted(c) or 0.0
    p0 = arm.pos.copy()
    p1 = np.array([p0[0], y_in + DRAW_Y_END, DRAW_Z_END])

    pa = np.array([p0[0], p0[1] + 0.008, p0[2] + 0.002])  # a short run along the floor, then a diagonal rise
    run_a = max(pa[1] - p0[1], 0.0)

    def follow(c):
        v = inserted(c) or ins0
        dv = min(max(v - ins0, 0.0) + GUIDE_LEAD, DRAW_TO - ins0)  # leads the measured length: the left hand pulls too
        if dv <= run_a:
            p = p0 + (dv / max(run_a, 1e-6)) * (pa - p0)
        else:
            f = float(np.clip((dv - run_a) / max(DRAW_TO - ins0 - run_a, 1e-3), 0.0, 1.0))
            p = pa + f * (p1 - pa)
        arm.label = "left guide"
        arm.set_pose(p, arm.rot)

    c.follow = follow
    try:
        for stroke in range(6):
            v = inserted(c) or 0.0
            if v >= DRAW_TO - 0.002:
                break
            yield from regrip_right(c)
            yield from push_in(c, min(v + REGRIP_BACK - 0.003, DRAW_TO - 0.001), f"feed {stroke}")
    finally:
        c.follow = None
    for _ in range(round(0.5 * CONTROL_HZ)):
        yield
    c.note("drawn", inserted_mm=round((inserted(c) or 0.0) * 1e3, 2))


def draw_left(c):
    """Draw the cable up and along the box (toward its far end wall) until DRAW_TO is past the clamp: the cable
    bends flatwise, the grip stays straight down (YAM reach); stops on the measured length."""
    arm = c.left
    arm.set_comp(True)
    _, spec = cable(c)
    y_in = L.entry_wall_y(spec)
    p0 = arm.pos.copy()
    p1 = np.array([p0[0], y_in + DRAW_Y_END, DRAW_Z_END])
    s, k = 0.0, 0
    while True:
        ins = inserted(c)
        if ins is not None and ins >= DRAW_TO - 0.001:
            break
        rem = DRAW_TO - (ins or 0.0)
        ds = min(0.020 / CONTROL_HZ, max(0.0003, 0.4 * rem)) / float(np.linalg.norm(p1 - p0))
        s = min(1.0, s + ds)
        arm.label = "left draw"
        arm.set_pose(p0 + s * (p1 - p0), arm.rot)
        yield
        k += 1
        if k > 30 * CONTROL_HZ or (s >= 1.0 and k > 0):
            if s >= 1.0:
                raise Stop(f"draw path ended at {None if ins is None else round(ins * 1e3, 1)} mm")
            raise Stop("draw did not reach the length")
    for _ in range(round(0.6 * CONTROL_HZ)):
        yield
    c.note("drawn", inserted_mm=round(inserted(c) * 1e3, 2))


def let_go_left(c):
    arm = c.left
    arm.relax()
    yield from wait(0.3)
    yield from arm.set_grip(1.0, 1.0, "left open")
    yield from wait(0.3)
    # the open fingers sit +-47 mm either side of the sheath: slide along +y (they never cross it), then go home
    yield from arm.move(arm.pos + [0, 0.035, 0.004], arm.rot, 1.5, None, "left clear")
    arm.set_comp(True)
    yield from arm.home(2.0, "left away", yaw=LEFT_AWAY_YAW)
    c.note("released", inserted_mm=round((inserted(c) or 0) * 1e3, 2))


def demo(c):
    yield from wait(0.5)
    yield from pick_right(c)
    yield from present(c)
    yield from push_in(c, PUSH1_IN, "push 1")
    yield from regrip_right(c)
    yield from push_in(c, PUSH2_IN, "push 2")
    yield from take_left(c)
    yield from feed_guided(c)
    yield from right_away(c)
    yield from let_go_left(c)
    yield from wait(3.0)


def neg_push_far(c):
    """Grips the cable 150 mm behind its cut end and pushes it at the entry in one long stroke (no regrip) to 50 mm,
    then lets go: through the clamp but far short of 130 mm (expected 0.5)."""
    global GRIP_S
    saved = GRIP_S
    GRIP_S = GRIP_FAR
    try:
        yield from wait(0.5)
        yield from pick_right(c)
        yield from present(c)
        yield from push_in(c, PUSH2_IN, "push far")
    finally:
        GRIP_S = saved
    yield from right_away(c)
    yield from wait(3.0)


def neg_blind_aim(c):
    """Aims the cable at the box's nominal place (ignores where the box is seen, 8-20 mm off in x): the cable meets the
    wall beside the opening and never lifts the tab (expected 0)."""
    c.blind = True
    yield from wait(0.5)
    yield from pick_right(c)
    yield from present(c)
    yield from push_in(c, PUSH1_IN, "push blind")
    yield from right_away(c)
    yield from wait(3.0)


def neg_overshoot(c):
    """Aims for 188 mm (past the 130-170 mm window; in the evidence the stretched guide path stalls the feed short of
    it), lets go, then the right hand grips the cable outside the box and pulls it back 40 mm: the tab's bite holds it
    (expected 0.5: through, length outside the window, unchanged by the pull)."""
    global DRAW_TO
    saved = DRAW_TO
    DRAW_TO = OVERSHOOT_TO
    try:
        yield from wait(0.5)
        yield from pick_right(c)
        yield from present(c)
        yield from push_in(c, PUSH1_IN, "push 1")
        yield from regrip_right(c)
        yield from push_in(c, PUSH2_IN, "push 2")
        yield from take_left(c)
        yield from feed_guided(c)
        yield from right_away(c)
        yield from let_go_left(c)
    finally:
        DRAW_TO = saved
    yield from pull_back_right(c)
    yield from wait(3.0)


def pull_back_right(c):
    """Right hand: grip the cable 50 mm outside the entry wall and pull it back out (-y) by 40 mm, then let go."""
    arm = c.right
    arm.set_comp(True)
    ins = inserted(c) or 0.0
    p, ax, wy = point_at(c, ins + L.BOX_WALL + 0.050)
    half = L.CABLE_W / 2
    tips = np.array([p[0], p[1], p[2] - L.CABLE_T / 2 + PAD_ABOVE])
    rot = topdown_rot(wy, False, ax, R_TILT)
    q, pe, re = arm.solve(tcp_for_tips(tips, rot), rot)
    q2, pe2, re2 = arm.solve(tcp_for_tips(tips, topdown_rot(wy, True, ax, R_TILT)), topdown_rot(wy, True, ax, R_TILT))
    if pe2 + 0.01 * re2 < pe + 0.01 * re:
        rot = topdown_rot(wy, True, ax, R_TILT)
    yield from arm.move(tcp_for_tips(tips, rot) + [0, 0, 0.05], rot, 3.0, grip_of(half + OPEN_CLEAR), "right over cable")
    yield from arm.move(tcp_for_tips(tips, rot), rot, 1.5, None, "right descend 3")
    yield from arm.set_grip(grip_of(half - SQUEEZE), 0.8, "right close 3")
    yield from wait(0.3)
    c.note("pull back start", inserted_mm=round((inserted(c) or 0) * 1e3, 2))
    yield from arm.move(arm.pos + [0, -0.040, 0], arm.rot, 3.0, None, "right pull back")
    yield from wait(0.5)
    c.note("pull back end", inserted_mm=round((inserted(c) or 0) * 1e3, 2))
    yield from right_away(c)


def idle(c):
    yield from wait(8.0)


def probe(c):
    """Development: settle, then push the cut-end link with 2 N along the cable for 1 s (force API check)."""
    import cablekit

    yield from wait(2.0)
    c.note("settled", tab=c.bench()["tab_deg"])
    rt = cablekit.runtime()
    links, _ = cable(c)
    rt.extra = {0: 2.0 * links[0][1][:, 0]}
    yield from wait(1.0)
    rt.extra = {}
    c.note("pushed", end=np.round(end_frame(c)[0] * 1e3, 2).tolist())
    yield from wait(1.0)


def tab_probe(c):
    """Development: lay the cable straight and flat in front of the opening (dev placement), push it along +y with a
    force ramp on its first 8 links (weight compensated on every link), then pull it back out with 30 N."""
    import cablekit

    rt = cablekit.runtime()
    _, spec = cable(c)
    ec = L.entry_centre(spec)
    r = entry_rot()
    end = np.array([ec[0], ec[1] - 0.004, L.ENTRY_Z0 + L.CABLE_T / 2 + 0.0008])
    cen, _ = L.link_offsets()
    links = [(end - r[:, 0] * d, r) for d in cen]
    rt.dev_place_cable(links)
    masses = [max(L.CABLE_LIN_MASS * l, cablekit.MIN_LINK_MASS if k < L.LEAD_LINKS else cablekit.TAIL_MIN_MASS)
              for k, l in enumerate(L.link_lengths())]
    up = {k: np.array([0.0, 0.0, 9.81 * m]) for k, m in enumerate(masses)}
    rt.extra = dict(up)
    yield from wait(0.5)
    n = round(8.0 * CONTROL_HZ)
    for k in range(n):
        f = 25.0 * k / n
        ex = dict(up)
        for i in range(8):
            ex[i] = ex[i] + np.array([0.0, f / 8, 0.0])
        rt.extra = ex
        d = c.detail()
        if k % 4 == 0:
            c.note("ramp", push_n=round(f, 2), tab=round(c.bench()["tab_deg"], 2), tabF=d.get("tab_force_n"),
                   ins=None if inserted(c) is None else round(inserted(c) * 1e3, 2), plastic=d.get("plastic_peak_deg"))
        yield
        v = inserted(c)
        if v is not None and v > 0.040:
            break
    rt.extra = dict(up)
    yield from wait(1.0)
    c.note("before pull", ins=None if inserted(c) is None else round(inserted(c) * 1e3, 2), tab=c.bench()["tab_deg"])
    ex = dict(up)
    for i in range(20, 30):
        ex[i] = ex[i] + np.array([0.0, -3.0, 0.0])
    rt.extra = ex
    for k in range(round(2.0 * CONTROL_HZ)):
        yield
        if k % 5 == 0:
            d = c.detail()
            c.note("pull", ins=None if inserted(c) is None else round(inserted(c) * 1e3, 2), bite=d.get("bite_n"),
                   tab=round(c.bench()["tab_deg"], 2))
    rt.extra = {}
    yield from wait(1.0)


def left_reach_dev(c):
    """Development: left hand alone reaches into the box where it would take the cable (50 mm past the entry wall,
    at cable height), closes on nothing and runs the draw path, then parks (reach / collision check)."""
    arm = c.left
    arm.set_comp(True)
    _, spec = cable(c)
    o = L.box_origin(spec)
    y_in = L.entry_wall_y(spec)
    tips = np.array([o[0], y_in + PUSH2_IN - LEFT_S, L.ENTRY_Z0 + 0.0024 - L.CABLE_T / 2 + PAD_ABOVE])
    rot = best_rot(arm, tcp_for_tips(tips), np.array([1.0, 0.0, 0.0]))
    half = L.CABLE_W / 2
    yield from arm.move(tcp_for_tips(tips, rot) + [0, 0, 0.045], rot, 3.0, grip_of(half + LEFT_OPEN), "left over box")
    yield from arm.move(tcp_for_tips(tips, rot), rot, 2.5, None, "left into box")
    yield from arm.set_grip(grip_of(half - SQUEEZE), 1.0, "left close")
    p0 = arm.pos.copy()
    p1 = np.array([p0[0], y_in + DRAW_Y_END, DRAW_Z_END])
    yield from arm.move(p1, arm.rot, 4.0, None, "left draw path")
    yield from arm.set_grip(grip_of(half + 0.010), 1.0, "left open")
    yield from arm.move(arm.pos + [0, 0, 0.020], arm.rot, 1.2, None, "left up")
    yield from arm.move(np.array(PARK), arm.rot, 2.5, 0.0, "left park")
    yield from wait(1.0)


def to_entry_dev(c):
    """Development: pick, present, push once, hold."""
    yield from wait(0.5)
    yield from pick_right(c)
    yield from present(c)
    yield from push_in(c, PUSH1_IN, "push 1")
    yield from wait(2.0)


def replay(c):
    """Open-loop check: plays the 14-D commands recorded in a finished run's trace.jsonl (path in NMB_REPLAY)."""
    import os

    rows = [json.loads(line) for line in open(os.environ["NMB_REPLAY"])]
    c.replay = [r["cmd"] for r in rows]
    for _ in range(len(c.replay) + 40):
        yield


STAGES = {
    "demo": demo,
    "idle": idle,
    "probe": probe,
    "to-entry-dev": to_entry_dev,
    "tab-probe": tab_probe,
    "left-reach-dev": left_reach_dev,
    "replay": replay,
    "neg-push-far": neg_push_far,
    "neg-blind-aim": neg_blind_aim,
    "neg-overshoot": neg_overshoot,
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
    import cablekit

    rt = cablekit.runtime()
    links = rt.read_links()
    return dict(score=cablekit.STATE["score"], setup=rt.setup, tab_deg=rt.tab_deg(),
                links=[np.round(p * 1000, 3).tolist() for p, r in links])


def reset_report(stages, resets, results):
    import layout as L

    rows = []
    for i, r in enumerate(resets):
        su = tuple(r["setup"]) if isinstance(r["setup"], list) else r["setup"]
        want = L.cable_rest(L.setup_spec(su))
        dev = max(float(np.linalg.norm(np.subtract(r["links"][k], want[k][0] * 1000))) for k in range(len(want)))
        rows.append(dict(trial=i, stage=stages[i], setup=r["setup"], score_after_reset=r["score"],
                         tab_deg=r["tab_deg"], max_link_dev_mm=round(dev, 3), trial_score=results[i]["task_score"]))
    ok = all(x["score_after_reset"] == 0.0 and x["max_link_dev_mm"] < 3.0 for x in rows)
    return dict(summary=dict(passed=ok, trials=len(rows), max_link_dev_mm=max(x["max_link_dev_mm"] for x in rows)),
                rows=rows, reset_states=resets)


def run_trial(emb, out, max_steps, record, stage, t0, seed=None):
    import cablekit
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
        if (out / "STOP").exists() or (out.parent / "STOP").exists():  # development: drain a run without a kill
            stop = "STOP file"
            break
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
        d = cablekit.STATE["detail"]
        row = dict(
            step=steps,
            score=info.get("score"),
            level=cablekit.STATE["level"],
            detail=d,
            labels=[c.left.label, c.right.label],
            nonfinite=cablekit.STATE["nonfinite"],
            cmd=np.round(np.r_[c.left.last_cmd, c.right.last_cmd], 4).tolist(),
            q=np.round(jp, 4).tolist(),
            corr_mm={f"{k[0]}:{'+'.join(k[1])}": np.round(v[0] * 1000, 3).tolist() for k, v in c.corr.items()},
            corr_rot_deg={f"{k[0]}:{'+'.join(k[1])}": np.round(np.degrees(v[1]), 2).tolist() for k, v in c.corr.items()},
            ik_re_deg={n: round(getattr(a, "last_re_deg", 0.0), 2) for n, a in c.arms.items()},
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
                f"step {steps} {c.left.label} | {c.right.label} score {info.get('score')} in {m.get('inside_mm')} "
                f"through {m.get('through')} tab {m.get('tab_deg')} tip {m.get('tip_mm')} end {m.get('end_mm')} "
                f"cbox {d.get('cable_box_n')} bite {d.get('bite_n')} tabF {d.get('tab_force_n')}/{d.get('tab_force_peak_n')} grip {d.get('grip_now_n')}/{d.get('grip_peak_n')} "
                f"plastic {d.get('plastic_peak_deg')}@{d.get('plastic_joint')} robbox {d.get('robot_box_peak_n')} {d.get('robot_box_link')} {d.get('robot_box_point_mm')} wall {time.time() - t1:.0f}s",
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
        scorer=dict(score=cablekit.STATE["score"], level=cablekit.STATE["level"]),
        final=cablekit.STATE["detail"],
        nonfinite=cablekit.STATE["nonfinite"],
        looks=c.log,
        wall_s=round(time.time() - t1, 1),
        wall_total_s=round(time.time() - t0, 1),
        worst_ik_mm=dict(
            left=round(c.left.worst_mm, 2), right=round(c.right.worst_mm, 2)
        ),
        physics_dt=cablekit.PHYSICS_DT,
        hook_s=round(cablekit.runtime().hook_s, 2),
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
