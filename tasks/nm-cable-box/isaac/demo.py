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
AIM_DZ = L.ENTRY_H-L.CABLE_T/2-.0018  # cable centre at mid-opening height when presented (bottom 2.1 mm above the lower edge)  # cable centre this far above the opening's lower edge
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
        self.pos, self.rot = L.fixture_pose(*fastik.tcp_pose(self.q, self.base))
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
        self.pos, self.rot = L.fixture_pose(*fastik.tcp_pose(self.q, self.base))

    def measured_tcp(self):
        return L.fixture_pose(*fastik.tcp_pose(self.measured, self.base))

    def solve(self, pos, rot):
        """IK from the current joint angles; other seeds only if that fails, and then the solution nearest the current
        angles (no jump to another arm configuration in mid-move)."""
        from yam_arena.constants import READY

        pos, rot = L.world_point(pos), L.FIXTURE_ROT @ rot
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
            self.pos, self.rot = L.fixture_pose(*fastik.tcp_pose(self.q, self.base))
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
    tips = p + [0, 0, -L.CABLE_T/2 + PAD_ABOVE]
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
    tips = p + rot[:,2] * 0.006
    half = L.CABLE_W / 2
    yield from arm.move(tcp_for_tips(tips, rot) + [0, 0, 0.038], rot, 3.0, grip_of(half + OPEN_CLEAR), "right over grip")
    yield from wait(0.3)
    p, ax, wy = point_at(c, GRIP_S)
    tips = p + rot[:,2] * 0.006
    yield from arm.move(tcp_for_tips(tips, rot), rot, 2.0, None, "right descend")
    yield from wait(0.3)
    yield from arm.set_grip(grip_of(half - SQUEEZE), 1.0, "right close")
    yield from wait(0.5)
    actual, actual_r = arm.measured_tcp()
    c.note("right gripped", finger_distance_mm=c.detail()["measure"]["tip_mm"],
           tcp_error_mm=((actual-arm.pos)*1000).round(1).tolist(),
           contact_n=c.detail()["grip_now_n"])
    if c.detail()["grip_now_n"] < 0.5:
        raise Stop("right hand missed the free cable end")
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
    pt = np.array([ec[0], y_in + depth + 0.010, L.ENTRY_Z1 - PUSH_DZ])  # cable pressed onto the opening's lower edge
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
    z_grip = L.ENTRY_Z1 - L.CABLE_T/2 - EDGE_CLEAR
    cx = 0.0
    hz = 0.0
    axial_offset = 0.0
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
        axial_error = p0[1] + sfr * (depth + y_in - p0[1]) - pe_[1]
        axial_offset = float(np.clip(axial_offset + 0.025 * axial_error, -0.008, 0.020))
        yg = gp0[1] + sfr * adv + axial_offset
        y_wall = y_in - L.BOX_WALL / 2
        z_line = z_grip - (y_wall - yg) * tan_p
        # closed loop on the measured cable centre height where it crosses the wall (next to the edge fulcrum)
        zw = wall_height(c, y_wall)
        if pe_[1] < y_in + 0.002:
            hz = float(np.clip(hz + 0.15 * (L.ENTRY_Z0 + AIM_DZ - pe_[2]), -0.023, 0.023))
        elif zw is not None:
            hz = float(np.clip(hz + 0.20 * (z_grip - zw), -0.015, 0.015))
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
    c.note(label, grip_n=c.detail()["grip_now_n"], inserted_mm=None if inserted(c) is None else round(inserted(c) * 1e3, 2))


def regrip_right(c):
    arm = c.right
    half = L.CABLE_W / 2
    arm.relax()
    yield from arm.set_grip(grip_of(half + OPEN_CLEAR), 0.8, "right open")
    arm.set_comp(True)
    # once let go, the outside part drops (it pivots on the opening's edge): find the sheath again REGRIP_BACK further
    # back along it, from its measured pose, and take it from above as at the first pick
    rot = arm.rot.copy()
    s_new = min(getattr(c, "grip_s", GRIP_S) + REGRIP_BACK, (inserted(c) or 0.0) + 0.058)
    yield from arm.move(arm.pos + [0, 0, 0.015], rot, 0.5, None, "right up")
    p, _, _ = point_at(c, s_new)
    tips = p + rot[:, 2] * 0.005
    yield from arm.move(tcp_for_tips(tips, rot) + [0, 0, 0.020], rot, 1.1, None, "right over cable")
    p, _, _ = point_at(c, s_new)
    tips = p + rot[:, 2] * 0.005
    yield from arm.move(tcp_for_tips(tips, rot), rot, 0.7, None, "right down")
    yield from arm.set_grip(grip_of(half - SQUEEZE), 0.8, "right close 2")
    yield from wait(0.3)
    c.corr.pop(("end", ("right",)), None)
    c.grip_s = s_new
    capture(c, "right")
    actual, rr = arm.measured_tcp()
    c.note("right regripped", arc_mm=round(s_new*1000),
           tcp_err_mm=((actual-arm.pos)*1000).round(1).tolist(), total_contact=c.detail()["grip_now_n"])


def take_left(c):
    """Straddle the free end at the back of the cavity, using measured pad position."""
    arm=c.left
    u=.0045
    _,spec=cable(c)
    target=point_at(c,u)[0];rot=best_rot(arm,tcp_for_tips(target),np.array([1.,0.,0.]))
    area=tcp_for_tips(target,rot)
    area[2]=L.BOX_RIM_Z+.062
    yield from arm.move(area,rot,3.4,grip_of(.0074),"square to box opening")
    start=arm.measured_tcp()[0]
    corr=np.zeros(3)
    from yam_arena import ik
    for k in range(1,112):
        f=ik.smoothstep(min(k/85,1.))
        wire=point_at(c,u)[0]
        goal=tcp_for_tips(wire+rot[:,2]*.0013,rot)
        ref=start+f*(goal-start)
        actual,_=arm.measured_tcp()
        corr=np.clip(corr+.05*(ref-actual),[-.005,-.010,-.003],[.005,.010,.003])
        arm.grip=grip_of(.0074-.00125*f)
        arm.set_pose(ref+corr,rot)
        yield
    gap_before=arm.measured_grip*FINGER_OPEN
    arm.relax()
    yield from arm.set_grip(0.,.8,"close on the lead")
    yield from wait(.3)
    wire=point_at(c,u)[0];tcp,rot=arm.measured_tcp()
    c.note("receiver contact",open_before_mm=round(gap_before*1000,2),
           closed_mm=round(arm.measured_grip*FINGER_OPEN*1000,2),
           force=c.detail()["grip_now_n"],offset_mm=((tcp+rot[:,2]*.013-wire)*1000).round(1).tolist())
    arm.set_comp(True)
    yield from arm.move(tcp+[0,0,.011],rot,1.,None,"test small draw")
    new_wire=point_at(c,u)[0]
    c.note("receiver test draw",lead_shift_mm=((new_wire-wire)*1000).round(1).tolist())
    if new_wire[2]-wire[2]<.003:
        raise Stop("the cable did not follow the receiver")

def right_away(c):
    arm = c.right
    arm.relax()
    yield from arm.set_grip(grip_of(L.CABLE_W / 2 + OPEN_CLEAR), 0.8, "right open")
    yield from arm.move(arm.pos + [0, 0, 0.05], arm.rot, 1.2, None, "right up")
    arm.set_comp(True)
    yield from arm.home(2.5, "right park", yaw=-0.7)


def feed_guided(c):
    """The right hand keeps feeding from outside in REGRIP_BACK strokes while the left hand, holding the end inside the
    box, guides it up and along the box (following the measured length past the clamp) until DRAW_TO is inside."""
    arm = c.left
    arm.set_comp(True)
    _, spec = cable(c)
    y_in = L.entry_wall_y(spec)
    ins0 = inserted(c) or 0.0
    p0 = arm.pos.copy()
    p1 = p0 + np.array([0.008, 0.026, 0.115])

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
    p1 = p0 + np.array([0.008, 0.026, 0.115])
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


def grasp_from_face(c):
    arm=c.left
    box=L.box_origin(cable(c)[1])
    hinge=box+np.array([L.ENTRY_X,-L.BOX_IN_Y/2,L.ENTRY_Z1-L.BOX_FLOOR_Z])
    library=np.load(HERE/"cavity_grasps.npz")
    candidates=[]
    for arc in np.arange(.008,min(.095,(inserted(c) or 0.)-.012),.005):
        target,tangent,_=point_at(c,arc)
        near=np.linalg.norm(library["points"]-(target-hinge),axis=1)
        for j in np.argsort(near)[:4]:
            if near[j]>.011:continue
            valid=library["clearance"][j]-near[j]
            for i in np.flatnonzero(valid>.00015):
                rot=library["rotations"][i]
                across=abs(rot[:,0]@tangent)
                if across>.60:continue
                # Prefer room on all sides, then a jaw axis across the stock.
                merit=min(valid[i],.007)-.010*across-.015*max(0.,.01-target[2]+L.BOX_RIM_Z)
                candidates.append((merit,float(arc),i,rot))
    if not candidates:
        c.note("no clear gripper pose",depth_mm=round(1000*(inserted(c) or 0.)))
        return False
    choices=[]
    ranked=sorted(candidates,key=lambda row:row[0],reverse=True)
    seen=set()
    for merit,arc,i,rot in ranked:
        if (arc,i) in seen:continue
        seen.add((arc,i))
        if len(seen)>70:break
        pt=point_at(c,arc)[0]
        target=tcp_for_tips(pt+rot[:,2]*.0025,rot)
        front=target-rot[:,2]*.065
        q,ep,er=arm.solve(target,rot)
        qf,ef,rf=arm.solve(front,rot)
        if max(ep,ef)>.004 or max(er,rf)>.14:continue
        choices.append((merit-ep-ef-.025*(er+rf)-.0001*np.linalg.norm(q-arm.measured),arc,rot,front,qf))
    if not choices:return False
    _,arc,rot,front,q1=max(choices,key=lambda row:row[0])
    q0=arm.q.copy()
    c.note("selected cavity grasp",arc_mm=round(arc*1000),stock_mm=(1000*point_at(c,arc)[0]).round(2).tolist())
    arm.set_comp(True)
    for k in range(60):
        u=(k+1)/60
        arm.q=q0+(3*u*u-2*u**3)*(q1-q0)
        arm.pos,arm.rot=L.fixture_pose(*fastik.tcp_pose(arm.q,arm.base))
        arm.grip=grip_of(.0076)
        arm.label="outside the central cable span"
        yield
    for k in range(95):
        wire=point_at(c,arc)[0]
        goal=tcp_for_tips(wire+rot[:,2]*.0025,rot)
        ref=goal-rot[:,2]*.065*max(0.,1.-(k+1)/56)
        tcp,rr=arm.measured_tcp()
        q,ep,er=arm.solve(tcp+np.clip(ref-tcp,-.009,.009),rot)
        if ep>.005 or er>.15:break
        arm.q=q;arm.pos,arm.rot=L.fixture_pose(*fastik.tcp_pose(q,arm.base))
        arm.grip=grip_of(.0076*max(0.,1.-max(0,k-59)/19))
        yield
    arm.grip=0.
    # Confirm the material point follows a short draw out of the cavity.
    initial=point_at(c,arc)[0]
    tcp,rr=arm.measured_tcp()
    for k in range(30):
        q,ep,er=arm.solve(tcp+[0.,0.,.009*min(1.,(k+1)/21)],rr)
        if ep>.004 or er>.14:break
        arm.q=q;arm.pos,arm.rot=L.fixture_pose(*fastik.tcp_pose(q,arm.base))
        yield
    moved=point_at(c,arc)[0]-initial
    c.note("tested central cable span",wire_mm=(1000*moved).round(1).tolist())
    if moved[2]<.0015:return False
    start,rot=arm.measured_tcp()
    for k in range(180):
        frac=min(1.,(k+1)/145.)
        desired=start+np.array([-.005,-.010,.083])*frac
        rr=L.rx(math.radians(14)*frac)@rot
        q,ep,er=arm.solve(desired,rr)
        if ep>.004 or er>.14:break
        arm.q=q;arm.pos,arm.rot=L.fixture_pose(*fastik.tcp_pose(q,arm.base))
        yield
        if (inserted(c) or 0.)>=.145:break
    c.note("draw from central span",arc_mm=round(1000*arc))
    return True


def joint_move(arm,target,duration,label):
    arm.label=label
    start=arm.measured.copy()
    steps=max(4,int(duration*CONTROL_HZ),int(np.max(abs(target-start))/.033))
    for k in range(steps):
        u=(k+1)/steps
        arm.q=start+(target-start)*(3*u*u-2*u*u*u)
        arm.pos,arm.rot=L.fixture_pose(*fastik.tcp_pose(arm.q,arm.base))
        yield


def pad_forces(c,hand,s):
    import cablekit
    rt=cablekit.runtime()
    forces=np.linalg.norm(rt.cv_grip.get_contact_force_matrix(dt=cablekit.PHYSICS_DT).detach().cpu().numpy(),axis=-1)
    side=len(rt.robot_links)//2
    start=side if hand=="right" else 0
    at=np.abs(L.link_offsets()[0]-s)<.023
    return [float(forces[at,i].max()) for i in range(start,start+side)
            if rt.robot_links[i].rsplit("/",1)[-1] in ("tip_left","tip_right")]


def acquire(c,hand,arcs,inside=False):
    from nm_pathplan import Planner
    arm=c.arms[hand]
    if not hasattr(c,"planners"):c.planners={}
    if hand not in c.planners:c.planners[hand]=Planner(cable(c)[1],arm.base)
    planner=c.planners[hand]
    for s in arcs:
        wire,axis,width=point_at(c,s)
        plans=planner.grasps(wire,axis,width,arm.measured,inside)
        selected=None
        for _,rot,half,pre,goal in plans[:10]:
            path=planner.path(arm.measured,pre,half)
            if path is not None:
                selected=(rot,half,path)
                break
        if selected is None:
            c.note("no collision-free pickup",arm=hand,arc_mm=round(s*1000),point_mm=(wire*1000).round(1).tolist())
            continue
        rot,half,path=selected
        arm.relax()
        arm.set_comp(True)
        arm.offset_clip=.28
        arm.grip=grip_of(half)
        for q in path:yield from joint_move(arm,q,.50,"around the framing")
        start,r=arm.measured_tcp()
        for k in range(78):
            wire,_,_=point_at(c,s)
            goal=tcp_for_tips(wire+.004*rot[:,2],rot)
            fraction=min(1.,(k+1)/38)
            ref=start+fraction*(goal-start)
            actual,_=arm.measured_tcp()
            q=planner.inverse(actual+np.clip(ref-actual,-.008,.008),rot,arm.measured)
            if q is None or not planner.clear(q,min(half,arm.measured_grip*FINGER_OPEN)):break
            arm.q=q
            arm.pos,arm.rot=L.fixture_pose(*fastik.tcp_pose(q,arm.base))
            arm.grip=grip_of(half*max(0.,1.-max(0,k-40)/21))
            arm.label="track material between pads"
            yield
        pads=pad_forces(c,hand,s)
        c.note("planned grasp contact",hand=hand,arc_mm=round(s*1000),pads_n=np.round(pads,2).tolist())
        if pads and min(pads)>.3:
            actual,r=arm.measured_tcp()
            wire,axis,_=point_at(c,s)
            direction=np.array([0.,0.,1.]) if inside else axis
            arm.grip=0.
            for k in range(17):
                q=planner.inverse(actual+direction*.006*min(1.,(k+1)/13),r,arm.measured)
                if q is None:break
                arm.q=q
                arm.pos,arm.rot=L.fixture_pose(*fastik.tcp_pose(q,arm.base))
                yield
            moved=float((point_at(c,s)[0]-wire)@direction)
            pads=pad_forces(c,hand,s)
            c.note("planned grip trial",travel_mm=round(1000*moved,2),pads_n=np.round(pads,2).tolist())
            if pads and min(pads)>.3 and (moved>.0007 or
                    ((inserted(c) or 0.)>.128 and min(pads)>1.2 and gripped_material(c,hand) is not None)):
                if hand=="right":c.grip_s=s
                return True
        arm.grip=grip_of(half)
        yield from wait(.45)
        if planner.clear(path[-1],half):yield from joint_move(arm,path[-1],.75,"back from the stock")
    return False


def gripped_material(c,hand):
    """Estimate the arc position clamped between both pads, from their contact reports."""
    import cablekit
    rt=cablekit.runtime()
    forces=np.linalg.norm(rt.cv_grip.get_contact_force_matrix(dt=cablekit.PHYSICS_DT).detach().cpu().numpy(),axis=-1)
    half=len(rt.robot_links)//2
    cols=[i for i in range(0 if hand=="left" else half,half if hand=="left" else 2*half)
          if rt.robot_links[i].rsplit("/",1)[-1] in ("tip_left","tip_right")]
    if len(cols)!=2:return None
    centers=L.link_offsets()[0]
    a,b=forces[:,cols[0]],forces[:,cols[1]]
    if min(a.max(),b.max())<.6:return None
    ia,ib=np.argmax(a),np.argmax(b)
    if abs(centers[ia]-centers[ib])>.030:return None
    return .5*(centers[ia]+centers[ib])


def push_gripped_segment(c,arc,stroke):
    arm=c.right
    spec=cable(c)[1]
    gate=L.entry_centre(spec)
    gate[1]=L.entry_wall_y(spec);gate[2]=L.ENTRY_Z1-.0035
    start,r=arm.measured_tcp()
    stock,ax,_=point_at(c,arc)
    along=gate-stock
    along/=max(1e-8,np.linalg.norm(along))
    along=.55*along+.45*ax
    along/=max(1e-8,np.linalg.norm(along))
    before=inserted(c) or 0.
    delta=np.zeros(3)
    for k in range(85):
        depth=inserted(c) or 0.
        if depth>=.144:break
        u=min(1.,(k+1)/60.)
        # Roll at the entry to redirect the emerging lead towards the open face.
        rotation=L.rx(math.radians(11)*u)
        guide=point_at(c,min(arc,max(.002,depth)))[0]
        dx=float(np.clip(gate[0]-guide[0],-.003,.003))
        delta[0]=.94*delta[0]+.06*dx
        target=gate+rotation@(start-gate)+along*u*.029+delta
        q,ep,er=arm.solve(target,rotation@r)
        if ep>.004 or er>.15:break
        arm.q=q;arm.grip=0.;arm.pos,arm.rot=L.fixture_pose(*fastik.tcp_pose(q,arm.base))
        yield
        if k>9 and gripped_material(c,"right") is None:break
        if depth>=before+.021:break
    c.note("feed observed material",stroke=stroke,arc_mm=round(arc*1000),previous_mm=round(1000*before,2),
           contact_arc_mm=round(1000*(gripped_material(c,"right") or 0.)))


def angle_loaded_stock(c,hand):
    # Shift the gripped sheath across the entry so that it presses on the moving tongue.
    arm=c.arms[hand]
    planner=c.planners[hand]
    spec=cable(c)[1]
    mouth=L.entry_centre(spec)
    mouth[1]=L.entry_wall_y(spec)
    mouth[2]=L.ENTRY_Z1-.004
    source,rot=arm.measured_tcp()
    depth=inserted(c) or 0.
    neck=point_at(c,depth)[0]
    side=np.clip(mouth-neck,-.009,.009)
    side[1]=0.
    c.note("loaded sheath at entry",hand=hand,neck_mm=(1000*(neck-mouth)).round(2).tolist(),
           pads=(1000*(gripped_material(c,hand) or 0.)))
    angles=[(side[0],.004,side[2]),(side[0],.012,.012),
            (side[0],.012,-.011),(side[0]-.006,.018,.006),(side[0]+.007,.014,.010)]
    offset=np.zeros(3)
    for stroke in angles:
        start=offset.copy()
        for k in range(40):
            u=min(1.,(k+1)/25.)
            offset=start+(np.asarray(stroke)-start)*u
            q=planner.inverse(source+offset,rot,arm.measured)
            if q is None or not planner.clear(q,max(0.,arm.measured_grip)*FINGER_OPEN):break
            arm.q=q
            arm.pos,arm.rot=L.fixture_pose(*fastik.tcp_pose(q,arm.base))
            arm.grip=0.
            arm.label="seat the sheath under the integral tab"
            yield
            m=c.detail()["measure"]
            if m["through"] and 130.4< (m["inside_mm"] or 0.) <167.:
                c.note("clamp bears on the sheath",hand=hand)
                return True
            if gripped_material(c,hand) is None and k>6:break
        c.note("sheath orientation",hand=hand,offset_mm=(offset*1000).round(2).tolist())
    return False


def demo(c):
    yield from wait(0.5)
    yield from pick_right(c)
    yield from present(c)
    yield from push_in(c,PUSH1_IN,"push 1")
    yield from regrip_right(c)
    yield from push_in(c,PUSH2_IN,"push 2")
    # Take a second bite outside before levering: a longer free span develops
    # curvature against the top entry and presents the lead at the face.
    arm=c.right
    yield from regrip_right(c)
    _,spec=cable(c)
    pivot=L.entry_centre(spec)
    p0,r0=arm.measured_tcp()
    for k in range(1,131):
        f=k/130
        rot=L.rx(math.radians(34)*f)
        arm.set_pose(pivot+rot@(p0-pivot)+[0,.023*f,-.005*f],rot@r0)
        yield
    c.note("angled lead",curve_mm=np.round([point_at(c,u)[0]*1000 for u in (.005,.015,.025,.035)],1).tolist())
    for stroke in range(6):
        depth=inserted(c) or 0.
        if depth>=.142 or depth<.035:break
        arc=gripped_material(c,"right")
        if arc is None or arc<depth+.009:
            arm.relax()
            arm.grip=grip_of(.014)
            yield from wait(.48)
            candidates=[depth+x for x in (.035,.050,.071,.093,.026)]
            if not (yield from acquire(c,"right",candidates)):break
            arc=gripped_material(c,"right")
        if arc is None:break
        if .128<(inserted(c) or 0.)<.168:
            yield from angle_loaded_stock(c,"right")
            break
        yield from push_gripped_segment(c,arc,stroke)
        if (inserted(c) or 0.)<=depth+.001:
            arm.relax()
            arm.grip=grip_of(.014)
            yield from wait(.48)
    depth=inserted(c) or 0.
    m=c.detail()["measure"]
    if depth>=.128:
        arm.relax()
        arm.grip=grip_of(.017)
        yield from wait(.65)
        yield from arm.home(1.9,"withdraw from the supply",yaw=-.65)
        yield from wait(1.1)
        if not c.detail()["measure"]["through"]:
            arcs=np.arange(.005,min(.100,depth-.010),.0075)
            order=sorted(arcs,key=lambda s:point_at(c,s)[0][2],reverse=True)
            if (yield from acquire(c,"left",order[:5])):
                yield from angle_loaded_stock(c,"left")
            c.left.relax()
            c.left.grip=grip_of(.018)
            yield from wait(.6)
    elif .052<depth:
        arm.relax()
        yield from arm.set_grip(grip_of(.016),.65,"release the supply")
        yield from arm.home(1.9,"make room for the receiver",yaw=-.65)
        yield from grasp_from_face(c)
        c.left.relax()
        yield from c.left.set_grip(grip_of(.013),.65,"release drawn cable")
    # Release the sheath and retreat along the approach vector before parking.
    arm.relax()
    yield from arm.set_grip(grip_of(.018),.8,"clear the outer sheath")
    yield from arm.home(2.5,"right park",yaw=-.7)
    yield from c.left.home(2.,"left park",yaw=LEFT_AWAY_YAW)
    yield from wait(4.0)


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
    entry = tcp_for_tips(tips,rot)
    entry[2] = L.BOX_RIM_Z + 0.069
    # From the free side of the stud, get in front of the face first, then approach normally.
    yield from arm.move(entry, rot, 3.5, grip_of(half + LEFT_OPEN), "left in front")
    pm, _ = arm.measured_tcp()
    c.note("front approach", tcp_err_mm=((pm-entry)*1000).round(1).tolist())
    yield from arm.move(tcp_for_tips(tips, rot), rot, 2.5, None, "left into box")
    yield from arm.set_grip(grip_of(half - SQUEEZE), 1.0, "left close")
    p0 = arm.pos.copy()
    p1 = p0 + np.array([0.008, 0.026, 0.115])
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
