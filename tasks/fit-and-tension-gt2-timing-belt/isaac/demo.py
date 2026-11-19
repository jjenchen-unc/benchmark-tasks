"""Scripted full-score demo for timing-belt in the yam_arena embodiment (Isaac Lab-Arena / PhysX).

    python demo.py task.py OUT [--max-steps N] [--no-record] [--stage demo|<variant>[,<variant>...]] [--device cpu]

Acts ONLY through the 14 joint/gripper commands of ``YamArenaEmbodiment.step``. It READS the belt links' and the
bench bodies' poses through ``beltkit.belt_state()`` / ``beltkit.bench_state()``; it never writes a pose, a joint
state or a score (closed loop on measured state, like a person watching the belt).

Plan (belt pinches: gripper approaching horizontally along +x, pads closing across the belt's thickness). The belt
starts hung over the joint (60T) pulley, in its teeth; the motor plate is at the slack end of its slot.
1. Right hand pinches the joint pulley's near strand at the point that will lie just above the motor (20T) pulley,
   carries it in front of the drive (so the belt clears the motor pulley's front flange) over the motor pulley and
   down its far (-y) side to 9 o'clock, 16.5 mm from its axis, and only then into the belt plane (the loop has ~16 mm
   of slack; the pinch's detour round the 16 mm flange takes most of it). It then walks the pinch round the pulley's
   bottom and out along the lower span to about its middle, 20 mm below it (the belt circulates round the joint
   pulley), so the belt wraps the small pulley and the hand lets go away from both pulleys: the slack belt lies on
   the inner finger by its own weight, and sliding the fingers off does not drag it over a flange.
2. The left hand pushes the motor (by its body's inner side) outward (-y) until the measured belt tension reaches
   11 N; holding it there (integral term frozen), the right hand grips the clamp lever's knob on the slide's outer
   side and turns it a quarter turn clockwise (handle from outward to forward), which clamps the plate.
3. Both hands let go and hold their wrist cameras 14 cm in front of and above the two pulleys (fingertips several cm
   clear) for the close-ups of the seated teeth, during the 3 s hands-off hold.
Both arms add the remaining joint error back to their commands while loaded (integral term on their own joint
readings), because the joint PD sags under load.
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
sys.path.insert(0, str(HERE.parent / 'src'))
import layout as L  # noqa: E402

CONTROL_HZ = 20
DT = 1.0 / CONTROL_HZ
FINGER_OPEN = .04695
PAD_TIP = .013          # pad tips past the TCP
PAD_HALF_W = .0137      # half pad width (gripper y)
FINGER_T = .0064        # finger thickness along the closing axis (closed finger block 12.8 mm)
GRIP_PINCH = 0.0        # fully closed on the 1.38 mm belt: ~3.4 N per finger
GRIP_OPEN = .30         # pads ~28 mm apart
BELT_MID = -.0003       # belt mid-thickness from the pitch line (local z)
SEEDS = [np.array(v, float) for v in ([0, 1, 1, -.6, 0, 0], [-.9, 1.8, .6, 1.2, -.9, -1.57], [1., 1.86, .72, 1.15, 1., -1.57],
                                      [-.8, 2.0, 1.5, -.5, -.6, .9], [.6, 2.0, 1.5, -.4, .5, 2.0], [0, 2, 1.5, -.5, 0, 0])]


def grip_of(gap_m):
    """Pad gap (m) -> yam_arena normalised grip (1 open = 94 mm, 0 closed)."""
    return float(np.clip(gap_m / (2 * FINGER_OPEN), 0., 1.))


class Stop(Exception):
    pass


class Arm:
    """One arm: smooth Cartesian moves (URDF IK), each starting where the last command left it."""

    def __init__(self, name, base_pos):
        from yam_arena import ik
        from yam_arena.constants import READY
        self.name, self.base = name, np.asarray(base_pos, float)
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = ik.tcp_pose(self.q, self.base)
        self.offset = np.zeros(6)
        self.measured = self.q.copy()
        self.measured_grip = 1.0
        self.comp = False
        self.freeze = False
        self.label = ''
        self.worst_mm = 0.

    @property
    def cmd(self):
        return np.r_[self.q + self.offset, self.grip]

    def observe(self, q):
        self.measured = np.asarray(q[:6], float)
        self.measured_grip = float(q[6]) if len(q) > 6 else 1.0
        if self.freeze:      # hold: keep the integral term as it is (no wind-up while pressed against a part)
            pass
        elif self.comp:
            self.offset = np.clip(self.offset + .3 * (self.q - self.measured), -.25, .25)
        else:
            self.offset *= .85

    def measured_tcp(self):
        from yam_arena import ik
        return ik.tcp_pose(self.measured, self.base)

    def solve(self, pos, rot, seed=None, iters=80):
        from yam_arena import ik
        best = None
        for s in [self.q if seed is None else seed] + ([] if seed is not None else SEEDS):
            q, pe, re = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), s, base_pos=self.base, iters=iters)
            if best is None or pe + .1 * re < best[1] + .1 * best[2]:
                best = (q, pe, re)
            if pe < 5e-4 and re < 5e-3:
                break
        return best

    def set_pose(self, pos, rot):
        q, pe, re = self.solve(pos, rot)
        if pe > .002 or re > math.radians(3):
            raise Stop(f'{self.name} unreachable at {self.label}: {pe * 1000:.1f} mm {math.degrees(re):.1f} deg')
        if np.max(np.abs(q - self.q)) > .6:  # IK branch jump: refuse (would whip the arm)
            raise Stop(f'{self.name} IK jump at {self.label}: {np.round(q - self.q, 2).tolist()}')
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), np.asarray(rot, float)

    def move(self, pos, rot, duration, grip=None, label='', joint=False):
        from yam_arena import ik
        self.label = label or self.label
        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        if joint:
            q0 = self.q.copy()
            q1, pe, re = self.solve(pos, rot, iters=200)
            if pe > .002 or re > math.radians(3):
                raise Stop(f'{self.name} unreachable at {label}: {pe * 1000:.1f} mm {math.degrees(re):.1f} deg')
            for k in range(1, n + 1):
                s = ik.smoothstep(k / n)
                self.q = q0 + s * (q1 - q0)
                self.grip = g0 + s * (g1 - g0)
                yield
            self.q, self.pos, self.rot = q1, np.asarray(pos, float), np.asarray(rot, float)
            return
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self.set_pose(p0 + s * (np.asarray(pos) - p0), ik.slerp_matrix(r0, np.asarray(rot), s))
            yield

    def set_grip(self, grip, duration=.8, label=''):
        yield from self.move(self.pos, self.rot, duration, grip, label)

    def home(self, duration=2.0, label=''):
        from yam_arena import ik
        from yam_arena.constants import READY
        self.label = label or f'{self.name} home'
        self.comp = False
        q0, q1 = self.q.copy(), np.array(READY, float)
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            self.q = q0 + ik.smoothstep(k / n) * (q1 - q0)
            self.pos, self.rot = ik.tcp_pose(self.q, self.base)
            yield


def wait(seconds):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


class Ctx:
    def __init__(self, emb=None):
        from yam_arena import station
        self.emb = emb
        self.left = Arm('left', station.BASES['left'])
        self.right = Arm('right', station.BASES['right'])
        self.log = []
        self.step = 0

    def pose(self, name):
        import beltkit
        return beltkit.bench_state()[name]

    def belt(self):
        import beltkit
        return beltkit.belt_state()

    def detail(self):
        import beltkit
        return beltkit.STATE['detail']

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack
        return Action(data=pack(self.left.cmd, self.right.cmd))

    def note(self, tag, **kw):
        d = self.detail()
        row = dict(at=tag, step=self.step, instant=d.get('instant'), cdist=d.get('centre_distance_mm'),
                   lever=d.get('lever_deg'),
                   tension=d.get('tension_n'), belt_load=d.get('belt_load'),
                   joint={k: (d.get('joint') or {}).get(k) for k in ('around', 'between', 'seated', 'near', 'wrap')},
                   motor={k: (d.get('motor') or {}).get(k) for k in ('around', 'between', 'seated', 'near', 'wrap')},
                   spans={k: v.get('max_gap_mm') for k, v in (d.get('spans') or {}).items()}, **kw)
        self.log.append(row)
        print('LOOK', json.dumps(row, default=float), flush=True)


def require(cond, name, info=None):
    if not cond:
        raise Stop(f'{name}: {json.dumps(info, default=float)}')


# ---------------------------------------------------------------------------------------------- grasp geometry
def pinch_rotation(normal, pitch_deg=0.0):
    """Gripper rotation approaching along +x (pitched down by ``pitch_deg``) whose closing axis (gripper x) is the
    belt's normal ``normal`` (world, in the y-z plane) made perpendicular to the approach."""
    p = math.radians(pitch_deg)
    gz = np.array([math.cos(p), 0., -math.sin(p)])
    gx = np.asarray(normal, float) - gz * np.dot(normal, gz)
    gx /= np.linalg.norm(gx)
    return np.column_stack([gx, np.cross(gz, gx), gz])


def best_roll(arm, p, normal, pitch_deg=0.0):
    """The closing-axis sign (+n or -n) with the better IK / joint-6 margin."""
    best = None
    for s in (1., -1.):
        r = pinch_rotation(s * np.asarray(normal, float), pitch_deg)
        q, pe, re = arm.solve(p, r, iters=150)
        cost = pe * 100 + re + abs(q[5]) / 2.094
        if best is None or cost < best[0]:
            best = (cost, r)
    return best[1]


def pinch_tcp(q_point, rot, depth=.008):
    """TCP that puts the point ``q_point`` between the pads ``depth`` ahead of the TCP along the approach."""
    return np.asarray(q_point, float) - rot[:, 2] * depth


def link_mid(c, r, i):
    """Mid-thickness point and outward normal of link i."""
    return c[i] + r[i][:, 2] * BELT_MID, r[i][:, 2]


def nearest_link(c, point, mask=None):
    d = np.linalg.norm(c - np.asarray(point, float), axis=1)
    if mask is not None:
        d = np.where(mask, d, np.inf)
    return int(np.argmin(d))


def pinch(c, a, k, gap, label, standoff=.045, via=None):
    """Pinch belt link k: approach to ``standoff`` in front of it (optionally through a via point), then move in
    while re-reading the link every control step, close, and check that the belt is between the pads (closed fingers
    on nothing read ~0). Retries once from the standoff with the link re-measured."""
    for attempt in range(2):
        cen, rot = c.belt()
        q, n = link_mid(cen, rot, k)
        r = best_roll(a, pinch_tcp(q, pinch_rotation(n)), n)
        if via is not None and attempt == 0:
            yield from a.move(np.asarray(via, float), r, 2.0, grip_of(gap), f'{label}_via', joint=True)
        yield from a.move(pinch_tcp(q, r) - [standoff, 0, 0], r, 2.5, grip_of(gap), f'{label}_approach',
                          joint=via is None or attempt > 0)
        yield from wait(.5)
        n_in = 45
        p0 = a.pos.copy()
        for kk in range(1, n_in + 1):
            cen, rot = c.belt()
            q, n = link_mid(cen, rot, k)
            s = kk / n_in
            target = pinch_tcp(q, r)
            a.label = f'{label}_in'
            a.set_pose(p0 + s * (target - p0), r)
            yield
        for _ in range(10):  # settle on the current position
            cen, rot = c.belt()
            q, n = link_mid(cen, rot, k)
            a.set_pose(pinch_tcp(q, r), r)
            yield
        yield from a.set_grip(GRIP_PINCH, .8, f'{label}_close')
        touch = 0.
        for _ in range(6):  # contact force between the arms and belt links while the fingers hold closed
            yield
            touch = max(touch, sum(v for n, v in (c.detail().get('arm_touch_max') or {}).items()
                                   if n.startswith('belt_')))
        opening = float(a.measured_grip)
        c.note(f'{label}_closed', link=k, opening=round(opening, 4), touch=round(touch, 3),
               q=np.round(q * 1000, 1).tolist())
        # a pinched 1.38 mm belt leaves the pads ~0.3-0.5 % open (thin links, 50 N grip) and pressed on belt links;
        # closed on nothing reads ~0 with no belt contact
        if opening > .002 and touch > .3:
            return
        yield from a.set_grip(grip_of(gap), .5, f'{label}_reopen')
        yield from a.move(a.pos - [standoff, 0, 0], a.rot, 1.0, label=f'{label}_retreat')
    raise Stop(f'{label}: missed the belt twice (link {k})')


def tilt_rotation(pitch_deg, yaw_deg=0.0, flip=False):
    """Approach pitched down by ``pitch_deg`` toward +x (yawed), closing axis in the approach's vertical plane."""
    p, y = math.radians(pitch_deg), math.radians(yaw_deg)
    gz = np.array([math.cos(p) * math.cos(y), math.cos(p) * math.sin(y), -math.sin(p)])
    side = np.array([-math.sin(y), math.cos(y), 0.])
    gx = np.cross(side, gz)
    if flip:
        gx = -gx
    return np.column_stack([gx, np.cross(gz, gx), gz])


# --------------------------------------------------------------------------------------------------- the script
def chain_dir(rot, k, along):
    """+1 if increasing link index runs along world direction ``along`` at link k, else -1."""
    return 1 if np.dot(rot[k][:, 0], along) > 0 else -1


def top_line_z(y, ym):
    """z of the drive's upper external tangent (where the taut top span runs) at y, motor axis at ym."""
    import belt_scoring as S
    n, d = S.tangent_line((L.JOINT_Y, L.AXIS_Z), L.r_hinge('joint'), (ym, L.AXIS_Z), L.r_hinge('motor'), True)
    return (d - n[0] * y) / n[1]


def rot_x(deg):
    t = math.radians(deg)
    return np.array([[1, 0, 0], [0, math.cos(t), -math.sin(t)], [0, math.sin(t), math.cos(t)]])


WRAP_R = .0165           # pinch centre this far from the motor axis while the belt is wrapped on (lower finger clears
                         # the 16 mm flange by ~1 mm)
WRAP_FRONT = .010        # transit plane this far in front of the belt plane (belt clears the motor pulley's flange)
WRAP_DK = 0              # pinch this many links off the computed one
WRAP_END_DEG = 100.0     # walk the pinch round the motor pulley's far side to straight below it
WRAP_T_STOP = 3.0        # stop walking round the motor pulley once the belt tension reaches this (N)


def wrap_waypoints(ym, y_pinch, z_pinch):
    """Pinch-centre waypoints (and world rotations about x relative to the pinch rotation) from the joint pulley's
    near strand, in front of the drive, to above the motor pulley and into the belt plane."""
    y0 = ym - .002
    z1 = L.AXIS_Z + math.sqrt(WRAP_R ** 2 - (y0 - ym) ** 2)
    xf = L.BELT_X - WRAP_FRONT
    return [((xf, y_pinch, z_pinch), 0., 1.5, 'wrap_out'),
            ((xf, (y_pinch + y0) / 2 + .01, L.AXIS_Z + .032), -45., 2.5, 'wrap_up'),
            ((xf, y0, z1 + .004), -90., 2.5, 'wrap_over'),
            ((L.BELT_X, y0, z1 + .004), -90., 2.0, 'wrap_in'),
            ((L.BELT_X, y0, z1), -90., 1.0, 'wrap_down')]


def side_waypoints(ym, y_pinch, z_pinch):
    """'side' wrap: carry the pinch in front of the drive over the motor pulley and down its far (-y) side to 9
    o'clock (16.5 mm from the axis, pads vertical: the belt held as it will run down that side), and only then into
    the belt plane: the strand to the joint pulley's top passes above the small pulley's flange and the strand to its
    bottom below it, so the belt enters the plane round the pulley without touching it."""
    xf = L.BELT_X - WRAP_FRONT
    ys = ym - WRAP_R
    return [((xf, y_pinch, z_pinch), 0., 1.5, 'wrap_out'),
            ((xf, (y_pinch + ys) / 2, L.AXIS_Z + .035), 0., 2.5, 'wrap_up'),
            ((xf, ys, L.AXIS_Z + .022), 0., 2.0, 'wrap_over'),
            ((xf, ys, L.AXIS_Z), 0., 1.5, 'wrap_side'),
            ((L.BELT_X, ys, L.AXIS_Z), 0., 2.0, 'wrap_in')]


WRAP_MODE = 'side'


def wrap_motor(c, hand='right', walk=True, release=True):
    """Step 1: the belt hangs on the joint (60T) pulley. Pinch its near strand at the point that will lie just above
    the motor pulley, carry it in front of the drive over the motor (20T) pulley, into the belt plane, and walk the
    pinch round the motor pulley's far (-y) side at 16.5 mm from its axis, so the belt wraps the small pulley and is
    drawn round its bottom (the loop has ~16 mm of slack; the pinch's detour takes ~15). Then let go."""
    a = c.right if hand == 'right' else c.left
    cen, rot = c.belt()
    ym = c.pose('motor_pulley')[0][1]
    rj = L.r_hinge('joint')
    ktj = nearest_link(cen, [L.BELT_X, L.JOINT_Y, L.AXIS_Z + rj])
    dn = chain_dir(rot, ktj, [0, -1, 0])          # from the joint pulley's top toward its near (-y) side
    y0 = ym - .002
    z1 = L.AXIS_Z + math.sqrt(WRAP_R ** 2 - (y0 - ym) ** 2)
    d = math.hypot(y0 - L.JOINT_Y, z1 - L.AXIS_Z)
    s = math.sqrt(d * d - rj * rj)                # tangent from the pinch to the joint pulley's top
    k = (ktj + dn * (int(round(s / L.LINK_LEN)) + WRAP_DK)) % L.N_LINKS
    c.note('wrap_pick', ktop=ktj, dir=dn, link=k, s_mm=round(s * 1000, 1))
    via = np.array([.30, (a.pos[1] + cen[k, 1]) / 2, L.AXIS_Z + .06])
    yield from pinch(c, a, k, .016, 'wrap', via=via)
    a.comp = True
    r0 = a.rot.copy()
    cen, rot = c.belt()
    side = walk and WRAP_MODE == 'side'
    a.xfix = 0.
    for (pt, ang, dur, label) in (side_waypoints if side else wrap_waypoints)(ym, cen[k, 1], cen[k, 2]):
        r = rot_x(ang) @ r0
        if side and label == 'wrap_in':   # into the belt plane: from here on, hold the pinched link in it
            yield from track_move(c, a, k, np.array(pt), r, dur, label)
        else:
            yield from a.move(pinch_tcp(np.array(pt), r), r, dur, label=label)
    yield from wait(1.0)
    c.note('wrap_on_side' if side else 'wrap_on_top')
    if side:
        yield from walk_to_span(c, a, k, ym, r0)
        if release:
            yield from span_release(c, hand)
        return
    if not walk:  # negative / diagnostic: just let the belt drop over the small pulley from above
        a.comp = False
        yield from a.set_grip(grip_of(.020), .6, 'drape_release')
        yield from a.move(a.pos + [-.045, 0, 0], a.rot, 1.2, label='drape_back_off')
        yield from wait(3.0)
        c.note('draped_only')
        return
    a0 = math.asin((ym - y0) / WRAP_R)            # current angle from the top toward -y
    r1 = rot_x(-90.) @ r0
    n = 90
    alpha = a0
    for kk in range(1, n + 1):
        t = float(c.detail().get('tension_n') or 0.)
        if t >= WRAP_T_STOP:
            break
        # round the far side to below the pulley: the strand toward the joint pulley's bottom then runs in the belt
        # plane under the small pulley's flange (it cannot get there in front of it: no slack to clear the flange)
        alpha = a0 + (math.radians(WRAP_END_DEG) - a0) * kk / n
        p = np.array([L.BELT_X, ym - WRAP_R * math.sin(alpha), L.AXIS_Z + WRAP_R * math.cos(alpha)])
        r = rot_x(math.degrees(alpha)) @ r1
        a.label = 'wrap_round'
        a.set_pose(pinch_tcp(p, r), r)
        yield
    yield from wait(1.0)
    c.note('wrap_round', alpha_deg=round(math.degrees(alpha), 1))
    if release:
        yield from wrap_release(c, hand)


SPAN_S = .045            # release the belt this far along the lower span from the motor pulley's axis (about mid-span)
SPAN_SAG = .020          # ... this far below the span's straight line: the belt wraps both pulleys with ~7 mm of the
                         # loop's 16 mm of slack left, so it lies slack on the inner finger (only its own weight)


def lower_span_z(y, ym):
    """z of the straight lower span (pitch line) between the two pulleys' bottoms at y."""
    s = (y - ym) / (L.JOINT_Y - ym)
    return L.AXIS_Z - L.r_hinge('motor') - s * (L.r_hinge('joint') - L.r_hinge('motor'))


XFIX_GAIN = .1           # per control step: integral correction of the pinch's x from the measured pinched links
XFIX_MAX = .006


def link_x_err(c, k, x_target=L.BELT_X):
    """Target x (default: the belt plane) minus the measured x of the pinched link and its neighbours (m)."""
    cen, _ = c.belt()
    idx = [(k + d) % L.N_LINKS for d in (-1, 0, 1)]
    return x_target - float(np.mean(cen[idx, 0]))


def track_move(c, a, k, p_end, r_end, duration, label, settle=False):
    """Move the pinch (belt mid point) straight to ``p_end`` / ``r_end`` while correcting the commanded x so that the
    MEASURED pinched link lies in the belt plane (the arm sags and the belt sits a little off-centre in the pads, ~2
    mm otherwise; closed loop on what one sees). With ``settle``, then hold until it is within 0.3 mm (<= 3 s)."""
    from yam_arena import ik
    a.label = label
    p0 = a.pos + a.rot[:, 2] * .008 - [a.xfix, 0., 0.]   # current pinch point without the correction
    r0 = a.rot.copy()
    n = max(1, round(duration * CONTROL_HZ))
    for kk in range(1, n + 1):
        s = ik.smoothstep(kk / n)
        r = ik.slerp_matrix(r0, r_end, s)
        p = p0 + s * (p_end - p0)
        a.xfix = float(np.clip(a.xfix + XFIX_GAIN * link_x_err(c, k, a.pos[0] + a.rot[0, 2] * .008 - a.xfix),
                               -XFIX_MAX, XFIX_MAX))
        a.set_pose(pinch_tcp(p + [a.xfix, 0., 0.], r), r)
        yield
    good = 0
    for _ in range(int(3 * CONTROL_HZ) if settle else 0):
        e = link_x_err(c, k)
        good = good + 1 if abs(e) < .0003 else 0
        if good >= 10:
            break
        a.xfix = float(np.clip(a.xfix + XFIX_GAIN * e, -XFIX_MAX, XFIX_MAX))
        a.set_pose(pinch_tcp(np.array([p_end[0] + a.xfix, p_end[1], p_end[2]]), r_end), r_end)
        yield


def walk_to_span(c, a, k, ym, r0):
    """From 9 o'clock on the motor pulley's far side, walk the pinch round the pulley's bottom (16.5 mm from its axis,
    the pads turning with the belt) to 6 o'clock, then out along the lower span to about its middle, a little below
    it: the belt now wraps the motor pulley from its top round its far side and bottom (the belt circulates round the
    joint pulley, which turns), and the pinch ends up away from both pulleys, where letting go does not drag the belt
    over a flange."""
    n = 60
    for kk in range(1, n + 1):
        alpha = math.radians(90. + 90. * kk / n)
        p = np.array([L.BELT_X + a.xfix, ym - WRAP_R * math.sin(alpha), L.AXIS_Z + WRAP_R * math.cos(alpha)])
        r = rot_x(math.degrees(alpha) - 90.) @ r0
        a.label = 'wrap_round'
        a.set_pose(pinch_tcp(p, r), r)
        yield
        a.xfix = float(np.clip(a.xfix + XFIX_GAIN * link_x_err(c, k), -XFIX_MAX, XFIX_MAX))
    y = ym + SPAN_S
    yield from track_move(c, a, k, np.array([L.BELT_X, y, lower_span_z(y, ym) - SPAN_SAG]), a.rot.copy(), 2.5,
                          'wrap_to_span', settle=True)
    yield from wait(1.0)
    c.note('wrap_at_span', x_err_mm=round(link_x_err(c, k) * 1000, 2), xfix_mm=round(a.xfix * 1000, 2))


def span_release(c, hand='right'):
    """Let go of the belt at the lower span: open (the belt hangs slack on the inner finger by its own weight), slide
    straight off it along the pulley axes, open fully."""
    a = c.right if hand == 'right' else c.left
    a.comp = False
    yield from a.set_grip(grip_of(.012), .6, 'wrap_release')
    yield from wait(.8)
    yield from a.move(a.pos + [-.045, 0, 0], a.rot, 2.0, label='wrap_back_off')
    yield from a.set_grip(grip_of(.020), .4, 'wrap_open')
    # clear of the drive along its front before going home (straight home in joint space sweeps the hand through
    # the belt, the motor pulley and the motor)
    yield from a.move(a.pos + [-.035, -.08, 0], a.rot, 1.5, label='wrap_clear')
    yield from a.home(2.0, 'wrap_home')
    yield from wait(.5)
    c.note('round_both')
    d = c.detail()
    require((d.get('joint') or {}).get('around') and (d.get('motor') or {}).get('around'), 'belt round both',
            dict(joint=d.get('joint'), motor=d.get('motor')))


def wrap_release(c, hand='right'):
    """Let go of the belt beside the motor pulley: step 2.5 mm radially out, open 8 mm (the inner finger, between
    belt and pulley, must not swing into the 16 mm flange; 8 mm leaves ~3 mm each side of the belt), slide straight off the belt along the axis, then open
    fully away from the drive."""
    a = c.right if hand == 'right' else c.left
    # first move the pinch 2.5 mm further out from the pulley axis (straight down at the bottom), so the fingers can
    # open 5 mm (clear of the belt on both faces, no drag when backing off) without the inner one reaching the flange
    out = (a.pos + a.rot[:, 2] * .008) - np.array([L.BELT_X, c.pose('motor_pulley')[0][1], L.AXIS_Z])
    out[0] = 0.
    out /= max(np.linalg.norm(out), 1e-9)
    yield from a.move(a.pos + out * .0025, a.rot, .8, label='wrap_out_radial')
    a.comp = False
    yield from a.set_grip(grip_of(.008), .6, 'wrap_release')
    yield from wait(1.0)   # let the freed belt settle onto the pulley before the fingers slide off it
    yield from a.move(a.pos + [-.045, 0, 0], a.rot, 2.5, label='wrap_back_off')
    yield from a.set_grip(grip_of(.020), .4, 'wrap_open')
    yield from wait(1.0)
    c.note('round_both')
    d = c.detail()
    require((d.get('joint') or {}).get('around') and (d.get('motor') or {}).get('around'), 'belt round both',
            dict(joint=d.get('joint'), motor=d.get('motor')))


KNOB_DEPTH = .006        # lever knob centre this far ahead of the TCP (pad tips 7 mm past it)
KNOB_PITCH = 0.0         # knob grasp approach pitch (level: keeps the right wrist low and away from the left arm)
LEVER_TURN_DEG = 93.0    # clamped from 75 deg (the lever springs back ~8 deg when let go); its stop is at 95
PUSH_PITCH_MOTOR = 60.0  # motor push

TENSION_TARGET = 11.0   # N (band 6.2-12.4; 2.25 mm deflection under 1 N): the soft link chain only runs straight in
                        # both spans in the upper half of the band




def tension(c, lock=True, target=TENSION_TARGET, release=None, let_go=True):
    """Step 2: the left hand pushes the motor (and its plate) outward (-y) by the motor body's inner (+y) side, well
    behind the belt plane. If ``release`` (a generator: the right hand letting go of the belt it holds below the motor
    pulley) is given, the plate is first pushed TAKE_UP_MM out to take up part of the slack so the belt stays on when
    the right hand lets go. Then (unless ``target`` is None) it creeps on until the belt tension reaches the target;
    the right hand turns the clamp lever while the left hand holds the motor."""
    a = c.left
    yp = c.pose('motor_block')[0][1]
    rp = tilt_rotation(PUSH_PITCH_MOTOR)
    mb = L.MOTOR_BOX
    edge_y = yp + mb['half']
    tip_z = L.AXIS_Z + .004
    x_c = (mb['x'][0] + mb['x'][1]) / 2 - .002

    def push_tcp(y_edge):
        tip = np.array([x_c, y_edge + PAD_HALF_W + .001, tip_z])
        return tip - rp[:, 2] * PAD_TIP

    yield from a.move(push_tcp(edge_y + .02) + [0, 0, .03], rp, 3.0, 0., 'push_approach', joint=True)
    yield from a.move(push_tcp(edge_y + .004), rp, 1.5, label='push_lower')
    y_cmd = edge_y
    y_end = L.motor_y(L.C_MAX) + .0008
    engaged = False
    a.comp = True   # the arm's joint PD alone gives only ~0.6 N per mm at the fingertip; hold the commanded position
    if release is not None:
        # take up the slack while the right hand still holds the belt round the motor pulley's far side: creep the
        # plate out (command leading the measured motor face by at most 1 mm) until the belt lies on the small pulley
        # between its flanges, or the plate stalls (belt taut round the pinch), or it is 2 mm short of taut
        a.comp = True
        y_stop = L.motor_y(L.C_TAUT - .002)
        last, still = c.pose('motor_block')[0][1], 0
        for _ in range(int(12 * CONTROL_HZ)):
            yb = c.pose('motor_block')[0][1]
            m = c.detail().get('motor') or {}
            if (m.get('between') and (m.get('wrap') or 0) >= 4) or yb <= y_stop or still >= 10:
                break
            still = still + 1 if abs(yb - last) < 2e-5 else 0
            last = yb
            edge = yb + mb['half']
            y_cmd = max(min(y_cmd, edge) - .0002, edge - .001)
            a.label = 'push_take_up'
            a.set_pose(push_tcp(y_cmd), rp)
            yield
        c.note('taken_up', plate_y_mm=round(float(c.pose('motor_block')[0][1]) * 1000, 3), stalled=still)
        # hold the arm where it is (command = its measured joints, integral term off and cleared: no wind-up into a
        # push) while the right hand lets go
        a.comp = False
        a.offset[:] = 0.
        a.q = a.measured.copy()
        yield from release
        a.pos, a.rot = a.measured_tcp()
        a.comp = True
        y_cmd = c.pose('motor_block')[0][1] + mb['half']
    if target is not None:
        for _ in range(int(30 * CONTROL_HZ)):  # creep outward on the measured plate until the tension reaches target
            t = float(c.detail().get('tension_n') or 0.0)
            yb = c.pose('motor_block')[0][1]
            if t >= target or yb <= y_end:
                break
            edge = yb + mb['half']
            if t <= 0.0:
                # slack belt: 0.3 mm per step, the command leading the measured motor face by at most 1 mm
                y_cmd = max(min(y_cmd, edge) - .0003, edge - .001)
            else:
                if not engaged:  # the belt bears (10 N of tension per mm of plate travel): restart from the face
                    engaged, y_cmd = True, edge
                y_cmd -= .00002   # then 0.02 mm per step (~0.2 N of belt tension per step)
            a.label = 'push_out'
            a.set_pose(push_tcp(y_cmd), rp)
            yield
        yield from wait(1.0)
        c.note('tensioned', plate_y_mm=round(float(c.pose('motor_block')[0][1]) * 1000, 3))
        if lock:
            # hold the plate with the integral term frozen: left winding up while blocked by the motor, it slowly
            # redistributed the joint torques and slid the fingertip off the motor's side
            a.freeze = True
            yield from clamp_lever(c)
            # the right wrist camera goes onto the motor pulley while the left hand still holds the motor (the 3 s
            # hands-off hold cannot start yet), so the episode ends with both close-ups on the drive
            yield from look(c, c.right, 'motor', 3.0)
            a.freeze = False
            if not let_go:
                return
            a.comp = False
            yield from a.move(a.pos + [0, .025, .04], a.rot, 1.2, label='push_away')
            yield from look(c, a, 'joint', 2.0)
            c.looking = True
            return
    a.comp = False
    yield from a.move(a.pos + [0, .025, .04], a.rot, 1.2, label='push_away')


def clamp_lever(c):
    """Right hand turns the lever (on the slide's outer side) a quarter turn clockwise (seen from above), handle from
    pointing outward to pointing forward: it grips the knob between its pads (approaching horizontally along the
    handle, which passes between the pads) and carries it round the thread axis; the left hand holds the motor."""
    a = c.right
    lp, lr = c.pose('lever')
    hub = lp.copy()
    lv = L.LEVER
    zk = hub[2] + (lv['hub_z'][1] - lv['hub_z'][0]) / 2 - lv['arm_t'] / 2 + .004   # 4 mm above the knob's middle
    th0 = math.atan2(-lr[1, 1], -lr[0, 1])     # handle direction (the lever's -y axis): outward when loose

    def grasp(theta, follow=False):
        h = np.array([math.cos(theta), math.sin(theta), 0.])
        knob = (c.pose('lever')[0] if follow else hub) + lv['arm'] * h   # follow the thread axis (moves with the plate)
        knob[2] = zk
        p = math.radians(KNOB_PITCH)              # approach along the handle toward the thread axis, pitched down
        gz = -h * math.cos(p) + np.array([0., 0., -math.sin(p)])
        gx = np.cross([0., 0., 1.], h)            # closing axis horizontal, across the handle
        r = np.column_stack([gx, np.cross(gz, gx), gz])
        return knob - gz * KNOB_DEPTH, r

    yield from a.home(2.5, 'lever_home')       # from wherever the hand was, without sweeping through the belt
    p, r = grasp(th0)
    # down onto the knob from 3 cm above it, pads open 30 mm on either side of it (the wrist is near its limit for
    # a horizontal approach from further out)
    yield from a.move(p + [0, 0, .03], r, 3.0, grip_of(.030), 'lever_above', joint=True)
    yield from a.move(p, r, 1.5, label='lever_approach')
    yield from a.set_grip(grip_of(.004), .8, 'lever_grip')
    a.comp = True    # the path follows the thread axis (re-read every step), so the integral term only removes sag
    steps = 60
    for k in range(1, steps + 1):
        a.label = 'lever_turn'
        a.set_pose(*grasp(th0 - math.radians(LEVER_TURN_DEG) * k / steps, follow=True))
        yield
    yield from wait(.5)
    c.note('lever_turned')
    a.comp = False
    yield from a.set_grip(grip_of(.030), .6, 'lever_release')
    yield from a.move(a.pos + [0, 0, .03], a.rot, .8, label='lever_up')
    yield from a.move(a.pos - a.rot[:, 2] * .03, a.rot, .8, label='lever_away')


# wrist camera in the TCP frame (yam_arena station calibration: camera 61 mm behind and 70 mm beside the TCP, looking
# ahead and 25 degrees in toward the fingers)
CAM_IN_TCP = np.array([0.0, -.0696, -.0617])
CAM_VIEW_IN_TCP = np.array([-.005, .428, .904]) / np.linalg.norm([-.005, .428, .904])


def _rot_about(axis, ang):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(ang) * k + (1 - math.cos(ang)) * k @ k


def look_pose(a, target, view, dist):
    """TCP pose whose wrist camera looks at ``target`` along world direction ``view`` from ``dist`` away (the roll
    about the view direction is chosen for the best IK)."""
    w = np.asarray(view, float) / np.linalg.norm(view)
    v = CAM_VIEW_IN_TCP
    ax = np.cross(v, w)
    r0 = _rot_about(ax, math.atan2(np.linalg.norm(ax), float(v @ w))) if np.linalg.norm(ax) > 1e-9 else np.eye(3)
    cam = np.asarray(target, float) - dist * w
    best = None
    for phi in np.radians(np.arange(-180, 180, 20)):
        r = _rot_about(w, phi) @ r0
        p = cam - r @ CAM_IN_TCP
        q, pe, re = a.solve(p, r, iters=150)
        cost = pe * 100 + re + .2 * abs(q[5]) / 2.094
        if best is None or cost < best[0]:
            best = (cost, p, r, pe)
    return best[1], best[2], best[3]


def look(c, a, which, duration):
    """Move arm ``a`` (joint space) so its wrist camera looks at the joint ('joint') or motor ('motor') pulley from in
    front and above, LOOK_DIST away."""
    y = L.JOINT_Y if which == 'joint' else c.pose('motor_pulley')[0][1]
    tgt = np.array([L.BELT_X - .004, y, L.AXIS_Z + .006])
    p, r, pe = look_pose(a, tgt, [math.cos(math.radians(35)), 0., -math.sin(math.radians(35))], LOOK_DIST)
    if pe > .002:
        c.note(f'look_{which}_unreachable', pe_mm=round(pe * 1000, 1))
        return
    yield from a.move(p, r, duration, 0., f'look_{which}', joint=True)


def inspect_pulleys(c):
    """Both hands hold their wrist cameras on the drive (left on the 60T joint pulley, right on the 20T motor pulley)
    from in front and above, fingertips well clear, so the close-ups show the teeth seated between the flanges."""
    ym = c.pose('motor_pulley')[0][1]
    for a, y, label in ((c.left, L.JOINT_Y, 'look_joint'), (c.right, ym, 'look_motor')):
        tgt = np.array([L.BELT_X - .004, y, L.AXIS_Z + .006])
        p, r, pe = look_pose(a, tgt, [math.cos(math.radians(35)), 0., -math.sin(math.radians(35))], LOOK_DIST)
        if pe > .002:
            c.note(f'{label}_unreachable', pe_mm=round(pe * 1000, 1))
            continue
        yield from a.move(p, r, 3.0, 0., label, joint=True)
    yield from wait(.5)
    cen, _ = c.belt()
    tips = []
    for a in (c.left, c.right):
        tp, tr = a.measured_tcp()
        tips.append(tp + tr[:, 2] * PAD_TIP)
    dmin = min(float(np.min(np.linalg.norm(cen - t, axis=1))) for t in tips)
    c.note('inspect', tip_to_belt_mm=round(dmin * 1000, 1))


LOOK_DIST = .14


def hands_off(c, seconds=4.0):
    if getattr(c, 'looking', False):   # both wrist cameras are already on the drive (end of the clamping step)
        yield from wait(seconds)
        c.note('hands_off')
        return
    yield from c.left.home(2.5, 'left_home')
    yield from c.right.home(2.5, 'right_home')
    yield from inspect_pulleys(c)
    yield from wait(seconds)
    c.note('hands_off')


def fit_and_tension(c, lock=True, target=TENSION_TARGET):
    yield from wrap_motor(c)
    yield from tension(c, lock=lock, target=target)


def fit(c):
    yield from fit_and_tension(c, lock=False, target=None)


def demo(c):
    yield from fit_and_tension(c)
    yield from hands_off(c)


def neg_no_tension(c):
    yield from fit(c)
    yield from hands_off(c)


def neg_no_lock(c):
    yield from fit_and_tension(c, lock=False)
    yield from hands_off(c)


def neg_over_tension(c):
    yield from fit_and_tension(c, target=13.5)   # above the 12.4 N band (2 T + friction ~31 N: near the arm's push)
    yield from hands_off(c)


def neg_hands_on(c):
    """Full job, but the left hand never lets go of the motor (it stays pressed on its side after the clamp)."""
    yield from wrap_motor(c)
    yield from tension(c, let_go=False)
    yield from wait(5.0)
    c.note('hands_on')


def neg_drape(c):
    """Diagnostic: lay the belt over the top of the small pulley in its plane and let go, without working it round
    (the 16 mm of slack cannot take the belt round the 16 mm flange in one go); hands off. Expected 0."""
    yield from wrap_motor(c, walk=False)
    yield from hands_off(c)


def disturb(c):
    """Reset-check disturbance: pinch the belt's near strand, pull it off the joint pulley and drop it on the table."""
    a = c.right
    cen, rot = c.belt()
    k = nearest_link(cen, [L.BELT_X, L.JOINT_Y - L.r_hinge('joint'), L.AXIS_Z - .045])
    yield from pinch(c, a, k, .012, 'disturb', via=np.array([.30, (a.pos[1] + cen[k, 1]) / 2, L.AXIS_Z + .06]))
    yield from a.move(a.pos + [-.06, 0, .05], a.rot, 2.0, label='pull_off')
    yield from a.move(a.pos + [-.04, -.08, -.06], a.rot, 2.0, label='carry')
    yield from a.set_grip(grip_of(.03), .5, 'drop')
    yield from a.home(2.0)
    yield from wait(1.0)


STAGES = {
    'idle': lambda c: wait(8.),
    'disturb': disturb,
    'fit': fit,
    'demo': demo,
    'neg-no-tension': neg_no_tension,
    'neg-no-lock': neg_no_lock,
    'neg-over-tension': neg_over_tension,
    'neg-hands-on': neg_hands_on,
    'neg-drape': neg_drape,
}


def script(c, stage='demo'):
    yield from STAGES[stage](c)


def _write_mp4(frames, path):
    import imageio
    with imageio.get_writer(path, fps=CONTROL_HZ, codec='libx264', quality=8, macro_block_size=8,
                            output_params=['-pix_fmt', 'yuv420p', '-movflags', '+faststart']) as w:
        for f in frames:
            w.append_data(np.asarray(f))


def run(task_path, out_dir, max_steps=4800, record=True, stage='demo', cam_width=320, cam_height=176, device=None):
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    t0 = time.time()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    kw = dict(device=device) if device else {}
    emb = yam_arena_embodiment(task=str(Path(task_path).resolve()), object_jitter=0, control_hz=CONTROL_HZ,
                               cam_width=cam_width, cam_height=cam_height, **kw)
    stages = stage.split(',')
    results, resets = [], []
    for i, st in enumerate(stages):
        tdir = out if len(stages) == 1 else out / f'trial{i}_{st}'
        res, reset_state = run_trial(emb, tdir, max_steps, record, st, t0)
        results.append(res)
        resets.append(reset_state)
    if len(stages) > 1:
        rep = reset_report(stages, resets, results)
        (out / 'reset_check.json').write_text(json.dumps(rep, indent=2, default=float))
        print('RESETCHECK', json.dumps(rep['summary']), flush=True)
    return results


def reset_state_now():
    import beltkit
    rt = beltkit.runtime()
    c, _ = rt.belt()
    d = beltkit.STATE
    return dict(score=rt.scorer.score, vetoes=list(rt.scorer.vetoes), instant=rt.scorer.instant,
                lever_deg=round(rt.lever_deg(), 3), clamped=rt.clamped,
                motor_y_mm=round(float(rt.pose('motor_block')[0][1]) * 1000, 3), tension_n=(rt.m or {}).get('tension_n'),
                bodies={n: np.round(rt.pose(n)[0] * 1000, 3).tolist() for n in rt.names if not n.startswith('belt_')},
                links=np.round(c * 1000, 2).tolist(), state_score=d['score'])


def reset_report(stages, resets, results):
    base = resets[0]
    rows = []
    for i, r in enumerate(resets):
        dev = max(float(np.linalg.norm(np.subtract(r['bodies'][n], base['bodies'][n]))) for n in base['bodies'])
        ldev = float(np.abs(np.subtract(r['links'], base['links'])).max())
        rows.append(dict(trial=i, stage=stages[i], score_after_reset=r['score'], vetoes_after_reset=r['vetoes'],
                         lever_after_reset=r['lever_deg'], clamped_after_reset=r['clamped'],
                         motor_y_mm_after_reset=r['motor_y_mm'], tension_after_reset=r['tension_n'], max_body_dev_from_trial0_mm=round(dev, 3),
                         max_link_dev_from_trial0_mm=round(ldev, 3), trial_score=results[i]['task_score'],
                         trial_stop=results[i]['stop']))
    ok = all(x['score_after_reset'] == 0. and not x['vetoes_after_reset'] and not x['clamped_after_reset']
             and x['max_body_dev_from_trial0_mm'] < 1.0 and x['max_link_dev_from_trial0_mm'] < 3.0 for x in rows)
    return dict(summary=dict(passed=ok, trials=len(rows),
                             max_body_dev_mm=max(x['max_body_dev_from_trial0_mm'] for x in rows),
                             max_link_dev_mm=max(x['max_link_dev_from_trial0_mm'] for x in rows)),
                rows=rows, reset_states=resets)


def run_trial(emb, out, max_steps, record, stage, t0):
    from inspect_robots import Scene
    from inspect_robots_yam.packing import STATE_KEY
    import beltkit

    out.mkdir(parents=True, exist_ok=True)
    t1 = time.time()
    obs = emb.reset(Scene(id='demo', instruction='demo'))
    print(f'BENCH reset done {time.time() - t1:.0f} s', flush=True)
    reset_state = reset_state_now()
    c = Ctx(emb)
    gen = script(c, stage)
    frames = {}

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    snap(obs)
    steps, ended, info, stop, success = 0, None, {}, None, False
    nan_seen = False
    belt_trace = []
    tracef = open(out / 'trace.jsonl', 'w')
    while steps < max_steps:
        if ended is None:
            try:
                next(gen)
            except StopIteration:
                ended = steps
            except Stop as e:
                stop, ended = str(e), steps
                print('STOP', stop, flush=True)
        res = emb.step(c.action())
        jp = np.asarray(res.observation.state[STATE_KEY], float).ravel()
        c.left.observe(jp[:7])
        c.right.observe(jp[7:14])
        steps += 1
        c.step = steps
        info = res.info
        snap(res.observation)
        d = beltkit.STATE['detail']
        cen, _ = c.belt()
        nan_seen = nan_seen or bool(np.isnan(cen).any())
        if steps % 10 == 0:
            belt_trace.append(np.r_[steps, cen.ravel(), c.pose('motor_pulley')[0][1]])
            if steps % 100 == 0:
                np.save(out / 'belt_trace.npy', np.array(belt_trace))
        row = dict(step=steps, score=info.get('score'), level=beltkit.STATE['level'], instant=d.get('instant'),
                   vetoes=[v['reason'] for v in beltkit.STATE['vetoes']], cdist=d.get('centre_distance_mm'),
                   band=d.get('band'), lever=d.get('lever_deg'), clamped=d.get('clamped'), hold_s=d.get('hold_s'),
                   tension=d.get('tension_n'), belt_load=d.get('belt_load'), tilt=d.get('block_tilt_deg'),
                   joint={k: (d.get('joint') or {}).get(k) for k in ('around', 'between', 'seated', 'near', 'wrap', 'radial_err_mm')},
                   motor={k: (d.get('motor') or {}).get(k) for k in ('around', 'between', 'seated', 'near', 'wrap', 'radial_err_mm')},
                   spans={k: v.get('max_gap_mm') for k, v in (d.get('spans') or {}).items()},
                   clearance_m=d.get('clearance_m'), touch=d.get('arm_touch_max'),
                   belt_zmin_mm=round((float(np.nanmin(cen[:, 2])) - L.TOP) * 1000, 1), nan=nan_seen,
                   pulley_deg=[round(math.degrees(math.atan2(R[2, 1], R[1, 1])), 2)
                               for R in (c.pose('joint_pulley')[1], c.pose('motor_pulley')[1])],
                   labels=[c.left.label, c.right.label],
                   cmd=np.round(np.r_[c.left.cmd, c.right.cmd], 4).tolist(), q=np.round(jp, 4).tolist())
        tracef.write(json.dumps(row, default=float) + '\n')
        if steps % 20 == 0:
            tracef.flush()
        if steps % 50 == 0:
            print(f"step {steps} {c.left.label}|{c.right.label} score {info.get('score')} inst {d.get('instant')} "
                  f"C {d.get('centre_distance_mm')} T {d.get('tension_n')} lever {d.get('lever_deg')} zmin {row['belt_zmin_mm']} "
                  f"wall {time.time() - t1:.0f}s", flush=True)
        if res.terminated:
            success = res.termination_reason == 'success'
            break
        if ended is not None and steps - ended >= 2 * CONTROL_HZ:
            break
    score = float(info.get('score', 0.) or 0.)
    result = dict(stage=stage, task_score=score, success=success, steps=steps, stop=stop, nan_seen=nan_seen,
                  scorer=dict(score=beltkit.STATE['score'], level=beltkit.STATE['level'],
                              vetoes=beltkit.STATE['vetoes']),
                  final=beltkit.STATE['detail'], looks=c.log,
                  reset_state={k: v for k, v in reset_state.items() if k not in ('bodies', 'links')},
                  wall_s=round(time.time() - t1, 1), wall_total_s=round(time.time() - t0, 1),
                  worst_ik_mm=dict(left=round(c.left.worst_mm, 2), right=round(c.right.worst_mm, 2)),
                  physics_dt=beltkit.PHYSICS_DT, control_hz=CONTROL_HZ)
    tracef.close()
    np.save(out / 'belt_trace.npy', np.array(belt_trace))
    (out / 'result.json').write_text(json.dumps(result, indent=2, default=float))
    if record:
        import imageio
        for cam, fs in frames.items():
            _write_mp4(fs, out / f'demo_{cam}.mp4')
        top = next((k for k in frames if 'top' in k), next(iter(frames), None))
        if top:
            imageio.imwrite(out / 'setup.png', frames[top][0])
            imageio.imwrite(out / 'final.png', frames[top][-1])
    print('RESULT', json.dumps({k: result[k] for k in ('stage', 'task_score', 'success', 'steps', 'stop')}), flush=True)
    return result, reset_state


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('task')
    ap.add_argument('out')
    ap.add_argument('--max-steps', type=int, default=4800)
    ap.add_argument('--no-record', action='store_true')
    ap.add_argument('--stage', default='demo')
    ap.add_argument('--cam', default='320x176')
    ap.add_argument('--device', default=None, help='PhysX device, e.g. cpu (rendering stays on the GPU)')
    a = ap.parse_args()
    w, h = map(int, a.cam.split('x'))
    run(a.task, a.out, a.max_steps, record=not a.no_record, stage=a.stage, cam_width=w, cam_height=h, device=a.device)
    sys.stdout.flush()
    os._exit(0)
