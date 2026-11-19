"""timing-belt scorer: pure functions on measured poses (no simulator). Unit tests: ../tests/test_scoring.py.

Levels (task.json "scoring"):
  0.25 belt round both pulleys (both inside the loop) and, wherever it is near a pulley, between its flanges;
  0.5  + belt tension (each span) within TENSION_BAND (6.2-12.4 N; person check: 2-4 mm of mid-span deflection under a
       1 N push), teeth seated round both wraps (hinges on the wrapped-hinge circle, teeth facing the pulley), both
       straight spans straight within 1 mm;
  0.75 + clamp lever turned to its clamped position (>= 75 deg clockwise from loose);
  1.0  + both grippers clear (no arm contact with belt / pulleys / motor plate / lever, every fingertip >= 3 cm from
       the belt and both pulleys) for 3 s while the 0.75 state holds.
Veto (0 for the trial): the belt falls off the table (any link more than 4 cm below the table top).
The trial's score is the level at its end (with the 3 s hold accumulated for 1.0).
"""
from __future__ import annotations

import math

import numpy as np

import layout as L

NEAR_MARGIN = 0.002         # a hinge within flange radius + 2 mm of a pulley axis is "at" that pulley
FLANGE_GAP_TOL = 0.0012     # link centre within +-1.2 mm (x) of the tooth-zone centre: between the flanges (6 mm
                            # belt in a 7 mm gap = 0.5 mm play, + 0.7 mm measurement/tilt tolerance)
WRAP_MARGIN = 0.0009        # hinges closer than R_hinge + 0.9 mm to the axis touch it (the first span hinge of a
                            # taut belt is at +1.14 mm on the 20T, +0.41 mm on the 60T; a tooth riding on a pulley
                            # tooth lifts its hinges up to +0.75 mm)
SEAT_TOL = 0.0004           # seated: wrapped hinge radius within R_HINGE +- 0.4 mm (a tooth on a tooth: +0.75 mm)
MIN_WRAP_HINGES = {'joint': 13, 'motor': 4}  # seated also needs most of the wrapped arc on the hinge circle: a taut
                            # belt has ~17-20 wrapped hinges on the 60T and ~6 on the 20T; a stretch standing on edge
                            # (twisted) in the 60T's flange gap leaves its hinges ~3 mm out (only the rest of the arc
                            # would otherwise be checked)
SPAN_TOL = 0.003            # straightedge gap along each span (a person's check: 1 mm; the simulated 0.1 g-link chain
                            # carries well under 1 N itself, so its spans sag and vibrate by up to about 2.8 mm even
                            # when the modelled tension is in the band)
MIN_SPAN_HINGES = 18        # 29 inner hinges per span at C_TAUT
CLEARANCE_M = 0.030
HOLD_S = 3.0
FALL_DEPTH_M = 0.040
LEVELS = (0.25, 0.5, 0.75, 1.0)


def winding(poly2, point2):
    """Winding number of the closed polygon (N,2) around point2."""
    p = np.asarray(poly2, float) - np.asarray(point2, float)
    a = np.arctan2(p[:, 1], p[:, 0])
    d = np.diff(np.r_[a, a[:1]])
    d = (d + np.pi) % (2 * np.pi) - np.pi
    return int(round(d.sum() / (2 * np.pi)))


def pulley_check(hinges, centres, rots, axis_yz, kind):
    """Belt state at one pulley. ``hinges`` [N,3] (start hinge of each link), ``centres`` [N,3] link centres,
    ``rots`` [N,3,3] link rotations, ``axis_yz`` the pulley axis point (y, z); the tooth-zone centre is x = BELT_X."""
    h = np.asarray(hinges, float)
    c = np.asarray(centres, float)
    yz = h[:, 1:] - np.asarray(axis_yz, float)
    r = np.linalg.norm(yz, axis=1)
    around = winding(h[:, 1:], axis_yz) != 0
    near = r < L.R_FLANGE[kind] + NEAR_MARGIN
    # a hinge is shared by link i-1 and link i: use both links' centres for the axial (between-flanges) test
    idx = np.nonzero(near)[0]
    dx = np.array([max(abs(c[i, 0] - L.BELT_X), abs(c[i - 1, 0] - L.BELT_X)) for i in idx]) if len(idx) else np.zeros(0)
    between = bool(len(idx) >= 2 and np.all(dx <= FLANGE_GAP_TOL))
    rh = L.r_hinge(kind)
    wrap = np.nonzero(r < rh + WRAP_MARGIN)[0]
    radial_err = (r[wrap] - rh) if len(wrap) else np.zeros(0)
    # teeth toward the pulley: the outward normal (local z) of both links at a wrapped hinge points away from the axis
    facing = True
    for i in wrap:
        for j in (i, i - 1):
            n = np.asarray(rots[j], float)[:, 2]
            v = np.r_[0.0, c[j, 1:] - np.asarray(axis_yz, float)]
            if np.dot(n, v) <= 0:
                facing = False
    dxw = np.array([max(abs(c[i, 0] - L.BELT_X), abs(c[i - 1, 0] - L.BELT_X)) for i in wrap]) if len(wrap) else np.zeros(0)
    # the first and last hinge of each contiguous run along the belt are where it leaves the pulley: on the 60T
    # pulley the first span hinge of a taut belt is only 0.41 mm outside the hinge circle, so those ends may be up
    # to WRAP_MARGIN out; every hinge inside a run must be within SEAT_TOL
    n_links = len(h)
    inset = set(int(i) for i in wrap)
    ends = {i for i in inset if (i - 1) % n_links not in inset or (i + 1) % n_links not in inset}
    inner_err = np.array([r[i] - rh for i in wrap if int(i) not in ends]) if len(wrap) else np.zeros(0)
    seated = bool(len(wrap) >= MIN_WRAP_HINGES[kind] and np.all(np.abs(inner_err) <= SEAT_TOL)
                  and np.all(radial_err >= -SEAT_TOL) and facing and np.all(dxw <= FLANGE_GAP_TOL))
    return dict(around=bool(around), near=int(len(idx)), between=between, wrap=int(len(wrap)), seated=seated,
                facing=bool(facing), max_dx_mm=round(float(dx.max()) * 1000, 3) if len(dx) else None,
                radial_err_mm=[round(float(e) * 1000, 3) for e in radial_err])


def tangent_line(c1, r1, c2, r2, upper=True):
    """External tangent of two circles in the (y, z) plane: unit normal n (pointing away from the circles) and
    offset d with n.p = d on the line (both circles on the inner side, at distances r1, r2)."""
    c1, c2 = np.asarray(c1, float), np.asarray(c2, float)
    v = c2 - c1
    dist = float(np.linalg.norm(v))
    e = v / dist
    perp = np.array([-e[1], e[0]])
    if (perp[1] < 0) == upper:
        perp = -perp
    s = (r1 - r2) / dist                  # n.e = (r1 - r2) / dist
    n = np.sqrt(max(0.0, 1 - s * s)) * perp + s * e
    return n, float(n @ c1 + r1)


def span_check(hinges, joint_yz, motor_yz):
    """Straightness of the top and bottom spans: hinges between the pulleys, clear of both flanges, measured
    against the pulleys' external tangent (where a taut belt runs)."""
    h = np.asarray(hinges, float)
    cj, cm = np.asarray(joint_yz, float), np.asarray(motor_yz, float)
    yz = h[:, 1:]
    far = (np.linalg.norm(yz - cj, axis=1) > L.R_FLANGE['joint'] + .002) & \
          (np.linalg.norm(yz - cm, axis=1) > L.R_FLANGE['motor'] + .002)
    inner = far & (yz[:, 0] > cm[0]) & (yz[:, 0] < cj[0])
    out = {}
    for name, upper in (('top', True), ('bottom', False)):
        n, d = tangent_line(cj, L.r_hinge('joint'), cm, L.r_hinge('motor'), upper)
        side = (yz[:, 1] > (cj[1] + cm[1]) / 2) if upper else (yz[:, 1] < (cj[1] + cm[1]) / 2)
        sel = inner & side
        err = np.abs(yz[sel] @ n - d) if sel.any() else np.zeros(0)
        dxs = np.abs(h[sel, 0] - L.BELT_X) if sel.any() else np.zeros(0)
        out[name] = dict(n=int(sel.sum()), max_gap_mm=round(float(err.max()) * 1000, 3) if len(err) else None,
                         straight=bool(sel.sum() >= MIN_SPAN_HINGES and err.max() <= SPAN_TOL
                                       and dxs.max() <= FLANGE_GAP_TOL))
    return out


def belt_tension(on_drive, centre_distance_m):
    """Belt tension per span (N) from geometry: 0 unless the belt is round both pulleys between their flanges; else
    PLATE_K / 2 per metre of centre distance past C_TENSION_ZERO (where the simulated belt is just taut with its teeth in
    the grooves;
    teeth riding on tooth tips do not change the model, they fail the scorer's seating check instead)."""
    if not on_drive:
        return 0.0
    return float(min(max(0.0, L.PLATE_K / 2 * (centre_distance_m - L.C_TENSION_ZERO)), L.BELT_T_CAP))


def measure(link_centres, link_rots, motor_y, lever_deg, tension_n=None, fingertips=(), arm_touch=None):
    """Every scoring signal from one set of measured poses (world, env-relative). ``motor_y``: y of the sliding
    motor pulley's axis; ``tension_n``: belt tension per span, or None to compute it with ``belt_tension`` from this
    measurement."""
    from layout import hinge_points
    c = np.asarray(link_centres, float)
    rots = np.asarray(link_rots, float)
    h, _ = hinge_points(c, rots)
    yj, za = L.JOINT_Y, L.AXIS_Z
    joint = pulley_check(h, c, rots, (yj, za), 'joint')
    motor = pulley_check(h, c, rots, (motor_y, za), 'motor')
    spans = span_check(h, (yj, za), (motor_y, za))
    if tension_n is None:
        on_drive = joint['around'] and motor['around'] and joint['between'] and motor['between']
        tension_n = belt_tension(on_drive, yj - motor_y)
    lo, hi = L.TENSION_BAND
    clear = np.inf
    pts = np.vstack([h, [[L.BELT_X, yj, za], [L.BELT_X, motor_y, za]]])
    radii = np.r_[np.zeros(len(h)), L.R_FLANGE['joint'], L.R_FLANGE['motor']]
    for tip in fingertips:
        d = np.linalg.norm(pts - np.asarray(tip, float), axis=1) - radii
        clear = min(clear, float(d.min()))
    touched = dict(arm_touch or {})
    fell = bool(np.any(c[:, 2] < L.TOP - FALL_DEPTH_M))
    return dict(joint=joint, motor=motor, spans=spans, centre_distance_mm=round((yj - motor_y) * 1000, 3),
                tension_n=round(float(tension_n), 3), band=bool(lo <= tension_n <= hi),
                lever_deg=round(float(lever_deg), 2), locked=bool(lever_deg <= L.LEVER_LOCK_DEG),
                clearance_m=clear, touched=touched, released=bool(clear >= CLEARANCE_M and not touched), fell=fell)


def instant_level(m):
    """Level reached by one measurement, before the hands-off hold (0, 0.25, 0.5, 0.75; 0.75 + released -> 'ready')."""
    if not (m['joint']['around'] and m['motor']['around'] and m['joint']['between'] and m['motor']['between']):
        return 0.0
    if not (m['band'] and m['joint']['seated'] and m['motor']['seated'] and m['spans']['top']['straight']
            and m['spans']['bottom']['straight']):
        return 0.25
    if not m['locked']:
        return 0.5
    return 0.75


class BeltScorer:
    """Steps on measurements; latches the fall veto; times the 3 s hands-off hold."""

    def __init__(self):
        self.vetoes = []
        self.hold_s = 0.0
        self.level = 0.0
        self.instant = 0.0
        self.best = 0.0
        self.t = 0.0

    def step(self, dt, m):
        self.t += dt
        if m['fell'] and not any(v['reason'] == 'belt fell off the table' for v in self.vetoes):
            self.vetoes.append(dict(reason='belt fell off the table', t=round(self.t, 4)))
        lvl = instant_level(m)
        self.instant = lvl
        if lvl >= 0.75 and m['released']:
            self.hold_s += dt
        else:
            self.hold_s = 0.0
        self.level = 1.0 if (lvl >= 0.75 and self.hold_s >= HOLD_S - 1e-9) else lvl
        self.best = max(self.best, self.level)
        return self.score

    @property
    def score(self):
        return 0.0 if self.vetoes else float(self.level)
