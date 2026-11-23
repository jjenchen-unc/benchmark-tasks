"""Pose, guide-entry history, single-fastener and clearance scoring for the Leviton deadfront."""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import layout as L
import numpy as np

FLAT_TOL_M = 0.0010  # back-face corner gap to the tub face (a 1 mm feeler does not enter at any corner)
CLEAR_M = 0.030
DRIVER_CLEAR_M = 0.015
STILL_V_M_S = 0.002
STILL_W_DEG_S = 2.0
HOLD_STEPS = 20  # 1 s at 20 Hz
DROP_V_M_S = 0.25
ROBOT_TOUCH_N = 0.05


def cover_in_panel(pos, rot, panel_origin):
    """Cover frame position relative to the panel origin (panel axes = station axes) and its rotation."""
    return np.asarray(pos, float) - np.asarray(panel_origin, float), np.asarray(rot, float)


def corner_gaps(pos, rot, panel_origin):
    """Panel-frame x (m, + into the wall) of the cover's four back-face corners; 0 = on the tub face plane, negative =
    standing off it toward the robot."""
    p, r = cover_in_panel(pos, rot, panel_origin)
    return [float((p + r @ c)[0]) for c in L.cover_corners_v()]


# Four longitudinal edges of each rectangular shroud, sampled at panel x = 0.
_SHROUD_EDGES = np.array([
    [L.space_center(i) + [0.0, dy, dz]
     for dy in (-L.FACE_W / 2, L.FACE_W / 2)
     for dz in (-L.FACE_H / 2, L.FACE_H / 2)]
    for i in range(len(L.SPACES))
])
_COLUMN_WINDOWS = dict(zip(L.COL_Y, L.window_rects()))
_WINDOWS = np.array([_COLUMN_WINDOWS[column] for column, _ in L.SPACES])


def shrouds_through_windows(pos, rot, panel_origin):
    """Check the complete cross-section at both faces of the steel plate, per shroud."""
    p, r = cover_in_panel(pos, rot, panel_origin)
    axis = r[0]  # panel +x expressed in the cover frame
    if axis[0] <= 1e-8:
        return [False] * len(L.SPACES)
    edges = (_SHROUD_EDGES - p) @ r
    through = np.ones(len(L.SPACES), dtype=bool)
    for face_x in (-L.COVER_T, 0.0):
        # Travel along each panel-fixed edge to intersect the given cover face.
        x = (face_x - edges[:, :, 0]) / axis[0]
        y = edges[:, :, 1] + x * axis[1]
        z = edges[:, :, 2] + x * axis[2]
        through &= ((x >= L.FACE_FRONT) & (x <= L.FACE_BACK)
                    & (y >= _WINDOWS[:, 0, None]) & (y <= _WINDOWS[:, 1, None])
                    & (z >= _WINDOWS[:, 2, None]) & (z <= _WINDOWS[:, 3, None])).all(axis=1)
    return through.tolist()


def guide_state(pos, rot, panel_origin):
    p, r = cover_in_panel(pos, rot, panel_origin)
    points = np.array([[0., y, L.GUIDE_Z] for y in (-L.GUIDE_Y, L.GUIDE_Y)])
    offsets = p + points @ r.T - points
    angle = math.degrees(math.atan2(r[0, 2], r[2, 2]))
    in_guides = bool((abs(offsets) <= [0.004, 0.004, 0.004]).all())
    return in_guides, angle, offsets


def point_cover_distance(point, pos, rot):
    """Distance (m) from a point to the cover plate's bounding slab (windows ignored: conservative for clearance)."""
    v = np.asarray(rot, float).T @ (np.asarray(point, float) - np.asarray(pos, float))
    dx = max(-L.COVER_T - v[0], 0.0, v[0])
    dy = max(abs(v[1]) - L.COVER_W / 2, 0.0)
    dz = max(abs(v[2]) - L.COVER_H / 2, 0.0)
    distances = [math.sqrt(dx*dx + dy*dy + dz*dz)]
    for sign in (-1., 1.):
        centre = L.grip_center(sign)
        radial = np.linalg.norm((v-centre)[1:])
        handle = L.grip_frame(sign).T @ (v-centre)
        distances.append(np.linalg.norm(np.maximum(abs(handle)-L.GRIP_HALF, 0.)))
        for radius, x0, x1 in ((L.MAGNET_R, -.009, -L.COVER_T), (.004, L.GRIP_X, -.009)):
            distances.append(math.hypot(max(0., radial-radius), max(0., x0-v[0], v[0]-x1)))
    return min(distances)


def measure(pos, rot, panel_origin, seated, tips, socket_pt, screw_pts):
    gaps = corner_gaps(pos, rot, panel_origin)
    in_guides, angle, offsets = guide_state(pos, rot, panel_origin)
    tip_d = min(point_cover_distance(t, pos, rot) for t in tips) if tips else 1.0
    drv_d = min(float(np.linalg.norm(np.asarray(socket_pt) - np.asarray(s))) for s in screw_pts)
    p, _ = cover_in_panel(pos, rot, panel_origin)
    return dict(
        gaps_mm=[round(g * 1e3, 3) for g in gaps],
        worst_gap_mm=round(-min(gaps) * 1e3, 3),
        flat=bool(max(abs(g) for g in gaps) <= FLAT_TOL_M),
        shrouds_through=shrouds_through_windows(pos, rot, panel_origin),
        guide_offsets_mm=(offsets*1000).round(2).tolist(),
        in_guides=in_guides,
        guide_angle=round(angle, 2),
        seated=[bool(s) for s in seated],
        tip_mm=round(tip_d * 1e3, 1),
        driver_mm=round(drv_d * 1e3, 1),
        clear=bool(tip_d >= CLEAR_M and drv_d >= DRIVER_CLEAR_M),
        cover_mm=[round(v * 1e3, 2) for v in p],
    )


def level_of(m):
    if not (m["in_guides"] and m["flat"] and all(m["shrouds_through"])):
        return 0.0
    if m["seated"] == [True]:
        return 1.0 if m["clear"] else 0.5
    return 0.25


@dataclass
class Monitor:
    """Per-physics-step latch: dropped cover."""

    guided: bool = False
    dropped: bool = False
    first: dict = field(default_factory=dict)
    t: float = 0.0

    def step(self, dt, vz, robot_on_cover_n, cover_supported):
        self.t += dt
        if vz < -DROP_V_M_S and robot_on_cover_n < ROBOT_TOUCH_N and not cover_supported and not self.dropped:
            self.dropped = True
            self.first["dropped"] = round(self.t, 3)

    def observe_guides(self, pos, rot, origin):
        inside, angle, _ = guide_state(pos, rot, origin)
        if inside and abs(angle - L.INSERT_ANGLE) <= 6:
            self.guided = True
            self.first.setdefault("guided", round(self.t, 3))
        elif not inside:
            self.guided = False

    def vetoes(self):
        return ["dropped"] if self.dropped else []


class Scorer:
    def __init__(self):
        self.hold = 0
        self.last = {}
        self.done = False

    def step(self, m, lin_speed, ang_speed_deg, monitor):
        veto = monitor.vetoes()
        lvl = level_of(m) if monitor.guided and not veto else 0.0
        still = lin_speed < STILL_V_M_S and ang_speed_deg < STILL_W_DEG_S
        if lvl >= 1.0 and still:
            self.hold += 1
        else:
            self.hold = 0
        if lvl >= 1.0 and self.hold < HOLD_STEPS:
            lvl = 0.5  # full credit needs the still, hands-clear state for 1 s
        if self.hold >= HOLD_STEPS:
            self.done = True
        self.last = dict(level=lvl, guided=monitor.guided, veto=veto, hold=self.hold, still=still, measure=m,
                         monitor_first=dict(monitor.first))
        return lvl
