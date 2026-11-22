"""Scorer for the panel-deadfront task (engine-independent, numpy only).

Inputs every control step: the dead-front cover's pose and speed, the panel origin, the two lower screws' state
(seated or not, from the runtime's screw model), the four finger pad tips and the nut driver's socket point, plus one
latched veto from the per-physics-step monitor (``Monitor``): the cover was dropped (it moved down faster than
0.25 m/s, the speed after a 3 mm free fall, while no robot finger touched it).

Levels (score at the end of the trial):
  0.25  cover hung on both lower screws (each shank inside its slot) and flat on the tub: every back-face corner within
        FLAT_TOL of the tub face, with all six breaker shrouds through the matching column openings.
  0.5   + one lower screw seated (head down on the cover).
  1.0   + both screws seated, every fingertip at least CLEAR_M from the cover, the driver off the screws, cover still
        for 1 s. Ends the trial.
  0     otherwise (cover not hung, held off flat by a breaker shroud or a crooked hang, dropped).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import layout as L
import numpy as np

FLAT_TOL_M = 0.0010  # back-face corner gap to the tub face (a 1 mm feeler does not enter at any corner)
SLOT_MARGIN_M = 0.0005
SEAT_TOL_M = 0.0020  # slot top within 2 mm of the shank: the cover sits on the screws at its height
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


def shank_in_slot(i, pos, rot, panel_origin):
    """Screw i's axis (at the cover's mid-thickness) in the cover frame: (dy from the slot centre, z) and whether the
    shank is inside the open-bottom slot with the slot's top resting on it (within SEAT_TOL_M)."""
    p, r = cover_in_panel(pos, rot, panel_origin)
    a = np.array([-L.COVER_T / 2, L.SCREW_Y[i], L.SCREW_Z])
    v = r.T @ (a - p)
    dy = float(v[1] - L.SCREW_Y[i])
    z = float(v[2])
    seat = L.SLOT_TOP - L.SHANK_R  # shank axis z when the slot top rests on it
    inside = abs(dy) <= L.SLOT_W / 2 - L.SHANK_R + SLOT_MARGIN_M and seat - SEAT_TOL_M <= z <= seat + SLOT_MARGIN_M
    return dy, z, inside


def point_cover_distance(point, pos, rot):
    """Distance (m) from a point to the cover plate's bounding slab (windows ignored: conservative for clearance)."""
    v = np.asarray(rot, float).T @ (np.asarray(point, float) - np.asarray(pos, float))
    dx = max(-L.COVER_T - v[0], 0.0, v[0])
    dy = max(abs(v[1]) - L.COVER_W / 2, 0.0)
    dz = max(abs(v[2]) - L.COVER_H / 2, 0.0)
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def measure(pos, rot, panel_origin, seated, tips, socket_pt, screw_pts):
    gaps = corner_gaps(pos, rot, panel_origin)
    slots = [shank_in_slot(i, pos, rot, panel_origin) for i in (0, 1)]
    tip_d = min(point_cover_distance(t, pos, rot) for t in tips) if tips else 1.0
    drv_d = min(float(np.linalg.norm(np.asarray(socket_pt) - np.asarray(s))) for s in screw_pts)
    p, _ = cover_in_panel(pos, rot, panel_origin)
    return dict(
        gaps_mm=[round(g * 1e3, 3) for g in gaps],
        worst_gap_mm=round(-min(gaps) * 1e3, 3),
        flat=bool(max(abs(g) for g in gaps) <= FLAT_TOL_M),
        shrouds_through=shrouds_through_windows(pos, rot, panel_origin),
        slot_dy_mm=[round(s[0] * 1e3, 3) for s in slots],
        slot_z_mm=[round(s[1] * 1e3, 3) for s in slots],
        hung=bool(all(s[2] for s in slots)),
        seated=[bool(s) for s in seated],
        tip_mm=round(tip_d * 1e3, 1),
        driver_mm=round(drv_d * 1e3, 1),
        clear=bool(tip_d >= CLEAR_M and drv_d >= DRIVER_CLEAR_M),
        cover_mm=[round(v * 1e3, 2) for v in p],
    )


def level_of(m):
    if not (m["hung"] and m["flat"] and all(m["shrouds_through"])):
        return 0.0
    n = sum(m["seated"])
    if n == 2:
        return 1.0 if m["clear"] else 0.5
    if n == 1:
        return 0.5
    return 0.25


@dataclass
class Monitor:
    """Per-physics-step latch: dropped cover."""

    dropped: bool = False
    first: dict = field(default_factory=dict)
    t: float = 0.0

    def step(self, dt, vz, robot_on_cover_n, cover_supported):
        self.t += dt
        if vz < -DROP_V_M_S and robot_on_cover_n < ROBOT_TOUCH_N and not cover_supported and not self.dropped:
            self.dropped = True
            self.first["dropped"] = round(self.t, 3)

    def vetoes(self):
        return ["dropped"] if self.dropped else []


class Scorer:
    def __init__(self):
        self.hold = 0
        self.last = {}
        self.done = False

    def step(self, m, lin_speed, ang_speed_deg, monitor):
        veto = monitor.vetoes()
        lvl = 0.0 if veto else level_of(m)
        still = lin_speed < STILL_V_M_S and ang_speed_deg < STILL_W_DEG_S
        if lvl >= 1.0 and still:
            self.hold += 1
        else:
            self.hold = 0
        if lvl >= 1.0 and self.hold < HOLD_STEPS:
            lvl = 0.5  # full credit needs the still, hands-clear state for 1 s
        if self.hold >= HOLD_STEPS:
            self.done = True
        self.last = dict(level=lvl, veto=veto, hold=self.hold, still=still, measure=m,
                         monitor_first=dict(monitor.first))
        return lvl
