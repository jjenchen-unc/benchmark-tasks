"""Scorer of the gpu-latch task (engine-independent, numpy only).

Measured every control step from the card, slot and latch poses, the latch-hook contact force and the fingertips:
* out: no point of the card's edge connector (gold-finger tab, retention lip, notch) is inside the slot's volume
  (u 40-145 mm along the slot, within 10 mm across it, below 2 mm above the housing top);
* forced extraction: the retention lip passed the hook without the rocker first clearing the seated card;
* dropped: the card descended by at least 30 mm during one unsupported flight;
* on the mat: every corner of the card's body inside the mat's outline, its lowest corner within 5 mm of the mat's top,
  lying flat (its face normal within 15 deg of vertical) and still (every collision vertex moving slower than 5 mm/s);
* released: every robot fingertip at least 40 mm from the card's body and no robot-card contact.

Score (value at the end of the trial):
  1.0   out, on the mat, released, latch intact, never dropped, for 60 control steps in a row (3 s; ends the trial);
  0.5   out of the slot with the latch intact and never dropped (held, or set down anywhere else);
  0     otherwise: still in the slot (latched), latch broken, or card dropped.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

DROP_M = 0.030  # unsupported vertical travel
MAT_DZ = 0.005
FLAT_DEG = 15.0
STILL_V = 0.005
CLEAR_M = 0.040
HOLD_STEPS = 60  # 3 s: the card stays put on the mat with both hands away
CONTROL_HZ = 20


def placement(vertices, com, velocity, mat_center, mat_half, mat_top, face_cos, mat_load, robot_contact, tip_distance):
    """Evaluate the physical card, including its mounting bracket and rotational motion."""
    corners_inside = bool(np.all(np.abs(vertices[:, :2] - mat_center) <= mat_half))
    gap = float(vertices[:, 2].min() - mat_top)
    on_mat = corners_inside and abs(gap) <= MAT_DZ and face_cos >= np.cos(np.deg2rad(FLAT_DEG)) and mat_load > 0.05
    point_velocities = velocity[:3] + np.cross(velocity[3:], vertices - com)
    max_speed = float(np.linalg.norm(point_velocities, axis=1).max())
    released = tip_distance >= CLEAR_M and not robot_contact
    return corners_inside, gap, on_mat, max_speed, released


@dataclass
class Measure:
    out: bool
    on_mat: bool
    released: bool
    still: bool
    hook_n: float  # latch-hook load this control interval (N, mean)
    fall_v: float  # largest downward speed of the card centre this control interval (m/s, positive down)
    forced: bool  # the lip passed the hook with the latch closed (this control interval)
    detail: dict
    unsupported_descent_m: float = 0.0


class Scorer:
    def __init__(self):
        self.broken = False
        self.dropped = False
        self.ever_out = False
        self.hold = 0
        self.passed = False
        self.level = 0.0
        self.hook_peak = 0.0
        self.last = {}

    def step(self, m: Measure):
        if self.passed:
            self.level = 1.0
            return self.level
        self.hook_peak = max(self.hook_peak, m.hook_n)
        if m.forced:
            self.broken = True
        if m.unsupported_descent_m >= DROP_M:
            self.dropped = True
        self.ever_out = self.ever_out or m.out
        bad = self.broken or self.dropped
        done = m.out and m.on_mat and m.released and m.still and not bad
        self.hold = self.hold + 1 if done else 0
        if self.hold >= HOLD_STEPS:
            self.passed = True
        if self.passed:
            level = 1.0
        elif m.out and not bad:
            level = 0.5
        else:
            level = 0.0
        self.level = level
        self.last = dict(level=level, out=m.out, on_mat=m.on_mat, released=m.released, still=m.still,
                         broken=self.broken, dropped=self.dropped, ever_out=self.ever_out, hold=self.hold,
                         hook_n=round(m.hook_n, 3), hook_peak_n=round(self.hook_peak, 3),
                         unsupported_descent_mm=round(1000 * m.unsupported_descent_m, 2),
                         fall_v=round(m.fall_v, 4), measure=m.detail)
        return level
