"""Scorer of the nm-cable-box task (engine-independent, numpy only).

Measured every control step from the cable link poses, the clamp tab angle and the fingertip positions:
* through: the cable passes through the box's entry opening and at least 20 mm of it is past the wall's inner face,
  with the clamp tab lifted onto the sheath (the angle is at least one degree above the modeled rest stop): the tab bears on the cable, so it
  cannot be pulled back out;
* inside_mm: length of cable past the wall's inner face, along the cable to its cut end (a person: a tape along the
  cable from the clamp to the cut end);
* released: every fingertip at least 120 mm from every point of the cable (hands clear);
* still: no point of the 200 mm of cable at the cut end moved more than 5 mm over the last second.

Score (value at the end of the trial):
  1.0  through, 130-170 mm inside, released, still for 1 s (ends the trial);
  0.5  through, released and still for 1 s, but the length is outside 130-170 mm;
  0    otherwise (not through the clamp, cable held, or cable moving).
"""

from __future__ import annotations

from dataclasses import dataclass

import layout as L
import numpy as np

CLEAR_M = 0.120  # both hands clear of the cable (the overhead camera then sees the result)
STILL_V_M_S = 0.005
HOLD_STEPS = 20  # 1 s at 20 Hz
TAB_THROUGH_DEG = L.TAB_REST_DEG + 1.0  # The sheath lifts the tongue at least one degree off the rest stop.


def seg_point_distance(p, a, b):
    ab = b - a
    t = float(np.clip((p - a) @ ab / max(ab @ ab, 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def cable_distance(point, pts):
    p = np.asarray(point, float)
    return min(seg_point_distance(p, pts[i], pts[i + 1]) for i in range(len(pts) - 1))


@dataclass
class Measure:
    inside_mm: float | None
    through: bool
    tab_deg: float
    tip_mm: float
    end_mm: list


def measure(spec, links, tab_deg, tips):
    pts = L.cable_points(links)
    ins = L.inserted_length(links, spec)
    lifted = tab_deg >= TAB_THROUGH_DEG
    through = ins is not None and ins >= L.THROUGH_MIN and lifted
    tip = min(cable_distance(t, pts) for t in tips) if tips else 1.0
    return Measure(
        inside_mm=None if ins is None else ins * 1e3,
        through=bool(through),
        tab_deg=float(tab_deg),
        tip_mm=tip * 1e3,
        end_mm=np.round(pts[0] * 1e3, 2).tolist(),
    )


def level_of(m: Measure, released: bool, still_steps: int):
    if not (m.through and released and still_steps >= HOLD_STEPS):
        return 0.0
    if abs(m.inside_mm - L.TARGET_IN * 1e3) <= L.TOL_IN * 1e3:
        return 1.0
    return 0.5


class Scorer:
    def __init__(self):
        self.still = 0
        self.level = 0.0
        self.last = {}

    def step(self, m: Measure, lin_v):
        released = m.tip_mm >= CLEAR_M * 1e3
        self.still = self.still + 1 if (m.through and released and lin_v < STILL_V_M_S) else 0
        level = level_of(m, released, self.still)
        self.level = level
        self.last = dict(
            level=level,
            still_steps=self.still,
            measure=dict(
                inside_mm=None if m.inside_mm is None else round(m.inside_mm, 2),
                through=m.through,
                tab_deg=round(m.tab_deg, 2),
                tip_mm=round(m.tip_mm, 2),
                end_mm=m.end_mm,
            ),
            lin_v_mm_s=round(lin_v * 1e3, 3),
        )
        return level
