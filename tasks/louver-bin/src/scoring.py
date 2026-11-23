"""Scorer of the louver-bin task (engine-independent, numpy only).

A bin HANGS on louver row k when its hanger tab is down in that row's channel (both ends of the tab between the lip's
back face and the panel face, at least 4 mm below the lip top, layout.hang_state) and the bin is level within 5 deg.
Nothing in the simulation holds a bin up except contact: a tab resting on top of the lip, or in front of it, is not in
the channel, and such a bin tips forward and drops when let go.

Score (value at the end of the trial, ``Scorer.step`` every control step):
  1.0  the new bin hangs on the SAME row as the bin that was already there, in the panel's other half, both of its
       tab ends on the louver, not overlapping the other bin; the other bin still hangs on its row; every fingertip
       5 cm or more from both bins; both bins still for 2 s (ends the trial);
  0.5  the same, but a hand is still on or within 5 cm of a bin (not let go) at the end;
  0    otherwise: on another row, resting on the lip edge, fallen, the other bin knocked off its row, or overlapping.
"""

from __future__ import annotations

from dataclasses import dataclass

import layout as L
import numpy as np

CLEAR_M = 0.050
LEVEL_DEG = 5.0
STILL_V_M_S = 0.003
HOLD_STEPS = 40  # 2 s at 20 Hz
MIN_GAP_M = 0.003  # the two bins must not overlap sideways (a few mm of air between them)


def box_distance(p, x0, x1, y0, y1, z0, z1):
    d = np.maximum([x0 - p[0], y0 - p[1], z0 - p[2]], [p[0] - x1, p[1] - y1, p[2] - z1])
    return float(np.linalg.norm(np.maximum(d, 0.0)))


def bin_distance(point, pos, rot):
    """Distance from a point to the bin's outer box (body plus hanger)."""
    q = np.asarray(rot).T @ (np.asarray(point, float) - np.asarray(pos, float))
    w = L.BIN_W / 2 + L.RIM_BEAD
    return box_distance(q, -L.BIN_L, L.HANGER_X, -w, w, -L.BIN_H, L.HANGER_T)


@dataclass
class Measure:
    target_row: int
    target_side: int
    bin: dict
    ref: dict
    tip_mm: float
    hangs: bool
    ref_hangs: bool
    side_ok: bool
    gap_mm: float


def hangs_on(st, row):
    return bool(st["in_channel"] and st["on_panel"] and st["row"] == row and st["level_deg"] <= LEVEL_DEG)


def measure(spec, pb, rb, pr, rr, tips):
    b = L.hang_state(spec, pb, rb)
    r = L.hang_state(spec, pr, rr)
    tip = min(min(bin_distance(t, pb, rb), bin_distance(t, pr, rr)) for t in tips) if tips else 1.0
    target_side = -spec["side"]
    yc = L.panel_origin(spec)[1]
    side_ok = bool(target_side * b["y_rel"] > 0)
    # sideways air gap between the two bins (negative = overlap)
    gap = max(r["y_lo"] - b["y_hi"], b["y_lo"] - r["y_hi"])
    _ = yc
    return Measure(target_row=spec["row"], target_side=target_side, bin=b, ref=r, tip_mm=tip * 1e3,
                   hangs=hangs_on(b, spec["row"]), ref_hangs=hangs_on(r, spec["row"]), side_ok=side_ok,
                   gap_mm=gap * 1e3)


def placed(m: Measure):
    return m.hangs and m.ref_hangs and m.side_ok and m.gap_mm >= MIN_GAP_M * 1e3


def level_of(m: Measure, clear: bool, still_steps: int):
    if not placed(m):
        return 0.0
    return 1.0 if (clear and still_steps >= HOLD_STEPS) else 0.5


class Scorer:
    def __init__(self):
        self.still = 0
        self.level = 0.0
        self.last = {}

    def step(self, m: Measure, lin_v):
        clear = m.tip_mm >= CLEAR_M * 1e3
        self.still = self.still + 1 if (placed(m) and clear and lin_v < STILL_V_M_S) else 0
        level = level_of(m, clear, self.still)
        self.level = level
        r = lambda d: {k: (round(v, 5) if isinstance(v, float) else v) for k, v in d.items()}  # noqa: E731
        self.last = dict(
            level=level,
            still_steps=self.still,
            measure=dict(target_row=m.target_row, target_side=m.target_side, bin=r(m.bin), ref=r(m.ref),
                         hangs=m.hangs, ref_hangs=m.ref_hangs, side_ok=m.side_ok, gap_mm=round(m.gap_mm, 2),
                         tip_mm=round(m.tip_mm, 2)),
            lin_v_mm_s=round(lin_v * 1e3, 3),
        )
        return level
