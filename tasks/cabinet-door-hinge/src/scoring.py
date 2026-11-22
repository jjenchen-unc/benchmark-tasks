"""Scorer for the cabinet-door-hinge task (engine-independent, numpy only).

Inputs every control step: the door's and the two hinge arms' poses, the door's speed, the two knuckle angles (door
opening, deg), the two hinges' plate states (free / hooked / clipped) and the four finger pad tips. Read at the end
of the trial (person check in brackets):

  1.0   both hinges clipped onto their plates (each arm's rear latch closed: pull the door gently, it stays on), the
        door swung shut (opening at most 3 deg: the door lies against the carcass front, gaps even, top edge level with
        the carcass top within 1.5 mm), every fingertip 30 mm or more from the door and the hinges, door still, held
        for 1 s. Ends the trial.
  0.5   both hinges clipped, door not shut (same hands-off and still conditions).
  0.25  exactly one hinge clipped (the door hangs on it; same conditions).
  0     otherwise (no hinge clipped: the door is on the bench, in the groove, or still held).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import layout as L
import numpy as np

CLOSED_DEG = 3.0
REVEAL_TOL_M = 0.0015
CLEAR_M = 0.030
STILL_V_M_S = 0.003
STILL_W_DEG_S = 1.5
HOLD_STEPS = 20  # 1 s at 20 Hz
LEVELS = {2: 0.5, 1: 0.25, 0: 0.0}


def point_box_dist(p, bp, br, center, size):
    """Distance from world point p to a box (center/size in the body frame (bp, br))."""
    q = br.T @ (np.asarray(p, float) - bp) - np.asarray(center, float)
    d = np.abs(q) - np.asarray(size, float) / 2
    return float(np.linalg.norm(np.maximum(d, 0.0)) + min(max(d.max(), -1.0), 0.0))


def clearance(tips, poses):
    """Smallest fingertip distance (m) to the door slab and the two hinge arms (channel + neck/boss envelope)."""
    if not tips:
        return 1.0
    boxes = []
    dp, dr = poses["door"]
    for _, c, s in L.door_pieces():
        boxes.append((dp, dr, c, s))
    for i in range(2):
        ap, ar = poses[f"arm{i}"]
        boxes.append((ap, ar, (-0.030, -0.002, 0.0), (0.120, 0.030, 0.030)))  # arm + neck + boss envelope
    return float(min(point_box_dist(t, *b) for t in tips for b in boxes))


def reveal_error(poses, spec):
    """Door top edge height minus its design height when shut (m)."""
    dp, dr = poses["door"]
    top = dp + dr @ np.array([L.DOOR_W / 2, 0.0, L.DOOR_H])
    want = L.Z_CAB + L.DOOR_DZ + L.DOOR_H
    return float(top[2] - want)


@dataclass
class Measure:
    hinges: tuple
    n_clipped: int
    alpha_deg: tuple
    closed: bool
    reveal_mm: float
    clear_mm: float
    still: bool
    door_z_mm: float


def measure(poses, speeds, alphas, hinge_states, tips, spec):
    n = sum(1 for s in hinge_states if s == "clipped")
    rev = reveal_error(poses, spec)
    closed = bool(max(alphas) <= CLOSED_DEG and abs(rev) <= REVEAL_TOL_M)
    return Measure(
        hinges=tuple(hinge_states),
        n_clipped=n,
        alpha_deg=tuple(round(a, 2) for a in alphas),
        closed=closed,
        reveal_mm=rev * 1e3,
        clear_mm=clearance(tips, poses) * 1e3,
        still=bool(speeds[0] <= STILL_V_M_S and speeds[1] <= STILL_W_DEG_S),
        door_z_mm=float(poses["door"][0][2]) * 1e3,
    )


def instant_level(m: Measure):
    """Level of the current state ignoring the 1 s hold."""
    if m.clear_mm < CLEAR_M * 1e3:
        return 0.0
    if m.n_clipped == 2 and m.closed:
        return 1.0
    return LEVELS[m.n_clipped]


@dataclass
class Scorer:
    hold: int = 0
    level: float = 0.0
    score: float = 0.0
    last: dict = field(default_factory=dict)
    peak: dict = field(default_factory=lambda: dict(clipped=0))

    def step(self, m: Measure):
        self.peak["clipped"] = max(self.peak["clipped"], m.n_clipped)
        lv = instant_level(m)
        if lv > 0 and m.still:
            self.hold += 1
        else:
            self.hold = 0
        self.level = lv if self.hold >= HOLD_STEPS else 0.0
        self.score = self.level
        self.last = dict(
            measure={k: (round(v, 3) if isinstance(v, float) else v) for k, v in m.__dict__.items()},
            instant=lv,
            hold=self.hold,
            level=self.level,
            peak=dict(self.peak),
        )
        return self.level

