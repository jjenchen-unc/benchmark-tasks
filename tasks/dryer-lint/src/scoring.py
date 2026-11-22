"""Scorer for the dryer-lint task (engine-independent, numpy only).

Inputs every control step: the lint screen's pose, the lint mat's pose, the dryer and bin positions, whether the trial
has lint, whether the mat's breakaway joint still holds it to the screen, and whether the dryer has been started (the
Start button pressed with at least 4 N for 0.5 s; read by the runtime from the button's contact force). The state of
the screen and the lint AT THE MOMENT OF START is latched: starting first and fixing things afterwards does not count.

Levels (score at the end of the trial):
  1.0  started while the screen was fully seated (grip down on the cabinet top at both ends, within 2.5 mm, right
       way round) and clean (no lint on it, none dropped into the dryer), still seated, and the lint (if any) is in the
       bin; held 1 s. Ends the trial.
  0.5  as 1.0, but the lint lies outside the bin (on the dryer top, the table or the floor).
  0.25 never started, but the lint is in the bin and the screen seated at the end.
  0    started with lint on the screen, lint inside the dryer, or the screen out or not seated (more than 2.5 mm proud
       at an end; a person sees the gap or feels the step); or the screen left out / dirty and nothing started.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import layout as L
import numpy as np

SEAT_TOL_M = 0.0025  # grip bottom within 2.5 mm of the cabinet top at both ends
SEAT_XY_M = 0.008  # screen within 8 mm of its slot position (it can only be lower than proud when in the slot)
SEAT_TILT_DEG = 3.0
ON_SCREEN_M = 0.006  # mat back face within 6 mm of the screen's front face, over its face
HOLD_STEPS = 20  # 1 s at 20 Hz


def tilt_deg(rot):
    return math.degrees(math.acos(float(np.clip(rot[2, 2], -1.0, 1.0))))


@dataclass
class Measure:
    end_gap_mm: list  # grip bottom above the cabinet top at the -y / +y ends (mm)
    dxy_mm: list  # screen offset from its seated position (x, y) in mm
    tilt_deg: float
    right_way: bool  # thin end toward +y, back face toward +x
    seated: bool
    lint: bool  # this trial has a lint mat
    bonded: bool  # breakaway joint still holds the mat to the screen
    mat_on_screen: bool
    mat_in_dryer: bool
    mat_in_bin: bool
    clean: bool  # no lint on the screen and none inside the dryer
    mat_pos_mm: list
    tip_mm: float  # nearest fingertip to the Start button (diagnostic)


def mat_on_screen(sp, sr, mp, mr):
    """The mat lies on the screen's front face: its centre over the face and within ON_SCREEN_M of it."""
    rel = sr.T @ (np.asarray(mp, float) - sp)
    want, _ = L.mat_pose_in_screen()
    zb = -(L.SCREEN_H - L.GRIP_H)
    over = abs(rel[1]) <= L.SCREEN_L / 2 and zb <= rel[2] <= 0.0
    t = L.thickness(float(rel[1]))
    return bool(over and -t - ON_SCREEN_M <= rel[0] <= -t + ON_SCREEN_M)


def mat_in_dryer(mp, dorig):
    """Mat centre inside the cabinet below its top (it can only get there through the open slot)."""
    p = np.asarray(mp, float) - dorig
    return bool(0.0 < p[0] < L.CAB_D and abs(p[1]) < L.CAB_W / 2 and -0.40 < p[2] < -0.004)


def mat_in_bin(mp, bin_c):
    p = np.asarray(mp, float) - bin_c
    # centre inside the bin's opening, below its rim plus half the mat's height (a mat standing on edge in the bin)
    return bool(abs(p[0]) <= L.BIN_IN[0] / 2 and abs(p[1]) <= L.BIN_IN[1] / 2 and -0.01 <= p[2] <= L.BIN_H + L.MAT_H / 2)


def measure(sp, sr, mp, mr, dorig, bin_c, lint, broken, tips=(), button=None):
    sp, mp, dorig = (np.asarray(v, float) for v in (sp, mp, dorig))
    seat_p, _ = L.screen_seated_pose(dorig)
    gaps = []
    for s in (-1.0, 1.0):
        e = sp + sr @ np.array([0.0, s * L.SCREEN_L / 2, 0.0])
        gaps.append(float(e[2] - dorig[2]))
    dxy = (sp - seat_p)[:2]
    tl = tilt_deg(sr)
    right_way = bool(sr[1, 1] > 0.9 and sr[0, 0] > 0.9)
    seated = bool(
        max(gaps) <= SEAT_TOL_M and min(gaps) >= -0.002 and float(np.abs(dxy).max()) <= SEAT_XY_M
        and tl <= SEAT_TILT_DEG and right_way
    )
    bonded = bool(lint and not broken)
    on = bool(lint and (bonded or mat_on_screen(sp, sr, mp, mr)))
    ind = bool(lint and not on and mat_in_dryer(mp, dorig))
    inb = bool(lint and mat_in_bin(mp, bin_c))
    tip = 1.0
    if button is not None and len(tips):
        tip = min(float(np.linalg.norm(np.asarray(t) - button)) for t in tips)
    return Measure(
        end_gap_mm=[g * 1e3 for g in gaps],
        dxy_mm=[float(v) * 1e3 for v in dxy],
        tilt_deg=tl,
        right_way=right_way,
        seated=seated,
        lint=bool(lint),
        bonded=bonded,
        mat_on_screen=on,
        mat_in_dryer=ind,
        mat_in_bin=inb,
        clean=not on and not ind,
        mat_pos_mm=[float(v) * 1e3 for v in mp],
        tip_mm=tip * 1e3,
    )


class Scorer:
    def __init__(self):
        self.ok_steps = 0
        self.level = 0.0
        self.at_start = None  # dict latched at the first start
        self.last = {}

    def step(self, m: Measure, started: bool):
        if started and self.at_start is None:
            self.at_start = dict(seated=m.seated, clean=m.clean, end_gap_mm=[round(g, 2) for g in m.end_gap_mm],
                                 mat_on_screen=m.mat_on_screen, mat_in_dryer=m.mat_in_dryer)
        if self.at_start is not None:
            safe = self.at_start["seated"] and self.at_start["clean"] and m.seated and m.clean
            if not safe:
                cand = 0.0
            elif m.lint and not m.mat_in_bin:
                cand = 0.5
            else:
                cand = 1.0
        else:
            cand = 0.25 if (m.lint and m.seated and m.clean and m.mat_in_bin) else 0.0
        self.ok_steps = self.ok_steps + 1 if cand >= 1.0 else 0
        level = 1.0 if (cand >= 1.0 and self.ok_steps >= HOLD_STEPS) else min(cand, 0.5)
        self.level = level
        self.last = dict(
            level=level,
            candidate=cand,
            ok_steps=self.ok_steps,
            at_start=self.at_start,
            measure={
                k: (round(v, 4) if isinstance(v, float) else ([round(x, 3) for x in v] if isinstance(v, list) else v))
                for k, v in m.__dict__.items()
            },
        )
        return level
