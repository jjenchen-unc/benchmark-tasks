"""Scorer for the snaptoggle-anchor task (engine-independent, numpy only).

Inputs every control step: the board face position and hole centre, the channel pose, the cap pose and its position q
along the straps (held by the ratchet), the tail (strap remainder) pose and whether the straps have snapped off, the
four finger pad tips, and one latched veto from the per-physics-step monitor: the board face was crushed (the robot
pushed on the board with more than 40 N for 0.1 s).

Levels (score at the end of the trial):
  1.0  set and finished: channel through the hole and across it on the back face (bearing: both ends within 1.0 mm
       of the back face), cap seated (flange no more than 1.5 mm proud of the face) on a snug anchor (the ratchet
       holds the cap within 1.0 mm of the seated position, so the channel cannot move more than 1.0 mm off the back
       face), straps snapped off at the cap (stub no more than 2 mm proud) and the strap tail clear of the hole
       (30 mm or more), every fingertip 100 mm or more from the cap, anchor still for 1 s (channel and cap moved less
       than 1 mm over the last second). Ends the trial.
  0.6  set as above but the straps not snapped off (or the tail left at the hole).
  0.3  channel through and across the hole within 3 mm of the back face, but the anchor not set (cap not seated or
       loose: more than 1.0 mm of play).
  0    otherwise (channel not through the hole, folded / in line with the hole, dangling in the cavity), or the veto.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import layout as L
import numpy as np

BEAR_GAP_M = 0.0010  # both channel ends within 1.0 mm of the back face
NEAR_GAP_M = 0.0030  # 0.3 level: within 3 mm
ACROSS_DEG = 15.0  # channel axis within 15 deg of the board plane
SPAN_MARGIN_M = 0.003  # the channel covers the hole with 3 mm to spare at each side (along its length)
PROUD_M = 0.0015  # cap flange at most 1.5 mm proud
SNUG_M = 0.0010  # ratchet holds the cap within 1.0 mm of the snug seated position
STUB_M = 0.0020
TAIL_CLEAR_M = 0.030
TIP_CLEAR_M = 0.100  # hands out of the way (also clears the overhead camera's view of the result)
STILL_V_M_S = 0.001  # mean speed over the last 1 s (channel and cap)
HOLD_STEPS = 20  # 1 s at 20 Hz
CRUSH_N = L.BOARD_CRUSH_N
CRUSH_S = L.BOARD_CRUSH_S


@dataclass
class Measure:
    through: bool  # channel centre behind the back face
    across: bool  # channel across the hole (axis in the board plane, covering the hole)
    gap_mm: list  # bearing-face gap to the back face at both channel ends (mm, + = off the board)
    axis_deg: float  # channel axis angle out of the board plane
    span_mm: float  # hole centre offset along the channel from its centre (mm)
    proud_mm: float  # cap flange inner face proud of the board face (mm)
    cap_radial_mm: float  # cap axis offset from the hole axis (mm)
    play_mm: float  # q - Q_SEAT: how far the ratchet lets the channel move off the back face (mm)
    stub_mm: float  # strap stub proud of the cap face (mm)
    snapped: bool
    tail_mm: float  # nearest point of the strap tail to the hole centre (mm)
    tip_mm: float  # nearest fingertip to the cap (mm)
    bearing: bool
    near: bool
    seated: bool
    snug: bool
    off: bool


def channel_gaps(xb, ch_pose):
    """Gap (m) of the channel's bearing face to the back face plane x = xb at both channel ends."""
    p, r = ch_pose
    out = []
    for s in (-1.0, 1.0):
        pt = p + r @ np.array([s * L.CH_L / 2, 0.0, -L.CH_H / 2])
        out.append(float(xb - pt[0]) * -1.0)
    return out


def measure(xf, hole, ch_pose, cap_pose, q, tail_pose, snapped, tips):
    xb = xf + L.BOARD_T
    hole = np.asarray(hole, float)
    cp, cr = ch_pose
    axis = cr[:, 0]
    axis_deg = math.degrees(math.asin(min(1.0, abs(float(axis[0])))))
    rel = cp - hole
    s_along = float(np.dot(rel[1:], axis[1:]) / max(1e-9, np.linalg.norm(axis[1:])))
    gaps = channel_gaps(xb, ch_pose)
    through = float(cp[0]) > xb
    covers = abs(s_along) <= L.CH_L / 2 - L.HOLE_R - SPAN_MARGIN_M
    lateral = float(np.linalg.norm(rel[1:] - s_along * axis[1:] / max(1e-9, np.linalg.norm(axis[1:]))))
    across = through and axis_deg <= ACROSS_DEG and covers and lateral <= L.CH_W / 2 + 0.001
    bearing = across and max(gaps) <= BEAR_GAP_M and min(gaps) >= -0.0005
    near = across and max(gaps) <= NEAR_GAP_M
    kp, kr = cap_pose
    inner = kp + kr @ np.array([-L.CAP_FLANGE_T, 0.0, 0.0])
    proud = float(xf - inner[0])
    cap_radial = float(np.linalg.norm((kp - hole)[1:]))
    play = float(q - L.Q_SEAT)
    stub = float(L.X_BRK - q)
    seated = proud <= PROUD_M and proud >= -0.0005 and cap_radial <= 0.0015
    snug = play <= SNUG_M
    tp, tr = tail_pose
    tail_pts = [tp + tr @ np.array([x, 0.0, 0.0]) for x in np.linspace(0.0, L.STRAP_LEN - L.X_BRK, 9)]
    tail_d = min(float(np.linalg.norm(t - hole)) for t in tail_pts)
    off = snapped and stub <= STUB_M and tail_d >= TAIL_CLEAR_M
    tip = min(float(np.linalg.norm(np.asarray(t) - kp)) for t in tips) if tips else 1.0
    return Measure(
        through=through,
        across=across,
        gap_mm=[g * 1e3 for g in gaps],
        axis_deg=axis_deg,
        span_mm=s_along * 1e3,
        proud_mm=proud * 1e3,
        cap_radial_mm=cap_radial * 1e3,
        play_mm=play * 1e3,
        stub_mm=stub * 1e3,
        snapped=bool(snapped),
        tail_mm=tail_d * 1e3,
        tip_mm=tip * 1e3,
        bearing=bearing,
        near=near,
        seated=seated,
        snug=snug,
        off=off,
    )


def cap_at_break_line(q):
    """The cap is seated (flange within PROUD_M of the snug seated position) and so its face is at the break line:
    only then does bending snap the straps there (the cap edge is the fulcrum)."""
    return abs(L.X_BRK - q) <= L.BREAK_CAP_M and (q - L.Q_SEAT) <= PROUD_M


def bend_moment(arms, brk, axis):
    """Bending moment (N m) about the break line from the robot's contact on the strap tail.
    arms: list of (finger_forces, other_forces, tips) per arm: the contact forces (3,) on the tail from that arm's two
    fingers, the sum of the forces from its other links, and its two pad-tip points. Per arm: the lateral part
    (perpendicular to the straps) of its net force (a pure pinch cancels in the sum; pinching the tab and rocking it,
    the maker's snap-off method, does not) x the distance along the straps from the break line to its nearest
    fingertip (finger pose, not contact points). Returns the largest per-arm moment."""
    axis = np.asarray(axis, float)
    best = 0.0
    for fingers, other, tips in arms:
        f = sum(np.asarray(x, float) for x in fingers) + np.asarray(other, float)
        if not np.any(f):
            continue
        lat = f - np.dot(f, axis) * axis
        lever = min(abs(float(np.dot(np.asarray(t, float) - brk, axis))) for t in tips)
        best = max(best, float(np.linalg.norm(lat)) * lever)
    return best


@dataclass
class Monitor:
    """Per-physics-step veto monitor (latched): the robot pushing on the board hard enough to crush its face."""

    crushed: bool = False
    push_s: float = 0.0
    peak: dict = field(default_factory=lambda: dict(robot_on_wall_n=0.0, push_s=0.0))

    def step(self, dt, robot_on_wall_n):
        self.peak["robot_on_wall_n"] = max(self.peak["robot_on_wall_n"], robot_on_wall_n)
        self.push_s = self.push_s + dt if robot_on_wall_n > CRUSH_N else 0.0
        self.peak["push_s"] = max(self.peak["push_s"], self.push_s)
        if self.push_s >= CRUSH_S - 1e-9:
            self.crushed = True

    @property
    def veto(self):
        return self.crushed


def level_of(m: Measure) -> float:
    """Level of a state, without the 1 s hold and the fingertip clearance (those are in Scorer)."""
    if m.bearing and m.seated and m.snug:
        return 1.0 if m.off else 0.6
    if m.near:
        return 0.3
    return 0.0


class Scorer:
    def __init__(self):
        self.still = 0
        self.level = 0.0
        self.last = {}

    def step(self, m: Measure, speed, monitor: Monitor):
        base = level_of(m)
        ok_full = base == 1.0 and m.tip_mm >= TIP_CLEAR_M * 1e3 and speed < STILL_V_M_S
        self.still = self.still + 1 if ok_full else 0
        if monitor.veto:
            level = 0.0
        elif base == 1.0:
            level = 1.0 if self.still >= HOLD_STEPS else 0.6
        else:
            level = base
        self.level = level
        self.last = dict(
            level=level,
            still_steps=self.still,
            veto=dict(crushed=monitor.crushed),
            measure={
                k: (round(v, 4) if isinstance(v, float) else ([round(x, 4) for x in v] if isinstance(v, list) else v))
                for k, v in m.__dict__.items()
            },
            speed_mm_s=round(speed * 1e3, 3),
            peak=dict(monitor.peak),
        )
        return level
