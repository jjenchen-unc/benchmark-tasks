"""Scorer for the h4-bulb-clip task (engine-independent, numpy only).

Inputs every control step: the bulb's pose and velocity, the bail (clip) angle, the catch slide position and the four
finger pad tips; plus two latched vetoes from the per-physics-step monitor (``Monitor``): the glass was touched by the
robot (any contact force between a robot link and the glass envelope, a separate collider body), or the bulb was
dropped (it moved down faster than 0.3 m/s, the speed after a 4.6 mm free fall, with no robot contact).

Levels (score at the end of the trial):
  0     any veto; otherwise bulb not seated (tabs not in their notches, flange cocked or proud, bulb out).
  0.25  seated: the flange's front face within 0.4 mm of the boss seat, tilt < 1 deg, roll within 4 deg of the fitted
        roll (only possible with all three tabs down in their notches); clip not over it.
  0.5   seated, the bail swung over the flange (within 6 deg of closed) but not hooked under its catch.
  1.0   seated, bail hooked under the catch (bail within 1.2 deg of closed and the catch back at rest over it), every
        fingertip 30 mm or more from the bulb and the bail, bulb still for 1 s. Ends the trial.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import layout as L
import numpy as np

SEAT_DX_M = 0.0004
SEAT_TILT_DEG = 1.0
SEAT_ROLL_DEG = 4.0
OVER_DEG = 6.0
LATCH_DEG = 1.2
CATCH_REST_TOL_M = 0.0006
CLEAR_M = 0.030
STILL_V_M_S = 0.003
STILL_W_DEG_S = 3.0
HOLD_STEPS = 20  # 1 s at 20 Hz
GLASS_TOUCH_N = 0.05
DROP_V_M_S = 0.30
ROBOT_TOUCH_N = 0.05


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def bulb_in_h(pos, rot):
    """Bulb pose relative to the seat frame H: (axial m, radial m, tilt deg, roll deg)."""
    d = np.asarray(pos, float) - L.H
    xb = rot[:, 0]
    tilt = math.degrees(math.acos(float(np.clip(xb[0], -1.0, 1.0))))
    # roll: angle of y_B about the bulb axis, measured in H's y-z plane
    yb = rot[:, 1]
    roll = math.degrees(math.atan2(yb[2], yb[1]))
    return float(d[0]), float(math.hypot(d[1], d[2])), tilt, roll


def seg_dist(p, a, b):
    p, a, b = (np.asarray(v, float) for v in (p, a, b))
    ab = b - a
    t = float(np.clip((p - a) @ ab / (ab @ ab), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def bulb_distance(point, pos, rot):
    """Distance from a point to the bulb, as a capsule of radius TAB_R around its axis from blade end to glass tip."""
    a = pos + rot @ np.array([L.BLADE_X[0], 0.0, 0.0])
    b = pos + rot @ np.array([L.GLASS_TIP, 0.0, 0.0])
    return max(seg_dist(point, a, b) - L.TAB_R, 0.0)


def clip_distance(point, phi_deg):
    """Distance from a point (station frame) to the bail wire (legs and bight)."""
    hp = L.H + L.hinge_point()
    r = L.clip_rot(phi_deg)
    best = 9.0
    for a, b in (
        ((0, 0, L.CLIP_HALF_W), (0, -L.CLIP_LEN, L.CLIP_HALF_W)),
        ((0, 0, -L.CLIP_HALF_W), (0, -L.CLIP_LEN, -L.CLIP_HALF_W)),
        ((0, -L.CLIP_LEN, -L.CLIP_HALF_W), (0, -L.CLIP_LEN, L.CLIP_HALF_W)),
    ):
        best = min(best, seg_dist(point, hp + r @ np.array(a), hp + r @ np.array(b)) - L.WIRE_R)
    return max(best, 0.0)


@dataclass
class Measure:
    axial_mm: float  # flange front face ahead (+) / behind (-) the seat face, on the bore axis
    radial_mm: float
    tilt_deg: float
    roll_deg: float  # roll from the fitted orientation
    clip_deg: float  # bail angle, 0 = closed
    catch_mm: float  # catch slide from rest (negative = cammed away)
    tip_bulb_mm: float  # nearest fingertip to the bulb
    tip_clip_mm: float  # nearest fingertip to the bail
    seated: bool
    over: bool
    latched: bool


def measure(pos, rot, clip_deg, catch_x, tips):
    ax, rad, tilt, roll = bulb_in_h(pos, rot)
    seated = (
        abs(ax) <= SEAT_DX_M
        and tilt <= SEAT_TILT_DEG
        and abs(wrap180(roll)) <= SEAT_ROLL_DEG
    )
    over = clip_deg <= OVER_DEG
    latched = clip_deg <= LATCH_DEG and abs(catch_x) <= CATCH_REST_TOL_M
    tb = min(bulb_distance(t, pos, rot) for t in tips) if tips else 1.0
    tc = min(clip_distance(t, clip_deg) for t in tips) if tips else 1.0
    return Measure(
        axial_mm=ax * 1e3,
        radial_mm=rad * 1e3,
        tilt_deg=tilt,
        roll_deg=wrap180(roll),
        clip_deg=clip_deg,
        catch_mm=catch_x * 1e3,
        tip_bulb_mm=tb * 1e3,
        tip_clip_mm=tc * 1e3,
        seated=seated,
        over=over,
        latched=latched,
    )


@dataclass
class Monitor:
    """Per-physics-step veto monitor (latched)."""

    glass: bool = False
    dropped: bool = False
    peak: dict = field(
        default_factory=lambda: dict(glass_n=0.0, fall_v=0.0, robot_on_bulb_n=0.0)
    )

    def step(self, dt, glass_n, bulb_vz, robot_on_bulb_n):
        """glass_n: |force| between robot links and the glass envelope; bulb_vz: the bulb's vertical velocity (m/s,
        up +); robot_on_bulb_n: |force| from robot links on the bulb (cap or glass)."""
        self.peak["glass_n"] = max(self.peak["glass_n"], glass_n)
        self.peak["robot_on_bulb_n"] = max(self.peak["robot_on_bulb_n"], robot_on_bulb_n)
        if glass_n > GLASS_TOUCH_N:
            self.glass = True
        falling = -bulb_vz if robot_on_bulb_n <= ROBOT_TOUCH_N else 0.0
        self.peak["fall_v"] = max(self.peak["fall_v"], falling)
        if falling > DROP_V_M_S:
            self.dropped = True

    @property
    def veto(self):
        return self.glass or self.dropped


class Scorer:
    def __init__(self):
        self.still = 0
        self.level = 0.0
        self.last = {}

    def step(self, m: Measure, lin_v, ang_v_deg, monitor: Monitor):
        still = lin_v < STILL_V_M_S and ang_v_deg < STILL_W_DEG_S
        clear = m.tip_bulb_mm >= CLEAR_M * 1e3 and m.tip_clip_mm >= CLEAR_M * 1e3
        ok_full = m.seated and m.latched and clear and still
        self.still = self.still + 1 if ok_full else 0
        if monitor.veto or not m.seated:
            level = 0.0
        elif ok_full and self.still >= HOLD_STEPS:
            level = 1.0
        elif m.latched or m.over:
            level = 0.5
        else:
            level = 0.25
        self.level = level
        self.last = dict(
            level=level,
            still_steps=self.still,
            veto=dict(glass=monitor.glass, dropped=monitor.dropped),
            measure={
                k: (round(v, 4) if isinstance(v, float) else v) for k, v in m.__dict__.items()
            },
            lin_v_mm_s=round(lin_v * 1e3, 3),
            ang_v_deg_s=round(ang_v_deg, 3),
            peak={k: round(v, 4) for k, v in monitor.peak.items()},
        )
        return level
