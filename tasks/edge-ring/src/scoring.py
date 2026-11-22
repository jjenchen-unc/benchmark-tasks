"""Scorer for the edge-ring task (engine-independent, numpy only).

Inputs every control step: the edge ring's pose and velocity, the lift-pin carrier height, the four finger pad tips,
and three latched vetoes from the per-physics-step monitor (``Monitor``): the chuck was touched, the ring was dropped
(it moved down faster than 0.2 m/s, the speed of a 2 mm free fall, with no robot contact), or the ring was pressed onto
the chamber (the robot touching it while the chamber parts push back with more than its weight + 10 N for 50 ms), or
the ring rested on the locating dowels' tips (the base ring, whose only parts above its top face are the dowels,
carries half the ring's weight or more for 0.5 s while the ring's bottom is more than 0.8 mm above the seat).

Levels (score at the end of the trial):
  0.5  seated (below) but a hand still inside the chamber bowl or within 30 mm of the ring.
  1.0  seated: ring bottom within 0.3 mm of the seat, tilt < 0.15 deg, lift pins down (so all three dowels are through
       its slots); every fingertip 30 mm or more from the ring and out of the bowl; ring still for 1 s. Ends the trial.
  0    otherwise (ring on the stand, on the lift pins, on the dowel tips), or any veto.
A ring left on the raised lift pins scores 0 whatever its placement: whether it would seat is only known once the pins
lower it, and a person checks the same outcome (seated or not). ``on_lift`` / ``clocked`` are reported for diagnosis
only.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import layout as L
import numpy as np

LIFT_DZ_M = 0.0015  # ring bottom within 1.5 mm of the raised pin tips
LIFT_TILT_DEG = 1.0
SLOT_ACROSS_M = 0.00035  # pin within 0.35 mm of its slot centre: the ring then seats when the pins lower it
SLOT_ALONG_M = 0.00035
SEAT_DZ_M = 0.0003
SEAT_TILT_DEG = 0.15
LIFT_UP_TOL_M = 0.0005
LIFT_DOWN_TOL_M = 0.0005
CLEAR_M = 0.030
STILL_V_M_S = 0.002
STILL_W_DEG_S = 2.0
HOLD_STEPS = 20  # 1 s at 20 Hz
# monitor thresholds
CHUCK_TOUCH_N = 0.05
DROP_V_M_S = 0.198  # downward speed after a 2.0 mm free fall: sqrt(2 g h)
FORCE_EXTRA_N = 10.0
FORCE_HOLD_S = 0.050
ROBOT_TOUCH_N = 0.05
TIPS_DZ_M = 0.0008  # ring bottom above the seat: above this only the dowels' tips / lead-ins can carry it
TIPS_SUPPORT_FRAC = 0.5  # base ring (dowels) carrying half the ring's weight or more
TIPS_HOLD_S = 0.5  # for 0.5 s (a lead-in slide while the pins lower takes < 0.1 s)


def tilt_deg(rot):
    return math.degrees(math.acos(float(np.clip(rot[2, 2], -1.0, 1.0))))


def yaw_deg(rot):
    return math.degrees(math.atan2(rot[1, 0], rot[0, 0]))


def pin_offsets(pos, rot):
    """Per locating pin: (across, along) offset in metres of the pin axis from its slot centre, in the slot's frame
    (ring frame, projected on the ring plane)."""
    out = []
    for a, pc in zip(L.LOC_ANGLES, L.loc_pin_centers()):
        pin_w = L.C + np.array([pc[0], pc[1], 0.0])
        rel = rot.T @ (pin_w - pos)
        ar = math.radians(a)
        t = np.array([-math.sin(ar), math.cos(ar)])
        r = np.array([math.cos(ar), math.sin(ar)])
        out.append((float(rel[:2] @ t), float(rel[:2] @ r - L.LOC_PITCH_R)))
    return out


def ring_distance(point, pos, rot):
    """Distance (m) from a point to the ring's annular solid."""
    p = rot.T @ (np.asarray(point, float) - pos)
    rr = math.hypot(p[0], p[1])
    dr = max(L.RING_R[0] - rr, 0.0, rr - L.RING_R[1])
    dz = max(-p[2], 0.0, p[2] - L.RING_T)
    return math.hypot(dr, dz)


def in_bowl(point):
    p = np.asarray(point, float) - L.C
    return math.hypot(p[0], p[1]) < L.BOWL_R[1] and p[2] < L.BOWL_TOP


@dataclass
class Measure:
    dz_mm: float  # ring bottom centre height above the seat (mm)
    tilt_deg: float
    clock_deg: float  # ring yaw in the chamber frame (0 = fitted)
    centre_mm: float  # ring axis offset from the cathode axis (mm)
    across_mm: list  # per pin
    along_mm: list
    lift_mm: float  # lift-pin tip height above the seat (mm)
    tip_mm: float  # nearest fingertip to the ring (mm)
    in_bowl: bool
    on_lift: bool
    clocked: bool
    seated: bool


def measure(pos, rot, lift_z, tips):
    off = pin_offsets(pos, rot)
    dz = float((pos - L.C)[2])
    tl = tilt_deg(rot)
    across = [o[0] for o in off]
    along = [o[1] for o in off]
    clocked = all(abs(a) <= SLOT_ACROSS_M for a in across) and all(
        abs(b) <= SLOT_ALONG_M for b in along
    )
    on_lift = (
        lift_z >= L.LIFT_UP - LIFT_UP_TOL_M
        and abs(dz - lift_z) <= LIFT_DZ_M
        and tl <= LIFT_TILT_DEG
    )
    seated = (
        abs(dz) <= SEAT_DZ_M
        and tl <= SEAT_TILT_DEG
        and lift_z <= L.LIFT_DOWN + LIFT_DOWN_TOL_M
    )
    tip = min(ring_distance(t, pos, rot) for t in tips) if tips else 1.0
    return Measure(
        dz_mm=dz * 1e3,
        tilt_deg=tl,
        clock_deg=yaw_deg(rot),
        centre_mm=float(np.hypot(*(pos - L.C)[:2])) * 1e3,
        across_mm=[a * 1e3 for a in across],
        along_mm=[b * 1e3 for b in along],
        lift_mm=lift_z * 1e3,
        tip_mm=tip * 1e3,
        in_bowl=any(in_bowl(t) for t in tips),
        on_lift=on_lift,
        clocked=clocked,
        seated=seated,
    )


@dataclass
class Monitor:
    """Per-physics-step veto monitor (latched)."""

    chuck: bool = False
    dropped: bool = False
    forced: bool = False
    on_tips: bool = False
    force_s: float = 0.0
    tips_s: float = 0.0
    peak: dict = field(
        default_factory=lambda: dict(
            chuck_n=0.0, fall_v=0.0, support_n=0.0, press_s=0.0
        )
    )

    def step(
        self,
        dt,
        chuck_n,
        ring_vz,
        support_n,
        robot_on_ring_n,
        weight_n=L.RING_MASS * 9.81,
        base_n=0.0,
        ring_dz=0.0,
    ):
        """chuck_n: |force| on the chuck from anything; ring_vz: the ring's vertical velocity (m/s, up +);
        support_n: |force| on the ring from the chamber parts and the stand; robot_on_ring_n: |force| from robot links;
        base_n: |force| on the ring from the base ring with its dowels; ring_dz: ring bottom above the seat (m)."""
        self.peak["chuck_n"] = max(self.peak["chuck_n"], chuck_n)
        if chuck_n > CHUCK_TOUCH_N:
            self.chuck = True
        falling = -ring_vz if robot_on_ring_n <= ROBOT_TOUCH_N else 0.0
        self.peak["fall_v"] = max(self.peak.get("fall_v", 0.0), falling)
        if falling > DROP_V_M_S:
            self.dropped = True
        pressing = (
            robot_on_ring_n > ROBOT_TOUCH_N and support_n > weight_n + FORCE_EXTRA_N
        )
        self.force_s = self.force_s + dt if pressing else 0.0
        self.peak["support_n"] = max(
            self.peak["support_n"],
            support_n if robot_on_ring_n > ROBOT_TOUCH_N else 0.0,
        )
        self.peak["press_s"] = max(self.peak["press_s"], self.force_s)
        if self.force_s >= FORCE_HOLD_S - 1e-9:
            self.forced = True
        on_tips = ring_dz > TIPS_DZ_M and base_n >= TIPS_SUPPORT_FRAC * weight_n
        self.tips_s = self.tips_s + dt if on_tips else 0.0
        self.peak["tips_s"] = max(self.peak.get("tips_s", 0.0), self.tips_s)
        if self.tips_s >= TIPS_HOLD_S - 1e-9:
            self.on_tips = True

    @property
    def veto(self):
        return self.chuck or self.dropped or self.forced or self.on_tips


class Scorer:
    def __init__(self):
        self.still = 0
        self.level = 0.0
        self.last = {}

    def step(self, m: Measure, lin_v, ang_v_deg, monitor: Monitor):
        still = lin_v < STILL_V_M_S and ang_v_deg < STILL_W_DEG_S
        clear_full = m.tip_mm >= CLEAR_M * 1e3 and not m.in_bowl
        ok_full = m.seated and clear_full and still
        self.still = self.still + 1 if ok_full else 0
        if monitor.veto:
            level = 0.0
        elif ok_full and self.still >= HOLD_STEPS:
            level = 1.0
        elif m.seated:
            level = 0.5
        else:
            level = 0.0
        self.level = level
        self.last = dict(
            level=level,
            still_steps=self.still,
            veto=dict(
                chuck=monitor.chuck,
                dropped=monitor.dropped,
                forced=monitor.forced,
                on_tips=monitor.on_tips,
            ),
            measure={
                k: (
                    round(v, 4)
                    if isinstance(v, float)
                    else ([round(x, 4) for x in v] if isinstance(v, list) else v)
                )
                for k, v in m.__dict__.items()
            },
            lin_v_mm_s=round(lin_v * 1e3, 3),
            ang_v_deg_s=round(ang_v_deg, 3),
            peak=dict(monitor.peak),
        )
        return level
