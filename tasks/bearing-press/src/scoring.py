"""Scorer for the bearing-press task (engine-independent, numpy only).

Per physics step the ``Monitor`` latches two failures from contact forces and poses:
  * jammed: while the ram pushes on the stack (ram contact force >= 20 N for 50 ms) with the bearing in the bore,
    the bearing is cocked more than 0.3 deg, or the press load is off-centre: the bore axis more than 2.0 mm from the
    ram axis, or the outer-ring driver more than 1.5 mm off the bearing's axis. A cocked or off-centre ring jams in a
    transition fit (here the fit model then locks it: the cap rises to 5 kN);
  * brinelled: force goes through the rolling elements: the 32 mm inner-ring disc or the ram itself pushes on the
    bearing with 20 N or more for 50 ms (mounting instructions forbid pressing through the inner ring when the outer
    ring is the one being fitted).
Levels (value at the end of the trial):
  0.25  the bearing is in the bore, square (<= 0.3 deg) and started at least 2 mm deep (its bottom 2 mm below the
        housing face) but not seated;
  0.5   seated (outer ring on the shoulder: bottom within 0.05 mm of it, tilt <= 0.15 deg) but a hand is still on the
        handle or within 30 mm of the handle bar or the work;
  1.0   seated, both hands released and 30 mm or more from the handle bar and the work, bearing still for 1 s (ends
        the trial);
  0     otherwise, or after either failure.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import layout as L
import numpy as np

RAM_PUSH_N = 20.0
HOLD_S = 0.050
TILT_JAM_DEG = 0.3
ECC_RAM_M = 0.0020
ECC_DRIVER_M = 0.0015
IN_BORE_XY_M = 0.0008
INNER_N = 20.0
START_DEPTH_M = 0.002
SEAT_DZ_M = 0.00005
SEAT_TILT_DEG = 0.15
CLEAR_M = 0.030
STILL_V_M_S = 0.002
STILL_W_DEG_S = 2.0
HOLD_STEPS = 20


def tilt_deg(rot):
    return math.degrees(math.acos(float(np.clip(rot[2, 2], -1.0, 1.0))))


def bearing_geometry(bearing, housing):
    """(xy offset of the bearing axis from the bore axis (m), bottom depth below the housing face (m),
    bottom height above the shoulder (m), tilt of the bearing relative to the housing (deg))."""
    bp, br = bearing
    hp, hr = housing
    rel = hr.T @ (bp - hp)  # bearing frame origin = bottom centre, in the housing frame
    off = float(np.hypot(rel[0], rel[1]))
    depth = L.HOUSING_H - float(rel[2])
    above_seat = float(rel[2]) - (L.HOUSING_H - L.SEAT_DEPTH)
    tilt = math.degrees(math.acos(float(np.clip((hr.T @ br)[2, 2], -1.0, 1.0))))
    return off, depth, above_seat, tilt


def in_bore(off, depth):
    return off < IN_BORE_XY_M + 0.0003 and depth > 0.0


def point_segment(p, a, b):
    p, a, b = (np.asarray(x, float) for x in (p, a, b))
    ab = b - a
    t = float(np.clip((p - a) @ ab / (ab @ ab), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def work_distance(p, housing, top_z):
    """Distance from a point to the work stack (a cylinder of the housing's radius from the table to top_z)."""
    hp, _ = housing
    p = np.asarray(p, float)
    dr = max(0.0, float(np.hypot(*(p - hp)[:2])) - L.HOUSING_R)
    dz = max(0.0, hp[2] - p[2], p[2] - top_z)
    return math.hypot(dr, dz)


@dataclass
class Measure:
    off_mm: float
    depth_mm: float
    above_seat_mm: float
    tilt_deg: float
    in_bore: bool
    started: bool
    seated: bool
    centre_mm: float  # bore axis from the ram axis
    handle_mm: float  # nearest fingertip to the handle bar
    work_mm: float  # nearest fingertip to the work stack
    theta_deg: float
    driver_off_mm: float  # outer-ring driver axis from the bearing axis (mm) while it sits on the bearing, else -1


def measure(poses, theta, tips):
    off, depth, above, tilt = bearing_geometry(poses["bearing"], poses["housing"])
    ib = in_bore(off, depth)
    started = ib and depth >= START_DEPTH_M and tilt <= TILT_JAM_DEG
    seated = ib and abs(above) <= SEAT_DZ_M and tilt <= SEAT_TILT_DEG
    hp = poses["housing"][0]
    centre = float(np.hypot(*(hp[:2] - L.RAM_XY)))
    a, b = L.handle_point(theta, 0.02), L.handle_point(theta, L.HANDLE_LEN + 0.026)
    top = max(poses["disc_out"][0][2] + L.DISC_T, hp[2] + L.HOUSING_H + 0.008)
    hd = min(point_segment(t, a, b) - L.HANDLE_R for t in tips) if tips else 1.0
    wd = min(work_distance(t, poses["housing"], top) for t in tips) if tips else 1.0
    dp, bp_ = poses["disc_out"][0], poses["bearing"][0]
    on_brg = abs(dp[2] - (bp_[2] + L.BEARING_B)) < 0.003
    drv = float(np.hypot(*(dp[:2] - bp_[:2]))) * 1e3 if on_brg else -1.0
    return Measure(
        driver_off_mm=drv,
        off_mm=off * 1e3,
        depth_mm=depth * 1e3,
        above_seat_mm=above * 1e3,
        tilt_deg=tilt,
        in_bore=ib,
        started=started,
        seated=seated,
        centre_mm=centre * 1e3,
        handle_mm=hd * 1e3,
        work_mm=wd * 1e3,
        theta_deg=math.degrees(theta),
    )


@dataclass
class Monitor:
    """Per-physics-step failure monitor (latched)."""

    jammed: bool = False
    brinelled: bool = False
    jam_reason: str = ""
    jam_s: float = 0.0
    inner_s: float = 0.0
    peak: dict = field(default_factory=lambda: dict(ram_n=0.0, inner_n=0.0, tilt_pushed_deg=0.0, ecc_pushed_mm=0.0,
                                                     driver_ecc_pushed_mm=0.0))

    def step(self, dt, m):
        """m: dict(ram_load N, inner_load N, bearing (pos, rot), housing (pos, rot), disc_out (pos, rot), depth m).
        Returns True once jammed."""
        self.peak["ram_n"] = max(self.peak["ram_n"], m["ram_load"])
        self.peak["inner_n"] = max(self.peak["inner_n"], m["inner_load"])
        off, depth, _, tilt = bearing_geometry(m["bearing"], m["housing"])
        pushing = m["ram_load"] >= RAM_PUSH_N and in_bore(off, depth)
        reason = ""
        if pushing:
            ecc = float(np.hypot(*(m["housing"][0][:2] - L.RAM_XY)))
            dpos = m["disc_out"][0]
            drv = float(np.hypot(*(dpos[:2] - m["bearing"][0][:2])))
            on_brg = abs(dpos[2] - (m["bearing"][0][2] + L.BEARING_B)) < 0.003
            self.peak["tilt_pushed_deg"] = max(self.peak["tilt_pushed_deg"], tilt)
            self.peak["ecc_pushed_mm"] = max(self.peak["ecc_pushed_mm"], ecc * 1e3)
            if on_brg:
                self.peak["driver_ecc_pushed_mm"] = max(self.peak["driver_ecc_pushed_mm"], drv * 1e3)
            if tilt > TILT_JAM_DEG:
                reason = f"cocked {tilt:.2f} deg"
            elif ecc > ECC_RAM_M:
                reason = f"off-centre under the ram {ecc * 1e3:.1f} mm"
            elif on_brg and drv > ECC_DRIVER_M:
                reason = f"driver off-centre {drv * 1e3:.1f} mm"
        self.jam_s = self.jam_s + dt if reason else 0.0
        if self.jam_s >= HOLD_S - 1e-9 and not self.jammed:
            self.jammed, self.jam_reason = True, reason
        self.inner_s = self.inner_s + dt if m["inner_load"] >= INNER_N else 0.0
        if self.inner_s >= HOLD_S - 1e-9:
            self.brinelled = True
        return self.jammed

    @property
    def veto(self):
        return self.jammed or self.brinelled


class Scorer:
    def __init__(self):
        self.still = 0
        self.level = 0.0
        self.last = {}

    def step(self, m: Measure, lin_v, ang_v_deg, monitor: Monitor):
        still = lin_v < STILL_V_M_S and ang_v_deg < STILL_W_DEG_S
        clear = m.handle_mm >= CLEAR_M * 1e3 and m.work_mm >= CLEAR_M * 1e3
        ok_full = m.seated and clear and still
        self.still = self.still + 1 if ok_full else 0
        if monitor.veto:
            level = 0.0
        elif ok_full and self.still >= HOLD_STEPS:
            level = 1.0
        elif m.seated:
            level = 0.5
        elif m.started:
            level = 0.25
        else:
            level = 0.0
        self.level = level
        self.last = dict(
            level=level,
            still_steps=self.still,
            veto=dict(jammed=monitor.jammed, jam_reason=monitor.jam_reason, brinelled=monitor.brinelled),
            measure={k: (round(v, 4) if isinstance(v, float) else v) for k, v in m.__dict__.items()},
            lin_v_mm_s=round(lin_v * 1e3, 3),
            ang_v_deg_s=round(ang_v_deg, 3),
            monitor_peak={k: round(v, 3) for k, v in monitor.peak.items()},
        )
        return level
