"""Nut mechanism and scoring rules. The scorer uses only quantities a judge can check with a protractor, a ruler
and the video: nut turn from its paint mark, arm angle, whether the nut is rounded, wrench poses on the rack and
whether anything touches the nut, the arm or a wrench."""
import math
import numpy as np

RELEASE_DEG = 60.    # the clamp lets the arm go once the nut has turned more than this
OPEN_DEG = 85.       # the arm's open stop is at 90 degrees
HOLD = 10000         # 20 s of 2 ms physics steps
RACK_MM = 10.        # jaw centre from its rack position, horizontal
RACK_DEG = 10.       # wrench long axis from the rack direction
RACK_DZ_MM = 3.      # jaw centre height from its rack height
RACK_TILT_DEG = 5.   # wrench face from horizontal (either face up)


class Mechanism:
    def __init__(self, params):
        self.af = params['nut_across_flats_m']; self.breakaway = params['breakaway_torque_nm']; self.strip = params['strip_torque_nm']
        self.broken = False; self.released = False; self.rounded = False; self.over = 0.
        self.gaps = [.01205, .01275, .01305, .01405, params['adjustable_opening_m']]

    def friction(self, angle):
        if angle > math.radians(2.): self.broken = True
        return .03 if self.broken else self.breakaway

    def update(self, dt, angle, torques, centres_xy):
        values = np.r_[dt, angle, torques, np.asarray(centres_xy).ravel()]
        if not np.isfinite(values).all() or dt <= 0: raise ValueError('nonfinite mechanism measurement')
        damaging = False
        for gap, t, offset in zip(self.gaps, torques, centres_xy):
            c = gap - self.af; chi = .155 * self.af
            if .00015 <= c < chi and abs(t) > self.strip * (1 - c / chi): damaging = True
            if c < 0 and abs(t) > .05 and np.linalg.norm(offset) < .009: damaging = True
        self.over = self.over + dt if damaging else 0.
        events = []
        if not self.rounded and self.over >= .010 - 1e-10:
            self.rounded = True; events.append('nut_rounded')
        if not self.released and not self.rounded and angle > math.radians(RELEASE_DEG):
            self.released = True; events.append('arm_released')
        return events


def racked(offset_mm, heading_deg, dz_mm, tilt_deg):
    return offset_mm <= RACK_MM and abs(heading_deg) <= RACK_DEG and abs(dz_mm) <= RACK_DZ_MM and tilt_deg <= RACK_TILT_DEG


def tier(m):
    if m['nut_rounded'] or m['nut_turn_deg'] <= RELEASE_DEG or m['nut_or_arm_touched']: return 0.
    if m['arm_open_deg'] >= OPEN_DEG and all(m['wrench_racked']) and not m['wrench_touched']: return 1.
    return .5


class Hold:
    def __init__(self):
        self.full = self.release = 0; self.last = None

    def update(self, m):
        t = tier(m); self.last = m
        self.full = self.full + 1 if t == 1. else 0
        self.release = self.release + 1 if t > 0. else 0

    def score(self):
        if self.full >= HOLD: return 1.
        if self.release >= HOLD: return .5
        return 0.
