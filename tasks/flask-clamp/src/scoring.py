"""Seat rule, shake test and scorer of the flask-clamp task (engine-independent, numpy only).

Seated (all of these, measured every control step from the flask, clamp and finger poses and the finger-flask
contacts):
* the flask's bottom is down on the clamp's base plate: bottom centre within 1.5 mm of the seat height, axis within
  2 deg of the clamp axis, bottom centre within 3 mm of the clamp axis;
* every one of the five fingers is in contact with the flask (contact force reported for that finger's collider) and
  sits at a hinge angle between 1.4 and 14 deg outward, i.e. its pad rests on the flask's conical body, not spread
  over the heel.

Released: every robot fingertip at least 40 mm from the flask's surface and from the free clamp's axis region.

Shake test (a person switches the shaker on for a few seconds): once the flask has been seated and released and still
for 0.5 s, the runtime runs the platform on its 19 mm orbit at 200 rpm for 2.5 s (0.5 s ramp up, 1.5 s, 0.5 s ramp
down). A hand coming back within 40 mm stops the test (it starts again when the state is valid again).

Score (value at the end of the trial, ``Scorer.step`` every control step):
  1.0   the shake test finished with the flask still seated at every step of it (ends the trial);
  0.5   the flask is seated in the free clamp but not released (a hand on or near it), or seated and released while
        the shake test is still running;
  0     otherwise: not seated (standing on the finger tips, tilted, beside the clamp, held in the air, on the bench),
        or knocked out of the clamp by the shake.
"""

from __future__ import annotations

from dataclasses import dataclass

import layout as L

SEAT_DZ_TOL = 0.0015
SEAT_RADIAL_TOL = 0.003
SEAT_TILT_DEG = 2.0
THETA_MIN = 0.024  # 1.4 deg
THETA_MAX = 0.244  # 14 deg
FINGER_CONTACT_N = 0.05
CLEAR_M = 0.040
STILL_V = 0.005  # m/s
STILL_STEPS = 10  # 0.5 s at 20 Hz before the shake starts
CONTROL_HZ = 20


def shake_steps():
    return round((2 * L.SHAKE_RAMP_S + L.SHAKE_HOLD_S) * CONTROL_HZ)


def shake_amplitude(t):
    """Orbit radius scale (0..1) at time t (s) since the shake started (ramp, hold, ramp)."""
    r, h = L.SHAKE_RAMP_S, L.SHAKE_HOLD_S
    if t <= 0:
        return 0.0
    if t < r:
        return t / r
    if t < r + h:
        return 1.0
    if t < 2 * r + h:
        return (2 * r + h - t) / r
    return 0.0


def seated_detail(rel, thetas, contacts):
    """rel: layout.flask_in_clamp(...); thetas: 5 hinge angles (rad); contacts: 5 finger-flask contact forces (N)."""
    down = abs(rel["dz"] - L.SEAT_Z) <= SEAT_DZ_TOL
    centred = rel["radial"] <= SEAT_RADIAL_TOL
    upright = rel["tilt_deg"] <= SEAT_TILT_DEG
    fingers_on = [bool(c >= FINGER_CONTACT_N) for c in contacts]
    fingers_closed = [bool(THETA_MIN <= t <= THETA_MAX) for t in thetas]
    ok = down and centred and upright and all(fingers_on) and all(fingers_closed)
    return ok, dict(down=down, centred=centred, upright=upright, fingers_on=fingers_on, fingers_closed=fingers_closed)


@dataclass
class Measure:
    seated: bool
    released: bool
    still: bool
    detail: dict


class Scorer:
    """Per control step: returns the level, and whether the runtime should run the platform orbit this step
    (``self.shaking``, with ``self.shake_t`` seconds since the shake started)."""

    def __init__(self):
        self.still_steps = 0
        self.shaking = False
        self.shake_k = 0
        self.shake_ok = True
        self.passed = False
        self.level = 0.0
        self.shakes_started = 0
        self.shakes_failed = 0
        self.last = {}

    @property
    def shake_t(self):
        return self.shake_k / CONTROL_HZ

    def step(self, m: Measure):
        if self.passed:
            self.level = 1.0
            return self.level
        valid = m.seated and m.released
        if self.shaking:
            if not m.released:  # a hand came back: stop the test
                self.shaking, self.shake_k, self.still_steps = False, 0, 0
            else:
                self.shake_k += 1
                if not m.seated:
                    self.shake_ok = False
                if self.shake_k >= shake_steps():
                    self.shaking = False
                    if self.shake_ok:
                        self.passed = True
                    else:
                        self.shakes_failed += 1
                    self.shake_k, self.still_steps = 0, 0
        else:
            self.still_steps = self.still_steps + 1 if (valid and m.still) else 0
            if self.still_steps >= STILL_STEPS:
                self.shaking, self.shake_k, self.shake_ok = True, 0, True
                self.shakes_started += 1
        if self.passed:
            level = 1.0
        elif m.seated:
            level = 0.5
        else:
            level = 0.0
        self.level = level
        self.last = dict(level=level, seated=m.seated, released=m.released, still=m.still,
                         shaking=self.shaking, shake_t=round(self.shake_t, 3), shake_ok=self.shake_ok,
                         shakes_started=self.shakes_started, shakes_failed=self.shakes_failed,
                         still_steps=self.still_steps, measure=m.detail)
        return level
