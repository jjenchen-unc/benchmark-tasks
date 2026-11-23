"""Scorer for the triple-beam task (pure Python, no simulator): weigh a sealed sample jar of unknown mass on an Ohaus
750-SO triple-beam balance.

Inputs each control step (20 Hz), all read from the simulator by the task kit (``isaac/balkit.py``):
* ``pointer_mm``: the pointer's height against the zero mark (+ above: pan side heavy), from the beam angle;
* ``speed_mm_s``: the pointer's speed;
* ``riders``: position of each rider on its beam (m from the fulcrum): ``r500``, ``r100`` (notched) and ``r10`` (the
  sliding 10 g poise);
* ``on_pan``: the jar stands on the pan;
* ``contact``: dict of robot-contact flags per balance part and the jar (PhysX contact forces between the robot's links
  and those bodies: collider identity, not positions);
* ``mass_g``: the jar's true mass (start state).

Rules (the trial's score is the level at its end):
* reading: the notched riders' notch values plus the poise's scale reading; there is no reading while a notched rider
  is not seated in a notch (more than 0.3 mm from a notch bottom, i.e. left on the crest between notches);
* at rest on zero: the pointer within +-0.9 mm of the zero mark (about +-0.18 g) and slower than 0.6 mm/s,
  continuously for 1.5 s;
* hands off: no robot contact with any part of the balance or the jar for 2 s;
* 1.0 (``success``; ends the trial): the jar on the pan, at rest on zero, hands off, every notched rider seated and the
  reading within 0.2 g of the jar's true mass;
* 0.5: the jar on the pan, at rest on zero, hands off, but a notched rider is not seated in its notch (no valid
  reading) or the reading is more than 0.2 g off;
* 0 otherwise: the beam on a stop or swinging, the pointer off the zero mark, a hand on the balance, the jar off the
  pan.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import layout as L  # noqa: E402

READ_TOL_G = 0.2
CLEAR_S = 2.0


class Scorer:
    def __init__(self):
        self.t = 0.0
        self.rest_s = 0.0
        self.clear_s = 0.0
        self.peak_pointer = 0.0
        self.last = {}

    def step(self, dt, pointer_mm, speed_mm_s, riders, on_pan, contact, mass_g):
        self.t += dt
        on_zero = abs(pointer_mm) <= L.ZERO_BAND_MM and speed_mm_s <= L.REST_SPEED_MM_S
        self.rest_s = self.rest_s + dt if on_zero else 0.0
        touching = any(bool(v) for v in contact.values())
        self.clear_s = 0.0 if touching else self.clear_s + dt
        reading = L.reading_g(riders)
        err = None if reading is None else reading - mass_g
        level = self.level(on_pan, reading, err)
        self.last = dict(
            level=level,
            reading_g=None if reading is None else round(reading, 3),
            err_g=None if err is None else round(err, 3),
            pointer_mm=round(pointer_mm, 3),
            speed_mm_s=round(speed_mm_s, 3),
            rest_s=round(self.rest_s, 3),
            clear_s=round(self.clear_s, 3),
            on_pan=bool(on_pan),
            seated={k: abs(L.notch_state(k, riders[k])[1]) <= L.SEAT_TOL for k in ("r500", "r100")},
            notches={k: L.notch_state(k, riders[k])[0] for k in ("r500", "r100")},
            poise_g=round(L.grams_of_s("r10", riders["r10"]), 3),
            contact={k: bool(v) for k, v in contact.items()},
        )
        return level

    def level(self, on_pan, reading, err):
        balanced = on_pan and self.rest_s >= L.REST_S - 1e-9 and self.clear_s >= CLEAR_S - 1e-9
        if not balanced:
            return 0.0
        if reading is not None and abs(err) <= READ_TOL_G:
            return 1.0
        return 0.5
