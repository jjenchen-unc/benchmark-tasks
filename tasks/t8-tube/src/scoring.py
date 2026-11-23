"""Scorer for the t8-tube task (pure Python, no simulator): take a dead 2-ft T8 lamp down from the strip fixture under
the shelf and lay it in the lamp-recycling carton on the bench.

Inputs each control step (20 Hz), all read from the simulator by the task kit (``isaac/tubekit.py``):
* ``holder``: the lampholder model's state: 'locked' (pins in the rotors, turned less than 87.4 deg), 'slot' (turned
  open: pins lined up with the slots, which open downward; nothing holds the lamp up), 'out' (both pin pairs below
  the slots) or 'broken' (a lampholder was loaded past its strength for 0.1 s);
* ``in_carton``: both base faces of the lamp inside the carton's inner outline and below its rim;
* ``speed``: the lamp's speed (m/s);
* ``contact``: dict of robot-contact flags for 'lamp', 'carton', 'fixture' (PhysX contact forces between the robot's
  links and those bodies: collider identity, not positions);
* ``dropped``: the kit's drop monitor fired: the lamp was out of the rotors, no robot link touched it, and it moved
  faster than 0.8 m/s (a free fall of more than ~3 cm): counted as a broken glass lamp;
* ``broken``: a lampholder broke.

Rules (the trial's score is the level at its end; vetoes latch 0):
* veto ``lamp_dropped`` and veto ``lampholder_broken`` (above);
* 1.0 (``success``; ends the trial): lamp out of the lampholders, lying in the carton, at rest (speed < 0.02 m/s), no
  robot contact with the lamp for CLEAR_S;
* 0 otherwise (still in the fixture, turned but not taken down, taken down but not laid in the carton, or any veto).
  There is no partial level: a lamp taken down but left in the hands or elsewhere is not a finished relamp, and a
  half-way level could be reached by a replayed trajectory that catches the lamp with one hand only.
"""

from __future__ import annotations

CLEAR_S = 4.0
REST_SPEED = 0.02


class Scorer:
    def __init__(self):
        self.t = 0.0
        self.veto = None
        self.veto_t = None
        self.clear_s = 0.0
        self.out_t = None
        self.carton_t = None
        self.last = {}

    def _veto(self, why):
        if self.veto is None:
            self.veto, self.veto_t = why, round(self.t, 3)

    def step(self, dt, holder, in_carton, speed, contact, dropped=False, broken=False):
        self.t += dt
        if dropped:
            self._veto("lamp_dropped")
        if broken or holder == "broken":
            self._veto("lampholder_broken")
        out = holder == "out"
        if out and self.out_t is None:
            self.out_t = round(self.t, 3)
        lamp_touch = bool(contact.get("lamp", False))
        self.clear_s = 0.0 if lamp_touch else self.clear_s + dt
        if out and in_carton and self.carton_t is None:
            self.carton_t = round(self.t, 3)
        level = self.level(out, in_carton, speed)
        self.last = dict(
            level=level,
            veto=self.veto,
            veto_t=self.veto_t,
            holder=holder,
            out=out,
            out_t=self.out_t,
            in_carton=bool(in_carton),
            carton_t=self.carton_t,
            speed=round(float(speed), 4),
            contact={k: bool(v) for k, v in contact.items()},
            clear_s=round(self.clear_s, 3),
        )
        return level

    def level(self, out, in_carton, speed):
        if self.veto is not None:
            return 0.0
        if out and in_carton and speed < REST_SPEED and self.clear_s >= CLEAR_S - 1e-9:
            return 1.0
        return 0.0
