"""Scorer for the regulator-set task (pure Python, no simulator): set a non-relieving shop-air regulator to 90 psi.

Inputs each control step (20 Hz), all read from the simulator by the task kit (``isaac/regkit.py``):
* ``gauge_psi``: what the gauge needle shows (from the needle's joint angle) and ``gauge_rate`` (psi/s);
* ``line_psi``: the downstream line pressure; ``set_psi``: the regulator's spring setpoint (the pressure it delivers
  under a draw: a person checks it by cracking the bleed or running the tool briefly and reading the gauge);
* ``knob_out_mm``: the knob's push-to-lock travel (0 = pushed in, locked; 4 = pulled out, free);
* ``valve_deg``: the bleed ball valve's lever (0 = closed, across the pipe; 90 = open, along the pipe);
* ``contact``: dict of robot-contact flags per part (knob, lever, the rest of the drop): PhysX contact forces between
  the robot's links and those bodies (collider identity, not positions).

Rule (the trial's score is the level at its end; binary, no partial credit):
* 1.0 (``success``; ends the trial): continuously for HOLD_S = 2 s: gauge within +-2 psi of 90 and steady (needle
  slower than 1 psi/s), the regulator's setpoint within +-2 psi of 90 (so the gauge stays there when air is drawn),
  the knob pushed in (locked: travel below 1 mm), the bleed closed (lever within 3 deg of its closed stop) and no
  robot contact with the drop for at least 1.5 s;
* 0 otherwise. A partial level is not given: every intermediate state (knob unlocked, bleed open, close but not in
  band) is one a blind policy can also reach.
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import layout as L  # noqa: E402

LOCKED_MM = 1.0


def conditions(gauge_psi, gauge_rate, line_psi, set_psi, knob_out_mm, valve_deg):
    """Per-step end-state conditions (without the hands-off hold)."""
    return dict(
        gauge=abs(gauge_psi - L.SETPOINT_PSI) <= L.TOL_PSI and abs(gauge_rate) <= L.GAUGE_STILL_PSI_S,
        line=abs(line_psi - L.SETPOINT_PSI) <= L.TOL_PSI,
        setting=abs(set_psi - L.SETPOINT_PSI) <= L.TOL_PSI,
        locked=knob_out_mm <= LOCKED_MM,
        bleed_closed=valve_deg <= L.VALVE_CLOSED_DEG,
    )


class Scorer:
    def __init__(self):
        self.t = 0.0
        self.ok_s = 0.0
        self.clear_s = 0.0
        self.last = {}

    def step(self, dt, gauge_psi, gauge_rate, line_psi, set_psi, knob_out_mm, valve_deg, contact):
        self.t += dt
        touching = any(bool(v) for v in contact.values())
        self.clear_s = 0.0 if touching else self.clear_s + dt
        c = conditions(gauge_psi, gauge_rate, line_psi, set_psi, knob_out_mm, valve_deg)
        c["clear"] = self.clear_s >= L.CLEAR_S - 1e-9
        self.ok_s = self.ok_s + dt if all(c.values()) else 0.0
        level = 1.0 if self.ok_s >= L.HOLD_S - 1e-9 else 0.0
        self.last = dict(
            level=level,
            gauge_psi=round(gauge_psi, 3),
            gauge_rate=round(gauge_rate, 3),
            line_psi=round(line_psi, 3),
            set_psi=round(set_psi, 3),
            knob_out_mm=round(knob_out_mm, 3),
            valve_deg=round(valve_deg, 2),
            ok_s=round(self.ok_s, 3),
            clear_s=round(self.clear_s, 3),
            cond={k: bool(v) for k, v in c.items()},
            contact={k: bool(v) for k, v in contact.items()},
        )
        return level
