"""Scorer for the gripper-sensor task (engine-independent, numpy only).

The scorer cycles the gripper as a person commissioning it would (pure functions of the switch's position in the
groove, the test part's width and the set screw):
  * jaws closed on the test part  -> magnet at u = per-jaw travel to the part   -> the LED must be ON
  * jaws open                     -> magnet at u = 0                            -> the LED must be OFF
  * jaws closed with no part      -> magnet at u = full stroke (15 mm)          -> the LED must be OFF
The LED is on while the switch's sensing point is within WINDOW / 2 of the magnet (layout.WINDOW = 2 mm).

Levels (score at the end of the trial):
  0.5   LED on with the jaws closed on the part and off with the jaws open.
  0.75  also off with the jaws closed empty (the switch detects the part, not the end of stroke).
  1.0   also on with the part 1.2 mm narrower and 1.2 mm wider (the switch sits within 0.4 mm of the middle of its
        switching window, as the switch maker advises), and locked: the set screw turned 90 deg clockwise with the screwdriver engaged in it, so a 5 N push along
        the groove does not move the switch; the override released, nothing touching the switch or its screw, both hands and the screwdriver 8 cm or more
        from the switch, held
        for 1 s. Ends the trial.
  0     anything else (including a locked switch in the wrong place).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import layout as L

HOLD_STEPS = 20  # 1 s at 20 Hz


def led(u_switch, u_magnet, window=L.WINDOW):
    """LED state of the switch for a magnet position (both in the piston coordinate, m)."""
    return abs(float(u_switch) - float(u_magnet)) <= window / 2 + 1e-12


def cycle(u_switch, travel):
    """The checks a person makes by cycling the gripper with the part in, open, and empty, and with the two
    tolerance blocks (the part 1.2 mm narrower and 1.2 mm wider)."""
    return dict(
        on_part=led(u_switch, travel),
        off_open=not led(u_switch, 0.0),
        off_empty=not led(u_switch, L.STROKE),
        on_tolerance=led(u_switch, travel - L.TOL_TRAVEL)
        and led(u_switch, travel + L.TOL_TRAVEL),
    )


def holds_push(locked, push_n=L.PUSH_TEST_N):
    """Does the switch stay put under a push of ``push_n`` along the groove?"""
    limit = L.LOCKED_HOLD_N if locked else L.SLIDE_STATIC_N
    return push_n < limit


def static_level(u_switch, travel, locked):
    """Level from the outcome alone (no hold): 0, 0.5, 0.75 or 1.0 (1.0 = locked and correct)."""
    c = cycle(u_switch, travel)
    if not (c["on_part"] and c["off_open"]):
        return 0.0
    if not c["off_empty"]:
        return 0.5
    if not (c["on_tolerance"] and holds_push(locked)):
        return 0.75
    return 1.0


@dataclass
class Scorer:
    """Per control step: the switch position, the part travel, the lock state, the override state and whether
    anything (robot or screwdriver) touches the switch or its screw. 1.0 latches after HOLD_STEPS clear steps."""

    clear_steps: int = 0
    done: bool = False
    last: dict = field(default_factory=dict)

    def step(
        self,
        u_switch,
        travel,
        locked,
        override_on,
        touching,
        screw_deg=0.0,
        hands_clear=True,
    ):
        lvl = static_level(u_switch, travel, locked)
        clear = (not override_on) and (not touching) and hands_clear
        if self.done:
            level = 1.0
        else:
            if lvl >= 1.0 and clear:
                self.clear_steps += 1
            else:
                self.clear_steps = 0
            if self.clear_steps >= HOLD_STEPS:
                self.done = True
                level = 1.0
            else:
                level = min(lvl, 0.75)
        c = cycle(u_switch, travel)
        self.last = dict(
            level=level,
            outcome_level=lvl,
            u_switch_mm=round(float(u_switch) * 1e3, 3),
            travel_mm=round(float(travel) * 1e3, 3),
            offset_mm=round((float(u_switch) - float(travel)) * 1e3, 3),
            locked=bool(locked),
            screw_deg=round(float(screw_deg), 1),
            override_on=bool(override_on),
            touching=bool(touching),
            hands_clear=bool(hands_clear),
            clear_steps=self.clear_steps,
            **c,
        )
        return level
