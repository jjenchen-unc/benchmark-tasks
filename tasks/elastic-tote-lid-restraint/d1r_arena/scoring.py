"""D1R pilot scorer (Arena/PhysX port). Pure Python, evaluator-only.

Consumes one measurement row per 20 Hz control step (see kit.measure). Implements the declared
rubric of the frozen task.json with the port's mechanism (strap.py):

  0    if any permanent failure ever occurred: hook bent open (cord tension above the 28 N hook
       rating), loose hook snapped back under cord pull (> 0.5 m/s while not touching a robot and its
       cord taut), cord through a tote wall/bottom/lid (fail-closed route check), non-finite
       state, missing measurement.
  1.0  exactly one strap on the tall tote, both its hooks seated on the two different eye bars,
       cord in the over-lid route, lid gap < 3 mm, hands off -- held continuously for the last
       >= 20 s (and true at the end).
  0.5  the same strap state with lid gap <= 10 mm and hands off for the last >= 5 s.
  0.25 exactly one strap has a hook seated on a tall-tote eye bar, hands off, for the last >= 5 s.
  else 0.

Required wrist/contact channels are implemented in guards.py and fail closed if absent.
GPU43235 validated channels, force-abort calibration and placed mechanism on its\narchived source. Later source changes need their own runtime evidence; see README.
"""
from __future__ import annotations

import math

GAP_FULL_M, GAP_HALF_M = 0.003, 0.010
HOLD_FULL_S, HOLD_PARTIAL_S = 20.0, 5.0
SNAPBACK_MPS = 0.5


class Scorer:
    unenforced_guards = ()

    def __init__(self, control_hz: float = 20.0):
        self.control_hz = control_hz
        self.steps = 0
        self.failure = None
        self.lid_gap_m = None
        self.max_hook_speed = 0.0
        self.run_full = 0  # consecutive steps meeting each band, ending at the latest step
        self.run_half = 0
        self.run_quarter = 0
        self.last_state = None
        self.hold_strap = None

    @property
    def gate(self):
        return ("unenforced:" + ",".join(self.unenforced_guards)) if self.unenforced_guards else ""

    def update(self, row: dict) -> None:
        self.steps += 1
        if row.get("guard_failure"):
            self.failure = self.failure or row["guard_failure"]
        if row.get("wrist_force_measured") is not True or row.get("robot_contacts_measured") is not True:
            self.failure = self.failure or "missing_guard_measurement"
        if row.get("numerical_warning"):
            self.failure = self.failure or "non_finite_state"
        tall = row.get("totes", {}).get("tote_tall")
        if tall is None or not math.isfinite(tall.get("lid_gap_m", float("nan"))):
            self.failure = self.failure or "missing_measurement"
            gap = float("inf")
        else:
            gap = self.lid_gap_m = tall["lid_gap_m"]
        for h in row.get("hooks", {}).values():
            self.max_hook_speed = max(self.max_hook_speed, h.get("speed_mps", 0.0))
        straps = row.get("straps")
        if straps is None:
            self.failure = self.failure or "missing_measurement"
            straps = {}
        for s in straps.values():
            if s.get("failure"):
                self.failure = self.failure or s["failure"]
        on_tote = [n for n, s in straps.items() if s.get("seated_a") is not None or s.get("seated_b") is not None]
        one = len(on_tote) == 1
        name = on_tote[0] if one else None
        if name != self.hold_strap:
            self.run_full = self.run_half = self.run_quarter = 0
            self.hold_strap = name
        s = straps.get(name, {})
        contacts = row.get("robot_contacts", {})
        expected = {"lid", *tuple(f"strap_{i}_{e}" for i in range(1,5) for e in ("a","b"))}
        if set(contacts) != expected or not all(isinstance(x,bool) for x in contacts.values()):
            self.failure = self.failure or "missing_contact_measurement"
        forces = row.get("wrist_forces_n", {})
        if set(forces) != {"left","right"} or not all(math.isfinite(x) and x >= 0 for x in forces.values()):
            self.failure = self.failure or "missing_wrist_measurement"
        hands_off = (bool(s.get("hands_off")) and bool(row.get("hands_off_lid"))
                     and not any(contacts.values()))
        both = (one and s.get("seated_a") is not None and s.get("seated_b") is not None
                and s["seated_a"] != s["seated_b"] and s.get("route") == "over_lid")
        full = both and hands_off and gap < GAP_FULL_M
        half = both and hands_off and gap <= GAP_HALF_M
        quarter = one and hands_off
        self.run_full = self.run_full + 1 if full else 0
        self.run_half = self.run_half + 1 if half else 0
        self.run_quarter = self.run_quarter + 1 if quarter else 0
        self.last_state = dict(strap=name, both_seated_over_lid=both, hands_off=hands_off, gap_m=gap)

    @property
    def held_s(self):
        return dict(full=self.run_full / self.control_hz, half=self.run_half / self.control_hz,
                    quarter=self.run_quarter / self.control_hz)

    @property
    def score(self) -> float:
        if self.failure or self.steps == 0:
            return 0.0
        h = self.held_s
        eps = 0.5 / self.control_hz
        if h["full"] >= HOLD_FULL_S - eps:
            return 1.0
        if h["half"] >= HOLD_PARTIAL_S - eps:
            return 0.5
        if h["quarter"] >= HOLD_PARTIAL_S - eps:
            return 0.25
        return 0.0
