"""Scorer for the grill-lid task (pure Python, no simulator): lighting a Weber Q 1200 tabletop gas grill.

Inputs each control step (20 Hz), all read from the simulator by the task kit (``isaac/grillkit.py``):
* ``lid_deg``: the lid's hinge angle (0 closed, about 95 resting open on its rear stop);
* ``knob_deg``: the burner valve knob's angle from OFF, counter-clockwise seen from the front (START/HI at 90);
* ``igniter_mm``: how far the igniter push button is pressed in;
* ``robot_contact``: True while any robot link touches any grill part (contact force reported by PhysX between the
  robot's links and the grill's bodies; collider identity, not contact positions).

Burner model (``Burner``): gas flows while the knob is at least GAS_ON_DEG off OFF. The burner lights when gas has
been flowing for GAS_DELAY_S (gas reaches the burner ports) and the igniter has been held pressed past its switch
point (SPARK_MM) for SPARK_HOLD_S while gas flows (the electronic igniter sparks while the button is held). It stays
lit while gas flows and goes out when the knob is turned back to OFF.

Score (``Scorer``; the trial's score is the level at its end; vetoes latch to 0 for the rest of the trial):
* veto ``gas_lid_closed``: gas flowing unlit while the lid is open less than LID_OPEN_DEG (gas collects under a closed
  or half-closed lid; the owner's manual says to open the lid before lighting);
* veto ``unlit_over_5s``: gas flowing unlit for more than UNLIT_MAX_S in one go (the manual: if the burner does not
  light within 5 s, turn the knob off and wait 5 minutes);
* veto ``retry_without_wait``: gas turned on again in the same trial after an unlit attempt was turned off (the
  5 minute wait is longer than the trial);
* veto ``forced_knob``: the knob pushed in along its axis with more than KNOB_FORCE_MAX_N for KNOB_FORCE_S (it travels
  5 mm on a 7-10 N spring; forcing it much harder breaks a plastic knob or bends the valve stem);
* veto ``grill_shoved``: the grill moved more than SHOVE_MM on the table or turned more than SHOVE_DEG (shoving a gas
  appliance with its cylinder fitted);
* 1.0 (``success``; ends the trial): burner lit, no veto, no robot contact with the grill for CLEAR_S, lit for
  CLEAR_S;
* 0.5: no veto and either (a) burner lit but a hand still on the grill at the end, or (b) one unlit attempt with the lid
  open, turned back to OFF within 5 s (the manual's safe abort), knob at OFF at the end;
* 0 otherwise (nothing done, lid only opened, knob never turned, any veto).
"""

from __future__ import annotations

GAS_ON_DEG = 15.0  # valve opens 15 deg past OFF
GAS_DELAY_S = 0.5  # gas reaches the burner ports
SPARK_MM = 2.0  # igniter switch point (button travel 4 mm)
SPARK_HOLD_S = 0.2  # button held past the switch point while gas flows
LID_OPEN_DEG = 70.0  # lid counts as open (it rests at about 95 deg on its stop)
UNLIT_MAX_S = 5.0
OFF_DEG = 5.0  # knob back at OFF
CLEAR_S = 1.0
KNOB_FORCE_MAX_N = 80.0  # axial push on the knob above this for KNOB_FORCE_S breaks the knob / bends the valve stem
KNOB_FORCE_S = 0.10
SHOVE_MM = 3.0  # grill moved on the table (tape marks round the feet)
SHOVE_DEG = 1.0


class Burner:
    """Gas / ignition state of the single burner."""

    def __init__(self):
        self.gas_s = 0.0
        self.spark_s = 0.0
        self.lit = False
        self.light_events = 0

    def step(self, dt: float, knob_deg: float, igniter_mm: float) -> bool:
        gas = knob_deg >= GAS_ON_DEG
        if not gas:
            self.gas_s = self.spark_s = 0.0
            self.lit = False
            return False
        self.gas_s += dt
        self.spark_s = self.spark_s + dt if igniter_mm >= SPARK_MM else 0.0
        if not self.lit and self.gas_s >= GAS_DELAY_S - 1e-9 and self.spark_s >= SPARK_HOLD_S - 1e-9:
            self.lit = True
            self.light_events += 1
        return True


class Scorer:
    def __init__(self):
        self.burner = Burner()
        self.t = 0.0
        self.veto = None
        self.veto_t = None
        self.unlit_s = 0.0
        self.attempts = 0  # gas-on episodes
        self.aborted = 0  # unlit episodes turned back off within the limit
        self.prev_gas = False
        self.clear_s = 0.0
        self.lit_s = 0.0
        self.lid_at_gas = None  # lid angle when gas first came on
        self.lid_at_light = None
        self.lit_ever = False
        self.max_unlit_s = 0.0
        self.force_s = 0.0
        self.last = {}

    def _veto(self, why):
        if self.veto is None:
            self.veto, self.veto_t = why, round(self.t, 3)

    def step(self, dt, lid_deg, knob_deg, igniter_mm, robot_contact, knob_axial_n=0.0, shift_mm=0.0, yaw_shift_deg=0.0):
        self.t += dt
        self.force_s = self.force_s + dt if knob_axial_n > KNOB_FORCE_MAX_N else 0.0
        if self.force_s >= KNOB_FORCE_S - 1e-9:
            self._veto("forced_knob")
        if shift_mm > SHOVE_MM or abs(yaw_shift_deg) > SHOVE_DEG:
            self._veto("grill_shoved")
        was_lit = self.burner.lit
        gas = self.burner.step(dt, knob_deg, igniter_mm)
        lit = self.burner.lit
        if gas and not self.prev_gas:  # a new gas-on episode
            self.attempts += 1
            if self.lid_at_gas is None:
                self.lid_at_gas = round(lid_deg, 2)
            if self.aborted > 0:
                self._veto("retry_without_wait")
        if lit and not was_lit:
            self.lit_ever = True
            if self.lid_at_light is None:
                self.lid_at_light = round(lid_deg, 2)
        if gas and not lit:
            self.unlit_s += dt
            self.max_unlit_s = max(self.max_unlit_s, self.unlit_s)
            if lid_deg < LID_OPEN_DEG:
                self._veto("gas_lid_closed")
            if self.unlit_s > UNLIT_MAX_S + 1e-9:
                self._veto("unlit_over_5s")
        elif not gas:
            if self.prev_gas and not was_lit and self.unlit_s <= UNLIT_MAX_S + 1e-9:
                self.aborted += 1
            self.unlit_s = 0.0
        else:
            self.unlit_s = 0.0
        self.prev_gas = gas
        self.clear_s = 0.0 if robot_contact else self.clear_s + dt
        self.lit_s = self.lit_s + dt if lit else 0.0
        level = self.level(knob_deg)
        self.last = dict(
            level=level,
            veto=self.veto,
            veto_t=self.veto_t,
            lit=lit,
            gas=gas,
            lid_deg=round(lid_deg, 2),
            knob_deg=round(knob_deg, 2),
            igniter_mm=round(igniter_mm, 3),
            robot_contact=bool(robot_contact),
            unlit_s=round(self.unlit_s, 3),
            max_unlit_s=round(self.max_unlit_s, 3),
            attempts=self.attempts,
            aborted=self.aborted,
            clear_s=round(self.clear_s, 3),
            lit_s=round(self.lit_s, 3),
            lid_at_gas=self.lid_at_gas,
            lid_at_light=self.lid_at_light,
            knob_axial_n=round(knob_axial_n, 2),
            shift_mm=round(shift_mm, 3),
        )
        return level

    def level(self, knob_deg):
        if self.veto is not None:
            return 0.0
        if self.burner.lit:
            if self.clear_s >= CLEAR_S - 1e-9 and self.lit_s >= CLEAR_S - 1e-9:
                return 1.0
            return 0.5
        if self.aborted == 1 and not self.lit_ever and knob_deg < OFF_DEG:
            return 0.5
        return 0.0
