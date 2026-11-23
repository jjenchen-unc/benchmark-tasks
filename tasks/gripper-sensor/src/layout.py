"""Scene layout and geometry of the gripper-sensor task (station frame: x forward from the arm bases, y to the robot's
left, z up; metres). Engine-independent (numpy only).

The bench: a pneumatic parallel gripper (SMC MHZ2-40D class, double acting) bolted by its mounting face to a vertical
flange plate on a bench test stand, lying on its back so that one auto-switch groove of the body faces up and runs
along x (the piston axis). A solid-state auto switch (SMC D-M9N class) sits loose in that groove. A 5/2 single-solenoid
valve (SMC SY3120 class) with a non-locking push manual override drives the gripper. A steel test part sits in a nest
between the fingers. A 2.5 mm precision slotted screwdriver stands in a rack.

Piston coordinate u (m, along +x): position of the piston magnet relative to its position with the jaws fully open
(u = 0). Closing moves the magnet toward the jaws (+x) by the same distance as each jaw travels (assumed 1:1, see
README fidelity). The switch's sensing point position is given in the same coordinate.
"""

from __future__ import annotations

import numpy as np

TOP = 0.7475  # station table top (yam_arena TABLE_TOP["station"])

# ------------------------------------------------------------------ gripper (MHZ2-40D class, representative outline)
STROKE = 0.015  # per jaw (30 mm opening/closing stroke in total)
BODY_L = 0.096  # along the piston axis (x)
BODY_W = 0.066  # along the jaw motion (y)
BODY_T = 0.042  # z (the groove face is the top face)
FLANGE_X = 0.252  # mounting face (back of the body) on the flange plate, x
BODY_C = np.array([FLANGE_X + BODY_L / 2, -0.040, TOP + 0.052])  # body centre
BODY_TOP = BODY_C[2] + BODY_T / 2
GROOVE_Y = BODY_C[1] + 0.014  # groove centre line (y), on the top face
GROOVE_W = 0.0036
MAGNET_X0 = FLANGE_X + 0.044  # magnet x with the jaws fully open (u = 0)
# jaws and fingers (jaws ride in a guide at the front of the body; fingers bolted on, aluminium blanks)
JAW_GUIDE_L = 0.016  # guide block length (x) in front of the body
FINGER_L = 0.050  # finger length beyond the guide (x)
FINGER_W = 0.012  # finger thickness (y)
FINGER_H = 0.026  # finger height (z)
GAP_OPEN = 0.060  # clear gap between the finger faces, jaws open
JAW_SPEED = 0.10  # m/s per jaw (pneumatic close / open)
FINGER_X0 = FLANGE_X + BODY_L + JAW_GUIDE_L  # finger root (x)
# ------------------------------------------------------------------ test part (steel block) and its nest
PART_L = 0.030  # along x
PART_H = 0.020
PART_X = FINGER_X0 + 0.024  # part centre x (between the fingers)
PART_MIN_TRAVEL = 0.0025  # per-jaw travel to touch the part, range drawn per seed
PART_MAX_TRAVEL = 0.0125
# ------------------------------------------------------------------ auto switch (D-M9N class) in the groove
SW_L = 0.0228  # length along the groove
SW_W = 0.0030
SW_H = 0.0042  # total height; the top sits SW_PROUD above the body face
SW_PROUD = 0.0010
SW_SENSE_BASE = 0.006  # D-M9N drawing: from non-cable end
TOL_TRAVEL = 0.0006  # 1.0 needs the LED on for parts 1.2 mm narrower and wider too (per-jaw travel +-0.6 mm)
LED_FROM_BACK = 0.0035  # LED window centre from the back (cable) end
SCREW_FROM_FRONT = 0.0025  # set-screw axis from the front end
SCREW_R = 0.00125  # M2.5 slotted set screw
CUP_R_IN = 0.00140  # capture cone around the screw's end: flat bottom radius (blade tip radius 1.25 mm)
CUP_R_TOP = 0.00240  # cone radius at the top (about 50 deg lead-in over CUP_DEPTH)
CUP_R_OUT = 0.00250
CUP_DEPTH = 0.0008
CUP_PROUD = 0.0002  # the cone rim stands 0.2 mm above the switch top; its bottom stays 0.4 mm above the body face
U_MIN, U_MAX = (
    -0.030,
    0.030,
)  # switch travel in the groove (sensing point, piston coordinate)
WINDOW = 0.0020  # switching window: LED on while |u_switch - u_magnet| <= WINDOW / 2
SLIDE_STATIC_N = 2.0  # force along the groove to start the loose switch sliding
SLIDE_DYNAMIC_N = 1.8  # keeps it sliding
SLIDE_GAIN = 0.010  # m/s of slide per N above the dynamic force
SLIDE_VMAX = 0.030
LOCKED_HOLD_N = 30.0  # a tightened set screw holds the switch against this force
LOCK_TURN_DEG = 90.0  # screwdriver turn (clockwise from above, while engaged) that tightens the set screw
ENGAGE_PUSH_N = 1.5  # axial push of the blade on the screw for the turn to count
ENGAGE_R = 0.0005  # blade tip within 0.5 mm of the screw axis (down in the capture cone) to engage
ENGAGE_HOLD_R = 0.0016  # once engaged it stays in the slot while the tip is on the head (within 1.6 mm) and pushed
ENGAGE_TILT_DEG = 12.0
ENGAGE_GRACE_S = (
    0.15  # an engaged blade stays engaged through push dips shorter than this
)
PUSH_TEST_N = (
    5.0  # scorer: a locked switch must not move under a 5 N push along the groove
)
HANDS_CLEAR_M = 0.080  # 1.0 also needs both hands (TCPs) and the screwdriver tip this far from the switch
# ------------------------------------------------------------------ valve (SY3120 class) with manual override
VALVE = np.array([0.290, 0.165, TOP])  # base centre on the table (sub-base bracket)
VALVE_BASE = (0.070, 0.030, 0.032)  # bracket + sub-base block
VALVE_BODY = (0.056, 0.010, 0.022)  # valve body on top of the sub-base (x, y, z)
OVR_R = 0.0022  # override button radius
OVR_PROUD = 0.0015
OVR_X = VALVE[0] - 0.012  # button centre (on the pilot end of the body)
OVR_PRESS_N = 5.0  # push to actuate (non-locking: released below OVR_RELEASE_N)
OVR_RELEASE_N = 3.0
# ------------------------------------------------------------------ screwdriver and rack
RACK = np.array([0.205, -0.180, TOP])  # rack block base centre
RACK_SIZE = (0.040, 0.040, 0.045)
RACK_HOLE = 0.0060  # square hole
DRV_HANDLE_R = 0.0075
DRV_HANDLE_L = 0.085
DRV_BLADE_R = 0.00125
DRV_BLADE_L = 0.040
DRV_MASS = 0.030
DRV_FRICTION = 0.9  # handle (soft grip) on the pads
HARD_FRICTION = 0.4
# ------------------------------------------------------------------ flange stand
STAND_PLATE = (0.010, 0.110, 0.085)  # vertical flange plate (x thickness, y, z)
STAND_BASE = (0.150, 0.130, 0.012)  # base plate (x, y, z)


def magnet_x(u):
    return MAGNET_X0 + u


_SENSE = [
    SW_SENSE_BASE
]  # sensing point of the switch fitted in the current trial (set on every reset)


def set_sense(spec):
    """Install the trial's switch (its sensing point) from a setup_spec() result."""
    _SENSE[0] = SW_SENSE_BASE + float(spec.get("sense", 0.0))


def switch_center_x(u_sw):
    """Centre x of the switch body for a sensing-point coordinate u_sw (current trial's switch)."""
    return MAGNET_X0 + u_sw + _SENSE[0] - SW_L / 2


def switch_u(center_x):
    return center_x - MAGNET_X0 - _SENSE[0] + SW_L / 2


def switch_z():
    """Centre z of the switch body."""
    return BODY_TOP + SW_PROUD - SW_H / 2


def screw_xy(u_sw):
    return np.array([switch_center_x(u_sw) + SW_L / 2 - SCREW_FROM_FRONT, GROOVE_Y])


def led_xy(u_sw):
    return np.array([switch_center_x(u_sw) - SW_L / 2 + LED_FROM_BACK, GROOVE_Y])


def part_width(travel):
    return GAP_OPEN - 2.0 * travel


def jaw_y(travel, side):
    """Centre y of a finger for a per-jaw travel (side +1 = the finger at +y)."""
    return BODY_C[1] + side * (GAP_OPEN / 2 - travel + FINGER_W / 2)


def ovr_top():
    return VALVE[2] + VALVE_BASE[2] + VALVE_BODY[2] + OVR_PROUD


def ovr_center():
    return np.array([OVR_X, VALVE[1], ovr_top()])


def rack_top():
    return RACK[2] + RACK_SIZE[2]


def driver_home():
    """Screwdriver frame (origin at the blade tip, +z along the blade toward the handle) standing in the rack, its
    handle's bottom face resting on the rack top."""
    return np.array([RACK[0], RACK[1], rack_top() - DRV_BLADE_L + 0.0002])


# ------------------------------------------------------------------ start states
SEEDED_GAP_M = 0.0035  # the switch starts at least 3.5 mm (sensing point) from the set window's centre
BLOCK_WIDTHS_MM = (36, 40, 44, 48, 52)
# Width in mm and initial sensing coordinate in metres.
SETUPS = ((44, -0.005), (52, 0.015), (36, 0.001), (48, 0.018), (40, -0.005))
START_U_RANGE = (-0.012, 0.024)
BODY_CENTER_RANGE = (
    -0.012,
    0.023,
)  # switch body centre relative to MAGNET_X0 (stays on the groove, within reach)


def setup_spec(setup):
    """Select a physical gauge block and starting location for one fixed D-M9N switch."""
    if isinstance(setup, tuple) and setup[0] == "seed":
        rng = np.random.default_rng(int(setup[1]))
        width_mm = int(rng.choice(BLOCK_WIDTHS_MM))
        t = (GAP_OPEN - width_mm / 1000) / 2
        sense = 0.0
        for _ in range(1000):
            u0 = float(rng.uniform(*START_U_RANGE))
            c0 = u0 + SW_SENSE_BASE + sense - SW_L / 2
            if (
                abs(u0 - t) >= SEEDED_GAP_M
                and BODY_CENTER_RANGE[0] <= c0 <= BODY_CENTER_RANGE[1]
            ):
                break
        return dict(
            travel=t,
            u0=u0,
            sense=sense,
            width=part_width(t),
            name=f"seed {int(setup[1])}",
        )
    k = int(setup) % len(SETUPS)
    width_mm, u0 = SETUPS[k]
    t = (GAP_OPEN - width_mm / 1000) / 2
    sense = 0.0
    return dict(travel=t, u0=u0, sense=sense, width=part_width(t), name=f"setup {k}")
