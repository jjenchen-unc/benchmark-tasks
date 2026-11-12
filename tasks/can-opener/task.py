"""On the station table: a bench can opener is clamped to the table in front of the arms, with a food can (7 cm wide)
already standing in its seat. A spring holds the opener's cutter lever up. The lever's black handle points toward the
robot; the crank with its wooden knob is on the right of the can. A wooden tray is to the front right. The robot must
hold the lever down so the cutter bites the lid, turn the crank clockwise until the can has turned once all the way
round, let the lever up, lift the cut lid off the can and put it in the tray.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from hingekit import Hinge, on_reset, on_step, part, set_targets, set_weld, weld  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Open the can with the bench can opener and put the lid in the wooden tray."

STEEL = (0.62, 0.64, 0.66)
TIN = (0.80, 0.81, 0.83)
IRON = (0.22, 0.23, 0.25)
BLACK = (0.06, 0.06, 0.06)
RED = (0.72, 0.12, 0.10)
WHITE = (0.95, 0.95, 0.92)
WOOD = (0.62, 0.42, 0.22)
TRAY_WOOD = (0.76, 0.58, 0.36)

# -- the opener, in its own frame: origin on the table under the can's axis, x forward, y left (metres) --------------
OPENER_XY = (0.30, 0.0)
CAN_R, CAN_H = 0.034, 0.075          # a short food can: 68 mm body, 76 mm over the seam, 8.4 cm tall with its lid
SEAT_Z = 0.012                       # top of the base plate
CAN_TOP = SEAT_Z + CAN_H
LID_R, LID_H = 0.036, 0.008          # the lid with its seam, which comes off with it
PIVOT = (0.075, 0.0625, 0.110)       # lever hinge, axis along y, behind the can on its left
HANDLE_PRESS = (-0.070, 0.0625, 0.123)   # middle of the handle's top face (lever closed)
LEVER_OPEN = math.radians(22)        # where its spring holds it
LEVER_CUTTING = math.radians(3)      # the cutter bites the lid while the lever is within 3 degrees of closed
CRANK_AXIS = (0.0, -0.090)           # crank shaft (vertical), on the right of the can
CRANK_Z = CAN_TOP + 0.012            # top of the gear housing
CRANK_R = 0.045                      # crank arm, shaft to knob
KNOB_H = 0.030
KNOB_TOP = CRANK_Z + 0.012 + KNOB_H
GEAR = 1.0 / 3.0                     # the can turns a third of a turn per turn of the crank

BASE_BOXES = [
    ((0.02, -0.02, SEAT_Z / 2), (0.16, 0.20, SEAT_Z), IRON),                         # base plate
    ((0.075, 0.0625, (SEAT_Z + 0.104) / 2), (0.03, 0.03, 0.104 - SEAT_Z), IRON),     # lever post
    ((*CRANK_AXIS, (SEAT_Z + CRANK_Z) / 2), (0.035, 0.030, CRANK_Z - SEAT_Z), IRON),  # crank gear housing
]
BASE_CYLS = [
    ((0.0, -0.048, CAN_TOP - 0.006), 0.014, 0.010, STEEL),                   # drive wheel on the seam
    (PIVOT, 0.006, 0.040, STEEL, "Y"),                                        # lever pin
]
CAN = Hinge("can", anchor=(0.0, 0.0, SEAT_Z), axis="Z", rate=20.0, damping=1.0, mass=0.35, cylinders=(
    ((0.0, 0.0, SEAT_Z + CAN_H / 2), CAN_R, CAN_H, TIN),
    ((0.0, 0.0, SEAT_Z + CAN_H / 2), CAN_R + 0.0006, 0.055, RED),             # paper label
), boxes=(
    ((-CAN_R - 0.0008, 0.0, SEAT_Z + CAN_H / 2), (0.002, 0.012, 0.055), WHITE),  # white stripe, facing the robot
))
LEVER = Hinge("lever", anchor=PIVOT, axis="Y", lower=0.0, upper=math.radians(24), start=LEVER_OPEN, rate=2.1,
              damping=0.02, mass=0.10, boxes=(
                  ((-0.0025, 0.0625, 0.110), (0.175, 0.015, 0.015), STEEL),        # lever bar
                  ((-0.065, 0.0625, 0.112), (0.050, 0.024, 0.022), BLACK),         # handle
                  ((0.000, 0.0425, 0.0995), (0.012, 0.025, 0.006), IRON),          # cutter head, over the rim
              ))
CRANK = Hinge("crank", anchor=(*CRANK_AXIS, CRANK_Z), axis="Z", damping=0.10, mass=0.05, boxes=(
    ((CRANK_AXIS[0], CRANK_AXIS[1] - CRANK_R / 2, CRANK_Z + 0.008), (0.012, CRANK_R + 0.012, 0.008), STEEL),
), cylinders=(((*CRANK_AXIS, CRANK_Z + 0.006), 0.009, 0.012, STEEL),))
KNOB_XY = (CRANK_AXIS[0], CRANK_AXIS[1] - CRANK_R)
KNOB = Hinge("knob", anchor=(*KNOB_XY, CRANK_Z + 0.012), axis="Z", parent="crank", damping=0.001, mass=0.01,
             cylinders=(((*KNOB_XY, CRANK_Z + 0.012 + KNOB_H / 2), 0.011, KNOB_H, WOOD),))

OPENER = part("opener", pos=(*OPENER_XY, TOP), boxes=BASE_BOXES, cylinders=BASE_CYLS, mass=None,
              hinges=(CAN, LEVER, CRANK, KNOB), color=IRON, friction=0.8, body_mass=3.0)
LID = part("lid", pos=(*OPENER_XY, TOP + CAN_TOP + LID_H / 2), mass=0.015, color=TIN, friction=0.8,
           cylinders=(((0.0, 0.0, 0.0), LID_R, LID_H, TIN),))
weld(LID, OPENER, "can")

# -- the tray (fixed): wooden, 12 x 12 cm inside, 2.5 cm walls ------------------------------------------------------
TRAY_X, TRAY_Y = 0.18, -0.17
TRAY_INSIDE = 0.12
TRAY_WALL, TRAY_WALL_H = 0.008, 0.025
TRAY_FLOOR = TOP + 0.006


def _tray():
    lx = ly = TRAY_INSIDE
    w, h = TRAY_WALL, TRAY_WALL_H
    return [
        Box("tray_floor", pos=(TRAY_X, TRAY_Y, TOP + 0.003), size=(lx + 2 * w, ly + 2 * w, 0.006), color=TRAY_WOOD),
        Box("tray_front", pos=(TRAY_X - (lx + w) / 2, TRAY_Y, TOP + h / 2), size=(w, ly + 2 * w, h), color=TRAY_WOOD),
        Box("tray_back", pos=(TRAY_X + (lx + w) / 2, TRAY_Y, TOP + h / 2), size=(w, ly + 2 * w, h), color=TRAY_WOOD),
        Box("tray_left", pos=(TRAY_X, TRAY_Y + (ly + w) / 2, TOP + h / 2), size=(lx, w, h), color=TRAY_WOOD),
        Box("tray_right", pos=(TRAY_X, TRAY_Y - (ly + w) / 2, TOP + h / 2), size=(lx, w, h), color=TRAY_WOOD),
    ]


OBJECTS = [OPENER, LID, *_tray()]

# -- the cut ---------------------------------------------------------------------------------------------------------
# The crank drives the can through the drive wheel (GEAR). While the lever is held within 3 degrees of closed, the
# cutter cuts the part of the rim that passes under it in the cutting direction (crank clockwise seen from above).
# The rim is tracked in 1-degree pieces; turning back over a cut piece, or turning with the lever up, cuts nothing.
# When every piece is cut, the lid comes free of the can.
RIM_BINS = 360
_STATE: dict = {}


@on_reset
def _reset_state(environment=None) -> None:
    _STATE.update(cut=np.zeros(RIM_BINS, bool), crank_prev=None, can_angle=0.0, released=False, lifted=False,
                  lid_z0=None, step=0, scored=None, last=(0.0, False))
    if environment is not None:
        set_weld("lid", True)
        set_targets(environment, "opener", {"can": 0.0})


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@on_step
def _turn_and_cut(environment) -> None:
    from hingekit import _part_state, _SPECS

    _STATE["step"] += 1
    joints = _part_state(environment, _SPECS["opener"]).joints
    crank = joints["crank"]
    if _STATE["crank_prev"] is None:
        _STATE["crank_prev"] = crank
    d = _wrap(crank - _STATE["crank_prev"])
    _STATE["crank_prev"] = crank
    a0 = _STATE["can_angle"]
    a1 = a0 + d * GEAR
    _STATE["can_angle"] = a1
    # the joint angle wraps, so aim the can's drive the short way from where it is
    set_targets(environment, "opener", {"can": joints["can"] + _wrap(a1 - joints["can"])})
    # The cutter is fixed; the rim point under it is at angle -a on the can. Clockwise (negative) turns cut.
    if joints["lever"] < LEVER_CUTTING and a1 < a0 and not _STATE["released"]:
        lo, hi = -a0, -a1
        k0, k1 = math.floor(math.degrees(lo)), math.ceil(math.degrees(hi))
        for k in range(k0, max(k1, k0 + 1)):
            _STATE["cut"][k % RIM_BINS] = True
    if not _STATE["released"] and _STATE["cut"].all():
        _STATE["released"] = True
        set_weld("lid", False)


_reset_state()


def cut_fraction() -> float:
    return float(_STATE["cut"].mean())


def lid_in_tray(lid) -> bool:
    """Lid centre inside the tray walls, lying on the tray floor (flat within 20 degrees) and still."""
    dx, dy = abs(lid.pos[0] - TRAY_X), abs(lid.pos[1] - TRAY_Y)
    inside = dx < TRAY_INSIDE / 2 and dy < TRAY_INSIDE / 2
    up = lid.rotation()[2, 2]
    return bool(inside and lid.pos[2] < TRAY_FLOOR + 0.02 and up > math.cos(math.radians(20)) and lid.resting())


def _update(objects) -> tuple[float, bool]:
    if _STATE["scored"] == _STATE["step"]:
        return _STATE["last"]
    _STATE["scored"] = _STATE["step"]
    lid = objects["lid"]
    if _STATE["lid_z0"] is None:
        _STATE["lid_z0"] = float(lid.pos[2])
    _STATE["lifted"] |= bool(_STATE["released"] and lid.pos[2] - _STATE["lid_z0"] > 0.03)
    if _STATE["released"] and lid_in_tray(lid):
        result = (1.0, True)
    elif _STATE["lifted"]:
        result = (0.75, False)
    elif _STATE["released"]:
        result = (0.5, False)
    else:
        result = (0.25 if cut_fraction() >= 0.5 else 0.0, False)
    _STATE["last"] = result
    return result


def success(objects) -> bool:
    """The cut lid lies flat in the tray and is still, for 1 s."""
    return _update(objects)[1]


def score(objects) -> float:
    """0.25 half the rim cut; 0.5 the lid cut free; 0.75 the lid lifted 3 cm off the can; 1.0 the lid in the tray."""
    return _update(objects)[0]

