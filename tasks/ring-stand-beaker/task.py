"""On the station table, in front of the arms: a laboratory ring stand and a 400 mL glass beaker of water.

The stand is a light cast-aluminium base plate (24 x 12 cm) with a 30 cm upright rod near its right end. A support ring
with a square of wire gauze on it is clamped to the rod 7 cm above the table; the ring's clamp turns around the rod
under a light push and stays where it is left. At the start the ring sticks out toward the robot, past the front edge
of the base. The beaker (8 cm wide, 11 cm tall, 0.48 kg with its water) stands on the table to the left.

The robot must set the beaker on the gauze and leave it there with the stand still standing.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py ...   (plays demo())
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from standkit import Swing, _rot, env, gripper_points, on_reset, on_step, part  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Cylinder  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 2.0
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Set the beaker of water on the wire gauze of the ring stand, and leave the stand standing."

ALU = (0.70, 0.71, 0.73)
STEEL = (0.45, 0.46, 0.48)
DARK = (0.18, 0.18, 0.20)
GAUZE = (0.60, 0.60, 0.58)
WATER = (0.55, 0.78, 0.95)

# -- the ring stand (stand frame: origin on the table under the rod's axis, x forward, y left) ------------------------
ROD_XY = np.array([0.32, -0.06])
PLATE = ((0.0, 0.09, 0.004), (0.12, 0.24, 0.008))   # base plate: x -6..6 cm, y -3..21 cm, 8 mm thick
ROD = ((0.0, 0.0, 0.158), 0.006, 0.30)              # 12 mm rod, 30 cm tall, standing on the plate
PLATE_KG, ROD_KG = 0.62, 0.12
STAND_COM = tuple(((PLATE_KG * np.array(PLATE[0]) + ROD_KG * np.array(ROD[0])) / (PLATE_KG + ROD_KG)).tolist())

# The support ring, in a frame on the rod's axis at the ring's height; at the start it points along -x (toward the
# robot). A boss clamps it to the rod; a 12 cm bar runs out to a ring of 10 cm diameter carrying a 12.5 cm square of
# wire gauze.
RING_H = 0.070                       # bar and ring centre above the table
RING_R = 0.18                        # ring centre from the rod's axis
RING_IN = 0.050                      # ring radius (to the wire's centre line)
GAUZE_SIDE = 0.125
GAUZE_TOP = 0.007                    # above RING_H
_seg = 2 * RING_IN * math.tan(math.pi / 8)
RING_BOXES = (
    ((0.0, 0.0, 0.0), (0.03, 0.03, 0.03), DARK),                                  # boss
    ((-(0.015 + RING_R - RING_IN) / 2, 0.0, 0.0), (RING_R - RING_IN - 0.015, 0.01, 0.01), STEEL),   # bar
    *(((-RING_R + RING_IN * math.cos(a), RING_IN * math.sin(a), 0.0), (0.006, _seg + 0.004, 0.008), STEEL, a)
      for a in np.arange(8) * math.pi / 4),                                       # the ring, eight straight pieces
    ((-RING_R, 0.0, GAUZE_TOP - 0.001), (GAUZE_SIDE, GAUZE_SIDE, 0.002), GAUZE),  # wire gauze
)
RING = Swing("ring", anchor=(0.0, 0.0, RING_H), boxes=RING_BOXES, mass=0.15, com=(-0.15, 0.0, 0.0), damping=0.3)
STAND = part("ring_stand", pos=(ROD_XY[0], ROD_XY[1], TOP), mass=PLATE_KG + ROD_KG, com=STAND_COM,
             boxes=(PLATE[0:2] + (ALU,),), cylinders=((ROD[0], ROD[1], ROD[2], STEEL),), swing=RING, color=ALU,
             friction=0.6)

BEAKER_R, BEAKER_H = 0.040, 0.110
BEAKER_XY = (0.18, 0.24)
BEAKER = Cylinder("beaker", pos=(*BEAKER_XY, TOP + BEAKER_H / 2), radius=BEAKER_R, height=BEAKER_H, mass=0.48,
                  color=WATER)

OBJECTS = [STAND, BEAKER]

# -- scoring -----------------------------------------------------------------------------------------------------------
TURN_DEG = 45.0          # partial credit: the ring turned this far from where it started
STAND_FALL_DEG = 5.0     # the stand counts as tipped past this tilt, at any time
STAND_MOVE_M = 0.03      # or if its base slid further than this
SPILL_DEG = 30.0         # the beaker counts as spilled past this tilt, at any time
CLEAR_M = 0.02           # hands-off: every gripper point this far from the beaker and the stand
_S: dict = {}


@on_reset
def _reset_state(environment=None) -> None:
    _S.update(step=0, scored=None, last=(0.0, False), start=None, fell=False, spilled=False, lifted=False)


@on_step
def _count_step(environment=None) -> None:
    _S["step"] += 1


_reset_state()


def _tilt_deg(quat_xyzw) -> float:
    x, y, _, _ = quat_xyzw
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def ring_centre(stand):
    """World position of the gauze's top centre."""
    return stand.swing_to_world((-RING_R, 0.0, GAUZE_TOP))


def _box_dist(p, centre, size) -> float:
    d = np.abs(np.asarray(p) - np.asarray(centre)) - np.asarray(size) / 2
    return float(np.linalg.norm(np.maximum(d, 0.0)))


def _clear(stand, beaker) -> bool:
    """Every gripper point at least CLEAR_M from the beaker and from the stand's plate, rod, bar and gauze."""
    environment = env()
    if environment is None:
        return False
    R = stand.rotation()
    Rsw = _rot(stand.swing_quat_xyzw)
    for p in gripper_points(environment):
        b = p - beaker.pos
        radial = math.hypot(b[0], b[1]) - BEAKER_R
        axial = abs(b[2]) - BEAKER_H / 2
        if math.hypot(max(radial, 0.0), max(axial, 0.0)) < CLEAR_M:
            return False
        local = R.T @ (p - stand.pos)
        if _box_dist(local, PLATE[0], PLATE[1]) < CLEAR_M or _box_dist(local, ROD[0], (0.012, 0.012, 0.30)) < CLEAR_M:
            return False
        sw = Rsw.T @ (p - stand.swing_pos)
        for centre, size, *_ in (RING_BOXES[1], RING_BOXES[-1]):
            if _box_dist(sw, centre, size) < CLEAR_M:
                return False
    return True


def status(objects) -> dict:
    stand, beaker = objects["ring_stand"], objects["beaker"]
    if _S["start"] is None:
        _S["start"] = stand.pos[:2].copy()
    stand_tilt = _tilt_deg(stand.quat_xyzw)
    beaker_tilt = _tilt_deg(beaker.quat_xyzw)
    moved = float(np.linalg.norm(stand.pos[:2] - _S["start"]))
    _S["fell"] |= stand_tilt > STAND_FALL_DEG or moved > STAND_MOVE_M
    _S["spilled"] |= beaker_tilt > SPILL_DEG
    bottom = float(beaker.pos[2]) - BEAKER_H / 2
    _S["lifted"] |= bool(bottom > TOP + 0.03 and beaker_tilt < SPILL_DEG)
    rc = ring_centre(stand)
    on_gauze = bool(abs(bottom - rc[2]) < 0.006 and np.linalg.norm(beaker.pos[:2] - rc[:2]) < 0.03
                    and beaker_tilt < 10.0)
    turned = abs(math.degrees(stand.swing_angle)) >= TURN_DEG
    return dict(stand_tilt=stand_tilt, beaker_tilt=beaker_tilt, moved=moved, on_gauze=on_gauze, turned=turned,
                ring_centre=rc, beaker_bottom=bottom, fell=_S["fell"], spilled=_S["spilled"], lifted=_S["lifted"])


def _update(objects) -> tuple[float, bool]:
    # success() and score() both run every control step; evaluate each step once.
    if _S["scored"] == _S["step"]:
        return _S["last"]
    _S["scored"] = _S["step"]
    st = status(objects)
    if st["fell"] or st["spilled"]:
        result = (0.0, False)
    else:
        done = st["on_gauze"] and st["stand_tilt"] < 3.0 and _clear(objects["ring_stand"], objects["beaker"])
        if done:
            result = (1.0, True)
        elif st["turned"] and st["lifted"]:
            result = (0.5, False)
        elif st["turned"]:
            result = (0.25, False)
        else:
            result = (0.0, False)
    _S["last"] = result
    return result


def success(objects) -> bool:
    """The beaker upright on the gauze, the stand upright and in place, both grippers clear; held for 2 s."""
    return _update(objects)[1]


def score(objects) -> float:
    """0 if the stand ever tipped (5 deg) or slid (3 cm) or the beaker ever tilted past 30 deg. Else 1.0 for the
    beaker on the gauze with the stand standing and the robot clear; 0.5 if the ring was turned 45 deg from its start
    and the beaker lifted 3 cm; 0.25 if the ring was turned 45 deg."""
    return _update(objects)[0]


# -- scripted demo (planned from the fixed start layout only) -----------------------------------------------------------
PUSH_OFF = 0.09          # gripper centre from the ring's centre line while pushing (the gauze is 6.25 cm half-wide)
SWING_END = -math.radians(92)
BEAKER_GRIP = 0.080      # gripper centre above the beaker's bottom
TRAVEL = TOP + 0.19


def _pusher(angle):
    """Gripper centre beside the gauze's trailing edge (seen turning clockwise from above) with the ring at ``angle``."""
    c, s = math.cos(angle), math.sin(angle)
    return (ROD_XY[0] - RING_R * c + PUSH_OFF * s, ROD_XY[1] - RING_R * s - PUSH_OFF * c, TOP + RING_H)


def _ring_xy(angle):
    return (ROD_XY[0] - RING_R * math.cos(angle), ROD_XY[1] - RING_R * math.sin(angle))


def demo():
    """Left arm: with closed fingers, push the gauze's edge round the rod, swinging the ring 90 degrees clockwise (seen
    from above) until it points along the base plate; then pick up the beaker and set it on the gauze. DEMO_VARIANT=no_swing skips the swing;
    swing_only stops after it; swing:<degrees> swings only that far before loading; tip presses the stand over."""
    variant = os.environ.get("DEMO_VARIANT", "")
    if variant == "nothing":
        return {}
    if variant == "tip":
        # press the overhanging gauze down until the stand tips
        rx, ry = _ring_xy(0.0)
        z = TOP + RING_H + GAUZE_TOP + PAD_TIP_PAST_TCP
        return {"right": [Waypoint(2.0, (rx, ry, z + 0.08), FINGER_CLOSED, "above the gauze"),
                          Waypoint(1.5, (rx, ry, z + 0.005), FINGER_CLOSED, "on the gauze"),
                          Waypoint(2.5, (rx, ry, z - 0.06), FINGER_CLOSED, "press down"),
                          Waypoint(1.5, (rx - 0.03, ry, z + 0.08), FINGER_CLOSED, "up")]}
    end = -math.pi / 2
    swing_end = SWING_END
    if variant.startswith("swing:"):  # swing only this many degrees, then load the ring where it is
        end = swing_end = -math.radians(float(variant[6:]))
    plan = []
    if variant != "no_swing":
        # Closed fingers beside the gauze's trailing edge, then round the rod: the ring's far side swings away from
        # the robot and comes to rest along the base plate.
        x0, y0, z0 = _pusher(0.0)
        plan += [Waypoint(2.0, (x0, y0, TOP + 0.11), FINGER_CLOSED, "above the gauze's edge"),
                 Waypoint(1.5, (x0, y0, z0), FINGER_CLOSED, "beside the gauze")]
        for a in np.linspace(0.0, swing_end, 11)[1:]:
            plan.append(Waypoint(0.7, _pusher(a), FINGER_CLOSED, "swing the ring"))
        xe, ye, ze = _pusher(swing_end)
        plan += [Waypoint(0.8, (xe - 0.03, ye, ze), FINGER_CLOSED, "back off"),
                 Waypoint(1.0, (xe - 0.03, ye, TOP + 0.19), FINGER_OPEN, "up")]
        if variant == "swing_only":
            plan.append(Waypoint(1.5, (0.18, 0.20, TOP + 0.15), FINGER_OPEN, "clear"))
            return {"left": plan}
    angle = 0.0 if variant == "no_swing" else end
    rx, ry = _ring_xy(angle)
    bx, by = BEAKER_XY
    grip_z = TOP + BEAKER_GRIP
    place_z = TOP + RING_H + GAUZE_TOP + BEAKER_GRIP + 0.003
    plan += [Waypoint(2.0, (bx, by, TRAVEL), FINGER_OPEN, "above the beaker"),
             Waypoint(1.5, (bx, by, grip_z), FINGER_OPEN, "around the beaker"),
             Waypoint(0.8, (bx, by, grip_z), FINGER_CLOSED, "grip"),
             Waypoint(1.5, (bx, by, TRAVEL), FINGER_CLOSED, "lift"),
             Waypoint(2.5, (rx, ry, TRAVEL), FINGER_CLOSED, "carry"),
             Waypoint(1.5, (rx, ry, place_z), FINGER_CLOSED, "lower onto the gauze"),
             Waypoint(0.8, (rx, ry, place_z), FINGER_OPEN, "let go"),
             # straight back toward the robot: the open fingers pass either side of the beaker
             Waypoint(2.0, (rx - 0.13, ry, place_z), FINGER_OPEN, "back out"),
             Waypoint(1.5, (rx - 0.15, ry + 0.05, TRAVEL), FINGER_OPEN, "clear")]
    return {"left": plan}
