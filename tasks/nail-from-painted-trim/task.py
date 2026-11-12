"""On the station table: a white-painted pine trim board (14 x 24 x 2 cm) clamped to the table, with one steel nail
standing 24 mm proud of its painted face (head underside) and 17 mm deep in the wood. A 0.45 kg claw hammer stands
head-down in a wooden pocket to the right, claw toward the robot, handle up through a collar. A scrap block of plywood
(10 x 5 x 1.5 cm, 40 g) lies on the bare table to the left. The nail slips in the wood at 80 N: more than the arm
can pull straight up, but the hammer rocking on its rounded claw-back multiplies the push at its grip about 6.5
times. The paint must not be marked: the hammer may rest on the scrap block but never touch the painted face.
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from partkit import Part, Slider, on_reset, on_step, part  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the painted board and its nail -------------------------------------------------------------------------------
NAIL_XY = np.array([0.33, 0.0])
BOARD = (0.14, 0.24, 0.02)  # x (depth), y (length), z (thickness)
BOARD_CENTRE = np.array([NAIL_XY[0] + 0.03, NAIL_XY[1] + 0.06, TOP + BOARD[2] / 2])
PAINT = TOP + BOARD[2]  # height of the painted face
EMBED = 0.017  # nail length in the wood
PROUD = 0.024  # head underside above the paint
SHANK_R, HEAD_R, HEAD_T = 0.0019, 0.0055, 0.003
NAIL_OUT = EMBED  # lift (m) at which the point leaves the wood
NAIL_FRICTION = 80.0  # N: the pull at which the nail slips in the wood (Coulomb friction)
NAIL_GIVE = 4.0e4  # N/m: elastic give of the wood fibres before the nail slips (2 mm at the slip force)

WHITE, STEEL = (0.95, 0.95, 0.92), (0.55, 0.57, 0.60)
_nail_local = np.r_[NAIL_XY - BOARD_CENTRE[:2], BOARD[2] / 2]  # nail axis at the paint, board frame
NAIL = Slider(
    "nail",
    rest=tuple(_nail_local),
    boxes=(),
    cylinders=(
        (tuple(_nail_local + (0, 0, (PROUD - EMBED) / 2)), SHANK_R, PROUD + EMBED, STEEL),
        (tuple(_nail_local + (0, 0, PROUD + HEAD_T / 2)), HEAD_R, HEAD_T, STEEL),
    ),
    lower=0.0,
    upper=EMBED + 0.002,
    rate=NAIL_GIVE,
    damping=200.0,
    mass=0.05,  # heavier than a real nail (6 g) so the solver keeps the claw from pushing through its head
    friction=0.3,  # bare steel: a pinch of the fingers slips on it
    joint_friction=0.0,
)
# 40 kg stands in for the clamps: the board does not move under the robot.
TRIM = part("trim", pos=tuple(BOARD_CENTRE), mass=40.0, boxes=[((0, 0, 0), BOARD, WHITE)], sliders=[NAIL],
            friction=0.6)

# -- the hammer ---------------------------------------------------------------------------------------------------
# Hammer frame: origin at the centre of the rounded claw-back (radius RB, axis along y); handle up (+z); claw toward
# -x, face toward +x. The claw's slot is open at its tip.
RB = 0.016
THROAT_X = -0.032  # where a nail sits fully in the claw
PRONG_Z = -0.011  # prong centre height
GRIP_Z = 0.21  # where the robot holds the handle
DARK, WOOD, BLUE = (0.25, 0.26, 0.28), (0.78, 0.58, 0.30), (0.12, 0.32, 0.62)

_ham_cyls = [((0, 0, 0), RB, 0.025, DARK, (math.cos(math.pi / 4), math.sin(math.pi / 4), 0.0, 0.0))]  # claw-back
_ham_boxes = [
    ((-0.016, 0, -0.009), (0.020, 0.025, 0.0095), DARK),  # claw root
    ((-0.026, 0, PRONG_Z), (0.006, 0.0172, 0.005), DARK),  # throat (closes the slot)
    ((-0.049, 0.0052, PRONG_Z), (0.040, 0.006, 0.005), DARK),  # prongs, slot 4.4 mm wide
    ((-0.049, -0.0052, PRONG_Z), (0.040, 0.006, 0.005), DARK),
    ((0.0, 0, 0.027), (0.028, 0.025, 0.054), DARK),  # eye
    ((0.031, 0, 0.041), (0.034, 0.025, 0.022), DARK),  # neck
    ((0.052, 0, 0.041), (0.008, 0.027, 0.027), DARK),  # face
    ((0.0, 0, 0.121), (0.024, 0.028, 0.134), WOOD),  # handle
    ((0.0, 0, 0.237), (0.024, 0.030, 0.098), BLUE),  # rubber grip
]
POCKET_XY = np.array([0.22, -0.12])  # where the hammer's claw-back stands
HAMMER = part("hammer", pos=(POCKET_XY[0], POCKET_XY[1], TOP + RB + 0.0005), mass=0.45, boxes=_ham_boxes,
              cylinders=_ham_cyls, com=(0.010, 0.0, 0.045), friction=1.0)

# A fixed wooden pocket on the table, 2 cm tall, round the hammer's head (1.5 mm clear): the hammer stands in it
# head-down, handle up, and lifts straight out.
_PX, _PY = (-0.0705, 0.0585), 0.0150
POCKET = [
    Box("pocket_back", pos=(POCKET_XY[0] + _PX[1] + 0.006, POCKET_XY[1], TOP + 0.01), size=(0.012, 0.054, 0.02),
        color=WOOD),
    Box("pocket_front", pos=(POCKET_XY[0] + _PX[0] - 0.006, POCKET_XY[1], TOP + 0.01), size=(0.012, 0.054, 0.02),
        color=WOOD),
    Box("pocket_left", pos=(POCKET_XY[0] + sum(_PX) / 2, POCKET_XY[1] + _PY + 0.006, TOP + 0.01),
        size=(_PX[1] - _PX[0], 0.012, 0.02), color=WOOD),
    Box("pocket_right", pos=(POCKET_XY[0] + sum(_PX) / 2, POCKET_XY[1] - _PY - 0.006, TOP + 0.01),
        size=(_PX[1] - _PX[0], 0.012, 0.02), color=WOOD),
]
# Above it, a collar keeps the handle upright: two rails either side of the handle (front and back), closed at the
# right by a stop bar on a post, open to the left so the hammer slides out that way.
COLLAR_Z = TOP + 0.145
POCKET += [
    Box("collar_back", pos=(POCKET_XY[0] + 0.0205, POCKET_XY[1] + 0.005, COLLAR_Z), size=(0.005, 0.06, 0.015),
        color=WOOD),
    Box("collar_front", pos=(POCKET_XY[0] - 0.0205, POCKET_XY[1] + 0.005, COLLAR_Z), size=(0.005, 0.06, 0.015),
        color=WOOD),
    Box("collar_stop", pos=(POCKET_XY[0], POCKET_XY[1] - 0.0285, COLLAR_Z), size=(0.046, 0.007, 0.015), color=WOOD),
    Box("collar_post", pos=(POCKET_XY[0], POCKET_XY[1] - 0.036, TOP + 0.0765), size=(0.010, 0.008, 0.153),
        color=WOOD),
]

# -- the scrap block ----------------------------------------------------------------------------------------------
BLOCK_SIZE = (0.10, 0.05, 0.015)
BLOCK = Box("block", pos=(0.22, 0.22, TOP + BLOCK_SIZE[2] / 2), size=BLOCK_SIZE, mass=0.04, color=(0.70, 0.52, 0.32))

OBJECTS = [TRIM, HAMMER, *POCKET, BLOCK]

# -- scoring ------------------------------------------------------------------------------------------------------
# Points on the hammer's surface (hammer frame) that can touch a flat face: the claw-back's rim, the claw's and
# head's lower and outer edges, the handle's corners.
_rim = [(RB * math.sin(t), y, -RB * math.cos(t)) for t in np.radians(np.arange(-180, 180, 15)) for y in (-0.0125, 0.0125)]
_box_pts = []
for spec in _ham_boxes:
    (cx, cy, cz), (sx, sy, sz) = spec[0], spec[1]
    _box_pts += [(cx + i * sx / 2, cy + j * sy / 2, cz + k * sz / 2) for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]
HAMMER_POINTS = np.array(_rim + _box_pts)

_STATE = {"marked": False, "anchor": 0.0}


def _nail_joint(env):
    import torch

    trim = env.unwrapped.scene["trim"]
    q = trim.data.joint_pos
    return trim, float(getattr(q, "torch", q)[0, 0]), torch


@on_reset
def _reset(env):
    """Nail fully driven in, nothing marked."""
    _STATE["marked"], _STATE["anchor"] = False, 0.0
    trim, _, torch = _nail_joint(env)
    zero = torch.zeros((1, 1), device=env.unwrapped.device)
    trim.write_joint_state_to_sim(zero, zero)
    trim.set_joint_position_target(zero)


@on_step
def _nail_friction(env):
    """Stick-slip friction of the nail in the wood. The nail hangs on a stiff spring (the wood's give) toward an anchor
    point; the anchor follows the nail only as far as needed to keep the spring force at the slip force. So the nail
    stays put against any pull below NAIL_FRICTION, and slides, against NAIL_FRICTION, under a harder pull."""
    trim, q, torch = _nail_joint(env)
    slip = NAIL_FRICTION / NAIL_GIVE if q < NAIL_OUT else 0.0  # out of the wood: nothing holds it
    _STATE["anchor"] = min(max(_STATE["anchor"], q - slip), q + slip)
    trim.set_joint_position_target(torch.full((1, 1), _STATE["anchor"], device=env.unwrapped.device))


def _rot(q_xyzw):
    x, y, z, w = q_xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def paint_touched(hammer) -> bool:
    """True if any part of the hammer is within 1.5 mm of the painted face, over the board."""
    pts = hammer.pos + HAMMER_POINTS @ _rot(hammer.quat_xyzw).T
    lo, hi = BOARD_CENTRE[:2] - np.array(BOARD[:2]) / 2, BOARD_CENTRE[:2] + np.array(BOARD[:2]) / 2
    over = np.all((pts[:, :2] > lo) & (pts[:, :2] < hi), axis=1)
    return bool(np.any(over & (pts[:, 2] < PAINT + 0.0015)))


def nail_lift(trim) -> float:
    return float(trim.sliders.get("nail", 0.0))


def block_on_board(block) -> bool:
    """The block lies flat on the painted face with its nearest edge within 2 cm of the nail."""
    flat = abs(block.pos[2] - (PAINT + BLOCK_SIZE[2] / 2)) < 0.004
    d = np.abs(_rot(block.quat_xyzw).T @ np.r_[NAIL_XY - block.pos[:2], 0.0])[:2] - np.array(BLOCK_SIZE[:2]) / 2
    return bool(flat and np.linalg.norm(np.maximum(d, 0.0)) < 0.02)


def score(objects):
    """0 once the hammer has touched the painted face. Otherwise 1.0 if the nail is out of the wood (raised 17 mm),
    0.5 if it is raised at least 5 mm, 0.25 if the scrap block lies on the board with an edge within 2 cm of the nail."""
    if paint_touched(objects["hammer"]):
        _STATE["marked"] = True
    if _STATE["marked"]:
        return 0.0
    lift = nail_lift(objects["trim"])
    if lift >= NAIL_OUT - 0.0005:
        return 1.0
    if lift >= 0.005:
        return 0.5
    return 0.25 if block_on_board(objects["block"]) else 0.0


def success(objects):
    return score(objects) == 1.0
