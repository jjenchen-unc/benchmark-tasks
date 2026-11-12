"""On the station table, in front of the arms: a small drawer cabinet with its drawer taken out.

The cabinet (wood, 40 cm wide, 26 cm deep, 17 cm tall, open at the top like a base cabinet before its worktop goes
on) stands with its empty opening facing the robot. Inside each side wall is a steel slide rail with a small front
roller at its front end. The drawer (39 cm wide front, 22 cm deep, 1.0 kg) stands on the table between the arms and
the cabinet, loaded with a 0.8 kg box. Each side of the drawer carries a steel runner; the rear half of each runner
has a hook plate underneath (orange) whose front end stops the drawer against the roller when it is pulled out.

The robot must put the drawer back into the cabinet on its slides, close it, and then pull it open again by about
5 cm.

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
from drawerkit import env, gripper_points, on_reset, on_step, part, rot  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
TOP = TABLE_TOP[SCENE]
CONTROL_HZ = 20

INSTRUCTION = ("Put the drawer back into the cabinet on its slides and close it. "
               "Then pull it open again by about 5 cm and leave it there.")

WOOD = (0.80, 0.66, 0.46)
WOOD_DARK = (0.55, 0.42, 0.28)
FRONT = (0.30, 0.42, 0.55)
STEEL = (0.62, 0.63, 0.65)
HOOK = (0.95, 0.50, 0.10)
LOAD = (0.20, 0.45, 0.80)
WOOD_MU, SLIDE_MU = 0.5, 0.05

# -- the cabinet (frame: origin on the table at the middle of the front face, x into the cabinet, z up) -------------
XF = 0.32                       # world x of the cabinet's front face
RAIL_TOP = 0.090                # slide rail top above the table
BUMP_H = 0.006                  # front roller standing above the rail
ROLLER_X = (0.002, 0.010)       # front roller, cabinet frame
CAB_BOXES = (
    ((0.129, 0.0, 0.006), (0.258, 0.400, 0.012), WOOD),                       # floor panel
    ((0.129, 0.191, 0.085), (0.258, 0.018, 0.170), WOOD),                     # side walls
    ((0.129, -0.191, 0.085), (0.258, 0.018, 0.170), WOOD),
    ((0.249, 0.0, 0.091), (0.018, 0.364, 0.158), WOOD),                       # back panel
    *(((0.1175, s * 0.171, RAIL_TOP - 0.006), (0.235, 0.022, 0.012), STEEL, SLIDE_MU) for s in (1, -1)),  # rails
    *(((sum(ROLLER_X) / 2, s * 0.171, RAIL_TOP + BUMP_H / 2), (ROLLER_X[1] - ROLLER_X[0], 0.022, BUMP_H), STEEL,
       SLIDE_MU) for s in (1, -1)),                                                                        # rollers
)
CABINET = part("cabinet", pos=(XF, 0.0, TOP), boxes=CAB_BOXES, mass=None, color=WOOD, friction=WOOD_MU)

# -- the drawer (frame: origin at the bottom of the front panel's outer face, middle; x into the drawer) ------------
DEPTH = 0.22
RUNNER_Z = (0.030, 0.042)       # runner bottom/top above the drawer bottom
HOOK_Z = (0.024, 0.030)         # hook plate bottom/top
HOOK_X = (0.103, 0.215)         # hook plate front (the hook) / back
FRONT_H = 0.100
WALL_H = 0.090
DRAWER_BOXES = (
    ((0.009, 0.0, FRONT_H / 2), (0.018, 0.390, FRONT_H), FRONT),              # front panel
    ((0.119, 0.0, 0.005), (0.202, 0.300, 0.010), WOOD_DARK),                  # bottom
    ((0.119, 0.144, WALL_H / 2), (0.202, 0.012, WALL_H), WOOD_DARK),          # sides
    ((0.119, -0.144, WALL_H / 2), (0.202, 0.012, WALL_H), WOOD_DARK),
    ((0.214, 0.0, WALL_H / 2), (0.012, 0.276, WALL_H), WOOD_DARK),            # back
    *(((0.1165, s * 0.161, 0.036), (0.197, 0.022, 0.012), STEEL, SLIDE_MU) for s in (1, -1)),   # runners
    *(((sum(HOOK_X) / 2, s * 0.161, sum(HOOK_Z) / 2), (HOOK_X[1] - HOOK_X[0], 0.022, HOOK_Z[1] - HOOK_Z[0]), HOOK,
       SLIDE_MU) for s in (1, -1)),                                                                       # hook plates
)
DRAWER_KG = 1.0
DRAWER_COM = (0.093, 0.0, 0.045)
SEATED_Z = RAIL_TOP - HOOK_Z[0]          # drawer bottom above the table when it rides on its slides (0.066)
CLOSED_X = XF - 0.018                     # front panel's outer face when closed (its back face on the cabinet)
STOP_OPEN = CLOSED_X - (XF + ROLLER_X[1] - HOOK_X[0])   # how far it opens before the hooks meet the rollers (75 mm)
START_X = 0.06                            # front panel's outer face at the start, drawer standing on the table
DRAWER = part("drawer", pos=(START_X, 0.0, TOP), boxes=DRAWER_BOXES, mass=DRAWER_KG, com=DRAWER_COM, color=FRONT,
              friction=0.8)

LOAD_SIZE = (0.12, 0.16, 0.06)
LOAD_X = 0.145                            # load centre, drawer frame (towards the back)
BOX = Box("parts_box", pos=(START_X + LOAD_X, 0.0, TOP + 0.010 + LOAD_SIZE[2] / 2), size=LOAD_SIZE, mass=0.8,
          color=LOAD)

OBJECTS = [CABINET, DRAWER, BOX]

# -- scoring -----------------------------------------------------------------------------------------------------------
HEIGHT_TOL = 0.004       # on its slides: drawer bottom within this of SEATED_Z
TILT_TOL = 2.5           # and level within this (deg)
CLOSED_TOL = 0.005       # closed: front within this of CLOSED_X
OPEN_RANGE = (0.03, 0.08)
CLEAR_M = 0.02           # hands off: every gripper point this far from the drawer and the box
LET_GO_S = 1.0           # closed and let go for this long
INNER = (0.018, 0.208, 0.138)   # drawer inside: x from, x to, half width
_S: dict = {}


@on_reset
def _reset_state(environment=None) -> None:
    _S.update(step=0, scored=None, last=(0.0, False), lost=False, closed_steps=0, closed_free=False)


@on_step
def _count_step(environment=None) -> None:
    _S["step"] += 1


_reset_state()


def _tilt_deg(quat_xyzw) -> float:
    x, y, _, _ = quat_xyzw
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def _box_dist(p, centre, size) -> float:
    d = np.abs(np.asarray(p) - np.asarray(centre)) - np.asarray(size) / 2
    return float(np.linalg.norm(np.maximum(d, 0.0)))


def _clear(drawer, box) -> bool:
    environment = env()
    if environment is None:
        return False
    Rd, Rb = rot(drawer.quat_xyzw), rot(box.quat_xyzw)
    for p in gripper_points(environment):
        local = Rd.T @ (p - drawer.pos)
        if any(_box_dist(local, c, s) < CLEAR_M for c, s, *_ in DRAWER_BOXES):
            return False
        if _box_dist(Rb.T @ (p - box.pos), (0, 0, 0), LOAD_SIZE) < CLEAR_M:
            return False
    return True


def status(objects) -> dict:
    drawer, box = objects["drawer"], objects["parts_box"]
    R = rot(drawer.quat_xyzw)
    tilt = _tilt_deg(drawer.quat_xyzw)
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    height = float(drawer.pos[2] - TOP)                    # drawer bottom above the table (front panel's foot)
    front = float(drawer.pos[0])
    hook_x = float((drawer.pos + R @ np.array([HOOK_X[0], 0.0, 0.0]))[0])
    on_slides = bool(abs(height - SEATED_Z) < HEIGHT_TOL and tilt < TILT_TOL and abs(drawer.pos[1]) < 0.012
                     and abs(yaw) < 3.0 and hook_x > XF + ROLLER_X[1] - 0.003)   # hooks behind the rollers or resting on them
    opened = CLOSED_X - front
    b = R.T @ (box.pos - drawer.pos)
    inside = bool(INNER[0] - 0.01 < b[0] < INNER[1] + 0.01 and abs(b[1]) < INNER[2] + 0.01 and b[2] < 0.10)
    _S["lost"] |= not inside or height < -0.05
    pitch = math.degrees(math.asin(float(np.clip(R[2, 0], -1, 1))))  # + : rear end up
    return dict(height=height, tilt=tilt, pitch=pitch, yaw=yaw, y=float(drawer.pos[1]), front=front, opened=opened, hook_x=hook_x, on_slides=on_slides,
                box_local=b, inside=inside, lost=_S["lost"], closed_free=_S["closed_free"])


def _update(objects) -> tuple[float, bool]:
    # success() and score() both run every control step; evaluate each step once.
    if _S["scored"] == _S["step"]:
        return _S["last"]
    _S["scored"] = _S["step"]
    st = status(objects)
    clear = _clear(objects["drawer"], objects["parts_box"])
    st["clear"] = clear
    closed = st["on_slides"] and abs(st["opened"]) < CLOSED_TOL and st["inside"] and clear
    _S["closed_steps"] = _S["closed_steps"] + 1 if closed else 0
    _S["closed_free"] |= _S["closed_steps"] >= LET_GO_S * CONTROL_HZ
    if st["lost"]:
        result = (0.0, False)
    elif (_S["closed_free"] and st["on_slides"] and OPEN_RANGE[0] <= st["opened"] <= OPEN_RANGE[1]
          and st["inside"] and clear):
        result = (1.0, True)
    elif _S["closed_free"] and st["on_slides"]:
        result = (0.5, False)
    elif st["on_slides"]:
        result = (0.25, False)
    else:
        result = (0.0, False)
    _S["last"] = result
    _S["status"] = st
    return result


def success(objects) -> bool:
    """Closed on its slides and let go for 1 s earlier; now on its slides, 3-8 cm open, box inside, hands off."""
    return _update(objects)[1]


def score(objects) -> float:
    """0 if the box ever leaves the drawer or the drawer falls off the table. Else 1.0 on success; 0.5 if the drawer
    was closed on its slides and let go for 1 s and is still on its slides; 0.25 if it is on its slides."""
    return _update(objects)[0]


# -- scripted demo (planned from the fixed start layout only) -----------------------------------------------------------
GRIP_X = 0.116           # side-wall grip, drawer frame (at the loaded drawer's centre of mass)
GRIP_Y = 0.144           # side wall centre
GRIP_Z = 0.075           # TCP above the drawer bottom: pad tips 28 mm down the 90 mm wall, above the runner
LIFT_Z = 0.094           # commanded drawer bottom while carried in; it hangs about 1 cm lower in the grip
ENTRY_OPEN = 0.060       # lowered onto the rails with the front this far from closed: hooks 15 mm past the rollers
FINGER_HALF = 0.010      # closed fingers' half thickness along x (yaw 0)
EASE = 0.4 * FINGER_OPEN # let go inside the cabinet with the fingers only partly open (the cabinet wall is close)


def _both(plan_left):
    """The right arm mirrors the left arm's plan in y (and in yaw)."""
    right = [Waypoint(w.duration, (w.pos[0], -w.pos[1], w.pos[2]), w.finger, w.label, -w.yaw) for w in plan_left]
    return {"left": plan_left, "right": right}


def demo():
    """Both arms grip the drawer's side walls at its centre of mass, lift it level, carry it into the opening with the
    hook plates above the front rollers until the hooks are past them, set it down on the rails and let go; then push
    it shut with closed fingers, back off, grip the top of the front panel and pull it 5 cm open.
    DEMO_VARIANT: nothing | floor (slide it in low, on the cabinet floor under the slides) | short (lift too little:
    the hooks meet the rollers) | insert_only | close_only | pull:<cm> (pull that far); combine with commas."""
    tokens = os.environ.get("DEMO_VARIANT", "").split(",")
    if "nothing" in tokens:
        return {}
    pull = 0.05
    for t in tokens:
        if t.startswith("pull:"):
            pull = float(t[5:]) / 100
    variant = next((t for t in tokens if t in ("floor", "short", "insert_only", "close_only")), "")
    lift = {"floor": 0.040, "short": 0.082}.get(variant, LIFT_Z)
    seat = {"floor": 0.010}.get(variant, SEATED_Z - 0.002)
    g0 = START_X + GRIP_X
    gin = CLOSED_X - ENTRY_OPEN + GRIP_X
    z = TOP + GRIP_Z
    plan = [
        Waypoint(2.0, (g0, GRIP_Y, z + 0.07), FINGER_OPEN, "above the side walls"),
        Waypoint(1.5, (g0, GRIP_Y, z), FINGER_OPEN, "around the side walls"),
        Waypoint(0.8, (g0, GRIP_Y, z), FINGER_CLOSED, "grip"),
        Waypoint(1.5, (g0, GRIP_Y, z + lift), FINGER_CLOSED, "lift level"),
        Waypoint(3.0, (gin, GRIP_Y, z + lift), FINGER_CLOSED, "carry it into the opening"),
        Waypoint(1.2, (gin, GRIP_Y, z + seat), FINGER_CLOSED, "set it on the rails"),
        Waypoint(0.8, (gin, GRIP_Y, z + seat), EASE, "let go"),
        Waypoint(1.2, (gin - 0.01, GRIP_Y, TOP + 0.18), EASE, "up"),
    ]
    if variant == "insert_only":
        plan.append(Waypoint(1.5, (0.20, 0.22, TOP + 0.20), FINGER_OPEN, "clear"))
        return _both(plan)
    front_now = CLOSED_X - ENTRY_OPEN
    pz = TOP + seat + 0.05
    plan += [
        Waypoint(1.5, (front_now - 0.04, 0.12, TOP + 0.18), FINGER_CLOSED, "in front of the drawer"),
        Waypoint(1.0, (front_now - 0.04, 0.12, pz), FINGER_CLOSED, "down"),
        Waypoint(2.0, (CLOSED_X - FINGER_HALF + 0.004, 0.12, pz), FINGER_CLOSED, "push it shut"),
        Waypoint(1.0, (CLOSED_X - 0.05, 0.12, pz), FINGER_CLOSED, "back off"),
        Waypoint(1.5, (CLOSED_X - 0.08, 0.16, TOP + 0.19), FINGER_OPEN, "hands off"),
    ]
    if variant == "close_only":
        plan.append(Waypoint(1.5, (0.20, 0.22, TOP + 0.20), FINGER_OPEN, "clear"))
        return _both(plan)
    # grip the top edge of the front panel (fingers closing along x) and pull
    px = CLOSED_X + 0.009
    top = TOP + seat + 0.002 + FRONT_H
    plan += [
        Waypoint(1.5, (px, 0.12, top + 0.02), FINGER_OPEN, "above the front panel", -math.pi / 2),
        Waypoint(1.2, (px, 0.12, top - 0.012), FINGER_OPEN, "around the panel's top edge", -math.pi / 2),
        Waypoint(0.8, (px, 0.12, top - 0.012), FINGER_CLOSED, "grip", -math.pi / 2),
        Waypoint(2.0, (px - pull, 0.12, top - 0.012), FINGER_CLOSED, "pull open", -math.pi / 2),
        Waypoint(0.8, (px - pull, 0.12, top - 0.012), FINGER_OPEN, "let go", -math.pi / 2),
        Waypoint(1.2, (px - pull - 0.02, 0.12, top + 0.02), FINGER_OPEN, "up", -math.pi / 2),
        Waypoint(1.5, (0.20, 0.22, TOP + 0.20), FINGER_OPEN, "clear"),
    ]
    return _both(plan)
