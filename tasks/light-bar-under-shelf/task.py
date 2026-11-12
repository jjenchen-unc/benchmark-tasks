"""A 21.5-inch desktop monitor stands pushed back under a free-standing pine desk shelf, with 4 cm between the top
edge of its screen and the underside of the shelf. A monitor light bar (a 45 cm bar on a clip with a counterweight
that hangs behind the screen) lies on its back on the desk in front. The robot must fit the light bar on the top edge
of the monitor and leave the monitor where it started.

The clip cannot be put over the edge while the monitor is under the shelf: its counterweight hangs 6.5 cm below the
top of the clip, and there are only 4 cm. The monitor must be slid forward out from under the shelf, the bar hung on
it, and the monitor slid back (the fitted bar adds 2.8 cm, so it passes under the shelf).

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score ...
Demo:    python demo.py task.py demo_out
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from barkit import finger_points, on_reset, on_step, part  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = "Fit the light bar on top of the monitor and leave the monitor where it started."

BLACK = (0.05, 0.05, 0.06)
GLASS = (0.10, 0.12, 0.16)
DARK = (0.16, 0.16, 0.17)
STEEL = (0.55, 0.56, 0.58)
PINE = (0.80, 0.64, 0.42)

# -- the monitor (21.5-inch, 50.4 x 38.4 x 17.9 cm on its stand, 2.67 kg) ---------------------------------------------
# Part frame: origin on the desk under the middle of the screen's front face, x back (away from the robot), y left.
SCREEN_W, SCREEN_T = 0.504, 0.016      # width; thickness of the panel at its top edge
SCREEN_BOTTOM, SCREEN_TOP = 0.089, 0.384
MONITOR_BOXES = (
    ((0.0395, 0.0, 0.0075), (0.179, 0.180, 0.015), BLACK),                       # stand base
    ((0.045, 0.0, 0.110), (0.020, 0.060, 0.190), BLACK),                         # stand neck
    ((SCREEN_T / 2, 0.0, (SCREEN_BOTTOM + SCREEN_TOP) / 2), (SCREEN_T, SCREEN_W, SCREEN_TOP - SCREEN_BOTTOM), BLACK),
    ((0.0325, 0.0, 0.200), (0.033, 0.300, 0.200), BLACK),                        # rear housing
    ((-0.0005, 0.0, 0.2415), (0.001, 0.476, 0.268), GLASS),                      # the picture area
)
MONITOR_MASS = 2.67
MONITOR_LO = np.array([-0.050, -SCREEN_W / 2, 0.0])
MONITOR_HI = np.array([0.129, SCREEN_W / 2, SCREEN_TOP])

# -- the light bar (45 cm, 0.53 kg) -------------------------------------------------------------------------------------
# Frame of the fitted bar: origin under the middle of the clip's top plate, level with the plate's underside, on the
# line of the screen's front face; x back, y along the bar, z up.
BAR_LEN, BAR_R = 0.45, 0.011
BAR_C = (-0.020, 0.0, 0.017)           # the bar's axis, in front of and above the edge
CLIP_W = 0.045
LIP_IN, ARM_IN = -0.001, 0.020         # inner faces of the front lip and of the counterweight (2.1 cm apart)
ARM_DROP = 0.065                       # the counterweight hangs this far below the top plate
BAR_BOXES = (
    ((0.0125, 0.0, 0.003), (0.075, CLIP_W, 0.006), DARK),                        # clip top plate
    ((LIP_IN - 0.003, 0.0, -0.006), (0.006, CLIP_W, 0.012), DARK),               # front lip
    ((ARM_IN + 0.015, 0.0, -ARM_DROP / 2), (0.030, CLIP_W, ARM_DROP), DARK),     # counterweight
)
BAR_CYLS = ((BAR_C, BAR_R, BAR_LEN, STEEL, "Y"),)
BAR_MASS = 0.53
# 0.25 kg in the bar, 0.28 kg in the clip and its counterweight: the centre of mass sits over the screen's edge,
# 9 mm behind its front face, so the fitted bar rests level.
BAR_COM = (0.009, 0.0, -0.008)
BAR_LO = np.array([-0.031, -BAR_LEN / 2, -ARM_DROP])
BAR_HI = np.array([0.050, BAR_LEN / 2, 0.028])

# -- the shelf: pine board 120 x 25 x 1.8 cm on two end panels, 42.4 cm clear underneath, 4.6 kg ----------------------
SHELF_FRONT, SHELF_D, SHELF_W, BOARD_T, WOOD_T = 0.36, 0.25, 1.20, 0.018, 0.018
SHELF_CLEAR = SCREEN_TOP + 0.040
SHELF_BOXES = (
    ((SHELF_D / 2, 0.0, SHELF_CLEAR + BOARD_T / 2), (SHELF_D, SHELF_W, BOARD_T), PINE),
    ((SHELF_D / 2, SHELF_W / 2 - WOOD_T / 2, SHELF_CLEAR / 2), (SHELF_D, WOOD_T, SHELF_CLEAR), PINE),
    ((SHELF_D / 2, -SHELF_W / 2 + WOOD_T / 2, SHELF_CLEAR / 2), (SHELF_D, WOOD_T, SHELF_CLEAR), PINE),
)

# -- fixed layout (world: x forward from the arms, y left, metres) ------------------------------------------------------
MONITOR_X = 0.42                       # screen face 6 cm behind the shelf's front edge
BAR_START = (0.123, 0.0)               # the bar lies on its back: counterweight flat on the desk, bar 7 cm up
BAR_TILT = math.pi / 2                 # turned 90 degrees about its length from the fitted pose

MONITOR = part("monitor", pos=(MONITOR_X, 0.0, TOP), boxes=MONITOR_BOXES, mass=MONITOR_MASS, color=BLACK,
               friction=0.5)
SHELF = part("shelf", pos=(SHELF_FRONT, 0.0, TOP), boxes=SHELF_BOXES, mass=4.6, color=PINE, friction=0.5)
LIGHT_BAR = part("light_bar", pos=(*BAR_START, TOP + ARM_IN + 0.030), boxes=BAR_BOXES, cylinders=BAR_CYLS,
                 mass=BAR_MASS, color=DARK, friction=0.6, tilt=BAR_TILT, com=BAR_COM)
OBJECTS = [SHELF, MONITOR, LIGHT_BAR]

# -- scoring -------------------------------------------------------------------------------------------------------------
TIP_LIMIT = math.radians(20)           # the monitor tipped over
SHELF_LIMIT = 0.010                    # the shelf was knocked more than 1 cm
BACK_LIMIT = 0.020                     # monitor back within 2 cm of its start ...
BACK_YAW = math.radians(5)             # ... and turned less than 5 degrees
CENTRE_LIMIT = 0.030                   # clip within 3 cm of the middle of the top edge
LEVEL_LIMIT = math.radians(5)          # bar level within 5 degrees
ROLL_LIMIT = math.radians(15)          # clip upright on the edge within 15 degrees
HANDS_OFF_BAR, HANDS_OFF_MONITOR = 0.03, 0.02
STILL = 0.001
HOLD_STEPS = 20                        # 1 s at 20 Hz

_STATE: dict = {}


def _reset_state(environment=None) -> None:
    _STATE.update(step=0, mon0=None, shelf0=None, failed="", released=False, finger_check=None, recent=[],
                  result=(0.0, False), info={})


_reset_state()
on_reset(_reset_state)


def _yaw(r: np.ndarray) -> float:
    return math.atan2(r[1, 0], r[0, 0])


def _box_gap(p, lo, hi) -> float:
    return float(np.linalg.norm(np.maximum(np.maximum(lo - p, p - hi), 0.0)))


def seated(mon, bar) -> tuple[bool, dict]:
    """The clip sits on the middle of the screen's top edge, lip in front and counterweight behind, bar level."""
    rel = mon.to_local(bar.pos)
    rr = mon.rotation().T @ bar.rotation()
    level = math.asin(min(1.0, abs(rr[2, 1])))                  # the bar's length against the horizontal
    roll = math.acos(max(-1.0, min(1.0, rr[2, 2])))             # the clip's up axis against the monitor's
    ok = bool(-0.006 < rel[0] < 0.003 and SCREEN_TOP - 0.003 < rel[2] < SCREEN_TOP + 0.006
              and abs(rel[1]) < CENTRE_LIMIT and level < LEVEL_LIMIT and roll < ROLL_LIMIT
              and abs(rr[1, 1]) > 0.9)                             # bar along the screen, not across it
    return ok, {"rel_mm": np.round(rel * 1000, 1).tolist(), "level_deg": round(math.degrees(level), 1),
                "roll_deg": round(math.degrees(roll), 1)}


@on_step
def _track(environment) -> None:
    from yam_arena.scene import object_states

    s = object_states(environment, ["monitor", "shelf", "light_bar"])
    mon, shelf, bar = s["monitor"], s["shelf"], s["light_bar"]
    _STATE["step"] += 1
    if _STATE["mon0"] is None:
        _STATE["mon0"] = (mon.pos.copy(), _yaw(mon.rotation()))
        _STATE["shelf0"] = shelf.pos.copy()
    # Failures are permanent: a monitor that tipped or a shelf that was knocked stays failed even if put back.
    if mon.rotation()[2, 2] < math.cos(TIP_LIMIT):
        _STATE["failed"] = _STATE["failed"] or "monitor tipped"
    if np.linalg.norm(shelf.pos - _STATE["shelf0"]) > SHELF_LIMIT:
        _STATE["failed"] = _STATE["failed"] or "shelf moved"
    # Hands off: every finger pad of both arms at least 3 cm from the light bar and 2 cm from the monitor.
    try:
        pts = finger_points(environment)
        gb = min(_box_gap(bar.to_local(p), BAR_LO, BAR_HI) for p in pts)
        gm = min(_box_gap(mon.to_local(p), MONITOR_LO, MONITOR_HI) for p in pts)
        _STATE["released"] = bool(gb >= HANDS_OFF_BAR and gm >= HANDS_OFF_MONITOR)
        _STATE["finger_check"] = True
    except Exception as exc:  # noqa: BLE001 - an unknown grip never counts as released
        if _STATE["finger_check"] is None:
            print(f"light-bar-under-shelf: finger check unavailable: {exc!r}", flush=True)
        _STATE["finger_check"], _STATE["released"] = False, False
    on_edge, info = seated(mon, bar)
    p0, yaw0 = _STATE["mon0"]
    moved = float(np.linalg.norm((mon.pos - p0)[:2]))
    turned = abs((_yaw(mon.rotation()) - yaw0 + math.pi) % (2 * math.pi) - math.pi)
    back = moved < BACK_LIMIT and turned < BACK_YAW
    recent = _STATE["recent"]
    recent.append((bar.pos.copy(), mon.pos.copy(), _STATE["released"]))
    del recent[:-HOLD_STEPS]
    still = len(recent) == HOLD_STEPS and all(
        np.linalg.norm(b - bar.pos) < STILL and np.linalg.norm(m - mon.pos) < STILL and free for b, m, free in recent)
    if _STATE["failed"]:
        result = (0.0, False)
    elif on_edge and still and back:
        result = (1.0, True)
    elif on_edge and still:
        result = (0.5, False)
    else:
        result = (0.0, False)
    _STATE["result"] = result
    _STATE["info"] = dict(info, seated=on_edge, released=_STATE["released"], still=still,
                          monitor_moved_mm=round(moved * 1000, 1), monitor_turned_deg=round(math.degrees(turned), 1),
                          shelf_moved_mm=round(float(np.linalg.norm(shelf.pos - _STATE["shelf0"])) * 1000, 1),
                          failed=_STATE["failed"])
    if _STATE["step"] % 100 == 0:
        print(f"light-bar-under-shelf: step {_STATE['step']} score {result[0]} {_STATE['info']}", flush=True)


def success(objects) -> bool:
    """The light bar sits on the middle of the monitor's top edge (counterweight behind, level), the monitor is back
    within 2 cm and 5 degrees of its start, the monitor never tipped and the shelf never moved, both hands are off and
    nothing moves over the last second; held 1 s."""
    return _STATE["result"][1]


def score(objects) -> float:
    """1.0 as success; 0.5 the bar sits on the edge, hands off and still, but the monitor is not back; 0 otherwise, and
    0 whenever the monitor tipped more than 20 degrees or the shelf moved more than 1 cm."""
    return _STATE["result"][0]


def demo():
    """Not the demo (the demo is demo.py, which needs tilted grips): both arms hold their ready pose."""
    return {}
