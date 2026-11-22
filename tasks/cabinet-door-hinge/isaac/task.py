"""Hang a wall-cabinet door on its two clip-on hinges, for yam_arena (Isaac Lab-Arena / PhysX): a 19 mm MDF door with
two clip-on cup hinges stands in a grooved 2x4 offcut in front of a wall-cabinet carcass that has the two matching
mounting plates on its left side panel; hang the door (hook both hinge arms onto their plates, clip each one on) and
swing it shut.

    inspect-robots run --embodiment yam_arena --policy <policy> -E task=task.py -E object_jitter=0 \\
        -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<task.json instruction>" \\
        --scorer task_score --max-steps 2400 --store-frames --log-dir <dir>

The bench is one custom-spawned PhysX assembly (``hingekit.py``). The scorer (``../src/scoring.py``) runs once per
control step on measured poses, joint angles and the hinges' plate states; ``score()`` returns its level at the end
of the trial (SCORE_OVER = "end"); the trial ends as soon as the full-score state has held for 1 s (inside the
scorer). Start states: a reset with a seed (inspect-robots always passes one) draws the carcass position (+-15 mm /
+-10 mm) and the door's start pose in its groove block (90..115 mm in front of and 15..35 mm outboard of its hung
position, turned up to +-4 deg); unseeded resets use the five named setups in turn (CDH_SETUP picks the first and,
when set, forces named setups). The full-score reference policy is ``official_run.py`` (registered as
``cabinet_door_scripted``).
"""

import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "src"))
import layout as L  # noqa: E402
from hingekit import HingeBench, STATE  # noqa: E402  (installs the yam_arena hooks)

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the 1 s still-and-clear hold is inside the scorer (level 1.0 needs it)
INSTRUCTION = json.loads((HERE.parent / "task.json").read_text())["instruction"]

# One task-file object stands for the whole bench (pose = carcass front; size only for yam_arena's checks).
OBJECTS = [
    HingeBench(
        "hinge_bench",
        pos=(float(L.X_FRONT0 + L.CAB_D / 2), float(L.Y_LEFT0 - L.CAB_W / 2), float(L.Z_CAB + L.CAB_H / 2)),
        size=(0.3, 0.3, 0.4),
        mass=None,
        color=(0.5, 0.62, 0.7),
    )
]


def score(states):
    return float(STATE["score"])


def success(states):
    return STATE["score"] >= 1.0


def distance(states):
    return 1.0 - float(STATE["score"])


def demo():
    """For ``--policy yam_scripted`` (open-loop waypoints): no plan, both arms hold READY, so the official harness
    run is the do-nothing baseline (score 0). The full-score reference demo is closed-loop on measured poses
    (``demo.py``, same embodiment, robot commands only)."""
    return {}
