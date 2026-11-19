"""timing-belt for yam_arena (Isaac Lab-Arena / PhysX): a GT2 timing belt hangs on the 60T joint pulley of a robot arm
joint's 3:1 belt drive; fit it round the 20T motor pulley too, teeth meshed, tension it by sliding the motor plate
outward and clamp the plate with its lever.

    inspect-robots run --embodiment yam_arena --policy <policy> -E task=task.py -E object_jitter=0 \\
        -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<task.json instruction>" \\
        --scorer task_score --max-steps <run_limit> --store-frames --log-dir <dir>

The bench is one custom-spawned PhysX assembly (``beltkit.py``); geometry comes from ``../src/layout.py`` and the
scorer is ``../src/belt_scoring.py``. The scorer runs on measured poses and PhysX contact reports during physics
stepping; ``score()`` returns its value at the end of the trial (SCORE_OVER = "end").
The full-score reference policy is ``official_run.py`` (registered as ``timing_belt_scripted``).
"""
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from beltkit import STATE, Bench  # noqa: E402  (installs the yam_arena hooks)
import layout as L  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the 3 s hands-off hold is inside the scorer (level 1.0 needs it)
INSTRUCTION = json.loads((pathlib.Path(__file__).resolve().parent.parent / "task.json").read_text())["instruction"]

# One task-file object stands for the whole bench (pose = motor plate; size only for yam_arena's checks).
OBJECTS = [Bench("bench", pos=(L.PLATE_FRONT, L.motor_y(L.C_START), L.AXIS_Z), size=(.04, .04, .04), mass=None,
                 color=(.7, .72, .74))]


def score(states):
    return float(STATE["score"])


def success(states):
    return STATE["score"] >= 1.0


def distance(states):
    return 1.0 - float(STATE["score"])


def demo():
    """For ``--policy yam_scripted`` (open-loop waypoints): no plan, both arms hold READY: the do-nothing baseline
    (score 0). The full-score reference demo is closed-loop on measured belt-link poses (``demo.py``)."""
    return {}
