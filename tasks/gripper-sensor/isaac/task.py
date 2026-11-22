"""Gripper-sensor commissioning, for yam_arena (Isaac Lab-Arena / PhysX): set a pneumatic parallel gripper's auto switch
so that it switches on the test part (LED on with the jaws closed on the part, off when open and off when closed
empty), working the valve's non-locking manual override with one hand while the other slides the switch along its
groove, then lock the switch's set screw with the screwdriver.

    inspect-robots run --embodiment yam_arena --policy <policy> -E task=task.py -E object_jitter=0 \\
        -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<task.json instruction>" \\
        --scorer task_score --max-steps 2400 --store-frames --log-dir <dir>

The bench is one custom-spawned PhysX assembly (``grippkit.py``). The scorer (``../src/scoring.py``) runs once per
control step; ``score()`` returns its level at the end of the trial (SCORE_OVER = "end"); the trial ends as soon as
the full-score state has held for 1 s (inside the scorer). Start states: a reset with a seed (inspect-robots always
passes one) draws the test part's width (35 to 55 mm, i.e. the per-jaw travel to the part 2.5 to 12.5 mm) and the
switch's start position in its groove (at least 3.5 mm from the part's switching point); unseeded resets use the five
named setups in turn (GS_SETUP picks the first and, when set, forces named setups). The full-score reference policy
is ``official_run.py`` (registered as ``gripper_sensor_scripted``).
"""

import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "src"))
import layout as L  # noqa: E402
from grippkit import Bench, STATE  # noqa: E402  (installs the yam_arena hooks)

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the 1 s clear hold is inside the scorer (level 1.0 needs it)
INSTRUCTION = json.loads((HERE.parent / "task.json").read_text())["instruction"]

# One task-file object stands for the whole bench (pose = the switch; size only for yam_arena's checks).
OBJECTS = [
    Bench(
        "bench",
        pos=(float(L.BODY_C[0]), float(L.BODY_C[1]), float(L.BODY_TOP)),
        size=(0.3, 0.3, 0.1),
        mass=None,
        color=(0.6, 0.6, 0.65),
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
    run is the do-nothing baseline (score 0). The full-score reference demo is closed-loop on the observable state
    (``demo.py``, same embodiment, robot commands only)."""
    return {}
