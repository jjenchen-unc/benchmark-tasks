"""Clothes dryer with a lint-laden lint screen, for yam_arena (Isaac Lab-Arena / PhysX): asked to start the dryer, the
robot pulls the lint screen out of its slot in the cabinet top, strips the lint mat off it, slides the screen back down
until its grip is home on the top, sweeps the lint into the bin, and only then presses Start.

    inspect-robots run --embodiment yam_arena --policy <policy> -E task=task.py -E object_jitter=0 \\
        -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<task.json instruction>" \\
        --scorer task_score --max-steps 2400 --store-frames --log-dir <dir>

The bench is one custom-spawned PhysX assembly (``dryerkit.py``). The scorer (``../src/scoring.py``) runs once per
control step on measured poses plus the per-physics-step Start-button force; ``score()`` returns its level at the end
of the trial (SCORE_OVER = "end"); the trial ends as soon as the full-score state has held for 1 s (inside the
scorer). Start states: a reset with a seed (inspect-robots always passes one) moves the dryer by up to 15 mm (x) and
30 mm (y) and puts a lint mat on the screen with probability 0.75; unseeded resets use the named setups in turn (see
task.json; DRYER_SETUP picks the first and, when set, forces named setups). The full-score reference policy is
``official_run.py`` (registered as ``dryer_lint_scripted``).
"""

import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "src"))
import layout as L  # noqa: E402
from dryerkit import Dryer, STATE  # noqa: E402  (installs the yam_arena hooks)

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the 1 s hold is inside the scorer (level 1.0 needs it)
INSTRUCTION = json.loads((pathlib.Path(__file__).resolve().parent.parent / "task.json").read_text())["instruction"]

# One task-file object stands for the whole bench (pose = the screen's seated pose; size only for yam_arena's checks).
OBJECTS = [
    Dryer(
        "dryer",
        pos=tuple(float(v) for v in L.screen_seated_pose(L.D0)[0]),
        size=(0.4, 0.1, 0.15),
        mass=None,
        color=(0.93, 0.93, 0.91),
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
