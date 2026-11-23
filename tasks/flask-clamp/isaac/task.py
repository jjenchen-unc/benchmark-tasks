"""Loading a 250 mL culture flask into an orbital shaker's spring-finger clamp, for yam_arena (Isaac Lab-Arena /
PhysX): take the empty Erlenmeyer flask from the bench and seat it in the free clamp on the shaker platform, between
the loaded culture flasks, with its bottom down on the clamp's base plate and all five fingers closed round its body,
then let go so that it stays put when the shaker runs.

    inspect-robots run --embodiment yam_arena --policy <policy> -E task=task.py -E object_jitter=0 \\
        -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<task.json instruction>" \\
        --scorer task_score --max-steps 1800 --store-frames --log-dir <dir>

The bench is one custom-spawned PhysX assembly (``flaskkit.py``). The scorer (``../src/scoring.py``) runs once per
control step on measured poses and the finger-flask contacts; ``score()`` returns its level at the end of the trial
(SCORE_OVER = "end"). Once the flask is seated and released, the scorer runs the shake test (the platform on its
19 mm orbit at 200 rpm for 2.5 s); the trial ends as soon as the flask has stayed seated through it (score 1.0).
Start states: a reset with a seed (inspect-robots always passes one) draws which clamp is free (one of four), moves
the shaker by up to 8 mm and places the flask on the bench beside the shaker on the free clamp's side (x 30-46 cm,
|y| 24-30 cm); unseeded resets use the named setups 0-3 in turn (layout.SETUPS; FLASK_SETUP picks the first and, when
set, forces named setups). The full-score reference policy is ``official_run.py`` (registered as
``flask_clamp_scripted``).
"""

import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "src"))
import layout as L  # noqa: E402
from flaskkit import Bench, STATE  # noqa: E402  (installs the yam_arena hooks)

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the shake test is inside the scorer (level 1.0 needs it)
INSTRUCTION = json.loads((HERE.parent / "task.json").read_text())["instruction"]

# One task-file object stands for the whole bench (pose = the shaker platform; size only for yam_arena's checks).
OBJECTS = [
    Bench(
        "bench",
        pos=(float(L.SHAKER_XY[0]), float(L.SHAKER_XY[1]), float(L.PLATFORM_Z)),
        size=(0.3, 0.3, 0.1),
        mass=None,
        color=(0.6, 0.6, 0.6),
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
