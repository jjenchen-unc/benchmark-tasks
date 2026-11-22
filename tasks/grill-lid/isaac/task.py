"""Light a Weber Q 1200 tabletop gas grill whose lid is closed, for yam_arena (Isaac Lab-Arena / PhysX): open the lid
past its balance point by the front handle, push in and turn the burner knob to START/HI, and press the igniter
within 5 s so that the burner lights; then take both hands off the grill.

    inspect-robots run --embodiment yam_arena --policy <policy> -E task=task.py -E object_jitter=0 \\
        -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<task.json instruction>" \\
        --scorer task_score --max-steps 2000 --store-frames --log-dir <dir>

The grill is one custom-spawned PhysX articulation (``grillkit.py``). The scorer (``../src/scoring.py``) runs once per
control step on the joint state and the robot-on-grill contact forces; ``score()`` returns its level at the end of the
trial (SCORE_OVER = "end"); the trial ends as soon as the full-score state (burner lit, hands off for 1 s) is reached
(inside the scorer). Start states: a reset with a seed (inspect-robots always passes one) moves the grill by up to
4 cm in x, 4.5 cm in y and turns it by up to 12 deg (layout.setup_spec); unseeded resets use the named setups in turn
(GRILL_SETUP picks the first and, when set, forces named setups). Lid closed, knob OFF at every start. The full-score
reference policy is ``official_run.py`` (registered as ``grill_lid_scripted``).
"""

import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "src"))
import layout as L  # noqa: E402
from grillkit import Grill, STATE  # noqa: E402  (installs the yam_arena hooks)

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the 1 s hands-off hold is inside the scorer (level 1.0 needs it)
INSTRUCTION = json.loads((HERE.parent / "task.json").read_text())["instruction"]

# One task-file object stands for the whole grill (pose = grill origin; size only for yam_arena's checks).
OBJECTS = [Grill("grill", pos=(L.NOMINAL[0], L.NOMINAL[1], L.TOP + 0.2), size=(0.4, 0.5, 0.36), mass=None,
                 color=(0.1, 0.1, 0.1))]


def score(states):
    return float(STATE["score"])


def success(states):
    return STATE["score"] >= 1.0


def distance(states):
    return 1.0 - float(STATE["score"])


def demo():
    """For ``--policy yam_scripted`` (open-loop waypoints): no plan, both arms hold READY, so the official harness
    run is the do-nothing baseline (score 0). The full-score reference demo is closed-loop on measured state
    (``demo.py``, same embodiment, robot commands only)."""
    return {}
