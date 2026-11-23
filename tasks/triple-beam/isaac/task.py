"""Societal-relatability task for yam_arena (Isaac Lab-Arena / PhysX): weigh a sealed sample jar of unknown mass on an
Ohaus 750-SO triple-beam balance (610 g capacity, 0.1 g readability). The robot sets the 100 g and 10 g riders in
their notches and slides the 10 g poise until the free, magnetically damped beam comes to rest with its pointer on the
zero mark, hands off.

    inspect-robots run --embodiment yam_arena --policy <policy> -E task=task.py -E object_jitter=0 \\
        -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<task.json instruction>" \\
        --scorer task_score --max-steps 2400 --store-frames --log-dir <dir>

The bench is custom-spawned PhysX bodies (``balkit.py``: the balance as one fixed-base articulation: beam on the
fulcrum, level pan on its bearing, riders on their beams; the jar a free body). The scorer (``../src/scoring.py``)
runs once per control step on the beam's angle and speed, the riders' positions, the jar's pose and the robot-on-part
contact forces; ``score()`` returns its level at the end of the trial; the trial ends as soon as the full-score state
is reached (inside the scorer). Start states: a reset with a seed (inspect-robots always passes one) draws the jar's
mass (120-480 g, 1-9 g past a 10 g step), moves the balance up to 8 mm and turns it up to 3 deg, and stands the jar
anywhere within 22 mm of the pan centre (layout.setup_spec); unseeded resets use the named setups in turn (TB_SETUP
picks the first and, when set, forces named setups). The full-score reference policy is ``official_run.py``
(registered as ``triple_beam_scripted``).
"""

import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "src"))
import layout as L  # noqa: E402
from balkit import Bench, STATE  # noqa: E402  (installs the yam_arena hooks)

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the at-rest and hands-off holds are inside the scorer (level 1.0 needs them)
INSTRUCTION = json.loads((HERE.parent / "task.json").read_text())["instruction"]

# One task-file object stands for the whole bench (pose = fulcrum; size only for yam_arena's checks).
OBJECTS = [Bench("bench", pos=(L.NOMINAL[0], L.NOMINAL[1], L.TOP + 0.05), size=(0.1, 0.1, 0.1), mass=None,
                 color=(0.9, 0.9, 0.9))]


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
