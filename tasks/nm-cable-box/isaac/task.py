"""NM cable into a nail-on outlet box through its push-in clamp, for yam_arena (Isaac Lab-Arena / PhysX): feed the
cut end of a 14/2 NM-B cable through the box's push-in cable clamp tab and draw it in until about 150 mm of cable is
inside, then let go.

    inspect-robots run --embodiment yam_arena --policy <policy> -E task=task.py -E object_jitter=0 \\
        -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<task.json instruction>" \\
        --scorer task_score --max-steps 3600 --store-frames --log-dir <dir>

The bench is one custom-spawned PhysX assembly (``cablekit.py``). The scorer (``../src/scoring.py``) runs once per
control step on measured poses; ``score()`` returns its level at the end of the trial (SCORE_OVER = "end"); the trial
ends as soon as the full-score state has held for 1 s (inside the scorer). Start states: a reset with a seed
(inspect-robots always passes one) moves the box (8-20 mm in x, up to 30 mm in y) and lays the cable at a seeded
place and angle; unseeded resets use the named setups 0-4 in turn (layout.SETUPS; NMB_SETUP picks the first and, when
set, forces named setups). The full-score reference policy is ``official_run.py`` (registered as
``nm_cable_box_scripted``).
"""

import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "src"))
import layout as L  # noqa: E402
from cablekit import Bench, STATE  # noqa: E402  (installs the yam_arena hooks)

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the 1 s still-and-released hold is inside the scorer (level 1.0 needs it)
INSTRUCTION = json.loads((HERE.parent / "task.json").read_text())["instruction"]

# One task-file object stands for the whole bench (pose = box entry; size only for yam_arena's checks).
OBJECTS = [
    Bench(
        "bench",
        pos=tuple(float(v) for v in L.entry_centre(L.setup_spec(0))),
        size=(0.1, 0.1, 0.07),
        mass=None,
        color=(0.1, 0.3, 0.75),
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
