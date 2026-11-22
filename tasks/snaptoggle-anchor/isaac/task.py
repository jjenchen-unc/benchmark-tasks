"""Strap toggle anchor in 1/2 in drywall, for yam_arena (Isaac Lab-Arena / PhysX): push the anchor's folded steel
channel through the pre-drilled 1/2 in hole, pull the straps until the channel bears on the back of the board, keep
them taut while sliding the cap along them flush into the hole, then snap the straps off at the cap.

    inspect-robots run --embodiment yam_arena --policy <policy> -E task=task.py -E object_jitter=0 \\
        -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<task.json instruction>" \\
        --scorer task_score --max-steps 2400 --store-frames --log-dir <dir>

The bench is one custom-spawned PhysX assembly (``snapkit.py``). The scorer (``../src/scoring.py``) runs once per
control step on measured poses plus the per-physics-step contact monitor; ``score()`` returns its level at the end of
the trial (SCORE_OVER = "end"); the trial ends as soon as the full-score state has held for 1 s (inside the scorer).
Start states: a reset with a seed (inspect-robots always passes one) draws the wall mock-up's lateral position (hole
centre y uniform in +-40 mm), its stand-off (+-10 mm) and the anchor's place on its block (+-12 mm in x and y);
unseeded resets use the named setups in turn (``SNAP_SETUP`` picks the first and, when set, forces named setups).
The full-score reference policy is ``official_run.py`` (registered as ``snaptoggle_scripted``).
"""

import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "src"))
import layout as L  # noqa: E402
from snapkit import Bench, STATE  # noqa: E402  (installs the yam_arena hooks)

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the 1 s still-and-clear hold is inside the scorer (level 1.0 needs it)
INSTRUCTION = json.loads((HERE.parent / "task.json").read_text())["instruction"]

# One task-file object stands for the whole bench (pose = channel centre; size only for yam_arena's checks).
OBJECTS = [
    Bench(
        "bench",
        pos=(float(L.BLOCK_PIVOT_X), float(L.BLOCK_Y), float(L.TOP + L.PAD_TOP + 0.003)),
        size=(0.06, 0.01, 0.006),
        mass=None,
        color=(0.7, 0.7, 0.7),
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
