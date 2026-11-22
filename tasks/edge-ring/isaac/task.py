"""Edge-ring installation in a 300 mm plasma-etch chamber, for yam_arena (Isaac Lab-Arena / PhysX): set the new silicon
edge ring from its stand onto the chamber's raised lift pins, clocked to its three locating pins, lower it with the
maintenance pendant, and take the hands out of the chamber, never touching the chuck.

    inspect-robots run --embodiment yam_arena --policy <policy> -E task=task.py -E object_jitter=0 \\
        -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<task.json instruction>" \\
        --scorer task_score --max-steps 3000 --store-frames --log-dir <dir>

The bench is one custom-spawned PhysX assembly (``ringkit.py``). The scorer (``../src/scoring.py``) runs once per
control step on measured poses plus the per-physics-step contact monitor; ``score()`` returns its level at the end of
the trial (SCORE_OVER = "end"); the trial ends as soon as the full-score state has held for 1 s (inside the scorer).
Start states: a reset with a seed (inspect-robots always passes one) draws the ring's stand angle uniformly from
-35..-10 or +10..+22 deg (never within 10 deg of fitting) and moves the stand by up to 10 mm; unseeded resets use the
named setups -35, -20, -15, +15, +20 deg in turn (see task.json; EDGE_SETUP picks the first and, when set, forces named
setups). The full-score reference policy is ``official_run.py`` (registered as
``edge_ring_scripted``).
"""

import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "src"))
import layout as L  # noqa: E402
from ringkit import Chamber, STATE  # noqa: E402  (installs the yam_arena hooks)

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = (
    0.0  # the 1 s still-and-clear hold is inside the scorer (level 1.0 needs it)
)
INSTRUCTION = json.loads(
    (pathlib.Path(__file__).resolve().parent.parent / "task.json").read_text()
)["instruction"]

# One task-file object stands for the whole bench (pose = ring seat centre; size only for yam_arena's checks).
OBJECTS = [
    Chamber(
        "chamber",
        pos=tuple(float(v) for v in L.C),
        size=(0.5, 0.5, 0.1),
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
    run is the do-nothing baseline (score 0). The full-score reference demo is closed-loop on measured poses
    (``demo.py``, same embodiment, robot commands only)."""
    return {}
