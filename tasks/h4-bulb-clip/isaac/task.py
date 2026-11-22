"""Change an H4 headlamp bulb from behind the lamp, for yam_arena (Isaac Lab-Arena / PhysX): take the new bulb from its
holder by its cap, turn it so its three unequal tabs drop into the notches of the lamp's bulb boss, seat it, and hook
the wire spring clip over it under its catch, without touching the glass.

    inspect-robots run --embodiment yam_arena --policy <policy> -E task=task.py -E object_jitter=0 \\
        -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<task.json instruction>" \\
        --scorer task_score --max-steps 2400 --store-frames --log-dir <dir>

The bench is one custom-spawned PhysX assembly (``h4kit.py``). The scorer (``../src/scoring.py``) runs once per
control step on measured poses plus the per-physics-step contact monitor; ``score()`` returns its level at the end of
the trial (SCORE_OVER = "end"); the trial ends as soon as the full-score state has held for 1 s (inside the scorer).
Start states: a reset with a seed (inspect-robots always passes one) draws the bulb's roll in its holder uniformly
over 360 deg, the clip's open angle from 95 to 105 deg and moves the holder by up to 10 mm; unseeded resets use the
named setups in turn (see task.json; H4_SETUP picks the first and, when set, forces named setups). The full-score
reference policy is ``official_run.py`` (registered as ``h4_bulb_scripted``).
"""

import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "src"))
import layout as L  # noqa: E402
from h4kit import Bench, STATE  # noqa: E402  (installs the yam_arena hooks)

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the 1 s still-and-clear hold is inside the scorer (level 1.0 needs it)
INSTRUCTION = json.loads((HERE.parent / "task.json").read_text())["instruction"]

# One task-file object stands for the whole bench (pose = seat centre; size only for yam_arena's checks).
OBJECTS = [
    Bench(
        "bench",
        pos=tuple(float(v) for v in L.H),
        size=(0.2, 0.2, 0.1),
        mass=None,
        color=(0.4, 0.4, 0.45),
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
