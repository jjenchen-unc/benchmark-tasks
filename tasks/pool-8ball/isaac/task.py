"""8-ball on a 7 ft bar-box pool table, for yam_arena (Isaac Lab-Arena / PhysX): pick up the house cue from its rest,
set it in a bridge with one hand, aim at the ghost-ball line and strike the cue ball once with the other hand so the
8-ball drops in the pocket marked with the coin, without the cue ball dropping (no scratch).

    inspect-robots run --embodiment yam_arena --policy <policy> -E task=task.py -E object_jitter=0 \\
        -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<task.json instruction>" \\
        --scorer task_score --max-steps 1500 --store-frames --log-dir <dir>

The table, balls and cue are one custom-spawned PhysX assembly (``poolkit.py``). The scorer (``../src/scoring.py``)
runs once per control step on the measured ball poses plus the per-physics-step contact monitor; ``score()`` returns
its level at the end of the trial (SCORE_OVER = "end"); the trial ends once the full-score state has held for 1 s
(inside the scorer). Start states: a reset with a seed (inspect-robots always passes one) draws the called pocket (left
or right side pocket), the 8-ball position, the cut angle (6-30 deg, either side) and the cue-ball distance (0.15-0.45
m) from the seed (``layout.setup_spec``); unseeded resets use the named setups in turn (POOL_SETUP picks the first and,
when set, forces named setups). The full-score reference policy is ``official_run.py`` (registered as
``pool_8ball_scripted``).
"""

import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "src"))
import layout as L  # noqa: E402
from poolkit import PoolTable, STATE  # noqa: E402  (installs the yam_arena hooks)

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the 1 s at-rest hold is inside the scorer (level 1.0 needs it)
INSTRUCTION = json.loads((HERE.parent / "task.json").read_text())["instruction"]

# One task-file object stands for the whole table (pose = table centre; size only for yam_arena's checks).
OBJECTS = [
    PoolTable(
        "pool",
        pos=(float((L.X0 + L.XF) / 2), 0.0, float(L.BED_Z)),
        size=(0.5, 0.5, 0.1),
        mass=None,
        color=(0.05, 0.38, 0.22),
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
