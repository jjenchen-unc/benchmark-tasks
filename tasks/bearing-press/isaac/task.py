"""Press a new SKF 61806-2RS1 bearing into an aluminium actuator output housing on a 1-ton bench arbor press, for
yam_arena (Isaac Lab-Arena / PhysX): start the bearing square in the bore, put the outer-ring driver on it, slide the
housing under the ram, and pull the press handle until the bearing seats on the bore's shoulder; then let go.

    inspect-robots run --embodiment yam_arena --policy <policy> -E task=task.py -E object_jitter=0 \\
        -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<task.json instruction>" \\
        --scorer task_score --max-steps 3000 --store-frames --log-dir <dir>

The bench is one custom-spawned PhysX assembly (``presskit.py``). The scorer (``../src/scoring.py``) runs once per
control step on measured poses plus the per-physics-step contact monitor; ``score()`` returns its level at the end of
the trial (SCORE_OVER = "end"); the trial ends as soon as the full-score state has held for 1 s (inside the scorer).
Start states: a reset with a seed (inspect-robots always passes one) draws the housing's place on the press table
(58..74 mm in front of the ram axis, +-12 mm sideways) and which tray slot holds which driver disc; unseeded resets use
the named setups in turn (``BP_SETUP`` picks the first and, when set, forces named setups). The full-score reference
policy is ``official_run.py`` (registered as ``bearing_press_scripted``).
"""

import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "src"))
import layout as L  # noqa: E402
from presskit import Bench, STATE  # noqa: E402  (installs the yam_arena hooks)

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the 1 s hold is inside the scorer (level 1.0 needs it)
INSTRUCTION = json.loads((pathlib.Path(__file__).resolve().parent.parent / "task.json").read_text())["instruction"]

# One task-file object stands for the whole bench (pose = ram axis on the press table; size only for yam_arena's checks).
OBJECTS = [
    Bench(
        "bench",
        pos=(float(L.RAM_XY[0]), float(L.RAM_XY[1]), float(L.PLATE_Z)),
        size=(0.3, 0.3, 0.1),
        mass=None,
        color=(0.2, 0.33, 0.3),
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
