"""Taking a single-slot workstation GPU out of its PCIe x16 slot, for yam_arena (Isaac Lab-Arena / PhysX): with its
bracket screw already out, hold the slot's retention latch pressed open with one hand while the other draws the card
straight up out of the slot, then lay the card on the anti-static mat and let go.

    inspect-robots run --embodiment yam_arena --policy <policy> -E task=task.py -E object_jitter=0 \\
        -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<task.json instruction>" \\
        --scorer task_score --max-steps 1500 --store-frames --log-dir <dir>

The bench is one custom-spawned PhysX assembly (``gpukit.py``). The scorer (``../src/scoring.py``) runs once per
control step on measured poses, the latch-hook contact force and the fingertip positions; ``score()`` returns its level
at the end of the trial (SCORE_OVER = "end"); the trial ends as soon as the card has lain on the mat, released, for
3 s (score 1.0). Start states: a reset with a seed (inspect-robots always passes one) moves the benchtable by up to
25 mm across the card and 60 mm along it and turns it by up to 5 deg, redrawn until clear of the replay guard (layout.setup_spec); unseeded resets use the named
setups 0-3 in turn (GPU_SETUP picks the first and, when set, forces named setups). The full-score reference policy is
``official_run.py`` (registered as ``gpu_latch_scripted``).
"""

import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "src"))
import layout as L  # noqa: E402
from gpukit import Rig, STATE  # noqa: E402  (installs the yam_arena hooks)

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the 3 s hold is inside the scorer
INSTRUCTION = json.loads((HERE.parent / "task.json").read_text())["instruction"]

# One task-file object stands for the whole bench (pose = the slot frame; size only for yam_arena's checks).
OBJECTS = [
    Rig(
        "rig",
        pos=(float(L.BENCH_XY[0]), float(L.BENCH_XY[1]), float(L.BOARD_Z)),
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
