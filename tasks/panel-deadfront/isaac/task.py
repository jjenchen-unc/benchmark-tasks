"""Load-centre dead-front refit, for yam_arena (Isaac Lab-Arena / PhysX): take the dead-front cover from its bench
stand, hang its lower slots on the two started screws of a wall-mounted 6-space load centre, swing it shut so that
every breaker's handle shroud passes through its window, hold it flat and drive both screws down with the ratcheting
nut driver from its rest, then let go.

    inspect-robots run --embodiment yam_arena --policy <policy> -E task=task.py -E object_jitter=0 \\
        -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<task.json instruction>" \\
        --scorer task_score --max-steps 3600 --store-frames --log-dir <dir>

The bench is one custom-spawned PhysX assembly (``panelkit.py``). The scorer (``../src/scoring.py``) runs once per
control step on measured poses plus the per-physics-step contact monitor and screw model; ``score()`` returns its
level at the end of the trial (SCORE_OVER = "end"); the trial ends as soon as the full-score state has held for 1 s
(inside the scorer). Start states: a reset with a seed (inspect-robots always passes one) draws the stand position,
the cover's place along the stand's groove, the panel's position on the wall stub and the six handles' ON / OFF
states from the seed (``layout.setup_spec``); unseeded resets use the named setups 0, 1, 2 in turn (PANEL_SETUP picks
the first and, when set, forces named setups). The full-score reference policy is ``official_run.py`` (registered as
``panel_deadfront_scripted``).
"""

import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "src"))
import layout as L  # noqa: E402
from panelkit import Panel, STATE  # noqa: E402  (installs the yam_arena hooks)

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the 1 s still-and-clear hold is inside the scorer (level 1.0 needs it)
INSTRUCTION = json.loads((HERE.parent / "task.json").read_text())["instruction"]

# One task-file object stands for the whole bench (pose = the cover's start position; size only for yam_arena's checks).
OBJECTS = [Panel("panel", pos=tuple(float(v) for v in L.STAND + [0.0, 0.0, 0.17]), size=(0.3, 0.3, 0.3), mass=None,
                 color=(0.6, 0.6, 0.6))]


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
