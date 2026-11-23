"""Hang a parts bin on a louvered panel, for yam_arena (Isaac Lab-Arena / PhysX): a steel louvered panel (Akro-Mils
30636 class, 36 x 19 in, six louver rows) stands at the back of the bench on a plywood board; one AkroBin (Akro-Mils
30230) already hangs on one of its rows, in one half of the panel; an identical bin stands on the bench in front. The
loose bin must be hung by the hanger lip at the top of its back on the SAME louver row, in the panel's empty half, and
let go.

    inspect-robots run --embodiment yam_arena --policy <policy> -E task=task.py -E object_jitter=0 \\
        -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<task.json instruction>" \\
        --scorer task_score --max-steps 1600 --store-frames --log-dir <dir>

The bench is one custom-spawned PhysX assembly (``louverkit.py``). The scorer (``../src/scoring.py``) runs once per
control step on measured poses; ``score()`` returns its level at the end of the trial (SCORE_OVER = "end"); the trial
ends as soon as the full-score state has held for 2 s (inside the scorer).
Start states: a reset with a seed (inspect-robots always passes one) draws the row of the hanging bin (2, 3 or 4 from
the bottom), which half it hangs in, its sideways place (up to 15 mm about a place 140 mm from the panel's centre
line), the panel's place (x 5 to 25 mm either way of nominal; y up to 15 mm) and the loose bin's place below the empty
half (x and y up to 20 mm, yaw up to 10 deg); for every seed but the default, the panel is never within 10 mm (x) of
the default seed's panel and the loose bin's x relative to the panel never within 8 mm of the default's (layout.py,
exclusion bands against open-loop replays); unseeded resets
use the named setups 0-2 in turn (layout.SETUPS; LB_SETUP picks the first and, when set, forces named setups). The
full-score reference policy is ``official_run.py`` (registered as ``louver_bin_scripted``).
"""

import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "src"))
import layout as L  # noqa: E402
from louverkit import Bench, STATE  # noqa: E402  (installs the yam_arena hooks)

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the 2 s still-and-clear hold is inside the scorer (level 1.0 needs it)
INSTRUCTION = json.loads((HERE.parent / "task.json").read_text())["instruction"]

# One task-file object stands for the whole bench (pose = panel; size only for yam_arena's checks).
OBJECTS = [
    Bench(
        "bench",
        pos=(float(L.XP_NOM) + 0.05, 0.0, float(L.PANEL_Z0 + L.PANEL_H / 2)),
        size=(0.1, 0.92, 0.48),
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
