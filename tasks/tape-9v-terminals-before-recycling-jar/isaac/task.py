"""battery-recycle for yam_arena (Isaac Lab-Arena / PhysX): put the used batteries from the bench into the
recycling jar. The 9 V's terminals must be taped first (the tape strip lies on the bench); a bare 9 V in the jar
scores 0.

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py -E object_jitter=0 \\
        -E control_hz=20 -E cam_width=320 -E cam_height=176 -E device=cpu --instruction "<task.json instruction>" \\
        --scorer task_score --max-steps <run_limit> --store-frames --log-dir <dir>

The bench is one custom-spawned PhysX assembly (``recyclekit.py``; geometry in ``layout.py``). The adhesive runs
after every physics step, the scorer (``scoring.py``) after every control step on measured poses, bonds and PhysX
contact reports; ``score()`` returns its value (SCORE_OVER = "end", the bare-9 V veto latches). The full-score
reference policy is ``official_run.py`` (registered as ``recycle_scripted``); ``demo()`` is empty, so
``--policy yam_scripted`` is the do-nothing baseline.
"""
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from recyclekit import STATE, Bench  # noqa: E402  (installs the yam_arena hooks)
import layout as L  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 2.0  # everything in the jar, released and still for 2 s
INSTRUCTION = json.loads((pathlib.Path(__file__).resolve().parent.parent / "task.json").read_text())["instruction"]

# One task-file object stands for the whole bench (pose = the 9 V's reset pose; size only for yam_arena's checks).
OBJECTS = [Bench("bench", pos=tuple(float(v) for v in L.NINE_V_POS), size=tuple(L.NINE_V_SIZE), mass=None,
                 color=(.06, .06, .06))]


def score(states):
    return float(STATE["score"])


def success(states):
    return bool(STATE["success"])


def distance(states):
    return 1.0 - float(STATE["score"])


def demo():
    """``--policy yam_scripted`` plays open-loop waypoints: no plan here, both arms hold READY (do-nothing baseline,
    score 0). The full-score reference demo is closed-loop on measured poses (``demo.py`` / ``official_run.py``)."""
    return {}
