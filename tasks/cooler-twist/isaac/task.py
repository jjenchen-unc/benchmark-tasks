"""AI-timelines task for yam_arena (Isaac Lab-Arena / PhysX): an AMD Wraith Prism cooler sits on an AM4 Ryzen processor
on an open-air test bench, held by its cam-lever clip; its thermal paste has dried. The robot is asked to take the
cooler off and put it on the anti-static mat. The dried paste holds harder in tension than the closed ZIF socket holds
the processor, so a straight pull takes the processor with it (pins bent); twisting the cooler a few degrees first
shears the paste.

    inspect-robots run --embodiment yam_arena --policy <policy> -E task=task.py -E object_jitter=0 \\
        -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<task.json instruction>" \\
        --scorer task_score --max-steps 1800 --store-frames --log-dir <dir>

The bench is custom-spawned PhysX bodies in one articulation (``coolerkit.py``: the test bench root, the processor on a
joint chain from the socket, the cooler on a joint chain from the processor, the cam lever on the cooler; the clip, the
ZIF hold and the paste film act through the joint drives). The scorer (``../src/scoring.py``) runs once per control
step on the processor's seat, the paste state, the cooler's pose and the robot-on-part contact forces; ``score()``
returns its level at the end of the trial; the trial ends as soon as the full-score state is reached (inside the
scorer). Start states: a reset with a seed (inspect-robots always passes one) moves the test bench 15-35 mm from its
nominal spot in any direction, turns it -90 +-12 deg and draws the paste's break torque (0.60-1.40 N m)
(layout.setup_spec); unseeded resets use the named setups in turn (CT_SETUP picks the first and, when set, forces named
setups). The full-score reference policy is ``official_run.py`` (registered as ``cooler_twist_scripted``).
"""

import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "src"))
import layout as L  # noqa: E402
from coolerkit import Bench, STATE  # noqa: E402  (installs the yam_arena hooks)

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the 3 s hands-off hold is inside the scorer (level 1.0 needs it)
INSTRUCTION = json.loads((HERE.parent / "task.json").read_text())["instruction"]

# One task-file object stands for the whole bench (pose = cooler origin; size only for yam_arena's checks).
OBJECTS = [Bench("bench", pos=(L.NOMINAL[0], L.NOMINAL[1], L.TOP + L.H_IHS + 0.05), size=(0.1, 0.1, 0.1), mass=None,
                 color=(0.9, 0.9, 0.9))]


def score(states):
    return float(STATE["score"])


def success(states):
    return STATE["score"] >= 1.0


def distance(states):
    return 1.0 - float(STATE["score"])


def demo():
    """For ``--policy yam_scripted`` (open-loop waypoints): no plan, both arms hold READY, so the official harness
    run is the do-nothing baseline (score 0). The full-score reference demo is closed-loop on measured state
    (``demo.py``, same embodiment, robot commands only)."""
    return {}
