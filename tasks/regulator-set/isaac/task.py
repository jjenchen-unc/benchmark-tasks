"""Impact-on-jobs task for yam_arena (Isaac Lab-Arena / PhysX): set a non-relieving shop-air regulator (1/4 in,
modelled on the SMC AR20 non-relieving option, push-to-lock knob) to 90 psi on its 2.5 in 0-160 psi gauge. The line
starts above 90 psi; turning the knob down does not lower a non-relieving regulator's outlet, so one hand opens the
downstream bleed ball valve while the other turns the knob, then the bleed is closed and the knob locked.

    inspect-robots run --embodiment yam_arena --policy <policy> -E task=task.py -E object_jitter=0 \\
        -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 --instruction "<task.json instruction>" \\
        --scorer task_score --max-steps 1800 --store-frames --log-dir <dir>

The drop is custom-spawned PhysX bodies (``regkit.py``: one fixed-base articulation: knob on its push-to-lock travel
and adjusting screw, the ball valve's lever, the gauge pointer). The scorer (``../src/scoring.py``) runs once per
control step on the gauge reading, the regulator's setpoint and line pressure, the knob travel, the lever angle and
the robot-on-part contact forces; ``score()`` returns its level at the end of the trial; the trial ends as soon as the
full-score state is reached (inside the scorer). Start states: a reset with a seed (inspect-robots always passes one)
draws the start pressure (104-122 psi: the regulator was left set high), the spring rate (12-18 psi per knob turn),
moves the drop up to 15 mm and turns it up to 6 deg (layout.setup_spec); unseeded resets use the named setups in turn
(RG_SETUP picks the first and, when set, forces named setups). The full-score reference policy is
``official_run.py`` (registered as ``regulator_set_scripted``).
"""

import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "src"))
import layout as L  # noqa: E402
from regkit import AirDrop, STATE  # noqa: E402  (installs the yam_arena hooks)

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.0  # the holds are inside the scorer (level 1.0 needs them)
INSTRUCTION = json.loads((HERE.parent / "task.json").read_text())["instruction"]

# One task-file object stands for the whole drop (pose = regulator body; size only for yam_arena's checks).
OBJECTS = [AirDrop("airdrop", pos=(L.NOMINAL[0], L.NOMINAL[1], L.TOP + L.H_BODY), size=(0.1, 0.1, 0.1), mass=None,
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
