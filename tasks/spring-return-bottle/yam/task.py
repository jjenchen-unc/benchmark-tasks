"""Open an upright free-standing bottle with a U safety loop and spring-return rocker.

Keep the spring-return safety loop down while pressing the lower rocker. Keep
the bottle upright on the table. Only the YAM arms may manipulate the bottle.
This PhysX port is experimental; its contact mechanism is being validated.
"""
import sys
import math
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from spring_runtime import SpringBottle, install, current_score
from yam_arena.task_file import TABLE_TOP

install()
SCENE = "station"
TOP = TABLE_TOP[SCENE]
OBJECTS = [SpringBottle(name="bottle", pos=(0.27, 0.0, TOP + 0.0005),
                  size=(0.10, 0.12, 0.282), mass=0.35,yaw=-math.pi/2)]
SUCCESS_HOLD_S = 0.0  # The evaluator already requires eight consecutive ticks.
SCORE_OVER = "end"


def success(objects):
    return current_score() == 1.0


def score(objects):
    return current_score()


def demo():
    """Candidate simultaneous hold-and-press plan; not yet verified."""
    from spring_motion import plan
    return plan(TOP)
