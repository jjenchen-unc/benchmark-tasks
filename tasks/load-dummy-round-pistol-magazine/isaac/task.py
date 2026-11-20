"""Load one dummy round into an empty pistol magazine with a loading tool (Impact on jobs). yam_arena task file
(Isaac Lab-Arena, PhysX).

An empty double-stack 9 mm magazine stands in a fixture on the table; one inert 9x19 dummy round and a 3D-printed
loading tool lie on the table. The robot puts the tool on the magazine, presses the follower down, slides the round
under the feed lips, raises the tool so its tooth pushes the round back to the rear wall, lifts the tool clear, and moves both grippers away
from the round.

Score (task_scoring.py, latched vetoes, otherwise the lowest level over the last 2 s, hands >= 50 mm from the round):
0 / 0.25 round held under the lips (not loaded, or loaded with the tool still on) / 1.0 loaded and tool off. The trial
ends on 1.0.
Scene, measurements and the per-substep monitors: magkit.py.
"""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import magkit  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
INSTRUCTION = ("Load the dummy round into the magazine with the loading tool: the round must sit fully back under the "
               "feed lips, bullet forward. Then lift the tool clear of the magazine (you may keep holding it) and move both "
               "grippers away from the round.")
OBJECTS = magkit.OBJECTS


def score(objects):
    return float(magkit.STATE["score"])


def success(objects):
    return magkit.STATE["score"] >= 1.0


def demo():
    return {}  # yam_scripted holds READY (do-nothing baseline)
