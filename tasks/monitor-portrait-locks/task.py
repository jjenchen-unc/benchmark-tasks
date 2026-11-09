"""Raise the monitor 130 mm, rotate it 90 degrees to portrait with no tilt, and
leave it released and stable for 20 s. A box beside the screen must not be touched.

Standalone Arena task. Measured contact/wrench/history
observers evaluate the source rubric. Native success does not transfer.
"""
import json,math,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from yam_arena.task_file import Box
from kit import Monitor,install,current_score
from contract import evaluate
install()
SCENE='station'
FIXTURE=json.loads((HERE/'fixture.json').read_text())
OBJECTS=[Monitor(name='monitor_stand',pos=tuple(FIXTURE['arena_root']),yaw=math.pi/2,size=(.42,.49,.60),mass=None),
         Box(name='left_robot_pedestal',pos=(0.,.305,.8975),size=(.20,.20,.30),mass=None,color=(.55,.55,.57)),
         Box(name='right_robot_pedestal',pos=(0.,-.305,.8975),size=(.20,.20,.30),mass=None,color=(.55,.55,.57))]
SCORE_OVER='end'
def score(objects):return current_score()
def demo():
    # Registration/neutral hold only; no full-score or policy success is claimed.
    return {}
