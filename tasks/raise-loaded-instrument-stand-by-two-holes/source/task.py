"""Raise the loaded instrument stand by two 25mm holes on both legs. Return both
pins to their holes, level the plate within 3mm, and release all contact for 1s.
Keep instrument acceleration below 3g (5ms filter) and downward leg speed <=0.2m/s.
Two YAM arms: 27/7Nm TOTAL including gravity, 40N per finger; wrist >80N for
0.1s aborts; keep wrist below60N. Physics/model feasibility remains unverified.
"""
from pathlib import Path
import sys,math
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
from kit import Stand,Box,install,current_score
install()
SCENE='station'
OBJECTS=[Stand(name='pins',pos=(.55,.15,.7475),yaw=-math.pi/2,size=(1.,1.,.65),mass=None),Box(name='left_robot_pedestal',pos=(.32,.305,.9475),size=(.22,.24,.40),mass=None,color=(.2,.2,.2)),Box(name='right_robot_pedestal',pos=(.32,-.305,.9475),size=(.22,.24,.40),mass=None,color=(.2,.2,.2))]
SCORE_OVER='end'
def score(objects):return current_score()
