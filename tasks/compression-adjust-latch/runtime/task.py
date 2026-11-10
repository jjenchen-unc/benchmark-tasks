"""Adjust the unloaded grip-length knob, turn the handle clockwise to lock the
compression latch, then leave both hands off every enclosure part for 20 seconds.
Full credit requires a lid/frame gap below 1 mm and pawl clamp above 20 N.
Sustained cam torque above 3 Nm for 0.1 seconds permanently damages the latch.
"""
import sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from kit import Latch,Box,install,current_score,FIX
install()
SCENE='station'
OBJECTS=[Latch(name='compression_latch',pos=tuple(FIX['arena_root']),size=(.324,.40,.21),mass=None)]
# Real fixed supports meet the exact installed YAM base-mesh underside.
from yam_arena.station import BASES
for side,base in BASES.items():
    top=base[2]+FIX.get('robot_base_rise_m',0.)+FIX.get('robot_base_mesh_bottom_z_m',-.001101112)
    height=top-.7475
    OBJECTS.append(Box(name=side+'_robot_pedestal',pos=(base[0],base[1],.7475+height/2),size=(.24,.12,height),color=(.35,.35,.37)))
SCORE_OVER='end'
def score(objects):return current_score()
def demo():return {}
