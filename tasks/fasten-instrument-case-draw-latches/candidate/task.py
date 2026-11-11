"""Close the blue-grey instrument case and fasten both spring draw latches.
Seat each hook on a keeper bar, tension it with the lever flat, and return both
secondary catches home. Leave both hands off the case and instrument for20s.
Do not tip either instrument more than20deg or either case more than15deg.
The cases stand loose on a wooden plinth; the green case remains closed.
"""
import math,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from kit import Case,Instrument,Plinth,world,install,current_score
install()
SCENE='station'
SCORE_OVER='end'
OBJECTS=[Plinth(name='plinth',pos=tuple(world([.04,.07,.83])),size=(.2,.6,.06),color=(.55,.42,.30))]
for c,x,color,instcolor in [('A',-.10,(.22,.30,.40),(.85,.8,.3)),('B',.18,(.24,.36,.28),(.8,.4,.3))]:
 OBJECTS.append(Case(name='case_'+c,label=c,pos=tuple(world([x,.06,.86])),yaw=-math.pi/2,mass=1.82,size=(.24,.16,.22),color=color))
 OBJECTS.append(Instrument(name='inst_'+c,pos=tuple(world([x,.07,.9465])),radius=.02,height=.16,mass=.3,color=instcolor))
def score(objects):return current_score()
