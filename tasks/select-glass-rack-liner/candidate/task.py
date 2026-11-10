"""Select a non-slip liner, lay it on the fixed20degree rack, stand a glass on
it, then leave both hands off for20seconds after0.5seconds settling, with5mm
maximum displacement and15degree normal alignment. Wet variant requires dipping
the liner in the pan first. A fallen/tipped/dropped glass vetoes all credit.
Six look-alike70x150x6mm liners, two74x70mm tumblers, shallow water pan, free
practice tray and fixed wooden rack. Development port; no transferred scores.
"""
from pathlib import Path
import sys
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
from c1_contract import *
from c1_kit import Part,install,current_score
install()
SCENE='station'
OBJECTS=[]
for i,p in enumerate(LINER_START):OBJECTS.append(Part(name=f'liner_{i+1}',kind='liner',number=i+1,pos=tuple(p),size=(.070,.150,.006),mass=.075,color=(.22,.24,.26)))
for i,p in enumerate(CUP_START):OBJECTS.append(Part(name=f'cup_{i+1}',kind='cup',number=i+1,pos=tuple(p),size=(.074,.074,.070),mass=.20,color=(.82,.90,.95)))
OBJECTS.append(Part(name='rack',kind='rack',pos=tuple(RACK_C),size=(.34,.24,.012),roll=ANGLE,color=(.62,.46,.30)))
for sx in [-1,1]:
 for sy in [-1,1]:
  p=NATIVE_RACK_C+sy*np.array([0.,np.cos(ANGLE),np.sin(ANGLE)])*(.12-.015)-np.array([0.,-np.sin(ANGLE),np.cos(ANGLE)])*.006
  p[0]+=sx*(.17-.015);height=p[2]-.80;p[2]=(.80+p[2])/2
  OBJECTS.append(Part(name=f'rack_leg_{"m" if sx<0 else "p"}_{"m" if sy<0 else "p"}',kind='leg',pos=tuple(world(p)),size=(.024,.024,float(height)),color=(.45,.33,.22)))
# Physical pan floor +4 walls; water volume is visual and a passive wet rule.
for name,p,size in [('floor',[.05,.035,.802],(.15,.18,.004)),('xm',[-.0225,.035,.8175],(.005,.18,.035)),('xp',[.1225,.035,.8175],(.005,.18,.035)),('ym',[.05,-.0525,.8175],(.14,.005,.035)),('yp',[.05,.1225,.8175],(.14,.005,.035))]:
 OBJECTS.append(Part(name='pan_'+name,kind='pan',pos=tuple(world(p)),size=size,color=(.8,.8,.82)))
OBJECTS.append(Part(name='tray',kind='tray',pos=tuple(world([-.25,.30,.804])),size=(.18,.14,.008),mass=.40,color=(.75,.72,.62)))
# Real pedestal top meets the exact SDK base collision-mesh bottom.
for side,p in ROBOT_BASES.items():
 h=p[2]+BASE_BOTTOM_Z-TABLE_Z
 OBJECTS.append(Part(name='c1_'+side+'_support',kind='support',pos=(p[0],p[1],TABLE_Z+h/2),size=(.12,.24,h),color=(.35,.35,.37)))
SCORE_OVER='end'
def score(objects):return current_score()
# Full executable state-aware reference is reference.py (genuine14D20Hz).
