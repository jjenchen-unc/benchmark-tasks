"""D3 integration candidate: real YAM/carton/foam physics; scoring remains gated."""
from pathlib import Path
import sys
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
from d3_kit import Carton,Face,Cargo,install,FIXTURE,current_score
install()
SCENE='station'
OBJECTS=[]
for i,c in enumerate(FIXTURE['cartons']):
 x,y,z=c['position_world_m'];n=i+1
 OBJECTS.extend([Carton(name=f'carton_{n}',number=n,pos=(x,y,z),size=(.178,.138,.084),mass=.215,color=(.72,.58,.4)),Face(name=f'pad_{n}',number=n,pos=(x,y,z+.0285),size=(.164,.124,.003),mass=.015,color=(.93,.86,.55))])
for i,label in enumerate('AB'):
 OBJECTS.append(Cargo(name='box_'+label,label=label,pos=(.29+.16*i,-.06,.7725),size=(.12,.068,.05),mass=FIXTURE['parameters']['box_mass'][i],color=(.80,.82,.85)))
SCORE_OVER='end'
def score(objects):return current_score()
def demo():return {}
