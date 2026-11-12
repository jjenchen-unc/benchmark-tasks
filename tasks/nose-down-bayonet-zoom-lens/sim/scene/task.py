"""Mount a compatible zoom lens on the downward-facing camera, lock it and set
50 mm. Release all cameras/lenses for a final continuous 20 s. Lens drop or mount
overload vetoes all credit. Fixed physical case, genuine two YAM parallel hands.
This development port has no runtime, full-goal, Dream-1 or media proof yet.
"""
import json,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from kit import Component,install,current_score
from yam_arena.task_file import Box
install()
F=json.loads((HERE/'fixture.json').read_text());SCENE='station';SCORE_OVER='end'
OBJECTS=[]
for b in F['bodies']:
 if b['parent'] is None:
  OBJECTS.append(Component(name=b['name'],pos=tuple(x+y for x,y in zip(b['pos'],F['translation'])),size=(.15,.3,.6),mass=sum(v['mass'] for v in F['bodies'] if v['name'].startswith(b['name'])) if b['name'].startswith('lens_') else None))
for g in F['world_geoms']:
 OBJECTS.append(Box(name=g['name'],pos=tuple(x+y for x,y in zip(g['pos'],F['translation'])),size=tuple(2*x for x in g['size']),color=tuple(g['rgba'][:3])))
# Robot pedestal adaptation is finalized only after nominal IK, and must be checked physically.
BASE_Z=.90
for side,y in [('left',.305),('right',-.305)]:
 OBJECTS.append(Box(name=side+'_b6_pedestal',pos=(0,y,(BASE_Z+.7475)/2),size=(.20,.20,BASE_Z-.7475),color=(.25,.25,.27)))
def score(objects):return current_score()
