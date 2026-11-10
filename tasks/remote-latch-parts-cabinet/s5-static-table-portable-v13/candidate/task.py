"""Two steel double-door cabinets, with side spring-return remote-release paddles.
Open cabinet A's right door and place its blue parts box upright on the table
outside the cabinet, with no robot touching it, stable for20seconds. A is on
world negativeY (right YAM arm). Each side paddle travels12mm and releases its connected latch at9mm.
Do not slam any door (>0.25m/s at knob on frame impact) or sustain wrist force
above80N for0.1seconds. Left/right mounts stand on252.5/352.5mm steel supports.
All model interaction is genuine14D joint targets at20Hz. Development only:
physics, collision clearance, safe forces and scores are unverified.
"""
import sys,json
from pathlib import Path
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
from s5_kit import Cabinet,Cargo,install,current_score,FIXTURE
from yam_arena.task_file import Box
install();SCENE='station';SCORE_OVER='end'
OBJECTS=[]
for b in FIXTURE['bodies']:
 if b['parent'] is not None:continue
 cls=Cargo if b['free'] else Cabinet;pos=tuple(x+y for x,y in zip(b['world_pos'],FIXTURE['translation']))
 OBJECTS.append(cls(name=b['name'],pos=pos,size=(.18,.36,.50) if cls is Cabinet else (.087,.05,.06),mass=.31 if cls is Cargo else None))
for side,x,y,h in [('left',-.08,-.5,.2525),('right',0.,-.305,.3525)]:OBJECTS.append(Box(name=side+'_s5_support',pos=(x,y,.7475+h/2),size=(.12,.12,h),color=(.3,.3,.32)))
def score(objects):return current_score()
