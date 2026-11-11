"""SH-T1 conditional teacher; read-only measured task feedback, public robot14D only.
No physics feasibility claim. Cap overtravel compensates measured arm compliance;
all passive catches, native thresholds and physical effort limits remain untouched.
"""
from pathlib import Path
import json,math
import numpy as np
from yam_arena import ik
from yam_arena.constants import READY,PAD_TIP_PAST_TCP
from yam_arena.urdf import axis_angle_matrix,quat_xyzw_to_matrix
F=json.loads((Path(__file__).parent/'fixture.json').read_text());S=F['fixtures']['SH-T1']['boxes']['1'];SHIFT=np.array(F['translation']);D=math.pi/180
BASE=(0.,-.305,.75)
def ry(a):return axis_angle_matrix(np.array([0.,1.,0.]),a)
def rx(a):return axis_angle_matrix(np.array([1.,0.,0.]),a)
def rz(a):return axis_angle_matrix(np.array([0.,0.,1.]),a)
def lever(al,th,ph):
 R=ry(-al);pivot=np.array([-.1,-.32,.912]);shaft=pivot+R@np.array([.165,.07,0.]);B=R@rz(th);L=B@rx(-ph)
 return shaft+B@np.array([0,0,.028])+L@np.array([0,-.105,0])+SHIFT,L@np.diag([-1.,1.,-1.])
def cap(depth,door):
 R=ry(-door);face=np.array([-.1,-.32,.912])+R@np.array([.165,-.085,.014])+SHIFT;hand=R@np.diag([-1.,1.,-1.])
 return face+hand[:,2]*(depth-PAD_TIP_PAST_TCP),hand
# Every phase has a finite deadline; early event transitions save controls.
PHASES=[('cap_approach',60),('cap_descend',40),('cap_feedback',160),('cap_retreat',40),('lever_approach',60),('lever_descend',40),('lever_grasp',40),('unlock_turn',320),('lever_lid_lift',400),('release_lever',40),('lift_retreat',60),('hands_off_hold',500)]
class Teacher:
 def __init__(self):
  self.seq=PHASES;self.index=0;self.tick=0;self.above=0.;self.finished=False;self.depth=0.;self.good=0;self.turn=0.;self.door=0.;self.pop=None;self.telemetry={};self.controls=0
 def advance(self):self.index+=1;self.tick=0;self.good=0;self.finished=self.index==len(self.seq)
 def next(self,measured14,wrist_force_n,fixture,raw_fingers):
  if self.finished:raise StopIteration
  self.controls+=1
  if self.controls>4198:raise RuntimeError('native action budget')
  q=fixture['qv'][0];al=float(q['door_1']);th=float(q['swing_1']);ph=float(q['pop_1']);cp=float(q['cap_1'])
  if not np.isfinite([al,th,ph,cp,*measured14,*wrist_force_n]).all():raise ValueError('nonfinite measurement')
  name,deadline=self.seq[self.index];self.tick+=1;u=min(1.,self.tick/deadline);jaw=0.;done=False
  guard=45. if name in ['lever_lid_lift','release_lever','lift_retreat'] else 30.
  self.above=self.above+.05 if max(wrist_force_n)>guard else 0.
  if self.above>=.1-1e-9:raise RuntimeError('untared force guard: '+name)
  if name.startswith('cap_'):
   if name=='cap_approach':depth=-.06
   elif name=='cap_descend':depth=-.06*(1-u)
   elif name=='cap_feedback':
    # 0.1mm/control maximum; 16mm hard target-overtravel bound, never task motion.
    self.depth=float(np.clip(self.depth+np.clip(.3*(.00265-cp),-.0001,.0001),0.,.016));depth=self.depth
    self.good=self.good+1 if cp>.0024 and ph>26*D else 0;done=self.good>=3
   else:depth=self.depth-(self.depth+.06)*u
   p,R=cap(depth,al)
   if name=='cap_retreat' and ph<22*D:raise RuntimeError('pop lost after cap release')
  else:
   if self.pop is None:self.pop=ph
   if name in ['lever_approach','lever_descend','lever_grasp']:
    if ph<22*D:raise RuntimeError('cannot grasp unpopped lever')
    # Use actual popped lever pose, rather than the old phantom32deg grasp.
    p,R=lever(al,th,ph);p-=R[:,2]*(.005 if name=='lever_approach' else .005*(1-u) if name=='lever_descend' else 0.)
    jaw=1. if name!='lever_grasp' else .18 # 22mm bar, about13N/pad nominal
    if name=='lever_grasp':
     gaps=-np.asarray(raw_fingers,dtype=float)
     contact=bool(gaps.shape==(2,) and np.isfinite(gaps).all() and np.min(gaps)>.0091 and np.max(gaps)<.015 and abs(gaps[0]-gaps[1])<.003)
     self.good=self.good+1 if contact else 0;done=self.good>=10
   elif name=='unlock_turn':
    if ph<22*D:raise RuntimeError('pop below native swing unlock threshold')
    # Target remains <=4deg ahead of measured swing; no blind full-turn replay.
    self.turn=min(S['angle']-3*D,self.turn+.5*D,th+4*D)
    a=-.02 if self.turn<=5*D else -.02+.02*(self.turn-5*D)/(S['comp']-5*D) if self.turn<=S['comp'] else 0.
    p,R=lever(a,self.turn,ph);jaw=.18
    done=th>S['angle']-7*D
   elif name=='lever_lid_lift':
    self.door=min(65*D,self.door+.3*D,al+3*D)
    p,R=lever(self.door,th,ph);jaw=.18
    done=al>=63*D
   else:
    p,R=lever(al,th,ph);jaw=1.
    if name=='release_lever':
     if self.tick==1:self.release=(p.copy(),R.copy())
     p,R=self.release
    else:
     p,R=self.release;p=p+np.array([.06,0.,0.])*(u if name=='lift_retreat' else 1.)
   if name=='hands_off_hold' and al<60*D:raise RuntimeError('native door hold lost')
  qarm,e,r=ik.solve(p,ik.matrix_to_quat_xyzw(R),q_init=np.asarray(measured14)[7:13],base_pos=BASE,iters=120)
  if e>.006 or r>.10:raise RuntimeError(f'IK failed {name}: {e:.6f}m/{r:.6f}rad')
  prev=np.asarray(measured14)[7:13];speed=np.array([5.5]*3+[20.]*3)*.05*.8;qarm=prev+np.clip(qarm-prev,-speed,speed)
  action=np.r_[READY,1.,qarm,jaw]
  self.telemetry={'phase':name,'cap_target_overtravel_m':self.depth,'measured_cap_m':cp,'measured_pop_rad':ph,'measured_swing_rad':th,'measured_door_rad':al,'target_position_m':p.tolist(),'ik_position_error_m':float(e),'ik_rotation_error_rad':float(r)}
  if self.tick>=deadline and name in ['cap_feedback','lever_grasp','unlock_turn','lever_lid_lift'] and not done:raise RuntimeError('measured progress deadline: '+name)
  if done or self.tick>=deadline:self.advance()
  return action,name
