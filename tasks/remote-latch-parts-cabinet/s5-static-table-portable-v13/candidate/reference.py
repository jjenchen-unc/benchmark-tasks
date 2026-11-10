"""Disclosed state-aware RR-T1 teacher; ONLY genuine14D robot actions mutate scene.
Not a verified demo. No task joint/pose/effort/latch/score/history writes.
"""
import math,json
import numpy as np
from yam_arena import ik
from yam_arena.demo import grip_rotation
from yam_arena.constants import PAD_TIP_PAST_TCP
from inspect_robots.types import Action
from s5_kit import FIXTURE
class GoalReached(Exception):pass
class Teacher:
 def __init__(self,emb,obs,log):self.emb=emb;self.obs=obs;self.log=log;self.action=np.asarray(obs.state['joint_pos'],float).copy();self.q=self.action[7:13].copy();self.pos,self.R=ik.tcp_pose(self.q,FIXTURE['robot_bases']['right']);self.yaw=0.;self.controls=0;self.paddle_bias=np.zeros(6);self.side="right";self.offset=7
 def step(self,target,label):
  from telemetry import dumps,snapshot
  before=snapshot(self.emb,self.obs)
  target=np.asarray(target,float).copy();nominal=target.copy()
  # Measured joint deflection compensation only during the loaded paddle pull.
  # Original public motor caps, gains, physics and nominal whole-goal route stay.
  raw=np.asarray(before['robots']['right']['raw8'],float)[:6]
  from paddle_feedback import update_bias
  self.paddle_bias=update_bias(self.paddle_bias,nominal[7:13],raw,label)
  target[7:13]+=self.paddle_bias
  if any(np.any(target[o:o+6]<ik.LOWER) or np.any(target[o:o+6]>ik.UPPER) for o in [0,7]):raise RuntimeError('paddle feedback joint limit')
  previous=getattr(self,'last_sent',target)
  if max(np.max(abs(target[o:o+6]-previous[o:o+6])) for o in [0,7])>.10:raise RuntimeError('public command discontinuity')
  if self.controls>=3646:raise RuntimeError('native3646 control budget exhausted')
  result=self.emb.step(Action(data=target));self.obs=result.observation;self.controls+=1;self.last_sent=target.copy()
  self.log.write(dumps(dict(label=label,action=target,nominal_action=nominal,paddle_bias=self.paddle_bias,before=before,state=np.asarray(self.obs.state['joint_pos']),info=result.info,measured=snapshot(self.emb,self.obs)))+'\n');self.log.flush()
  if result.terminated:
   if result.termination_reason=='success':raise GoalReached()
   raise RuntimeError('episode terminated: '+str(result.termination_reason))
  from s5_kit import ENGINE
  if ENGINE.last and max(ENGINE.last['wrist_force_n'])>60:raise RuntimeError('teacher60N force guard')
  return result
 def select_arm(self,side):
  self.side=side;self.offset=0 if side=='left' else 7;self.q=np.asarray(self.obs.state['joint_pos'],float)[self.offset:self.offset+6].copy();self.action=np.asarray(self.obs.state['joint_pos'],float).copy();self.pos,self.R=ik.tcp_pose(self.q,FIXTURE['robot_bases'][side])
 def move(self,p,yaw=None,grip=None,speed=.04,label='move',seconds=None,rotation=None):
  p=np.asarray(p,float);start=self.pos.copy();oldyaw=self.yaw;newyaw=self.yaw if yaw is None else yaw;Rstart=self.R.copy();Rend=grip_rotation(newyaw) if rotation is None else np.asarray(rotation,float);oldgrip=self.action[self.offset+6];newgrip=oldgrip if grip is None else grip
  duration=max(np.linalg.norm(p-start)/speed,float(np.linalg.norm(ik._rot_error(Rend,Rstart)))/.25,1. if abs(newgrip-oldgrip)>.01 else .1) if seconds is None else seconds
  n=max(1,math.ceil(duration*20))
  for k in range(n):
   s=(k+1)/n;s=s*s*(3-2*s);pt=start+(p-start)*s;ya=oldyaw+(newyaw-oldyaw)*s;q,e,r=ik.solve(pt,ik.matrix_to_quat_xyzw(ik.slerp_matrix(Rstart,Rend,s)),q_init=self.q,base_pos=FIXTURE['robot_bases'][self.side],iters=80)
   if e>.002 or r>.03:raise RuntimeError(f'IK failure {label}: position {e}, angle {r}')
   if np.max(abs(q-self.q))>.10:raise RuntimeError('IK branch/speed discontinuity '+label)
   self.q=q;self.action[self.offset:self.offset+6]=q;self.action[self.offset+6]=oldgrip+(newgrip-oldgrip)*s;self.step(self.action.copy(),label)
  self.pos=p;self.yaw=newyaw;self.R=Rend
 def joint_move(self,q,label):
  q=np.asarray(q,float);start=self.q.copy();n=max(1,math.ceil(float(np.max(abs(q-start)))/.025))
  for k in range(n):
   f=(k+1)/n;f=f*f*(3-2*f);self.q=start+(q-start)*f;self.action[self.offset:self.offset+6]=self.q;self.step(self.action.copy(),label)
  self.pos,self.R=ik.tcp_pose(self.q,FIXTURE['robot_bases'][self.side])
 def wait(self,s,label):
  for _ in range(math.ceil(s*20)):self.step(self.action.copy(),label)
def knob(theta):
 c,s=math.cos(theta),math.sin(theta);return np.array([.25-.006+c*(-.040)-s*.136,-.29-.178+s*(-.040)+c*.136,.7475+.455])
def nominal_points():
 return dict(paddle_front=(.15,-.01,1.1975),paddle_side=(.36,-.01,1.1975),paddle=(.36,-.08,1.1975),paddle_pull=(.36,-.045,1.1975),box_front=(.16,-.378,.788),box_tab=(.265,-.378,.788),box_out=(.13,-.378,.796),box_setdown=(.05,-.10,.7795),**{'door_'+str(deg):knob(math.radians(deg)).tolist() for deg in [0,12,25,50,75,100]})
def _execute(emb,obs,log):
 from s5_kit import ENGINE
 t=Teacher(emb,obs,log);P=nominal_points()
 # Command the left posture previously assumed by the finite door-path audit.
 # It must move through genuine controls from the actual reset posture.
 t.select_arm('left');t.joint_move([0.,.3,2.4,-.5,0.,0.],label='left_park_before_door')
 parked_left=t.action[:7].copy()
 t.select_arm('right');t.action[:7]=parked_left
 # Get in front first, then slide along outside side face; no route over cabinet.
 t.move((.15,-.38,1.1975),math.pi/2,1.,label='high_front')
 t.move(P['paddle_front'],math.pi/2,1.,label='paddle_front');t.move(P['paddle_side'],math.pi/2,1.,label='along_side');t.move(P['paddle'],math.pi/2,1.,label='onto_rear_paddle');t.move(t.pos,grip=0.,seconds=1.,label='grip_paddle')
 t.move(P['paddle_pull'],grip=0.,speed=.012,label='pull_rear_paddle');t.wait(.5,'paddle_release_hold')
 for _ in range(60):
  if ENGINE.latches.latch['AR']=='open':break
  t.step(t.action.copy(),'paddle_release_hold')
 if ENGINE.latches.latch['AR']!='open':raise RuntimeError('robot paddle pull failed to release rightdoor')
 t.move(P['paddle'],grip=0.,label='return_paddle');t.move(t.pos,grip=1.,seconds=.5,label='let_go_paddle');t.move(P['paddle_side'],label='retreat_side');t.move(P['paddle_front'],label='retreat_front')
 theta=ENGINE.last['door_q']['AR'];p=knob(theta)
 t.move(p+np.array([-.06,0,0]),theta,1.,label='knob_approach');t.move(p,theta,1.,label='knob_contact');t.move(t.pos,grip=0.,seconds=1.,label='grasp_knob')
 for a in np.linspace(theta,50*math.pi/180,18)[1:]:t.move(knob(a),a,0.,speed=.04,label='open_arc_first')
 # Regrasp at50degrees to respect actual SDK wrist roll limits; do not fake
 # the motor range or force the remaining arc through an unreachable yaw.
 t.move(t.pos,grip=1.,seconds=.5,label='release_for_regrasp');t.move(t.pos+np.array([-.06,0,0]),label='back_for_regrasp');t.move(t.pos,yaw=50*math.pi/180-math.pi,label='symmetric_jaw_rotation')
 theta=ENGINE.last['door_q']['AR'];t.move(knob(theta),theta-math.pi,1.,label='reapproach_knob');t.move(t.pos,grip=0.,seconds=1.,label='regrasp_knob')
 for a in np.linspace(theta,100*math.pi/180,18)[1:]:t.move(knob(a),a-math.pi,0.,speed=.04,label='open_arc_second')
 if ENGINE.last['door_q']['AR']<85*math.pi/180:raise RuntimeError('door tracking failed')
 t.move(t.pos,grip=1.,seconds=.5,label='release_knob');t.move(t.pos+np.array([-.06,0,0]),label='clear_knob')
 # Park the original right arm high before the relocated left handles the box.
 t.joint_move([t.q[0],1.5,2.5,-1.,0.,t.q[5]],label='right_park_above')
 t.select_arm('left')
 t.move(t.pos,grip=.5325,rotation=t.R,seconds=1.,label='left_empty_jaw_preset')
 from pathlib import Path
 plan=json.loads((Path(__file__).parent/'contact-route.json').read_text())
 if abs(ENGINE.last['door_q']['AR']-plan['door_angle_rad'])>.05:raise RuntimeError('new path door angle binding outside0.05rad')
 for q in plan['approach']:t.joint_move(q,label='box_contact_approach')
 t.move(t.pos,grip=0.,rotation=t.R,seconds=1.,label='pinch_box_tab')
 for stage in ['lift8mm','extract_box','carry_to_table','lower_box']:
  for q in plan[stage]:t.joint_move(q,label=stage)
 from telemetry import snapshot
 raw=np.asarray(snapshot(emb,t.obs)['robots']['left']['raw8'],float)
 t.action[6]=float(np.clip(max(raw[6:]/-.04695),0.,1.))
 t.move(t.pos,grip=1.,rotation=t.R,seconds=1.,label='release_box')
 for q in plan['hands_off']:t.joint_move(q,label='hands_off')
 t.wait(20.5,'final20s_hold')
 return dict(controls=t.controls,status=ENGINE.status(),reference_verified=False)

def run(emb,obs,log):
 try:return _execute(emb,obs,log)
 except GoalReached:
  from s5_kit import ENGINE
  return dict(controls=ENGINE.controls,status=ENGINE.status(),terminated_success=True,reference_verified=False)
