"""Disclosed state-aware robot teacher; only Action(data=14D) changes the scene.
No task pose/joint/effort/score writes. Unvalidated proposed full-goal sequence.
"""
import json,math,os
from numeric_json import dumps
from pathlib import Path
import numpy as np
from kit import arr,rotation,F
def reseat_step(e,corr,tol=.00025,engage=.0005,step=.0004,total=.006,capture=.0125):
 """Bounded height step toward the current goal hole; 0 when aligned/unresolvable.
 Hysteresis: starts only above engage=.5 mm (0.25 mm pin clearance + 0.25 mm
 margin; actual v12 L reseat legL=.0502503, e=+0.25027 mm, stays a strict
 no-op), then once started (corr!=0) continues down to the 0.25 mm clearance."""
 if not math.isfinite(e) or abs(e)>capture or abs(e)<=(tol if corr!=0. else engage):return 0.
 dz=float(np.clip(-e,-step,step));dz=float(np.clip(corr+dz,-total,total))-corr
 return 0. if abs(dz)<1e-12 else dz
# v18: measured-velocity damped tight centering (private planner only). v17
# actual L hole3: leg pinned at the +-250 um wall while 100 um/tick target steps
# wound ~1.3 mm of unanswered lead, then coasted 300 um (115,99,67,22 um/tick)
# after dz reached 0, crossing to the opposite wall. de_tick = measured leg
# displacement over the previous 50 ms control (read-only joint_pos; the 2 ms
# joint_vel is stick-slip noise: +-4 mm/s with zero 50 ms displacement).
# Stationary (|de|<=10 um/tick): exactly the old step. Moving: brake on the
# predicted position e+3*de (3 ticks = measured coast/peak ratio 303/115 um).
DAMP_STILL=1e-5;DAMP_KP=.4;DAMP_LEAD=3.
def damped_reseat_step(e,de,corr,tol=.0001,engage=.00015,step=.0001,total=.006,capture=.0125):
 if not math.isfinite(de):raise ValueError('Nonfinite measured reseat displacement')
 if abs(de)<=DAMP_STILL:return reseat_step(e,corr,tol,engage,step,total,capture)
 if not math.isfinite(e) or abs(e)>capture:return 0.
 dz=float(np.clip(-DAMP_KP*(e+DAMP_LEAD*de),-step,step));dz=float(np.clip(corr+dz,-total,total))-corr
 return 0. if abs(dz)<1e-12 else dz
# v13s: servo-bias unwind before a wound-up grip opens. Same per-tick rates and
# caps as the existing integrator in tick(); bias only shrinks toward 0, so the
# command moves monotonically from nominal+bias to the unchanged nominal IK pose.
SERVO_RATE=np.array([.0025]*3+[.005]*3);SERVO_CAP=np.array([.20]*3+[.45]*3)
# v19: measured axial seat target. v18 actual L hole3 grip offset (tcp_y+pin)
# .12141 vs hole2 .12612; frozen nominal y .125 is reached (tcp .12516) with pin
# -1.94 mm. Target from captured grip frame: tcp_y+pin (pin goal 0), signed -y,
# bounded per tick/total, held (no advance) at >=45 N measured wrist force.
AXIAL_STEP=.00025;AXIAL_TOTAL=.006
def axial_seat_step(tcp_y,pin,nominal_y,corr,force):
 if not all(math.isfinite(v) for v in (tcp_y,pin,nominal_y,corr,force)) or force<0:raise ValueError('Finite measured axial state required')
 if pin>1e-7 or pin<-.012-1e-7:raise ValueError('Pin outside native float32 mechanism bounds')
 if not -AXIAL_TOTAL-1e-12<=corr<=0.:raise ValueError('Invalid axial correction accumulator')
 if force>=FORCE_BIAS_HIGH_N:return 0.
 want=min(0.,tcp_y+pin-nominal_y)
 dy=float(np.clip(want,-AXIAL_STEP,0.));dy=float(np.clip(corr+dy,-AXIAL_TOTAL,0.))-corr
 return 0. if abs(dy)<1e-12 else dy
def unwind_step(b):return b-np.clip(b,-SERVO_RATE,SERVO_RATE)
def wound(b):return float(np.max(np.abs(np.asarray(b,float))/SERVO_CAP))>=.9
# Private control targets provide margin to the unchanged60N instruction.
# These do not change native80N/100ms abort or reported measured forces.
FORCE_BIAS_LOW_N=40.;FORCE_BIAS_HIGH_N=45.
def force_bias_step(b,error,force):
 b=np.asarray(b,float);error=np.asarray(error,float)
 if b.shape!=(6,) or error.shape!=(6,) or not np.isfinite(b).all() or not np.isfinite(error).all() or not math.isfinite(force) or force<0:raise ValueError('Finite measured force and6D bias/error required')
 if np.any(np.abs(b)>SERVO_CAP+1e-10):raise ValueError('Bias exceeds original cap')
 if force>=FORCE_BIAS_HIGH_N:return unwind_step(b)
 if force>FORCE_BIAS_LOW_N:return b.copy()
 return np.clip(b+np.clip(.2*error,-SERVO_RATE,SERVO_RATE),-SERVO_CAP,SERVO_CAP)

def force_regulation_enabled(arm,phase,leg=None,pin=None):
 """Only measured-preload windows; preserve original pull/seating integration.
 Left regulation applies fully withdrawn/between25mm holes, with1mm margin
 from hole alignment. Mechanical capture/limits/thresholds are unchanged.
 """
 if arm not in (0,1) or not isinstance(phase,str):raise ValueError('Expected arm and phase')
 if arm==1:return phase.startswith('L_knob_') or phase in ('L_closed_grip_track','L_unload','L_pull','L_withdraw_track','L_withdraw_overtravel')
 if phase!='L_raise':return False
 if leg is None or pin is None or not math.isfinite(leg) or not math.isfinite(pin):raise ValueError('Finite measured left leg/pin needed')
 return pin<=-.0105 and abs(leg-.025*round(leg/.025))>.001

class Teacher:
 def __init__(self,emb,obs,record):
  from inspect_robots.types import Action
  self.Action=Action;self.emb=emb;self.obs=obs;self.record=record;self.target=np.array(obs.state['joint_pos'],float);self.poses=[None,None];self.rots=[None,None]
  from yam_arena.ik import tcp_pose
  from yam_arena.station import BASES
  self.bases=[np.array(BASES[s])+[.32,0,.40] for s in ['left','right']]
  for a in range(2):self.poses[a],self.rots[a]=tcp_pose(self.target[a*7:a*7+6],self.bases[a])
  from pair_diagnostic import PairDiagnostic
  self.pair_diagnostic=PairDiagnostic(emb)
  self.servo_bias=np.zeros((2,6));self.servo_enabled=False;self.unwinding=[False,False]
 def tick(self,phase):
  self.pair_diagnostic.phase=phase
  if len(self.record)>=4572:raise RuntimeError('reference time budget exhausted')
  diagnostic=None
  if phase=='L_knob_approach':
   from approach_diagnostic import capture
   diagnostic=capture(self.emb)
   if not getattr(self,'diagnostic_topology_written',False):
    from approach_diagnostic import topology
    diagnostic['stage_collision_topology']=topology();self.diagnostic_topology_written=True
  # Public robot position control only. Bias integrates measured joint tracking
  # error; existing physical PD gains, torque/velocity limits stay unchanged.
  nominal=self.target.copy();command=nominal.copy();feedback_wrist=None;force_regulated=[False,False]
  if self.servo_enabled:
   feedback_wrist=[]
   for side in ['left','right']:
    sensor=self.emb._base.scene[side+'_pins_wrench'];raw_force=np.asarray(arr(sensor.data.force))
    if raw_force.shape!=(1,len(sensor.body_names),3) or not np.isfinite(raw_force).all():raise RuntimeError('Measured wrist sensor inventory/finite check failed')
    feedback_wrist.append(float(np.linalg.norm(raw_force[0,sensor.body_names.index('gripper')])))
   actual=np.asarray(self.obs.state['joint_pos'],float)
   for a in range(2):
    sl=slice(a*7,a*7+6)
    if self.unwinding[a]:self.servo_bias[a]=unwind_step(self.servo_bias[a])
    else:
     leg=self.q('legL_z') if a==0 and phase=='L_raise' else None
     pin=self.q('pinL_x') if a==0 and phase=='L_raise' else None
     force_regulated[a]=force_regulation_enabled(a,phase,leg,pin)
     self.servo_bias[a]=force_bias_step(self.servo_bias[a],nominal[sl]-actual[sl],feedback_wrist[a] if force_regulated[a] else 0.)
    from yam_arena.ik import LOWER,UPPER
    command[sl]=np.clip(nominal[sl]+self.servo_bias[a],LOWER,UPPER)
    self.servo_bias[a]=command[sl]-nominal[sl]
  self.pending_command=command.copy()
  r=self.emb.step(self.Action(data=command));self.obs=r.observation;self.record.append(dict(phase=phase,action=command.tolist(),nominal_action=nominal.tolist(),servo_bias=self.servo_bias.tolist(),feedback_wrist=feedback_wrist,force_regulated=force_regulated,info=r.info))
  if os.environ.get('PINS_ACTION_TRACE'):
   import kit
   row=dict(control=len(self.record),phase=phase,action=command,nominal_action=nominal,servo_bias=self.servo_bias,feedback_wrist=feedback_wrist,force_regulated=force_regulated,actual_joint_pos=self.obs.state['joint_pos'],status=kit.ENGINE.status())
   if diagnostic is not None:row.update(before_approach=diagnostic,after_approach=capture(self.emb))
   with open(os.environ['PINS_ACTION_TRACE'],'a') as f:f.write(dumps(row)+'\n')
  if r.terminated:raise RuntimeError('terminated: '+str(r.termination_reason))
 def pose(self,a,p,R,phase):
  from yam_arena.ik import solve,matrix_to_quat_xyzw
  q,pe,re=solve(p,matrix_to_quat_xyzw(R),q_init=self.target[a*7:a*7+6],base_pos=self.bases[a],iters=80,damping=.02)
  if pe>.003 or re>.03:
   q,pe,re=solve(p,matrix_to_quat_xyzw(R),q_init=q,base_pos=self.bases[a],iters=240,damping=.005)
  if pe>.003 or re>.03:raise RuntimeError(f'actual SDK IK rejects {phase}: {pe}m {re}rad')
  self.target[a*7:a*7+6]=q;self.poses[a]=np.array(p);self.rots[a]=R
 def move(self,a,p,R,secs,phase,jaw=None):
  from yam_arena.ik import slerp_matrix
  p0=self.poses[a].copy();R0=self.rots[a].copy();g=self.target[a*7+6];n=max(1,round(secs*20))
  for i in range(1,n+1):
   t=i/n;f=t*t*(3-2*t);self.pose(a,p0+(np.array(p)-p0)*f,slerp_matrix(R0,R,f),phase)
   if jaw is not None:self.target[a*7+6]=g+(jaw-g)*f
   self.tick(phase)
 def geom(self,name):
  asset=self.emb._base.scene['pins'];b=next(b for b in F['bodies'] if any(g['name']==name for g in b['geoms']));g=next(g for g in b['geoms'] if g['name']==name);i=asset.body_names.index(b['name']);return arr(asset.data.body_pos_w)[0,i]+rotation(arr(asset.data.body_quat_w)[0,i])@g['pos']
 def q(self,name):
  asset=self.emb._base.scene['pins'];return float(arr(asset.data.joint_pos)[0,asset.joint_names.index(name)])
 def joint_move(self,a,q,secs,phase,jaw=1.):
  from yam_arena.ik import tcp_pose
  start=self.target[a*7:a*7+6].copy();n=round(secs*20)
  for j in range(1,n+1):
   f=j/n;f=f*f*(3-2*f);self.target[a*7:a*7+6]=start+(q-start)*f;self.target[a*7+6]=jaw
   self.poses[a],self.rots[a]=tcp_pose(self.target[a*7:a*7+6],self.bases[a]);self.tick(phase)
 def grasp(self,a,p,R,label):
  if a==0 and label.startswith('L'):
   from yam_arena.ik import solve,matrix_to_quat_xyzw
   # Side entry keeps the forearm below the instrument. A yaw-first joint
   # waypoint clears the measured pedestal corner before the low approach.
   high=np.array(p)+[-.06,-.06,.12]
   q,pe,re=solve(high,matrix_to_quat_xyzw(R),q_init=[-2.128,3.084,1.568,-1.625,-1.013,-1.571],base_pos=self.bases[a],iters=300,damping=.005)
   if pe>.003 or re>.03:raise RuntimeError('side staging unreachable')
   home=self.target[:6].copy();yaw=home.copy();yaw[0]=q[0];self.knob_route=(home,yaw,q.copy(),high.copy())
   self.joint_move(0,yaw,2.,label+'_yaw_clear');self.joint_move(0,q,2.,label+'_fold_clear')
  self.move(a,p-R[:,2]*.06,R,1.5,label+'_approach',jaw=.6 if a==1 else 1.);self.move(a,p,R,1.,label+'_reach');self.move(a,p,R,.5,label+'_close',jaw=.12 if a==0 else 0.)
 def release(self,a,label):
  # v13r 1245: arm1 j4 bias at the .45 cap held a ~198 mm commanded-TCP preload
  # in the closed grip; opening released it into the leg (634 N, -0.336 m/s).
  # Only when wound (>=90% of a cap; actual L prefix max .82/.43, R pin .40):
  # unwind <=92 ticks with the jaw still closed, then open; poses unchanged.
  if self.servo_enabled and wound(self.servo_bias[a]):
   self.unwinding[a]=True
   for _ in range(92):
    if float(np.max(np.abs(self.servo_bias[a])))<1e-9:break
    self.tick(label+'_unwind')
   self.move(a,self.poses[a],self.rots[a],.5,label+'_open',jaw=.6 if a==1 else 1.);self.unwinding[a]=False
  else:self.move(a,self.poses[a],self.rots[a],.5,label+'_open',jaw=.6 if a==1 else 1.)
  self.move(a,self.poses[a]-self.rots[a][:,2]*.07,self.rots[a],1.,label+'_retreat')
  if a==0 and label.startswith('L'):
   home,yaw,q,high=self.knob_route
   self.move(0,high,self.rots[0],1.5,label+'_side_clear')
   self.joint_move(0,q,1.,label+'_staging_return');self.joint_move(0,yaw,2.,label+'_unfold_clear');self.joint_move(0,home,2.,label+'_yaw_return')
  elif a==0:
   self.move(0,self.poses[0]+[0,0,.25],self.rots[0],2.,label+'_raised_reset')
   self.joint_move(0,np.array([0.,1.,1.,-.6,0.,0.]),2.,label+'_home_reset')
 def axial_align(self,leg,RK):
  from yam_arena.ik import tcp_pose
  tcp,_=tcp_pose(np.asarray(self.obs.state['joint_pos'],float)[:6],self.bases[0])
  sensor=self.emb._base.scene['left_pins_wrench'];raw_force=np.asarray(arr(sensor.data.force))
  if raw_force.shape!=(1,len(sensor.body_names),3) or not np.isfinite(raw_force).all():raise RuntimeError('Measured axial wrist inventory/finite check failed')
  force=float(np.linalg.norm(raw_force[0,sensor.body_names.index('gripper')]))
  dy=axial_seat_step(float(tcp[1]),self.q('pin'+leg+'_x'),float(self.poses[0][1]),self.axial_corr,force)
  if dy==0.:return
  kp=self.poses[0].copy();kp[1]+=dy;self.pose(0,kp,RK,leg+'_reseat_track');self.axial_corr+=dy
 def reseat_align(self,leg,goal,RH,RK,corr,tight=False):
  e=self.q('leg'+leg+'_z')-goal
  # v17: v16 actual L hole3 tight stage measured leg response 18-94 um/tick to
  # 400 um steps and a +-250 um wall-to-wall limit cycle; tight step 100 um only.
  de=e-self.reseat_prev_e if self.reseat_prev_e is not None else 0.;self.reseat_prev_e=e
  dz=damped_reseat_step(e,de,corr) if tight else reseat_step(e,corr)
  if dz==0.:return corr
  hp=self.poses[1].copy();hp[2]+=dz;self.pose(1,hp,RH,leg+'_reseat_track')
  kn=self.geom('knob'+leg);kp=self.poses[0].copy();kp[0]=kn[0];kp[2]=float(np.clip(kn[2]-e,kp[2]-.0004,kp[2]+.0004));self.pose(0,kp,RK,leg+'_reseat_track')
  return corr+dz
 def run(self):
  B=np.array([[0,1,0],[-1,0,0],[0,0,1.]])
  # Jaw reversal is physically equivalent for the round knob/capsule handle;
  # select based on actual SDK IK, never guessed Panda joint coordinates.
  for hole in [2,3]:
   for leg in 'LR':
    kp=self.geom('knob'+leg);hp=self.geom('handle_'+leg);candidates=[B@np.array([[0,-1,0],[-1,0,0],[0,0,-1.]])@np.diag([s,s,1]) for s in [1,-1]]
    from yam_arena.ik import solve,matrix_to_quat_xyzw
    def best(a,p,rs):
     rows=[(solve(p,matrix_to_quat_xyzw(R),q_init=self.target[a*7:a*7+6],base_pos=self.bases[a],iters=150,damping=.02),R) for R in rs];valid=[z for z in rows if z[0][1]<=.003 and z[0][2]<=.03]
     if not valid:raise RuntimeError('no reachable grasp orientation')
     z=min(valid,key=lambda z:np.linalg.norm(z[1]-self.rots[a]));return z[1]
    RK=np.array([[0.,0.,1.],[0.,1.,0.],[-1.,0.,0.]]) if leg=='L' else best(0,kp,candidates);RH=best(1,hp,[B@np.diag([s,-s,-1.]) for s in [1,-1]])
    self.grasp(1,hp,RH,leg+'_support');self.grasp(0,kp,RK,leg+'_knob')
    self.servo_enabled=True
    # Let the unsaturated knob jaw and public position feedback settle under
    # measured contact before demanding spring work; no native state writes.
    for _ in range(40):self.tick(leg+'_closed_grip_track')
    if leg=='L':
     # Object-specific stronger grasp, only after the measured fingers center.
     robot=self.emb._base.scene['left_robot']
     fingers=arr(robot.data.joint_pos)[0,-2:]
     if abs(float(fingers[0]-fingers[1]))>.0005 or any(not -.0115<float(x)<-.0104 for x in fingers):
      raise RuntimeError('left knob not centered for stronger unsaturated grasp')
     self.move(0,self.poses[0],RK,1.,leg+'_knob_strengthen',jaw=.085)
    q0=self.q('leg'+leg+'_z')
    # Small handle lift unloads the seated pin through actual contact.
    self.move(1,self.poses[1]+[0,0,.0006],RH,1.,leg+'_unload')
    self.move(0,self.poses[0]+[0,.011,0],RK,1.5,leg+'_pull')
    # Measured-plateau overtravel (43971: hand reached +11 mm, pin plateaued
    # ~2 mm behind). Once per leg inside the unchanged 120-tick bound: at most
    # 2.5 mm extra public TCP travel over at most 10 controls, same smoothstep
    # profile as move(), stopping as soon as the unchanged -10.5 mm source goal
    # is met. `extra` is the travel actually commanded, used by the reseat.
    extra=0.;fired=False;hist=[]
    for k in range(120):
     pin=self.q('pin'+leg+'_x')
     if pin<=-.0105:break
     hist.append(pin)
     if not fired and k>=60 and len(hist)>10 and hist[-11]-pin<.0005:
      fired=True;plan=float(np.clip(pin+.011,0.,.0025))
      p0=self.poses[0].copy();R0=self.rots[0].copy()
      for i in range(1,11 if plan>0. else 1):
       if self.q('pin'+leg+'_x')<=-.0105:break
       t=i/10;f=t*t*(3-2*t);self.pose(0,p0+[0,plan*f,0],R0,leg+'_withdraw_overtravel');extra=plan*f
       self.tick(leg+'_withdraw_overtravel')
      if self.q('pin'+leg+'_x')<=-.0105:break
     self.tick(leg+'_withdraw_track')
    if self.q('pin'+leg+'_x')>-.0105:raise RuntimeError('physical pin did not withdraw; no software unlock')
    goal=.025*hole;start=self.poses[1].copy();startleft=self.poses[0].copy();n=max(1,round((goal-q0)/.008*20))
    for j in range(1,n+1):
     f=j/n;f=f*f*(3-2*f);dz=(goal-q0)*f;hnow=self.geom('handle_'+leg);kn=self.geom('knob'+leg)
     p=start.copy();p[:2]=hnow[:2];p[2]=start[2]+dz;self.pose(1,p,RH,leg+'_raise')
     p=startleft.copy();p[0]=kn[0];p[2]=kn[2];self.pose(0,p,RK,leg+'_follow');self.tick(leg+'_raise')
    self.move(0,self.poses[0]+[0,-.011-extra,0],RK,1.5,leg+'_seat')
    corr=0.;self.reseat_prev_e=None;self.axial_corr=0.
    for reseat_tick in range(80):
     if self.q('pin'+leg+'_x')>-.001:break
     # v13: measured leg-height hole alignment inside the SAME 80 ticks. Only
     # when the measured leg is nearest the CURRENT goal hole (|e|<=12.5 mm)
     # and outside 0.25 mm pin clearance; both hands step the same bounded dz
     # (<=0.4 mm/tick, <=6 mm total); left target is the real knob geometry
     # shifted to hole alignment so the knob grip no longer anchors the leg.
     corr=self.reseat_align(leg,goal,RH,RK,corr,tight=(reseat_tick>=20))
     if leg=='L':self.axial_align(leg,RK)
     self.tick(leg+'_reseat_track')
    if self.q('pin'+leg+'_x')<=-.001:raise RuntimeError('physical pin not reseated')
    self.release(0,leg+'_pin_release');self.release(1,leg+'_handle_release')
  for _ in range(24):self.tick('final_hands_off_hold')
  return self.emb._pins_done
