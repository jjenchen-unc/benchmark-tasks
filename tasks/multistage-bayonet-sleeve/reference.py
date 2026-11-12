"""Disclosed state-aware teacher. Only genuine14D20Hz SDK Actions change state.
No object poses, joints, latch, score, external force or simulator writes.
Candidate reference, not runtime/feasibility evidence until actually executed.
"""
import json,math
import numeric
import numpy as np
from scipy.spatial.transform import Rotation
from kinematics import solve,orientation
from scoring import CONTROL_LIMIT
from spring_ff import compensate
YAW0=-math.pi/2

def run(emb,observation,output,mode='reference'):
 from inspect_robots.types import Action
 from yam_arena.constants import TCP_OFFSET
 from contacts import arr,rot_xyzw
 import kit
 numeric.preflight(emb,observation,output)
 desired=np.asarray(observation.state['joint_pos'],float).copy();actual=desired.copy();robot=emb._base.scene['left_robot'];origin=arr(emb._base.scene.env_origins)[0];n=0;latest=observation;log=open(output/'robot-actions.jsonl','w',buffering=1);measure=open(output/'robot-physics.jsonl','w',buffering=1);phases=[];payload=False
 # Pure solver uses measured physical base placement; never changes base/object state.
 import kinematics
 kinematics.BASE=arr(robot.data.body_pos_w)[0,robot.body_names.index('base')]-origin
 def tcp():
  i=robot.body_names.index('gripper');R=rot_xyzw(arr(robot.data.body_quat_w)[0,i]);return arr(robot.data.body_pos_w)[0,i]-origin+R@np.array(TCP_OFFSET),R
 def sleeve():
  a=emb._base.scene['coupling_sleeve'];return arr(a.data.root_pos_w)[0]-origin,rot_xyzw(arr(a.data.root_quat_w)[0])
 def step(label):
  nonlocal n,actual,latest
  if n>=CONTROL_LIMIT:raise RuntimeError('6000control budget exhausted')
  if desired.shape!=(14,) or not np.isfinite(desired).all():raise ValueError('invalid14D')
  # Read-only telemetry is serialized before the Action, never scene writes.
  command=desired.copy()
  if payload:
   command[:6],ff=compensate(desired[:6],kinematics.BASE,kit._OBSERVER.last)
  else:ff=None
  raw={}
  for side in ('left','right'):
   a=emb._base.scene[side+'_robot'];names=list(a.body_names);i=names.index('gripper');positions=arr(a.data.body_pos_w)[0];quats=arr(a.data.body_quat_w)[0];Rg=rot_xyzw(quats[i]);raw[side]=dict(body_names=names,raw8q=arr(a.data.joint_pos)[0],body_positions_w=positions,body_quaternions_xyzw=quats,tcp=positions[i]-origin+Rg@np.array(TCP_OFFSET))
  log.write(numeric.dumps(dict(event='before_action',control=n+1,phase=label,action=command,nominal_action=desired,load_feedforward=ff,raw_robots=raw,status=kit.current_status(),observer_last=kit._OBSERVER.last))+'\n')
  before=kit._OBSERVER.history.samples;r=emb.step(Action(data=command.copy()));n+=1;latest=r.observation;actual=np.asarray(latest.state['joint_pos'],float)
  log.write(numeric.dumps(dict(control=n,phase=label,action=command.tolist(),status=kit.current_status()))+'\n')
  for row in kit._OBSERVER.rows:measure.write(numeric.dumps(dict(control=n,phase=label,measurement=row))+'\n')
  if kit._OBSERVER.history.samples-before!=25:raise RuntimeError('expected25 physical substeps')
  if r.terminated:raise RuntimeError(str(r.termination_reason))
 def move(p,yaw,jaw,count,label,check=True):
  p=np.asarray(p,float);p0,R0=tcp();j0=desired[6];R=orientation(yaw);rv=Rotation.from_matrix(R@R0.T).as_rotvec();count=max(count,math.ceil(1.5*np.linalg.norm(rv)/.45*20),math.ceil(1.5*np.linalg.norm(p-p0)/.08*20));phases.append(dict(phase=label,start_control=n,count=count,target=p.tolist(),yaw=yaw))
  for i in range(count):
   t=(i+1)/count;f=t*t*(3-2*t);Ri=Rotation.from_rotvec(rv*f).as_matrix()@R0;q,ep,er=solve(p0+(p-p0)*f,Ri,actual[:6])
   if ep>.001 or er>.01:raise RuntimeError('IK waypoint unreachable '+str((label,ep,er)))
   desired[:6]=q;desired[6]=j0+(jaw-j0)*f;step(label)
  if check and np.linalg.norm(tcp()[0]-p)>.015:raise RuntimeError('physical TCP tracking error >15mm at '+label)
 def jaw(value,count,label):
  start=desired[6]
  for i in range(count):desired[6]=start+(value-start)*(i+1)/count;step(label)
 def wait(count,label):
  for _ in range(count):step(label)
 try:
  if mode=='noop':wait(100,'noop_negative')
  else:
   p,R=sleeve();move(p+[0,0,.172],YAW0,1.,80,'approach');move(p+[0,0,.052],YAW0,1.,60,'descend');jaw(0.,40,'close_40N_fingers');wait(15,'grip_settle');payload=True
   sp,sR=sleeve();tp,tR=tcp();off=tR.T@(tp-sp);seat=sp.copy();seat[:2]=np.array(kit.FIX['stand_world'])[:2];seat[2]-=.002;bx=bth=0.;lat=np.zeros(2)
   path=kit.FIX['parameters']['path_C']
   if mode=='straight-pull':path=[[0.,0.],[.1,0.]]
   if mode=='wrong-turn':path=[[0.,0.],[-.012,0.],[-.012,-90.],[.006,-90.],[.006,-70.],[.1,-70.]]
   for idx,((x0,t0),(x,t)) in enumerate(zip(path,path[1:])):
    if t==t0:bx=0.
    def target():
     yaw=YAW0-math.radians(t+bth);return seat+[lat[0],lat[1],x+bx]+orientation(yaw)@off,yaw
    pos,yaw=target();count=max(30,math.ceil(abs(x-x0)/.025*20),math.ceil(abs(t-t0)/18*20));move(pos,yaw,0.,count,'stage_'+str(idx),check=mode not in ('straight-pull','wrong-turn'))
    if mode in ('straight-pull','wrong-turn'):continue
    if x<.1:
     for aim in range(6):
      sp,sR=sleeve();ex=x-(sp[2]-seat[2]);turn=-math.degrees(math.atan2(sR[1,0],sR[0,0]));et=t-turn
      if abs(ex)<.0007 and abs(et)<1 and np.linalg.norm(sp[:2]-seat[:2])<.001:break
      bx=float(np.clip(bx+ex,-.005,.005));bth=float(np.clip(bth+et,-6.,6.));lat=np.clip(lat+seat[:2]-sp[:2],-.006,.006);pos,yaw=target();move(pos,yaw,0.,30,'aim_'+str(idx))
     sp,sR=sleeve()
     if abs(x-(sp[2]-seat[2]))>.004 or abs(t+math.degrees(math.atan2(sR[1,0],sR[0,0])))>5.:raise RuntimeError('mechanical stage did not follow genuine robot controls')
   if mode in ('straight-pull','wrong-turn'):
    wait(40,'negative_hold');return dict(mode=mode,controls=n,status=kit.current_status(),full_goal_demonstrated=False,model_calls=0)
   sp,sR=sleeve();tp,tR=tcp()
   if sp[2]-seat[2]<.085:raise RuntimeError('sleeve not physically removed')
   # Grasp-offset compensation is read-only planning, never an object write.
   offz=tp[2]-sp[2];tray=np.array([.05,.26,.812])+np.array(kit.FIX['translation']);up=max(tp[2]+.03,tray[2]+offz+.10)
   move([tp[0],tp[1],up],YAW0+math.pi/6,0.,50,'lift_clear');move([tray[0],tray[1],up],YAW0,0.,100,'carry_to_tray');move(tray+[0,0,offz+.02],YAW0,0.,70,'tray_descend');move(tray+[0,0,offz+.001],YAW0,0.,70,'tray_rest');payload=False;jaw(1.,40,'release');move([tray[0],tray[1],up+.06],YAW0,1.,70,'retreat');wait(100 if mode=='short-hold' else 440,'hands_off_hold')
  return dict(kind='genuine14D20Hz robot candidate',mode=mode,controls=n,status=kit.current_status(),full_goal_demonstrated=kit.current_score()==1.,model_calls=0)
 finally:
  log.close();measure.close();(output/'reference-phases.json').write_text(numeric.dumps(phases,indent=2)+'\n')
