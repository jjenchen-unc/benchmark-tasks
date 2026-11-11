"""Disclosed state-aware genuine14D20Hz robot reference ATTEMPT.
Selects the holding lens from fixed-case simulator parameters; plans actual SDK
IK using measured transforms. No task body/joint pose, effort, latch or history
writes. This whole-goal controller is unvalidated and may stop on physical/IK
failure. Its handoff, loaded flip, capture and ring grasp require actual proof.
"""
from pathlib import Path
import json,math
import numpy as np
from mechanics import rotation
from kit import arr

def run(out,case=None):
 from inspect_robots import Scene
 from inspect_robots.types import Action
 from yam_arena.inspect_embodiment import yam_arena_embodiment
 from yam_arena import ik
 from yam_arena.constants import TCP_OFFSET
 import kit
 from tracking import Tracking
 emb=yam_arena_embodiment(task=str(Path(__file__).parent/'task.py'),device='cpu',control_hz=20,object_jitter=0,cam_width=320,cam_height=176)
 controls=[];phases=[];obs=None;result=None;E=None;o=None
 jaw_command={"left":1.,"right":1.};loaded_rel={};nominal_targets={}
 Rside=np.array([[-1.,0,0],[0,0,-1.],[0,-1.,0]]) # baseward approach, jaw opening along world X
 def rz(a):return np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1.]])
 def tool(side):
  a=emb._base.scene[side+'_robot'];i=a.body_names.index('gripper');R=rotation(arr(a.data.body_quat_w)[0,i]);return arr(a.data.body_pos_w)[0,i]+R@np.array(TCP_OFFSET),R
 def telemetry():
  # Privileged observations only; public Action remains 14D.
  robots={}
  for side in ['left','right']:
   a=emb._base.scene[side+'_robot']
   robots[side]=dict(joint_names=list(a.joint_names),raw_joint_pos=arr(a.data.joint_pos)[0].tolist(),body_names=list(a.body_names),body_pos_w=arr(a.data.body_pos_w)[0].tolist(),body_quat_xyzw=arr(a.data.body_quat_w)[0].tolist())
   state={}
   for name in ['joint_vel','applied_torque','computed_torque','gravity_compensation_forces']:
    try:state[name]=arr(getattr(a.data,name))[0].tolist()
    except AttributeError:state[name]=None
   robots[side]['drive_state']=state
  objects={}
  for name in ['body_left','body_right','lens_blue','lens_purple','lens_orange']:
   a=E.assets[name]
   objects[name]=dict(body_names=list(a.body_names),body_pos_w=arr(a.data.body_pos_w)[0].tolist(),body_quat_xyzw=arr(a.data.body_quat_w)[0].tolist())
  return dict(robots=robots,objects=objects,force_observation=E.last,normal_contact_observation={name:dict(selection=c.selection,contacts=c.last_contacts) for name,c in getattr(E,'robot_contacts',{}).items()})
 def send(q,label):
  nonlocal obs,result
  if len(controls)>=3034:raise RuntimeError('Reference reached original3034control limit')
  if E.last and max(E.last['wrist_n'])>30.:raise RuntimeError('Reference default30N force guard stopped motion; task80N100ms abort is separate')
  q=np.asarray(q,float)
  if q.shape!=(14,) or not np.isfinite(q).all():raise ValueError('Invalid robot Action')
  controls.append(dict(control=len(controls)+1,phase=label,action=q.tolist()));result=emb.step(Action(data=q.copy()));obs=result.observation
  with (out/'robot-journal.jsonl').open('a') as journal:
   journal.write(json.dumps(dict(control=len(controls),phase=label,action=q.tolist(),measured_joint_pos=np.asarray(obs.state['joint_pos'],float).tolist(),telemetry=telemetry(),status=E.status()),allow_nan=False)+'\n')
  if result.terminated:raise StopIteration(result.termination_reason)
 def phase(label):
  phases.append(dict(phase=label,control=len(controls),status=E.status()));(out/'progress.json').write_text(json.dumps(phases[-1],indent=2,allow_nan=False))
 def move(side,pos,R,seconds,label,grip=None):
  phase(label);start=np.asarray(obs.state['joint_pos'],float);idx=0 if side=='left' else 7;current=start[idx:idx+6].copy();g0=jaw_command[side];g1=g0 if grip is None else grip
  base=emb.rig.bases[side];offset=np.zeros(6)
  loaded=side in loaded_rel and jaw_command[side]<.9 and side in nominal_targets
  if loaded:
   # Plan from the previous intended physical endpoint, retaining its loaded
   # command-minus-nominal offset. Measured state remains IK/feedback input.
   p0,R0=ik.tcp_pose(nominal_targets[side],base)
   offset=np.asarray(controls[-1]['action'],float)[idx:idx+6]-nominal_targets[side]
   if np.any(np.abs(offset)>.2+1e-12):raise RuntimeError(label+': held command offset outside finite envelope')
  else:p0,R0=tool(side)
  n=max(1,round(seconds*20))
  for j in range(n):
   s=ik.smoothstep((j+1)/n);p=p0+s*(np.array(pos)-p0);A=ik.slerp_matrix(R0,R,s);current,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(A),current,base_pos=base,iters=100,damping=.025)
   if pe>.003 or re>.03:raise RuntimeError(f'{label}: actual SDK IK error {pe:.6f}m/{re:.6f}rad at{j}/{n}')
   commanded=current+offset
   if np.any(commanded<ik.LOWER) or np.any(commanded>ik.UPPER):raise RuntimeError(label+': held-offset command outside public joint limits')
   nominal_targets[side]=current.copy()
   q=np.asarray(controls[-1]['action'] if controls else obs.state['joint_pos'],float).copy();q[idx:idx+6]=commanded;q[6]=jaw_command['left'];q[13]=jaw_command['right'];q[idx+6]=g0+s*(g1-g0);send(q,label)
  jaw_command[side]=g1
  if label in ('right_loaded_half_flip','left_loaded_flip_complete'):
   # Actual v16 retained the lens, but ended26.82mm behind the legal IK goal.
   # Settle through bounded public joint targets BEFORE the original20mm check.
   from endpoint_control import LoadedEndpoint
   target=np.asarray(controls[-1]['action'],float).copy()
   servo=LoadedEndpoint(current,target[idx:idx+6],np.asarray(obs.state['joint_pos'],float)[idx:idx+6],base)
   stable=0
   for _ in range(120):
    measured_R,measured_A=tool(side)
    stable=stable+1 if np.linalg.norm(measured_R-np.asarray(pos))<=.003 and np.linalg.norm(ik._rot_error(R,measured_A))<=.03 else 0
    if stable>=5:break
    target[idx:idx+6]=servo.update(np.asarray(obs.state['joint_pos'],float)[idx:idx+6])
    send(target,label+'_endpoint_settle');retention(side,o,loaded_rel[side],label+'_endpoint_settle')
   else:raise RuntimeError(label+': bounded endpoint correction exhausted')
  measured,_=tool(side)
  if np.linalg.norm(measured-np.array(pos))>.02:raise RuntimeError(label+': robot tracking error>20mm; no loaded success claim')
 def fingers(side,g,seconds,label):
  p,R=tool(side);move(side,p,R,seconds,label,g)
 def grasp_transform(side,o):
  p,R=tool(side);lp,lR=E.pose(o,o);return R.T@(lp-p),R.T@lR
 def retention(side,o,rel,label):
  rp,rR=grasp_transform(side,o);dp=float(np.linalg.norm(rp-rel[0]));dr=float(np.linalg.norm(ik._rot_error(rR,rel[1])))
  if dp>.005 or dr>.08:raise RuntimeError(f'{label}: lost grasp relative pose {dp:.6f}m/{dr:.6f}rad')
 def compression(side,label):
  # Independent raw positions, never public averaged gap. K*error is only a
  # quasi-static drive estimate; force/contact calibration remains unverified.
  a=emb._base.scene[side+'_robot'];raw=arr(a.data.joint_pos)[0]
  errors=np.array([-float(raw[a.joint_names.index(n)])-.04695*jaw_command[side] for n in ['joint7','joint8']])
  if not np.isfinite(errors).all() or np.any(errors<.001) or np.any(errors>.0075):raise RuntimeError(label+': no bounded bilateral finger compression '+str(errors.tolist()))
 def quiet(side,o,rel,label):
  phase(label)
  for _ in range(10):
   q=np.asarray(controls[-1]['action'],float).copy();q[6]=jaw_command['left'];q[13]=jaw_command['right'];send(q,label);retention(side,o,rel,label)
  compression(side,label)
 def obj_move(side,o,pos,R,rel,seconds,label):
  retention(side,o,rel,label+'_before')
  rp,rR=rel;A=R@rR.T;move(side,np.array(pos)-A@rp,A,seconds,label)
  retention(side,o,rel,label+'_after')
  if label in ('nose_down_preseat','nose_down_insert'):
   align_loaded(side,o,np.asarray(pos),R,label+'_alignment',cap=100)
 def align_loaded(side,o,target_p,target_R,label,cap=100):
  # Keep the previous loaded command offset across the physical-goal change.
  # Replanning from measured joints alone erased24mm of gravity-load offset.
  retention(side,o,loaded_rel[side],label+"_before")
  phase(label)
  from endpoint_control import LoadedEndpoint
  p,R=tool(side);rp,rR=grasp_transform(side,o)
  wanted_R=target_R@rR.T;wanted_p=np.asarray(target_p)-wanted_R@rp
  idx=0 if side=='left' else 7;base=emb.rig.bases[side]
  measured=np.asarray(obs.state['joint_pos'],float)[idx:idx+6]
  nominal,pe,re=ik.solve(wanted_p,ik.matrix_to_quat_xyzw(wanted_R),measured,base_pos=base,iters=100,damping=.025)
  if pe>.003 or re>.03:raise RuntimeError(label+': measured-object correction has no legal IK')
  q=np.asarray(controls[-1]['action'],float).copy()
  servo=LoadedEndpoint(nominal,q[idx:idx+6],measured,base)
  for j in range(cap):
   retention(side,o,loaded_rel[side],label)
   lp,lR=E.pose(o,o)
   if np.linalg.norm(np.asarray(target_p)-lp)<.0015 and np.linalg.norm(ik._rot_error(target_R,lR))<.015:return
   q[idx:idx+6]=servo.update(np.asarray(obs.state['joint_pos'],float)[idx:idx+6])
   nominal_targets[side]=nominal.copy()
   q[6]=jaw_command['left'];q[13]=jaw_command['right'];send(q,label)
  raise RuntimeError(label+': actual lens failed bounded measured-pose alignment')
 def settle_proof_lift(side,o,rel):
  label='right_proof_lift_integral';phase(label)
  nominal=np.asarray(controls[-1]['action'],float).copy();target=nominal.copy()
  idx=0 if side=='left' else 7
  wanted_p,wanted_R=ik.tcp_pose(nominal[idx:idx+6],emb.rig.bases[side])
  servo=Tracking(nominal[idx:idx+6],np.asarray(obs.state['joint_pos'],float)[idx:idx+6]);stable=0
  for _ in range(60):
   measured=np.asarray(obs.state['joint_pos'],float)[idx:idx+6]
   target[idx:idx+6]=servo.update(measured)
   # Public joint limits checked before dispatch; no effort/gain/task writes.
   if np.any(target[idx:idx+6]<ik.LOWER) or np.any(target[idx:idx+6]>ik.UPPER):
    raise RuntimeError(label+': compensated target outside public joint limits')
   send(target,label);retention(side,o,rel,label);compression(side,label)
   p,R=tool(side)
   stable=stable+1 if np.linalg.norm(p-wanted_p)<=.003 and np.linalg.norm(ik._rot_error(R,wanted_R))<=.03 else 0
   if stable>=5:return target
  raise RuntimeError(label+': bounded integral tracking failed')
 def check_loaded_flip(side,o,pos,R,rel,seconds):
  # Check from the current measured arm/grasp before relinquishing support.
  p0,R0=tool(side);rp,rR=rel;A=R@rR.T;p1=np.asarray(pos)-A@rp
  idx=0 if side=='left' else 7;cur=np.asarray(obs.state['joint_pos'],float)[idx:idx+6].copy()
  for j in range(round(seconds*20)):
   s=ik.smoothstep((j+1)/round(seconds*20));B=ik.slerp_matrix(R0,A,s)
   cur,pe,re=ik.solve(p0+s*(p1-p0),ik.matrix_to_quat_xyzw(B),cur,base_pos=emb.rig.bases[side],iters=100,damping=.025)
   if pe>.003 or re>.03:raise RuntimeError('Measured loaded flip infeasible before right release')
 def retreat(side,seconds,label,distance=.10):
  p,R=tool(side);move(side,p-distance*R[:,2],R,seconds,label)
 try:
  obs=emb.reset(Scene(id='B6-scripted-reference',instruction='Mount a compatible zoom lens nose-down, lock, set50mm, let go for20s. Disclosed simulator-state teacher.'),seed=42);E=kit.ENGINE
  if E.error:raise RuntimeError(E.error)
  params=E.f['parameters'];o=max(['lens_blue','lens_purple'],key=lambda x:params['zoom_friction_'+x.split('_')[1]])
  lp,lR=E.pose(o,o);pickup_R=np.diag([1.,-1,-1]);ring=lp+lR[:,2]*.060
  move('right',ring+[0,0,.12],pickup_R,5,'right_pick_approach',1.)
  move('right',ring,pickup_R,4,'right_grip_ring',1.);fingers('right',.55,2,'right_close');compression('right','right_close');rightrel=grasp_transform('right',o);loaded_rel['right']=rightrel
  obj_move('right',o,lp+[0,0,.020],lR,rightrel,1.,'right_proof_lift')
  pickup_hold=settle_proof_lift('right',o,rightrel)
  lifted,_=E.pose(o,o)
  if lifted[2]-lp[2]<.015:raise RuntimeError('Pickup failed: lens did not lift15mm off tray')
  phase('right_pickup_retention')
  for _ in range(10):
   send(pickup_hold,'right_pickup_retention');retention('right',o,rightrel,'right_pickup_retention');compression('right','right_pickup_retention')
  if case=='first-lift':raise StopIteration('bounded_first_lift_complete_native_goal_not_attempted')
  obj_move('right',o,lp+[0,0,.12],lR,rightrel,3,'right_lift')
  handoff=np.array([.35,0.,.95]);obj_move('right',o,handoff,np.eye(3),rightrel,6,'right_handoff_carry')
  # Share inversion with a 75deg right pitch, leaving 15deg correction reserve.
  # Left takes a measured-axis lateral barrel grasp for the remaining105deg.
  half_angle=math.radians(75.) # 15deg right pitch reserve for measured grasp correction
  half_flip=np.array([[1.,0,0],[0,math.cos(half_angle),-math.sin(half_angle)],[0,math.sin(half_angle),math.cos(half_angle)]])
  obj_move('right',o,handoff,half_flip,rightrel,4,'right_loaded_half_flip')
  align_loaded('right',o,handoff,half_flip,'right_measured_half_flip_alignment')
  actual_handoff,actual_half=E.pose(o,o);left_tcp=actual_handoff+actual_half[:,2]*.035
  approach=np.array([1.,0,0]);jaw=-np.cross(approach,actual_half[:,2]);jaw=jaw/np.linalg.norm(jaw)
  hand_R=np.column_stack((jaw,np.cross(approach,jaw),approach))
  move('left',left_tcp-hand_R[:,2]*.06,hand_R,5,'left_handoff_approach',1.);move('left',left_tcp,hand_R,4,'left_handoff_grip',1.);fingers('left',.55,2,'left_close_barrel');compression('left','left_close_barrel');leftrel=grasp_transform('left',o);loaded_rel['left']=leftrel
  mp,mR=E.pose('body_right','body_right')
  check_loaded_flip('left',o,np.array([.35,0.,1.010]),mR,leftrel,4)
  fingers('right',1.,2,'right_release');quiet('left',o,leftrel,'left_handoff_retention')
  p,R=tool('right');move('right',p+[0.,-.04,0.],R,2,'right_handoff_clearance')
  p,R=tool('right');move('right',p+[0.,0.,.12],R,2,'right_handoff_retreat')
  mp,mR=E.pose('body_right','body_right');goal=mp+mR[:,2]*.0006
  # Actual held-grasp/load-margin scan selects a60mm lift:1.010m centre.
  # This controller waypoint admits the observed load offset below joint4 stop.
  leftrel=grasp_transform('left',o)
  check_loaded_flip('left',o,np.array([.35,0.,1.010]),mR,leftrel,4)
  obj_move('left',o,np.array([.35,0.,1.010]),mR,leftrel,4,'left_loaded_flip_complete')
  ring_approach_R=tool('left')[1].copy()
  obj_move('left',o,goal+mR[:,2]*.06,mR,leftrel,5,'nose_down_preseat')
  obj_move('left',o,goal,mR,leftrel,4,'nose_down_insert')
  # Modest measured-state push until genuine feet/face load engages capture.
  for j in range(5):
   if E.cap['body_right']==o:break
   obj_move('left',o,goal-mR[:,2]*(j+1)*.0005,mR,leftrel,.5,'seat_contact_push')
  if E.cap['body_right']!=o:raise RuntimeError('No physical contact capture; abort without changing latch')
  d=1 if params['lock_dir_R']=='ccw' else -1;angle=d*math.radians(params['lock_angle_R']+1.2)
  for a in np.linspace(0,angle,40)[1:]:
   obj_move('left',o,goal,mR@rz(a),leftrel,.2,'bayonet_turn')
   if E.pin['body_right']:break
  if not E.pin['body_right']:raise RuntimeError('Physical lock not engaged')
  barrel_grasp_R=tool('left')[1].copy()
  fingers('left',1.,2,'left_release_barrel')
  p,R=tool('left');move('left',p+[0,0,-.12],R,4,'left_leave_barrel')
  # Read the actual ring body pose and regrasp the ring without moving barrel.
  rp,rR=E.pose(o,o+'_zoomring');ring_tcp=rp+rR[:,2]*.025;A=rz(math.pi/6)@ring_approach_R
  move('left',ring_tcp-A[:,2]*.06,A,4,'zoom_approach',1.);move('left',ring_tcp,A,3,'zoom_grasp',1.);fingers('left',.55,2,'zoom_close');compression('left','zoom_close')
  graspP,graspR=tool('left');zrP,zrR=E.pose(o,o+'_zoomring');relP=graspR.T@(zrP-graspP);relR=graspR.T@zrR;target=(50-18)/37*math.pi/2
  z0,_=E.joint(o,o+'_zoom');lp,lR=E.pose(o,o)
  for a in np.linspace(z0,target,30)[1:]:
   wanted=lR@rz(a);T=wanted@relR.T
   ring_goal=lp+lR[:,2]*(.05+.020*a/(math.pi/2))
   move('left',ring_goal-T@relP,T,.2,'zoom_turn')
  fingers('left',1.,2,'zoom_release');retreat('left',4,'zoom_retreat')
  phase('final_hands_off_20s');q=np.asarray(obs.state['joint_pos'],float)
  for _ in range(401):send(q,'final_hands_off_20s')
 except StopIteration as e:phases.append(dict(termination=str(e),control=len(controls)))
 except Exception as e:phases.append(dict(failure=type(e).__name__+': '+str(e),control=len(controls)))
 finally:
  actual=E.status() if E else None;raw=actual['measured_rubric']['score'] if actual else 0.
  (out/'actions.json').write_text(json.dumps(controls,allow_nan=False));(out/'phases.json').write_text(json.dumps(phases,indent=2,allow_nan=False));emb.close()
 return dict(state='attempt_completed',controller_scope=__doc__,selected_lens=o if E else None,controls=len(controls),actual_status=actual,measured_rubric_score=raw,task_score=actual['score'] if actual else 0.,full_goal_verified=False,raw_measured_goal=bool(actual and raw==1. and not actual['failure']),physics_verified=False,last_phase=phases[-1] if phases else None)
