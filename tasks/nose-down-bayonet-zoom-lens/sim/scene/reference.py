"""Scripted demo: open loop from the published layout, 14D actions at 20 Hz.

It uses the blue lens. At run time it reads only object and robot poses from
the simulator (what a camera sees), the robot's own joint positions and the
step count. It never reads friction or other parameters, contact or wrist
forces, or the task's internal capture/pin state. Its checks are pose
comparisons: the lock is reached when the overhead mount ring has turned at
least 57.15 degrees (published 56.4 degree lock plus 0.75), and the zoom setting
comes from the zoom ring angle relative to the barrel.

case='probe:<lens>' is the observability probe (sim/probe.py): before the
route starts, that lens and the blue lens swap start positions (a pose write
by the probe, never by the demo), the same route trial-fits it, and after
release the zoom ring and front element are watched for 5 s.
"""
from pathlib import Path
import json,math
import numpy as np
from mechanics import rotation
from kit import arr

TWIST=(50-18)/37*math.pi/2
FRONT0=.07
LOCK_DEG=56.4 # published lock angle of the overhead mount
LOCK_DIR=-1 # the overhead mount turns clockwise about its own axis

def run(out,case=None):
 from inspect_robots import Scene
 from inspect_robots.types import Action
 from yam_arena.inspect_embodiment import yam_arena_embodiment
 from yam_arena import ik
 from yam_arena.constants import TCP_OFFSET
 from tracking import Tracking
 emb=yam_arena_embodiment(task=str(Path(__file__).parent/'task.py'),device='cpu',control_hz=20,object_jitter=0,cam_width=320,cam_height=176)
 controls=[];phases=[];decisions=[];obs=None;result=None;o=None
 jaw_command={"left":1.,"right":1.};loaded_rel={};nominal_targets={};moving_load_ready=False
 def rz(a):return np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1.]])
 def pose(asset,body):
  a=emb._base.scene[asset];i=a.body_names.index(body);return arr(a.data.body_pos_w)[0,i],rotation(arr(a.data.body_quat_w)[0,i])
 def yaw_between(Ra,Rb):M=Ra.T@Rb;return math.atan2(M[1,0],M[0,0])
 def ring_angle(lens):return yaw_between(pose(lens,lens)[1],pose(lens,lens+'_zoomring')[1])
 def focal(lens):return 18+37*ring_angle(lens)/(math.pi/2)
 def extension(lens):pb,Rb=pose(lens,lens);pf,_=pose(lens,lens+'_front');return float((Rb.T@(pf-pb))[2]-FRONT0)
 def mount_turn_deg():return LOCK_DIR*math.degrees(yaw_between(pose('body_right','body_right')[1],pose('body_right','body_right_ring')[1]))
 def decide(**row):
  row['control']=len(controls);decisions.append(row);print('B6_DECISION '+json.dumps(row),flush=True)
  (out/'decisions.json').write_text(json.dumps(decisions,indent=2))
 def tool(side):
  a=emb._base.scene[side+'_robot'];i=a.body_names.index('gripper');R=rotation(arr(a.data.body_quat_w)[0,i]);return arr(a.data.body_pos_w)[0,i]+R@np.array(TCP_OFFSET),R
 def send(q,label):
  nonlocal obs,result
  if len(controls)>=3034:raise RuntimeError('Reference reached 3034 control limit')
  q=np.asarray(q,float)
  if q.shape!=(14,) or not np.isfinite(q).all():raise ValueError('Invalid robot Action')
  controls.append(dict(control=len(controls)+1,phase=label,action=q.tolist()));result=emb.step(Action(data=q.copy()));obs=result.observation
  if result.terminated:raise StopIteration(result.termination_reason)
 def phase(label):
  phases.append(dict(phase=label,control=len(controls)));(out/'progress.json').write_text(json.dumps(phases[-1],indent=2,allow_nan=False))
 def move(side,pos,R,seconds,label,grip=None):
  phase(label);start=np.asarray(obs.state['joint_pos'],float);idx=0 if side=='left' else 7;current=start[idx:idx+6].copy();g0=jaw_command[side];g1=g0 if grip is None else grip
  base=emb.rig.bases[side];offset=np.zeros(6)
  loaded=side in loaded_rel and jaw_command[side]<.9 and side in nominal_targets
  if loaded:
   p0,R0=ik.tcp_pose(nominal_targets[side],base)
   offset=np.asarray(controls[-1]['action'],float)[idx:idx+6]-nominal_targets[side]
   if np.any(np.abs(offset)>.2+1e-12):raise RuntimeError(label+': held command offset outside finite envelope')
  else:p0,R0=tool(side)
  n=max(1,round(seconds*20));moving_servo=None
  if moving_load_ready and loaded and grip is None:
   from endpoint_control import MovingLoad
   moving_servo=MovingLoad(nominal_targets[side],np.asarray(controls[-1]['action'],float)[idx:idx+6],current,base)
  for j in range(n):
   s=ik.smoothstep((j+1)/n);p=p0+s*(np.array(pos)-p0);A=ik.slerp_matrix(R0,R,s);current,pe,re=ik.solve(p,ik.matrix_to_quat_xyzw(A),current,base_pos=base,iters=100,damping=.025)
   if pe>.003 or re>.03:raise RuntimeError(f'{label}: actual SDK IK error {pe:.6f}m/{re:.6f}rad at{j}/{n}')
   commanded=current+offset if moving_servo is None else moving_servo.update(current,np.asarray(obs.state['joint_pos'],float)[idx:idx+6],p,ik.quat_xyzw_to_matrix(ik.matrix_to_quat_xyzw(A)))
   if moving_servo is not None and not np.array_equal(moving_servo.nominal,current):
    current=moving_servo.nominal.copy();accepted_p,accepted_R=ik.tcp_pose(current,base)
    if np.linalg.norm(accepted_p-p)>.003 or np.linalg.norm(ik._rot_error(ik.quat_xyzw_to_matrix(ik.matrix_to_quat_xyzw(A)),accepted_R))>.03:raise RuntimeError(label+': accepted moving nominal outside original IK guard')
   if np.any(commanded<ik.LOWER) or np.any(commanded>ik.UPPER):raise RuntimeError(label+': held-offset command outside public joint limits')
   nominal_targets[side]=current.copy()
   q=np.asarray(controls[-1]['action'] if controls else obs.state['joint_pos'],float).copy();q[idx:idx+6]=commanded;q[6]=jaw_command['left'];q[13]=jaw_command['right'];q[idx+6]=g0+s*(g1-g0);send(q,label)
  jaw_command[side]=g1
  if label in ('right_loaded_half_flip','left_loaded_flip_complete'):
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
  if np.linalg.norm(measured-np.array(pos))>.02:raise RuntimeError(label+': robot tracking error>20mm')
 def fingers(side,g,seconds,label):
  if label not in ('left_release_barrel','zoom_release'):
   p,R=tool(side);return move(side,p,R,seconds,label,g)
  target=np.asarray(controls[-1]['action'],float).copy();idx=6 if side=='left' else 13
  phase(label);start_p,start_R=tool(side);g0=float(target[idx]);n=max(1,round(seconds*20))
  from endpoint_control import MovingLoad
  arm=0 if side=='left' else 7;base=emb.rig.bases[side]
  nominal=np.asarray(nominal_targets[side],float).copy()
  if label=='zoom_release':nominal=np.asarray(obs.state['joint_pos'],float)[arm:arm+6].copy()
  hold_p,hold_R=ik.tcp_pose(nominal,base)
  servo=MovingLoad(nominal,target[arm:arm+6],np.asarray(obs.state['joint_pos'],float)[arm:arm+6],base)
  opening_ramp_controls=10 if label=='zoom_release' else n
  for j in range(n):
   q=target.copy();q[arm:arm+6]=servo.update(nominal,np.asarray(obs.state['joint_pos'],float)[arm:arm+6],hold_p,hold_R)
   fraction=min((j+1)/opening_ramp_controls,1.)
   q[idx]=g0+ik.smoothstep(fraction)*(g-g0);send(q,label)
  jaw_command[side]=g
  measured,_=tool(side)
  if np.linalg.norm(measured-start_p)>.02:raise RuntimeError(label+': robot tracking error>20mm')
  if label=='zoom_release':nominal_targets[side]=servo.nominal.copy()
 def hold(label,n):
  phase(label);q=np.asarray(controls[-1]['action'],float).copy()
  for _ in range(n):send(q,label)
 def grasp_transform(side,lens):
  p,R=tool(side);lp,lR=pose(lens,lens);return R.T@(lp-p),R.T@lR
 def retention(side,lens,rel,label):
  rp,rR=grasp_transform(side,lens);dp=float(np.linalg.norm(rp-rel[0]));dr=float(np.linalg.norm(ik._rot_error(rR,rel[1])))
  if dp>.005 or dr>.08:raise RuntimeError(f'{label}: lost grasp relative pose {dp:.6f}m/{dr:.6f}rad')
 def compression(side,label):
  # Robot's own finger joint positions: the jaws stopped on the part, not closed on air.
  a=emb._base.scene[side+'_robot'];raw=arr(a.data.joint_pos)[0]
  errors=np.array([-float(raw[a.joint_names.index(n)])-.04695*jaw_command[side] for n in ['joint7','joint8']])
  if not np.isfinite(errors).all() or np.any(errors<.001) or np.any(errors>.0075):raise RuntimeError(label+': no bounded bilateral finger closure '+str(errors.tolist()))
 def quiet(side,lens,rel,label):
  phase(label)
  for _ in range(10):
   q=np.asarray(controls[-1]['action'],float).copy();q[6]=jaw_command['left'];q[13]=jaw_command['right'];send(q,label);retention(side,lens,rel,label)
  compression(side,label)
 def obj_move(side,lens,pos,R,rel,seconds,label):
  retention(side,lens,rel,label+'_before')
  rp,rR=rel;A=R@rR.T;move(side,np.array(pos)-A@rp,A,seconds,label)
  retention(side,lens,rel,label+'_after')
  if label in ('nose_down_preseat','nose_down_insert'):
   align_loaded(side,lens,np.asarray(pos),R,label+'_alignment',cap=100)
 def align_loaded(side,lens,target_p,target_R,label,cap=100):
  retention(side,lens,loaded_rel[side],label+"_before")
  phase(label)
  from endpoint_control import LoadedEndpoint
  rp,rR=grasp_transform(side,lens)
  wanted_R=target_R@rR.T;wanted_p=np.asarray(target_p)-wanted_R@rp
  idx=0 if side=='left' else 7;base=emb.rig.bases[side]
  measured=np.asarray(obs.state['joint_pos'],float)[idx:idx+6]
  nominal,pe,re=ik.solve(wanted_p,ik.matrix_to_quat_xyzw(wanted_R),measured,base_pos=base,iters=100,damping=.025)
  if pe>.003 or re>.03:raise RuntimeError(label+': measured-object correction has no legal IK')
  q=np.asarray(controls[-1]['action'],float).copy()
  servo=LoadedEndpoint(nominal,q[idx:idx+6],measured,base)
  for j in range(cap):
   retention(side,lens,loaded_rel[side],label)
   lp,lR=pose(lens,lens)
   if np.linalg.norm(np.asarray(target_p)-lp)<.0015 and np.linalg.norm(ik._rot_error(target_R,lR))<.015:return
   q[idx:idx+6]=servo.update(np.asarray(obs.state['joint_pos'],float)[idx:idx+6])
   nominal_targets[side]=nominal.copy()
   q[6]=jaw_command['left'];q[13]=jaw_command['right'];send(q,label)
  raise RuntimeError(label+': lens failed bounded measured-pose alignment')
 def settle_proof_lift(side,lens,rel):
  label='right_proof_lift_integral';phase(label)
  nominal=np.asarray(controls[-1]['action'],float).copy();target=nominal.copy()
  idx=0 if side=='left' else 7
  wanted_p,wanted_R=ik.tcp_pose(nominal[idx:idx+6],emb.rig.bases[side])
  servo=Tracking(nominal[idx:idx+6],np.asarray(obs.state['joint_pos'],float)[idx:idx+6]);stable=0
  for _ in range(60):
   measured=np.asarray(obs.state['joint_pos'],float)[idx:idx+6]
   target[idx:idx+6]=servo.update(measured)
   if np.any(target[idx:idx+6]<ik.LOWER) or np.any(target[idx:idx+6]>ik.UPPER):raise RuntimeError(label+': compensated target outside public joint limits')
   send(target,label);retention(side,lens,rel,label);compression(side,label)
   p,R=tool(side)
   stable=stable+1 if np.linalg.norm(p-wanted_p)<=.003 and np.linalg.norm(ik._rot_error(R,wanted_R))<=.03 else 0
   if stable>=5:return target
  raise RuntimeError(label+': bounded integral tracking failed')
 def check_loaded_flip(side,lens,pos,R,rel,seconds):
  p0,R0=tool(side);rp,rR=rel;A=R@rR.T;p1=np.asarray(pos)-A@rp
  idx=0 if side=='left' else 7;cur=np.asarray(obs.state['joint_pos'],float)[idx:idx+6].copy()
  for j in range(round(seconds*20)):
   s=ik.smoothstep((j+1)/round(seconds*20));B=ik.slerp_matrix(R0,A,s)
   cur,pe,re=ik.solve(p0+s*(p1-p0),ik.matrix_to_quat_xyzw(B),cur,base_pos=emb.rig.bases[side],iters=100,damping=.025)
   if pe>.003 or re>.03:raise RuntimeError('Loaded flip infeasible before right release')
 def retreat(side,seconds,label,distance=.10):
  p,R=tool(side);move(side,p-distance*R[:,2],R,seconds,label)
 pickup_R=np.diag([1.,-1,-1])
 def swap_start(a_name,b_name):
  import torch
  pa=emb._base.scene[a_name].data.root_pose_w.torch.clone();pb=emb._base.scene[b_name].data.root_pose_w.torch.clone()
  for name,p in ((a_name,pb),(b_name,pa)):
   x=emb._base.scene[name];x.write_root_pose_to_sim_index(root_pose=p);x.write_root_velocity_to_sim_index(root_velocity=torch.zeros_like(x.data.default_root_vel.torch))
  hold('probe_settle',20)
 try:
  instruction=json.loads((Path(__file__).parents[2]/'task.json').read_text())['instruction']
  obs=emb.reset(Scene(id='B6-scripted-demo',instruction=instruction),seed=42)
  o='lens_blue';probe=case.split(':',1)[1] if case and case.startswith('probe:') else None
  if probe:
   q=np.asarray(obs.state['joint_pos'],float);controls.append(dict(control=0,phase='start',action=q.tolist()))
   if probe!=o:swap_start(probe,o)
   o=probe;controls.pop(0)
  lp,lR=pose(o,o);ring=lp+lR[:,2]*.060
  move('right',ring+[0,0,.12],pickup_R,5,'right_pick_approach',1.)
  move('right',ring,pickup_R,4,'right_grip_ring',1.);fingers('right',.55,2,'right_close');compression('right','right_close');rightrel=grasp_transform('right',o);loaded_rel['right']=rightrel
  obj_move('right',o,lp+[0,0,.020],lR,rightrel,1.,'right_proof_lift')
  pickup_hold=settle_proof_lift('right',o,rightrel)
  lifted,_=pose(o,o)
  if lifted[2]-lp[2]<.015:raise RuntimeError('Pickup failed: lens did not lift 15 mm off tray')
  phase('right_pickup_retention')
  for _ in range(10):
   send(pickup_hold,'right_pickup_retention');retention('right',o,rightrel,'right_pickup_retention');compression('right','right_pickup_retention')
  obj_move('right',o,lp+[0,0,.12],lR,rightrel,3,'right_lift')
  moving_load_ready=True
  handoff=np.array([.35,0.,.95]);obj_move('right',o,handoff,np.eye(3),rightrel,6,'right_handoff_carry')
  half_angle=math.radians(75.)
  half_flip=np.array([[1.,0,0],[0,math.cos(half_angle),-math.sin(half_angle)],[0,math.sin(half_angle),math.cos(half_angle)]])
  obj_move('right',o,handoff,half_flip,rightrel,4,'right_loaded_half_flip')
  align_loaded('right',o,handoff,half_flip,'right_measured_half_flip_alignment')
  actual_handoff,actual_half=pose(o,o);left_tcp=actual_handoff+actual_half[:,2]*.035
  approach=np.array([1.,0,0]);jaw=-np.cross(approach,actual_half[:,2]);jaw=jaw/np.linalg.norm(jaw)
  hand_R=np.column_stack((jaw,np.cross(approach,jaw),approach))
  move('left',left_tcp-hand_R[:,2]*.06,hand_R,5,'left_handoff_approach',1.);move('left',left_tcp,hand_R,4,'left_handoff_grip',1.);fingers('left',.55,2,'left_close_barrel');compression('left','left_close_barrel');leftrel=grasp_transform('left',o);loaded_rel['left']=leftrel
  mp,mR=pose('body_right','body_right')
  check_loaded_flip('left',o,np.array([.35,0.,1.010]),mR,leftrel,4)
  fingers('right',1.,2,'right_release');quiet('left',o,leftrel,'left_handoff_retention')
  p,R=tool('right');move('right',p+[0.,-.04,0.],R,2,'right_handoff_clearance')
  p,R=tool('right');move('right',p+[0.,0.,.12],R,2,'right_handoff_retreat')
  mp,mR=pose('body_right','body_right');goal=mp+mR[:,2]*.0006
  leftrel=grasp_transform('left',o)
  check_loaded_flip('left',o,np.array([.35,0.,1.010]),mR,leftrel,4)
  obj_move('left',o,np.array([.35,0.,1.010]),mR,leftrel,4,'left_loaded_flip_complete')
  ring_approach_R=tool('left')[1].copy()
  obj_move('left',o,goal+mR[:,2]*.06,mR,leftrel,5,'nose_down_preseat')
  obj_move('left',o,goal,mR,leftrel,4,'nose_down_insert')
  for j in range(2):obj_move('left',o,goal-mR[:,2]*(j+1)*.0005,mR,leftrel,.5,'seat_contact_push')
  best=0.
  for a in np.linspace(0,LOCK_DIR*math.radians(LOCK_DEG+1.2),40)[1:]:
   obj_move('left',o,goal,mR@rz(a),leftrel,.2,'bayonet_turn');best=max(best,mount_turn_deg())
   if best>=LOCK_DEG+.75:break
  lp,lR=pose(o,o);gap=float((mR.T@(lp-mp))[2])
  locked=best>=LOCK_DEG+.75
  decide(check='lock',lens=o,mount_ring_turn_deg=round(best,2),lens_gap_mm=round(1000*gap,2),rule='locked if the mount ring turned >= 57.15 deg with the lens',result='locked' if locked else 'not locked')
  if not locked:
   if probe:raise StopIteration('probe: '+o+' does not lock')
   raise RuntimeError(o+' did not lock')
  fingers('left',1.,2,'left_release_barrel')
  p,R=tool('left');move('left',p+[0,0,-.12],R,4,'left_leave_barrel')
  rp,rR=pose(o,o+'_zoomring');ring_tcp=rp+rR[:,2]*.025;A=rz(math.pi/6)@ring_approach_R
  move('left',ring_tcp-A[:,2]*.06,A,4,'zoom_approach',1.);move('left',ring_tcp,A,3,'zoom_grasp',1.);fingers('left',.55,2,'zoom_close');compression('left','zoom_close')
  graspP,graspR=tool('left');zrP,zrR=pose(o,o+'_zoomring');relP=graspR.T@(zrP-graspP);relR=graspR.T@zrR
  z0=ring_angle(o);lp,lR=pose(o,o)
  for a in np.linspace(z0,TWIST,30)[1:]:
   T=lR@rz(a)@relR.T;move('left',lp+lR[:,2]*.05-T@relP,T,.2,'zoom_turn')
  planned=TWIST
  for trim in range(2):
   error=TWIST-ring_angle(o)
   if abs(math.degrees(error))<=.3:break
   planned+=error;T=lR@rz(planned)@relR.T
   move('left',lp+lR[:,2]*.05-T@relP,T,(math.ceil((int(abs(math.degrees(error))/10./.002)+100)/25))/20.,'zoom_angle_trim')
   hold('zoom_angle_trim_settle',8)
  decide(check='zoom_set',lens=o,focal_mm=round(focal(o),2),rule='ring angle from poses, trimmed to within 0.3 deg of 50 mm')
  fingers('left',1.,2,'zoom_release')
  f0,e0,n0=focal(o),extension(o),len(controls)
  retreat('left',4,'zoom_retreat',distance=.08)
  if probe:
   hold('probe_watch',100-(len(controls)-n0))
   decide(check='creep',lens=o,window_s=round((len(controls)-n0)/20,2),focal_mm_at_release=round(f0,2),focal_mm_after=round(focal(o),2),front_extension_change_mm=round(1000*(extension(o)-e0),2))
   raise StopIteration('probe complete')
  decide(check='hands_off',lens=o,focal_mm=round(focal(o),2),mount_ring_turn_deg=round(mount_turn_deg(),2))
  phase('final_hands_off_20s');q=np.asarray(obs.state['joint_pos'],float)
  for _ in range(420):send(q,'final_hands_off_20s')
 except StopIteration as e:phases.append(dict(termination=str(e),control=len(controls)))
 except Exception as e:phases.append(dict(failure=type(e).__name__+': '+str(e),control=len(controls)))
 finally:
  (out/'actions.json').write_text(json.dumps(controls,allow_nan=False));(out/'phases.json').write_text(json.dumps(phases,indent=2,allow_nan=False));emb.close()
 return dict(state='attempt_completed',controller_scope=__doc__,lens=o,decisions=decisions,controls=len(controls),last_phase=phases[-1] if phases else None)
