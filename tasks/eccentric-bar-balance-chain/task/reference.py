"""Disclosed state-aware teacher. ONLY14D robot Actions change task objects.
No task state setters, forces, goal flags or evaluator timer writes.
"""
import numpy as np
from fixture import SHIFT,POSITIONS,SUPPORT,ROBOT_BASES,PARAMETERS

def planned_waypoints():
 points=[]
 def add(label,pos,jaw,n):points.append((label,np.asarray(pos,float),jaw,n))
 carry=1.12+SHIFT[2]
 for j,c in enumerate('ABC'):
  pick=POSITIONS[c]+[round(PARAMETERS[c]['com'],6),0,.01]
  add(c+'_approach',[*pick[:2],carry],.96,35)
  add(c+'_pregrasp',pick+[0,0,.05],.96,25)
  add(c+'_grasp',pick,.96,25)
  #50mm object, command44mm:5000N/m x3mm=15N/finger nominal;40N cap remains.
  add(c+'_close',pick,.044/.0939,20)
  add(c+'_lift',[*pick[:2],carry],.044/.0939,40)
  x=SUPPORT[0];y=SUPPORT[1]
  z=SUPPORT[2]+.08+j*.05+.025+.01
  add(c+'_carry',[x,y,carry],.044/.0939,40)
  add(c+'_preplace',[x,y,z+.012],.044/.0939,25)
  add(c+'_place',[x,y,z-.001],.044/.0939,30)
  add(c+'_release',[x,y,z-.001],.96,25)
  add(c+'_retreat',[x,y,min(z+.08,1.175+SHIFT[2])],.96,30)
 add('park',[.20,-.18,1.10+SHIFT[2]],.96,35)
 return points

# Privileged native COM is disclosed teacher input, never a model observation.
# Feedback changes only public robot Actions. Physics, material, goal and budget are unchanged.
MAX_SETTLE=32
BUDGET=1896
MAX_SEAT=40
SEAT_TRAVEL=.004
SEAT_STEP=.0002

def run(emb,obs,record,variant='full'):
 from inspect_robots.types import Action
 from yam_arena import ik
 from yam_arena.demo import grip_rotation
 from kit import arr
 q=np.asarray(obs.state['joint_pos'],float).copy();current,_=ik.tcp_pose(q[7:13],ROBOT_BASES['right']);R=grip_rotation(0.)
 def send(target,label):
  import kit
  from strict_json import dumps
  target=np.asarray(target,float)
  if target.shape!=(14,) or not np.isfinite(target).all():raise ValueError('invalid action14')
  # Force status and last raw tick through real strict JSON before every Action.
  dumps({'status':kit.ENGINE.status(),'tick':kit.ENGINE.last})
  if kit.ENGINE.error:raise RuntimeError(kit.ENGINE.error)
  if kit.ENGINE.controls>=BUDGET:raise RuntimeError('native action budget exhausted')
  result=emb.step(Action(data=np.asarray(target,float)));record(label,target,result)
  if result.terminated:raise RuntimeError('episode terminated: '+str(result.termination_reason))
  return result
 offset=np.zeros(3);rot_offset=np.eye(3)
 for waypoint_index,(label,goal,jaw,n) in enumerate(planned_waypoints()):
  if variant=='one_bar' and label.startswith('B_'):break
  if variant=='two_bar' and label.startswith('C_'):break
  if variant=='hold_on' and label=='C_release':break
  if waypoint_index==0:
   solved,pe,re=ik.solve(goal,ik.matrix_to_quat_xyzw(R),q[7:13],base_pos=ROBOT_BASES['right'],iters=250)
   if pe>.003 or re>.03:raise RuntimeError('initial approach IK failed')
   startq=q[7:13].copy();n=max(n,int(np.ceil(np.max(abs(solved-startq))/.04)))
   for i in range(1,n+1):
    t=i/n;t=t*t*(3-2*t);q[7:13]=startq+t*(solved-startq);q[13]=jaw;send(q,label+'_joint_approach')
   current=goal.copy();continue
  start=current.copy();j0=q[13]
  loaded=label.split('_')[-1] in ('lift','carry','preplace','place')
  if label.endswith('_release'):
   import kit
   j0=max(j0,float(max(np.asarray(kit.ENGINE.last['raw_robot']['right']['joint_pos'])[-2:]/-.04695)))
  for i in range(1,n+1):
   t=i/n;t=t*t*(3-2*t);p=start+(goal-start)*t
   if label.endswith('_release'):
    q[13]=j0+(jaw-j0)*t;send(q,label);continue
   solved,pe,re=ik.solve(p+(offset if loaded else 0),ik.matrix_to_quat_xyzw(rot_offset@R if loaded else R),q[7:13],base_pos=ROBOT_BASES['right'],iters=120)
   if pe>.003 or re>.03:raise RuntimeError(f'IK tracking preflight failed {label}:{i} pos={pe} rot={re}')
   q[7:13]=solved;q[13]=j0+(jaw-j0)*t;send(q,label)
  current=goal.copy()
  if label.endswith('_release'):
   current,_=ik.tcp_pose(q[7:13],ROBOT_BASES['right']);offset=np.zeros(3);rot_offset=np.eye(3)
  # Actual loaded object pose, independent fingers and native contact graph decide progression.
  # Finite correction range and finite extra controls: four stages x32 per bar.
  if label.split('_')[-1] in ('lift','carry','preplace','place'):
   import kit
   from contract import rotation
   c=label[0]
   def measured():
    row=kit.ENGINE.last
    if row is None:raise RuntimeError('missing actual feedback')
    bar=row['bars'][c];rb=rotation(bar['quat_xyzw'])
    com=np.array(bar['pos'])+rb@np.array([round(PARAMETERS[c]['com'],6),0,0])
    return row,rb,com
   desired_com=goal-np.array([0,0,.01])
   for correction in range(MAX_SETTLE):
    row,rb,com=measured();err=desired_com-com
    axiserr=ik._rot_error(np.eye(3),rb)
    # A balanced grasp must retain the bar and remain nearly level before travel/release.
    fingers=np.array(row['raw_robot']['right']['joint_pos'])[-2:]
    if not np.isfinite(np.r_[err,axiserr,fingers]).all():raise RuntimeError('nonfinite loaded feedback')
    if not any(a==c and b=='robot' for a,b in row['contacts']):raise RuntimeError('lost grasp '+label)
    if np.max(np.abs(fingers))>.035:raise RuntimeError('unretained independent fingers '+label)
    if np.linalg.norm(err)<.002 and np.linalg.norm(axiserr)<.025:
     if not label.endswith('_place') or any(a==c and b==('support' if c=='A' else chr(ord(c)-1)) for a,b in row['contacts']):break
    dp=.35*err;dp*=min(1.,.002/max(np.linalg.norm(dp),1e-12));offset+=dp
    if np.linalg.norm(offset)>.06:raise RuntimeError('bounded translation correction exhausted '+label)
    rv=.25*axiserr;angle=np.linalg.norm(rv);rv*=min(1.,.012/max(angle,1e-12))
    from yam_arena.urdf import axis_angle_matrix
    angle=np.linalg.norm(rv)
    if angle:rot_offset=axis_angle_matrix(rv/angle,angle)@rot_offset
    if np.linalg.norm(ik._rot_error(rot_offset,np.eye(3)))>.18:raise RuntimeError('bounded rotation correction exhausted '+label)
    solved,pe,re=ik.solve(goal+offset,ik.matrix_to_quat_xyzw(rot_offset@R),q[7:13],base_pos=ROBOT_BASES['right'],iters=160)
    if pe>.003 or re>.03:raise RuntimeError('feedback IK failed '+label)
    q[7:13]=solved;send(q,label+'_measured_settle')
   row,rb,com=measured()
   if np.linalg.norm(desired_com-com)>.004 or np.linalg.norm(ik._rot_error(np.eye(3),rb))>.04:raise RuntimeError('actual bar pose unconverged '+label)
   if label.endswith('_place'):
    below='support' if c=='A' else chr(ord(c)-1)
    # Pose convergence is insufficient for contact: use a finite measured TCP
    # anchored descent, never the accumulated position integral as a seat start.
    # 4mm/0.2mm increments, <=40 controls including repeated servo observation.
    # Existing native wrist30N stop/80N abort, motor and workspace guards apply.
    if not any(a==c and b==below for a,b in row['contacts']):
     anchor,_=ik.tcp_pose(np.asarray(row['raw_robot']['right']['joint_pos'][:6]),ROBOT_BASES['right'])
     seatR=rot_offset@R;depth=0.
     for seat in range(MAX_SEAT):
      row,rb,com=measured()
      if any(a==c and b==below for a,b in row['contacts']):break
      if not any(a==c and b=='robot' for a,b in row['contacts']):raise RuntimeError('lost grasp while seating '+label)
      if np.linalg.norm(com[:2]-np.array([SUPPORT[0],SUPPORT[1]]))>.004 or np.linalg.norm(ik._rot_error(np.eye(3),rb))>.04:raise RuntimeError('seat alignment lost '+label)
      force=np.asarray(row['wrist_force_n'],float)
      fingers=np.asarray(row['raw_robot']['right']['joint_pos'],float)[-2:]
      if force.shape!=(2,) or not np.isfinite(np.r_[force,fingers]).all():raise RuntimeError('invalid seating force/fingers '+label)
      if np.max(np.abs(fingers))>.035:raise RuntimeError('unretained seating fingers '+label)
      if max(force)>=30.:raise RuntimeError('seat reached native wrist stop '+label)
      tcp,_=ik.tcp_pose(np.asarray(row['raw_robot']['right']['joint_pos'][:6]),ROBOT_BASES['right'])
      if np.linalg.norm(tcp-anchor)>.006:raise RuntimeError('measured seating excursion exhausted '+label)
      # Advance only after the previous small target is physically approached.
      if depth==0. or abs(tcp[2]-(anchor[2]-depth))<.0003:depth=min(SEAT_TRAVEL,depth+SEAT_STEP)
      target=anchor-np.array([0.,0.,depth])
      solved,pe,re=ik.solve(target,ik.matrix_to_quat_xyzw(seatR),q[7:13],base_pos=ROBOT_BASES['right'],iters=160,tol=1e-5)
      if pe>.0003 or re>.03:raise RuntimeError('seating IK failed '+label)
      q[7:13]=solved;send(q,label+'_contact_seek')
     row,rb,com=measured()
     if np.linalg.norm(desired_com-com)>.004 or np.linalg.norm(ik._rot_error(np.eye(3),rb))>.04:raise RuntimeError('seated bar pose unconverged '+label)
    if not any(a==c and b==below for a,b in row['contacts']):raise RuntimeError('no actual support before release '+label)
 for _ in range(380 if variant=='truncate' else 420):send(q,'final_hold')
 return q
