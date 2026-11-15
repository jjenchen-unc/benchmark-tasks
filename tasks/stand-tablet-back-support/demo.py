"""State-feedback YAM controller plus explicitly labelled evaluator-only fixtures."""
import argparse,json,math,os,sys,traceback
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
from yam_arena import ik,station
from yam_arena.constants import READY,TCP_OFFSET,FINGER_OPEN
from yam_arena.demo import grip_rotation
import runtime
from runtime import arr
from mechanism import ROOT,PANEL2_HINGE_OFFSET
TOP=.7475
TOP_GRIP_LOCAL=np.array([0.,.085,0.])
TOP_PARK_STANDOFF=.040+(.100-TOP_GRIP_LOCAL[1])

class Trial:
 def __init__(self,out,record=True):
  from inspect_robots import Scene
  from yam_arena.inspect_embodiment import yam_arena_embodiment
  self.out=Path(out);self.out.mkdir(parents=True,exist_ok=True);self.record=record
  self.emb=yam_arena_embodiment(task=str(Path(__file__).with_name('task.py')),object_jitter=0,
    control_hz=20,cam_width=480,cam_height=264,device='cpu')
  obs=self.emb.reset(Scene(id='demo',instruction=self.emb.task.module.INSTRUCTION))
  self.frames=[];self.capture(obs);self.rows=[];self.arms={};self.ik_errors=[];self.phase='reset'
  for side in ['left','right']:
   q=np.array(READY);p,r=ik.tcp_pose(q,station.BASES[side])
   self.arms[side]=dict(q=q,p=p,r=r,f=FINGER_OPEN,goal=np.r_[q,[FINGER_OPEN]*2],correction=np.zeros(3))
  self.trace=(self.out/'trace.jsonl').open('w',buffering=1)
  (self.out/'runtime.json').write_text(json.dumps(dict(physics_dt=self.emb._base.physics_dt,
     step_dt=self.emb._base.step_dt,paths=runtime.ACTIVE.paths,filters=runtime.ACTIVE.filters,scoring=runtime.CONFIG,hand_collision_bounds=runtime.ACTIVE.hand_bounds,control_hz=20),indent=2))
 def new_log(self,out,fixture=False,reset=False):
  from inspect_robots import Scene
  self.out=Path(out);self.out.mkdir(parents=True,exist_ok=True)
  self.frames=[];self.rows=[];self.fixture_initialized=fixture;self.ik_errors=[]
  self.trace=(self.out/'trace.jsonl').open('w',buffering=1)
  if reset:
   obs=self.emb.reset(Scene(id='fresh-robot-start',instruction=self.emb.task.module.INSTRUCTION))
   self.capture(obs)
   self.arms={}
   for side in ['left','right']:
    q=np.array(READY);p,r=ik.tcp_pose(q,station.BASES[side])
    self.arms[side]=dict(q=q,p=p,r=r,f=FINGER_OPEN,goal=np.r_[q,[FINGER_OPEN]*2],correction=np.zeros(3))
  (self.out/'runtime.json').write_text(json.dumps(dict(physics_dt=self.emb._base.physics_dt,step_dt=self.emb._base.step_dt,paths=runtime.ACTIVE.paths,filters=runtime.ACTIVE.filters,scoring=runtime.CONFIG,hand_collision_bounds=runtime.ACTIVE.hand_bounds,control_hz=20,fixture_initialized=fixture),indent=2))
 def initialize_fixture(self,mode):
  from mechanism import initial_poses
  angle=math.pi/3
  poses=initial_poses(np.array([.25,0,TOP+.12*math.sin(angle)+.006]),(R.from_euler('z',-90,degrees=True)*R.from_euler('x',60,degrees=True)).as_quat(),*((angle,-2*angle) if mode=='deployed' else (0.,-math.pi)))
  for n,(pos,quat) in poses.items():
   runtime.ACTIVE.views[n].set_transforms(np.array([[*pos,*quat]],np.float32),np.array([0],np.int32))
   runtime.ACTIVE.views[n].set_velocities(np.zeros((1,6),np.float32),np.array([0],np.int32))
  runtime.ACTIVE.scorer.reset()
  self.fixture_initialized=True

 def initialize_capture_fixture(self,path=None):
  """Evaluator-only measured checkpoint; never a fresh-start feasibility demo."""
  import warp as wp
  fixture_path=Path(path) if path else Path('capture_fixture.json')
  if not fixture_path.is_absolute():fixture_path=Path(__file__).parent/fixture_path
  data=json.loads(fixture_path.read_text())
  self.fixture_initialized=True;self.frames=[]
  for i,side in enumerate(('left','right')):
   robot=self.emb._base.scene[side+'_robot']
   robot.write_joint_state_to_sim_index(
    position=wp.array(np.array([data[side+'_q']],np.float32),dtype=wp.float32,device=robot.device),
    velocity=wp.array(np.array([data[side+'_qd']],np.float32),dtype=wp.float32,device=robot.device))
   command=np.array(data['commands'][i]);q=command[:6];f=float(command[6]*FINGER_OPEN)
   p,r=ik.tcp_pose(q,station.BASES[side])
   self.arms[side]=dict(q=q,p=p,r=r,f=f,goal=np.r_[q,[f]*2],correction=np.zeros(3))
  for n,b in data['body_poses'].items():
   runtime.ACTIVE.views[n].set_transforms(np.array([[*b['pos'],*b['quat']]],np.float32),np.array([0],np.int32))
   runtime.ACTIVE.views[n].set_velocities(np.array([data['body_velocities'][n]],np.float32),np.array([0],np.int32))
  runtime.ACTIVE.scorer.reset()
  meta=json.loads((self.out/'runtime.json').read_text())
  meta.update(fixture_initialized=True,full_goal_proven=False,capture_fixture=data)
  (self.out/'runtime.json').write_text(json.dumps(meta,indent=2))
  hand=next(x for x in data['hand_body_poses'] if x['name']=='right/gripper')
  hr=R.from_quat(hand['quat']).as_matrix();hp=np.array(hand['pos'])+hr@TCP_OFFSET
  tp=np.array(data['body_poses']['tablet']['pos']);tr=R.from_quat(data['body_poses']['tablet']['quat']).as_matrix()
  reference=(hr.T@(tp-hp),hr.T@tr)
  initial_body_error=max(float(np.linalg.norm(runtime.ACTIVE.pose(n)[0]-b['pos'])) for n,b in data['body_poses'].items())
  initial_joint_error=max(float(np.max(np.abs(arr(self.emb._base.scene[side+'_robot'].data.joint_pos)[0]-data[side+'_q']))) for side in ('left','right'))
  assert initial_body_error<1e-5 and initial_joint_error<1e-5, ('checkpoint state write mismatch',initial_body_error,initial_joint_error)
  history=[]
  for _ in range(40):
   self.tick('capture_fixture_settle');history.append(self.grasp_relation('right'))
  relation=self.grasp_relation('right')
  dp=float(np.linalg.norm(relation[0]-reference[0]));dr=float(np.linalg.norm(R.from_matrix(relation[1]@reference[1].T).as_rotvec()))
  drift_p=float(np.linalg.norm(history[-1][0]-history[-20][0]));drift_r=float(np.linalg.norm(R.from_matrix(history[-1][1]@history[-20][1].T).as_rotvec()))
  contacts=runtime.ACTIVE.current_contacts['tablet']
  bilateral=all(any('/RightArm/' in path and path.endswith('/'+tip) and force>1. for path,force in contacts.items()) for tip in ('tip_left','tip_right'))
  report=dict(source_run=data['source_run'],source_step=data['source_step'],fixture_initialized=True,full_goal_proven=False,bilateral_contact=bilateral,initial_body_position_error_m=initial_body_error,initial_joint_error_rad=initial_joint_error,relative_translation_m=dp,relative_rotation_deg=math.degrees(dr),last_0p95s_relative_translation_m=drift_p,last_0p95s_relative_rotation_deg=math.degrees(drift_r),tablet_table_contact=runtime.ACTIVE.latest['tablet_table_contact'],stand_table_contact=runtime.ACTIVE.latest['stand_table_contact'])
  (self.out/'capture_fixture_verification.json').write_text(json.dumps(report,indent=2))
  print('CAPTURE_FIXTURE_VERIFICATION',report,flush=True)
  assert bilateral and drift_p<.005 and drift_r<math.radians(3.), ('capture checkpoint did not restore stable hold',report)

 def capture(self,obs):
  if self.record:
   if 'overview_cam' in self.emb._base.scene.sensors:
    view=arr(self.emb._base.scene['overview_cam'].data.output['rgb'])[0,:,:,:3].astype(np.uint8)
    import cv2
    view=cv2.resize(view,(480,264))
    self.frames.append(np.vstack([np.hstack([view,obs.images['top_cam']]),np.hstack([obs.images['left_cam'],obs.images['right_cam']])]))
   else:self.frames.append(np.hstack([np.asarray(obs.images[k]) for k in sorted(obs.images)]))
 def tcp(self,side):
  robot=self.emb._base.scene[side+'_robot'];i=list(robot.body_names).index('gripper')
  pose=arr(robot.data.body_link_pose_w)[0,i];rot=R.from_quat(pose[3:]).as_matrix()
  return pose[:3]+rot@TCP_OFFSET,rot
 def solve(self,side,p,r,seed):
  def fun(q):
   pp,rr=ik.tcp_pose(q,station.BASES[side]);return np.r_[pp-p,.2*R.from_matrix(r@rr.T).as_rotvec()]
  fit=least_squares(fun,np.clip(seed,ik.LOWER+1e-8,ik.UPPER-1e-8),bounds=(ik.LOWER,ik.UPPER),max_nfev=100)
  if np.linalg.norm(fit.fun)>.002:
   alt=[-2.6,2.5,1.8,-1.67,-.4,1.93] if side=='left' else ([2.54,2.2,.54,1.22,.92,1.93] if p[0]<0 else [0,2.3,1,-1.6,-1,1.5])
   f=least_squares(fun,np.clip(alt,ik.LOWER+1e-8,ik.UPPER-1e-8),bounds=(ik.LOWER,ik.UPPER),max_nfev=100)
   if np.linalg.norm(f.fun)<np.linalg.norm(fit.fun):fit=f
  return fit.x,np.linalg.norm(fit.fun[:3]),np.linalg.norm(fit.fun[3:])/.2
 def tick(self,label):
  assert len(self.rows)<self.emb.task.module.MAX_CONTROL_STEPS, '180-second task action limit exceeded'
  from inspect_robots.types import Action
  from inspect_robots_yam.packing import pack
  cmd=[np.r_[a['goal'][:6],np.clip(a['goal'][6]/FINGER_OPEN,0,1)] for a in self.arms.values()]
  res=self.emb.step(Action(data=pack(*cmd)));self.capture(res.observation)
  row=dict(runtime.ACTIVE.latest);row.update(step=len(self.rows)+1,phase=label,commands=[c.tolist() for c in cmd])
  for side in ['left','right']:
   robot=self.emb._base.scene[side+'_robot'];row[side+'_tcp']=self.tcp(side)[0].tolist()
   row[side+'_q']=arr(robot.data.joint_pos)[0].tolist();row[side+'_qd']=arr(robot.data.joint_vel)[0].tolist()
   row[side+'_effort']=arr(robot.data.applied_torque)[0].tolist() if hasattr(robot.data,'applied_torque') else None
  self.rows.append(row);self.trace.write(json.dumps(row)+'\n')
  if len(self.rows)%100==0:print('STEP',len(self.rows),label,'score',row['score'],flush=True)
 def save(self):
  (self.out/'result.json').write_text(json.dumps(dict(steps=len(self.rows),final=self.rows[-1] if self.rows else {},
    overlap_diagnostic=runtime.ACTIVE.overlap_monitor.overall_report(),full_goal_proven=bool(self.rows and self.rows[-1]['score']==1. and not getattr(self,'fixture_initialized',self.emb.task.module.DIAGNOSTIC_FIXTURE)),fixture_initialized=bool(getattr(self,'fixture_initialized',self.emb.task.module.DIAGNOSTIC_FIXTURE)),scope='Evaluator-initialized diagnostic, never a fresh-start full-goal demonstration' if getattr(self,'fixture_initialized',False) else 'Native Arena public-action development; stage may be incomplete'),indent=2))
 def finish(self):
  if getattr(self,'evaluator_mode',False):
   self.trace.close();return
  self.save();self.trace.close()
  if self.frames:
   import imageio
   imageio.imwrite(self.out/'setup.png',self.frames[0]);imageio.imwrite(self.out/'final.png',self.frames[-1])
   imageio.mimsave(self.out/'demo.mp4',self.frames,fps=20,codec='libx264',output_params=['-pix_fmt','yuv420p','-movflags','+faststart','-maxrate','3500k','-bufsize','7000k'])
 def go(self,side,label,duration,p=None,r=None,f=None,joint=False,qgoal=None,align=False,checked=False):
  a=self.arms[side];p0,r0,f0=a['p'].copy(),a['r'].copy(),a['f'];q0=a['q'].copy()
  p1=p0 if p is None else np.array(p);r1=r0 if r is None else r;f1=f0 if f is None else f
  if qgoal is not None:p1,r1=ik.tcp_pose(qgoal,station.BASES[side]);joint=True
  qe=None
  if qgoal is not None:
   assert np.all(qgoal>=ik.LOWER) and np.all(qgoal<=ik.UPPER)
   qe=qgoal
  elif joint:
   qe,pe,re=self.solve(side,p1,r1,q0);self.ik_errors.append([pe,re])
  for k in range(1,round(duration*20)+1):
   s=ik.smoothstep(k/round(duration*20));p=p0+s*(p1-p0);r=ik.slerp_matrix(r0,r1,s)
   if joint:q=q0+s*(qe-q0);pe=re=0.
   else:
    if align:a['correction']=np.clip(a['correction']+.15*(p-self.tcp(side)[0]),-.005,.005)
    q,pe,re=self.solve(side,p+a['correction'],r,a['q'])
   if checked:assert pe<.003 and re<math.radians(2.), ('unreachable waypoint',label,k,pe,re)
   self.ik_errors.append([pe,re]);a['q']=q;a['goal']=np.r_[q,[f0+s*(f1-f0)]*2];self.tick(label)
  a['p'],a['r'],a['f']=p1,r1.copy(),f1
  print(label,'ipad_z',round(runtime.ACTIVE.pose('tablet')[0][2]-TOP,4),'tcp_err_mm',round(np.linalg.norm(self.tcp(side)[0]-p1)*1000,2),flush=True)
  if checked:assert np.linalg.norm(self.tcp(side)[0]-p1)<.005, ('pose tracking failed',label,self.tcp(side)[0],p1)
  self.save()
  if self.frames:
   import imageio
   imageio.imwrite(self.out/(label+'.png'),self.frames[-1])

 def push(self):
  from scoring import CORNERS
  ip,iq,ri=runtime.ACTIVE.pose('tablet');corners=CORNERS@ri.T+ip
  rot=grip_rotation(0.);p0=np.array([corners[:,0].max()+.015,ip[1],ip[2]+.008]);p1=p0+[-float(os.environ.get('COMBINED_PUSH','.10')),0,0]
  for n,t,p,r,f,j in [('preopen',1.5,None,None,0,False),('approach_push',3.5,p0+[0,0,.09],rot,0,True),('lower_push',2,p0,rot,0,False),('push',5,p1,rot,0,False),('withdraw_push',2,p1+[0,0,.10],rot,0,False)]:self.go('left',n,t,p,r,f,j)



 def pivot(self):
  # Closed original fingertips press the overhanging lower screen against the
  # real desk-edge fulcrum. No world joint, pose edit, or fictitious attachment.
  ip,iq,ri=runtime.ACTIVE.pose('tablet')
  assert -.15<ip[0]<-.12, ('unexpected post-push center',ip)
  pivot=np.array([-.19,ip[1],TOP]);pivot_local=ri.T@(pivot-ip)
  local_contact=np.array([0.,-.105,.0035])
  rot=R.from_euler('y',55,degrees=True).as_matrix()@np.column_stack(([0,0,1],[0,-1,0],[1,0,0]))
  offsets=np.array(json.loads(Path(__file__).with_name('press_geometry.json').read_text())['closed_mesh_world_offsets_r55'])
  def target(angle):
   desired=ri@R.from_rotvec([angle,0.,0.]).as_matrix()
   contact=pivot+desired@(local_contact-pivot_local)
   support=offsets[np.argmin(offsets@desired[:,2])]
   result=contact-support
   # Opposed fingertips can exchange the extremal support vertex. Keep the
   # hand lateral position continuous, allowing the contact point to slide
   # across the wide screen instead of commanding a 10 mm sideways jump.
   dy=(ip[1]+.00495)-result[1]
   result[1]+=dy;result[2]-=desired[1,2]/desired[2,2]*dy
   return result
  p=target(0.)
  # Measured prior pivot dynamics predict a slightly sliding desk fulcrum.
  # Preposition the right hand before pressing so it can catch the raised bare
  # top promptly. This is a planned hand pose, never a tablet pose assignment.
  # CPU45567/geometry audit chooses this bounded free-air joint pose directly.
  # Its actual FK is the target: no claim that an unreachable nominal55mm
  # parallel pose was reached. Measured tablet-aligned insertion remains exact.
  right_q=np.array([1.2206781615471551,2.513747307640217,1.9474695026289242,-1.69296999,-1.0869710768144356,-.7586198874825214])
  seed=np.array([-2.609636,2.301980,1.704900,-1.516663,-.295202,2.020039])
  q,pe,re=self.solve('left',p+[0,0,.04],rot,seed)
  assert pe<.002 and re<math.radians(1.), ('press hover unreachable',pe,re)
  # Clear LEFT's old push-withdraw location before placing centered RIGHT.
  # Otherwise the two hands overlap near the tablet top (diagnostic45575).
  self.go('left','pivot_hover',4.,f=0.,qgoal=q,checked=True)
  self.go('right','top_preopen',1.,f=-.030)
  self.go('right','top_preposition',4.,f=-.030,qgoal=right_q,checked=True)
  self.go('left','pivot_touch',2.,p,rot,0.,checked=True)
  a=self.arms['left']
  for k in range(1,101):
   angle=math.radians(30.)*ik.smoothstep(k/100);p=target(angle)
   q,pe,re=self.solve('left',p,rot,a['q'])
   assert pe<.002 and re<math.radians(1.), ('press path unreachable',k,pe,re)
   a['q']=q;a['p']=p;a['r']=rot;a['f']=0.;a['goal']=np.r_[q,[0.,0.]]
   self.tick('pivot_press')
   caught=any(path.endswith('/tip_right') and '/RightArm/' in path and force>1. for path,force in runtime.ACTIVE.current_contacts['tablet'].items())
   if caught:
    gap=self.tablet_in_right_jaw_gap()
    assert gap['inside_gap'], ('parked right finger contacts wrong side of tablet',gap)
   if runtime.ACTIVE.latest['tilt_deg']>=20.:break
  self.right_top_grasp(prepositioned=True)
  self.go('left','pivot_release_press',2.,self.arms['left']['p']+[0,0,.06],rot,0.,checked=True)
  for _ in range(40):self.tick('right_top_hold')
  assert any('/RightArm/' in path for path in runtime.ACTIVE.current_contacts['tablet']), 'right top grip lost after pusher release'

 def tablet_in_right_jaw_gap(self):
  # Signed geometry distinguishes inner pad capture from the outer underside
  # collision found in45520. Coordinates are actual hand-local closingX.
  hp,hr=self.tcp('right');tp,tq,tr=runtime.ACTIVE.pose('tablet')
  normal=hr.T@tr[:,2];center=hr.T@(tp-hp)
  if abs(normal[0])<.8:return dict(inside_gap=False,reason='tablet not aligned with closing axis')
  contact_depth=.011
  x=float((normal@center-normal[2]*contact_depth)/normal[0]);half=.0035/abs(normal[0])
  q=arr(self.emb._base.scene['right_robot'].data.joint_pos)[0]
  # Original collision STL distal pad inner surfaces differ from joint opening
  # by about32 microns. Use the measured opening as a conservative bound.
  lo=float(q[6]);hi=float(-q[7]);local=tr.T@(hp+hr@np.array([x,0.,contact_depth])-tp)
  inside=bool(x-half>=lo-.0005 and x+half<=hi+.0005 and abs(local[0])<=.09 and abs(local[1])<=.12)
  return dict(inside_gap=inside,tablet_closing_interval_m=[x-half,x+half],jaw_inner_interval_m=[lo,hi],tablet_local_probe=local.tolist())

 def top_grasp_target(self,tp=None,tr=None):
  if tp is None:tp,tq,tr=runtime.ACTIVE.pose('tablet')
  # Bare top edge, away from the attached rear hinge. Jaw-symmetric 180-degree
  # roll is necessary for the right arm's actual bounded IK at this location.
  x=tr[:,2];z=-tr[:,1];y=np.cross(z,x)
  rot=np.column_stack([x,y,z])@R.from_euler('z',180,degrees=True).as_matrix()
  target=tp+tr@TOP_GRIP_LOCAL-.0105*z
  return target,rot
 def right_top_grasp(self,prepositioned=False):
  p,r=self.top_grasp_target()
  # Seed is the CPU-verified 40-degree raised-tablet branch; solve checks the
  # actual measured tablet pose, so an unexpected pivot cannot be mistaken for
  # a successful grasp of the planned pose.
  if not prepositioned:
   seed=np.array([1.2705071435,2.0626430925,1.7487768837,-1.5958380744,-.82091545,-.4489179259])
   q,pe,re=self.solve('right',p-.060*r[:,2],r,seed)
   assert pe<.002 and re<math.radians(1.), ('top approach unreachable',pe,re)
   self.go('right','top_preopen',1.,f=-.015)
   self.go('right','top_approach',4.,f=-.015,qgoal=q,checked=True)
  p,r=self.top_grasp_target()
  self.go('right','top_insert',1.5 if prepositioned else 2.,p,r,-.030 if prepositioned else -.015,checked=True)
  gap=self.tablet_in_right_jaw_gap()
  print('PRECLAMP_JAW_GAP',gap,flush=True)
  assert gap['inside_gap'], ('tablet is not between open jaws',gap)
  p,r=self.top_grasp_target()
  self.go('right','top_close',1.5,p,r,0.,checked=True)
  self.go('right','top_clamp_settle',.25)
  assert any('/RightArm/' in path for path in runtime.ACTIVE.current_contacts['tablet']), 'top grasp has no right-hand tablet contact'

 def carry_pose_checked(self,side,label,pos,rotation,duration=6.,settle=False,verify_pose=True):
  # Plan the object's rigid-grasp path, then execute only public joint actions.
  # Object transforms are read for planning/verification and never assigned.
  start,_,start_r=runtime.ACTIVE.pose('tablet');localp,localr=self.grasp_relation(side)
  a=self.arms[side];seed=a['q'].copy();plan=[]
  for k in range(1,round(duration*20)+1):
   s=ik.smoothstep(k/round(duration*20));tp=start+s*(np.asarray(pos)-start)
   tr=ik.slerp_matrix(start_r,rotation,s);hr=tr@localr.T;hp=tp-hr@localp
   q,pe,re=self.solve(side,hp,hr,seed)
   assert pe<.002 and re<math.radians(1.), ('carry path unreachable',label,k,pe,re)
   plan.append((q,hp,hr));seed=q
  for q,hp,hr in plan:
   a['q'],a['p'],a['r'],a['f']=q,hp,hr,0.;a['goal']=np.r_[q,[0.,0.]]
   self.tick(label)
  if settle:
   relations=[]
   for _ in range(40):
    self.tick(label+'_free_hold');relations.append(self.grasp_relation(side))
   dp=float(np.linalg.norm(relations[-1][0]-relations[-20][0]));dr=float(np.linalg.norm(R.from_matrix(relations[-1][1]@relations[-20][1].T).as_rotvec()))
   print('FREE_GRIP_DRIFT',dict(last_window_s=.95,translation_m=dp,rotation_deg=math.degrees(dr),translation_speed_m_s=dp/.95,angular_speed_deg_s=math.degrees(dr)/.95),flush=True)
   assert dp<.005 and dr<math.radians(3.), ('continued free-held grip drift',dp,dr)
  actual,_,actual_r=runtime.ACTIVE.pose('tablet')
  pe=float(np.linalg.norm(actual-np.asarray(pos)));re=float(np.linalg.norm(R.from_matrix(actual_r@rotation.T).as_rotvec()))
  print('CARRY_TRACKING',label,pe,math.degrees(re),flush=True)
  if verify_pose:assert pe<.015 and re<math.radians(8.), ('carried tablet deviated',label,pe,re)
  self.save()
 def outside_carry(self):
  self.go('left','left_park_for_outside_carry',3.,qgoal=np.array(READY),f=FINGER_OPEN)
  desired=R.from_euler('z',-90,degrees=True).as_matrix()@R.from_euler('x',60,degrees=True).as_matrix()
  self.carry_pose_checked('right','tablet_outside_front',[-.25,.11,TOP+.13],desired,6.,settle=True)
  for _ in range(40):self.tick('outside_gravity_exposure')
  assert any('/RightArm/' in path for path in runtime.ACTIVE.current_contacts['tablet']), 'right top hold lost outside desk'

 def expose_leaf(self):
  # A selected physical/IK-checked reorientation recipe, executed exclusively
  # through RIGHT joint actions. No hinge drive or task-body pose assignment.
  profile=json.loads(Path(__file__).with_name('exposure_profile.json').read_text())
  def hinge_angles():
   tr=runtime.ACTIVE.pose('tablet')[2];r1=runtime.ACTIVE.pose('panel1')[2];r2=runtime.ACTIVE.pose('panel2')[2]
   a=tr.T@r1;b=r1.T@r2;q1=math.atan2(a[2,1],a[1,1]);q2=math.atan2(b[2,1],b[1,1])
   if q2>0:q2-=2*math.pi
   return q1,q2
  for _ in range(round(profile.get('initial_hold_s',1.)*20)):self.tick('exposure_initial_hold')
  for index,stage in enumerate(profile['stages']):
   stage=dict(stage,name=('exposure_negative','exposure_rock','exposure_return')[index])
   if index<2 and hinge_angles()[1]>math.radians(-150.):
    print('EXPOSURE_ALREADY_OPEN',stage['name'],list(map(math.degrees,hinge_angles())),flush=True);continue
   rotation=R.from_euler('z',stage['yaw_deg'],degrees=True).as_matrix()@R.from_euler('x',stage['tilt_deg'],degrees=True).as_matrix()
   # Exposure is achieved by actual panel separation, not by hitting a private
   # nominal negative tablet angle. Settle before evaluating the actual hold.
   self.carry_pose_checked('right',stage['name'],stage['center_world_m'],rotation,stage['motion_s'],verify_pose=False)
   relations=[];worlds=[]
   for _ in range(round(stage['hold_s']*20)):
    self.tick(stage['name']+'_hold');relations.append(self.grasp_relation('right'));worlds.append(runtime.ACTIVE.pose('tablet'))
   if len(relations)>=40:
    dp=float(np.linalg.norm(relations[-1][0]-relations[-20][0]));dr=float(np.linalg.norm(R.from_matrix(relations[-1][1]@relations[-20][1].T).as_rotvec()))
    wp=float(np.linalg.norm(worlds[-1][0]-worlds[-20][0]));wr=float(np.linalg.norm(R.from_matrix(worlds[-1][2]@worlds[-20][2].T).as_rotvec()))
    print('EXPOSURE_HOLD_DRIFT',stage['name'],dict(relative_m=dp,relative_deg=math.degrees(dr),world_m=wp,world_deg=math.degrees(wr)),flush=True)
    assert max(dp,wp)<.005 and max(dr,wr)<math.radians(3.), ('continued drift during exposure hold',stage['name'],dp,dr,wp,wr)
   contacts=runtime.ACTIVE.current_contacts['tablet']
   assert all(any('/RightArm/' in path and path.endswith('/'+tip) and force>1. for path,force in contacts.items()) for tip in ('tip_left','tip_right')), ('lost bilateral tablet grip during exposure',stage['name'])
   if index==2:
    for refinement in range(3):
     actual,_,actual_r=runtime.ACTIVE.pose('tablet');pe=float(np.linalg.norm(actual-stage['center_world_m']));re=float(np.linalg.norm(R.from_matrix(actual_r@rotation.T).as_rotvec()))
     if pe<.015 and re<math.radians(8.):break
     if refinement==2:break
     self.carry_pose_checked('right','exposure_return_refine_'+str(refinement+1),stage['center_world_m'],rotation,2.,settle=True,verify_pose=False)
    assert pe<.015 and re<math.radians(8.), ('exposure return pose deviated after settling',pe,re)
  q1,q2=hinge_angles()
  report=dict(q1_deg=math.degrees(q1),q2_deg=math.degrees(q2),fixture_initialized=getattr(self,'fixture_initialized',False),full_goal_proven=False)
  print('LEAF_EXPOSURE',report,flush=True)
  (self.out/'leaf_exposure.json').write_text(json.dumps(report,indent=2))
  # This controller access check prevents repeating the known double-layer
  # pinch. It is not part of the task scorer or task success definition.
  assert q2>math.radians(-150.), ('return leaf remains too closely stacked for this side-pinch controller',report)

 def outside_fold_and_place(self):
  # RIGHT supports the tablet throughout. LEFT approaches the exposed panel
  # from its side, with original fingers; all fold motion is contact driven.
  z=np.array([1.,-.1,0.]);z/=np.linalg.norm(z);x=np.array([0.,0.,1.])
  basis=np.column_stack([x,np.cross(z,x),z])@R.from_euler('z',180,degrees=True).as_matrix()
  def leaf_target():
   pp,pq,pr=runtime.ACTIVE.pose('panel2');hr=pr@basis
   hp=pp+pr@np.array([-.05,-.050,0.])-.0105*hr[:,2]
   return hp,hr
  p,r=leaf_target();seed=np.array([3.0034407363,2.8246296665,1.1472969415,-.7528973464,-1.4230423413,.4428996497])
  q,pe,re=self.solve('left',p-.040*r[:,2],r,seed)
  assert pe<.002 and re<math.radians(1.), ('leaf approach unreachable',pe,re)
  self.go('left','leaf_preopen',1.,f=-.015)
  swung=np.array(READY);swung[0]=q[0]
  self.go('left','leaf_high_swing',3.,qgoal=swung,f=-.015,checked=True)
  self.go('left','leaf_approach',4.,f=-.015,qgoal=q,checked=True)
  p,r=leaf_target();self.go('left','leaf_insert',2.,p,r,-.015,checked=True)
  p,r=leaf_target();self.go('left','leaf_close',1.5,p,r,0.,checked=True)
  assert any('/LeftArm/' in path for path in runtime.ACTIVE.current_contacts['panel2']), 'no actual left panel2 grasp'
  tp,tq,tr=runtime.ACTIVE.pose('tablet');r1=runtime.ACTIVE.pose('panel1')[2];r2=runtime.ACTIVE.pose('panel2')[2]
  a1=tr.T@r1;a2=r1.T@r2;q1=math.atan2(a1[2,1],a1[1,1]);q2=math.atan2(a2[2,1],a2[1,1])
  if q2>0:q2-=2*math.pi
  a=self.arms['left']
  for k in range(1,121):
   f=ik.smoothstep(k/120);u=q1+f*(math.pi/3-q1);v=q2+f*(-2*math.pi/3-q2)
   tp,tq,tr=runtime.ACTIVE.pose('tablet');ru=R.from_rotvec([u,0,0]).as_matrix();rv=R.from_rotvec([v,0,0]).as_matrix()
   grip=tp+tr@(ROOT+ru@(PANEL2_HINGE_OFFSET+rv@np.array([-.05,-.125,0.])))
   r=tr@ru@rv@basis;p=grip-.0105*r[:,2];q,pe,re=self.solve('left',p,r,a['q'])
   assert pe<.002 and re<math.radians(1.), ('leaf fold waypoint unreachable',k,pe,re)
   a.update(q=q,p=p,r=r,f=0.,goal=np.r_[q,[0.,0.]])
   self.tick('outside_leaf_fold')
  for _ in range(20):self.tick('fold_held_settle')
  # The distal part of the deployed leaf overlaps the real desk while both
  # palms remain outside its front edge. Lower BOTH hands until real contact.
  pp,pq,pr=runtime.ACTIVE.pose('panel2')
  from mechanism import PANEL_SIZE
  low=float(pp[2]-np.abs(pr[2])@(PANEL_SIZE/2)-TOP)
  assert -.002<low<.045, ('unexpected deployed leaf height',low)
  starts={side:(a['p'].copy(),a['r'].copy(),a['q'].copy()) for side,a in self.arms.items()}
  supported=False
  for k in range(1,61):
   dz=ik.smoothstep(k/60)*(max(low,0.)+.0005)
   for side,a in self.arms.items():
    p0,r0,q0=starts[side];p=p0+[0.,0.,-dz];q,pe,re=self.solve(side,p,r0,a['q'])
    assert pe<.002 and re<math.radians(1.), ('supported lower unreachable',side,k,pe,re)
    a.update(q=q,p=p,r=r0,f=0.,goal=np.r_[q,[0.,0.]])
   self.tick('bimanual_outside_lower')
   if bool(set(runtime.ACTIVE.current_contacts['panel2'])&runtime.ACTIVE.desk_paths):
    supported=True;break
  assert supported, 'leaf did not contact the real desk'
  # Narrow release, then retreat away from the leaf side and desk front before
  # widening the fingers. RIGHT keeps the tablet pose while leaf slides later.
  a=self.arms['left'];p=a['p'].copy();r=a['r'].copy()
  self.go('left','leaf_narrow_release',.5,f=-.003)
  self.go('left','leaf_inplane_retreat',1.,p-.040*r[:,2],r,-.003,checked=True)
  self.go('left','leaf_clear_open',.5,f=-.025)
  self.go('left','leaf_high_return',3.,qgoal=swung,f=FINGER_OPEN)
  self.go('left','left_final_park',3.,qgoal=np.array(READY),f=FINGER_OPEN)
  self.transfer_seated()
  self.finish_placement()

 def endpoint_gap(self):
  tp,tq,tr=runtime.ACTIVE.pose('tablet');pp,pq,pr=runtime.ACTIVE.pose('panel2')
  return max(float(np.linalg.norm(tp+tr@np.array([x,-.120,-.006])-pp-pr@np.array([x,-.075,0.]))) for x in (-.045,.045))

 def carry_pose_feedback(self,label,pos,rotation,duration):
  """Measured object feedback through bounded public robot position targets."""
  side='right';a=self.arms[side];start,_,startr=runtime.ACTIVE.pose('tablet')
  hp,hr=self.tcp(side)
  # Preserve the existing servo preload at the start, then adapt it slowly to
  # the actual object error. This does not change actuator gains or effort caps.
  bias_p=a['p']-hp;bias_r=R.from_matrix(a['r']@hr.T).as_rotvec()
  count=round(duration*20)
  for k in range(1,count+41):
   fraction=ik.smoothstep(min(k/count,1.));desired_p=start+fraction*(np.asarray(pos)-start);desired_r=ik.slerp_matrix(startr,rotation,fraction)
   actual,_,actual_r=runtime.ACTIVE.pose('tablet')
   error_p=desired_p-actual;error_r=R.from_matrix(desired_r@actual_r.T).as_rotvec()
   bias_p+=.08*error_p;bias_r+=.08*error_r
   for value,limit in ((bias_p,.015),(bias_r,math.radians(8.))):
    length=np.linalg.norm(value)
    if length>limit:value*=limit/length
   gp,gr=self.grasp_relation(side);target_r=desired_r@gr.T;target_p=desired_p-target_r@gp
   target_p+=bias_p;target_r=R.from_rotvec(bias_r).as_matrix()@target_r
   q,pe,re=self.solve(side,target_p,target_r,a['q'])
   assert pe<.002 and re<math.radians(1.), ('object feedback waypoint unreachable',label,k,pe,re)
   a.update(q=q,p=target_p,r=target_r,f=0.,goal=np.r_[q,[0.,0.]])
   self.tick(label if k<=count else label+'_feedback_hold')
   if k%20==0:print('OBJECT_FEEDBACK',label,k,dict(position_error_m=float(np.linalg.norm(error_p)),rotation_error_deg=math.degrees(float(np.linalg.norm(error_r))),endpoint_gap_m=self.endpoint_gap()),flush=True)
  actual,_,actual_r=runtime.ACTIVE.pose('tablet');pe=float(np.linalg.norm(actual-pos));re=float(np.linalg.norm(R.from_matrix(actual_r@rotation.T).as_rotvec()))
  assert pe<.005 and re<math.radians(3.), ('object feedback did not settle',label,pe,re)
  self.save()

 def transfer_seated(self):
  # Observed released native fixture45480 support pose, used only as a planning
  # reference. Task scoring still accepts its declared broad orientation range.
  reference_r=R.from_quat([.35661911964416504,-.35671156644821167,-.6106441020965576,.6104862093925476]).as_matrix()
  p,_,r=runtime.ACTIVE.pose('tablet');height=TOP+.106247546672821
  self.carry_pose_feedback('supported_pose_align',np.array([p[0],p[1],height]),reference_r,3.)
  self.carry_pose_feedback('supported_inboard_feedback',np.array([-.105,p[1],height]),reference_r,6.)

 def finish_placement(self):
  # Settle the tablet's own lower edge as well as the stand onto the desk.
  # Actual support geometry and grip compliance determine the required travel;
  # a private nominal 10 mm gap was not a physical or scoring requirement.
  low=float(runtime.ACTIVE.latest['lowest_corner_m']);a=self.arms['right']
  max_lower=min(max(low+.004,.004),.020)
  p0=a['p'].copy();r0=a['r'].copy();supported=False
  for k in range(1,61):
   p=p0+[0.,0.,-max_lower*ik.smoothstep(k/60)]
   q,pe,re=self.solve('right',p,r0,a['q'])
   assert pe<.002 and re<math.radians(1.), ('final contact lowering unreachable',k,pe,re)
   a.update(q=q,p=p,r=r0,f=0.,goal=np.r_[q,[0.,0.]])
   self.tick('tablet_bottom_contact_lower')
   if runtime.ACTIVE.latest['tablet_table_contact'] and runtime.ACTIVE.latest['stand_table_contact']:
    for _ in range(10):self.tick('tablet_bottom_contact_hold')
    if runtime.ACTIVE.latest['tablet_table_contact'] and runtime.ACTIVE.latest['stand_table_contact']:
     supported=True;break
  assert supported, ('bounded final lowering did not establish both desk contacts',max_lower,runtime.ACTIVE.latest)
  assert runtime.ACTIVE.latest['tablet_table_contact'] and runtime.ACTIVE.latest['stand_table_contact'], 'held negative lacks both real tabletop supports'
  # Actual held final-pose negative: same orientation/support, right still
  # touching. It must score zero, then final released hold is assessed afresh.
  for _ in range(100):self.tick('held_final_pose_negative')
  assert runtime.ACTIVE.latest['score']==0., 'held tablet incorrectly scored'
  a=self.arms['right'];p=a['p'].copy();r=a['r'].copy()
  self.go('right','right_final_release',1.,f=-.025)
  self.go('right','right_final_withdraw',3.,p+np.array([0.,-.200,0.]),r,-.025,checked=True)
  for _ in range(120):self.tick('final_released_hold')
  assert runtime.ACTIVE.latest['score']==1., ('released stand did not reach full score',runtime.ACTIVE.latest)

 def grasp_relation(self,side):
  hp,hr=self.tcp(side);tp,tq,tr=runtime.ACTIVE.pose('tablet')
  return hr.T@(tp-hp),hr.T@tr

def main():
 ap=argparse.ArgumentParser();ap.add_argument('out');ap.add_argument('--stage',default='smoke',choices=['smoke','push','pivot','outside_carry','outside_full','fixture_capture','fixture_place','fixture_transfer_place','fixture_exposure','fixture_exposure_fold','fixture_deployed','fixture_folded','suite']);ap.add_argument('--no-record',action='store_true');ap.add_argument('--capture-fixture');ap.add_argument('--physics-dt',type=float,choices=[.0005,.001],default=.001);a=ap.parse_args()
 os.environ['STAND_PHYSICS_DT']=str(a.physics_dt)
 if os.environ.get('STAND_DIAGNOSTIC_FIXTURE'):raise ValueError('inherited fixture environment is forbidden')
 t=None
 try:
  t=Trial(Path(a.out)/'initial_smoke' if a.stage=='suite' else a.out,not a.no_record)
  capture_fixture=a.stage in ('fixture_capture','fixture_place','fixture_transfer_place','fixture_exposure','fixture_exposure_fold')
  if a.capture_fixture and not capture_fixture:raise ValueError('--capture-fixture is allowed only for explicit capture diagnostics')
  if capture_fixture:t.initialize_capture_fixture(a.capture_fixture)
  elif a.stage.startswith('fixture_'):t.initialize_fixture('deployed' if a.stage=='fixture_deployed' else 'upright_folded')
  if not capture_fixture:
   for _ in range(160 if a.stage.startswith('fixture_') else 20):t.tick('fixture_diagnostic' if a.stage.startswith('fixture_') else 'initial_settle')
  if a.stage in ('push','pivot','outside_carry','outside_full'):t.push()
  if a.stage in ('pivot','outside_carry','outside_full'):t.pivot()
  if a.stage in ('outside_carry','outside_full'):t.outside_carry()
  if a.stage in ('outside_full','fixture_exposure','fixture_exposure_fold'):t.expose_leaf()
  if a.stage in ('outside_full','fixture_exposure_fold'):t.outside_fold_and_place()
  if a.stage=='fixture_transfer_place':t.transfer_seated();t.finish_placement()
  if a.stage=='fixture_place':t.finish_placement()
  t.finish()
  if a.stage=='outside_full':
   t.new_log(Path(a.out)/'reset_check',reset=True)
   for _ in range(40):t.tick('post_success_fresh_reset')
   state=runtime.ACTIVE.latest
   assert state['score']==0. and state['tilt_deg']<1. and np.linalg.norm(np.array(state['tablet_pos'])-[-.04,.10,TOP+.00665])<.002, ('fresh reset differs from canonical flat start',state)
   t.finish()
  if a.stage=='suite':
   for mode in ['deployed','upright_folded']:
    t.new_log(Path(a.out)/('fixture_'+mode),fixture=True)
    t.initialize_fixture(mode)
    for _ in range(180):t.tick('evaluator_fixture_'+mode)
    t.finish()
   t.new_log(Path(a.out)/'reset_check',reset=True)
   for _ in range(40):t.tick('fresh_reset_settle')
   assert runtime.ACTIVE.latest['score']==0. and runtime.ACTIVE.latest['tilt_deg']<1.
   t.finish()
 except BaseException:
  error=traceback.format_exc();print(error,flush=True)
  Path(a.out).mkdir(parents=True,exist_ok=True);(Path(a.out)/'exception.txt').write_text(error)
  if t is not None:t.finish()
  raise
if __name__=='__main__':main()
