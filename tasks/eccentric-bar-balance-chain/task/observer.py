"""Read-only actual body/contact/wrench observer, sampled after scene.update."""
import json,os
from strict_json import dumps,plain
import numpy as np
from kit import arr
from contract import History,REQUIRED,rotation
from fixture import SHIFT
from contacts import Contacts
class Observer:
 def __init__(self,emb):
  self.emb=emb;self.base=emb._base;self.history=History();self.controls=0;self.done=False;self.channels=set();self.last=None;self.error=None
  from isaaclab_physx.physics import PhysxManager
  view=PhysxManager.get_physics_sim_view()
  paths=[]
  for side in ('left','right'):
   robot=self.base.scene[side+'_robot'];actual=robot.root_view.link_paths
   if len(actual)!=1:raise ValueError('requires one actual robot articulation')
   paths+=list(actual[0])
  self.robots=paths
  # Resolve actual rigid-body path, no assumptions about tensor shape/root name.
  self.bodies={c:list(self.base.scene['bar_'+c].root_view.prim_paths)[0] for c in 'ABC'}
  envroot=self.bodies['A'].rsplit('/',1)[0]
  from pxr import Usd,UsdPhysics
  import omni.usd
  stage=omni.usd.get_context().get_stage()
  from contact_paths import external_colliders
  self.external_roots=external_colliders(stage,envroot,{envroot+'/LeftArm',envroot+'/RightArm',*self.bodies.values()},UsdPhysics.CollisionAPI,UsdPhysics.RigidBodyAPI)
  self.external=list(self.external_roots)
  self.pairs=[];self.filters_by_bar={}
  for i,c in enumerate('ABC'):
   filters=[self.bodies[d] for d in 'ABC' if d!=c]+paths+self.external
   # Missing fixed prim filters are rejected rather than silently treated as no contact.
   import omni.usd
   stage=omni.usd.get_context().get_stage()
   filters=[p for p in filters if stage.GetPrimAtPath(p).IsValid()]
   labels=[next((d for d,pth in self.bodies.items() if p==pth),'robot' if p in paths else 'support' if self.external_roots.get(p,'').endswith('/support') else 'other') for p in filters]
   self.filters_by_bar[c]=filters
   contacts=Contacts(view,[self.bodies[c]],filters)
   if contacts.view.filter_count!=len(filters):raise ValueError('contact filters unresolved')
   self.pairs.append((c,contacts,labels))
  self.nonfinger_paths=[p for p in paths if p.split('/')[-1] not in ('base','tip_left','tip_right')]
  self.link_contacts=Contacts(view,self.nonfinger_paths,list(self.bodies.values())+self.external+self.robots)
  self.trace=open(os.environ['B4_TRACE'],'a',buffering=1) if os.environ.get('B4_TRACE') else None
  self.motor_limits={}
  for side in ('left','right'):
   robot=self.base.scene[side+'_robot'];caps=arr(robot.root_view.get_dof_max_forces())[0];self.motor_limits[side]=dict(zip(robot.joint_names,caps.tolist()))
   for n in range(1,9):
    expected=27 if n<=3 else 7 if n<=6 else 40
    if abs(self.motor_limits[side]['joint'+str(n)]-expected)>1e-5:raise ValueError('actual drive ceiling mismatch')
 def read(self,dt):
  bars={}
  for c in 'ABC':
   a=self.base.scene['bar_'+c];state=arr(a.data.root_link_state_w)[0];p=state[:3];q=state[3:7];v=state[7:10];w=state[10:13]
   bars[c]={'pos':p.tolist(),'quat_xyzw':q.tolist(),'lin_vel':v.tolist(),'ang_vel':w.tolist()}
  pairs=[];details={};robot_tcp={};workspace_ok={};raw_robot={}
  from yam_arena.constants import TCP_OFFSET
  for side in ('left','right'):
   robot=self.base.scene[side+'_robot'];i=robot.body_names.index('gripper');state=arr(robot.data.body_link_state_w)[0,i]
   raw_robot[side]={'joint_names':list(robot.joint_names),'joint_pos':arr(robot.data.joint_pos)[0].tolist(),'body_names':list(robot.body_names),'body_link_state_w':arr(robot.data.body_link_state_w)[0].tolist(),'wrench_force':arr(self.base.scene[side+'_b4_wrench'].data.force)[0].tolist()}
   tcp=state[:3]+rotation(state[3:7])@np.array(TCP_OFFSET)-arr(self.base.scene.env_origins)[0]
   robot_tcp[side]=tcp.tolist();workspace_ok[side]=bool(np.all(tcp>=np.array([-.30,-.40,.825])+SHIFT) and np.all(tcp<=np.array([.35,.40,1.18])+SHIFT))
  outside_contact=False
  for c,view,labels in self.pairs:
   view.read(dt);details[c]=view.last_contacts
   for d in view.last_contacts:
    if d['separation_m']<=0 or abs(d['normal_force_n'])>0:
     pairs.append((c,labels[d['filter_index']]))
     path=self.filters_by_bar[c][d['filter_index']]
     if labels[d['filter_index']]=='robot':
      side='left' if '/LeftArm/' in path else 'right'
      outside_contact=outside_contact or not workspace_ok[side]
  forces=[]
  for side in ('left','right'):
   sensor=self.base.scene[side+'_b4_wrench'];F=arr(sensor.data.force)[0,sensor.body_names.index('gripper')];forces.append(float(np.linalg.norm(F)))
  # Coulomb-contact resultant is bounded above by |normal|+|friction|.
  # Conservative guard proxy; exact native per-contact resultant equivalence is UNVALIDATED.
  self.link_contacts.read(dt);loads={p:0. for p in self.nonfinger_paths}
  for d in self.link_contacts.last_contacts:loads[self.nonfinger_paths[d['sensor_index']]]+=abs(d['normal_force_n'])
  friction,points,count,start=self.link_contacts.view.get_friction_data(dt)
  friction=arr(friction).reshape(-1,3);count=arr(count).astype(int);start=arr(start).astype(int)
  if np.any(count<0) or int(count.sum())>=self.link_contacts.capacity:raise ValueError('friction capacity invalid')
  for pair,(n,i) in enumerate(zip(count.ravel(),start.ravel())):
   if n:
    if i<0 or i+n>len(friction):raise ValueError('friction range invalid')
    force=float(np.linalg.norm(friction[i:i+n],axis=1).sum())
    if not np.isfinite(force):raise ValueError('nonfinite friction force')
    loads[self.nonfinger_paths[pair//count.shape[-1]]]+=force
  return plain({'robot_tcp':robot_tcp,'workspace_ok':workspace_ok,'outside_workspace_robot_bar_contact':outside_contact,'link_force_n':loads,'bars':bars,'contacts':pairs,'contact_details':details,'wrist_force_n':forces,'raw_robot':raw_robot})
 def sample(self,dt):
  if self.error:return
  try:
   self.last=self.read(dt);self.history.update(self.last,dt);self.channels.update(REQUIRED)
   if self.trace:self.trace.write(dumps({'dt':dt,'row':self.last,'history':self.history.result(self.channels)})+'\n')
  except Exception as exc:self.error=f'{type(exc).__name__}: {exc}';self.history.failure='measurement_unavailable';self.channels.clear()
 def status(self):return plain(dict(self.history.result(self.channels),measurement_error=self.error,controls=self.controls,actual_motor_limits=self.motor_limits))
