"""Passive force law, contact-triggered hook connections and measured DL-T2 history.
No controller can access this mutating mechanism API. Calibration is unverified.
"""
import math,traceback
import numpy as np
from kit import FIXTURE,SPECS,arr,rotation
from scoring import History
from contacts import Contacts
class Physics:
 def __init__(self,emb):
  self.emb=emb;self.base=emb._base;self.settling=True;self.done=False;self.controls=0;self.error=None;self.history=History();self.contacts_ready=False;self.logs=[]
  self.parts={c:self.base.scene['case_'+c] for c in ['A','B']};self.bodymeta={b['name']:b for b in FIXTURE['bodies']};self.jointmeta={j['name']:j for b in FIXTURE['bodies'] for j in b['joints']}
 def initialize(self):
  import torch
  self.error=None;self.done=False;self.controls=0;self.history=History();self.logs=[]
  self.engaged={n:next((i for i in range(3) if next(e for e in FIXTURE['connections'] if e['name']==n+f'_eq{i}')['enabled']),-1) for n in FIXTURE['springs']};self.armed={n:False for n in self.engaged};self.taut={n:self.engaged[n]>=0 for n in self.engaged};self.dwell={n:[0.,0.,0.] for n in self.engaged};self.locked={n:False for n in self.engaged}
  from isaaclab.sim.utils import get_current_stage
  from pxr import UsdPhysics
  stage=get_current_stage();self.eq={}
  for c,a in self.parts.items():
   z=torch.zeros_like(a.data.joint_pos.torch);q=z.clone()
   for i,name in enumerate(a.joint_names):q[:,i]=FIXTURE['initial_joints'][name]
   a.write_joint_state_to_sim(q,z);a.set_joint_effort_target(z)
   # Pinned PhysX tensor API defines these as static/dynamic EFFORT, not the legacy load-scaled coefficient.
   # Match native MuJoCo frictionloss in SI, including stiction; no explicit tanh torque is added below.
   friction=torch.tensor([[self.jointmeta[n]['friction'] for n in a.joint_names]],device=self.base.device,dtype=torch.float32)
   a.write_joint_friction_coefficient_to_sim_index(joint_friction_coeff=friction,joint_dynamic_friction_coeff=friction,joint_viscous_friction_coeff=z)
   for name in a.joint_names:
    j=self.jointmeta[name];limits=j['range'].copy()
    self.limit(a,name,limits)
   root=str(a.root_view.link_paths[0][0]).split('/links/')[0]
   for e in FIXTURE['connections']:
    if not e['name'].startswith(c):continue
    prim=stage.GetPrimAtPath(root+'/connections/'+e['name'])
    if not prim:raise RuntimeError('Missing authored hook joint '+str(root))
    self.eq[e['name']]=UsdPhysics.SphericalJoint(prim).GetJointEnabledAttr();self.eq[e['name']].Set(e['enabled'])
  for side in ['left','right']:self.base.scene[side+'_robot']._s2_stop=False
 def limit(self,a,name,values):
  import torch
  ids=torch.tensor([a.joint_names.index(name)],device=self.base.device,dtype=torch.long);v=torch.tensor([[values]],device=self.base.device,dtype=torch.float32)
  a.write_joint_position_limit_to_sim_index(limits=v,joint_ids=ids,warn_limit_violation=False)
 def begin(self):
  self.settling=False;self.history=History();self.controls=0;self.error=None;self.done=False
  if not self.contacts_ready:self.setup_contacts()
 def setup_contacts(self):
  from isaaclab_physx.physics import PhysxManager
  from isaaclab.sim.utils import get_current_stage
  from pxr import UsdPhysics
  sim=PhysxManager.get_physics_sim_view();self.simview=sim;self.robots={s:list(self.base.scene[s+'_robot'].root_view.link_paths[0]) for s in ['left','right']};robots=self.robots['left']+self.robots['right'];self.casepaths={c:list(a.root_view.link_paths[0]) for c,a in self.parts.items()};inst=[str(self.base.scene['inst_'+c].root_view.prim_paths[0]) for c in ['A','B']]
  self.target=Contacts(sim,self.casepaths['A']+[inst[0]],robots)
  self.captures={n:Contacts(sim,[next(p for p in self.casepaths[n[0]] if p.endswith('/'+n+'_hook'))],[next(p for p in self.casepaths[n[0]] if p.endswith('/lid_'+n[0]))]) for n in self.engaged}
  envroot=self.casepaths['A'][0].split('/case_A/')[0];static=[]
  for p in get_current_stage().Traverse():
   if str(p.GetPath()).startswith(envroot+'/') and p.HasAPI(UsdPhysics.CollisionAPI):
    ancestor=p
    while ancestor and str(ancestor.GetPath())!=envroot and not ancestor.HasAPI(UsdPhysics.RigidBodyAPI):ancestor=ancestor.GetParent()
    static.append(str(ancestor.GetPath()) if ancestor and ancestor.HasAPI(UsdPhysics.RigidBodyAPI) else str(p.GetPath()))
  filters=sorted(set(static+robots+self.casepaths['A']+self.casepaths['B']+inst))
  self.nonfinger={s:Contacts(sim,[p for p in self.robots[s] if p.rsplit('/',1)[-1] in ('link1','link2','link3','link4','link5','gripper')],filters) for s in ['left','right']};self.contacts_ready=True
 def states(self):
  origin=arr(self.base.scene.env_origins)[0];bodies={};joint={};vel={}
  for c,a in self.parts.items():
   bodies.update({n:(p-origin,rotation(q)) for n,p,q in zip(a.body_names,arr(a.data.body_pos_w)[0],arr(a.data.body_quat_w)[0])});joint.update(zip(a.joint_names,arr(a.data.joint_pos)[0]));vel.update(zip(a.joint_names,arr(a.data.joint_vel)[0]))
  return bodies,joint,vel
 def site(self,bodies,name):
  b=next(b for b in FIXTURE['bodies'] if any(s['name']==name for s in b['sites']));s=next(s for s in b['sites'] if s['name']==name);p,R=bodies[b['name']];return p+R@s['pos']
 def pre_step(self):
  if self.error:return
  import torch
  try:
   for c,a in self.parts.items():
    q=arr(a.data.joint_pos)[0];v=arr(a.data.joint_vel)[0];eff=[]
    for i,n in enumerate(a.joint_names):
     j=self.jointmeta[n];force=-j['stiffness']*(q[i]-j['ref'])-j['damping']*v[i]
     if n.endswith('_slide'):force-=FIXTURE['springs'][n[:-6]]*max(0.,q[i]+.004)
     eff.append(force)
    a.set_joint_effort_target(torch.tensor([eff],device=self.base.device,dtype=torch.float32))
  except Exception:self.error=traceback.format_exc()
 def post_step(self,dt):
  if self.error:return
  try:
   if abs(dt-.002)>1e-9:raise ValueError('Native mechanism requires500Hz physics')
   bodies,joint,vel=self.states()
   # During the SDK settle, passive springs/closed-case connections already act.
   if self.settling:return
   for n in self.engaged:
    a=self.parts[n[0]];th=float(joint[n+'_lever']);stretch=float(joint[n+'_slide'])+.004
    if n[0]=='A':
     home=joint[n+'_catch']<.0015;locked=bool(home and (self.locked[n] or th<math.radians(2.6)))
     if locked!=self.locked[n]:self.limit(a,n+'_lever',[0.,math.radians(2.6) if locked else 1.9]);self.locked[n]=locked
    e=self.engaged[n]
    if e>=0:
     if stretch>.003:self.taut[n]=True
     if stretch<.0005 and (th>math.radians(105) or (self.taut[n] and th>math.radians(70))):self.eq[n+f'_eq{e}'].Set(False);self.engaged[n]=-1;self.armed[n]=False;self.taut[n]=False;self.logs.append({'event':'release','latch':n,'t':self.history.seconds})
     continue
    tip=self.site(bodies,n+'_hook_tip');dist=[np.linalg.norm(tip-self.site(bodies,n+f'_pin{i}')) for i in range(3)]
    if not self.armed[n]:
     if min(dist)>.015:self.armed[n]=True
     continue
    pressed=self.captures[n].read(dt)[0] and th<math.radians(100)
    for i in range(3):
     self.dwell[n][i]=self.dwell[n][i]+dt if pressed and dist[i]<.0025 else 0.
     if self.dwell[n][i]>=.15-1e-10:
      self.eq[n+f'_eq{i}'].Set(True);self.engaged[n]=i;self.dwell[n]=[0.,0.,0.];self.taut[n]=False;self.logs.append({'event':'capture','latch':n,'bar':i,'t':self.history.seconds});break
   touch=self.target.read(dt)[0];forces=[];linkforces=[]
   for side in ['left','right']:
    sensor=self.base.scene[side+'_s2_wrench'];forces.append(float(np.linalg.norm(arr(sensor.data.force)[0,sensor.body_names.index('gripper')])))
    view=self.nonfinger[side];view.read(dt);perbody={}
    for r in view.last_contacts:perbody[r['sensor_index']]=perbody.get(r['sensor_index'],0.)+abs(r['normal_force_n'])
    # Tangential forces increase the upper bound used for the safety stop.
    ff,fp,count,start=map(arr,view.view.get_friction_data(dt));friction=np.zeros(view.view.sensor_count)
    # Pinned PhysX integer pair tables: zero-count slices add exactly +0.
    # Row-major active pairs retain the original nonempty reduction/add order.
    sparse=(ff.ndim==2 and ff.shape[1]==3 and ff.dtype.kind=='f' and count.ndim==2 and count.dtype.kind in 'iu' and start.dtype.kind in 'iu' and start.shape==count.shape and count.shape[0]<=view.view.sensor_count)
    pairs=np.argwhere(count!=0) if sparse else ((i,k) for i in range(count.shape[0]) for k in range(count.shape[1]))
    for i,k in pairs:
     lo,num=int(start[i,k]),int(count[i,k]);friction[i]+=np.linalg.norm(ff[lo:lo+num],axis=-1).sum()
    linkforces.append(max([perbody.get(i,0.)+friction[i] for i in range(view.view.sensor_count)],default=0.))
   tilts=[];case_tilts=[]
   for c in ['A','B']:
    inst=self.base.scene['inst_'+c];tilts.append(math.degrees(math.acos(np.clip(rotation(arr(inst.data.root_quat_w)[0])[2,2],-1,1))));case_tilts.append(math.degrees(math.acos(np.clip(bodies['case_'+c][1][2,2],-1,1))))
   row=dict(lid=float(joint['A_lid']),lever=[float(joint[f'A{j}_lever']) for j in range(2)],slide=[float(joint[f'A{j}_slide']) for j in range(2)],catch=[float(joint[f'A{j}_catch']) for j in range(2)],engaged=[self.engaged[f'A{j}'] for j in range(2)],robot_contact=touch,case_tilt_deg=case_tilts,instrument_tilt_deg=tilts,wrist_n=forces,nonfinger_n=linkforces)
   self.history.update(row,dt)
   for i,side in enumerate(['left','right']):self.base.scene[side+'_robot']._s2_stop=self.history.guard_stopped[i]
  except Exception:self.error=traceback.format_exc();self.history.failed='measurement_unavailable'
 def credit(self):return self.history.score() if not self.error else 0.
 def reason(self):
  if self.error:return 'measurement_unavailable'
  if self.history.failed:return self.history.failed
  if self.credit()>=1:return 'success'
  if self.controls>=7964:return 'time_limit'
  return None
 def status(self):return dict(self.history.status(),controls=self.controls,score=self.credit(),measurement_error=self.error,engaged=self.engaged,mechanism_events=self.logs[-20:])
