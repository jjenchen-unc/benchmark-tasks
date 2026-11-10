"""Environment-only passive spring/cable-latch realization and substep sensors.
No policy entry point accepts fixture poses, limits, forces or latch state.
"""
import json,os,math
import numpy as np
from s5_kit import arr,FIXTURE
from s5_rules import Latches,History,torque,DEG
from s5_contacts import Contacts,body_paths,rot_xyzw
class Physics:
 def __init__(self,emb):
  self.emb=emb;self.base=emb._base;self.settling=True;self.done=False;self.trace=None;self.controls=0;self.error=None;self.last=None
  from isaaclab_physx.physics import PhysxManager
  self.simview=PhysxManager.get_physics_sim_view();robots=body_paths(self.base.scene['left_robot'])+body_paths(self.base.scene['right_robot']);boxes=body_paths(self.base.scene['A_box_R']);envroot=boxes[0].split('/A_box_R')[0]
  # Static station table has no rigidbody at its Xform root. Filter the
  # actual registered collision leaves (verified by placed positive/air tests).
  from pxr import Usd,UsdPhysics
  import omni.usd
  table_path=envroot+'/table';table_prim=omni.usd.get_context().get_stage().GetPrimAtPath(table_path)
  if not table_prim.IsValid():raise ValueError('missing actual station table')
  table_colliders=[str(p.GetPath()) for p in Usd.PrimRange(table_prim)
                   if p.HasAPI(UsdPhysics.CollisionAPI) and p.GetAttribute('physics:collisionEnabled').Get() is not False]
  if not table_colliders:raise ValueError('station table collision leaves unresolved')
  self.robot_contact=Contacts(self.simview,boxes,robots);self.table_contact=Contacts(self.simview,boxes,table_colliders)
  self.selection=dict(robots=robots,box=boxes,table=table_path,table_collision_prims=table_colliders);self.initialize()
 def initialize(self):
  import torch
  self.settling=True;self.latches=Latches(FIXTURE['cabinets']);self.hist=History();self.done=False;self.error=None;self.controls=0;self.last=None
  for name in ['cab_A','cab_B']:
   obj=self.base.scene[name];q=torch.zeros_like(obj.data.joint_pos.torch);obj.write_joint_state_to_sim(q,q);obj.set_joint_effort_target(q)
  self.pre_step()
 def begin(self):
  # Keep physical/latch state reached by passive settling; reset only evaluator
  # time histories. Root/joint restoration took place inside actual env.reset.
  self.hist=History();self.controls=0;self.settling=False;self.error=None;self.done=False
  if self.trace:self.trace.close()
  self.trace=open(os.environ['S5_TRACE'],'a',buffering=1) if os.environ.get('S5_TRACE') else None
  if self.trace:self.trace.write(json.dumps({'event':'begin','selection':self.selection,'clock_dt':float(self.base.sim.get_physics_dt())})+'\n')
 def joint_state(self):
  q={};v={}
  for n in ['cab_A','cab_B']:
   obj=self.base.scene[n];q.update(zip(obj.joint_names,arr(obj.data.joint_pos)[0].tolist()));v.update(zip(obj.joint_names,arr(obj.data.joint_vel)[0].tolist()))
  return q,v
 def pre_step(self):
  import torch
  q,v=self.joint_state();self.latches.update(q,self.hist.time)
  for name in ['cab_A','cab_B']:
   obj=self.base.scene[name];c=FIXTURE['cabinets'][name[-1]];eff=[];targets=[];limits=[]
   for j in obj.joint_names:
    n=j[0]
    if '_hinge_' in j:
     s=j[-1];key=n+s;eff.append(torque(q[j],c['f2'+s],c['two'+s],self.latches.latch[key],self.latches.armed[key]));targets.append(0.);limits.append([-.15*DEG,self.latches.upper(key)])
    else:eff.append(0.);targets.append(-.004);limits.append([0.,.012])
   tensor=lambda x:torch.tensor([x],device=self.base.device,dtype=torch.float32)
   obj.set_joint_effort_target(tensor(eff));obj.set_joint_position_target(tensor(targets));obj.write_joint_position_limit_to_sim_index(limits=tensor(limits),warn_limit_violation=False)
 def row(self,dt):
  q,v=self.joint_state();obj=self.base.scene['A_box_R'];state=arr(obj.data.root_link_state_w)[0];origin=arr(self.base.scene.env_origins)[0];p=state[:3]-origin;R=rot_xyzw(state[3:7]);pose=arr(obj.data.root_com_state_w)[0];spatial=float(np.linalg.norm(pose[7:13]));table=self.emb.rig.name
  bounds=[]
  body=next(b for b in FIXTURE['bodies'] if b['name']=='A_box_R')
  for g in body['geoms']:
   centre=p+R@np.array(g['pos']);extent=abs(R)@np.array(g['size']);bounds.extend([centre-extent,centre+extent])
  lo=np.min(bounds,axis=0);hi=np.max(bounds,axis=0)
  # Actual station table is centre x=.36, widths1.1x1.3, top=.7475.
  # Full box envelope must stay inside tabletop, not only its centre.
  inside=bool(lo[0]>=-.19 and hi[0]<=.91 and lo[1]>=-.65 and hi[1]<=.65)
  tc,tf,tn=self.table_contact.read(dt);rc,rf,rn=self.robot_contact.read(dt);forces=[]
  for side in ['left','right']:
   sensor=self.base.scene[side+'_s5_wrench'];forces.append(float(np.linalg.norm(arr(sensor.data.force)[0,sensor.body_names.index('gripper')])))
  x,y,z=p;lx=x-.25;ly=y+.29
  return dict(door_q={n+s:q[f'{n}_hinge_{s}'] for n in 'AB' for s in 'LR'},door_v={n+s:v[f'{n}_hinge_{s}'] for n in 'AB' for s in 'LR'},wrist_force_n=forces,released=any(e[1]=='AR' and e[2]=='release' for e in self.latches.events),latches=dict(self.latches.latch),paddle_q={j:q[j] for j in q if 'paddle' in j},box=dict(pos=p.tolist(),z=float(z),upright_cos=float(R[2,2]),spatial_speed=spatial,outside_cabinet=not(-.02<lx<.20 and abs(ly)<.20),inside_table=inside,table_contact=tc,robot_contact=rc),contacts=dict(table=self.table_contact.last_contacts,robot=self.robot_contact.last_contacts))
 def post_step(self,dt):
  if self.settling:return
  try:
   row=self.row(dt)
   if not np.isfinite(np.r_[list(row['door_q'].values()),list(row['door_v'].values()),row['box']['pos'],row['box']['spatial_speed'],row['wrist_force_n']]).all():raise ValueError('nonfinite physical state')
   self.hist.add(row,dt);self.last=row
   if self.trace:self.trace.write(json.dumps(dict(dt=dt,physics_s=self.hist.time,measurement=row,status=self.hist.result()))+'\n')
  except Exception as e:self.error=type(e).__name__+': '+str(e);self.hist.failure='measurement_unavailable'
 def reason(self):
  if self.hist.failure:return self.hist.failure
  if self.controls>=3646:return 'time_limit'
  if self.status()['score']>=1:return 'success'
  return None
 def status(self):return dict(self.hist.result(),controls=self.controls,measurement_error=self.error,mechanism_events=self.latches.events,physics_verified=False)
 def close(self):
  if self.trace:self.trace.close();self.trace=None
