"""Environment-owned passive mechanism and numeric sensor history at2ms.
No policy accesses this module's write methods. Contact selections require real validation.
"""
import json,math,os
from pathlib import Path
import numpy as np
from contacts import Contacts,arr,body_paths,rot_xyzw
from mechanism import Hydraulic
from scoring import History
class Physics:
 def __init__(self,emb,fixture,specs):
  self.emb,self.base,self.f,self.specs=emb,emb._base,fixture,specs;self.settling=True;self.error=None
  from isaaclab_physx.physics import PhysxManager
  self.simview=PhysxManager.get_physics_sim_view();self.jacks={tag:self.base.scene['jack_'+tag] for tag in ('S','T')}
  self.cabinet=self.base.scene['cabinet'];robots=sum((body_paths(self.base.scene[side+'_robot']) for side in ('left','right')),[])
  paths={tag:body_paths(a) for tag,a in self.jacks.items()};cab=self.cabinet.root_view.prim_paths
  if not cab or len(cab)!=1:raise ValueError('one cabinet rigid body required')
  self.cab_path=str(cab[0]);env=self.cab_path.rsplit('/',1)[0]
  pads={tag:[p for p in paths[tag] if p.endswith('/ram_'+tag)] for tag in ('S','T')}
  self.pad={tag:Contacts(self.simview,pads[tag],[self.cab_path,*robots]) for tag in pads}
  from pxr import Usd,UsdPhysics
  import omni.usd
  stage=omni.usd.get_context().get_stage()
  moving=[self.cab_path,*robots,*sum(paths.values(),[])]
  static=[];rails=[]
  for prim in Usd.PrimRange(stage.GetPrimAtPath(env),Usd.TraverseInstanceProxies()):
   path=str(prim.GetPath())
   if prim.HasAPI(UsdPhysics.CollisionAPI) and not any(path==v or path.startswith(v+'/') for v in moving):
    static.append(path)
    if any(path==env+'/'+v or path.startswith(env+'/'+v+'/') for v in ('rail_a','rail_b')):rails.append(path)
  if len(rails)!=2:raise ValueError('resolve both actual rail colliders: '+str(rails))
  self.robot=Contacts(self.simview,[self.cab_path],robots)
  self.rails=Contacts(self.simview,[self.cab_path],rails)
  self.carried=Contacts(self.simview,[self.cab_path],pads['S'])
  self.all_contacts=Contacts(self.simview,[self.cab_path],[*robots,*sum(paths.values(),[]),*static])
  self.selections={'pad':{k:v.selection for k,v in self.pad.items()},'robot':self.robot.selection,'rails':self.rails.selection,'all':self.all_contacts.selection}
  self.trace=open(os.environ['HJ_PHYSICS_TRACE'],'w',buffering=1) if os.environ.get('HJ_PHYSICS_TRACE') else None
  self.episode=0
  self.initialize()
 def initialize(self):
  self.episode+=1
  self.hyd={t:Hydraulic(self.f['parameters'],t) for t in self.jacks};self.forces={t:0. for t in self.jacks};self.history=History();self.controls=0;self.done=False;self.error=None
  for tag,a in self.jacks.items():
   q=a.data.default_joint_pos.torch.clone();v=a.data.default_joint_vel.torch.clone();a.write_joint_state_to_sim(q,v)
  self.last=None;self.first_failure=None
 def begin(self):
  self.history=History();self.controls=0;self.done=False;self.error=None;self.settling=False
 def pre(self,dt):
  import torch
  for tag,a in self.jacks.items():
   q=dict(zip(a.joint_names,arr(a.data.joint_pos)[0]));v=dict(zip(a.joint_names,arr(a.data.joint_vel)[0]));h=self.hyd[tag]
   V,tau,closed=h.update(q['lever_'+tag],v['lever_'+tag],q['valve_'+tag],self.forces[tag],dt)
   target=torch.zeros_like(a.data.joint_pos.torch);effort=torch.zeros_like(target)
   target[:,a.joint_names.index('ram_'+tag)]=float(V);target[:,a.joint_names.index('lever_'+tag)]=.55;effort[:,a.joint_names.index('lever_'+tag)]=float(tau)
   a.set_joint_position_target(target);a.set_joint_effort_target(effort)
 def post(self,dt):
  if self.error:return
  try:
   for tag,c in self.pad.items():
    c.read(dt);self.forces[tag]=sum(abs(v['normal_force_n']*v['normal_w'][2]) for v in c.last_contacts)
   robot,_,_=self.robot.read(dt);rails,_,_=self.rails.read(dt);pad,_,_=self.carried.read(dt);self.all_contacts.read(dt)
   p=arr(self.cabinet.data.root_pos_w)[0];q=arr(self.cabinet.data.root_quat_w)[0];R=rot_xyzw(q);origin=arr(self.base.scene.env_origins)[0]
   glass=[g for b in self.f['bodies'] if b['name']=='cabinet' for g in b['geoms'] if g['name'].startswith('cabinet_glass_')];peak=0.
   for c in self.all_contacts.last_contacts:
    local=R.T@(np.array(c['point_w'])-p)
    # Cabinet panels share a single rigid body. Conservative point-in-expanded-panel
    # attributes boundary frame contacts to glass too; disclose and calibrate.
    if any(np.all(abs(local-np.array(g['pos']))<=np.array(g['size'])+.001) for g in glass):peak=max(peak,abs(c['normal_force_n']))
   wrist=[]
   for side in ('left','right'):
    s=self.base.scene[side+'_hj_wrench'];wrist.append(float(np.linalg.norm(arr(s.data.force)[0,s.body_names.index('gripper')])))
   row=dict(height=float(p[2]-origin[2]-(.935+self.f['translation'][2])),vz=float(arr(self.cabinet.data.root_lin_vel_w)[0,2]),speed=float(np.linalg.norm(arr(self.cabinet.data.root_lin_vel_w)[0])),spin=float(np.linalg.norm(arr(self.cabinet.data.root_ang_vel_w)[0])),tilt=math.acos(float(np.clip(R[2,2],-1,1))),glass_force=peak,wrist_force=wrist,on_rails=rails,on_pad=pad,robot_contact=robot,flow_lowered=self.hyd['S'].lowered,ram_volume=self.hyd['S'].V,pad_forces=self.forces.copy(),controls=self.controls)
   self.last=row
   if not self.settling:self.history.update(row,dt)
   from telemetry import dumps,substep_detail
   first_failure=not self.settling and self.history.failure is not None and self.first_failure is None
   entry=dict(episode=self.episode,dt=dt,settling=self.settling,row=row,history=dict(physics_s=self.history.t,samples=self.history.samples,failure=self.history.failure,wrist_high_s=self.history.wrist_high))
   if not self.settling and (self.controls<=4 or first_failure):entry['detail']=substep_detail(self)
   if first_failure:self.first_failure=json.loads(dumps(entry))
   if self.trace:self.trace.write(dumps(entry)+'\n')
  except Exception as ex:
   self.error=f'{type(ex).__name__}: {ex}';self.history.failure='measurement_unavailable'
 def status(self):
  # Empty until root produces matching actual channel-calibration evidence.
  record=json.loads(Path(__file__).with_name('validated_channels.json').read_text())
  r=self.history.result(record['channels']);r.update(measurement_error=self.error,controls=self.controls,last=self.last,physics_device=str(self.base.device));return r
