"""Actual passive-material integration. Task credit remains gated while observers are verified."""
from pathlib import Path
import math,numpy as np
from d3_material import GlobalPad,rotation_xyzw
from d3_shock import ShockWindow,proper_acceleration
def arr(x):
 if hasattr(x,'torch'):x=x.torch
 elif type(x).__module__.startswith('warp'):
  import warp as wp
  x=wp.to_torch(x)
 return x.detach().cpu().numpy().copy() if hasattr(x,'detach') else np.asarray(x).copy()
class Physics:
 def __init__(self,emb,fixture,specs):
  self.emb=emb;self.base=emb._base;self.fixture=fixture;self.specs=specs;self.settling=True;self.controls=0;self.samples=0;self.physics_s=0.;self.materials=[];self.contact_error=None;self.contact_frames=0;self.contact_nonempty=0;self.contact_examples=[]
  for i,c in enumerate(fixture['cartons']):
   n=i+1;carton=self.base.scene[f'carton_{n}'];face=self.base.scene[f'pad_{n}'];idx=carton.body_names.index('base');mass=arr(carton.root_view.get_masses())[0];com=np.array(c['base_com_local_m']);If=.015*np.diag([.0605**2+.003**2/12,.0805**2+.003**2/12,.0805**2+.0605**2])
   model=GlobalPad(fixture['parameters']['foam_stiffness'][i],fixture['parameters']['foam_stroke'][i],float(mass[idx]),.015,np.array(c['base_inertia_body_kg_m2']),If,rest_offset=tuple(np.array([0,0,.0285])-com))
   self.materials.append(dict(number=n,carton=carton,face=face,index=idx,model=model,phase='idle',latched=False))
  from d3_tensor_contacts import TensorContacts
  self.tensor_contacts=TensorContacts(self.base)
  self.initialize_episode_state()
 def initialize_episode_state(self):
  import torch
  self.settling=True
  for c in self.materials:
   obj=c['carton'];obj.write_joint_position_limit_to_sim_index(limits=torch.tensor([[[0.,1.75]]],device=self.base.device),warn_limit_violation=False)
   obj.data.default_joint_pos.torch.fill_(1.75);q=torch.full((1,1),1.75,device=self.base.device);obj.write_joint_state_to_sim(q,torch.zeros_like(q));c.update(phase='idle',latched=False)
  self.controls=0
 def begin_episode(self):
  self.settling=False;self.controls=0;self.samples=0;self.physics_s=0.;self.shocks={x:ShockWindow() for x in 'AB'};self.prev_v={x:arr(self.base.scene['box_'+x].data.root_lin_vel_w)[0] for x in 'AB'};self.contact_error=None;self.contact_frames=0;self.contact_nonempty=0;self.contact_examples=[];self.update_visuals()
 def needs_fine_step(self):return any(c['phase'] in ['hold','drop','window'] for c in self.materials)
 def pre_step(self,dt):
  import torch
  for c in self.materials:
   obj,face=c['carton'],c['face'];cache=getattr(self,'_post_packet',None) if not self.settling else None
   if cache is not None:b=cache['carton_com'][c['number']][c['index']];f=cache['faces'][c['number']]
   else:b=arr(obj.data.body_com_state_w)[0,c['index']];f=arr(face.data.root_com_state_w)[0]
   unpack=lambda s:(s[:3],s[3:7],s[7:10],s[10:13]);w=c['model'].wrench(unpack(b),unpack(f),dt);force=np.zeros((1,len(obj.body_names),3),np.float32);torque=np.zeros_like(force);force[0,c['index']]=w['base_force'];torque[0,c['index']]=w['base_torque']
   obj.set_external_force_and_torque(torch.as_tensor(force,device=self.base.device),torch.as_tensor(torque,device=self.base.device),is_global=True)
   face.set_external_force_and_torque(torch.as_tensor(w['face_force'],dtype=torch.float32,device=self.base.device).reshape(1,1,3),torch.as_tensor(w['face_torque'],dtype=torch.float32,device=self.base.device).reshape(1,1,3),is_global=True)
   c['last_wrench']=w;c['pre_base_com']=b;c['pre_face_com']=f
 def post_step(self,dt):
  if self.settling:return
  self.samples+=1;self.physics_s+=dt
  for x in 'AB':
   cache=getattr(self,'_post_packet',None);v=cache['boxes'][x][7:10] if cache is not None else arr(self.base.scene['box_'+x].data.root_lin_vel_w)[0];self.shocks[x].add(dt,proper_acceleration(v,self.prev_v[x],dt));self.prev_v[x]=v
  for c in self.materials:c['post_face_com']=self._post_packet['faces'][c['number']] if getattr(self,'_post_packet',None) is not None else arr(c['face'].data.root_com_state_w)[0]
  self.read_contact_stream(dt)
  if self.samples%max(1,round(.05/dt))==0:self.update_visuals()
 def read_contact_stream(self,dt):
  try:
   frame=self.tensor_contacts.snapshot(dt);self.contact_frames+=1
   if frame['normal_contact_count']>0:self.contact_nonempty+=1
   if not self.contact_examples:self.contact_examples=frame['examples']
  except Exception as e:self.contact_error=type(e).__name__+': '+str(e)
 def update_visuals(self):
  from pxr import UsdGeom
  from isaaclab.sim.utils import get_current_stage
  stage=get_current_stage()
  for c in self.materials:
   b=arr(c['carton'].data.body_link_state_w)[0,c['index']];f=arr(c['face'].data.root_link_state_w)[0];B=rotation_xyzw(b[3:7]);F=rotation_xyzw(f[3:7])
   lower=np.array([[x,y,.001] for x in [-.082,.082] for y in [-.062,.062]]);lower=(b[:3]+lower@B.T-f[:3])@F
   upper=np.array([[x,y,-.0015] for x in [-.082,.082] for y in [-.062,.062]]);path=c['face'].root_view.prim_paths[0]+'/foam_skin';mesh=UsdGeom.Mesh(stage.GetPrimAtPath(path))
   if mesh:mesh.GetPointsAttr().Set(np.vstack([lower,upper]).astype(np.float32))
 def status(self):
  return {'stage':'foundation_unverified_contacts_and_score','submission_ready':False,'controls':self.controls,'physics_samples':self.samples,'physics_s':self.physics_s,'contact_frames':self.contact_frames,'contact_nonempty_frames':self.contact_nonempty,'contact_error':self.contact_error,'shock':{x:w.status() for x,w in getattr(self,'shocks',{}).items()},'cartons':[{'number':c['number'],'phase':c['phase'],'latched':c['latched'],'lid_q_rad':arr(c['carton'].data.joint_pos).tolist()} for c in self.materials]}

 def close(self):
  self.tensor_contacts.view=None
