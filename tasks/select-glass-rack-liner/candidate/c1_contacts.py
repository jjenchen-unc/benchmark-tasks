"""Actual PhysX raw actor contacts. No distance-based substitutes for contact."""
import numpy as np
from c1_kit import arr
from c1_force_bound import effective_mu_bound,contact_norm_upper_bound
class Contacts:
 def __init__(self,base):
  from isaaclab_physx.physics import PhysxManager
  from isaaclab.sim.utils import get_current_stage
  from pxr import UsdPhysics
  self.base=base;self.stage=get_current_stage();self.cache={};self.env=str(base.scene['liner_1'].root_view.prim_paths[0]).rsplit('/',1)[0]
  self.mobile=[self.env+'/'+n for n in [*[f'liner_{i}' for i in range(1,7)],'cup_1','cup_2','tray']]
  self.robots=[]
  for side in ['left','right']:self.robots.extend(list(base.scene[side+'_robot'].root_view.link_paths[0]))
  self.sensors=self.mobile+self.robots;self.roles={p:20+i for i,p in enumerate(self.mobile[:6])};self.roles.update({self.mobile[6]:10,self.mobile[7]:11,self.mobile[8]:32});self.roles.update({p:1 for p in self.robots})
  statics=[]
  for prim in self.stage.Traverse():
   if not prim.HasAPI(UsdPhysics.CollisionAPI) or not str(prim.GetPath()).startswith(self.env+'/'):continue
   parent=prim;dynamic=False
   while parent and str(parent.GetPath())!='/':
    if parent.HasAPI(UsdPhysics.RigidBodyAPI):dynamic=True;break
    parent=parent.GetParent()
   if not dynamic:statics.append(str(prim.GetPath()))
  self.filters=self.sensors+statics
  coefficients=[]
  for prim in self.stage.Traverse():
   if prim.HasAPI(UsdPhysics.MaterialAPI):
    material=UsdPhysics.MaterialAPI(prim)
    coefficients.extend([float(material.GetStaticFrictionAttr().Get()),float(material.GetDynamicFrictionAttr().Get())])
  self.mu_bound=effective_mu_bound(coefficients);self.material_coefficients=coefficients
  self.capacity=16384
  sim=PhysxManager.get_physics_sim_view();self.view=sim.create_rigid_contact_view(self.sensors,filter_patterns=[self.filters]*len(self.sensors),max_contact_data_count=self.capacity)
  if self.view.sensor_count!=len(self.sensors) or list(self.view.sensor_paths)!=self.sensors:raise RuntimeError('Contact sensor body registration/order differs')
 def role(self,path):
  if path in self.roles:return self.roles[path]
  if any(path==p or path.startswith(p+'/') for p in self.robots):return 1
  if path==self.env+'/rack' or path.startswith(self.env+'/rack/'):return 30
  from pxr import UsdPhysics
  p=self.stage.GetPrimAtPath(path)
  if not p:raise ValueError('Unresolved contact actor '+path)
  while p and str(p.GetPath())!='/':
   if p.HasAPI(UsdPhysics.RigidBodyAPI) and not UsdPhysics.RigidBodyAPI(p).GetKinematicEnabledAttr().Get():raise ValueError('Unregistered dynamic contact actor '+path)
   p=p.GetParent()
  return 31
 def sample(self,dt):
  import warp as wp
  force,point,normal,separation,count,start,actor=map(arr,self.view.get_raw_contact_data(dt))
  count=count.reshape(-1);start=start.reshape(-1)
  if count.sum()>=self.capacity or np.any(count<0):raise ValueError('Contact buffer overflow')
  rows=[]
  for i,(a,n) in enumerate(zip(start,count)):
   a,n=int(a),int(n)
   if a<0 or a+n>len(force):raise ValueError('Invalid contact range')
   rows.extend((i,k) for k in range(a,a+n))
  needed=sorted({int(actor[k]) for i,k in rows}-set(self.cache))
  if needed:
   ids=wp.array(np.array(needed,dtype=np.uint64),dtype=wp.uint64,device='cpu');self.cache.update(zip(needed,map(str,self.view.get_other_actor_paths_from_ids(ids))))
  touch=set();links={};examples=[];self.diagnostic_contacts=[]
  for i,k in rows:
   f=float(force[k,0]);sep=float(separation[k,0]);p=self.cache[int(actor[k])]
   if not p or not np.isfinite(np.r_[f,sep,point[k],normal[k]]).all():raise ValueError('Missing/nonfinite raw contacts')
   if sep<=0 or abs(f)>0:
    a,b=self.role(self.sensors[i]),self.role(p);touch.add((a,b));touch.add((b,a))
    if a==1 and not self.sensors[i].endswith(('/tip_left','/tip_right','/base')):links[self.sensors[i]]=links.get(self.sensors[i],0.)+abs(f)*float(np.linalg.norm(normal[k]))
   self.diagnostic_contacts.append({'sensor':self.sensors[i],'other':p,'actor_id':int(actor[k]),'force_n':f,'separation_m':sep,'point':point[k].tolist(),'normal':normal[k].tolist()})
   if len(examples)<20:examples.append({'sensor':self.sensors[i],'other':p,'force_n':f,'separation_m':sep})
  # Tangential contact magnitudes for link safety: all dynamic objects and
  # robot bodies are filters. Static shapes are included. Friction anchors do not have a proven1:1
  # normal-contact mapping and are not used as per-contact magnitudes.
  ff,fp,fc,fs=map(arr,self.view.get_friction_data(dt))
  if int(fc.sum())>=self.capacity:raise ValueError('Friction buffer overflow')
  for i,path in enumerate(self.sensors):
   if self.role(path)!=1 or path.endswith(('/tip_left','/tip_right','/base')):continue
   for j in range(fc.shape[1]):
    a,n=int(fs[i,j]),int(fc[i,j]);v=ff[a:a+n]
    if a<0 or a+n>len(ff) or not np.isfinite(v).all():raise ValueError('Invalid friction range')
    # Anchors can combine several contacts; they are recorded but are NOT
    # mislabelled as a per-contact vector mapping. Coulomb bound below
    # avoids cancellation and the unknown normal/anchor correspondence.
    pass
  links={p:contact_norm_upper_bound([n],self.mu_bound) for p,n in links.items()}
  return sorted(touch),links,examples
 def close(self):self.view=None
