"""Actual PhysX tensor contacts, using the installed ContactSensor backend API.
Raw normal contacts carry actor identities; filtered friction adds tangential forces.
Force sign/completeness still require actual calibration before scoring is enabled.
"""
import numpy as np
from d3_contacts_math import BodyRole,frame_contacts
def arr(x):
 if hasattr(x,'torch'):x=x.torch
 elif type(x).__module__.startswith('warp'):
  import warp as wp
  x=wp.to_torch(x)
 return x.detach().cpu().numpy().copy() if hasattr(x,'detach') else np.asarray(x).copy()

def contact_filter_index(path,filters):
 exact=[i for i,name in enumerate(filters) if path==name]
 if len(exact)==1:return exact[0]
 if len(exact)>1:raise ValueError('Duplicate actual rigid-body filters')
 ancestors=[(len(name),i) for i,name in enumerate(filters) if path.startswith(name+'/')]
 if not ancestors:raise ValueError('Unknown robot actor filter: '+path)
 longest=max(length for length,i in ancestors);matches=[i for length,i in ancestors if length==longest]
 if len(matches)!=1:raise ValueError('Ambiguous nearest rigid-body filter: '+path)
 return matches[0]

class TensorContacts:
 def __init__(self,base):
  from isaaclab_physx.physics import PhysxManager
  from isaaclab.sim.utils import get_current_stage
  from pxr import UsdPhysics
  self.base=base;self.stage=get_current_stage();self.roles={};self.actor_cache={};self.frames=0
  env=str(base.scene['pad_1'].root_view.prim_paths[0]).rsplit('/',1)[0];self.env=env
  self.sensors=[]
  for n in range(1,5):
   for link in ['base','lid']:
    path=f'{env}/carton_{n}/{link}';assert self.stage.GetPrimAtPath(path).HasAPI(UsdPhysics.RigidBodyAPI);self.sensors.append(path);self.roles[path]=BodyRole('mobile',item=f'carton{n}')
   path=f'{env}/pad_{n}';self.sensors.append(path);self.roles[path]=BodyRole('mobile',item=f'carton{n}')
  for label in 'AB':
   path=env+'/box_'+label;self.sensors.append(path);self.roles[path]=BodyRole('mobile',item=label)
  self.robot_roots=[env+'/LeftArm',env+'/RightArm']
  self.filters=sorted(str(prim.GetPath()) for prim in self.stage.Traverse() if prim.HasAPI(UsdPhysics.RigidBodyAPI) and any(str(prim.GetPath()).startswith(x+'/') for x in self.robot_roots))
  if not self.filters:raise ValueError('No actual robot rigid bodies registered as contact filters')
  simulation=PhysxManager.get_physics_sim_view()
  self.view=simulation.create_rigid_contact_view(self.sensors,filter_patterns=[self.filters]*len(self.sensors),max_contact_data_count=4096)
  if getattr(self.view,'_backend',None) is None:raise RuntimeError('PhysX tensor contact view failed to initialize with actual body paths')
  if list(self.view.sensor_paths)!=self.sensors:raise ValueError('Tensor sensor path order differs from registered actual bodies')
  if self.view.sensor_count!=14 or self.view.filter_count!=len(self.filters):raise ValueError('Unexpected physical contact view shape')
  self.latest=None
 def role(self,path):
  if path in self.roles:return self.roles[path]
  from pxr import UsdPhysics
  for arm,prefix in enumerate(self.robot_roots):
   if path==prefix or path.startswith(prefix+'/'):
    self.roles[path]=BodyRole('robot',arm=arm);return self.roles[path]
  prim=self.stage.GetPrimAtPath(path)
  if not prim:raise ValueError('Unknown actor path in contact measurement: '+path)
  p=prim
  while p and p.GetPath().pathString!='/':
   if p.HasAPI(UsdPhysics.RigidBodyAPI):
    if not UsdPhysics.RigidBodyAPI(p).GetKinematicEnabledAttr().Get():raise ValueError('Unregistered dynamic actor: '+path)
    break
   p=p.GetParent()
  self.roles[path]=BodyRole('static');return self.roles[path]
 def snapshot(self,dt):
  import warp as wp
  raw=self.view.get_raw_contact_data(dt);force,point,normal,separation,count,start,actorid=map(arr,raw)
  count=count.reshape(-1);start=start.reshape(-1)
  if int(count.sum())>=self.view.max_contact_data_count:raise ValueError('Contact data capacity exhausted')
  rows=[];indices=[]
  for i,(a,n) in enumerate(zip(start,count)):
   a,n=int(a),int(n)
   if a<0 or n<0 or a+n>len(force):raise ValueError('Contact data range invalid')
   indices.extend(range(a,a+n));rows.extend((i,k) for k in range(a,a+n))
  needed=sorted({int(actorid[k]) for k in indices}-set(self.actor_cache))
  if needed:
   cpu_ids=wp.array(np.asarray(needed,dtype=np.uint64),dtype=wp.uint64,device='cpu');paths=self.view.get_other_actor_paths_from_ids(cpu_ids)
   self.actor_cache.update(zip(needed,map(str,paths)))
  records=[];normal_sums=np.zeros((14,3));examples=[];normal_magnitude=np.zeros((14,len(self.filters)));raw_robot_vectors=np.zeros((14,len(self.filters),3));contact_present=np.zeros((14,len(self.filters)),dtype=bool)
  for i,k in rows:
   other=self.actor_cache[int(actorid[k])];f=float(force[k,0])*normal[k];pos=point[k];sep=float(separation[k,0])
   if not np.isfinite(np.r_[f,pos,sep]).all():raise ValueError('Nonfinite contact data')
   if not other:raise ValueError('Unresolved contact actor ID')
   self.role(other);normal_sums[i]+=f;records.append((self.sensors[i],other,sep,f*dt))
   if any(other.startswith(prefix+'/') for prefix in self.robot_roots):
    j=contact_filter_index(other,self.filters);normal_magnitude[i,j]+=abs(float(force[k,0]));raw_robot_vectors[i,j]+=f;contact_present[i,j]=contact_present[i,j] or sep<=0. or np.linalg.norm(f)>0.
   if len(examples)<24:examples.append({'sensor':self.sensors[i],'other':other,'force_n':f.tolist(),'point_w':pos.tolist(),'normal':normal[k].tolist(),'separation_m':sep})
  ff,fp,fc,fs=map(arr,self.view.get_friction_data(dt));friction=np.zeros((14,len(self.filters),3));friction_moment=np.zeros_like(friction)
  if int(fc.sum())>=self.view.max_contact_data_count:raise ValueError('Friction data capacity exhausted')
  for i in range(14):
   for j in range(len(self.filters)):
    a,n=int(fs[i,j]),int(fc[i,j])
    if a<0 or n<0 or a+n>len(ff):raise ValueError('Friction data range invalid')
    if n:
     values=ff[a:a+n];positions=fp[a:a+n]
     if not np.isfinite(values).all() or not np.isfinite(positions).all():raise ValueError('Nonfinite friction data')
     friction[i,j]=values.sum(0);friction_moment[i,j]=np.cross(positions,values).sum(0)
  graph=frame_contacts(records,self.roles,dt);net=arr(self.view.get_net_contact_forces(dt));matrix=arr(self.view.get_contact_force_matrix(dt))
  if not np.allclose(matrix,raw_robot_vectors,atol=.001,rtol=1e-4):raise ValueError('Raw actor normals disagree with actual filtered force matrix')
  self.frames+=1
  self.latest={'sensor_paths':self.sensors,'filter_patterns':self.filters,'filter_paths':self.view.filter_paths,'normal_contact_count':int(count.sum()),'normal_forces_n':normal_sums.tolist(),'net_api_forces_n':net.tolist(),'normal_force_matrix_n':matrix.tolist(),'normal_magnitude_by_filter_n':normal_magnitude.tolist(),'contact_present_by_filter':contact_present.tolist(),'friction_force_by_filter_n':friction.tolist(),'friction_moment_world_origin_by_filter_nm':friction_moment.tolist(),'direct_items':sorted(graph['direct']),'indirect_items':sorted(graph['indirect']),'touched_items':sorted(graph['touched']),'mobile_edges':sorted(graph['mobile_edges']),'examples':examples}
  return self.latest
