"""Actual filtered normal+friction forces/moments; normal + friction channels; B1 calibration pending."""
import numpy as np
def arr(x):
 if hasattr(x,'torch'):x=x.torch
 elif type(x).__module__.startswith('warp'):
  import warp as wp
  x=wp.to_torch(x)
 return x.detach().cpu().numpy().copy() if hasattr(x,'detach') else np.asarray(x).copy()
class Contacts:
 def __init__(self,sim,sensors,filters):
  self.sensors,self.filters=sensors,filters;self.capacity=8192
  self.view=sim.create_rigid_contact_view(sensors,filter_patterns=[filters]*len(sensors),max_contact_data_count=self.capacity)
  if self.view.sensor_count!=len(sensors) or self.view.filter_count!=len(filters):raise ValueError('unresolved actual contact view')
 def read(self,dt,anchor=None):
  force,point,normal,separation,count,start=map(arr,self.view.get_contact_data(dt));ff,fp,fc,fs=map(arr,self.view.get_friction_data(dt))
  if int(count.sum())>=self.capacity or int(fc.sum())>=self.capacity:raise ValueError('contact capacity exhausted')
  total=np.zeros((len(self.sensors),len(self.filters),3));moment=np.zeros_like(total);hit=np.zeros(total.shape[:2],bool)
  magnitudes=np.zeros(len(self.sensors))
  for i in range(len(self.sensors)):
   for j in range(len(self.filters)):
    a,n=int(start[i,j]),int(count[i,j]);b,m=int(fs[i,j]),int(fc[i,j])
    if min(a,n,b,m)<0 or a+n>len(force) or b+m>len(ff):raise ValueError('invalid contact range')
    # Empty pairs contribute exact initialized zeros; range validation stays above.
    if n==0 and m==0:continue
    f=force[a:a+n].reshape(-1,1)*normal[a:a+n];p=point[a:a+n];t=ff[b:b+m];pt=fp[b:b+m]
    if not np.isfinite(np.r_[f.ravel(),p.ravel(),t.ravel(),pt.ravel()]).all():raise ValueError('nonfinite contact')
    hit[i,j]=bool(np.any(separation[a:a+n]<=0) or np.any(f))
    total[i,j]=f.sum(0)+t.sum(0)
    magnitudes[i]+=float(np.linalg.norm(f,axis=1).sum()+np.linalg.norm(t,axis=1).sum())
    origin=np.zeros(3) if anchor is None else np.asarray(anchor)
    moment[i,j]=np.cross(p-origin,f).sum(0)+np.cross(pt-origin,t).sum(0)
  self.magnitude_by_sensor=dict(zip(self.sensors,map(float,magnitudes)))
  return hit,total,moment
