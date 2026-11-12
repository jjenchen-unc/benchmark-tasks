import math,numpy as np
T=np.array([.42,-.17,-.05]);A=np.diag([-1.,-1.,1.])
def arr(x):
 x=getattr(x,'torch',x)
 return x.detach().cpu().numpy() if hasattr(x,'detach') else x.numpy() if hasattr(x,'numpy') else np.asarray(x)
def rotation(q):
 x,y,z,w=q
 return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def native(p):return A@(np.asarray(p)-T)
def world(p):return A@np.asarray(p)+T
def extent(kind,size,R):
 if kind=='box':return abs(R)@size
 if kind=='sphere':return np.ones(3)*size[0]
 z=R[:,2];return size[1]*abs(z)+size[0]*np.sqrt(np.maximum(0,1-z*z))
def contents(drawer,box,jar,jar_touch):
 p,R=drawer;rel=R.T@(box-p);jr=R.T@(jar[0]-p)
 boxin=abs(rel[0])<.1 and abs(rel[1])<.13 and -.04<rel[2]<.03
 jin=abs(jr[0])<.075 and abs(jr[1])<.105 and -.035<jr[2]<.065
 jtable=abs(jar[0][2]-.835)<.005 and jar[1][2,2]>math.cos(math.radians(10)) and jar[0][0]<.075
 return bool(boxin and (jin or jtable) and not jar_touch)
class History:
 def __init__(self):self.damage_grace_s=0.;self.raw=0.;self.time=0.;self.hold=0.;self.wrist=np.zeros(2);self.failure=None;self.samples=0;self.last=None
 def update(self,r,dt):
  if not 0<dt<=.05:raise ValueError('invalid physical clock')
  self.samples+=1;self.time+=dt;self.last=r
  numeric=np.r_[r['drawer_p'],r['drawer_R'].ravel(),r['wrist'],r['max_x'],r['jar_force'],r['jar_speed'],r['jar_cos']]
  if not np.isfinite(numeric).all():self.failure='nonfinite_measurement'
  self.wrist=np.where(np.asarray(r['wrist'])>80,self.wrist+dt,0.)
  if np.any(self.wrist>=.1-1e-10):self.failure='wrist_80N_100ms'
  if self.time>self.damage_grace_s and (max(r['jar_force'])>15 or max(r['jar_speed'])>.3 or min(r['jar_cos'])<math.cos(math.radians(30))):self.failure='jar_damage'
  if r['drop']:self.failure='object_out_of_bounds'
  clear=r['max_x']<.12
  table=abs(r['drawer_p'][2]-.835)<.006 and r['drawer_p'][0]+.10<.10 and clear
  ok=r['contents'] and table and r['drawer_R'][2,2]>math.cos(math.radians(3)) and not r['drawer_touch']
  self.hold=self.hold+dt if ok and not self.failure else 0.
  self.raw=0. if self.failure or not r['contents'] or not clear else (1. if self.hold>=1.-1e-9 else .5 if table else .25)
 def result(self,validated=False):
  return {'score':float(getattr(self,'raw',0)) if validated else 0.,'candidate_score':float(getattr(self,'raw',0)),'hold_s':self.hold,'physics_s':self.time,'samples':self.samples,'failure':self.failure,'credit_gate':'validated' if validated else 'runtime_channels_unvalidated'}
