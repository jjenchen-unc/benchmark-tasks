"""D3 physical-latch events and finite-force automatic drop tester.
Pure state machines: a backend applies returned joint limits and real forces.
No object pose/velocity/score writes. Runtime channels need independent verification.
"""
import math
import numpy as np
class LidLatch:
 def __init__(self):self.latched=False;self.armed=True;self.release_s=0.
 def update(self,dt,q,robot_lid_contact,normal_push_n,hinge_constraint_nm):
  values=np.array([dt,q,normal_push_n,hinge_constraint_nm],float)
  if not np.isfinite(values).all() or not 0<dt<=.002 or normal_push_n<0 or type(robot_lid_contact) is not bool:raise ValueError('Actual finite lid contact/reaction required')
  event=None
  if not self.latched:
   if q<.012 and robot_lid_contact and normal_push_n>=5.:
    self.latched=True;self.release_s=0.;event='lock'
  else:
   self.release_s=self.release_s+dt if robot_lid_contact and hinge_constraint_nm<-.5 else 0.
   if self.release_s>=.05-1e-12:
    self.latched=False;self.armed=True;self.release_s=0.;event='unlock'
  return event
class Harness:
 def __init__(self,number,rest_z):
  if type(number) is not int or not 1<=number<=4:raise ValueError('Unknown carton')
  self.number=number;self.rest_z=float(rest_z);self.phase='idle';self.quiet=0.;self.boxes=[];self.peak={};self.records=[];self.contact=False;self.max_rise=0.;self.reference=None;self.t0=None;self.release=None;self.land=None
 def fine(self):return self.phase in ['hold','drop','window']
 def touched(self,touched):return bool(set(touched)&({f'carton{self.number}'}|set(self.boxes)))
 def pre(self,dt,now,root,com,R,velocity,angular_velocity,latched,armed,touched,inside,total_mass):
  # root/link and COM positions have different roles; do not mix them.
  root,com,R,velocity,angular_velocity=[np.asarray(v,dtype=float) for v in [root,com,R,velocity,angular_velocity]]
  if any(not np.isfinite(v).all() for v in [root,com,R,velocity,angular_velocity]) or not 0<dt<=.002 or not math.isfinite(now) or not math.isfinite(total_mass) or total_mass<=0:raise ValueError('Invalid measured harness frame')
  started=False
  if self.phase=='idle':
   busy=bool(set(touched)&({f'carton{self.number}'}|set(inside)))
   rest=root[2]<self.rest_z+.01 and R[2,2]>.99
   self.quiet=self.quiet+dt if latched and armed and not busy and rest else 0.
   if self.quiet>=2.-1e-12:
    self.phase='lift';self.t0=now;self.reference=com.copy();self.root_reference=root.copy();self.rotation=R.copy();self.mass=float(total_mass);self.boxes=list(inside);self.peak={n:0. for n in inside};self.quiet=0.;self.contact=False;self.max_rise=0.;self.release=None;self.land=None;started=True
  if self.phase in ['lift','hold','drop','window'] and self.touched(touched):self.contact=True
  force=np.zeros(3);moment=np.zeros(3)
  if self.phase=='lift':
   u=min(1.,max(0.,now-self.t0));height=.3*(10*u**3-15*u**4+6*u**5);vz=.3*(30*u**2-60*u**3+30*u**4)
   if u>=1.:self.phase='hold';self.t0=now
  if self.phase=='hold':
   height=.3;vz=0.
   if now-self.t0>=.3-1e-12:
    self.phase='drop';self.release={'t':now,'rise_m':float(root[2]-self.rest_z),'tucked':bool(latched)}
  if self.phase in ['lift','hold']:
   force=400.*(self.reference+[0.,0.,height]-com)+30.*(np.array([0.,0.,vz])-velocity)+np.array([0.,0.,self.mass*9.81])
   norm=np.linalg.norm(force)
   if norm>25.:force*=25./norm
   error=.5*sum((np.cross(R[:,i],self.rotation[:,i]) for i in range(3)),np.zeros(3));moment=5.*error-.3*angular_velocity;norm=np.linalg.norm(moment)
   if norm>2.:moment*=2./norm
  return {'force_w_n':force,'moment_w_nm':moment,'started':started,'armed':False if started else armed,'phase':self.phase}
 def post(self,now,root_z,latched,inside,shock_abs_g,broken):
  if self.phase not in ['lift','hold','drop','window']:return
  self.max_rise=max(self.max_rise,float(root_z-self.rest_z))
  if self.phase in ['drop','window']:
   for n in self.boxes:
    g=float(shock_abs_g[n])
    if not math.isfinite(g) or g<0:raise ValueError('Nonfinite source shock')
    self.peak[n]=max(self.peak[n],g)
  if self.phase=='drop':
   elapsed=now-self.release['t']
   if (root_z<self.rest_z+.003 and elapsed>.1) or elapsed>1.:
    self.phase='window';self.land=now
  elif self.phase=='window' and now-self.land>=2.:
   self.records.append({'carton':self.number,'boxes':list(self.boxes),'peak_g':dict(self.peak),'robot_contact':self.contact,'released':True,'rise_m':self.max_rise,'postlanding_s':now-self.land,'completed':True,'ok':{n:bool(self.peak[n]<50. and n in inside and self.release['tucked'] and latched and not broken[n] and not self.contact) for n in self.boxes},'t_release':self.release['t'],'t_land':self.land})
   self.phase='idle';self.quiet=0.;self.boxes=[];self.peak={}
 def open_record(self):
  if self.phase=='idle':return None
  return {'carton':self.number,'boxes':list(self.boxes),'completed':False}
