"""Model-only geometric pad adapter; inherited native sampling otherwise exact."""
import math
import numpy as np
from drawers_math import arr,rotation,native,A,extent,contents
from drawers_physics import Physics as BasePhysics
from pad_region import contact_is_pad
class ModelPhysics(BasePhysics):
 def read(self,dt):
  poses=self.poses();geoms=self.geoms();self.contacts.read(dt);rows=self.contacts.last_contacts
  robots={}
  for path,(side,name) in self.robot_lookup.items():
   asset=self.base.scene[side+'_robot'];s=arr(asset.data.body_link_state_w)[0,asset.body_names.index(name)];robots[path]=(s[:3]-arr(self.base.scene.env_origins)[0],rotation(s[3:7]),s[7:10],s[10:13])
  touched=set();jar_force={'U_jar':0.,'L_jar':0.};jar_speed={'U_jar':0.,'L_jar':0.}
  origin=arr(self.base.scene.env_origins)[0]
  for c in rows:
   name=self.sensor_names[c['sensor_index']];other=self.filters[c['filter_index']];point=np.array(c['point_w'])-origin;normal=np.array(c['normal_w']);force=abs(c['normal_force_n'])
   if c['separation_m']>0 and force==0:continue
   if other in self.robot_lookup:touched.add(name)
   if name not in jar_force:continue
   # Whole distal pad volume excluded before BOTH force and speed sums.
   # touched remains complete, matching the separate native content-touch law.
   pad_poses={path:(self.robot_lookup[path][1],state[0],state[1]) for path,state in robots.items() if self.robot_lookup[path][1] in ('tip_left','tip_right')}
   if contact_is_pad(self.sensor_paths[c['sensor_index']],other,point,pad_poses):continue
   jar_force[name]+=force
   p,R,v,w=poses[name];va=v+np.cross(w,point-p)
   if other in robots:op,oR,ov,ow=robots[other]
   elif other in self.sensor_paths:op,oR,ov,ow=poses[self.sensor_names[self.sensor_paths.index(other)]]
   else:op,ov,ow=np.zeros(3),np.zeros(3),np.zeros(3)
   vb=ov+np.cross(ow,point-op)
   # PhysX normal convention must be calibrated in the foundation probe.
   jar_speed[name]=max(jar_speed[name],max(0.,float(np.dot(va-vb,normal))))
  p,R,*_=poses['drawer_U'];pn=native(p);Rn=A@R
  maxx=max(native(g['p'])[0]+extent(g['kind'],np.array(g['size']),A@g['R'])[0] for g in geoms if g['asset']=='drawer_U')
  jar=poses['U_jar'];jr=(native(jar[0]),A@jar[1]);box=native(poses['U_box'][0]);wrist=[]
  for side in ('left','right'):
   sensor=self.base.scene[side+'_drawers_wrench'];wrist.append(float(np.linalg.norm(arr(sensor.data.force)[0,sensor.body_names.index('gripper')])))
  drop=any(np.any(native(s[0])<np.array([-.75,-.85,.70])) or np.any(native(s[0])>np.array([.75,.85,1.60])) for n,s in poses.items() if n in self.sensor_names)
  r={'drawer_p':pn,'drawer_R':Rn,'max_x':maxx,'contents':contents((pn,Rn),box,jr,'U_jar' in touched),'drawer_touch':any(n.startswith('drawer_U') for n in touched),'wrist':wrist,'jar_force':list(jar_force.values()),'jar_speed':list(jar_speed.values()),'jar_cos':[poses[n][1][2,2] for n in jar_force],'drop':bool(drop),'contact_count':len(rows),'stop_on':dict(self.stop_on)}
  self.last=r;return r

def install():
 import drawers_physics
 drawers_physics.Physics=ModelPhysics
