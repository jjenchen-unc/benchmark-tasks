"""Actual state/contact sampling and source conditional collision mechanisms.
Credit channels intentionally EMPTY until root foundation validation. USD collision
toggles require an actual positive/negative stop response check in this backend.
"""
import math,json
import numpy as np
from drawers_math import arr,rotation,native,world,A,extent,contents,History
from contacts import Contacts
VALIDATED_CHANNELS=frozenset()
REQUIRED=frozenset(('geometry','clock','reset','contacts','jar_normal_speed','pad_identity','wrist','stop_masks','motor_total','finger_40N'))
class Physics:
 def __init__(self,emb,fixture,assets):
  self.emb=emb;self.base=emb._base;self.fixture=fixture;self.assets=assets;self.running=False;self.done=False;self.error=None;self.contacts=None;self.initialize()
 def initialize(self):
  self.history=History();self.controls=0;self.done=False;self.error=None;self.armed={'slot_U':True,'slot_L':True,'slot_E':True};self.stop_on={};self.last=None
 def reset_physical_joints(self):
  import torch
  for a in self.assets:
   if not a['articulation']:continue
   asset=self.base.scene[a['name']];q=asset.data.joint_pos.torch;zeros=torch.zeros_like(q);asset.write_joint_state_to_sim(zeros,zeros);asset.set_joint_effort_target(zeros)
  # Enable every stop at reset; settling re-evaluates measured releases.
  from pxr import UsdPhysics
  import omni.usd
  stage=omni.usd.get_context().get_stage()
  for prim in stage.Traverse():
   path=str(prim.GetPath())
   if '/carcasses/' in path and ('_stop' in path or '_hook' in path):UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(True)
 def begin(self):
  failure,error=self.history.failure,self.error
  self.initialize();self.history.failure=failure;self.error=error;self.running=True;self._make_contacts()
 def poses(self):
  origin=arr(self.base.scene.env_origins)[0];out={}
  for a in self.assets:
   if a['fixed']:
    for i in a['body_ids']:
     b=self.fixture['bodies'][i];out[b['name']]=(world(b['pos']),A@rotation(np.r_[b['quat'][1:],b['quat'][0]]),np.zeros(3),np.zeros(3))
   else:
    asset=self.base.scene[a['name']]
    if a['articulation']:
     rows=arr(asset.data.body_link_state_w)[0]
     for name,s in zip(asset.body_names,rows):out[name]=(s[:3]-origin,rotation(s[3:7]),s[7:10],s[10:13])
    else:
     s=arr(asset.data.root_link_state_w)[0];out[a['name']]=(s[:3]-origin,rotation(s[3:7]),s[7:10],s[10:13])
  return out
 def geoms(self):
  poses=self.poses();out=[]
  for a in self.assets:
   for i in a['body_ids']:
    b=self.fixture['bodies'][i];p,R,*_=poses[b['name']]
    for g in b['geoms']:
     if not(g['contype'] or g['conaffinity']):continue
     Q=R@rotation(np.r_[g['quat'][1:],g['quat'][0]]);c=p+R@np.array(g['pos']);out.append(dict(g,asset=a['name'],body=b['name'],p=c,R=Q,extent=extent(g['kind'],np.array(g['size']),Q)))
  return out
 def pre_step(self):
  # Only environment passive spring targets and native conditional collision
  # masks; no task pose/joint state, force, score or history writes here.
  import torch
  for a in self.assets:
   if a['articulation']:
    asset=self.base.scene[a['name']];refs={j['name']:j['springref'] for i in a['body_ids'] for j in self.fixture['bodies'][i]['joints'] if j['type']!=0}
    asset.set_joint_position_target(torch.tensor([refs[j] for j in asset.joint_names],device=asset.device,dtype=asset.data.joint_pos.torch.dtype).unsqueeze(0))
  poses=self.poses()
  from pxr import UsdPhysics
  import omni.usd
  stage=omni.usd.get_context().get_stage()
  for slot,kind,sy,face in [('slot_U','paddle',-.17,.12),('slot_L','pinch',.17,.12),('slot_E','paddle',-.51,.20)]:
   occupied=None
   for name,dkind in [('drawer_U','paddle'),('drawer_L','pinch')]:
    if dkind!=kind:continue
    w,R,*_=poses[name];p=native(w);R=A@R
    if abs(p[1]-sy)<.03 and abs(p[2]-.86)<.04 and p[0]+.14>face-.03:occupied=(name,p,R);break
   if occupied is None:continue
   name,p,R=occupied;asset=self.base.scene[name];q=dict(zip(asset.joint_names,arr(asset.data.joint_pos)[0]));release=all(abs(q[name+'_paddle_'+side+'_j'])>.004 for side in ('left','right')) if kind=='paddle' else all(q[name+'_lever_'+side+'_j']>.0035 for side in ('left','right'))
   bx=p[0]+R[0,0]*.13;a,b=face+.109,face+.117
   if self.armed[slot] and bx+.003<a-.001:self.armed[slot]=False
   elif not self.armed[slot] and bx-.003>b+.003:self.armed[slot]=True
   deep=bx+.003>a+.002 and bx-.003<b-.002;on=self.armed[slot] and not release and not deep
   self.stop_on[slot]=on
   hook_on=True
   if slot=='slot_L':
    bz=p[2]+R[2,0]*.13-R[2,2]*.016;lift=bz-(.86-.016);pitch=math.degrees(math.atan2(-R[2,0],R[0,0]));hdeep=bx+.003>face+.002 and bx-.003<face+.006;hook_on=not(lift>=.006 or pitch>=8) and not hdeep
   for prim in stage.Traverse():
    path=str(prim.GetPath())
    if '/carcasses/world/'+slot+'_stop' in path:UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(bool(on))
    if '/carcasses/world/'+slot+'_hook' in path:UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Set(bool(hook_on))
 def _make_contacts(self):
  from isaaclab_physx.physics import PhysxManager
  self.simview=PhysxManager.get_physics_sim_view()
  self.sensor_names=[];self.sensor_paths=[];self.path_states={}
  self.env=str(self.base.scene['left_robot'].root_view.link_paths[0][0]).split('/LeftArm/')[0]
  self.robot_paths=[];self.robot_lookup={}
  for side in ('left','right'):
   asset=self.base.scene[side+'_robot'];paths=list(asset.root_view.link_paths[0])
   self.robot_paths+=paths
   for p,n in zip(paths,asset.body_names):self.robot_lookup[p]=(side,n)
  for a in self.assets:
   if a['fixed']:continue
   asset=self.base.scene[a['name']]
   if a['articulation']:
    paths=list(asset.root_view.link_paths[0]);names=list(asset.body_names)
   else:paths=[self.env+'/'+a['name']+'/'+a['name']];names=[a['name']]
   self.sensor_paths+=paths;self.sensor_names+=names
  fixed=[self.env+'/carcasses/'+b['name']+'/'+g['name'].replace('-','_minus_') for b in self.fixture['bodies'] if b['name'] in ('world','cabinet_1','cabinet_2') for g in b['geoms'] if g['contype'] or g['conaffinity']]
  import omni.usd
  from static_binding import bind
  self.static_roles=bind(omni.usd.get_context().get_stage(),self.env,fixed)
  self.filters=self.robot_paths+self.sensor_paths+[r['filter_path'] for r in self.static_roles.values()]
  self.contacts=Contacts(self.simview,self.sensor_paths,self.filters)
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
   # No named pad surface in SDK URDF. Conservatively count every contact;
   # pad_identity channel remains explicitly blocked, so this cannot grant credit.
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
 def post_step(self,dt):
  if self.error:return
  try:self.history.update(self.read(dt),dt)
  except Exception as e:self.error=type(e).__name__+': '+str(e);self.history.failure='measurement_unavailable'
 def status(self):return dict(self.history.result(REQUIRED<=VALIDATED_CHANNELS and not self.error),controls=self.controls,measurement_error=self.error,missing_channels=sorted(REQUIRED-VALIDATED_CHANNELS))
