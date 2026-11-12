"""Environment passive rules and read-only history. Never called by a policy.
Capture joints preserve their measured relative frame (no pose teleport).
"""
from pathlib import Path
import json,os,math
import numpy as np
from mechanics import History,rotation,seat_measure,detent,helix
from contacts import Contacts,body_paths
from kit import arr
LENSES=('lens_orange','lens_blue','lens_purple')
FAMILY={'lens_orange':'L','lens_blue':'R','lens_purple':'R'}
class Physics:
 def __init__(self,emb,fixture,specs):
  self.emb=emb;self.base=emb._base;self.f=fixture;self.specs=specs;self.history=History();self.settling=True;self.done=False;self.controls=0;self.error=None;self.last=None
  self.assets={n:self.base.scene[n] for n in specs};self.trace=None;self.force_trace=None;self.cap={};self.pin={};self.click={};self.pull={};self.lift_time={};self.cooldown={};self.constraints={};self.pin_constraints={}
  from isaaclab_physx.physics import PhysxManager
  self.simview=PhysxManager.get_physics_sim_view();robots=body_paths(self.base.scene['left_robot'])+body_paths(self.base.scene['right_robot']);self.robots=robots
  self.paths={n:body_paths(self.assets[n]) for n in ['body_left','body_right',*LENSES]};self.envroot=self.paths['lens_blue'][0].split('/lens_blue/')[0]
  self.robot_contacts={n:Contacts(self.simview,self.paths[n],robots) for n in ['body_left','body_right',*LENSES]}
  self.bracket_contacts=Contacts(self.simview,robots,[self.envroot+'/body_right_bracket_arm',self.envroot+'/body_right_bracket_post'])
  # Seat-face vs rear feet normal loads only, not general shell contacts.
  self.seat_contacts={(m,o):Contacts(self.simview,[self.body_path(o,o)],[self.body_path(m,m)]) for m in ['body_left','body_right'] for o in LENSES if FAMILY[o]==('L' if m=='body_left' else 'R')}
  import omni.usd
  self.stage=omni.usd.get_context().get_stage();self.native_pin_limits={};self.pin_angles={};self.make_constraints();self.prepare_reset()
 def body_path(self,asset,body):return next(p for p in self.paths[asset] if p.split('/')[-1]==body)
 def pose(self,asset,body):
  a=self.assets[asset];i=a.body_names.index(body);return arr(a.data.body_pos_w)[0,i],rotation(arr(a.data.body_quat_w)[0,i])
 def joint(self,asset,name):
  a=self.assets[asset];i=a.joint_names.index(name);return float(arr(a.data.joint_pos)[0,i]),float(arr(a.data.joint_vel)[0,i])
 def make_constraints(self):
  from pxr import UsdPhysics,Sdf,Gf
  for m in ['body_left','body_right']:
   for o in LENSES:
    if (m,o) not in self.seat_contacts:continue
    j=UsdPhysics.FixedJoint.Define(self.stage,self.envroot+'/b6_capture_'+m+'_'+o)
    j.CreateBody0Rel().SetTargets([Sdf.Path(self.body_path(m,m+'_ring'))]);j.CreateBody1Rel().SetTargets([Sdf.Path(self.body_path(o,o))]);j.CreateJointEnabledAttr(False);j.CreateExcludeFromArticulationAttr(True);j.CreateCollisionEnabledAttr(True);self.constraints[m,o]=j
 def activate_current(self,j,a,ab,b,bb):
  # Paired anchor at body b's current origin; target relative orientation current.
  # This stores physical contact capture, never overwrites a body or joint state.
  from pxr import Gf
  from yam_arena.ik import matrix_to_quat_xyzw
  pa,Ra=self.pose(a,ab);pb,Rb=self.pose(b,bb);q=matrix_to_quat_xyzw(Ra.T@Rb)
  j.CreateLocalPos0Attr(Gf.Vec3f(*map(float,Ra.T@(pb-pa))));j.CreateLocalPos1Attr(Gf.Vec3f(0,0,0));j.CreateLocalRot0Attr(Gf.Quatf(float(q[3]),Gf.Vec3f(*map(float,q[:3]))));j.CreateLocalRot1Attr(Gf.Quatf(1,Gf.Vec3f(0,0,0)));j.GetJointEnabledAttr().Set(True)
 def set_pin(self,m,engaged):
  from unique_pin import write_limits
  a=self.assets[m];i=a.joint_names.index(m+'_turn')
  if m not in self.native_pin_limits:self.native_pin_limits[m]=arr(a.root_view.get_dof_limits())
  limits=self.native_pin_limits[m].copy()
  if engaged:
   theta=self.joint(m,m+'_turn')[0]
   if not limits[0,i,0]<=theta<=limits[0,i,1]:raise RuntimeError('Pin outside native hinge limits')
   limits[0,i,:]=theta;self.pin_angles[m]=theta
  write_limits(a,limits)
  self.pin[m]=bool(engaged)
  if not engaged:self.pin_angles.pop(m,None)
 def prepare_reset(self):
  for m in self.native_pin_limits:self.set_pin(m,False)
  for j in list(self.constraints.values())+list(self.pin_constraints.values()):j.GetJointEnabledAttr().Set(False)
  self.cap={m:None for m in ['body_left','body_right']};self.pin={m:False for m in self.cap};self.click={m:False for m in self.cap};self.pull={m:0. for m in self.cap};self.lift_time={m:0. for m in self.cap};self.cooldown={};self.capture_targets={};self.error=None
 def restore_joints(self):
  import torch
  for n,a in self.assets.items():
   if hasattr(a,'joint_names'):
    z=torch.zeros_like(a.data.joint_pos.torch);a.set_joint_effort_target(z);a.set_joint_position_target(z);a.write_joint_state_to_sim(z,z)
 def clock(self):
  return float(self.base.sim.get_physics_step_count())*float(self.base.sim.get_physics_dt())
 def begin_episode(self):
  self.history=History();self.controls=0;self.done=False;self.settling=False;self.error=None;self.started_physics_step=int(self.base.sim.get_physics_step_count());self.started_sim_time=self.clock()
  if self.trace:self.trace.close()
  self.trace=open(os.environ['B6_TRACE'],'a',buffering=1) if os.environ.get('B6_TRACE') else None
  if self.force_trace:self.force_trace.close()
  self.force_trace=open(Path(os.environ['B6_TRACE']).with_name('force-vectors.jsonl'),'a',buffering=1) if os.environ.get('B6_TRACE') else None
 def pre_step(self,dt):
  # Passive spring/detent/helix effort only. Numeric joint state is never written here.
  import torch
  for n,a in self.assets.items():
   if not hasattr(a,'joint_names'):continue
   q=arr(a.data.joint_pos)[0];v=arr(a.data.joint_vel)[0];eff=np.zeros(len(a.joint_names))
   if n.startswith('lens_'):
    iz=a.joint_names.index(n+'_zoom');ix=a.joint_names.index(n+'_ext');F,T=helix(q[iz],q[ix],v[iz],v[ix]);eff[ix]=F;eff[iz]=T
   elif n in self.cap:
    fam='L' if n=='body_left' else 'R';d=1 if self.f['parameters']['lock_dir_'+fam]=='ccw' else -1;i=a.joint_names.index(n+'_turn');eff[i]=detent(q[i],d,self.f['parameters']['lock_angle_'+fam],self.f['parameters']['click_torque_'+fam],self.click[n])
   a.set_joint_effort_target(torch.tensor([eff],dtype=a.data.joint_pos.torch.dtype,device=self.base.device))
 def read_contacts(self,dt):
  held={};wrenches={}
  for n,c in self.robot_contacts.items():
   held[n],_,_=c.read(dt)
   # Preserve the normal contact cache for held/touch and force auditing.
   # Lens pull-out uses the full source-native contact vector below.
   F=np.zeros(3)
   for p in c.last_contacts:F+=p['normal_force_n']*np.array(p['normal_w'])
   # Source-native robot_force includes normal AND tangential robot contact
   # on every descendant lens body. Held/touch and seat-pressure stay exact.
   if n in LENSES:F=c.full_force(dt)
   wrenches[n]=F
  held['overhead_bracket'],_,_=self.bracket_contacts.read(dt)
  push={}
  for pair,c in self.seat_contacts.items():
   c.read(dt);push[pair]=sum(abs(p['normal_force_n']) for p in c.last_contacts)
  return held,wrenches,push
 def rules(self,dt,held,wrenches,push):
  p=self.f['parameters']
  for m in self.cap:
   fam='L' if m=='body_left' else 'R';d=1 if p['lock_dir_'+fam]=='ccw' else -1;lk=p['lock_angle_'+fam];th,_=self.joint(m,m+'_turn');deg=math.degrees(th)*d;mp,mR=self.pose(m,m)
   if self.cap[m] is None and abs(math.degrees(th))<3:
    for o in LENSES:
     if (m,o) not in self.constraints or o in self.cap.values():continue
     lp,lR=self.pose(o,o);meas=seat_measure(mp,mR,lp,lR)
     if self.cooldown.get((m,o)):
      if abs(meas['axial_gap_m'])>.003 or meas['radial_m']>.01:self.cooldown[m,o]=False
      continue
     yaw=(meas['yaw_deg']-math.degrees(th)+180)%360-180
     if meas['radial_m']<.0025 and abs(meas['axial_gap_m'])<.0015 and meas['tilt_deg']<4 and abs(yaw)<6 and push[m,o]>2:
      self.activate_current(self.constraints[m,o],m,m+'_ring',o,o);self.cap[m]=o;self.capture_targets[m]=dict(seat_z=meas['axial_gap_m']+.0003,seat_yaw=math.radians(yaw));break
   if self.cap[m] and m in self.capture_targets:
    # Native passive centring: constraint anchor moves <=0.5mm/s, <=2deg/s.
    # Joint-relative frame updates induce only solver reaction; never body poses.
    from pxr import Gf
    from yam_arena import ik
    j=self.constraints[m,self.cap[m]];target=self.capture_targets[m]
    cur=np.array(j.GetLocalPos0Attr().Get());goal=np.array([0.,0.,target['seat_z']]);new=cur+np.clip(goal-cur,-.0005*dt,.0005*dt);j.GetLocalPos0Attr().Set(Gf.Vec3f(*new))
    q=j.GetLocalRot0Attr().Get();qr=np.r_[q.GetImaginary(),q.GetReal()];C=rotation(qr);a=target['seat_yaw'];T=np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1.]])
    angle=np.linalg.norm(ik._rot_error(T,C));s=min(1.,math.radians(2)*dt/max(angle,1e-12));qnew=ik.matrix_to_quat_xyzw(ik.slerp_matrix(C,T,s));j.GetLocalRot0Attr().Set(Gf.Quatf(float(qnew[3]),Gf.Vec3f(*map(float,qnew[:3]))))
   if not self.click[m] and deg>=lk:self.click[m]=True
   btn,_=self.joint(m,m+'_button_slide');pressed=btn>.0025
   if self.click[m] and not self.pin[m] and deg>=lk+.8 and not pressed:
    self.set_pin(m,True)
   if pressed and self.pin[m]:self.set_pin(m,False)
   if self.click[m] and not self.pin[m] and deg<lk-1:self.click[m]=False
   o=self.cap[m]
   if o and not self.pin[m] and abs(math.degrees(th))<3:
    mass=sum(b['mass'] for b in self.f['bodies'] if b['name'].startswith(o));pull=float(mR[:,2]@(wrenches[o]+np.array([0,0,-9.81*mass])))
    if push[m,o]>1:pull=0.
    alpha=1-(1-.2)**(dt/.002);self.pull[m]=(1-alpha)*self.pull[m]+alpha*pull;self.lift_time[m]=self.lift_time[m]+dt if self.pull[m]>2 else 0.
    if self.lift_time[m]>=.024-1e-9:
     self.constraints[m,o].GetJointEnabledAttr().Set(False);self.cap[m]=None;self.cooldown[m,o]=True;self.pull[m]=self.lift_time[m]=0.
   else:self.pull[m]=self.lift_time[m]=0.
 def measurement(self):
  out=dict(lens_on_mount=None,captured=None,pin_engaged=None,axial_gap_m=None,radial_m=None,tilt_deg=None,family_ok=None,lock_angle_err_deg=None,focal_mm=None)
  mp,mR=self.pose('body_right','body_right');found=[]
  for o in LENSES:
   lp,lR=self.pose(o,o);a=seat_measure(mp,mR,lp,lR)
   if abs(a['axial_gap_m'])<=.001 and a['radial_m']<=.0025 and a['tilt_deg']<=4:found.append((o,a))
  if len(found)>1:raise ValueError('multiple seated lenses')
  if found:
   o,a=found[0];th,_=self.joint('body_right','body_right_turn');z,_=self.joint(o,o+'_zoom');d=1 if self.f['parameters']['lock_dir_R']=='ccw' else -1
   out.update({k:a[k] for k in ['axial_gap_m','radial_m','tilt_deg']});out.update(lens_on_mount=o,family_ok=FAMILY[o]=='R',captured=self.cap['body_right']==o,pin_engaged=self.pin['body_right'] and self.cap['body_right']==o,lock_angle_err_deg=abs(d*math.degrees(th)-self.f['parameters']['lock_angle_R']),focal_mm=18+37*z/(math.pi/2))
  return out
 def post_step(self,dt):
  if self.error:return
  try:
   held,F,push=self.read_contacts(dt);self.rules(dt,held,F,push)
   wrists=[]
   for s in ['left','right']:
    sensor=self.base.scene[s+'_b6_wrench'];wrists.append(float(np.linalg.norm(arr(sensor.data.force)[0,sensor.body_names.index('gripper')])))
   torque={}
   for m in self.cap:
    a=self.assets[m];i=a.body_names.index(m+'_ring');raw=arr(a.root_view.get_link_incoming_joint_force())[0,i]
    # Actual43856 signed optical/transverse loads identify joint-frame X.
    from reaction_frame import optical_torsion
    torque[m]=optical_torsion(raw)
   vel={o:float(arr(self.assets[o].data.root_lin_vel_w)[0,2]) for o in LENSES}
   row=dict(wrist_n=wrists,stop_nm=torque,drop_velocity_z=vel,held={o:held[o] for o in LENSES},captured={o:o in self.cap.values() for o in LENSES},robot_touch=bool(any(held.values())),measurement=self.measurement(),push_n={m+'__'+o:f for (m,o),f in push.items()},mounts={m:dict(lens=self.cap[m],pin=self.pin[m],theta=self.joint(m,m+'_turn')[0]) for m in self.cap})
   self.last=row
   if not self.settling:
    self.history.update(row,dt)
    if self.trace:self.trace.write(json.dumps(dict(sim_time=self.clock(),control=self.controls,row=row,status=self.status()))+'\n')
    if self.force_trace:self.force_trace.write(json.dumps(dict(sim_time=self.clock(),control=self.controls,dt=dt,objects={o:self.robot_contacts[o].last_force_packet for o in LENSES},input_wrenches_used_by_rules_N={o:self.robot_contacts[o].last_force_packet['total_world_N'] for o in LENSES},captured_after_rules=dict(self.cap),pull_after_rules_N=dict(self.pull),lift_time_after_rules_s=dict(self.lift_time)),allow_nan=False)+'\n')
  except Exception as e:
   self.error=type(e).__name__+': '+str(e);self.history.invalid=True;self.history.failure='measurement_unavailable'
 def status(self):return dict(self.history.result(),measurement_error=self.error,controls=self.controls,backend='CPU PhysX/GPU RTX')
 def termination_reason(self):
  if self.error:return 'measurement_unavailable'
  if self.history.failure:return self.history.failure
  if self.controls>=3034:return 'time_limit'
  if self.status()['score']==1.:return 'success'
  return None
 def close(self):
  if self.trace:self.trace.close();self.trace=None
  if self.force_trace:self.force_trace.close();self.force_trace=None
