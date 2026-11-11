"""Evaluator-only eight-step causal boundary; never writes task/robot state."""
import math
from pathlib import Path
import numpy as np
from kit import arr,rotation

def snapshot(E):
 out={'settling':E.settling,'history_samples':E.history.samples,'measurement_error':E.error,'assets':{}}
 for name in ('inst_A','inst_B','case_A','case_B','left_robot','right_robot'):
  a=E.base.scene[name];d=a.data;values={}
  for field in ('root_pos_w','root_quat_w','root_lin_vel_w','root_ang_vel_w','body_pos_w','body_quat_w','joint_pos','joint_vel'):
   try:values[field]=arr(getattr(d,field))
   except AttributeError:pass
  if name.startswith('inst_'):values['tilt_deg']=math.degrees(math.acos(float(np.clip(rotation(arr(d.root_quat_w)[0])[2,2],-1,1))))
  values['body_names']=getattr(a,'body_names',None);out['assets'][name]=values
 return out

def passive(E):
 out={}
 for c,a in E.parts.items():
  q,v=arr(a.data.joint_pos)[0],arr(a.data.joint_vel)[0];rows=[]
  for i,n in enumerate(a.joint_names):
   j=E.jointmeta[n];spring=-j['stiffness']*(q[i]-j['ref']);damp=-j['damping']*v[i];friction=-j['friction']*math.tanh(v[i]/.001)
   from kit import FIXTURE
   draw=-FIXTURE['springs'][n[:-6]]*max(0.,q[i]+.004) if n.endswith('_slide') else 0.
   rows.append(dict(name=n,position=q[i],velocity=v[i],spring=spring,damping=damp,friction=friction,draw_spring=draw,total=spring+damp+friction+draw))
  out[c]={'source_law_at_current_state':rows,'readback':{}}
  for field in ('joint_effort_target','applied_torque','computed_torque','joint_pos_limits'):
   try:out[c]['readback'][field]=arr(getattr(a.data,field))
   except AttributeError:out[c]['readback'][field]={'unavailable':True}
 return out

def constraint_rows(E,stage):
 from pxr import UsdPhysics
 poses={}
 for a in E.parts.values():
  for path,p,q in zip(a.root_view.link_paths[0],arr(a.data.body_pos_w)[0],arr(a.data.body_quat_w)[0]):poses[str(path)]=(p,rotation(q))
 out=[]
 for prim in stage.Traverse():
  path=str(prim.GetPath())
  if '/case_' not in path or not prim.IsA(UsdPhysics.Joint):continue
  j=UsdPhysics.Joint(prim);enabled=j.GetJointEnabledAttr().Get();targets=[j.GetBody0Rel().GetTargets(),j.GetBody1Rel().GetTargets()]
  points=[];frames=[]
  for side,paths in enumerate(targets):
   pos=j.GetLocalPos0Attr().Get() if side==0 else j.GetLocalPos1Attr().Get();pos=np.array(pos if pos is not None else [0.,0.,0.])
   quat=j.GetLocalRot0Attr().Get() if side==0 else j.GetLocalRot1Attr().Get()
   q=[*quat.GetImaginary(),quat.GetReal()] if quat is not None else [0.,0.,0.,1.]
   p,R=poses[str(paths[0])] if paths else (np.zeros(3),np.eye(3))
   points.append(p+R@pos);frames.append(R@rotation(q))
  axis=prim.GetAttribute('physics:axis').Get();axis_index={'X':0,'Y':1,'Z':2}.get(axis,0)
  delta=points[1]-points[0]
  out.append(dict(path=path,type=prim.GetTypeName(),enabled=bool(enabled if enabled is not None else True),bodies=[[str(t) for t in x] for x in targets],world_points=points,world_frames=frames,displacement_m=delta,distance_m=float(np.linalg.norm(delta)),axis=axis,axial_displacement_m=float(delta@frames[0][:,axis_index]),lower_limit=prim.GetAttribute('physics:lowerLimit').Get(),upper_limit=prim.GetAttribute('physics:upperLimit').Get()))
 return out

def install(out):
 from physics import Physics
 from first_cause import strict_json
 old_init,old_post=Physics.initialize,Physics.post_step
 contacts=None;stage=None;count=0
 stream=(Path(out)/'reset-causal-boundary.jsonl').open('x',buffering=1)
 def record(E,phase,dt=None):
  row=snapshot(E);row.update(phase=phase,callback=count,dt=dt,passive=passive(E),constraints=constraint_rows(E,stage))
  row['robot_wrist']={}
  for side in ('left','right'):
   s=E.base.scene[side+'_s2_wrench'];row['robot_wrist'][side]={'body_names':s.body_names,'force':arr(s.data.force)}
  if dt is not None:
   contacts.read(dt);row['contacts']={'selection':contacts.selection,'records':contacts.last_contacts}
  stream.write(strict_json(row).replace('\n','')+'\n');stream.flush()
 def initialize(E):
  nonlocal contacts,stage
  result=old_init(E)
  from isaaclab_physx.physics import PhysxManager
  from isaaclab.sim.utils import get_current_stage
  from pxr import UsdPhysics
  from contacts import Contacts
  stage=get_current_stage();paths=set()
  for prim in stage.Traverse():
   if not prim.HasAPI(UsdPhysics.CollisionAPI):continue
   owner=prim
   while owner and not owner.HasAPI(UsdPhysics.RigidBodyAPI):owner=owner.GetParent()
   paths.add(str(owner.GetPath()) if owner else str(prim.GetPath()))
  sensors=[str(E.base.scene['inst_'+c].root_view.prim_paths[0]) for c in ('A','B')]+[str(p) for c in ('A','B') for p in E.parts[c].root_view.link_paths[0]]
  contacts=Contacts(PhysxManager.get_physics_sim_view(),sensors,sorted(paths))
  assert contacts.view.sensor_count==len(sensors) and contacts.view.filter_count==len(paths),'contact index ownership mismatch'
  record(E,'initialized_before_settle');return result
 def post(E,dt):
  nonlocal count
  result=old_post(E,dt);count+=1;record(E,'settle' if E.settling else 'first_control',float(dt))
  if count==8:
   raise RuntimeError('EVALUATOR_ONLY eight causal boundary callbacks captured; no controller executed')
  return result
 Physics.initialize,Physics.post_step=initialize,post
