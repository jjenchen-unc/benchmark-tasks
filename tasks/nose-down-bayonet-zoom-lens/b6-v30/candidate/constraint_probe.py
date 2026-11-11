"""EVALUATOR ONLY, read-only first-control constraint/backend instrumentation."""
import json,math
import numpy as np
from kit import arr
from mechanics import rotation

def residual(pa,Ra,pb,Rb,p0,R0,p1,R1):
 A=Ra@R0;B=Rb@R1
 return dict(translation_m=(pa+Ra@p0-pb-Rb@p1).tolist(),rotation_rad=math.acos(float(np.clip((np.trace(A.T@B)-1)/2,-1,1))))

def install(out):
 from b6_physics import Physics
 old_activate,old_pre,old_post=Physics.activate_current,Physics.pre_step,Physics.post_step
 count=[0];armed=[False]
 def snapshot(E,phase):
  row=dict(phase=phase,physics_step=int(E.base.sim.get_physics_step_count()),controls=E.controls,assets={},constraints=[])
  poses={};direct={}
  for n in ['body_right','lens_blue']:
   a=E.assets[n];v=a.root_view
   data=dict(backend_link_names=list(v.shared_metatype.link_names),body_names=list(a.body_names),joint_names=list(a.joint_names),cached_pos=arr(a.data.body_pos_w).tolist(),cached_quat_xyzw=arr(a.data.body_quat_w).tolist(),q=arr(a.data.joint_pos).tolist(),qd=arr(a.data.joint_vel).tolist(),backend={})
   for method in ['get_link_transforms','get_link_incoming_joint_force','get_masses','get_coms','get_inertias','get_dof_limits','get_dof_positions','get_dof_velocities']:
    try:data['backend'][method]=dict(value=arr(getattr(v,method)()).tolist())
    except Exception as exc:data['backend'][method]=dict(unavailable=type(exc).__name__+': '+str(exc))
   for i,b in enumerate(a.body_names):
    path=E.body_path(n,b);poses[path]=E.pose(n,b)
    if 'value' in data['backend']['get_link_transforms']:
     bi=data['backend_link_names'].index(b);t=np.array(data['backend']['get_link_transforms']['value'])[0,bi];direct[path]=(t[:3],rotation(t[3:7]))
   row['assets'][n]=data
  for j in list(E.constraints.values())+list(E.pin_constraints.values()):
   if not j.GetJointEnabledAttr().Get():continue
   a=str(j.GetBody0Rel().GetTargets()[0]);b=str(j.GetBody1Rel().GetTargets()[0])
   def frame(side):
    p=np.array(getattr(j,'GetLocalPos'+side+'Attr')().Get());q=getattr(j,'GetLocalRot'+side+'Attr')().Get();return p,rotation(np.r_[q.GetImaginary(),q.GetReal()])
   p0,R0=frame('0');p1,R1=frame('1')
   item=dict(path=str(j.GetPath()),body0=a,body1=b,local_pos0=p0.tolist(),local_pos1=p1.tolist(),local_R0=R0.tolist(),local_R1=R1.tolist())
   if a in poses and b in poses:item['cached_world_residual']=residual(*poses[a],*poses[b],p0,R0,p1,R1)
   if a in direct and b in direct:item['backend_world_residual_xyzw']=residual(*direct[a],*direct[b],p0,R0,p1,R1)
   row['constraints'].append(item)
  with (out/'constraint-first-control.jsonl').open('a') as f:f.write(json.dumps(row,allow_nan=False)+'\n')
 def activate(E,*args,**kw):
  armed[0]=True;snapshot(E,'before_activate:'+str(args[0].GetPath()));r=old_activate(E,*args,**kw);snapshot(E,'after_activate:'+str(args[0].GetPath()));return r
 def pre(E,dt):
  if armed[0] and count[0]<25:snapshot(E,'pre_substep_'+str(count[0]))
  return old_pre(E,dt)
 def post(E,dt):
  r=old_post(E,dt)
  if armed[0] and count[0]<25:snapshot(E,'post_substep_'+str(count[0]));count[0]+=1
  return r
 Physics.activate_current=activate;Physics.pre_step=pre;Physics.post_step=post
