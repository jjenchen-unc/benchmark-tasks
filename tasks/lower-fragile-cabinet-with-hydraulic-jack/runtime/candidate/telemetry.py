"""Strict finite JSON; privileged observer records, never task-state mutations."""
import json
import numpy as np

def native(v):
 if isinstance(v,np.ndarray):return native(v.tolist())
 if isinstance(v,np.generic):return native(v.item())
 if v is None or isinstance(v,(str,bool,int,float)):return v
 if isinstance(v,(list,tuple)):return [native(x) for x in v]
 if isinstance(v,dict):
  if not all(isinstance(k,str) for k in v):raise TypeError('non-string key')
  return {k:native(x) for k,x in v.items()}
 raise TypeError('unsupported journal type '+type(v).__name__)
def dumps(v):return json.dumps(native(v),allow_nan=False)
def snapshot(emb,obs,phase,action=None):
 import kit
 from contacts import arr
 from yam_arena.runtime import tcp_world
 row=dict(phase=phase,status=kit.current_status(),observer=dict(state_time=obs.state_time,joint_pos=obs.state['joint_pos']),tick=kit.ENGINE.controls,robots={},contacts={})
 if action is not None:row['action']=action
 for side in ('left','right'):
  a=emb._base.scene[side+'_robot'];d=a.data
  w=emb._base.scene[side+'_hj_wrench']
  row['robots'][side]=dict(wrench_body_names=w.body_names,wrench_force=arr(w.data.force),wrench_torque=arr(w.data.torque),joint_names=a.joint_names,raw8=arr(d.joint_pos),raw8_vel=arr(d.joint_vel),link_names=a.body_names,link_pos_w=arr(d.body_pos_w),link_quat_w=arr(d.body_quat_w),tcp_w=tcp_world(a),applied_torque=arr(d.applied_torque))
 engine=kit.ENGINE
 for name,c in [('robot',engine.robot),('rails',engine.rails),('all',engine.all_contacts),*[(f'pad_{k}',v) for k,v in engine.pad.items()]]:row['contacts'][name]=dict(selection=c.selection,raw=getattr(c,'last_contacts',[]))
 row['jack_joints']={k:dict(names=a.joint_names,q=arr(a.data.joint_pos),v=arr(a.data.joint_vel)) for k,a in engine.jacks.items()}
 return native(row)
def before_action(emb,obs,action,stream):
 # Serialization completed BEFORE constructing/sending Action.
 payload=dumps(snapshot(emb,obs,'before_action',action));stream.write(payload+'\n')
 emb._hj_observer_action=native(action)


def substep_detail(engine):
 from contacts import arr
 # Read-only privileged diagnostic, only first four controls and first failure.
 row=dict(robots={},contacts={},action=getattr(engine.emb,'_hj_observer_action',None))
 for side in ('left','right'):
  a=engine.base.scene[side+'_robot'];d=a.data;w=engine.base.scene[side+'_hj_wrench']
  row['robots'][side]=dict(joint_names=a.joint_names,raw8=arr(d.joint_pos),raw8_vel=arr(d.joint_vel),link_names=a.body_names,link_pos_w=arr(d.body_pos_w),link_quat_w=arr(d.body_quat_w),wrench_body_names=w.body_names,wrench_force=arr(w.data.force),wrench_torque=arr(w.data.torque),applied_torque=arr(d.applied_torque))
 for name,c in [('robot',engine.robot),('rails',engine.rails),('all',engine.all_contacts),*[(f'pad_{k}',v) for k,v in engine.pad.items()]]:
  row['contacts'][name]=dict(selection=c.selection,raw=c.last_contacts)
 row['jack_joints']={k:dict(names=a.joint_names,q=arr(a.data.joint_pos),v=arr(a.data.joint_vel)) for k,a in engine.jacks.items()}
 return native(row)
