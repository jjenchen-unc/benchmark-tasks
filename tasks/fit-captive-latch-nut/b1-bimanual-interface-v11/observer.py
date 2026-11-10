"""Read-only actual state; missing raw8/contacts/wrench raises before Action."""
import json
import numpy as np
from scipy.spatial.transform import Rotation

def numeric(x):
 if isinstance(x,np.ndarray):return numeric(x.tolist())
 if isinstance(x,np.generic):return numeric(x.item())
 if isinstance(x,dict):
  if not all(isinstance(k,str) for k in x):raise TypeError('telemetry keys must be strings')
  return {k:numeric(v) for k,v in x.items()}
 if isinstance(x,(tuple,list)):return [numeric(v) for v in x]
 if x is None or isinstance(x,(str,bool,int,float)):return x
 raise TypeError('unsupported telemetry '+str(type(x)))
def emit(path,row):
 line=json.dumps(numeric(row),allow_nan=False,separators=(',',':'))
 with path.open('a') as f:f.write(line+'\n');f.flush()
 return json.loads(line)
_grasp_views={}
_fixture_views={}
def observe(E,case,phase,action=None):
 from contacts import arr
 from yam_arena.constants import TCP_OFFSET
 from yam_arena import ik,YAM_URDF
 from yam_arena.urdf import forward_kinematics
 row=dict(kind='ACTUAL_RUNTIME_OBSERVER',phase=phase,controls=E.controls,status=E.status(),physics_tick=int(E.base.sim.get_physics_step_count()),physics_dt=float(E.base.sim.get_physics_dt()),control_dt=float(E.base.step_dt),robots={},action=action)
 if E.error:raise RuntimeError(E.error)
 if abs(row['physics_dt']-.002)>1e-9 or abs(row['control_dt']-.05)>1e-9:raise RuntimeError('original clock contract changed')
 for side in ('left','right'):
  a=E.base.scene[side+'_robot'];q=arr(a.data.joint_pos)[0];names=a.joint_names
  if len(q)!=8 or set(names)!={f'joint{i}' for i in range(1,9)}:raise RuntimeError('actual raw8 unavailable')
  raw=np.array([q[names.index(f'joint{i}')] for i in range(1,9)])
  links=arr(a.data.body_link_state_w)[0];base=arr(a.data.root_pos_w)[0];g=links[a.body_names.index('gripper')];R=Rotation.from_quat(g[3:7]).as_matrix();tcp=g[:3]+R@TCP_OFFSET
  fp,fr=ik.tcp_pose(raw[:6],base_pos=base);pe=float(np.linalg.norm(fp-tcp));re=float(Rotation.from_matrix(fr@R.T).magnitude())
  if pe>.001 or re>.005:raise RuntimeError(f'actual FK mismatch {side}: {pe}/{re}')
  sensor=E.base.scene[side+('_b1_wrench' if case=='b1-wrench' else '_b2_wrench')]
  force=arr(sensor.data.force);torque=arr(sensor.data.torque)
  if force.shape!=(1,len(sensor.body_names),3) or torque.shape!=force.shape:raise RuntimeError('actual wrench unavailable')
  fk=forward_kinematics(YAM_URDF,{f'joint{i}':float(raw[i-1]) for i in range(1,9)})
  link_errors={}
  for name,v in zip(a.body_names,links):
   if name not in fk:continue
   transform=fk[name];actual_R=Rotation.from_quat(v[3:7]).as_matrix();lp=float(np.linalg.norm(base+transform[:3,3]-v[:3]));lr=float(Rotation.from_matrix(transform[:3,:3]@actual_R.T).magnitude())
   link_errors[name]=dict(position_m=lp,rotation_rad=lr)
   if lp>.001 or lr>.005:raise RuntimeError(f'actual raw8 FK mismatch {side}/{name}: {lp}/{lr}')
  row['robots'][side]=dict(raw8=raw,raw8_link_fk_errors=link_errors,unmatched_fk_links=[n for n in a.body_names if n not in fk],base=base,links=dict(zip(a.body_names,links)),tcp=tcp,tcp_rotation=R,wrench_force=force,wrench_torque=torque,wrench_convention=str(sensor.cfg.convention),wrench_body_names=sensor.body_names,fk_error_m=pe,fk_error_rad=re)
 if case=='b1-wrench':
  states=arr(E.asset.data.body_link_state_w)[0];head=states[E.bids['latch_head']]
  if id(E) not in _grasp_views:
   from contacts import Contacts
   from isaaclab_physx.physics import PhysxManager
   paths=sum([list(E.base.scene[s+'_robot'].root_view.link_paths[0]) for s in ('left','right')],[])
   _grasp_views[id(E)]=Contacts(PhysxManager.get_physics_sim_view(),paths,E.tools)
  view=_grasp_views[id(E)];row['robot_tool_contacts']=view.read(.002);row['robot_tool_sensors']=view.sensors;row['robot_tool_filters']=view.filters
  from isaaclab_physx.physics import PhysxManager
  if id(E) not in _fixture_views:
   _fixture_views[id(E)]=Contacts(PhysxManager.get_physics_sim_view(),E.tools,list(E.asset.root_view.link_paths[0]))
  fixture_view=_fixture_views[id(E)]
  row['tool_fixture_contacts']=fixture_view.read(.002,head[:3]);row['tool_fixture_sensors']=fixture_view.sensors;row['tool_fixture_filters']=fixture_view.filters
  row['fixture_link_states']=dict(zip(E.asset.body_names,states));row['actual_nut_head_state']=head
  row['fixture_joint_names']=E.asset.joint_names;row['fixture_raw_q']=arr(E.asset.data.joint_pos)[0]
  row['nut_contacts']=E.nut_contact.read(.002,head[:3]);row['hands_off_contacts']=E.hands_off.read(.002);row['nonfinger_contacts']=E.nonfinger.read(.002);row['last_substep']=E.last
  row['tools']={n:arr(E.base.scene[n].data.root_link_state_w)[0] for n in E.names}
 else:
  E.read_state();row['poses']=E.poses;row['native_joints']=E.q;row['mechanism']=E.state;row['task_contacts']=E.contacts.read(.002);row['robot_contacts']=E.robot_contacts.read(.002);row['last_substep']=E.history.last
 # Validate numeric serialization before callers can advance physics.
 return json.loads(json.dumps(numeric(row),allow_nan=False))
