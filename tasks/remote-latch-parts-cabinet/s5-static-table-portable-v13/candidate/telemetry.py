"""Read-only strict numeric journals. No history/state/force writes."""
import json,math
import numpy as np

def numeric(x):
 if x is None or isinstance(x,(str,bool,int)):return x
 if isinstance(x,float):
  if not math.isfinite(x):raise ValueError('nonfinite telemetry')
  return x
 if isinstance(x,np.ndarray):return numeric(x.tolist())
 if isinstance(x,np.generic):return numeric(x.item())
 if isinstance(x,(list,tuple)):return [numeric(v) for v in x]
 if isinstance(x,dict):
  if not all(isinstance(k,str) for k in x):raise TypeError('telemetry keys must be strings')
  return {k:numeric(v) for k,v in x.items()}
 raise TypeError('unsupported telemetry '+str(type(x)))
def dumps(x):return json.dumps(numeric(x),allow_nan=False)
def snapshot(emb,obs):
 from s5_kit import ENGINE,arr,FIXTURE
 from yam_arena import ik
 from yam_arena.constants import TCP_OFFSET
 from s5_contacts import rot_xyzw
 robots={}
 for side in ['left','right']:
  obj=emb._base.scene[side+'_robot'];q=arr(obj.data.joint_pos)[0]
  if q.shape!=(8,) or not np.isfinite(q).all():raise ValueError('invalid raw8 '+side)
  arm=np.array([q[obj.joint_names.index('joint'+str(i))] for i in range(1,7)])
  p,R=ik.tcp_pose(arm,FIXTURE['robot_bases'][side])
  links=arr(obj.data.body_link_state_w);gripper=links[0,obj.body_names.index('gripper')];measured_tcp=gripper[:3]+rot_xyzw(gripper[3:7])@np.asarray(TCP_OFFSET)
  robots[side]=dict(joint_names=obj.joint_names,raw8=q,body_names=obj.body_names,body_link_state_w=arr(obj.data.body_link_state_w),tcp_fk_position=p,tcp_fk_rotation=R,tcp_measured_position=measured_tcp,tcp_fk_measured_error_m=float(np.linalg.norm(measured_tcp-p)),raw_fingers=[q[obj.joint_names.index(n)] for n in ['joint7','joint8']])
 # Separate read-only cabinet/robot contact channel (old trace saw box only).
 if not hasattr(ENGINE,'_paddle_telemetry_contacts'):
  from s5_contacts import Contacts,body_paths
  sensors=body_paths(emb._base.scene['cab_A']);filters=body_paths(emb._base.scene['right_robot'])
  ENGINE._paddle_telemetry_contacts=Contacts(ENGINE.simview,sensors,filters)
  ENGINE._paddle_telemetry_selection=dict(sensors=sensors,filters=filters)
 contact=ENGINE._paddle_telemetry_contacts;hit,peak,count=contact.read(.002)
 q,v=ENGINE.joint_state()
 mechanism=dict(joint_q=q,joint_v=v,events=ENGINE.latches.events,cabinet_robot_contacts=dict(selection=ENGINE._paddle_telemetry_selection,hit=hit,peak_normal_force_n=peak,count=count,records=contact.last_contacts),paddle_spring_target_m=-.004,A_paddle2_drive_stiffness_n_m=500.,paddle_drive_damping_ns_m=3.)
 state=np.asarray(obs.state['joint_pos'])
 if state.shape!=(14,) or not np.isfinite(state).all():raise ValueError('invalid public14 state')
 return numeric(dict(mechanism=mechanism,status=ENGINE.status(),tick=ENGINE.hist.samples,observer=ENGINE.row(.002),public14=state,robots=robots))
def preflight(emb,obs,out):
 # Real runtime NumPy and observer metadata are serialized BEFORE first Action.
 s=snapshot(emb,obs);encoded=dumps(s)
 if json.loads(encoded)!=s:raise ValueError('numeric telemetry roundtrip mismatch')
 (out/'first-action-preflight.json').write_text(encoded+'\n')
 return s
