"""Read-only retained1ms state around the failed phase. No simulator writes."""
import json
from c1_kit import arr
from c1_contract import rotation_xyzw

def record(engine,dt,forces,links,sums):
 from yam_arena.constants import TCP_OFFSET
 import numpy as np
 if not hasattr(engine,'diagnostic_trace'):
  engine.diagnostic_trace=open(__import__('pathlib').Path(__import__('os').environ['C1_TRACE']).with_name('guard-1ms.jsonl'),'w',buffering=1)
 robots=[]
 for side in ['left','right']:
  robot=engine.base.scene[side+'_robot'];sensor=engine.base.scene[side+'_c1_wrench'];j=robot.body_names.index('gripper');p=arr(robot.data.body_pos_w)[0];q=arr(robot.data.body_quat_w)[0];origin=arr(engine.base.scene.env_origins)[0]
  robots.append(dict(side=side,body_names=list(robot.body_names),body_pos_w=p.tolist(),body_quat_w=q.tolist(),joint_names=list(robot.joint_names),raw_joint_pos=arr(robot.data.joint_pos)[0].tolist(),raw_joint_vel=arr(robot.data.joint_vel)[0].tolist(),tcp_world=(p[j]+rotation_xyzw(q[j])@np.asarray(TCP_OFFSET)).tolist(),env_origin=origin.tolist(),wrench_body_names=list(sensor.body_names),wrench_force=arr(sensor.data.force)[0].tolist()))
 row=dict(sample=engine.samples,action=(engine.samples-1)//50+1,substep=(engine.samples-1)%50+1,sim_s=engine.history.seconds,dt=dt,wrist_norm_n=forces,guard_streak_s=engine.guard_streak.tolist(),guard_frozen=engine.freeze_arms,link_instant_upper_n=links,link20ms_by_body_n=sums,link20ms_max_n=engine.link_max,contacts=engine.contacts.diagnostic_contacts,robots=robots)
 engine.diagnostic_trace.write(json.dumps(row,allow_nan=False)+'\n')
