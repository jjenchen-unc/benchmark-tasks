"""Read-only per-physics-sample reset/onset instrumentation; no simulator startup."""
import time
from strict_json import dumps
from drawers_math import arr

def install(out):
 from drawers_physics import Physics
 phase={'name':'reset','index':0}
 original_post,original_begin,original_pre=Physics.post_step,Physics.begin,Physics.pre_step
 stream=(out/'reset-physics.jsonl').open('x',buffering=1)
 def snapshot(engine,event,dt=None):
  robots={}
  for side in ('left','right'):
   robot=engine.base.scene[side+'_robot'];sensor=engine.base.scene[side+'_drawers_wrench']
   robots[side]={'raw8':arr(robot.data.joint_pos)[0],
    'joint_names':robot.joint_names,
    'joint_velocity':arr(robot.data.joint_vel)[0],
    'default_position':arr(robot.data.default_joint_pos)[0],
    'target_position':arr(robot.actuators.target_command.position)[0],
    'target_velocity':arr(robot.actuators.target_command.velocity)[0],
    'target_effort':arr(robot.actuators.target_command.effort)[0],
    'body_names':robot.body_names,'all_links':arr(robot.data.body_link_state_w)[0],
    'base':arr(robot.data.root_link_pose_w)[0],
    'wrench_body_names':sensor.body_names,'wrench_force':arr(sensor.data.force)}
  stream.write(dumps({'event':event,'phase':phase['name'],'index':phase['index'],
   'monotonic_s':time.monotonic(),'dt_s':dt,
   'ready14':arr(engine.emb._ready)[0],
   'sim_current_time_s':getattr(getattr(engine.base,'sim',None),'current_time',None),
   'history_time_s':engine.history.time,'history_samples':engine.history.samples,
   'wrist_streak_s':engine.history.wrist,'failure':engine.history.failure,
   'physics':engine.last,'robots':robots,
   'contacts':getattr(engine.contacts,'last_contacts',[]),
   'sensor_names':engine.sensor_names,'sensor_paths':engine.sensor_paths,
   'filter_paths':engine.filters})+'\n')
 def pre(engine):
  original_pre(engine)
  if engine.running:snapshot(engine,'before_scene_write')
 def post(engine,dt):
  original_post(engine,dt)
  phase['index']+=1
  snapshot(engine,'post_physics',dt)
 def begin(engine):
  snapshot(engine,'before_begin')
  original_begin(engine)
  phase['name']='action'
  snapshot(engine,'after_begin')
 Physics.post_step,Physics.begin,Physics.pre_step=post,begin,pre
 return stream
