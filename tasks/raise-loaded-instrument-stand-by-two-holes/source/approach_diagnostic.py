"""Read-only capture for first offending approach; no scene/controller writes."""
def capture(emb):
 from kit import arr,ENGINE
 row={'robots':{},'status':ENGINE.status(),'task_contacts':ENGINE.contacts.last_contacts}
 for side in ('left','right'):
  robot=emb._base.scene[side+'_robot'];sensor=emb._base.scene[side+'_pins_wrench']
  row['robots'][side]={'joint_names':list(robot.joint_names),'raw8':arr(robot.data.joint_pos)[0],
   'joint_vel':arr(robot.data.joint_vel)[0],'body_names':list(robot.body_names),
   'body_positions':arr(robot.data.body_pos_w)[0],'body_quaternions':arr(robot.data.body_quat_w)[0],
   'link_paths':robot.root_view.link_paths,'wrench_body_names':list(sensor.body_names),
   'wrist_forces':arr(sensor.data.force)[0]}
 asset=emb._base.scene['pins']
 row['task']={'body_names':list(asset.body_names),'body_positions':arr(asset.data.body_pos_w)[0],'body_quaternions':arr(asset.data.body_quat_w)[0],'joint_names':list(asset.joint_names),'joint_pos':arr(asset.data.joint_pos)[0],'joint_vel':arr(asset.data.joint_vel)[0]}
 from numeric_json import plain
 return plain(row) # detach all arrays immediately; no live simulator views


def topology():
 import omni.usd
 stage=omni.usd.get_context().get_stage()
 return [{'path':str(p.GetPath()),'type':p.GetTypeName(),
  'attributes':{a.GetName():str(a.Get()) for a in p.GetAttributes() if a.GetName().startswith(('physics:','physxCollision:','physxRigidBody:'))},
  'relationships':{r.GetName():[str(x) for x in r.GetTargets()] for r in p.GetRelationships() if r.GetName().startswith('physics:')}}
  for p in stage.Traverse() if any('CollisionAPI' in s or 'RigidBodyAPI' in s for s in p.GetAppliedSchemas())]
