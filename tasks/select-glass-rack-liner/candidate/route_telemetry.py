"""Read-only numeric observer. No simulator writes or credit changes."""
from c1_kit import arr
from c1_json import dumps

def snapshot(run):
 import c1_kit
 robots=[]
 for robot in run.robot:
  robots.append(dict(body_names=list(robot.body_names),joint_names=list(robot.joint_names),raw8=arr(robot.data.joint_pos)[0],body_pos_w=arr(robot.data.body_pos_w)[0],body_quat_w=arr(robot.data.body_quat_w)[0]))
 row=dict(contacts=getattr(c1_kit.ENGINE.contacts,'diagnostic_contacts',[]),status=c1_kit.ENGINE.status(),action14=run.command,measured14=run.q,robots=robots,tcp=[dict(position=run.tcp(a)[0],rotation=run.tcp(a)[1]) for a in range(2)])
 return __import__('json').loads(dumps(row))
