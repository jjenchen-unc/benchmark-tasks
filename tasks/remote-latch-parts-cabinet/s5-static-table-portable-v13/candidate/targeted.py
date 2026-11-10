"""EVALUATOR ONLY: injected paddle displacement, spring return, wrist abort, reset.
No robotic feasibility or credit; kept separate from genuine teacher.
"""
import numpy as np
from inspect_robots import Scene
from inspect_robots.types import Action

def run(emb,obs,out):
 import torch
 import s5_kit
 from telemetry import dumps,snapshot,preflight
 E=s5_kit.ENGINE;robot_q=np.asarray(obs.state['joint_pos'],float);cab=emb._base.scene['cab_A']
 idx=cab.joint_names.index('A_paddle2_slide');q=cab.data.joint_pos.torch.clone();v=torch.zeros_like(q);q[0,idx]=.011
 cab.write_joint_state_to_sim(q,v)
 rows=[]
 for _ in range(10):
  r=emb.step(Action(data=robot_q));rows.append(snapshot(emb,r.observation))
  if r.terminated:break
 spring_q=[row['observer']['paddle_q']['A_paddle2_slide'] for row in rows]
 release=any(e[1:] == ['AR','release'] for e in E.latches.events)
 (out/'evaluator-paddle.json').write_text(dumps(dict(evaluator_only=True,rows=rows,release_seen=release,spring_q=spring_q))+'\n')
 obs=emb.reset(Scene(id='S5-targeted-reset-after-loaded-paddle',instruction=emb.task.docs),seed=99117);E=s5_kit.ENGINE
 reset=snapshot(emb,obs);(out/'reset-after-paddle.json').write_text(dumps(reset)+'\n');restored=E.controls==0 and E.hist.samples==0 and E.hist.time==0 and not E.latches.events
 # Evaluator-prepared table support and 21second release window: no grasp credit.
 box=emb._base.scene['A_box_R'];origin=s5_kit.arr(emb._base.scene.env_origins)[0]
 pose=torch.tensor([[.05+float(origin[0]),-.10+float(origin[1]),.7795+float(origin[2]),0.,0.,0.,1.]],device=emb._base.device,dtype=torch.float32)
 box.write_root_link_pose_to_sim_index(root_pose=pose);box.write_root_link_velocity_to_sim_index(root_velocity=torch.zeros((1,6),device=emb._base.device))
 supported=[]
 with (out/'evaluator-table-support.jsonl').open('w') as f:
  for _ in range(420):
   r=emb.step(Action(data=np.asarray(obs.state['joint_pos'],float)));row=snapshot(emb,r.observation);f.write(dumps(row)+'\n');supported.append(row['observer']['box']['table_contact'])
   if r.terminated:break
 table_result=dict(evaluator_only=True,support_samples=sum(supported),controls=len(supported),status=E.status(),positive_robot_contact_verified=False)
 (out/'evaluator-table-result.json').write_text(dumps(table_result)+'\n')
 obs=emb.reset(Scene(id='S5-targeted-before-overload',instruction=emb.task.docs),seed=99119);E=s5_kit.ENGINE
 from negative import run as overload
 abort=overload(emb,obs,out)
 obs=emb.reset(Scene(id='S5-targeted-reset-after-overload',instruction=emb.task.docs),seed=99118);E=s5_kit.ENGINE
 after=snapshot(emb,obs)
 return dict(evaluator_only=True,robotic_credit=False,table_result=table_result,paddle_release_seen=release,spring_return=bool(spring_q[-1]<spring_q[0]),reset_after_paddle=restored,overload=abort,reset_after_overload=E.hist.failure is None and E.controls==0 and E.hist.samples==0,reset_snapshot=after)
