"""Run the scripted demo controller (--plan), replay a recorded 14D action log, or hold the start posture, through the task's embodiment and live scorer, recording the three cameras."""
from pathlib import Path
import sys,json,time,argparse
p=argparse.ArgumentParser();p.add_argument('--actions',type=Path);p.add_argument('--noop',type=int,default=0);p.add_argument('--plan',action='store_true');p.add_argument('--out',type=Path,required=True);p.add_argument('--rtx-driver-check-override',action='store_true');a=p.parse_args()
O=Path(__file__).resolve().parent;sys.path.insert(0,str(O));sys.path.insert(0,str(O/'candidate'))
out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
rows=[json.loads(x) for x in a.actions.read_text().splitlines()] if a.actions else []
from yam_arena.task_file import load
t=load(O/'candidate/task.py')
import kit
from first_cause import numeric
from isaaclab.app import AppLauncher
if a.rtx_driver_check_override:
 old=AppLauncher.__init__
 def init(self,*args,**kw):
  kw['kit_args']=(kw.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip();return old(self,*args,**kw)
 AppLauncher.__init__=init
import numpy as np
from inspect_robots import Scene
from inspect_robots.types import Action
from yam_arena.inspect_embodiment import yam_arena_embodiment
from video_capture import Capture
start=time.monotonic()
emb=yam_arena_embodiment(task=str(O/'candidate/task.py'),device='cpu',object_jitter=0,control_hz=20,cam_width=320,cam_height=176)
obs=emb.reset(Scene(id='s2-replay',instruction=t.docs),seed=42);E=kit.ENGINE
from PIL import Image
for n,rgb in obs.images.items():Image.fromarray(rgb).save(out/(n+'.png'))
cap=Capture(out,obs.images)
if a.noop:rows=[np.asarray(obs.state['joint_pos'],float).tolist()]*a.noop
live={'reason':None,'score':None}
f=(out/'journal.jsonl').open('w');original=emb.step
def step(action):
 r=original(action);cap.append(r.observation.images);live['score']=r.info.get('score')
 f.write(json.dumps(numeric({'status':E.status(),'last_row':E.history.last,'action':np.asarray(action.data,float).tolist(),'info_score':live['score']}))+'\n')
 if E.controls%200==1:print(json.dumps({'control':E.controls,'score':E.credit(),'info_score':live['score'],'hold_s':E.history.hold,'t':round(time.monotonic()-start)}),flush=True)
 if r.terminated:live['reason']=r.termination_reason
 return r
emb.step=step
if a.plan:
 from reference import Reference
 controller=Reference(emb,obs,out).run();print('CONTROLLER '+json.dumps(controller),flush=True)
 (out/'demo-actions.jsonl').write_text(''.join(json.dumps(json.loads(x)['q_command'])+'\n' for x in (out/'robot-actions.jsonl').read_text().splitlines()))
else:
 for act in rows:
  r=emb.step(Action(data=np.array(act,float)))
  if r.terminated:break
f.close();reason,info_score=live['reason'],live['score']
s=E.status();media=cap.close()
res={'steps':E.controls,'logged_actions':len(rows),'termination_reason':reason,'live_score':s['score'],'info_score':info_score,'hold_s':s['hold_s'],'failure':s['failure'],'peak_instrument_tilt_deg':s['peak_instrument_tilt_deg'],'peak_case_tilt_deg':s['peak_case_tilt_deg'],'engaged':s['engaged'],'frames_per_camera':media['frames_per_camera'],'wall_s':round(time.monotonic()-start,1)}
(out/'result.json').write_text(json.dumps(numeric(res),indent=2)+'\n');print('FINAL '+json.dumps(numeric(res)),flush=True)
emb.close()
