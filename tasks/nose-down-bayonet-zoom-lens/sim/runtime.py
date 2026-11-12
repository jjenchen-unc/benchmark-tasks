"""Scripted reference, fixed-action replay or reset-only setup with camera recording; no task writes."""
from pathlib import Path
import sys,os,argparse,runpy,dataclasses,time
O=Path(__file__).resolve().parent;C=O/'scene';sys.path.insert(0,str(O));sys.path.insert(0,str(C))
from protocol import load
from strict_json import dumps

def main():
 p=argparse.ArgumentParser();p.add_argument('phase',choices=['reference','replay','setup']);p.add_argument('--out',type=Path,required=True);p.add_argument('--actions',type=Path);p.add_argument('--rtx-driver-check-override',action='store_true');a=p.parse_args()
 out=a.out.resolve()
 if out.exists() or out.is_relative_to(C):raise ValueError('fresh output outside candidate required')
 if a.phase=='replay' and not a.actions:raise ValueError('reference actions required')
 actions=load(a.actions) if a.actions else None
 if actions is not None:
  if type(actions) is not list or not 0<len(actions)<=3034:raise ValueError('native action budget')
  from protocol import number
  for i,row in enumerate(actions):
   if type(row['control']) is not int or row['control']!=i+1 or len(row['action'])!=14:raise ValueError('action contract')
   for x in row['action']:number(x)
 from yam_arena.task_file import load as task_load
 one=task_load(C/'task.py');two=task_load(C/'task.py')
 assert one.objects==two.objects and one.module.SCORE_OVER==two.module.SCORE_OVER
 sys.audit('b6.task_constructed',str(one.path),len(one.objects))
 from isaaclab.app import AppLauncher
 if a.rtx_driver_check_override:
  previous=AppLauncher.__init__
  def launch(self,*args,**kw):
   kw['kit_args']=(kw.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip();return previous(self,*args,**kw)
  AppLauncher.__init__=launch
 from yam_arena.inspect_embodiment import YamArenaEmbodiment,yam_arena_embodiment
 from inspect_robots import Scene
 from inspect_robots.types import Action
 import kit
 from video_capture import Capture
 out.mkdir(parents=True,exist_ok=False);os.environ['B6_TRACE']=str(out/'physics.jsonl')
 instruction=load(O.parent/'task.json')['instruction'];journal=[];cap=None;engine=None;emb=None;error=None;close_error=None;native=None;reset_count=0
 reset,step,close=YamArenaEmbodiment.reset,YamArenaEmbodiment.step,YamArenaEmbodiment.close
 def write(obs,action):
  row=dict(control=kit.ENGINE.controls,action=action,status=kit.ENGINE.status(),measured_joint_pos=obs.state['joint_pos'])
  journal.append(row)
  with (out/'journal.jsonl').open('a') as f:f.write(dumps(row)+'\n')
 def reset_checked(self,scene,*args,**kwargs):
  nonlocal cap,engine,reset_count,emb
  reset_count+=1
  if reset_count!=1:raise RuntimeError('single fresh episode only')
  emb=self;obs=reset(self,dataclasses.replace(scene,instruction=instruction),*args,**kwargs);engine=kit.ENGINE
  if engine.error:raise RuntimeError(engine.error)
  seed=kwargs.get('seed',args[0] if args else None)
  if type(seed) is not int:raise ValueError('numeric Inspect seed required')
  (out/'seed-contract.json').write_text(dumps(dict(physical_seed=42,inspect_trial_seed=seed,passed_unchanged=True,instruction=instruction))+'\n')
  import cv2,numpy as np
  rgb=np.concatenate([obs.images[n] for n in ('top_cam','left_cam','right_cam')],axis=1)
  if not cv2.imwrite(str(out/'setup.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)):raise RuntimeError('setup encoding failed')
  cap=Capture(out,obs.images);write(obs,None);return obs
 def step_checked(self,action):
  result=step(self,action);cap.append(result.observation.images);write(result.observation,action.data);return result
 def close_checked(self):
  nonlocal close_error
  try:
   if cap is not None:cap.close()
  except BaseException as exc:close_error=type(exc).__name__+': '+str(exc)
  return close(self)
 YamArenaEmbodiment.reset=reset_checked;YamArenaEmbodiment.step=step_checked;YamArenaEmbodiment.close=close_checked
 started=time.monotonic()
 try:
  if a.phase=='reference':
   import reference
   native=reference.run(out,None) # Whole route; explicit first-lift diagnostic never selected.
  else:
   emb=yam_arena_embodiment(task=str(C/'task.py'),device='cpu',control_hz=20,object_jitter=0,cam_width=320,cam_height=176)
   emb.reset(Scene(id='B6-portable-'+a.phase,instruction=instruction),seed=42)
   if a.phase=='replay':
    for row in actions:
     result=emb.step(Action(data=row['action']))
     if result.terminated:break
    (out/'actions.json').write_text(dumps(actions[:kit.ENGINE.controls])+'\n')
   emb.close();emb=None
 except BaseException as exc:
  error=type(exc).__name__+': '+str(exc)
  if emb is not None:
   try:emb.close()
   except BaseException as close_exc:close_error=type(close_exc).__name__+': '+str(close_exc)
 finally:
  r=dict(kind='ACTUAL_B6_PORTABLE',phase=a.phase,completed=error is None,error=error,close_error=close_error,native_result=native,controls=engine.controls if engine else 0,final_status=engine.status() if engine else None,termination_reason=engine.termination_reason() if engine else None,wall_s=time.monotonic()-started)
  (out/'result.json').write_text(dumps(r,indent=2)+'\n')
 if error or close_error:raise RuntimeError(error or close_error)
 print(dumps(dict(phase=a.phase,controls=r['controls'],score=r['final_status']['score'] if engine else None,termination_reason=r['termination_reason'],measured_rubric=r['final_status']['measured_rubric'] if engine else None)))
if __name__=='__main__':main()
