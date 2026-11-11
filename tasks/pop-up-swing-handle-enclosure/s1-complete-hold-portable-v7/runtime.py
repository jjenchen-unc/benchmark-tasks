"""Chosen isolated S1 replay/setup. No evaluator placement or load path."""
from pathlib import Path
import sys,os,argparse,time,traceback,functools
O=Path(__file__).resolve().parent;sys.path.insert(0,str(O));sys.path.insert(0,str(O/'candidate'))
from protocol import check
from strict_json import dumps
p=argparse.ArgumentParser();p.add_argument('phase',choices=['replay','setup']);p.add_argument('--out',type=Path,required=True);a=p.parse_args();m=check()
from verify_runtime import verify
verify()
out=a.out.resolve()
if out.exists() or out.is_relative_to(O/'candidate'):raise ValueError('fresh output required')
os.environ['S1_TARGET']='SH-T1'
from yam_arena.task_file import load
first=load(O/'candidate/task.py');second=load(O/'candidate/task.py')
if len(first.objects)!=1 or len(second.objects)!=1:raise ValueError('pure double TaskFile construction failed')
sys.audit('s1.task_constructed',str(second.path))
from isaaclab.app import AppLauncher
if os.environ.get('S1_LEGACY_RTX')=='1':
 old=AppLauncher.__init__
 @functools.wraps(old)
 def init(self,*args,**kw):
  kw['kit_args']=(kw.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip();return old(self,*args,**kw)
 AppLauncher.__init__=init
from inspect_robots import Scene
from inspect_robots.types import Action
from yam_arena.inspect_embodiment import yam_arena_embodiment
import s1_kit as kit,numpy as np,observer
if kit.VERIFIED_CHANNELS:raise RuntimeError('uncredited replay required')
out.mkdir(parents=True,exist_ok=False);os.environ['S1_TRACE']=str(out/'physics.jsonl');emb=None;capture=None;start=time.monotonic()
r=dict(kind='ACTUAL_S1_RUNTIME',phase=a.phase,manifest=m,promoted_channels=[],completed=False,error=None)
try:
 emb=yam_arena_embodiment(task=str(O/'candidate/task.py'),device='cpu',object_jitter=0,control_hz=20,cam_width=320,cam_height=176);observer.emb=emb
 instruction=__import__('protocol').load(O/'candidate/task.draft.json')['instruction']
 obs=emb.reset(Scene(id='s1-final-'+a.phase,instruction=instruction),seed=42)
 from PIL import Image
 for name,rgb in obs.images.items():
  if name not in {'top_cam','left_cam','right_cam'}:raise ValueError('unexpected camera')
  Image.fromarray(rgb).save(out/(name+'.png'))
 (out/'reset.json').write_text(dumps(observer.observe(obs),indent=2)+'\n')
 if a.phase=='replay':
  from reference import Teacher
  from video_capture import Capture
  teacher=Teacher();capture=Capture(out,obs.images)
  with (out/'controller-stream.jsonl').open('x',buffering=1) as log:
   for control in range(1,4199):
    before=observer.observe(obs);data,phase=teacher.next(obs.state['joint_pos'],kit.ENGINE.last['wrist_force_n'] if kit.ENGINE.last else [0.,0.],before['fixture'],before['robot']['robots']['right']['raw_joint_pos'][-2:])
    result=emb.step(Action(data=np.asarray(data)));obs=result.observation;capture.append(obs.images)
    log.write(dumps(dict(control=control,phase=phase,action=data,feedback=teacher.telemetry,pre=before,post=observer.observe(obs)))+'\n')
    if result.terminated or teacher.finished:break
 r['completed']=True
except BaseException as exc:r['error']=type(exc).__name__+': '+str(exc);r['traceback']=traceback.format_exc();raise
finally:
 r['wall_s']=time.monotonic()-start
 if kit.ENGINE is not None:r['final_status']=kit.ENGINE.status()
 if capture is not None:r['media']=capture.close()
 if emb is not None:
  try:emb.close()
  except BaseException as exc:r['close_error']=repr(exc)
 (out/'result.json').write_text(dumps(r,indent=2)+'\n')
