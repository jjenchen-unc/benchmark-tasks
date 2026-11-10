"""Portable genuine S5 reference/setup; source checks before SDK boundary."""
from pathlib import Path
import sys,os,argparse,time,traceback,functools
O=Path(__file__).resolve().parent;C=O/'candidate'
sys.path.insert(0,str(O));sys.path.insert(0,str(C))
from protocol import check
from strict_json import dumps
p=argparse.ArgumentParser();p.add_argument('phase',choices=['replay','setup']);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
m=check()
from verify_runtime import verify
verify()
out=a.out.resolve()
if out.exists() or out.is_relative_to(C):raise ValueError('fresh output required')
from yam_arena.task_file import load
loaded=load(C/'task.py');task=loaded.module
if len(loaded.objects)!=8:raise ValueError('native eight objects required')
sys.audit('s5.task_constructed',str(loaded.path),len(loaded.objects))
# Pure YAM construction precedes AppLauncher. No simulator import above.
from isaaclab.app import AppLauncher
old=AppLauncher.__init__
@functools.wraps(old)
def init(self,*args,**kw):
 kw['kit_args']=(kw.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip();return old(self,*args,**kw)
AppLauncher.__init__=init
from inspect_robots import Scene
from yam_arena.inspect_embodiment import yam_arena_embodiment
from observer import observe,emit
import s5_kit as kit,s5_rules
if s5_rules.VALIDATED_CHANNELS:raise RuntimeError('replay must remain uncredited')
os.environ['S5_TRACE']=str(out/'physics.jsonl')
out.mkdir(parents=True,exist_ok=False);emb=None;capture=None;start=time.monotonic()
r=dict(kind='ACTUAL_S5_RUNTIME',phase=a.phase,manifest=m,promoted_channels=[],completed=False,error=None)
try:
 emb=yam_arena_embodiment(task=str(C/'task.py'),device='cpu',object_jitter=0,control_hz=20,cam_width=320,cam_height=176)
 obs=emb.reset(Scene(id='s5-final-'+a.phase,instruction=loaded.docs),seed=20261109)
 emit(out/'journal.jsonl',observe(emb,obs,'reset'))
 from PIL import Image
 for name,rgb in obs.images.items():
  if name not in {'top_cam','left_cam','right_cam'}:raise ValueError('unexpected camera')
  Image.fromarray(rgb).save(out/(name+'.png'))
 if a.phase=='replay':
  from video_capture import Capture
  capture=Capture(out,obs.images);original=emb.step;last=[obs]
  def step(action):
   emit(out/'pre-actions.jsonl',observe(emb,last[0],'before_action',action.data))
   result=original(action);last[0]=result.observation;capture.append(result.observation.images)
   emit(out/'journal.jsonl',observe(emb,result.observation,'after_action',action.data));return result
  emb.step=step
  from reference import run
  with (out/'robot-actions.jsonl').open('x') as f:r['controller_result']=run(emb,obs,f)
 r['completed']=True
except BaseException as exc:
 r['error']=type(exc).__name__+': '+str(exc);r['traceback']=traceback.format_exc();raise
finally:
 r['wall_s']=time.monotonic()-start
 if kit.ENGINE is not None:r['final_status']=kit.ENGINE.status()
 try:
  if capture is not None:r['media']=capture.close()
 except BaseException as exc:r['close_error']=type(exc).__name__+': '+str(exc)
 try:
  if emb is not None:emb.close()
 except BaseException as exc:r['close_error']=type(exc).__name__+': '+str(exc)
 (out/'result.json').write_text(dumps(r,indent=2)+'\n')
