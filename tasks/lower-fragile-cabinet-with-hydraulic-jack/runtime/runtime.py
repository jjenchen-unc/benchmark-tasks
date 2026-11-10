"""Actual isolated replay/setup. Native controller/physics bytes remain exact."""
from pathlib import Path
import sys,os,argparse,time,traceback,functools
O=Path(__file__).resolve().parent;C=O/'candidate'
sys.path.insert(0,str(O));sys.path.insert(0,str(C))
from protocol import check,load as load_json
from strict_json import dumps
p=argparse.ArgumentParser();p.add_argument('phase',choices=['replay','setup']);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
m=check()
from verify_runtime import verify
verify()
from yam_arena.task_file import load
task=load(C/'task.py')
# Pure definitions must complete before this import, under the real -I launcher.
from isaaclab.app import AppLauncher
if os.environ.get('HYDRAULIC_LEGACY_RTX')=='1':
 original=AppLauncher.__init__
 @functools.wraps(original)
 def init(self,*args,**kw):
  kw['kit_args']=(kw.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip();return original(self,*args,**kw)
 AppLauncher.__init__=init
out=a.out.resolve()
if out.exists() or out.is_relative_to(C):raise ValueError('fresh output required')
out.mkdir(parents=True);os.environ['HJ_PHYSICS_TRACE']=str(out/'physics.jsonl')
from inspect_robots import Scene
from yam_arena.inspect_embodiment import yam_arena_embodiment
from telemetry import snapshot,before_action
import kit
r={'kind':'ACTUAL_HYDRAULIC_RUNTIME','phase':a.phase,'manifest':m,'completed':False,'error':None};emb=None;capture=None;t=time.monotonic()
stream=(out/'trace.jsonl').open('x',buffering=1)
try:
 emb=yam_arena_embodiment(task=str(C/'task.py'),device='cpu',object_jitter=0,control_hz=20,cam_width=320,cam_height=176)
 obs=emb.reset(Scene(id='hydraulic-final-'+a.phase,instruction=load_json(C/'task.draft.json')['instruction']),seed=87321)
 if set(obs.images)!={'left_cam','right_cam','top_cam'} or any(v.std()<=5 for v in obs.images.values()):raise RuntimeError('actual reset RGB unavailable')
 stream.write(dumps(snapshot(emb,obs,'initial'))+'\n')
 from PIL import Image
 for name,rgb in obs.images.items():Image.fromarray(rgb).save(out/(name+'.png'))
 if a.phase=='replay':
  from video_capture import Capture
  capture=Capture(out,obs.images)
  def record(res,action):
   stream.write(dumps(snapshot(emb,res.observation,'after_action',action))+'\n');capture.append(res.observation.images)
  emb._hj_pre_action=lambda obs,action:before_action(emb,obs,action,stream)
  from reference import Driver,Completed
  try:Driver(emb,obs,record).run()
  except Completed:r['ended_on_success']=True
 r['completed']=True
except BaseException as e:
 r['error']=type(e).__name__+': '+str(e);r['traceback']=traceback.format_exc();raise
finally:
 if kit.ENGINE is not None:r['final']=kit.ENGINE.status()
 if capture is not None:r['media']=capture.close()
 stream.close();r['wall_s']=time.monotonic()-t
 if emb is not None:
  if kit.ENGINE is not None and kit.ENGINE.trace:kit.ENGINE.trace.close()
  try:emb.close()
  except BaseException as e:r['close_error']=str(e)
 (out/'result.json').write_text(dumps(r,indent=2)+'\n')
