"""Portable actual launcher; genuine controller and read-only observations."""
from pathlib import Path
import sys,os,argparse,time,traceback
O=Path(__file__).resolve().parent
sys.path.insert(0,str(O));sys.path.insert(0,str(O/'candidate'))
from protocol import check
from strict_json import dumps
p=argparse.ArgumentParser();p.add_argument('phase',choices=['replay','setup']);p.add_argument('--out',type=Path,required=True);p.add_argument('--legacy-rtx-override',action='store_true');a=p.parse_args()
m=check();out=a.out.resolve()
if out.exists() or out.is_relative_to(O/'candidate'):raise ValueError('fresh output required')
from yam_arena.task_file import load
# Use the actual TaskFile API twice as the embodiment loader will do.
t1=load(O/'candidate/task.py');t2=load(O/'candidate/task.py')
if len(t1.objects)!=5 or len(t2.objects)!=5:raise ValueError('TaskFile object inventory')
import kit
from isaaclab.app import AppLauncher
if a.legacy_rtx_override:
 old=AppLauncher.__init__
 def init(self,*args,**kw):
  kw['kit_args']=(kw.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip();return old(self,*args,**kw)
 AppLauncher.__init__=init
from inspect_robots import Scene
from yam_arena.inspect_embodiment import yam_arena_embodiment
from first_cause import numeric
import physics
if physics.VALIDATED_CHANNELS:raise RuntimeError('replay must remain uncredited')
out.mkdir(parents=True,exist_ok=False);emb=None;capture=None;start=time.monotonic()
r={'kind':'ACTUAL_S2_RUNTIME','phase':a.phase,'manifest':m,'promoted_channels':[],'completed':False,'error':None}
def emit(E,action=None):
 row={'status':E.status(),'last_row':E.history.last,'action':action,'physics_dt':.002,'control_dt':.05}
 with (out/'journal.jsonl').open('a') as f:f.write(dumps(numeric(row))+'\n');f.flush();os.fsync(f.fileno())
try:
 emb=yam_arena_embodiment(task=str(O/'candidate/task.py'),device='cpu',object_jitter=0,control_hz=20,cam_width=320,cam_height=176)
 obs=emb.reset(Scene(id='s2-'+a.phase,instruction=t1.docs),seed=42);E=kit.ENGINE;emit(E)
 from PIL import Image
 for n,rgb in obs.images.items():Image.fromarray(rgb).save(out/(n+'.png'))
 if a.phase=='replay':
  from video_capture import Capture
  capture=Capture(out,obs.images)
  original=emb.step
  def step(action):
   result=original(action);capture.append(result.observation.images);emit(E,action.data);return result
  emb.step=step
  from reference import Reference
  r['controller_result']=Reference(emb,obs,out).run()
 r['completed']=True
except BaseException as exc:r['error']=type(exc).__name__+': '+str(exc);r['traceback']=traceback.format_exc();raise
finally:
 r['wall_s']=time.monotonic()-start
 if kit.ENGINE is not None:r['final_status']=kit.ENGINE.status()
 if capture is not None:r['media']=capture.close()
 if emb is not None:
  try:emb.close()
  except BaseException as exc:r['close_error']=repr(exc)
 r['source_after']=check()
 r['consumed_source_sha256']={str(Path(x.__file__).resolve()):__import__('hashlib').sha256(Path(x.__file__).read_bytes()).hexdigest() for x in list(sys.modules.values()) if getattr(x,'__file__',None) and Path(x.__file__).is_file()}
 (out/'result.json').write_text(dumps(numeric(r),indent=2)+'\n')
