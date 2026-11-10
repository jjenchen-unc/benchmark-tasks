"""Chosen isolated runtime. Genuine reference/setup, no evaluator placements or loads."""
from pathlib import Path
import sys,os,argparse,time,traceback,hashlib
O=Path(__file__).resolve().parents[1]/'b1-bimanual-interface-v11'
sys.path.insert(0,str(O));sys.path.insert(0,str(O/'candidate'))
from protocol import check,receipt
from strict_json import dumps
p=argparse.ArgumentParser();p.add_argument('phase',choices=['demo']);p.add_argument('--review',type=Path,required=True);p.add_argument('--out',required=True,type=Path);a=p.parse_args()
m=check();assert m['fingerprint']=='0d5b3276c7873feb72cea378e91eb0c166ebce7d32ec24a722fad1cbc41e072d';credit=receipt(a.review);os.environ['B1_ROOT_REVIEW']=str(a.review.resolve())
from verify_runtime import verify
verify()
out=a.out.resolve()
if out.exists() or out.is_relative_to(O/'candidate'):raise ValueError('fresh output required')
import kit
from yam_arena.task_file import load
constructed=load(O/'model_task.py').module
if not constructed.OBJECTS:raise ValueError('pure TaskFile construction absent')
# Pure YAM definitions above, simulator must not load until this boundary.
from isaaclab.app import AppLauncher
if os.environ.get('B1B2_LEGACY_RTX')=='1':
 old=AppLauncher.__init__
 def init(self,*args,**kw):
  kw['kit_args']=(kw.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip();return old(self,*args,**kw)
 AppLauncher.__init__=init
from inspect_robots import Scene
from yam_arena.inspect_embodiment import yam_arena_embodiment
from observer import observe,emit,numeric
import physics,history
from protocol import CHANNELS
if physics.VALIDATED_RUNTIME_CHANNELS!=frozenset(CHANNELS) or history.VALIDATED_RUNTIME_CHANNELS!=frozenset(CHANNELS):raise RuntimeError('credited demo requires matching physical review')
out.mkdir(parents=True,exist_ok=False);emb=None;capture=None;start=time.monotonic()
r={'kind':'ACTUAL_B1_CREDITED_DEMO','phase':a.phase,'manifest':m,'promoted_channels':sorted(CHANNELS),'credit_sha256':hashlib.sha256(a.review.read_bytes()).hexdigest(),'entry_envelope_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'completed':False,'error':None}
try:
 emb=yam_arena_embodiment(task=str(O/'model_task.py'),device='cpu',object_jitter=0,control_hz=20,cam_width=320,cam_height=176)
 obs=emb.reset(Scene(id='b1-final-'+a.phase,instruction=__import__('protocol').load(O/'candidate/task.draft.json')['instruction']),seed=42);E=kit.ENGINE
 emit(out/'journal.jsonl',observe(E,'b1-wrench','reset'))
 from PIL import Image
 for name,rgb in obs.images.items():
  if name not in {'top_cam','left_cam','right_cam'}:raise ValueError('unexpected camera')
  Image.fromarray(rgb).save(out/(name+'.png'))
 if a.phase=='demo':
  from video_capture import Capture
  capture=Capture(out,obs.images)
  # Intercept only the actual embodiment action return. Teacher bytes stay exact.
  original=emb.step
  def step(action):
   # Preserve v6's strict before-action measurement validation as well as after.
   emit(out/'pre-actions.jsonl',observe(E,'b1-wrench','before_action',action.data))
   result=original(action)
   capture.append(result.observation.images)
   emit(out/'journal.jsonl',observe(E,'b1-wrench','after_action',action.data))
   return result
  emb.step=step
  from reference import Teacher
  from reference import ReferenceComplete
  try:r['controller_result']=Teacher(emb,E,[]).run()
  except ReferenceComplete:r['controller_result']=E.status()
 r['completed']=True
except BaseException as exc:
 r['error']=type(exc).__name__+': '+str(exc);r['traceback']=traceback.format_exc();raise
finally:
 r['wall_s']=time.monotonic()-start
 if kit.ENGINE is not None:r['final_status']=kit.ENGINE.status()
 if capture is not None:r['media']=capture.close()
 if emb is not None:
  try:emb.close()
  except BaseException as exc:r['close_error']=type(exc).__name__+': '+str(exc)
 r['source_after']=check();r['entry_envelope_after_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
 (out/'result.json').write_text(dumps(numeric(r),indent=2)+'\n')
