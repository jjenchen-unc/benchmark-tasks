"""Credited production entry with unchanged native controller and recording view."""
from pathlib import Path
import argparse,sys,json,os,hashlib,functools,time
O=Path(__file__).resolve().parent;C=O/'candidate';sys.path.insert(0,str(C));sys.path.insert(0,str(O))
p=argparse.ArgumentParser();p.add_argument('--mode',choices=['setup','demo'],required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--review',required=True);a=p.parse_args();a.out=a.out.resolve()
from package_checks import check
before=check()
from model_credit import validate_model
receipt=validate_model(C,a.review);os.environ['DRAWERS_CREDIT_REVIEW']=a.review
assert not a.out.exists();a.out.mkdir(parents=True)
from strict_json import dumps
def save(name,value):(a.out/name).write_text(dumps(value,indent=2)+'\n')
save('source-registration.json',{'candidate_manifest':json.loads((C/'manifest.json').read_text()),'outer_launch_inputs':json.loads((O/'launch-input-sha256.json').read_text()),'credit':receipt,'mode':a.mode,'model_calls':0})
from yam_arena.task_file import load
loaded=load(C/'model_entry.py');module=loaded.module
from overview_capture import install,OverviewCapture,physical_snapshot
install()
from isaaclab.app import AppLauncher
original=AppLauncher.__init__
@functools.wraps(original)
def init(self,*args,**kw):kw['kit_args']=(kw.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip();return original(self,*args,**kw)
AppLauncher.__init__=init
from inspect_robots import Scene
from yam_arena.inspect_embodiment import yam_arena_embodiment
import drawers_kit as kit
emb=None;overview=None;capture=None;report={'mode':a.mode,'model_calls':0,'completed':False};started=time.monotonic()
try:
 instruction=json.loads((C/'task.draft.json').read_text())['instruction'];emb=yam_arena_embodiment(task=str(C/'model_entry.py'),device='cpu',object_jitter=0,control_hz=20,cam_width=320,cam_height=176)
 obs=emb.reset(Scene(id='drawers-credited-public-'+a.mode,instruction=instruction),seed=42);status=kit.ENGINE.status()
 if status['credit_gate']!='validated' or status['missing_channels'] or status['measurement_error'] is not None:raise RuntimeError('Actual production scorer credit unavailable')
 overview=OverviewCapture(emb,a.out,obs);report['zero_step_state']=physical_snapshot(emb)
 if a.mode=='demo':
  from video_capture import Capture
  capture=Capture(a.out,obs.images)
  from reference import run,FullGoalReached
  def record(label,observation,result,command=None):
   capture.append(observation.images);overview.append(observation)
   with (a.out/'journal.jsonl').open('a') as f:f.write(dumps({'label':label,'action14':command,'status':kit.ENGINE.status(),'physics':kit.ENGINE.last,'termination':result.termination_reason if result.terminated else None})+'\n')
   if result.terminated and result.termination_reason!='success':raise RuntimeError('Early public demo termination '+str(result.termination_reason))
  try:run(emb,obs,record)
  except FullGoalReached:pass
  status=kit.ENGINE.status()
  if status['score']!=1. or status['candidate_score']!=1. or status['hold_s']<1. or status['missing_channels'] or status['measurement_error'] is not None or status['failure'] is not None:raise RuntimeError('Actual credited full goal not reached')
 report.update(completed=True,status=kit.ENGINE.status(),instruction=instruction,wall_s=time.monotonic()-started)
except BaseException as error:
 report['error']=type(error).__name__+': '+str(error);raise
finally:
 try:
  if capture is not None:report['policy_media']=capture.close()
  if overview is not None:report['overview_media']=overview.close()
  after=check();report['source_unchanged']=before==after
  if not report['source_unchanged']:raise RuntimeError('Consumed candidate changed')
  if report.get('completed') and a.mode=='demo':
   assert report['policy_media']['all_encoded'] and report['policy_media']['frames_per_camera']==report['status']['controls']+1 and report['overview_media']['frames']==report['status']['controls']+1
 finally:
  save('result.json',report)
  if emb is not None:emb.close()
