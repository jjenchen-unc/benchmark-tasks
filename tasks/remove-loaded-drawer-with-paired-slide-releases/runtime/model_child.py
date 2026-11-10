"""Actual Inspect Dream entry; validates credit before importing simulator/provider."""
from pathlib import Path
import sys,os,argparse,time,hashlib,json,functools
O=Path(__file__).resolve().parent;C=O/'candidate';sys.path.insert(0,str(C));sys.path.insert(0,str(O))
from model_credit import validate_model
from strict_json import dumps
p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--review',required=True);a=p.parse_args()
receipt=validate_model(C,a.review)
os.environ['DRAWERS_CREDIT_REVIEW']=a.review
from yam_arena.task_file import load
loaded=load(C/'model_entry.py');task_module=loaded.module
sys.audit('drawers.model_task_constructed',str(loaded.path))
a.out=a.out.resolve();a.out.mkdir(parents=True,exist_ok=False)
from overview_capture import install as install_overview,OverviewCapture
install_overview();overview=None
meta=json.loads((C/'task.draft.json').read_text())
(a.out/'registration.json').write_text(dumps({'physical_case':'DR-T1/S','physical_seed':42,'inspect_trial_seed':'recorded at reset, passed unchanged','budget':3586,'control_hz':20,'instruction':meta['instruction'],'policy':'dream','model':'dreamlabs/dream-1-beta','rows_per_request':4,'review_sha256':hashlib.sha256((C/a.review).resolve().read_bytes()).hexdigest(),'source_manifest':json.loads((C/'manifest.json').read_text()),'outer_launch_inputs':json.loads((O/'launch-input-sha256.json').read_text()),'recording_overview_only':True,'credit':receipt},indent=2)+'\n')
from isaaclab.app import AppLauncher
old=AppLauncher.__init__
@functools.wraps(old)
def init(self,*args,**kw):
 kw['kit_args']=(kw.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip();return old(self,*args,**kw)
AppLauncher.__init__=init
import inspect_robots_dream.session as session_module
# SDK transient errors may retry in this existing session only. Session
# expiration/replacement and episode restart remain invalid; log every retry.
import dream
official_retry_after=session_module._retry_after
def retryable_existing_session(error):
 if isinstance(error,dream.DreamSessionExpired):return None
 pause=official_retry_after(error)
 if pause is not None:record({'kind':'transport_retry','error_class':type(error).__name__,'reason':getattr(error,'reason',None),'code':getattr(error,'code',None),'pause_s':pause})
 return pause
session_module._retry_after=retryable_existing_session
from inspect_robots_dream.session import DreamSession
from yam_arena.inspect_embodiment import YamArenaEmbodiment
original_reset=YamArenaEmbodiment.reset;original_step=YamArenaEmbodiment.step
reset_count=0

def checked_reset(self,*args,**kw):
 global reset_count,overview
 reset_count+=1
 if reset_count!=1:raise RuntimeError('single episode only; no reset recovery')
 seed=kw.get('seed',args[1] if len(args)>1 else None)
 if type(seed) is not int:raise ValueError('integer Inspect seed required')
 obs=original_reset(self,*args,**kw)
 overview=OverviewCapture(self,a.out,obs)
 (a.out/'seed-contract.json').write_text(dumps({'physical_seed':42,'inspect_trial_seed':seed,'passed_unchanged':True})+'\n')
 return obs

def checked_step(self,action):
 result=original_step(self,action)
 if overview is None:raise RuntimeError('Missing recording overview after reset')
 overview.append(result.observation)
 import drawers_kit as kit
 with (a.out/'action-journal.jsonl').open('a') as f:f.write(dumps({'action':action.data,'status':kit.ENGINE.status(),'physics':kit.ENGINE.last})+'\n')
 return result
YamArenaEmbodiment.reset=checked_reset;YamArenaEmbodiment.step=checked_step
start,act=DreamSession.start,DreamSession.act;episode_started=False
with (a.out/'continuity.jsonl').open('x',buffering=1) as log:
 def record(row):log.write(dumps(row)+'\n')
 def wrapped_start(self,*args,**kw):
  global episode_started
  if episode_started:raise RuntimeError('Dream episode restart prohibited')
  episode_started=True;result=start(self,*args,**kw);self._drawers_state={'session':None,'count':0,'finished':None};record({'kind':'episode_start'});return result
 def wrapped_act(self,*args,**kw):
  state=self._drawers_state;t=time.monotonic();chunk=act(self,*args,**kw);end=time.monotonic();ident=hashlib.sha256(chunk.session_id.encode()).hexdigest() if chunk.session_id else None
  continuous=bool(ident) and (state['session'] is None or ident==state['session'])
  record({'kind':'chunk','index':state['count']+1,'session_sha256':ident,'continuous':continuous,'idle_s':None if state['finished'] is None else t-state['finished'],'request_s':end-t,'attempts':chunk.attempts})
  if type(chunk.attempts) is not int or chunk.attempts<1 or not continuous:raise RuntimeError('retry/session discontinuity invalid')
  state.update(session=ident,count=state['count']+1,finished=end);return chunk
 DreamSession.start,DreamSession.act=wrapped_start,wrapped_act
 sys.argv=['inspect-robots','run','--embodiment','yam_arena','--policy','dream','-P','model=dreamlabs/dream-1-beta','-P','rows_per_request=4','-E','task='+str(C/'model_entry.py'),'-E','control_hz=20','-E','device=cpu','-E','object_jitter=0','-E','cam_width=320','-E','cam_height=176','--instruction',meta['instruction'],'--scorer','task_score','--max-steps','3586','--seed','42','--store-frames','--no-prompt','--log-dir',str(a.out/'inspect')]
 from inspect_robots.cli import main
 try:main()
 finally:
  if overview is not None:overview.close()
