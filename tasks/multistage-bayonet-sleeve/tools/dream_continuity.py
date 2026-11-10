"""Official Dream policy with per-episode server-session continuity checks."""
import argparse,functools,hashlib,json,sys,time,os
from pathlib import Path
p=argparse.ArgumentParser(add_help=False);p.add_argument('--continuity-log',type=Path,required=True);a,forward=p.parse_known_args()
# Construct actual pure task and verify pins/receipt before simulator or model imports.
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from runtime_binding import verify_runtime
verify_runtime()
from yam_arena.task_file import load
loaded_task=load(Path(__file__).resolve().parents[1]/'task.py')
task_module=loaded_task.module
sys.audit('b7.task_constructed',str(loaded_task.path),str(task_module.__file__))
from validation_receipt import verify_receipt
if not verify_receipt():raise RuntimeError('B7 actual receipt unavailable')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from package_checks import preflight
from numeric import dumps
preflight(Path(__file__).resolve().parents[1])
if os.environ.get('B4_LEGACY_RTX_OVERRIDE')=='1':
 from isaaclab.app import AppLauncher
 old=AppLauncher.__init__
 @functools.wraps(old)
 def init(self,*args,**kw):kw['kit_args']=(kw.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip();return old(self,*args,**kw)
 AppLauncher.__init__=init
import inspect_robots_dream.session as session_module
# Preserve official SDK handling of retryable transport/server faults within the
# same episode. Session expiration/replacement remains invalid, with no fallback.
import dream
official_retry_after=session_module._retry_after
def retryable_existing_session(error):
 if isinstance(error,dream.DreamSessionExpired):return None
 pause=official_retry_after(error)
 if pause is not None:
  record({'kind':'transport_retry','error_class':type(error).__name__,'reason':getattr(error,'reason',None),'code':getattr(error,'code',None),'pause_s':pause})
 return pause
session_module._retry_after=retryable_existing_session
from inspect_robots_dream.session import DreamSession
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from numeric import dumps
from yam_arena.inspect_embodiment import YamArenaEmbodiment
original_reset=YamArenaEmbodiment.reset
def checked_reset(self,*args,**kw):
 physical={'physical_seed':42,'setting':'purple','goal':'remove_C','inspect_trial_seed':kw.get('seed',args[1] if len(args)>1 else None)}
 if type(physical['inspect_trial_seed']) is not int:raise RuntimeError('Expected integer Inspect reset seed')
 (a.continuity_log.parent/'seed-contract.json').write_text(json.dumps(physical,allow_nan=False)+'\n')
 return original_reset(self,*args,**kw)
YamArenaEmbodiment.reset=checked_reset
original_step=YamArenaEmbodiment.step
def checked_step(self,action):
 # Pure serialization validation of actual telemetry before every Action.
 module=sys.modules.get('kit')
 if module is None:raise RuntimeError('B4 runtime not loaded')
 dumps(module.current_status())
 return original_step(self,action)
YamArenaEmbodiment.step=checked_step
start,act=DreamSession.start,DreamSession.act
episode_started=False
with a.continuity_log.open('x',buffering=1) as log:
 def record(row):log.write(json.dumps(row)+'\n')
 def wrapped_start(self,*args,**kw):

  global episode_started
  if episode_started:raise RuntimeError('Episode restart prohibited')
  episode_started=True
  result=start(self,*args,**kw);self._s4_session_state={'session':None,'count':0,'finished':None};record({'kind':'episode_start'});return result
 def wrapped_act(self,*args,**kw):
  state=self._s4_session_state;t=time.monotonic();chunk=act(self,*args,**kw);end=time.monotonic();ident=hashlib.sha256(chunk.session_id.encode()).hexdigest() if chunk.session_id else None;continuous=bool(ident) and (state['session'] is None or ident==state['session']);record({'kind':'chunk','index':state['count']+1,'session_sha256':ident,'continuous':continuous,'idle_s':None if state['finished'] is None else t-state['finished'],'request_s':end-t,'attempts':chunk.attempts})
  if type(chunk.attempts) is not int or chunk.attempts<1:raise RuntimeError('Invalid request attempt count')
  if not continuous:raise RuntimeError('Dream session changed/missing; episode invalid')
  state.update(session=ident,count=state['count']+1,finished=end);return chunk
 DreamSession.start,DreamSession.act=wrapped_start,wrapped_act
 sys.argv=['inspect-robots',*forward]
 from inspect_robots.cli import main
 main()
