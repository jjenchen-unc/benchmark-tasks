"""Official Dream policy with per-episode server-session continuity checks."""
import argparse,functools,hashlib,json,sys,time,os
from pathlib import Path
import faulthandler
faulthandler.enable(all_threads=True)
p=argparse.ArgumentParser(add_help=False);p.add_argument('--continuity-log',type=Path,required=True);p.add_argument('--forward-argv-json',type=Path);a,forward=p.parse_known_args()
# Kit crashreporter's proc-cmdline reader truncates OS argv at newlines.
# Decode the exact Inspect arguments from a source-bound JSON file instead.
if a.forward_argv_json is not None:
 if forward:raise RuntimeError('Do not combine JSON argv transport with inline Inspect arguments')
 forward=json.loads(a.forward_argv_json.read_text())
 if not isinstance(forward,list) or any(not isinstance(value,str) for value in forward):raise RuntimeError('Expected a JSON array of exact CLI strings')
def _startup_checkpoint(event):
 with (a.continuity_log.parent/'startup-stages.jsonl').open('a',buffering=1) as stream:
  stream.write(json.dumps({'event':event,'monotonic_s':time.monotonic(),'pid':os.getpid(),'isolated':bool(sys.flags.isolated)})+'\n')
_startup_checkpoint('helper_entered_before_task_runtime_imports')
# Construct actual pure task and verify pins/receipt before simulator or model imports.
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'task'))
_startup_checkpoint('before_runtime_binding_import')
from runtime_binding import verify_runtime
verify_runtime()
_startup_checkpoint('runtime_pins_verified')
_startup_checkpoint('before_pure_task_definition_import')
from yam_arena.task_file import load
loaded_task=load(Path(__file__).resolve().parents[1]/'task/task.py')
_startup_checkpoint('pure_task_constructed')
task_module=loaded_task.module
sys.audit('b7.task_constructed',str(loaded_task.path),str(task_module.__file__))
from validation_receipt import validated_channels,CHANNELS
if validated_channels()!=CHANNELS:raise RuntimeError('B4 actual receipt unavailable')
_startup_checkpoint('all_seven_sourcebound_channels_verified')
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from package_checks import preflight,dumps
preflight(Path(__file__).resolve().parents[1])
_startup_checkpoint('package_preflight_passed')
_startup_checkpoint('before_optional_AppLauncher_import')
if os.environ.get('B4_LEGACY_RTX_OVERRIDE')=='1':
 from isaaclab.app import AppLauncher
 old=AppLauncher.__init__
 @functools.wraps(old)
 def init(self,*args,**kw):
  kw['kit_args']=(kw.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip()
  # Inspect already parsed and stored its policy/task/instruction settings.
  # Keep those arguments out of Kit's separate native command-line parser.
  saved_argv=sys.argv
  sys.argv=[saved_argv[0]]
  try:result=old(self,*args,**kw)
  finally:sys.argv=saved_argv
  if os.environ.get('B4_STARTUP_DIAGNOSTIC_ONLY')=='1':
   roots=verify_runtime()
   proof={'purpose':'STARTUP DIAGNOSTIC ONLY; no model score','native_constructor_returned':True,'runtime_pins':roots,'task_environments':0,'robot_actions':0,'model_inference':0,'main_file':sys.modules['__main__'].__file__,'main_name':sys.modules['__main__'].__name__,'main_spec_is_none':sys.modules['__main__'].__spec__ is None,'config':self._sim_app_config,'experience':self._sim_experience_file}
   (a.continuity_log.parent/'EXACT-ENTRY-STARTUP-DIAGNOSTIC-PASS.json').write_text(json.dumps(proof,indent=2)+'\n')
   _startup_checkpoint('exact_script_native_constructor_returned_stop_before_task_environment')
   os._exit(86)
  return result
 AppLauncher.__init__=init
_startup_checkpoint('before_Dream_session_import')
import inspect_robots_dream.session as session_module
_startup_checkpoint('Dream_session_imported')
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
from package_checks import dumps
_startup_checkpoint('before_embodiment_import')
from yam_arena.inspect_embodiment import YamArenaEmbodiment
_startup_checkpoint('embodiment_imported')
original_reset=YamArenaEmbodiment.reset
def checked_reset(self,*args,**kw):
 physical={'physical_seed':42,'setting':'S','goal':'stack_ABC','inspect_trial_seed':kw.get('seed',args[1] if len(args)>1 else None)}
 if type(physical['inspect_trial_seed']) is not int:raise RuntimeError('Expected integer Inspect reset seed')
 (a.continuity_log.parent/'seed-contract.json').write_text(json.dumps(physical,allow_nan=False)+'\n')
 _startup_checkpoint('actual_embodiment_reset_entered')
 return original_reset(self,*args,**kw)
YamArenaEmbodiment.reset=checked_reset
original_step=YamArenaEmbodiment.step
def checked_step(self,action):
 # Pure serialization validation of actual telemetry before every Action.
 module=sys.modules.get('kit')
 if module is None:raise RuntimeError('B4 runtime not loaded')
 dumps(module.ENGINE.status())
 return original_step(self,action)
YamArenaEmbodiment.step=checked_step
start,act=DreamSession.start,DreamSession.act
episode_started=False
_startup_checkpoint('before_continuity_log_open')
with a.continuity_log.open('x',buffering=1) as log:
 _startup_checkpoint('continuity_log_opened')
 def record(row):log.write(json.dumps(row)+'\n')
 def wrapped_start(self,*args,**kw):

  global episode_started
  if episode_started:raise RuntimeError('Episode restart prohibited')
  episode_started=True
  _startup_checkpoint('actual_Dream_episode_start_entered')
  result=start(self,*args,**kw);self._s4_session_state={'session':None,'count':0,'finished':None};record({'kind':'episode_start'});return result
 def wrapped_act(self,*args,**kw):
  state=self._s4_session_state;t=time.monotonic();chunk=act(self,*args,**kw);end=time.monotonic();ident=hashlib.sha256(chunk.session_id.encode()).hexdigest() if chunk.session_id else None;continuous=bool(ident) and (state['session'] is None or ident==state['session']);record({'kind':'chunk','index':state['count']+1,'session_sha256':ident,'continuous':continuous,'idle_s':None if state['finished'] is None else t-state['finished'],'request_s':end-t,'attempts':chunk.attempts})
  if type(chunk.attempts) is not int or chunk.attempts<1:raise RuntimeError('Invalid request attempt count')
  if not continuous:raise RuntimeError('Dream session changed/missing; episode invalid')
  state.update(session=ident,count=state['count']+1,finished=end);return chunk
 DreamSession.start,DreamSession.act=wrapped_start,wrapped_act
 if os.environ.get('B4_STARTUP_DIAGNOSTIC_ONLY')=='1':
  def _no_diagnostic_inference(*args,**kw):raise RuntimeError('Startup-only diagnostic forbids inference')
  DreamSession.act=_no_diagnostic_inference
  dream.DreamClient.get_action=_no_diagnostic_inference
  dream.client.PolicyClient.get_action=_no_diagnostic_inference
 sys.argv=['inspect-robots',*forward]
 _startup_checkpoint('before_actual_CLI_import')
 from inspect_robots.cli import main
 _startup_checkpoint('actual_CLI_imported_before_main')
 main()
