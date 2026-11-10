"""Official Dream policy with per-episode server-session continuity checks."""
import argparse,functools,hashlib,json,sys,time,os
from pathlib import Path
p=argparse.ArgumentParser(add_help=False);p.add_argument('--continuity-log',type=Path,required=True);a,forward=p.parse_known_args()
if os.environ.get('S4_DRIVER_CHECK_OVERRIDE')=='1':
 from isaaclab.app import AppLauncher
 old=AppLauncher.__init__
 @functools.wraps(old)
 def init(self,*args,**kw):kw['kit_args']=(kw.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip();return old(self,*args,**kw)
 AppLauncher.__init__=init
import inspect_robots_dream.session as session_module
# Fail on the first refusal; no retry, fallback or reset recovery.
session_module._retry_after=lambda error: None
from inspect_robots_dream.session import DreamSession
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from package_checks import dumps
from yam_arena.inspect_embodiment import YamArenaEmbodiment
original_step=YamArenaEmbodiment.step
def checked_step(self,action):
 # Pure serialization validation of actual telemetry before every Action.
 module=sys.modules.get('kit')
 if module is None:raise RuntimeError('S4 runtime not loaded')
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
  if chunk.attempts!=1:raise RuntimeError('Retried request invalid')
  if not continuous:raise RuntimeError('Dream session changed/missing; episode invalid')
  state.update(session=ident,count=state['count']+1,finished=end);return chunk
 DreamSession.start,DreamSession.act=wrapped_start,wrapped_act
 sys.argv=['inspect-robots',*forward]
 from inspect_robots.cli import main
 main()
