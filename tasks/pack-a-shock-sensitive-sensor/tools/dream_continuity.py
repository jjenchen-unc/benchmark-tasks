"""Official Dream policy with per-episode server-session continuity checks."""
import argparse,functools,hashlib,json,sys,time,os
from pathlib import Path
p=argparse.ArgumentParser(add_help=False);p.add_argument('--continuity-log',type=Path,required=True);a,forward=p.parse_known_args()
if os.environ.get('D3_DRIVER_CHECK_OVERRIDE')=='1':
 from isaaclab.app import AppLauncher
 old=AppLauncher.__init__
 @functools.wraps(old)
 def init(self,*args,**kw):kw['kit_args']=(kw.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip();return old(self,*args,**kw)
 AppLauncher.__init__=init
from inspect_robots_dream.session import DreamSession
start,act=DreamSession.start,DreamSession.act
with a.continuity_log.open('x',buffering=1) as log:
 def record(row):log.write(json.dumps(row)+'\n')
 def wrapped_start(self,*args,**kw):
  result=start(self,*args,**kw);self._d3_session_state={'session':None,'count':0,'finished':None};record({'kind':'episode_start'});return result
 def wrapped_act(self,*args,**kw):
  state=self._d3_session_state;t=time.monotonic();chunk=act(self,*args,**kw);end=time.monotonic();ident=hashlib.sha256(chunk.session_id.encode()).hexdigest() if chunk.session_id else None;continuous=bool(ident) and (state['session'] is None or ident==state['session']);record({'kind':'chunk','index':state['count']+1,'session_sha256':ident,'continuous':continuous,'idle_s':None if state['finished'] is None else t-state['finished'],'request_s':end-t,'attempts':chunk.attempts})
  if not continuous:raise RuntimeError('Dream session changed/missing; episode invalid')
  state.update(session=ident,count=state['count']+1,finished=end);return chunk
 DreamSession.start,DreamSession.act=wrapped_start,wrapped_act
 sys.argv=['inspect-robots',*forward]
 from inspect_robots.cli import main
 main()
