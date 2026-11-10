"""Single-session genuine model route; no evaluator placements/loads."""
import sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
from model_preflight import check as _model_preflight
if '--fresh-review' not in sys.argv:raise ValueError('ROOT fresh public replay review required')
_model_preflight(HERE,Path(sys.argv[sys.argv.index('--fresh-review')+1]))
from c1_launch import bootstrap
bootstrap(__file__)
import argparse,hashlib,json,time,os
from c1_json import dumps
from c1_validation import validated_channels,CHANNELS
p=argparse.ArgumentParser();p.add_argument('--fresh-review',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--public-case',action='store_true');p.add_argument('--target',default='C1-T1');p.add_argument('--seed',type=int,default=3867854786);a=p.parse_args()
from c1_launch import output_directory
output_directory(a.out)
if not a.public_case or a.target!='C1-T1':raise ValueError('Requires fixed public task')
if validated_channels()!=CHANNELS:raise RuntimeError('ROOT-approved source-bound receipt unavailable')
os.environ['C1_TARGET']='C1-T1';os.environ['C1_TRACE']=str(a.out/'measurements.jsonl')
from c1_public import INSTRUCTION
continuity_log=a.out/'continuity.jsonl'
registration={'physical_seed':42,'target':'C1-T1','policy':'dream','model':'dreamlabs/dream-1-beta','max_steps':2268,'control_hz':20,'seed':42,'rows_per_request':4,'object_jitter':0,'device':'cpu','camera_size':[320,176],'scorer':'task_score','instruction':INSTRUCTION,'inspect_reset_seed':'derived by Inspect and passed unchanged','source_sha256':json.loads((a.out/'source-sha256.json').read_text())}
(a.out/'registration.json').write_text(dumps(registration,indent=2)+'\n')
forward=['run','--embodiment','yam_arena','--policy','dream','-P','model=dreamlabs/dream-1-beta','-P','rows_per_request=4','-E','task='+str(HERE/'public_entry.py'),'-E','device=cpu','-E','object_jitter=0','-E','control_hz=20','-E','cam_width=320','-E','cam_height=176','--instruction',INSTRUCTION,'--scorer','task_score','--max-steps','2268','--seed','42','--store-frames','--log-dir',str(a.out/'model')]
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
from yam_arena.inspect_embodiment import YamArenaEmbodiment
original_reset=YamArenaEmbodiment.reset
def checked_reset(self,*args,**kw):
 physical={'physical_seed':42,'target':'C1-T1','inspect_trial_seed':kw.get('seed',args[1] if len(args)>1 else None)}
 if type(physical['inspect_trial_seed']) is not int:raise RuntimeError('Expected integer Inspect reset seed')
 (continuity_log.parent/'seed-contract.json').write_text(json.dumps(physical,allow_nan=False)+'\n')
 return original_reset(self,*args,**kw)
YamArenaEmbodiment.reset=checked_reset
original_step=YamArenaEmbodiment.step
def checked_step(self,action):
 # Pure serialization validation of actual telemetry before every Action.
 module=sys.modules.get('c1_kit')
 if module is None:raise RuntimeError('C1 runtime not loaded')
 before=module.ENGINE.status();dumps(before)
 result=original_step(self,action)
 with (a.out/'model-actions.jsonl').open('a') as stream:stream.write(dumps({'control':module.ENGINE.controls,'action14':action.data,'before':before,'after':module.ENGINE.status()})+'\n')
 return result
YamArenaEmbodiment.step=checked_step
start,act=DreamSession.start,DreamSession.act
episode_started=False
with continuity_log.open('x',buffering=1) as log:
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
