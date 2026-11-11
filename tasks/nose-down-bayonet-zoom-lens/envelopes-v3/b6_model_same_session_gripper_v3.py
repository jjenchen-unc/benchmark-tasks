"""Actual Inspect Dream entry; validates credit before importing simulator/provider."""
from pathlib import Path
import sys,os,argparse,time,hashlib,json,functools
O=Path(os.environ['B6_FROZEN_BUNDLE']).resolve();C=O/'candidate';sys.path.insert(0,str(O));sys.path.insert(0,str(C))
from protocol import receipt as validate_model,check
from strict_json import dumps
p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--review',required=True);a=p.parse_args()
review=validate_model(a.review);check()
# Versioned task-local policy conversion; no installed SDK or physical source edit.
import importlib.util
ADAPTER=Path(__file__).resolve().with_name('gripper_saturation.py')
ADAPTER_SHA256='b525f6c889d0accbfa7ac3183371150138d4b717a6ea5814f45e4dc2f8eeac06'
if hashlib.sha256(ADAPTER.read_bytes()).hexdigest()!=ADAPTER_SHA256:raise ValueError('frozen shared gripper adapter changed')
_spec=importlib.util.spec_from_file_location('_b6_public_gripper_saturation',ADAPTER);_adapter=importlib.util.module_from_spec(_spec);_spec.loader.exec_module(_adapter)
from verify_runtime import verify
verify()
os.environ['B6_ROOT_REVIEW']=str(Path(a.review).resolve())
os.environ['B6_TRACE']=str(a.out.resolve()/'physics.jsonl')
from yam_arena.task_file import load
loaded=load(O/'model_task.py');second=load(O/'model_task.py');task_module=loaded.module
assert loaded.objects==second.objects
sys.audit('b6.model_task_constructed',str(loaded.path))
meta=json.loads((C/'task.draft.json').read_text())
from isaaclab.app import AppLauncher
old=AppLauncher.__init__
@functools.wraps(old)
def init(self,*args,**kw):
 kw['kit_args']=(kw.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip();return old(self,*args,**kw)
AppLauncher.__init__=init
a.out=a.out.resolve();a.out.mkdir(parents=True,exist_ok=False)
meta=json.loads((C/'task.draft.json').read_text())
(a.out/'registration.json').write_text(dumps({'physical_case':'B6-nose_down_zoom','physical_seed':42,'inspect_trial_seed':'recorded at reset, passed unchanged','budget':3034,'control_hz':20,'instruction':meta['instruction'],'policy':'dream','model':'dreamlabs/dream-1-beta','rows_per_request':4,'policy_conversion':_adapter.POLICY_LABEL,'gripper_adapter_sha256':ADAPTER_SHA256,'review_sha256':hashlib.sha256(Path(a.review).resolve().read_bytes()).hexdigest(),'source_manifest':check(),'credit':review},indent=2)+'\n')

(a.out/'external-model-envelope.json').write_text(dumps({'kind':'B6_external_same_session_transport_gripper_v3','wrapper_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'original_model_child_sha256':hashlib.sha256((O/'model_child.py').read_bytes()).hexdigest(),'source_manifest':check(),'gripper_adapter_sha256':ADAPTER_SHA256,'policy_conversion':_adapter.POLICY_LABEL,'scope':'official transient retries within a single continuous session only; expiry/newsession/restart forbidden; policy checkpoint/instruction3034budget/cameras/scorer unchanged; all finite public grippers projected to existing[0,1], allarmvalues/rows/order preserved via originaldecoder'},indent=2)+'\n')
import inspect_robots_dream.session as session_module
import inspect_robots_dream._station as station
if hashlib.sha256(Path(station.__file__).read_bytes()).hexdigest()!='0b24ccab52cabf8e14279b599c2173bed25f16c1a3d9656d8d739886b27fc0c7':raise ValueError('original SDK station decoder changed')
station.rows=_adapter.PublicGripperRows(station.rows,a.out/'gripper-projection.jsonl')
# Preserve official retryable timeout/protocol/warming handling, but forbid
# SDK expiry recovery because it allocates a different provider session.
import dream
SDK_retry_after=session_module._retry_after
def same_session_retry(error):
 if isinstance(error,dream.DreamSessionExpired):return None
 return SDK_retry_after(error)
session_module._retry_after=same_session_retry
from inspect_robots_dream.session import DreamSession
from yam_arena.inspect_embodiment import YamArenaEmbodiment
original_reset=YamArenaEmbodiment.reset;original_step=YamArenaEmbodiment.step
reset_count=0

def checked_reset(self,*args,**kw):
 global reset_count
 reset_count+=1
 if reset_count!=1:raise RuntimeError('single episode only; no reset recovery')
 seed=kw.get('seed',args[1] if len(args)>1 else None)
 if type(seed) is not int:raise ValueError('integer Inspect seed required')
 obs=original_reset(self,*args,**kw)
 import kit
 (a.out/'reset-status.json').write_text(dumps(kit.ENGINE.status())+'\n')
 (a.out/'seed-contract.json').write_text(dumps({'physical_seed':42,'inspect_trial_seed':seed,'passed_unchanged':True})+'\n')
 return obs

def checked_step(self,action):
 import kit
 result=original_step(self,action)
 with (a.out/'action-journal.jsonl').open('a') as f:f.write(dumps({'action':action.data,'status':kit.ENGINE.status(),'physics':kit.ENGINE.last})+'\n')
 return result
YamArenaEmbodiment.reset=checked_reset;YamArenaEmbodiment.step=checked_step
start,act=DreamSession.start,DreamSession.act;episode_started=False
with (a.out/'continuity.jsonl').open('x',buffering=1) as log:
 def record(row):log.write(dumps(row)+'\n')
 def wrapped_start(self,*args,**kw):
  global episode_started
  if episode_started:raise RuntimeError('Dream episode restart prohibited')
  episode_started=True;result=start(self,*args,**kw);self._b6_state={'session':None,'count':0,'finished':None};record({'kind':'episode_start'});return result
 def wrapped_act(self,*args,**kw):
  state=self._b6_state;t=time.monotonic();started_unix=time.time()
  try:chunk=act(self,*args,**kw)
  except BaseException as error:
   record({'kind':'request_failure','request_started_unix_s':started_unix,'finished_unix_s':time.time(),'request_s':time.monotonic()-t,'error_class':type(error).__name__,'cause_class':type(error.__cause__).__name__ if error.__cause__ else None,'last_session_sha256':state['session'],'completed_chunks':state['count']})
   raise
  end=time.monotonic();ident=hashlib.sha256(chunk.session_id.encode()).hexdigest() if chunk.session_id else None
  continuous=bool(ident) and (state['session'] is None or ident==state['session'])
  record({'kind':'chunk','index':state['count']+1,'session_sha256':ident,'continuous':continuous,'idle_s':None if state['finished'] is None else t-state['finished'],'request_s':end-t,'attempts':chunk.attempts,'rows':len(chunk.rows)})
  if type(chunk.attempts) is not int or chunk.attempts<1 or not continuous:raise RuntimeError('retry/session discontinuity invalid')
  state.update(session=ident,count=state['count']+1,finished=end);return chunk
 DreamSession.start,DreamSession.act=wrapped_start,wrapped_act
 sys.argv=['inspect-robots','run','--embodiment','yam_arena','--policy','dream','-P','model=dreamlabs/dream-1-beta','-P','rows_per_request=4','-E','task='+str(O/'model_task.py'),'-E','control_hz=20','-E','device=cpu','-E','object_jitter=0','-E','cam_width=320','-E','cam_height=176','--instruction',meta['instruction'],'--scorer','task_score','--max-steps','3034','--seed','42','--store-frames','--no-prompt','--log-dir',str(a.out/'inspect')]
 from inspect_robots.cli import main
 main()

(a.out/'source-after.json').write_text(dumps(check(),indent=2)+'\n')
