"""Prepared actual Inspect entry. Never run before user opens Brev and ROOT review."""
from pathlib import Path
import sys,os,argparse,json,hashlib,functools
O=Path(__file__).resolve().parent;sys.path.insert(0,str(O));sys.path.insert(0,str(O/'candidate'))
from protocol import receipt,check
from strict_json import dumps
p=argparse.ArgumentParser();p.add_argument('--review',required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--legacy-rtx-override',action='store_true');a=p.parse_args()
r=receipt(a.review);m=check();os.environ['S2_ROOT_REVIEW']=str(Path(a.review).resolve())
if os.environ.get('S2_EVALUATOR_DIAGNOSTICS'):raise ValueError('model observation diagnostics must be unset')
from yam_arena.task_file import load
t=load(O/'model_task.py');a.out=a.out.resolve();a.out.mkdir(parents=True,exist_ok=False)
(a.out/'registration.json').write_text(dumps({'manifest':m,'physical_seed':42,'budget':7964,'control_hz':20,'instruction':t.docs,'ROOT_review_sha256':hashlib.sha256(Path(a.review).read_bytes()).hexdigest()},indent=2)+'\n')
from isaaclab.app import AppLauncher
if a.legacy_rtx_override:
 old=AppLauncher.__init__
 @functools.wraps(old)
 def init(self,*args,**kw):
  kw['kit_args']=(kw.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip();return old(self,*args,**kw)
 AppLauncher.__init__=init
from yam_arena.inspect_embodiment import YamArenaEmbodiment
old_reset,old_step=YamArenaEmbodiment.reset,YamArenaEmbodiment.step
resets=0
def reset(self,*args,**kw):
 global resets
 resets+=1
 if resets!=1:raise RuntimeError('single episode; no reset recovery')
 seed=kw.get('seed',args[1] if len(args)>1 else None)
 if type(seed) is not int:raise ValueError('integer harness seed')
 obs=old_reset(self,*args,**kw)
 (a.out/'seed-contract.json').write_text(dumps({'inspect_seed':seed,'physical_seed':42,'passed_unchanged':True})+'\n');return obs
def step(self,action):
 result=old_step(self,action)
 import kit
 from first_cause import numeric
 with (a.out/'actions.jsonl').open('a') as f:f.write(dumps(numeric({'action':action.data,'status':kit.ENGINE.status(),'last_row':kit.ENGINE.history.last}))+'\n')
 return result
YamArenaEmbodiment.reset,YamArenaEmbodiment.step=reset,step
# Provider import is deliberately below complete physical receipt validation.
import inspect_robots_dream.session as sessions
sessions._retry_after=lambda error:None
from inspect_robots_dream.session import DreamSession
start,act=DreamSession.start,DreamSession.act;started=False;session_id=None
def begin(self,*args,**kw):
 global started
 if started:raise RuntimeError('single continuous model episode')
 started=True;return start(self,*args,**kw)
def action(self,*args,**kw):
 global session_id
 chunk=act(self,*args,**kw)
 ident=hashlib.sha256(chunk.session_id.encode()).hexdigest() if chunk.session_id else None
 if not ident or type(chunk.attempts) is not int or chunk.attempts!=1 or session_id not in (None,ident):raise RuntimeError('model retry/session discontinuity')
 session_id=ident
 with (a.out/'continuity.jsonl').open('a') as f:f.write(dumps({'session_sha256':ident,'attempts':chunk.attempts})+'\n')
 return chunk
DreamSession.start,DreamSession.act=begin,action
sys.argv=['inspect-robots','run','--embodiment','yam_arena','--policy','dream','-P','model=dreamlabs/dream-1-beta','-P','rows_per_request=4','-E','task='+str(O/'model_task.py'),'-E','device=cpu','-E','control_hz=20','-E','object_jitter=0','-E','cam_width=320','-E','cam_height=176','--instruction',t.docs,'--scorer','task_score','--max-steps','7964','--seed','42','--store-frames','--no-prompt','--log-dir',str(a.out/'inspect')]
from inspect_robots.cli import main
main()
(a.out/'source-after.json').write_text(dumps(check(),indent=2)+'\n')
