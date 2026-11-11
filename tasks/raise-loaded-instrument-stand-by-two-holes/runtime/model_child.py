"""Actual Inspect Dream entry (pattern of published Drawers model_child): installed SDK policy 'dream',
dreamlabs/dream-1-beta, rows_per_request=4, one continuous session, gripper saturation wired to the SDK station decoder."""
import argparse,hashlib,json,functools,sys,time
from pathlib import Path
R=Path(__file__).resolve().parent.parent;sys.path.insert(0,str(R/'source'));sys.path.insert(0,str(R/'runtime'))
import preflight
ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--seed',type=int,default=42);a=ap.parse_args();a.out=a.out.resolve();a.out.mkdir(parents=True,exist_ok=False)
pre=preflight.run()
import guard,overview_capture,gripper_saturation
meta=json.loads((R/'task.draft.json').read_text());instr=meta['instruction']
if '\n' in instr or '\r' in instr:raise guard.GuardViolation('instruction must be single-line (Kit argv newline bug)')
(a.out/'instruction.txt').write_text(instr)
(a.out/'registration.json').write_text(json.dumps(dict(preflight=pre,physical_case=42,object_jitter=0,inspect_seed=a.seed,seed_passed_unchanged=True,budget=guard.BUDGET,control_hz=20,policy='dream',model='dreamlabs/dream-1-beta',rows_per_request=4,policy_conversion=gripper_saturation.POLICY_LABEL,gripper_adapter_sha256=guard.ADAPTER_SHA256,recording_overview_only=True),indent=1))
overview_capture.install()
from isaaclab.app import AppLauncher
old=AppLauncher.__init__
@functools.wraps(old)
def init(self,*x,**k):k['kit_args']=(k.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip();return old(self,*x,**k)
AppLauncher.__init__=init
import inspect_robots_dream._station as station
station.rows=gripper_saturation.PublicGripperRows(station.rows,a.out/'gripper-projection.jsonl')
import inspect_robots_dream.session as sm,dream
offic=sm._retry_after
def retry(e):
 if isinstance(e,dream.DreamSessionExpired):return None
 return offic(e)
sm._retry_after=retry
from inspect_robots_dream.session import DreamSession
from yam_arena.inspect_embodiment import YamArenaEmbodiment
o_reset,o_step=YamArenaEmbodiment.reset,YamArenaEmbodiment.step;S=dict(resets=0,steps=0,ov=None,session=None,chunks=0,started=False)
def reset(self,*x,**k):
 S['resets']+=1
 if S['resets']!=1:raise RuntimeError('single episode only')
 seed=k.get('seed',x[1] if len(x)>1 else None);guard.harness_seed(seed)
 obs=o_reset(self,*x,**k);S['ov']=overview_capture.OverviewCapture(self,a.out,obs)
 (a.out/'seed-contract.json').write_text(json.dumps(dict(physical_case=42,inspect_trial_seed=seed,passed_unchanged=True)));return obs
def step(self,action):
 if S['steps']>=guard.BUDGET:raise guard.GuardViolation('4572 budget exhausted')
 S['steps']+=1;r=o_step(self,action);S['ov'].append(r.observation);return r
YamArenaEmbodiment.reset,YamArenaEmbodiment.step=reset,step
st,act=DreamSession.start,DreamSession.act
log=(a.out/'continuity.jsonl').open('x',buffering=1)
def wstart(self,*x,**k):
 if S['started']:raise RuntimeError('Dream episode restart prohibited')
 S['started']=True;return st(self,*x,**k)
def wact(self,*x,**k):
 c=act(self,*x,**k);sid=hashlib.sha256(c.session_id.encode()).hexdigest() if c.session_id else None
 ok=bool(sid) and S['session'] in (None,sid);log.write(json.dumps(dict(chunk=S['chunks']+1,session_sha256=sid,continuous=ok,attempts=c.attempts))+'\n')
 if not ok:raise RuntimeError('session replacement/missing: run fails, not 0')
 S['session']=sid;S['chunks']+=1;return c
DreamSession.start,DreamSession.act=wstart,wact
sys.argv=['inspect-robots','run','--embodiment','yam_arena','--policy','dream','-P','model=dreamlabs/dream-1-beta','-P','rows_per_request=4','-E','task='+str(R/'source/task.py'),'-E','control_hz=20','-E','device=cpu','-E','object_jitter=0','-E','cam_width=320','-E','cam_height=176','--instruction',instr,'--scorer','task_score','--max-steps',str(guard.BUDGET),'--seed',str(a.seed),'--store-frames','--no-prompt','--log-dir',str(a.out/'inspect')]
from inspect_robots.cli import main
try:main()
finally:
 if S['ov']:S['ov'].close()
 log.close()
