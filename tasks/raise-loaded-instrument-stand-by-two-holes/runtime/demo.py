"""Native scripted demo: frozen Teacher(GuardedEmbodiment,obs,record); success only via cached actual Result."""
import argparse,functools,json,os,runpy,sys
from pathlib import Path
R=Path(__file__).resolve().parent.parent;sys.path.insert(0,str(R/'source'));sys.path.insert(0,str(R/'runtime'))
import preflight
ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--seed',type=int,default=42);a=ap.parse_args();a.out=a.out.resolve();a.out.mkdir(parents=True,exist_ok=False)
pre=preflight.run();(a.out/'preflight.json').write_text(json.dumps(pre,indent=1))
os.environ['PINS_TRACE']=str(a.out/'measurements.jsonl');os.environ['PINS_ACTION_TRACE']=str(a.out/'robot-actions.jsonl')
from isaaclab.app import AppLauncher
_init=AppLauncher.__init__
@functools.wraps(_init)
def _driver(self,*args,**kw):
 kw['kit_args']=(kw.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip();return _init(self,*args,**kw)
AppLauncher.__init__=_driver
import guard,overview_capture
overview_capture.install()
from inspect_robots import Scene
from yam_arena.inspect_embodiment import yam_arena_embodiment
import kit
from reference import Teacher
emb=guard.GuardedEmbodiment(yam_arena_embodiment(task=str(R/'source/task.py'),device='cpu',object_jitter=0,control_hz=20,cam_width=320,cam_height=176))
obs=emb.reset(Scene(id=f'pins-demo-{a.seed}',instruction=json.loads((R/'task.draft.json').read_text())['instruction']),seed=guard.harness_seed(a.seed))
ov=overview_capture.OverviewCapture(emb,a.out,obs)
st=emb._emb.step
def step(action):r=st(action);ov.append(r.observation);return r
emb._emb.step=step
rec=[];t=Teacher(emb,obs,rec)
try:res=guard.run_reference(t,lambda tt:tt.run(),lambda:kit.ENGINE.status()['score'])
finally:
 t.pair_diagnostic.close();ov.close();emb._emb.close()
 (a.out/'demo-status.json').write_text(json.dumps(kit.ENGINE.status() if kit.ENGINE else None,default=float,indent=1))
(a.out/'demo-result.json').write_text(json.dumps(res,indent=1));print(json.dumps(res))
