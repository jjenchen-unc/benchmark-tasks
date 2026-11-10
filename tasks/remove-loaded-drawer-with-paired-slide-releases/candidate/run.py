"""Cold-start exact snapshot runner. Foundation requires allocated GPU; never on login."""
from pathlib import Path
import argparse,sys,json,hashlib,shutil,runpy,traceback,time
ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--mode',choices=['foundation','reference'],default='foundation');ap.add_argument('--legacy-rtx-override',action='store_true');ap.add_argument('--device',default='cpu');a=ap.parse_args()
HERE=Path(__file__).resolve().parent;a.out=a.out.resolve();a.out.mkdir(parents=True,exist_ok=False);frozen=a.out/'source';shutil.copytree(HERE,frozen,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
hashes={str(p.relative_to(frozen)):hashlib.sha256(p.read_bytes()).hexdigest() for p in frozen.rglob('*') if p.is_file()};(a.out/'sourcehashes.json').write_text(json.dumps(hashes,indent=2)+'\n');sys.path.insert(0,str(frozen));started=time.monotonic();emb=None;stage='cold_snapshot_task_execution'
def serial(x):
 if hasattr(x,'tolist'):return x.tolist()
 raise TypeError(type(x).__name__)
def save(name,obj):(a.out/name).write_text(json.dumps(obj,indent=2,default=serial)+'\n')
try:
 if any(n in sys.modules for n in ('yam_arena.scene','yam_arena.actions','yam_arena.inspect_embodiment')):raise RuntimeError('SDK already imported before immutable task execution')
 runpy.run_path(str(frozen/'task.py'),run_name='immutable_drawers_task')
 save('cold-execution.json',{'executed':str(frozen/'task.py'),'sha256':hashes['task.py'],'before_cold_sdk':True})
 if a.legacy_rtx_override:
  from isaaclab.app import AppLauncher
  original=AppLauncher.__init__
  def init(self,*args,**kwargs):kwargs['kit_args']=(kwargs.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip();return original(self,*args,**kwargs)
  AppLauncher.__init__=init
 from inspect_robots import Scene
 from yam_arena.inspect_embodiment import yam_arena_embodiment
 stage='create_embodiment';emb=yam_arena_embodiment(task=str(frozen/'task.py'),device=a.device,object_jitter=0.,control_hz=20,cam_width=320,cam_height=176)
 stage='reset';obs=emb.reset(Scene(id='drawers-'+a.mode,instruction='Take the loaded red-tab drawer completely out of its chest and set it level on the table, box retained, jars intact, hands off for one second.'),seed=42)
 def record(label,obs,result):
  import drawers_kit as kit
  row={'stage':label,'physics':kit.ENGINE.last,'status':kit.ENGINE.status(),'joint_pos':obs.state['joint_pos'],'termination':result.termination_reason}
  with (a.out/'journal.jsonl').open('a') as f:f.write(json.dumps(row,default=serial)+'\n')
  save('progress.json',{'stage':label,'elapsed_s':time.monotonic()-started,'status':kit.ENGINE.status()})
 stage=a.mode
 if a.mode=='foundation':from evaluator import run
 else:from reference import run
 try:result=run(emb,obs,record)
 except Exception as exc:
  from reference import FullGoalReached
  if a.mode!='reference' or not isinstance(exc,FullGoalReached):raise
  import drawers_kit as kit
  result=kit.ENGINE.status()
 save('result.json',{'status':'completed_candidate_driver','mode':a.mode,'result':result,'physics_verified':False,'full_goal_verified':False,'model_score':None,'media':None})
except Exception as exc:
 save('result.json',{'status':'failed','mode':a.mode,'stage':stage,'error':str(exc),'traceback':traceback.format_exc(),'physics_verified':False,'full_goal_verified':False,'model_score':None,'media':None});raise
finally:
 if emb is not None:emb.close()
