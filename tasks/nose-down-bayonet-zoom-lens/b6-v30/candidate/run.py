"""Freeze and execute copied task before cold SDK imports; no model/API calls.
Run on an explicitly allocated GPU node for RTX, CPU PhysX. Never run on login.
"""
from pathlib import Path
import argparse,hashlib,json,os,runpy,shutil,sys,time,traceback
sys.dont_write_bytecode=True
HERE=Path(__file__).resolve().parent

def save(p,obj):p.write_text(json.dumps(obj,indent=2,allow_nan=False)+'\n')
def manifest(root):return {str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in root.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--mode',choices=['foundation','reference','negative'],default='foundation');ap.add_argument('--case',default='unlocked');ap.add_argument('--rtx-driver-check-override',action='store_true');a=ap.parse_args()
 a.out=a.out.resolve();a.out.mkdir(parents=True,exist_ok=False);save(a.out/'STATUS.json',dict(state='preparing_snapshot',physics_verified=False,model_calls=0));save(a.out/'result.json',dict(state='incomplete',full_goal_verified=False,physics_verified=False,model_calls=0))
 source=a.out/'source';shutil.copytree(HERE,source,ignore=shutil.ignore_patterns('__pycache__','*.pyc'));expected=manifest(source);save(a.out/'sourcehashes.json',expected)
 try:
  forbidden={'yam_arena.scene','yam_arena.actions','yam_arena.inspect_embodiment'}&sys.modules.keys()
  if forbidden:raise RuntimeError('Cold source execution required; already loaded: '+str(forbidden))
  sys.path.insert(0,str(source));runpy.run_path(str(source/'task.py'),run_name='b6_frozen_task_bootstrap')
  import kit
  if Path(kit.__file__).resolve().parent!=source:raise RuntimeError('Copied source was not the executed source')
  save(a.out/'bootstrap.json',dict(executed_task=str(source/'task.py'),kit=str(kit.__file__),cold_sdk_modules=True,sourcehashes=expected))
  if a.rtx_driver_check_override:
   from isaaclab.app import AppLauncher
   previous=AppLauncher.__init__
   def launch(self,*args,**kw):
    kw['kit_args']=(kw.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip();return previous(self,*args,**kw)
   AppLauncher.__init__=launch
  os.environ['B6_TRACE']=str(a.out/'physics.jsonl')
  driver=__import__({'foundation':'foundation_driver','reference':'reference','negative':'negative_driver'}[a.mode]);result=driver.run(a.out,a.case)
  result.update(source_unchanged=manifest(source)==expected,model_calls=0,media_verified=False);save(a.out/'result.json',result);save(a.out/'STATUS.json',dict(state='complete',mode=a.mode,full_goal_verified=result.get('full_goal_verified',False),model_calls=0))
 except BaseException as e:
  save(a.out/'result.json',dict(state='failed',error=type(e).__name__+': '+str(e),traceback=traceback.format_exc(),source_unchanged=manifest(source)==expected,physics_verified=False,full_goal_verified=False,model_calls=0));save(a.out/'STATUS.json',dict(state='failed',error=type(e).__name__+': '+str(e)));raise
if __name__=='__main__':main()
