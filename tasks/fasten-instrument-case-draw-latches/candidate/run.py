"""Immutable copied task executes BEFORE cold SDK imports. Requires allocated RTX GPU.
Modes foundation/reference/evaluator are distinct. Never calls a model.
"""
import argparse,hashlib,json,os,runpy,shutil,sys,traceback
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--mode',choices=['foundation','reference','evaluator'],default='foundation');p.add_argument('--legacy-rtx-override',action='store_true');a=p.parse_args()
a.out=a.out.resolve();a.out.mkdir(parents=True,exist_ok=False);source=Path(__file__).resolve().parent;target=a.out/'source';target.mkdir();sys.dont_write_bytecode=True
for f in source.rglob('*'):
 if f.is_file() and '__pycache__' not in f.parts and f.suffix!='.pyc':
  dest=target/f.relative_to(source);dest.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(f,dest);dest.chmod(0o444)
hashes={str(f.relative_to(target)):hashlib.sha256(f.read_bytes()).hexdigest() for f in target.rglob('*') if f.is_file()};(a.out/'sourcehashes.json').write_text(json.dumps(hashes,indent=2)+'\n');sys.path.insert(0,str(target));emb=None
manifest=json.loads((target/'snapshot-manifest.json').read_text())
assert all(hashes.get(n)==h for n,h in manifest['source_files'].items()), 'frozen source hash mismatch'
try:
 # Actual execution of the frozen task, never import the mutable author tree.
 from yam_arena.task_file import load as load_task_file
 task_file=load_task_file(target/'task.py');loaded={'OBJECTS':task_file.objects,'SCENE':task_file.scene,'__doc__':task_file.docs};(a.out/'bootstrap.json').write_text(json.dumps({'executed_task':str(target/'task.py'),'sha256':hashes['task.py'],'before_inspect_import':True,'object_count':len(loaded['OBJECTS']),'scene':loaded['SCENE']},indent=2))
 if a.legacy_rtx_override:
  from isaaclab.app import AppLauncher
  init0=AppLauncher.__init__
  def init(self,*args,**kw):kw['kit_args']=(kw.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip();return init0(self,*args,**kw)
  AppLauncher.__init__=init
 from inspect_robots import Scene
 from inspect_robots.types import Action
 from yam_arena.inspect_embodiment import yam_arena_embodiment
 import numpy as np
 from PIL import Image
 import kit
 from first_cause import write_snapshot, strict_json, count_substeps
 from settling_diagnostic import install as install_diagnostic
 if a.mode=='foundation':install_diagnostic(a.out)
 os.environ['S2_EVALUATOR_DIAGNOSTICS']='1'
 emb=yam_arena_embodiment(task=str(target/'task.py'),device='cpu',object_jitter=0,control_hz=20,cam_width=320,cam_height=176)
 report={'mode':a.mode,'model_calls':0,'physics_verified':False,'checks':{},'rounds':[]}
 if a.mode=='foundation':
  for episode,seed in enumerate([42]):
   obs=emb.reset(Scene(id=f's2-foundation-{episode}',instruction=loaded['__doc__']),seed=seed);E=kit.ENGINE;initial=E.status();q=np.array(obs.state['joint_pos'],float)
   for name,rgb in obs.images.items():Image.fromarray(rgb).save(a.out/f'episode-{episode}-{name}.png')
   # Serialize actual reset telemetry, including a real NumPy measurement-shaped row,
   # before constructing any Action. A failed preflight cannot consume a control.
   count_substeps(emb)
   write_snapshot(a.out/f'episode-{episode}-pre-action.json',E,emb,q,phase='pre_action',control=0,result=None)
   for control in range(1,2):
    try:
     result=emb.step(Action(data=q.copy()))
    except BaseException as exc:
     write_snapshot(a.out/'first-failure.json',E,emb,q,phase='step_exception',control=control,result=None,error=exc)
     raise
    write_snapshot(a.out/f'episode-{episode}-control-{control:02d}.json',E,emb,q,phase='control_return',control=control,result=result)
    if result.terminated:
     write_snapshot(a.out/'first-termination.json',E,emb,q,phase='first_termination',control=control,result=result)
     # No next action or reset. finally closes only after the evidence is durable.
     raise RuntimeError('S2 first termination captured; see first-termination.json')
   report['checks']['single_control_captured']=True
 elif a.mode=='reference':
  from reference import Reference
  obs=emb.reset(Scene(id='s2-reference',instruction=loaded['__doc__']),seed=42);report['reference']=Reference(emb,obs,a.out).run()
 elif a.mode=='evaluator':
  from evaluator import run
  report['evaluator']=run(emb,a.out)
 report['all_checks_passed']=bool(report['checks']) and all(report['checks'].values());report['physics_verified']=False;report['scope']='Root-run foundation only; no force/capture/goal/model validation is inferred from neutral execution.'
 (a.out/'result.json').write_text(strict_json(report)+'\n')
 if a.mode=='foundation' and not report['all_checks_passed']:raise RuntimeError('Foundation checks failed')
except BaseException as exc:
 (a.out/'error.json').write_text(json.dumps({'type':type(exc).__name__,'message':str(exc),'traceback':traceback.format_exc()},indent=2)+'\n');raise
finally:
 # Freeze actual loaded module files as well as the already frozen task inputs.
 consumed={str(Path(m.__file__).resolve()):hashlib.sha256(Path(m.__file__).read_bytes()).hexdigest() for m in list(sys.modules.values()) if getattr(m,'__file__',None) and Path(m.__file__).is_file()}
 (a.out/'consumed-modules-sha256.json').write_text(json.dumps(consumed,sort_keys=True,indent=2,allow_nan=False)+'\n')
 if emb:emb.close()
