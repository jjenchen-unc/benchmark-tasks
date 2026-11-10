"""Freeze before importing task/SDK modules; process-only proven RTX override."""
import functools,hashlib,json,os,runpy,shutil,sys
from pathlib import Path

def bootstrap(runner):
 source=Path(runner).resolve().parent
 try:out=Path(sys.argv[sys.argv.index('--out')+1]).resolve()
 except (ValueError,IndexError):return # argparse supplies the usage error
 marker=os.environ.get('C1_FROZEN_SOURCE')
 if marker:
  if source!=Path(marker).resolve():raise RuntimeError('Refusing live-source execution in a frozen run')
  expected=json.loads((out/'source-sha256.json').read_text())
  actual={str(p.relative_to(source)):hashlib.sha256(p.read_bytes()).hexdigest() for p in source.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
  if actual!=expected:raise RuntimeError('Frozen source hash mismatch')
  return
 out.mkdir(parents=True,exist_ok=False);frozen=out/'source-frozen'
 shutil.copytree(source,frozen,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
 hashes={str(p.relative_to(frozen)):hashlib.sha256(p.read_bytes()).hexdigest() for p in frozen.rglob('*') if p.is_file()}
 manifest=json.loads((frozen/'manifest.json').read_text())
 for name,h in manifest['files'].items():
  if hashes.get(name)!=h:raise RuntimeError('Frozen copy differs from source manifest: '+name)
 (out/'source-sha256.json').write_text(json.dumps(hashes,indent=2)+'\n')
 (out/'launch.json').write_text(json.dumps({'source_frozen':str(frozen),'runner':Path(runner).name,'process_only_RTX_driver_check_override':True,'host_driver_or_SDK_changed':False},indent=2)+'\n')
 os.environ['C1_STARTUP_TRACE']=str(out/'startup-stages.jsonl')
 os.environ['C1_FROZEN_SOURCE']=str(frozen);sys.path.insert(0,str(frozen));sys.modules.pop('c1_launch',None)
 # Model preflight loaded pure task helpers before freeze. Reload their frozen origins.
 for name in ('c1_validation','model_preflight'):sys.modules.pop(name,None)
 # This import precedes all task/scene/action imports, matching the validated
 # UNC AppLauncher pattern. It changes only this process's Kit argument.
 from yam_arena.task_file import load
 loaded=load(frozen/('public_entry.py' if '--public-case' in sys.argv else 'task.py'))
 (out/'task-construction.json').write_text(json.dumps({'objects':len(loaded.objects),'scene':loaded.scene,'task_sha256':loaded.sha256})+'\n')
 from isaaclab.app import AppLauncher
 original=AppLauncher.__init__
 @functools.wraps(original)
 def init(self,*args,**kwargs):
  kwargs['kit_args']=(kwargs.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip()
  return original(self,*args,**kwargs)
 AppLauncher.__init__=init
 runpy.run_path(str(frozen/Path(runner).name),run_name='__main__');raise SystemExit(0)

def output_directory(out):
 out=Path(out)
 if os.environ.get('C1_FROZEN_SOURCE'):
  if not(out/'source-sha256.json').is_file():raise RuntimeError('Missing frozen source receipt')
 else:raise RuntimeError('Runtime must execute from its verified frozen source copy')
 return out
