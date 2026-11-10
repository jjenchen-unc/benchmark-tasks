"""Targeted package source pinning only. No imports of policy/CLI/simulator/provider."""
from pathlib import Path
import importlib.metadata as metadata,hashlib,json,sys,argparse
O=Path(__file__).resolve().parent
NAMES=('inspect-robots','inspect-robots-dream','yam-arena')
def snapshot():
 result={}
 for name in NAMES:
  d=metadata.distribution(name);files={}
  for f in d.files or []:
   s=str(f)
   if s.endswith('.py') and '..' not in Path(s).parts:
    p=Path(d.locate_file(f))
    if p.is_file():files[s]=hashlib.sha256(p.read_bytes()).hexdigest()
  # Editable YAM metadata may list only .pth; resolve its known top-level source
  # via find_spec without importing the package or initializing anything.
  if name=='yam-arena' and not any('yam_arena/' in f for f in files):
   import importlib.util
   spec=importlib.util.find_spec('yam_arena');root=Path(next(iter(spec.submodule_search_locations)))
   files={str(Path('yam_arena')/p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in root.rglob('*.py')}
  result[name]={'version':d.version,'sha256':files}
 return result
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--freeze',action='store_true');a=p.parse_args();r=snapshot()
 if a.freeze:(O/'runtime-source-pins.json').write_text(json.dumps(r,indent=2)+'\n')
 else:
  if r!=json.loads((O/'runtime-source-pins.json').read_text()):raise SystemExit('Existing runtime sources differ; ROOT review required, no install/retry')
 print(json.dumps({'status':'CPU_SOURCE_PINS_ONLY','packages':{k:len(v['sha256']) for k,v in r.items()},'model_calls':0,'simulation_starts':0}))
