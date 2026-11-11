"""Reuse targeted unchanged installed API pins; no SDK/provider import or full scan."""
from pathlib import Path
import importlib.metadata as md,json,hashlib
O=Path(__file__).resolve().parent
def verify():
 pins=json.loads((O/'runtime-source-pins.json').read_text());count=0
 for name,row in pins.items():
  dist=md.distribution(name)
  if dist.version!=row['version']:raise ValueError('runtime version mismatch '+name)
  for rel,expected in row['sha256'].items():
   # Editable YAM sources use original saved absolute paths; remap by installed
   # package metadata, not historical server task roots.
   p=Path(dist.locate_file(rel))
   if not p.is_file():
    import importlib.util
    package=rel.split('/')[0];spec=importlib.util.find_spec(package)
    if not spec or not spec.submodule_search_locations:raise ValueError('installed source missing '+rel)
    p=Path(next(iter(spec.submodule_search_locations))).joinpath(*Path(rel).parts[1:])
   if hashlib.sha256(p.read_bytes()).hexdigest()!=expected:raise ValueError('runtime API source mismatch '+rel)
   count+=1
 return {'passed':True,'targeted_sources':count,'whole_sdk_scan':False,'sdk_imported':False}
if __name__=='__main__':print(json.dumps(verify()))
