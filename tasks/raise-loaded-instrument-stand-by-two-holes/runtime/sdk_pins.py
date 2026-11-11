from pathlib import Path
import hashlib,importlib.util,json
def verify():
 pins=json.loads(Path(__file__).with_name('sdk-source-pins.json').read_text());count=0
 for name,files in pins.items():
  spec=importlib.util.find_spec(name)
  if not spec or not spec.submodule_search_locations:raise RuntimeError('missing SDK package '+name)
  root=Path(next(iter(spec.submodule_search_locations)))
  for rel,digest in files.items():
   p=root/rel
   if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=digest:raise RuntimeError('SDK source mismatch '+name+'/'+rel)
   count+=1
 return count
