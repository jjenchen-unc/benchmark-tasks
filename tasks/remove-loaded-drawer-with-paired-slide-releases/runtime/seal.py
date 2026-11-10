"""Rebuild private input manifests/archive; never modifies task behavior or SDK."""
from pathlib import Path
import hashlib,json,tarfile
O=Path(__file__).resolve().parent;C=O/'candidate'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def write(p,v):p.write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
m=json.loads((C/'manifest.json').read_text());m.pop('consumed_sources_sha256',None);m.pop('proof_map_sha256',None);m.pop('fingerprint',None)
m['sha256']={str(p.relative_to(C)):sha(p) for p in sorted(C.rglob('*')) if p.is_file() and p.name!='manifest.json' and '__pycache__' not in p.parts}
m['provenance_consumed_source_map']='../inherited-v18-consumed-source-sha256.json'
m['provenance_consumed_source_map_sha256']=sha(O/'inherited-v18-consumed-source-sha256.json')
m['fingerprint']=hashlib.sha256(json.dumps(m,sort_keys=True,separators=(',',':')).encode()).hexdigest();write(C/'manifest.json',m)
# Historic external absolute paths are immutable provenance only, never runtime lookup.
files=[p for p in O.rglob('*') if p.is_file() and not p.is_symlink() and not any(x in p.relative_to(O).parts for x in ('work','checks','__pycache__')) and (p.parent!=O or p.name not in {'launch-input-sha256.json','full-consumed-source-sha256.json','archive-sha256.json','drawers-v19-final-validation.tgz','result.json','REPORT.md'})]
launch={str(p.relative_to(O)):sha(p) for p in sorted(files)};write(O/'launch-input-sha256.json',launch)
write(O/'full-consumed-source-sha256.json',{'portable_inputs':launch,'historical_consumed_map':'inherited-v18-consumed-source-sha256.json','historical_consumed_map_sha256':sha(O/'inherited-v18-consumed-source-sha256.json'),'historical_absolute_paths':'PROVENANCE_ONLY; not runtime dependencies; original SDK/mesh proofs reused','targeted_runtime_source_pins':json.loads((O/'runtime-source-pins.json').read_text())})
with tarfile.open(O/'drawers-v19-final-validation.tgz','w:gz') as t:
 for p in sorted([*files,O/'launch-input-sha256.json',O/'full-consumed-source-sha256.json',O/'REPORT.md',O/'result.json']):
  if p.is_file():t.add(p,arcname='drawers-v19/'+str(p.relative_to(O)),recursive=False)
write(O/'archive-sha256.json',{'drawers-v19-final-validation.tgz':sha(O/'drawers-v19-final-validation.tgz')})
print(json.dumps({'portable_files':len(launch),'candidate_files':len(m['sha256']),'fingerprint':m['fingerprint']}))
