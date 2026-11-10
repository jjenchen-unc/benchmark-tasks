from pathlib import Path
import sys,json,hashlib
R=Path(__file__).resolve().parent;sys.path.insert(0,str(R/'candidate'))
from c1_validation import strict,sha,number,source_map,verify
C=R/'candidate';m=strict(C/'manifest.json');files={str(p.relative_to(C)):sha(p) for p in sorted(C.rglob('*')) if p.is_file() and p.name!='manifest.json' and '__pycache__' not in p.parts}
assert files==m['files'];assert m['source_fingerprint']==hashlib.sha256(''.join(n+'\0'+files[n]+'\n' for n in sorted(files)).encode()).hexdigest()
actual={str(p.relative_to(C)):sha(p) for p in C.rglob('*') if p.is_file() and '__pycache__' not in p.parts};assert actual==strict(R/'candidate-source-map.json')
for n,h in strict(R/'replay-source-hashes.json').items():assert sha(C/n)==h
actions=strict(R/'historical43748-actions.json');assert 0<len(actions)<=2268
for row in actions:
 assert type(row) is list and len(row)==14
 for v in row:number(v)
for name in ('actions.json','result.json','source-sha256.json'):
 assert sha(R/('historical43748-'+name))==strict(R/'historical43748-provenance.json')[name]

package=strict(R/'package-manifest.json')
actual_package={str(p.relative_to(R)):sha(p) for p in R.rglob('*') if p.is_file() and p.name!='package-manifest.json' and '__pycache__' not in p.parts}
assert package==actual_package,'outer package integrity mismatch'

print(json.dumps(dict(package_integrity=True,actions=len(actions),baseline_channels_enabled=verify(C),model_ready=False,submission_ready=False)))
