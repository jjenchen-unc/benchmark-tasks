"""Seal only package inputs; no SDK imports/scans. Run before archive creation."""
from pathlib import Path
import json,hashlib
O=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
files={str(p.relative_to(O)):sha(p) for p in sorted(O.rglob('*')) if p.is_file() and not any(n in p.relative_to(O).parts for n in ('__pycache__','checks','work')) and p.name not in ('manifest.json','REPORT.md','result.json','archive.json') and p.suffix not in ('.gz','.tar')}
# The embedded native manifest is unchanged, included by explicit relative binding.
files['candidate/candidate-source.json']=sha(O/'candidate/candidate-source.json');files['provenance/manifest.json']=sha(O/'provenance/manifest.json')
fingerprint=hashlib.sha256(json.dumps(files,sort_keys=True,separators=(',',':')).encode()).hexdigest()
(O/'manifest.json').write_text(json.dumps({'schema':'b1-v8-portable','files':files,'fingerprint':fingerprint,'native_manifest_sha256':sha(O/'candidate/candidate-source.json'),'physical_proof':False},indent=2)+'\n')
print(json.dumps({'files':len(files),'fingerprint':fingerprint}))
