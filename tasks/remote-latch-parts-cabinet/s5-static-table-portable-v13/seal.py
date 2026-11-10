"""Freeze portable inputs without scanning SDK or changing native proof identities."""
from pathlib import Path
import json,hashlib
O=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
files={str(p.relative_to(O)):sha(p) for p in sorted(O.rglob('*')) if p.is_file() and not any(n in p.relative_to(O).parts for n in ('__pycache__','checks','work')) and p.name not in ('manifest.json','REPORT.md','result.json','archive.json','full-consumed-source-map.json') and p.suffix not in ('.gz','.tar')}
# Preserve and explicitly bind embedded original manifest, including its fingerprint.
files['candidate/snapshot-manifest.json']=sha(O/'candidate/snapshot-manifest.json')
fingerprint=hashlib.sha256(json.dumps(files,sort_keys=True,separators=(',',':')).encode()).hexdigest()
(O/'manifest.json').write_text(json.dumps(dict(schema='s5-v13-portable',files=files,fingerprint=fingerprint,native_source_fingerprint='dd957715b71e2f0c2696e2a201c3bba35f6d961bb73d4f6273307e76824d79dd',physical_proof=False),indent=2)+'\n')
print(json.dumps(dict(files=len(files),fingerprint=fingerprint)))
