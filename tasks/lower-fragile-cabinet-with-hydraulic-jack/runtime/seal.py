"""Explicit development sealing only; never called by runtime."""
from pathlib import Path
import hashlib,json
O=Path(__file__).resolve().parent
exclude={'manifest.json','archive.json','result.json','REPORT.md','hydraulic-v5.tar.gz','full-consumed-source-map.json'}
m={str(p.relative_to(O)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(O.rglob('*')) if p.is_file() and p.relative_to(O).parts[0] not in ('checks','work','validation-proofs') and p.name not in exclude and '__pycache__' not in p.parts and p.suffix!='.pyc'}
fingerprint=hashlib.sha256(json.dumps(m,sort_keys=True,separators=(',',':')).encode()).hexdigest()
(O/'manifest.json').write_text(json.dumps({'sha256':m,'fingerprint':fingerprint,'native_embedded_manifest':'candidate/sourcehashes.json','native_embedded_manifest_preserved_exact':True},indent=2)+'\n')
print(fingerprint,len(m))
