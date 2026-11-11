from pathlib import Path
import json,hashlib
O=Path(__file__).resolve().parent
# Native source manifests remain byte exact; refreshed outer manifest binds additions.
files={str(p.relative_to(O)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(O.rglob('*')) if p.is_file() and p.parts[len(O.parts)] not in {'work','checks'} and p.name not in {'manifest.json','archive.json','result.json','REPORT.md','full-consumed-source-map.json'} and p.suffix not in {'.gz','.pyc'} and '__pycache__' not in p.parts}
fingerprint=hashlib.sha256(json.dumps(files,sort_keys=True,separators=(',',':')).encode()).hexdigest()
(O/'manifest.json').write_text(json.dumps(dict(files_sha256=files,fingerprint_sha256=fingerprint),indent=2,sort_keys=True)+'\n')
print(fingerprint)
