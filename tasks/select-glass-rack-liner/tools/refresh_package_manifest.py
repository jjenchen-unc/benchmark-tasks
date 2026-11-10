from pathlib import Path
import hashlib,json
root=Path(__file__).resolve().parents[1]
files={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(root.rglob('*')) if p.is_file() and '__pycache__' not in p.parts and p.name!='package-manifest.json'}
(root/'package-manifest.json').write_text(json.dumps(files,indent=2)+'\n')
print(json.dumps({'package_files':len(files),'manifest_updated':True}))
