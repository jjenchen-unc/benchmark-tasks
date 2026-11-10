"""Copy a portable immutable runtime input; does not launch SDK/physics/jobs."""
import argparse,hashlib,json,shutil
from pathlib import Path
HERE=Path(__file__).resolve().parent
if __name__=='__main__':
 ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
 if a.out.exists() or HERE in a.out.resolve().parents:raise ValueError('new output outside source required')
 shutil.copytree(HERE,a.out,ignore=shutil.ignore_patterns('__pycache__','*.pyc','sourcehashes.json'))
 hashes={str(p.relative_to(a.out)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(a.out.rglob('*')) if p.is_file()}
 (a.out/'sourcehashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
 for p in a.out.rglob('*'):
  if p.is_file():p.chmod(0o444)
 print(json.dumps(dict(source=str(a.out.resolve()),files=len(hashes),fingerprint=hashlib.sha256(json.dumps(hashes,sort_keys=True).encode()).hexdigest())))
