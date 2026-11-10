"""Strict package/action/source preflight; no simulator initialization."""
from pathlib import Path
import hashlib,json,sys
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE/'task'))
from strict_json import dumps

def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def preflight(root=HERE):
 root=Path(root)
 expected=json.loads((root/'package-manifest.json').read_text())
 actual={str(p.relative_to(root)):digest(p) for p in root.rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.name!='package-manifest.json'}
 if expected!=actual:raise RuntimeError('package inventory/hash mismatch')
 from runtime_binding import verify_runtime
 verify_runtime()
 from validation_receipt import verify_receipt,CHANNELS
 if verify_receipt(root/'task')!=CHANNELS:raise RuntimeError('actual receipt mismatch')
 rows=[json.loads(line) for line in (root/'task/robot-actions.jsonl').read_text().splitlines()]
 if len(rows)!=1441 or len(rows)>1896:raise RuntimeError('action count mismatch')
 for row in rows:
  a=row['action14']
  if len(a)!=14 or any(type(x) not in (int,float) for x in a):raise ValueError('action JSON must contain 14 numbers')
  dumps(row)
 return rows
if __name__=='__main__':
 rows=preflight();print(dumps({'passed':True,'actions':len(rows),'simulation_starts':0,'model_calls':0}))
