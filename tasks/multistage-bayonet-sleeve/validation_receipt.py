from pathlib import Path
import json,hashlib
HERE=Path(__file__).resolve().parent
def verify_receipt(root=HERE,check_runtime=True):
 try:
  r=json.loads((root/'calibration.json').read_text(),parse_constant=lambda x:(_ for _ in ()).throw(ValueError(x)))
  digest=r.pop('self_sha256')
  if hashlib.sha256(json.dumps(r,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()!=digest:return False
  if type(r['schema_version']) is not int or r['schema_version']!=1:return False
  if type(r['control_hz']) is not int or r['control_hz']!=20 or type(r['physics_dt']) is not float or r['physics_dt']!=.002:return False
  if r['physics_device']!='cpu' or r['passed'] is not True:return False
  from scoring import REQUIRED
  if set(r['checks'])!=REQUIRED or any(v is not True for v in r['checks'].values()):return False
  required={str(p.relative_to(root)) for p in root.rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.name not in {'calibration.json','manifest.json'}}
  if set(r['source_sha256'])!=required:return False
  for name,digest in r['source_sha256'].items():
   p=Path(name)
   if p.is_absolute() or '..' in p.parts or hashlib.sha256((root/p).read_bytes()).hexdigest()!=digest:return False
  if check_runtime:
   from runtime_binding import verify_runtime
   verify_runtime()
  return True
 except (OSError,ValueError,KeyError,TypeError,RuntimeError,ImportError):return False
