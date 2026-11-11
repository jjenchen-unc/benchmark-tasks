from pathlib import Path
import json,hashlib
O=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
files={str(p.relative_to(O)):sha(p) for p in sorted(O.rglob('*')) if p.is_file() and (p.parent==O and p.suffix in ('.py','.md','.sbatch','.json') and p.name in {'protocol.py','runtime.py','model_task.py','model_child.py','pre_sdk.py','strict_json.py','video_capture.py','verify_runtime.py','runtime-source-pins.json','seal.py','test_protocol.py','test_fresh_boundary.py','OPERATOR.md','README.md','GPU_REQUEST_UNSUBMITTED.sbatch'} or p.is_relative_to(O/'candidate') or p.is_relative_to(O/'provenance')) and '__pycache__' not in p.parts}
m=dict(files=files,fingerprint=hashlib.sha256(json.dumps(files,sort_keys=True,separators=(',',':')).encode()).hexdigest())
(O/'manifest.json').write_text(json.dumps(m,indent=2,allow_nan=False)+'\n')
print(m['fingerprint'])
