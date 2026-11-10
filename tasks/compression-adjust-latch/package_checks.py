"""Pure integrity checks; no SDK or simulator imports."""
from pathlib import Path
import hashlib,json,math

def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def scalar(v):
    # numpy is optional until actual execution.
    if type(v).__module__.split('.')[0]=='numpy':
        if hasattr(v,'tolist'):return v.tolist()
        if hasattr(v,'item'):return v.item()
    raise TypeError('Unsupported telemetry: '+type(v).__name__)
def dumps(v):return json.dumps(v,default=scalar,allow_nan=False)
def preflight(root):
    root=Path(root);rt=root/'runtime';expected=json.loads((root/'runtime-manifest.json').read_text())
    actual={str(p.relative_to(rt)):digest(p) for p in rt.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
    if actual!=expected:raise ValueError('Runtime integrity failed')
    receipt=json.loads((rt/'calibration.json').read_text())
    if not receipt['passed'] or len(receipt['source_sha256'])!=10 or not all(receipt['checks'].values()):raise ValueError('Invalid calibration')
    if any(actual.get(k)!=v for k,v in receipt['source_sha256'].items()):raise ValueError('Calibration source mismatch')
    prov=json.loads((root/'demo-provenance.json').read_text())
    if prov['verified_runtime_manifest']!=actual or digest(root/'demo-actions.jsonl')!=prov['journal_sha256']:raise ValueError('Action provenance mismatch')
    if digest(root/'evidence/result.json')!=prov['result_sha256']:raise ValueError('Actual result provenance mismatch')
    original=json.loads((root/'evidence/result.json').read_text())
    if original['source_sha256']!=actual or original['status']['score']!=1. or original['controls']!=1959:raise ValueError('Actual source/result mismatch')
    rows=[json.loads(x) for x in (root/'demo-actions.jsonl').read_text().splitlines()]
    if len(rows)!=1959 or len(rows)>3848:raise ValueError('Action count mismatch')
    for n,row in enumerate(rows,1):
        a=row['action']
        if row['control']!=n or row['status']['samples']!=25*n:raise ValueError('Control/sample mismatch')
        if len(a)!=14 or any(type(v) not in (int,float) or not math.isfinite(v) for v in a):raise ValueError('Invalid14D')
        if not all(0<=a[i]<=1 for i in (6,13)):raise ValueError('Invalid jaw')
    if rows[-1]['status']['score']!=1.:raise ValueError('No actual full score')
    task=json.loads((root/'task.draft.json').read_text())
    if task['instruction']!=json.loads((rt/'task.draft.json').read_text())['instruction']:raise ValueError('Instruction mismatch')
    if len(task['scores'])>300 or len(task['fidelity'])>500 or task['media'] is not None:raise ValueError('Draft metadata invalid')
    return rows
