from pathlib import Path
import sys,json,hashlib,math
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
def preflight(root=HERE):
 from validation_receipt import verify_receipt
 if not verify_receipt(root):raise RuntimeError('receipt/source/runtime mismatch')
 m=json.loads((root/'manifest.json').read_text())['files_sha256']
 actual={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in root.rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.name!='manifest.json'}
 if m!=actual:raise RuntimeError('manifest mismatch')
 actions=json.loads((root/'saved-actions.json').read_text())
 if len(actions)!=1490 or any(len(a)!=14 or any(type(v) not in (int,float) or not math.isfinite(v) for v in a) for a in actions):raise ValueError('actions')
 return actions
if __name__=='__main__':print(json.dumps(dict(actions=len(preflight()),passed=True,simulation_starts=0)))
