from pathlib import Path
import importlib.util,json,hashlib
HERE=Path(__file__).resolve().parent
def verify_runtime():
 for name,digest in json.loads((HERE/'runtime-pins.json').read_text()).items():
  spec=importlib.util.find_spec(name)
  if spec is None or spec.origin is None or hashlib.sha256(Path(spec.origin).read_bytes()).hexdigest()!=digest:raise RuntimeError('runtime source mismatch '+name)
 spec=importlib.util.find_spec('isaaclab')
 if spec is None or spec.origin is None:raise RuntimeError('missing isaaclab root')
 source=Path(spec.origin).resolve().parents[2]
 for relative,digest in json.loads((HERE/'runtime-force-pins.json').read_text()).items():
  if hashlib.sha256((source/relative).read_bytes()).hexdigest()!=digest:raise RuntimeError('force source mismatch '+relative)
 return True
