"""Read-only dynamic installed source resolution; no simulator imports."""
from pathlib import Path
import hashlib,importlib.util,importlib.metadata as md,json,sys,sysconfig
HERE=Path(__file__).resolve().parent

def roots():
 out={'site':Path(sysconfig.get_paths()['purelib']),'stdlib':Path(sysconfig.get_paths()['stdlib'])}
 for key,module,levels in [('yam','yam_arena',1),('arena','isaaclab_arena',2),('lab','isaaclab',2)]:
  spec=importlib.util.find_spec(module)
  if spec is None or spec.origin is None:raise RuntimeError('missing runtime '+module)
  p=Path(spec.origin).resolve()
  # Source layouts are package/project/source/project/__init__.py; discover .git root.
  candidates=[v for v in p.parents if (v/'.git').exists()]
  if not candidates:raise RuntimeError('missing checkout provenance '+module)
  out[key]=candidates[0]
 return out

def verify_runtime():
 expected=json.loads((HERE/'runtime-expected.json').read_text()); actual=roots()
 if sys.version.split()[0]!=expected['python']:raise RuntimeError('python pin mismatch')
 for n,v in expected['versions'].items():
  if md.version(n)!=v:raise RuntimeError('version pin mismatch '+n)
 # Read git refs without spawning processes (preSDK audit forbids spawn).
 def head(root):
  git=root/'.git'
  if git.is_file():git=(root/git.read_text().strip().split(': ',1)[1]).resolve()
  value=(git/'HEAD').read_text().strip()
  if not value.startswith('ref: '):return value
  ref=value[5:]; path=git/ref
  if path.exists():return path.read_text().strip()
  for line in (git/'packed-refs').read_text().splitlines():
   if line.endswith(' '+ref):return line.split()[0]
  raise RuntimeError('unresolved checkout HEAD')
 for key in ['arena','yam']:
  if head(actual[key])!=expected['sources'][key]['head']:raise RuntimeError('checkout mismatch '+key)
 for pin in json.loads((HERE/'runtime-source-pins.json').read_text()):
  p=actual[pin['root']]/pin['relative']
  if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=pin['sha256']:raise RuntimeError('source pin mismatch '+str(p))
 return {k:str(v) for k,v in actual.items()}
