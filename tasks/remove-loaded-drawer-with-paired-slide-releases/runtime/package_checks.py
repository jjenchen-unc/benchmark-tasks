"""Portable read-only source checks; no SDK scan, model, simulator or credentials."""
from pathlib import Path
import sys,hashlib,json,ast
O=Path(__file__).resolve().parent;C=O/'candidate';sys.path.insert(0,str(C))
from runtime_credit import load,sha,validate
from fresh_evidence import relative_file

def check():
 m=load(C/'manifest.json');fingerprint=m.pop('fingerprint')
 if hashlib.sha256(json.dumps(m,sort_keys=True,separators=(',',':')).encode()).hexdigest()!=fingerprint:raise ValueError('manifest fingerprint mismatch')
 expected=m['sha256']
 for p in C.rglob('*'):
  if p.is_symlink() or not p.resolve().is_relative_to(C.resolve()):raise ValueError('candidate path escapes/symlink')
 actual={str(p.relative_to(C)):sha(p) for p in C.rglob('*') if p.is_file() and p.name!='manifest.json' and '__pycache__' not in p.parts}
 if actual!=expected:raise ValueError('candidate file map mismatch')
 for name,d in load(O/'launch-input-sha256.json').items():
  if sha(relative_file(O,name))!=d:raise ValueError('portable input changed: '+name)
 for name,d in load(O/'core-sha256.json').items():
  if sha(relative_file(C,name))!=d:raise ValueError('native core changed: '+name)
 docs=ast.get_docstring(ast.parse((C/'task.py').read_text()))
 if docs!=ast.get_docstring(ast.parse((C/'model_entry.py').read_text())) or docs!=load(C/'task.draft.json')['instruction']:raise ValueError('instruction mismatch')
 validate(C,O/'reviewed-actual-portable/ROOT-review.json')
 return expected
if __name__=='__main__':print(json.dumps({'status':'PASS_CPU_SOURCE_ONLY','source_files':len(check()),'physical_approval':False,'model_calls':0,'simulator_initializations':0}))
