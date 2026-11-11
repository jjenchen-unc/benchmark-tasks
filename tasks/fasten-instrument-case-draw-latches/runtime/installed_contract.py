"""Resolve consumed sources in the installed environment; never alter the SDK."""
from pathlib import Path
import ast,hashlib,importlib.util,json,sysconfig
ROOT=Path(__file__).resolve().parent
def _metadata_and_code(path,aliases):
 tree=ast.parse(Path(path).read_text());metadata={};code=[]
 def normalize(v):
  if isinstance(v,dict):return {k:normalize(x) for k,x in v.items()}
  if isinstance(v,list):return [normalize(x) for x in v]
  if isinstance(v,str):
   for actual,original in sorted(aliases.items(),key=lambda x:-len(x[0])):
    if v==actual or v.startswith(actual+'/'):return original+v[len(actual):]
  return v
 for node in tree.body:
  if isinstance(node,ast.AnnAssign) and isinstance(node.target,ast.Name) and node.target.id in ('MAPPING','NAMESPACES'):
   if node.target.id in metadata:raise ValueError('duplicate editable metadata')
   metadata[node.target.id]=normalize(ast.literal_eval(node.value))
  else:code.append(node)
 if set(metadata)!={'MAPPING','NAMESPACES'}:raise ValueError('editable metadata missing')
 payload={'metadata':metadata,'code_ast':ast.dump(ast.Module(body=code,type_ignores=[]),include_attributes=False)}
 return hashlib.sha256(json.dumps(payload,sort_keys=True,separators=(',',':')).encode()).hexdigest()
def verify_editable(path,expected,contract,aliases):
 row=contract.get(Path(path).name)
 if row is None or row['original_sha256']!=expected:raise ValueError('unreviewed editable finder')
 if _metadata_and_code(path,aliases)!=row['semantic_sha256']:raise ValueError('editable finder code or path mapping changed')
def installed_paths():
 purelib=Path(sysconfig.get_path('purelib'));stdlib=Path(sysconfig.get_path('stdlib'))
 finder=purelib/'__editable___isaaclab_arena_0_3_0_finder.py';tree=ast.parse(finder.read_text())
 mappings=[ast.literal_eval(n.value) for n in tree.body if isinstance(n,ast.AnnAssign) and getattr(n.target,'id','')=='MAPPING']
 if len(mappings)!=1 or 'isaaclab_arena' not in mappings[0]:raise ValueError('installed Arena location unavailable')
 arena=str(Path(mappings[0]['isaaclab_arena']).parent);yam=str(Path(importlib.util.find_spec('yam_arena').origin).parent.parent)
 aliases={arena:'/opt/IsaacLab-Arena',yam:'/opt/yam-arena'}
 prefixes={'/opt/python/cpython-3.12.16-linux-x86_64-gnu/lib/python3.12/':str(stdlib)+'/', '/opt/IsaacLab-Arena/.venv/lib/python3.12/site-packages/':str(purelib)+'/', '/opt/yam-arena/':yam+'/'}
 return prefixes,aliases
def verify_installed(pins):
 prefixes,aliases=installed_paths();contract=json.loads((ROOT/'editable-finder-contract.json').read_text());exact=equivalent=0
 for original,expected in pins.items():
  prefix=next((k for k in prefixes if original.startswith(k)),None)
  if prefix is None:raise ValueError('unreviewed installed source prefix')
  path=Path(prefixes[prefix]+original[len(prefix):]);actual=hashlib.sha256(path.read_bytes()).hexdigest()
  if actual==expected:exact+=1;continue
  if not path.name.startswith('__editable___') or path.suffix!='.py':raise ValueError('installed consumed source changed '+original)
  verify_editable(path,expected,contract,aliases);equivalent+=1
 return {'pins':len(pins),'exact_bytes':exact,'editable_install_mapping_equivalent':equivalent}
