"""Resolve the source-pinned installation across unchanged SDK root locations."""
from pathlib import Path
import ast,hashlib,importlib.util,json,sys

def finder_semantic_sha256(source,roots):
    # Editable finder maps are generated dictionaries; only their key order and
    # installation root strings may differ. Every other AST node remains pinned.
    for old,new in sorted(roots.items(),key=lambda item:-len(str(item[1]))):
        source=source.replace(str(new),old)
    tree=ast.parse(source);seen=set()
    for node in tree.body:
        if isinstance(node,ast.AnnAssign) and isinstance(node.target,ast.Name) and node.target.id in ('MAPPING','NAMESPACES'):
            value=ast.literal_eval(node.value)
            if type(value) is not dict or node.target.id in seen:raise ValueError('Invalid finder mapping')
            seen.add(node.target.id)
            node.value=ast.Constant(json.dumps(value,sort_keys=True,separators=(',',':')))
    if seen!={'MAPPING','NAMESPACES'}:raise ValueError('Missing finder mapping')
    return hashlib.sha256(ast.dump(tree,include_attributes=False).encode()).hexdigest()

def installed_roots():
    lab=importlib.util.find_spec('isaaclab');yam=importlib.util.find_spec('yam_arena')
    if lab is None or lab.origin is None or yam is None or yam.origin is None:raise RuntimeError('Missing pinned installation')
    return {'/opt/python/cpython-3.12.16-linux-x86_64-gnu':Path(sys.base_prefix),
            '/opt/IsaacLab-Arena/.venv':Path(sys.prefix),
            '/opt/IsaacLab-Arena':Path(lab.origin).resolve().parents[5],
            '/opt/yam-arena':Path(yam.origin).resolve().parents[1]}

def resolve_source(name,roots):
    path=Path(name)
    if not path.is_absolute() or '..' in path.parts:raise ValueError('Invalid pinned path')
    for old,new in sorted(roots.items(),key=lambda item:-len(item[0])):
        if name.startswith(old+'/'):return Path(new)/name[len(old)+1:]
    raise ValueError('Unrecognized runtime root')

def verify_runtime(root):
    root=Path(root);pins=json.loads((root/'runtime-source-sha256.json').read_text());special=json.loads((root/'editable-runtime-semantics.json').read_text());roots=installed_roots()
    if len(pins)!=211 or len(special)!=16 or not set(special).issubset(pins):raise RuntimeError('Incomplete runtime pins')
    for name,digest in pins.items():
        path=resolve_source(name,roots);raw=path.read_bytes()
        if hashlib.sha256(raw).hexdigest()==digest:continue
        row=special.get(name)
        if row is None or row['original_sha256']!=digest or '__editable__' not in Path(name).name:raise RuntimeError('Runtime source mismatch '+name)
        if finder_semantic_sha256(raw.decode(),roots)!=row['normalized_finder_ast_sha256']:raise RuntimeError('Editable runtime semantics mismatch '+name)
    return True
