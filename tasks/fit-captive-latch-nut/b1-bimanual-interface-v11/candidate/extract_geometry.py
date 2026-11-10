"""Execute source geometry functions offline; never import MuJoCo or robot assets."""
import ast, hashlib, json, math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
source = HERE / 'provenance/B1_wrench.py'
ns = dict(np=np, ET=ET, HOME=np.zeros(7))
functions = {'other', 'v3', 'parameters', 'sub', 'box', 'hexmesh', 'wrench_gap',
             'add_wrench', 'nut_fixture', 'wrench_rack', 'add_swing_arm'}
executed = []
for node in ast.parse(source.read_text()).body:
    if isinstance(node, ast.Assign):
        try:
            exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), 'exec'), ns)
        except NameError:
            pass
    elif isinstance(node, ast.FunctionDef) and node.name in functions:
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), 'exec'), ns)
        executed.append(node.name)
r = ET.Element('mujoco')
for name in ['asset', 'worldbody', 'equality', 'contact']:
    ET.SubElement(r, name)
w = r.find('worldbody'); params = ns['parameters'](42)
ns['wrench_rack'](r, w, params)
ns['nut_fixture'](r, w, 'latch', ns['W1_NUT'], ns['AF'][params['nut_size_yellow']], '.95 .85 .1 1')
ns['add_swing_arm'](r, w)
(HERE / 'assets/source-geometry.xml').write_text(ET.tostring(r, encoding='unicode'))

def vec(s, default):
    return list(map(float, s.split())) if s else default

def quat(e):
    return vec(e.get('quat'), [1, 0, 0, 0]) if e.get('quat') else np.roll(Rotation.from_euler('xyz', vec(e.get('euler'), [0, 0, 0])).as_quat(), 1).tolist()

bodies = []
def visit(e, parent, name=None):
    b = dict(name=name or e.get('name'), parent=parent, pos=vec(e.get('pos'), [0,0,0]),
             quat=quat(e), free=e.find('freejoint') is not None,
             joints=[dict(j.attrib) for j in e.findall('joint')], geoms=[])
    for g in e.findall('geom'):
        a = dict(g.attrib)
        a.update(type=g.get('type', 'sphere'), pos=vec(g.get('pos'), [0,0,0]), quat=quat(g),
                 size=vec(g.get('size'), []), rgba=vec(g.get('rgba'), [.6,.6,.65,1]),
                 friction=vec(g.get('friction'), [1,.005,.0001]),
                 contype=int(g.get('contype','1')), conaffinity=int(g.get('conaffinity','1')))
        b['geoms'].append(a)
    bodies.append(b)
    for c in e.findall('body'):
        visit(c, b['name'] if e.tag != 'worldbody' else None)
visit(w, None, 'fixed_fixture')
f = dict(target='W1_latch_release', source_seed=42,
         seed_semantics='One fixed physical source S/42 case; Inspect seeds identify episodes and do not randomize task properties.',
         rotation=[[-1.,0.,0.],[0.,-1.,0.],[0.,0.,1.]], translation=[.40,.10,-.0525],
         params=params, bodies=bodies,
         meshes={e.get('name'):vec(e.get('vertex'),[]) for e in r.findall('asset/mesh')},
         source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
         executed_source_geometry_functions=executed,
         source_objects_bounds=[[-.75,-.85,.70],[.75,.85,1.60]], source_table_z=.8)
(HERE/'fixture.json').write_text(json.dumps(f, indent=2)+'\n')
print(json.dumps(dict(bodies=len(bodies), geoms=sum(len(b['geoms']) for b in bodies), parameters=params)))
