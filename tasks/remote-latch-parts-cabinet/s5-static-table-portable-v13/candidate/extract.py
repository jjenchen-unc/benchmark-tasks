"""Static extraction: execute exact archived source; never build/step native physics."""
import hashlib,json,runpy,xml.etree.ElementTree as ET
from pathlib import Path
HERE=Path(__file__).resolve().parent
src=HERE/'source/benches/S5_remote_release.py'
b=runpy.run_path(str(src));p=b['parameters'](42);cabs=b['cabinets_for']('RR-T1',p)
vec=lambda s,default: [float(x) for x in (s or default).split()]
def body(node,parent=None,world=None):
 pos=vec(node.get('pos'),'0 0 0');wp=[a+c for a,c in zip(world or [0,0,0],pos)];geoms=[];joints=[]
 for g in node.findall('geom'):
  geoms.append(dict(name=g.get('name'),kind=g.get('type'),pos=vec(g.get('pos'),'0 0 0'),quat_wxyz=vec(g.get('quat'),'1 0 0 0'),size=vec(g.get('size'),'0 0 0'),mass=float(g.get('mass','0')),rgba=vec(g.get('rgba'),'0.62 0.64 0.66 1'),collision=g.get('contype','1')!='0',friction=1.1))
 for j in node.findall('joint'):
  joints.append(dict(name=j.get('name'),kind=j.get('type'),axis=vec(j.get('axis'),'0 0 1'),range=vec(j.get('range'),'0 1'),stiffness=float(j.get('stiffness','0')),springref=float(j.get('springref','0')),damping=float(j.get('damping','0')),armature=float(j.get('armature','.02')),frictionloss=float(j.get('frictionloss','0'))))
 out=[dict(name=node.get('name'),parent=parent,world_pos=wp,geoms=geoms,joints=joints,free=node.find('freejoint') is not None)]
 for child in node.findall('body'):out+=body(child,node.get('name'),wp)
 return out
bodies=[]
for name,c in cabs.items():
 xml=b['cabinet_xml'](name,c);(HERE/f'assets/{name}.xml').write_text(xml)
 for node in ET.fromstring('<world>'+xml+'</world>'):bodies+=body(node)
f=dict(seed_semantics='Fixed physical case sampled with source seed42; any harness episode seed keeps identical geometry/properties. No sampled map is exposed to model observations.',native_seed=42,target='RR-T1',parameters=p,cabinets=cabs,bodies=bodies,translation=[.15,0,-.0525],robot_bases={'left':[0,.305,1.1],'right':[0,-.305,1.1]},table_z=.7475,source_sha256=hashlib.sha256(src.read_bytes()).hexdigest(),static_only=True,control_hz=20,physics_dt=.002,time_limit_s=182.3)
(HERE/'fixture.json').write_text(json.dumps(f,indent=2)+'\n')
print(json.dumps(dict(source_executed=True,native_build_called=False,physics_steps=0,bodies=len(bodies),geoms=sum(len(x['geoms']) for x in bodies),source_sha256=f['source_sha256'])))
