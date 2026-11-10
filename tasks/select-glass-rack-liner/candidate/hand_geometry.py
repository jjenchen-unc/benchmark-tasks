"""Pure exact SDK collision-mesh and FK measurements; no simulation."""
import hashlib,json,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np

def measure():
 from yam_arena import YAM_URDF
 from yam_arena.urdf import forward_kinematics,rpy_matrix
 from yam_arena.analysis import read_stl
 from yam_arena.constants import TCP_OFFSET
 root=ET.parse(YAM_URDF).getroot();rows=[];hashes={str(YAM_URDF):hashlib.sha256(Path(YAM_URDF).read_bytes()).hexdigest()}
 for gap in [0.,.0628,.09]:
  fk=forward_kinematics(YAM_URDF,{**{f'joint{i}':0. for i in range(1,7)},'joint7':-gap/2,'joint8':-gap/2});G=np.linalg.inv(fk['gripper']);records=[]
  for name in ['base','tip_left','tip_right','gripper']:
   verts=[]
   for c in root.find("link[@name='"+name+"']").findall('collision'):
    o=c.find('origin');T=np.eye(4)
    if o is not None:T[:3,3]=np.fromstring(o.get('xyz','0 0 0'),sep=' ');T[:3,:3]=rpy_matrix(np.fromstring(o.get('rpy','0 0 0'),sep=' '))
    mesh=c.find('geometry/mesh')
    if mesh is None:continue
    path=Path(YAM_URDF).parent/mesh.get('filename');v=read_stl(path).reshape(-1,3);hashes[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest();M=T if name=='base' else G@fk[name]@T;verts.append(v@M[:3,:3].T+M[:3,3])
   if verts:
    v=np.concatenate(verts);records.append({'link':name,'low':v.min(0).tolist(),'high':v.max(0).tolist(),'frame':'base' if name=='base' else 'gripper'})
  rows.append({'jaw_gap_m':gap,'collision_bounds':records})
 return {'kind':'PURE_ACTUAL_SDK_COLLISION_STL_FK_NO_PHYSICS','tcp_offset_m':TCP_OFFSET,'measurements':rows,'sha256':hashes}
if __name__=='__main__':print(json.dumps(measure(),indent=2))
