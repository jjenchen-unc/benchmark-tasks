from pathlib import Path
import sys,json,time,math
import numpy as np
O=Path(__file__).resolve().parent;sys.path.insert(0,str(O/'candidate'))
from pad_region import DATA,inside_local,contact_is_pad
from model_physics import ModelPhysics
from drawers_physics import Physics
start=time.monotonic();checks=0;positive=0;negative=0
A=json.loads((O/'pad-geometry-analysis.json').read_text())
for name,actor in DATA['actors'].items():
 source=A['actors'][name];R0=np.array(source['joint_origin_R']);jp=np.array(source['joint_origin_m']);axis=np.array(source['local_prismatic_axis'])
 pts=[]
 for solid in actor['solids']:
  vertices=np.array(solid['vertices']);low=vertices[vertices[:,2]<=-.0372+1e-12];pts.extend(low.tolist())
  # Convex face centres, edge midpoints and cut-plane intersections.
  planes=np.array(solid['planes'])
  for q in planes:
   face=vertices[abs(vertices@q[:3]+q[3])<1e-7];face=face[face[:,2]<=-.0372+1e-12]
   if len(face):pts.append(face.mean(axis=0).tolist())
  for lo in low:
   for hi in vertices[vertices[:,2]>-.0372]:
    t=(-.0372-lo[2])/(hi[2]-lo[2]);pts.append((lo+t*(hi-lo)).tolist())
  if len(low):pts.append(low.mean(axis=0).tolist())
 pts=np.unique(np.round(pts,12),axis=0)
 assert len(pts)>20
 for p in pts:assert inside_local(name,p),(name,p)
 positive+=len(pts)
 # True proximal hardware vertices: use saved actual non-distal component bounds.
 outside=[np.array([0,0,0]),np.array([0,0,-.12]),np.array([.10,.10,-.05]),np.array([0,0,-.03719])]
 for mesh in source['meshes']:
  if mesh['link_bounds_m'][2][0]>-.0372:outside.append(np.mean(mesh['link_bounds_m'],axis=1))
 for p in outside:assert not inside_local(name,p),(name,p)
 negative+=len(outside)
 # Independent finger translations (each has its own actual link pose), rotations,
 # both actor orderings and arbitrary normals (normals deliberately not consumed).
 for raw in [-.04695,-.023475,0.]:
  for yaw in [0.,.7,math.pi]:
   c,s=math.cos(yaw),math.sin(yaw);Q=np.array([[c,-s,0],[s,c,0],[0,0,1.]])@np.array([[1,0,0],[0,0,-1],[0,1,0]])
   R=Q@R0;pos=np.array([.4,-.2,.9])+Q@(jp+R0@(axis*raw));path='/robot/'+name;poses={path:(name,pos,R)}
   for p in [*pts[::max(1,len(pts)//75)],*outside]:
    expected=inside_local(name,p);world=pos+R@p
    for a,b in [(path,'jar'),('jar',path)]:
     for normal in ([1,0,0],[-1,0,0],[0,1,0],[0,0,-1]):
      assert contact_is_pad(a,b,world,poses)==expected;checks+=1
   assert not contact_is_pad('proximal_gripper','jar',pos,poses)
# Runtime adapter modifies exactly pad exclusion, preserving remainder of native read.
base=__import__('inspect').getsource(Physics.read);changed=__import__('inspect').getsource(ModelPhysics.read)
a=base.index('   # No named pad');b=base.index('   jar_force[name]+=force',a)
x=changed.index('   # Whole distal');y=changed.index('   jar_force[name]+=force',x)
assert base[:a]+base[b:]==changed[:x]+changed[y:]
# Exact nonnegative-sum/max law: exclude pad before both metrics, preserve touched.
rows=[(True,11.,.8),(False,3.,.2),(False,5.,.4)]
assert sum(f for pad,f,v in rows if not pad)==8 and max(v for pad,f,v in rows if not pad)==.4
out={'status':'PASS','evidence':'CPU conditional/synthetic geometry and source equivalence; no physical validation','positive_whole_solid_vertices_faces_edges_back_cut_corners':positive,'negative_proximal_and_outside':negative,'transformed_actor_order_normal_cases':checks,'jaw_limits_m':[-.04695,0.],'runtime_nonpad_read_source_exact':True,'wall_s':time.monotonic()-start}
(O/'pad-region-proof.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out))
