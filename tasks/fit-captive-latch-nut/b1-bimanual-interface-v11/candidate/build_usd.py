"""Offline pxr authoring of source geometry. No simulator or native imports."""
from pathlib import Path
import json,math
import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,Gf,Sdf
HERE=Path(__file__).resolve().parent
F=json.loads((HERE/'fixture.json').read_text())
def qf(q):return Gf.Quatf(float(q[0]),Gf.Vec3f(*map(float,q[1:])))
def xf(s,path,p,q):
 x=UsdGeom.Xform.Define(s,path);x.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float,p)));x.AddOrientOp().Set(qf(q));return x.GetPrim()
def attr(p,n,v,t):p.CreateAttribute(n,t,custom=False).Set(v)
def align(axis):
 a=np.array(axis,float);a/=np.linalg.norm(a)
 if a[0]<-.999999:return [0,0,0,1]
 q=np.r_[1+a[0],np.cross([1,0,0],a)];return q/np.linalg.norm(q)
def rot(q):return Rotation.from_quat(np.roll(q,-1))
def volume(g):
 z=g['size'];typ=g['type']
 if typ=='box':return 8*np.prod(z)
 if typ=='sphere':return 4/3*math.pi*z[0]**3
 if typ=='cylinder':return math.pi*z[0]**2*2*z[1]
 if typ=='mesh':return ConvexHull(np.array(F['meshes'][g['mesh']]).reshape(-1,3)).volume
 raise ValueError(typ)
def mass_properties(geoms):
 if not geoms:return 1e-6,np.zeros(3),np.full(3,1e-9),np.array([1.,0,0,0])
 parts=[]
 for g in geoms:
  mass=float(g.get('mass',1000*volume(g)));sz=g['size'];typ=g['type'];centre=np.zeros(3)
  if typ=='box':I=mass/3*np.diag([sz[1]**2+sz[2]**2,sz[0]**2+sz[2]**2,sz[0]**2+sz[1]**2])
  elif typ=='sphere':I=np.eye(3)*2/5*mass*sz[0]**2
  elif typ=='cylinder':I=np.diag([mass*(3*sz[0]**2+(2*sz[1])**2)/12]*2+[mass*sz[0]**2/2])
  else:
   pts=np.array(F['meshes'][g['mesh']]).reshape(-1,3);h=ConvexHull(pts);V=0.;first=np.zeros(3);second=np.zeros((3,3))
   for face,eq in zip(h.simplices,h.equations):
    a,b,c=pts[face]
    if np.dot(np.cross(b-a,c-a),eq[:3])<0:b,c=c,b
    v=np.linalg.det(np.stack([a,b,c]))/6;s=a+b+c;V+=v;first+=v*s/4;second+=v*(np.outer(s,s)+np.outer(a,a)+np.outer(b,b)+np.outer(c,c))/20
   centre=first/V;S=second*mass/V;I=np.trace(S)*np.eye(3)-S-mass*(np.dot(centre,centre)*np.eye(3)-np.outer(centre,centre))
  r=rot(g['quat']).as_matrix();parts.append((mass,np.array(g['pos'])+r@centre,r@I@r.T))
 total=sum(v[0] for v in parts);com=sum(m*p for m,p,I in parts)/total;inertia=np.zeros((3,3))
 for m,p,I in parts:
  d=p-com;inertia+=I+m*(np.dot(d,d)*np.eye(3)-np.outer(d,d))
 values,axes=np.linalg.eigh(inertia)
 if np.linalg.det(axes)<0:axes[:,0]*=-1
 return total,com,values,np.roll(Rotation.from_matrix(axes).as_quat(),1)

def shape(s,bpath,g):
 path=bpath+'/'+g['name'].replace('-','minus');typ=g['type'];sz=g['size']
 if typ=='box':x=UsdGeom.Cube.Define(s,path);x.CreateSizeAttr(2);scale=sz
 elif typ=='sphere':x=UsdGeom.Sphere.Define(s,path);x.CreateRadiusAttr(sz[0]);scale=None
 elif typ=='cylinder':x=UsdGeom.Cylinder.Define(s,path);x.CreateAxisAttr('Z');x.CreateRadiusAttr(sz[0]);x.CreateHeightAttr(2*sz[1]);scale=None
 elif typ=='mesh':
  pts=np.array(F['meshes'][g['mesh']]).reshape(-1,3);h=ConvexHull(pts);faces=h.simplices.copy()
  for i,(face,eq) in enumerate(zip(faces,h.equations)):
   a,b,c=pts[face]
   if np.dot(np.cross(b-a,c-a),eq[:3])<0:faces[i]=face[[0,2,1]]
  x=UsdGeom.Mesh.Define(s,path);x.CreatePointsAttr([Gf.Vec3f(*v) for v in pts]);x.CreateFaceVertexCountsAttr([3]*len(faces));x.CreateFaceVertexIndicesAttr(faces.ravel().tolist());x.CreateSubdivisionSchemeAttr('none');scale=None
 else:raise ValueError(typ)
 x.AddTranslateOp().Set(Gf.Vec3d(*g['pos']));x.AddOrientOp().Set(qf(g['quat']))
 if scale:x.AddScaleOp().Set(Gf.Vec3f(*scale))
 x.CreateDisplayColorAttr([Gf.Vec3f(*g['rgba'][:3])]);p=x.GetPrim()
 mat=UsdShade.Material.Define(s,path+'_material');shader=UsdShade.Shader.Define(s,path+'_material/Shader');shader.CreateIdAttr('UsdPreviewSurface');shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*g['rgba'][:3]));shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.65);mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface');UsdShade.MaterialBindingAPI.Apply(p).Bind(mat)
 if g['contype'] or g['conaffinity']:
  UsdPhysics.CollisionAPI.Apply(p);p.AddAppliedSchema('PhysxCollisionAPI');attr(p,'physxCollision:contactOffset',.0001,Sdf.ValueTypeNames.Float);attr(p,'physxCollision:restOffset',0.,Sdf.ValueTypeNames.Float)
  if typ=='mesh':UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr('convexHull')
  pm=UsdPhysics.MaterialAPI.Apply(mat.GetPrim());pm.CreateStaticFrictionAttr(g['friction'][0]);pm.CreateDynamicFrictionAttr(g['friction'][0]);pm.CreateRestitutionAttr(0);UsdShade.MaterialBindingAPI.Apply(p).Bind(mat,UsdShade.Tokens.weakerThanDescendants,'physics')
 return p

def label(s,path,name):
 # Source prose calls the size bands "stamped", but source visuals contained
 # only colours. Add visible numeric decals, without changing collision/mass.
 text={'w12':'12','w1_2':'1/2','w13':'13','w14':'14'}.get(name)
 if text is None:return
 font={'1':['010','110','010','010','111'],'2':['111','001','111','100','111'],'3':['111','001','111','001','111'],'4':['101','101','111','001','001'],'/':['001','001','010','100','100']}
 pitch=.00065;width=(len(text)*4-1)*pitch
 for i,ch in enumerate(text):
  for row,line in enumerate(font[ch]):
   for col,ink in enumerate(line):
    if ink=='1':
     x=UsdGeom.Cube.Define(s,path+f'/size_{i}_{row}_{col}');x.CreateSizeAttr(1)
     x.AddTranslateOp().Set(Gf.Vec3d((i*4+col+.5)*pitch-width/2,-.05+(2-row)*pitch,.00292))
     x.AddScaleOp().Set(Gf.Vec3f(.00055,.00055,.00002));x.CreateDisplayColorAttr([Gf.Vec3f(.97,.97,.97)])

def author(name,selected,free=False):
 out=HERE/'assets'/f'{name}.usda';s=Usd.Stage.CreateNew(str(out));UsdGeom.SetStageUpAxis(s,'Z');UsdGeom.SetStageMetersPerUnit(s,1);root='/Wrench';s.SetDefaultPrim(xf(s,root,[0,0,0],[1,0,0,0]));poses={};by={b['name']:b for b in F['bodies']};paths={}
 if not free:
  rp=xf(s,root+'/links/base',[0,0,0],[1,0,0,0]);UsdPhysics.RigidBodyAPI.Apply(rp);UsdPhysics.MassAPI.Apply(rp).CreateMassAttr(1e-6);UsdPhysics.ArticulationRootAPI.Apply(s.GetDefaultPrim());s.GetDefaultPrim().AddAppliedSchema('PhysxArticulationAPI');attr(s.GetDefaultPrim(),'physxArticulation:enabledSelfCollisions',True,Sdf.ValueTypeNames.Bool)
  j=UsdPhysics.FixedJoint.Define(s,root+'/joints/world');j.CreateBody1Rel().SetTargets([root+'/links/base'])
 def world(b):
  if b['name'] in poses:return poses[b['name']]
  p=np.array(b['pos']);r=rot(b['quat'])
  if b['parent']:
   pp,pr=world(by[b['parent']]);p=pp+pr.apply(p);r=pr*r
  poses[b['name']]=(p,r);return p,r
 for b in selected:
  p,r=world(b);bp=root if free else root+'/links/'+b['name'];paths[b['name']]=bp
  if free:
   prim=s.GetDefaultPrim(); # Root placed by Arena spec, local body frame remains unchanged.
  else:prim=xf(s,bp,p,np.roll(r.as_quat(),1))
  UsdPhysics.RigidBodyAPI.Apply(prim);prim.AddAppliedSchema('PhysxRigidBodyAPI');attr(prim,'physxRigidBody:enableCCD',True,Sdf.ValueTypeNames.Bool);mass=UsdPhysics.MassAPI.Apply(prim);total,com,inertia,axes=mass_properties(b['geoms']);mass.CreateMassAttr(total);mass.CreateCenterOfMassAttr(Gf.Vec3f(*com));mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia));mass.CreatePrincipalAxesAttr(qf(axes))
  for g in b['geoms']:shape(s,bp,g)
  if free:label(s,bp,b['name'])
 if not free:
  for b in selected:
   bp=paths[b['name']];pp=paths.get(b['parent'],root+'/links/base');p,r=world(b)
   parentp,parentr=world(by[b['parent']]) if b['parent'] else (np.zeros(3),Rotation.identity())
   spec=b['joints'][0] if b['joints'] else None
   if spec:
    j=UsdPhysics.RevoluteJoint.Define(s,root+'/joints/'+spec['name']);j.CreateAxisAttr('X');a=rot(align(list(map(float,spec.get('axis','0 0 1').split()))));j.CreateLocalRot1Attr(qf(np.roll(a.as_quat(),1)));j.CreateLocalRot0Attr(qf(np.roll((parentr.inv()*r*a).as_quat(),1)))
    bounds=list(map(float,spec.get('range','-1000000 1000000').split()));
    if spec['name']=='arm_hinge' or spec['name'].endswith('_slip'):bounds=[0,0]
    j.CreateLowerLimitAttr(math.degrees(bounds[0]));j.CreateUpperLimitAttr(math.degrees(bounds[1]));dr=UsdPhysics.DriveAPI.Apply(j.GetPrim(),'angular');dr.CreateTypeAttr('force');dr.CreateStiffnessAttr(0);dr.CreateDampingAttr(float(spec.get('damping',0))*math.pi/180);dr.CreateMaxForceAttr(float('inf'))
    j.GetPrim().AddAppliedSchema('PhysxJointAPI');attr(j.GetPrim(),'physxJoint:armature',float(spec.get('armature',0)),Sdf.ValueTypeNames.Float)
    j.GetPrim().AddAppliedSchema('PhysxJointAxisAPI:angular')
    for f in ['staticFrictionEffort','dynamicFrictionEffort']:attr(j.GetPrim(),'physxJointAxis:angular:'+f,float(spec.get('frictionloss',0)),Sdf.ValueTypeNames.Float)
   else:j=UsdPhysics.FixedJoint.Define(s,root+'/joints/fixed_'+b['name']);j.CreateLocalRot0Attr(qf(np.roll((parentr.inv()*r).as_quat(),1)))
   j.CreateLocalPos0Attr(Gf.Vec3f(*parentr.inv().apply(p-parentp)));j.CreateLocalPos1Attr(Gf.Vec3f(0,0,0));j.CreateBody0Rel().SetTargets([pp]);j.CreateBody1Rel().SetTargets([bp]);j.CreateCollisionEnabledAttr(False)
  # Preserve native contype/conaffinity exclusions between every task shape.
  gs=[(paths[b['name']]+'/'+g['name'].replace('-','minus'),g) for b in selected for g in b['geoms']]
  for pa,a in gs:
   excluded=[pb for pb,b in gs if pa!=pb and not ((a['contype']&b['conaffinity']) or (b['contype']&a['conaffinity']))]
   if excluded:UsdPhysics.FilteredPairsAPI.Apply(s.GetPrimAtPath(pa)).CreateFilteredPairsRel().SetTargets(excluded)
  # Source explicit world exclusions: arm and hex head clear their supporting
  # post/washer. Preserve these exclusions; no tool/arm or robot/arm exception.
  fixed=[paths['fixed_fixture']+'/'+g['name'] for g in by['fixed_fixture']['geoms']]
  for name in ['swing_arm','latch_head']:
   for g in by[name]['geoms']:
    rel=UsdPhysics.FilteredPairsAPI.Apply(s.GetPrimAtPath(paths[name]+'/'+g['name'])).CreateFilteredPairsRel()
    for target in fixed:rel.AddTarget(Sdf.Path(target))
 s.GetRootLayer().Save();return str(out)
if __name__=='__main__':
 print(author('fixture',[b for b in F['bodies'] if not b['free']]))
 for b in F['bodies']:
  if b['free']:print(author(b['name'],[b],True))
