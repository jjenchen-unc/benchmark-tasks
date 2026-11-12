"""Offline task-only USD authoring. Never imports or runs native physics."""
from pathlib import Path
import json,math
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,Sdf,Gf
HERE=Path(__file__).resolve().parent

def attr(p,n,v,t):p.CreateAttribute(n,t,custom=False).Set(v)
def q(v):return Gf.Quatd(float(v[0]),Gf.Vec3d(*map(float,v[1:])))
def R(v):return np.asarray(Gf.Matrix3d(q(v))).T
def xf(s,p,pos,quat):
 x=UsdGeom.Xform.Define(s,p);x.AddTranslateOp().Set(Gf.Vec3d(*map(float,pos)));x.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(q(quat));return x.GetPrim()
def quatf(v):return Gf.Quatf(float(v[0]),Gf.Vec3f(*map(float,v[1:])))
def mul(a,b):
 z=q(a)*q(b);return np.r_[z.GetReal(),z.GetImaginary()]
def inv(a):return np.r_[a[0],-np.array(a[1:])]
def align(axis):
 a=np.array(axis,float);v=np.r_[1+a[0],np.cross([1,0,0],a)];return v/np.linalg.norm(v) if np.linalg.norm(v)>1e-10 else np.array([0,0,0,1])
def shape(s,p,g):
 sz=g['size']
 if g['kind']=='box':x=UsdGeom.Cube.Define(s,p);x.CreateSizeAttr(1);x.AddScaleOp().Set(Gf.Vec3f(*(2*np.array(sz))))
 elif g['kind']=='cylinder':x=UsdGeom.Cylinder.Define(s,p);x.CreateRadiusAttr(sz[0]);x.CreateHeightAttr(2*sz[1]);x.CreateAxisAttr('Z')
 else:x=UsdGeom.Sphere.Define(s,p);x.CreateRadiusAttr(sz[0])
 x.AddTranslateOp().Set(Gf.Vec3d(*g['pos']));x.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(q(g['quat']))
 ops=x.GetOrderedXformOps();x.SetXformOpOrder(sorted(ops,key=lambda o:{UsdGeom.XformOp.TypeTranslate:0,UsdGeom.XformOp.TypeOrient:1,UsdGeom.XformOp.TypeScale:2}[o.GetOpType()]))
 UsdGeom.Gprim(x).CreateDisplayColorAttr([Gf.Vec3f(*g['rgba'][:3])])
 # Native VIS parts remain visual, never converted to force-bearing lugs.
 if g['contype'] or g['conaffinity']:
  UsdPhysics.CollisionAPI.Apply(x.GetPrim());x.GetPrim().AddAppliedSchema('PhysxCollisionAPI');attr(x.GetPrim(),'physxCollision:contactOffset',.0001,Sdf.ValueTypeNames.Float);attr(x.GetPrim(),'physxCollision:restOffset',0.,Sdf.ValueTypeNames.Float)
  mp='/B6/materials/mu_'+str(g['friction'][0]).replace('.','_');m=UsdShade.Material.Define(s,mp);a=UsdPhysics.MaterialAPI.Apply(m.GetPrim());a.CreateStaticFrictionAttr(g['friction'][0]);a.CreateDynamicFrictionAttr(g['friction'][0]);a.CreateRestitutionAttr(0.)
  UsdShade.MaterialBindingAPI.Apply(x.GetPrim()).Bind(m,UsdShade.Tokens.weakerThanDescendants,'physics')
 return x

def author(name,bs,f):
 s=Usd.Stage.CreateNew(str(HERE/'assets'/f'{name}.usda'));UsdGeom.SetStageUpAxis(s,'Z');UsdGeom.SetStageMetersPerUnit(s,1.)
 root=xf(s,'/B6',[0,0,0],[1,0,0,0]);s.SetDefaultPrim(root)
 articulated=any(b['joints'] for b in bs)
 if articulated:
  UsdPhysics.ArticulationRootAPI.Apply(root);root.AddAppliedSchema('PhysxArticulationAPI');attr(root,'physxArticulation:enabledSelfCollisions',True,Sdf.ValueTypeNames.Bool);attr(root,'physxArticulation:solverPositionIterationCount',64,Sdf.ValueTypeNames.Int);attr(root,'physxArticulation:solverVelocityIterationCount',8,Sdf.ValueTypeNames.Int)
 origin=np.array(bs[0]['pos']);initialR=R(bs[0]['quat']);byname={b['name']:b for b in bs}
 # Asset-local world frame retains native orientation. Spawn with identity orientation.
 for b in bs:
  bp='/B6/links/'+b['name'];p=xf(s,bp,initialR.T@(np.array(b['pos'])-origin),mul(inv(bs[0]['quat']),b['quat']))
  if articulated or name.startswith('lens_') or name.startswith('tray_'):
   body_api=UsdPhysics.RigidBodyAPI.Apply(p);p.AddAppliedSchema('PhysxRigidBodyAPI')
   if name.startswith('tray_'):body_api.CreateKinematicEnabledAttr(True)
   ma=UsdPhysics.MassAPI.Apply(p);ma.CreateMassAttr(max(b['mass'],.000001));ma.CreateCenterOfMassAttr(Gf.Vec3f(*b['com']));ma.CreateDiagonalInertiaAttr(Gf.Vec3f(*np.maximum(b['inertia'],1e-10)));ma.CreatePrincipalAxesAttr(quatf(b['inertia_quat']))
  for g in b['geoms']:shape(s,bp+'/'+g['name'],g)
  for j in b['joints']:
   parent=byname[b['parent']];jp='/B6/joints/'+j['name'];joint=(UsdPhysics.PrismaticJoint if j['kind']=='slide' else UsdPhysics.RevoluteJoint).Define(s,jp);joint.CreateAxisAttr('X');joint.CreateBody0Rel().SetTargets([Sdf.Path('/B6/links/'+parent['name'])]);joint.CreateBody1Rel().SetTargets([Sdf.Path(bp)]);joint.CreateCollisionEnabledAttr(False)
   anchor=np.array(b['pos'])+R(b['quat'])@j['anchor'];joint.CreateLocalPos0Attr(Gf.Vec3f(*(R(parent['quat']).T@(anchor-np.array(parent['pos'])))));joint.CreateLocalPos1Attr(Gf.Vec3f(*j['anchor']));frame=align(j['axis']);joint.CreateLocalRot1Attr(quatf(frame));joint.CreateLocalRot0Attr(quatf(mul(mul(inv(parent['quat']),b['quat']),frame)))
   scale=180/math.pi if j['kind']=='hinge' else 1.;joint.CreateLowerLimitAttr(j['range'][0]*scale);joint.CreateUpperLimitAttr(j['range'][1]*scale)
   p=joint.GetPrim();p.AddAppliedSchema('PhysxJointAPI');attr(p,'physxJoint:armature',j['armature'],Sdf.ValueTypeNames.Float)
   axis='angular' if j['kind']=='hinge' else 'linear';p.AddAppliedSchema('PhysxJointAxisAPI:'+axis)
   for key in ['staticFrictionEffort','dynamicFrictionEffort']:attr(p,'physxJointAxis:'+axis+':'+key,j['friction'],Sdf.ValueTypeNames.Float)
   drive=UsdPhysics.DriveAPI.Apply(p,axis);drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(j['stiffness']/scale);drive.CreateDampingAttr(j['damping']/scale);drive.CreateMaxForceAttr(float('inf'));drive.CreateTargetPositionAttr(0.)
 if name.startswith('body_'):
  fixed=UsdPhysics.FixedJoint.Define(s,'/B6/joints/world_fixed');fixed.CreateBody1Rel().SetTargets([Sdf.Path('/B6/links/'+name)])
 # Preserve native contact masks; robots are added to mask exclusions at runtime.
 geoms=[(b,g) for b in bs for g in b['geoms'] if g['contype'] or g['conaffinity']]
 for b,g in geoms:
  rel=UsdPhysics.FilteredPairsAPI.Apply(s.GetPrimAtPath('/B6/links/'+b['name']+'/'+g['name'])).CreateFilteredPairsRel()
  for b2,g2 in geoms:
   if b['name']==b2['name']:continue
   if not ((g['contype']&g2['conaffinity']) or (g2['contype']&g['conaffinity'])):rel.AddTarget(Sdf.Path('/B6/links/'+b2['name']+'/'+g2['name']))
 # Source explicitly excludes zoom-ring/front siblings.
 if name.startswith('lens_'):UsdPhysics.FilteredPairsAPI.Apply(s.GetPrimAtPath('/B6/links/'+name+'_zoomring')).CreateFilteredPairsRel().AddTarget(Sdf.Path('/B6/links/'+name+'_front'))
 s.GetRootLayer().Save()

if __name__=='__main__':
 f=json.loads((HERE/'fixture.json').read_text());bodies=f['bodies']
 for name in ['body_left','body_right','lens_orange','lens_blue','lens_purple','tray_left','tray_right']:
  bs=[next(b for b in bodies if b['name']==name)];names={name}
  for b in bodies:
   if b['parent'] in names:bs.append(b);names.add(b['name'])
  author(name,bs,f)
 print(json.dumps({'assets':7,'physics_steps':0,'backend_validation':False}))
