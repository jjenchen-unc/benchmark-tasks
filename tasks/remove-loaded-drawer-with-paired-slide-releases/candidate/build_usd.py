"""Compile task primitives; no simulator imports or physics execution."""
import json,math
from pathlib import Path
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,Gf,Sdf
HERE=Path(__file__).resolve().parent
F=json.loads((HERE/'fixture.json').read_text())
def quat(v,prec='d'):
 return (Gf.Quatd if prec=='d' else Gf.Quatf)(float(v[0]),(Gf.Vec3d if prec=='d' else Gf.Vec3f)(*map(float,v[1:])))
def xf(s,path,pos=(0,0,0),q=(1,0,0,0)):
 x=UsdGeom.Xform.Define(s,path);x.AddTranslateOp().Set(Gf.Vec3d(*pos));x.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(quat(q));return x.GetPrim()
def align(a):
 a=np.asarray(a);q=np.r_[1+a[0],np.cross([1,0,0],a)]
 return q/np.linalg.norm(q) if np.linalg.norm(q)>1e-9 else np.array([0,0,0,1.])
def author(name,body_ids):
 s=Usd.Stage.CreateNew(str(HERE/(name+'.usda')));UsdGeom.SetStageUpAxis(s,'Z');UsdGeom.SetStageMetersPerUnit(s,1.)
 p=xf(s,'/Asset');s.SetDefaultPrim(p);rootbody=F['bodies'][body_ids[0]];art=name.startswith('drawer');fixed=name=='carcasses'
 if art:UsdPhysics.ArticulationRootAPI.Apply(p)
 for i in body_ids:
  b=F['bodies'][i];pos=b['pos'] if i!=body_ids[0] else [0,0,0];bp='/Asset/'+b['name'];p=xf(s,bp,pos,b['quat'])
  if not fixed:
   UsdPhysics.RigidBodyAPI.Apply(p);ma=UsdPhysics.MassAPI.Apply(p);ma.CreateMassAttr(b['mass']);ma.CreateCenterOfMassAttr(Gf.Vec3f(*b['com']));ma.CreateDiagonalInertiaAttr(Gf.Vec3f(*b['inertia']));ma.CreatePrincipalAxesAttr(quat(b['inertia_quat'],'f'))
  for g in b['geoms']:
   gp=bp+'/'+g['name'].replace('-', '_minus_');sz=g['size']
   if g['kind']=='box':
    sh=UsdGeom.Cube.Define(s,gp);sh.CreateSizeAttr(1.);sh.AddScaleOp().Set(Gf.Vec3f(*(2*np.array(sz))))
   elif g['kind']=='sphere':sh=UsdGeom.Sphere.Define(s,gp);sh.CreateRadiusAttr(sz[0])
   else:
    sh=UsdGeom.Cylinder.Define(s,gp);sh.CreateRadiusAttr(sz[0]);sh.CreateHeightAttr(sz[1]*2);sh.CreateAxisAttr('Z')
   sh.AddTranslateOp().Set(Gf.Vec3d(*g['pos']));sh.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(quat(g['quat']));ops=sh.GetOrderedXformOps();sh.SetXformOpOrder(sorted(ops,key=lambda o:{UsdGeom.XformOp.TypeTranslate:0,UsdGeom.XformOp.TypeOrient:1,UsdGeom.XformOp.TypeScale:2}[o.GetOpType()]))
   UsdGeom.Gprim(sh).CreateDisplayColorAttr([Gf.Vec3f(*g['rgba'][:3])]);UsdGeom.Gprim(sh).CreateDisplayOpacityAttr([g['rgba'][3]])
   if g['rgba'][3]==0:UsdGeom.Imageable(sh).CreateVisibilityAttr('invisible')
   if g['contype'] or g['conaffinity']:
    c=UsdPhysics.CollisionAPI.Apply(sh.GetPrim());c.CreateCollisionEnabledAttr(True)
    for k,v in [('contactOffset',.0005),('restOffset',0.)]:sh.GetPrim().CreateAttribute('physxCollision:'+k,Sdf.ValueTypeNames.Float).Set(v)
    sh.GetPrim().AddAppliedSchema('PhysxCollisionAPI')
    mu=g['friction'][0];mat=UsdShade.Material.Define(s,'/Asset/materials/mu_'+str(mu).replace('.','_'));pm=UsdPhysics.MaterialAPI.Apply(mat.GetPrim());pm.CreateStaticFrictionAttr(mu);pm.CreateDynamicFrictionAttr(mu);pm.CreateRestitutionAttr(0.)
    UsdShade.MaterialBindingAPI.Apply(sh.GetPrim()).Bind(mat,UsdShade.Tokens.weakerThanDescendants,'physics')
  for j in b['joints']:
   if j['type']==0:continue
   joint=UsdPhysics.PrismaticJoint.Define(s,'/Asset/joints/'+j['name']);joint.CreateBody0Rel().SetTargets([Sdf.Path('/Asset/'+F['bodies'][b['parent']]['name'])]);joint.CreateBody1Rel().SetTargets([Sdf.Path(bp)]);joint.CreateAxisAttr('X');joint.CreateLocalPos0Attr(Gf.Vec3f(*b['pos']));joint.CreateLocalPos1Attr(Gf.Vec3f(0));joint.CreateLocalRot0Attr(quat(align(j['axis']),'f'));joint.CreateLocalRot1Attr(quat(align(j['axis']),'f'));joint.CreateLowerLimitAttr(j['range'][0]);joint.CreateUpperLimitAttr(j['range'][1]);joint.CreateCollisionEnabledAttr(False)
   drive=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'linear');drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(j['stiffness']);drive.CreateDampingAttr(j['damping']);drive.CreateTargetPositionAttr(j['springref']);drive.CreateMaxForceAttr(float('inf'))
 s.GetRootLayer().Save()
 from drawers_math import rotation,extent
 bounds=[]
 for i in body_ids:
  b=F['bodies'][i];bp=np.array([0.,0.,0.]) if i==body_ids[0] else np.array(b['pos'])
  R=rotation(np.r_[b['quat'][1:],b['quat'][0]])
  for g in b['geoms']:
   c=bp+R@np.array(g['pos']);Q=R@rotation(np.r_[g['quat'][1:],g['quat'][0]]);e=extent(g['kind'],np.array(g['size']),Q);bounds.append((c-e,c+e))
 size=(np.max([v[1] for v in bounds],axis=0)-np.min([v[0] for v in bounds],axis=0)).tolist()
 return {'size':size,'name':name,'body_ids':body_ids,'native_pos':rootbody['pos'],'articulation':art,'fixed':fixed,'mass':sum(F['bodies'][i]['mass'] for i in body_ids)}
if __name__=='__main__':
 # Static cabinets and world rails share a local frame at the native origin.
 fixed=[i for i,b in enumerate(F['bodies']) if i>0 and not b['joints'] and b['parent']==0]
 # World body includes rail geometry. Cabinet static poses are all zero.
 specs=[author('carcasses',[0]+fixed)]
 for i,b in enumerate(F['bodies']):
  if not b['joints'] or b['joints'][0]['type']!=0:continue
  ids=[i]+[k for k,c in enumerate(F['bodies']) if c['parent']==i and k!=i]
  specs.append(author(b['name'],ids))
 (HERE/'assets.json').write_text(json.dumps(specs,indent=2)+'\n');print(json.dumps({'assets':len(specs),'physics_executed':False}))
