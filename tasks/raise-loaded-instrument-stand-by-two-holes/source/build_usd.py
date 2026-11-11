"""Offline pxr compiler of source primitives; no simulator imports or stepping."""
import json,math
from pathlib import Path
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics,UsdShade,Gf,Sdf
HERE=Path(__file__).resolve().parent
F=json.loads((HERE/'fixture.json').read_text())
def q(v):return Gf.Quatd(float(v[0]),Gf.Vec3d(*map(float,v[1:])))
def xf(stage,path,p,r=(1,0,0,0)):
 x=UsdGeom.Xform.Define(stage,path);x.AddTranslateOp().Set(Gf.Vec3d(*map(float,p)));x.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(q(r));return x.GetPrim()
def custom(p,n,v,t):p.CreateAttribute(n,t,custom=False).Set(v)
def build():
 stage=Usd.Stage.CreateNew(str(HERE/'stand.usda'));UsdGeom.SetStageUpAxis(stage,'Z');UsdGeom.SetStageMetersPerUnit(stage,1)
 root=xf(stage,'/Pins',[0,0,0]);stage.SetDefaultPrim(root);UsdPhysics.ArticulationRootAPI.Apply(root);root.AddAppliedSchema('PhysxArticulationAPI');custom(root,'physxArticulation:enabledSelfCollisions',True,Sdf.ValueTypeNames.Bool)
 custom(root,'physxArticulation:solverPositionIterationCount',64,Sdf.ValueTypeNames.Int);custom(root,'physxArticulation:solverVelocityIterationCount',8,Sdf.ValueTypeNames.Int)
 origin=np.array([0,0,.8]);bodies={b['name']:b for b in F['bodies']};paths={};positions={};shapes=[];joint_records=[]
 def rigid(n,pos,mass,com=(0,0,0),inertia=(1e-10,1e-10,1e-10),iq=(1,0,0,0)):
  p=xf(stage,'/Pins/links/'+n,pos);paths[n]=p.GetPath();positions[n]=np.array(pos);UsdPhysics.RigidBodyAPI.Apply(p);p.AddAppliedSchema('PhysxRigidBodyAPI');custom(p,'physxRigidBody:maxDepenetrationVelocity',.5,Sdf.ValueTypeNames.Float)
  ma=UsdPhysics.MassAPI.Apply(p);ma.CreateMassAttr(mass);ma.CreateCenterOfMassAttr(Gf.Vec3f(*com));ma.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia));ma.CreatePrincipalAxesAttr(Gf.Quatf(iq[0],Gf.Vec3f(*iq[1:])))
 rigid('root',[0,0,0],1e-6)
 for n in ['plate_y_frame','plate_z_frame']:rigid(n,np.array(bodies['plate']['pos'])-origin,1e-6)
 for b in F['bodies']:
  n=b['name'];p=np.array(b['pos'])-origin
  if b['mass']>0:rigid(n,p,b['mass'],b['com'],b['inertia'],b['iquat']);prim=stage.GetPrimAtPath(paths[n])
  else:prim=xf(stage,'/Pins/static/'+n,p);paths[n]=prim.GetPath();positions[n]=p
  for g in b['geoms']:
   path=str(paths[n])+'/'+g['name'];sz=g['size'];typ=g['kind']
   if typ=='box':s=UsdGeom.Cube.Define(stage,path);s.CreateSizeAttr(1);s.AddScaleOp().Set(Gf.Vec3f(*(2*np.array(sz))))
   elif typ=='cylinder':s=UsdGeom.Cylinder.Define(stage,path);s.CreateRadiusAttr(sz[0]);s.CreateHeightAttr(2*sz[1]);s.CreateAxisAttr('Z')
   else:s=UsdGeom.Capsule.Define(stage,path);s.CreateRadiusAttr(sz[0]);s.CreateHeightAttr(2*sz[1]);s.CreateAxisAttr('Z')
   s.AddTranslateOp().Set(Gf.Vec3d(*g['pos']));s.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(q(g['quat']));ops=s.GetOrderedXformOps();s.SetXformOpOrder([o for o in ops if o.GetOpType()==UsdGeom.XformOp.TypeTranslate]+[o for o in ops if o.GetOpType()==UsdGeom.XformOp.TypeOrient]+[o for o in ops if o.GetOpType()==UsdGeom.XformOp.TypeScale]);s.CreateDisplayColorAttr([Gf.Vec3f(*g['rgba'][:3])]);s.CreateDisplayOpacityAttr([g['rgba'][3]])
   if any(g['mask']):
    UsdPhysics.CollisionAPI.Apply(s.GetPrim());s.GetPrim().AddAppliedSchema('PhysxCollisionAPI');custom(s.GetPrim(),'physxCollision:contactOffset',.0001,Sdf.ValueTypeNames.Float);custom(s.GetPrim(),'physxCollision:restOffset',0.,Sdf.ValueTypeNames.Float)
    mat=UsdShade.Material.Define(stage,'/Pins/materials/'+g['name']);mp=UsdPhysics.MaterialAPI.Apply(mat.GetPrim());mp.CreateStaticFrictionAttr(g['friction'][0]);mp.CreateDynamicFrictionAttr(g['friction'][0]);mp.CreateRestitutionAttr(0.);UsdShade.MaterialBindingAPI.Apply(s.GetPrim()).Bind(mat,materialPurpose='physics');shapes.append((s.GetPrim(),n,g))
 # Exact source masks and explicit exclusions, including pin-only hidden hole walls.
 for a,an,ag in shapes:
  rel=UsdPhysics.FilteredPairsAPI.Apply(a).CreateFilteredPairsRel()
  for c,cn,cg in shapes:
   eligible=(ag['mask'][0]&cg['mask'][1]) or (cg['mask'][0]&ag['mask'][1])
   excluded=(an.startswith('leg') and cn in ['plate','slotR','inner'+an[-1]]) or (cn.startswith('leg') and an in ['plate','slotR','inner'+cn[-1]])
   if a!=c and (not eligible or excluded):rel.AddTarget(c.GetPath())
 def joint(n,parent,child,j=None,ball=False,world=False,anchor_child=(0,0,0),anchor_parent=None):
  cls=UsdPhysics.SphericalJoint if ball else UsdPhysics.FixedJoint if j is None else UsdPhysics.PrismaticJoint if j['kind']=='slide' else UsdPhysics.RevoluteJoint
  o=cls.Define(stage,'/Pins/joints/'+n)
  if not world:o.CreateBody0Rel().SetTargets([paths[parent]])
  o.CreateBody1Rel().SetTargets([paths[child]])
  a1=np.array(anchor_child);a0=positions[child]+a1-(positions[parent] if not world else 0) if anchor_parent is None else np.array(anchor_parent)
  o.CreateLocalPos0Attr(Gf.Vec3f(*map(float,a0)));o.CreateLocalPos1Attr(Gf.Vec3f(*map(float,a1)))
  if ball:o.CreateExcludeFromArticulationAttr(True);return
  if j:
   axis='XYZ'[int(np.argmax(np.abs(j['axis'])))];o.CreateAxisAttr(axis);fac=1 if j['kind']=='slide' else 180/math.pi
   if j['limited']:o.CreateLowerLimitAttr(j['range'][0]*fac);o.CreateUpperLimitAttr(j['range'][1]*fac)
   drv=UsdPhysics.DriveAPI.Apply(o.GetPrim(),'linear' if j['kind']=='slide' else 'angular');drv.CreateTypeAttr('force');drv.CreateStiffnessAttr(j['K']/fac);drv.CreateDampingAttr(j['D']/fac);drv.CreateTargetPositionAttr(j['ref']*fac)
   o.GetPrim().AddAppliedSchema('PhysxJointAPI');o.GetPrim().AddAppliedSchema('PhysxJointAxisAPI:'+('linear' if j['kind']=='slide' else 'angular'));custom(o.GetPrim(),'physxJointAxis:'+('linear' if j['kind']=='slide' else 'angular')+':staticFrictionEffort',j['friction'],Sdf.ValueTypeNames.Float);custom(o.GetPrim(),'physxJointAxis:'+('linear' if j['kind']=='slide' else 'angular')+':dynamicFrictionEffort',j['friction'],Sdf.ValueTypeNames.Float);custom(o.GetPrim(),'physxJoint:armature',j['armature'],Sdf.ValueTypeNames.Float)
   joint_records.append(dict(j,parent=parent,child=child))
 joint('world_fixed','root','root',world=True)
 for b in F['bodies']:
  n=b['name']
  if b['mass']<=0:continue
  if n=='plate':
   for j,pa,ch in zip(b['joints'],['root','plate_y_frame','plate_z_frame'],['plate_y_frame','plate_z_frame','plate']):joint(j['name'],pa,ch,j)
  else:
   pa=b['parent'] or 'root';joint(b['joints'][0]['name'] if b['joints'] else n+'_fixed',pa,n,b['joints'][0] if b['joints'] else None)
 for leg in 'LR':joint('join'+leg,'plate' if leg=='L' else 'slotR','leg'+leg,ball=True,anchor_child=(0,0,.17),anchor_parent=(-.06,-.15,-.01) if leg=='L' else (0,0,0))
 stage.GetRootLayer().Save();(HERE/'joints.json').write_text(json.dumps(joint_records,indent=2));print(json.dumps(dict(prims=sum(1 for p in stage.Traverse()),bodies=len(paths),joints=len(joint_records),physics_steps=0)))
if __name__=='__main__':build()
