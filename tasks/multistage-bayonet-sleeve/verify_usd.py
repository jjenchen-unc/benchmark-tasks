"""Pure pxr audit of actual authored shape dimensions/transforms/materials."""
import json,math
from pathlib import Path
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics
HERE=Path(__file__).resolve().parent

def run():
 f=json.loads((HERE/'fixture.json').read_text());checks={};counts={}
 for kind in ('fixture','sleeve'):
  stage=Usd.Stage.Open(str(HERE/('coupling-'+kind+'.usda')));prims=[p for p in stage.Traverse() if p.IsA(UsdGeom.Cube) or p.IsA(UsdGeom.Cylinder)];counts[kind]=len(prims)
  expected=[g for b in f['bodies'] if (b['name']=='sleeve_C')==(kind=='sleeve') for g in b['geoms']];checks[kind+'_all_shapes']=len(prims)==len(expected)
  for g in expected:
   safe=g['name'].replace('+','p').replace('-','m').replace('.','_');ps=[p for p in prims if p.GetName().endswith('_'+safe)]
   if len(ps)!=1:raise AssertionError(g['name'])
   p=ps[0];s=g['size'];ops={o.GetOpType():o.Get() for o in UsdGeom.Xformable(p).GetOrderedXformOps()};assert np.allclose(ops[UsdGeom.XformOp.TypeTranslate],g['pos'],atol=1e-10),g['name'];q=ops[UsdGeom.XformOp.TypeOrient];assert np.allclose([q.GetReal(),*q.GetImaginary()],g['quat'],atol=1e-10)
   if g['type']=='box':assert np.allclose(ops[UsdGeom.XformOp.TypeScale],2*np.array(s),atol=1e-8)
   else:assert abs(UsdGeom.Cylinder(p).GetRadiusAttr().Get()-s[0])<1e-10 and abs(UsdGeom.Cylinder(p).GetHeightAttr().Get()-2*s[1])<1e-10
   assert bool(p.HasAPI(UsdPhysics.CollisionAPI))==g['collision']
  checks[kind+'_dimensions_rotations_collision_exact']=True
  checks[kind+'_no_external_asset_references']=not stage.GetRootLayer().subLayerPaths and not stage.GetRootLayer().GetExternalReferences()
 fixture=Usd.Stage.Open(str(HERE/'coupling-fixture.usda'));j=UsdPhysics.PrismaticJoint(fixture.GetPrimAtPath('/Asset/plunger_slide_C'));checks['source_plunger_range']=abs(j.GetLowerLimitAttr().Get()+.032)<1e-8 and j.GetUpperLimitAttr().Get()==0;checks['source_plunger_axis']=j.GetAxisAttr().Get()=='Z';checks['physical_fixed_tray_floor']=fixture.GetPrimAtPath('/Asset/tray_floor').HasAPI(UsdPhysics.RigidBodyAPI)
 stage=Usd.Stage.Open(str(HERE/'coupling-sleeve.usda'));checks['source_sleeve_mass']=abs(UsdPhysics.MassAPI(stage.GetPrimAtPath('/Asset')).GetMassAttr().Get()-.25)<1e-9
 return dict(kind='static pxr asset audit; no physics or rendering',passed=all(checks.values()),checks=checks,shape_counts=counts)
if __name__=='__main__':
 r=run();print(json.dumps(r,indent=2));raise SystemExit(0 if r['passed'] else 1)
