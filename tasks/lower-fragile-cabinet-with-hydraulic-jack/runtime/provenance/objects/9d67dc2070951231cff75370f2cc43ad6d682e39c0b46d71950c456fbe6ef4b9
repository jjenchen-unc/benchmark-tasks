"""Offline pxr audit of authored physical geometry and joints. No simulation."""
import json
from pathlib import Path
import numpy as np
from pxr import Gf,Usd,UsdGeom,UsdPhysics,UsdShade
P=Path(__file__).resolve().parent;F=json.loads((P/'fixture.json').read_text());checks={};details={}
def geometry(stage,path,g,expected):
 prim=stage.GetPrimAtPath(path)
 if g['collision']:
  binding=UsdShade.MaterialBindingAPI(prim).GetDirectBindingRel('physics').GetTargets()
  checks[g['name']+'_material_inside_reference']=bool(binding) and all(str(v).startswith('/Fixture/') and stage.GetPrimAtPath(v).IsValid() for v in binding)
 M=UsdGeom.XformCache().GetLocalToWorldTransform(prim);centre=np.array(M.Transform(Gf.Vec3d(0,0,0)))
 checks[g['name']+'_centre_exact']=bool(np.allclose(centre,expected,atol=1e-7))
 if g['kind']=='box':
  extent=np.array([np.linalg.norm(np.array(M.TransformDir(Gf.Vec3d(*v)))) for v in np.eye(3)])
  checks[g['name']+'_size_exact']=bool(np.allclose(extent,2*np.array(g['size']),atol=1e-7))
 else:
  o=(UsdGeom.Capsule if g['kind']=='capsule' else UsdGeom.Cylinder)(prim)
  checks[g['name']+'_size_exact']=abs(o.GetRadiusAttr().Get()-g['size'][0])<1e-7 and abs(o.GetHeightAttr().Get()-2*g['size'][1])<1e-7

for tag in ('S','T'):
 stage=Usd.Stage.Open(str(P/('jack_'+tag+'.usda')));bodies=[b for b in F['bodies'] if b['name'].endswith('_'+tag)];pairs=[]
 for b in bodies:
  p=stage.GetPrimAtPath('/Fixture/links/'+b['name']);checks[b['name']+'_rigid']=p.HasAPI(UsdPhysics.RigidBodyAPI)
  for g in b['geoms']:
   root=next(v for v in bodies if v['name']=='jack_'+tag);origin=np.array(root['world_pos']);origin[2]=.8
   expected=np.array(b['world_pos'])-origin+np.array(g['pos'])-(np.array([0,0,.8]) if b is root else 0.)
   # Root body frame is rebased to table top; its geometric local coordinates are too.
   if b is root:expected=np.array(g['pos'])-np.array([0,0,.8])
   geometry(stage,'/Fixture/links/'+b['name']+'/'+g['name'],g,expected)
   p=stage.GetPrimAtPath('/Fixture/links/'+b['name']+'/'+g['name']);checks[g['name']+'_collision']=p.HasAPI(UsdPhysics.CollisionAPI)==g['collision']
  for j in b['joints']:
   p=stage.GetPrimAtPath('/Fixture/joints/'+j['name']);checks[j['name']+'_real_joint']=p.IsA(UsdPhysics.Joint)
   checks[j['name']+'_connected']=len(UsdPhysics.Joint(p).GetBody0Rel().GetTargets())==len(UsdPhysics.Joint(p).GetBody1Rel().GetTargets())==1
 details[tag]={'prims':sum(1 for p in stage.Traverse()),'bodies':len(bodies),'geoms':sum(len(b['geoms']) for b in bodies),'joints':[j['name'] for b in bodies for j in b['joints']]}
stage=Usd.Stage.Open(str(P/'cabinet.usda'));p=stage.GetDefaultPrim();b=next(b for b in F['bodies'] if b['name']=='cabinet');checks['cabinet_free']=p.HasAPI(UsdPhysics.RigidBodyAPI) and not any(p.IsA(UsdPhysics.FixedJoint) for p in stage.Traverse());checks['cabinet_mass_exact']=abs(UsdPhysics.MassAPI(p).GetMassAttr().Get()-b['mass'])<1e-6
for g in b['geoms']:geometry(stage,'/Fixture/'+g['name'],g,np.array(g['pos']))
checks['no_panda_or_native_asset_dependencies']=all(not any('panda' in a.lower() or 'robosuite' in a.lower() or 'mujoco' in a.lower() for a in s.GetRootLayer().GetExternalReferences()) for s in (Usd.Stage.Open(str(P/(n+'.usda'))) for n in ('jack_S','jack_T','cabinet')))
for name in ('jack_S','jack_T','cabinet'):
 composed=Usd.Stage.CreateInMemory();root=composed.DefinePrim('/World/task','Xform');root.GetReferences().AddReference(str(P/(name+'.usda')))
 resolved=[]
 for prim in Usd.PrimRange(root):
  for rel in prim.GetRelationships():
   if rel.GetName().startswith(('physics:body','material:binding')):
    resolved.extend((str(t),bool(composed.GetPrimAtPath(t))) for t in rel.GetTargets())
 checks[name+'_composed_targets_resolve']=bool(resolved) and all(v for _,v in resolved)

print(json.dumps({'kind':'pxr asset authoring audit; not backend proof','checks':checks,'details':details,'all_pass':all(checks.values())},indent=2));raise SystemExit(0 if all(checks.values()) else 1)
