"""USD structural checks only: no physics/app/GPU."""
import json,math
from pathlib import Path
import numpy as np
from pxr import Usd,UsdGeom,UsdPhysics
HERE=Path(__file__).resolve().parent
s=Usd.Stage.Open(str(HERE/'latch.usda'));R='/CompressionLatch';root=s.GetPrimAtPath(R);f=json.loads((HERE/'fixture.json').read_text());c=json.loads((HERE/'compiled-fixture.json').read_text());cache=UsdGeom.XformCache();checks={}
checks['articulation']=root.HasAPI(UsdPhysics.ArticulationRootAPI)
checks['all_source_shapes']=all(s.GetPrimAtPath(R+'/links/'+c['names']['body'][bi]+'/'+name).IsValid() for name,bi in zip(c['names']['geom'],c['geom_bodyid']))
checks['root_body_frame_zero']=np.linalg.norm(np.array(cache.GetLocalToWorldTransform(s.GetPrimAtPath(R+'/links/enclosure')).ExtractTranslation()))<1e-12
checks['frame_height130mm']=abs(float(cache.GetLocalToWorldTransform(s.GetPrimAtPath(R+'/links/enclosure/flange')).ExtractTranslation()[2])+.00125-.13)<1e-9
checks['lid_native_gap']=abs(float(cache.GetLocalToWorldTransform(s.GetPrimAtPath(R+'/links/lid')).ExtractTranslation()[2])-.1368)<1e-9
checks['four_joints']=sum(p.IsA(UsdPhysics.PrismaticJoint) or p.IsA(UsdPhysics.RevoluteJoint) for p in s.Traverse())==4
checks['lid_stiffness']=abs(UsdPhysics.DriveAPI(s.GetPrimAtPath(R+'/joints/lid_slide'),'linear').GetStiffnessAttr().Get()-f['spec']['k'])<.01
checks['flange_collides']=s.GetPrimAtPath(R+'/links/enclosure/flange').HasAPI(UsdPhysics.CollisionAPI)
checks['gasket_visual_only']=not s.GetPrimAtPath(R+'/links/enclosure/gasket_strip').HasAPI(UsdPhysics.CollisionAPI)
checks['pawl_collides']=s.GetPrimAtPath(R+'/links/pawl/pawl_tip').HasAPI(UsdPhysics.CollisionAPI)
checks['clockwise_axis']=abs(UsdPhysics.RevoluteJoint(s.GetPrimAtPath(R+'/joints/handle')).GetLocalRot1Attr().Get().GetReal())<1e-9
checks['no_native_asset_paths']=not s.GetRootLayer().externalReferences
checks={k:bool(v) for k,v in checks.items()}
report={'kind':'static USD structure, zero physics steps','checks':checks,'passed':all(checks.values())};print(json.dumps(report,indent=2));assert report['passed'],report
