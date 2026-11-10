# Visual-only massless cord path. No collider, rigid-body API, forces or scoring writes.
import numpy as np
class CordVisuals:
 def __init__(self):
  from isaaclab.sim.utils.stage import get_current_stage
  from pxr import UsdGeom,Gf
  self.stage=get_current_stage();self.parts={}
  for i in range(4):
   for j in range(3):
    cap=UsdGeom.Capsule.Define(self.stage,f"/World/D1CordVisuals/strap_{i+1}/part_{j}")
    cap.CreateAxisAttr("Z");cap.CreateRadiusAttr(.004);cap.CreateHeightAttr(.001)
    cap.CreateDisplayColorAttr().Set([Gf.Vec3f(.055,.055,.055)])
    trans=next((o for o in cap.GetOrderedXformOps() if o.GetOpType()==UsdGeom.XformOp.TypeTranslate),None)
    if trans is None:trans=cap.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble)
    orient=next((o for o in cap.GetOrderedXformOps() if o.GetOpType()==UsdGeom.XformOp.TypeOrient),None)
    if orient is None:orient=cap.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble)
    self.parts[i,j]=(cap,trans,orient)
 def update(self,i,points):
  from pxr import Gf,UsdGeom
  pts=[np.asarray(p,float) for p in points]
  for j in range(3):
   cap,trans,orient=self.parts[i,j]
   if j>=len(pts)-1:UsdGeom.Imageable(cap.GetPrim()).MakeInvisible();continue
   a,b=pts[j:j+2];v=b-a;L=float(np.linalg.norm(v))
   if not np.isfinite(L) or L<1e-8:UsdGeom.Imageable(cap.GetPrim()).MakeInvisible();continue
   UsdGeom.Imageable(cap.GetPrim()).MakeVisible();cap.GetHeightAttr().Set(L)
   trans.Set(Gf.Vec3d(*((a+b)/2)))
   orient.Set(Gf.Rotation(Gf.Vec3d(0,0,1),Gf.Vec3d(*(v/L))).GetQuat())
