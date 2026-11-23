"""Author the collar, shank and contact shoe on the stationary left gripper body."""
import numpy as np

def install(root):
    from pxr import Gf,Usd,UsdGeom,UsdPhysics,PhysxSchema
    from scipy.spatial import ConvexHull
    from scipy.spatial.transform import Rotation
    import retainer_tool
    stage=root.GetStage()
    body=next(p for p in Usd.PrimRange(root) if p.GetName()=='gripper' and p.HasAPI(UsdPhysics.RigidBodyAPI))
    rootpath=str(body.GetPath())+'/RetainerTool'
    UsdGeom.Xform.Define(stage,rootpath)
    for name,v,rho in retainer_tool.components():
        hull=ConvexHull(v)
        mesh=UsdGeom.Mesh.Define(stage,rootpath+'/'+name)
        faces=[]
        for tri,eq in zip(hull.simplices,hull.equations):
            a,b,c=v[tri]
            faces.extend(tri if np.cross(b-a,c-a)@eq[:3]>0 else tri[::-1])
        mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in v.tolist()])
        mesh.CreateFaceVertexIndicesAttr([int(i) for i in faces])
        mesh.CreateFaceVertexCountsAttr([3]*len(hull.simplices))
        mesh.CreateSubdivisionSchemeAttr('none')
        mesh.CreateExtentAttr([Gf.Vec3f(*v.min(axis=0).tolist()),Gf.Vec3f(*v.max(axis=0).tolist())])
        mesh.CreateDisplayColorAttr([Gf.Vec3f(.50,.54,.56) if rho>5000 else Gf.Vec3f(.26,.29,.32)])
        prim=mesh.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexHull')
        cfg=PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cfg.CreateContactOffsetAttr(.00015);cfg.CreateRestOffsetAttr(0.)
        PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim).CreateMinThicknessAttr(.00005)
    # Combine the original gripper inertia with the metal solids using the parallel-axis theorem.
    api=UsdPhysics.MassAPI.Apply(body); m0=api.GetMassAttr().Get()
    q=api.GetPrincipalAxesAttr().Get()
    r=Rotation.from_quat(list(q.GetImaginary())+[q.GetReal()]).as_matrix()
    c0=np.array(api.GetCenterOfMassAttr().Get()); i0=r@np.diag(api.GetDiagonalInertiaAttr().Get())@r.T
    if not (m0 and m0>0 and np.isfinite(c0).all() and np.trace(i0)>0):
        raise ValueError('missing original gripper inertial data')
    m1,c1,i1=retainer_tool.mass_properties()
    c=(m0*c0+m1*c1)/(m0+m1); inertia=i0+i1
    for m,dx in [(m0,c0-c),(m1,c1-c)]:inertia+=m*((dx@dx)*np.eye(3)-np.outer(dx,dx))
    vals,axes=np.linalg.eigh(inertia)
    if np.linalg.det(axes)<0:axes[:,0]*=-1
    px,py,pz,pw=Rotation.from_matrix(axes).as_quat().tolist()
    api.CreateMassAttr(float(m0+m1));api.CreateCenterOfMassAttr(Gf.Vec3f(*c.tolist()))
    api.CreateDiagonalInertiaAttr(Gf.Vec3f(*vals.tolist()));api.CreatePrincipalAxesAttr(Gf.Quatf(pw,px,py,pz))
    print('H4_TOOL',body.GetPath(),'kg',m1,'COM',c1.tolist(),flush=True)
