"""Joint paths for cable pickup using the installed YAM URDF and collision meshes."""
from functools import lru_cache
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import coal
import numpy as np
import pinocchio as pin
import trimesh
from scipy.spatial.transform import Rotation
from yam_arena import YAM_URDF
from yam_arena.constants import READY, TCP_OFFSET, PAD_TIP_PAST_TCP
import layout as L


def bvh(mesh):
    model=coal.BVHModelOBBRSS()
    model.beginModel(len(mesh.faces),len(mesh.vertices))
    vertices=coal.StdVec_Vec3s()
    triangles=coal.StdVec_Triangle()
    vertices.extend(np.asarray(mesh.vertices))
    triangles.extend([coal.Triangle(*map(int,f)) for f in mesh.faces])
    model.addSubModel(vertices,triangles)
    model.endModel()
    return model


@lru_cache(maxsize=1)
def arm_model():
    model=pin.buildModelFromUrdf(str(YAM_URDF))
    xml=ET.parse(YAM_URDF)
    geometry=[]
    for name in ("link2","link3","link4","link5","gripper","tip_left","tip_right"):
        meshes=[]
        for part in xml.findall(f".//link[@name='{name}']/collision"):
            source=Path(YAM_URDF).parent/part.find("geometry/mesh").get("filename")
            mesh=trimesh.load_mesh(source,process=False)
            origin=part.find("origin")
            if origin is not None:
                frame=np.eye(4)
                frame[:3,:3]=Rotation.from_euler("xyz",np.fromstring(origin.get("rpy","0 0 0"),sep=" ")).as_matrix()
                frame[:3,3]=np.fromstring(origin.get("xyz","0 0 0"),sep=" ")
                mesh.apply_transform(frame)
            meshes.append(mesh)
        geometry.append((model.getFrameId(name),bvh(trimesh.util.concatenate(meshes))))
    return model,geometry


def block(bounds):
    low,high=np.array(bounds).reshape(3,2).T
    mesh=trimesh.creation.box(extents=high-low)
    mesh.apply_translation((low+high)/2)
    return mesh


class Planner:
    def __init__(self,spec,base):
        self.base=np.asarray(base)
        self.model,self.geometry=arm_model()
        self.data=self.model.createData()
        self.id=self.model.getFrameId("gripper")
        self.hinge=L.entry_centre(spec).copy()
        self.hinge[1]=L.entry_wall_y(spec)
        self.hinge[2]=L.ENTRY_Z1
        sx=L.BOX_IN_X/2+L.SIDE_WALL-L.ENTRY_X
        zr=L.BOX_RIM_Z-L.ENTRY_Z1
        back=zr-L.BOX_PROUD-L.STUD_H
        bounds=list(L.shell_parts().values())+[
            (sx,sx+L.STUD_T,L.ENTRY_HEIGHT-.490,L.ENTRY_HEIGHT,back,back+L.STUD_H),
            (-.24,sx+L.STUD_T+.018,L.ENTRY_HEIGHT-.015,L.ENTRY_HEIGHT,back-.03,zr+.015),
            (-.39,sx+L.STUD_T,-.25,-.14,-.061,-.043)]
        solids=[block(b) for b in bounds]
        ctr=L.guide_center(spec)-self.hinge
        r=L.DRAPE_RADIUS-L.CABLE_W/2
        support=block((getattr(L,"STOCK_SUPPORT_X0",-.345),0.,-r,getattr(L,"STOCK_SUPPORT_YMAX",r),*getattr(L,"STOCK_SUPPORT_Z",(-.09,.014))));support.apply_translation(ctr);solids.append(support)
        rounded=trimesh.creation.cylinder(radius=r,height=.104,sections=32)
        rounded.apply_translation(ctr+[0.,0.,-.038]);solids.append(rounded)
        self.scene=bvh(trimesh.util.concatenate(solids))
        self.identity=coal.Transform3s()
        self.request=coal.CollisionRequest()
        self.request.security_margin=.0008
        self.rng=np.random.default_rng(12)
        self.lo=self.model.lowerPositionLimit[:6].copy()
        self.hi=self.model.upperPositionLimit[:6].copy()

    def inverse(self,point,rotation,seed):
        target=L.world_point(point)-self.base
        orientation=L.FIXTURE_ROT@rotation
        q=np.r_[np.asarray(seed,float),0.,0.]
        for i in range(65):
            pin.computeJointJacobians(self.model,self.data,q)
            pin.updateFramePlacements(self.model,self.data)
            frame=self.data.oMf[self.id]
            lever=frame.rotation@TCP_OFFSET
            dp=target-frame.translation-lever
            dr=pin.log3(orientation@frame.rotation.T)
            if np.linalg.norm(dp)<.00035 and np.linalg.norm(dr)<.005:return q[:6].copy()
            J=pin.getFrameJacobian(self.model,self.data,self.id,pin.LOCAL_WORLD_ALIGNED)[:,:6].copy()
            J[:3]-=pin.skew(lever)@J[3:]
            J[3:]*=.18
            dq=J.T@np.linalg.solve(J@J.T+.00022*np.eye(6),np.r_[dp,.18*dr])
            q[:6]=np.clip(q[:6]+np.clip(dq,-.25,.25),self.lo,self.hi)
        if np.linalg.norm(dp)<.0025 and np.linalg.norm(dr)<.055:return q[:6].copy()
        return None

    def clear(self,q,half):
        pin.framesForwardKinematics(self.model,self.data,np.r_[q,-half,-half])
        for frame_id,shape in self.geometry:
            frame=self.data.oMf[frame_id]
            point=self.base+frame.translation
            if point[2]<L.TOP-.002:return False
            tf=coal.Transform3s(L.FIXTURE_ROT.T@frame.rotation,L.fixture_point(point)-self.hinge)
            if coal.collide(shape,tf,self.scene,self.identity,self.request,coal.CollisionResult()):return False
        return True

    def segment(self,first,last,half):
        count=max(1,int(math.ceil(np.max(abs(last-first))/.10)))
        return all(self.clear(first+(last-first)*t/count,half) for t in range(1,count+1))

    def path(self,start,end,half):
        if self.segment(start,end,half):return [end]
        # Route out towards the open bay before changing grasp direction.
        trees=[[(np.asarray(start),-1)],[(np.asarray(end),-1)]]
        for k in range(500):
            a,b=trees[k%2],trees[1-k%2]
            aim=b[-1][0] if k%4==0 else self.rng.uniform(self.lo,self.hi)
            parent=min(range(len(a)),key=lambda i:np.linalg.norm(a[i][0]-aim))
            origin=a[parent][0]
            d=aim-origin
            node=origin+d*min(1.,.38/max(1e-8,np.linalg.norm(d)))
            if not self.segment(origin,node,half):continue
            a.append((node,parent))
            closest=min(range(len(b)),key=lambda i:np.linalg.norm(b[i][0]-node))
            if not self.segment(node,b[closest][0],half):continue
            def chain(t,i):
                route=[]
                while i>=0:
                    point,i=t[i];route.append(point)
                return route
            r=chain(a,len(a)-1)[::-1]+chain(b,closest)
            if k%2:r=r[::-1]
            return r[1:]
        return None

    def grasps(self,wire,axis,width,start,inside=False):
        axis=np.asarray(axis)/np.linalg.norm(axis)
        width=width-axis*(width@axis)
        width/=max(np.linalg.norm(width),1e-8)
        thick=np.cross(axis,width)
        options=[]
        for angle in np.deg2rad(np.arange(-180,180,30)):
            jaw=width*math.cos(angle)+thick*math.sin(angle)
            side=np.cross(axis,jaw)
            for sign in (1,-1):
                for lean in (-.38,.26):
                    z=sign*side+lean*axis;z/=np.linalg.norm(z)
                    if (inside and z[2]>-.72) or (not inside and z[2]>.7):continue
                    R=np.column_stack((jaw,np.cross(z,jaw),z))
                    tcp=wire-z*(PAD_TIP_PAST_TCP-.004)
                    half=.5*(L.CABLE_W*abs(jaw@width)+L.CABLE_T*abs(jaw@thick))+.0037
                    for seed in (start,READY):
                        pose=self.inverse(tcp,R,seed)
                        if pose is None or not self.clear(pose,half):continue
                        pre=self.inverse(tcp-z*.035,R,pose)
                        if pre is None or not self.clear(pre,half):continue
                        if not self.segment(pre,pose,half):continue
                        cost=np.linalg.norm(pre-start)+.15*np.linalg.norm(pose-pre)+.45*abs(lean)
                        options.append((cost,R,half,pre,pose))
                        break
        return sorted(options,key=lambda v:v[0])
