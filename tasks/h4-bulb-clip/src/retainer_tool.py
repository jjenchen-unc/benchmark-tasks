"""Split collar and narrow two-face shoe on the stationary left YAM gripper boss.

All coordinates are in the stock gripper frame. The flat shoe supports the rear
and the outboard side of the central vertical wire; the opposing faces are open.
"""
import math
import numpy as np

# Millimetres, measured from the gripper link origin. The two flat bearing faces
# are 0.8 mm outside this line, the radius of the modeled wire.
TIP=np.array([0.,-.106,.040])
AXIAL=np.array([0.,-1.,0.])
LATERAL=np.array([-1.,0.,0.])
CROSS=np.array([0.,0.,1.])

def components():
    from layout import box_pts,annular_sector
    # 57.1 ID split clamp on the 57.0 mm stationary cylindrical boss in the URDF.
    for seg in range(16):
        a=seg*22.5+(0.45 if seg in (0,8) else 0)
        b=(seg+1)*22.5-(0.45 if seg in (7,15) else 0)
        v=annular_sector(.02855,.033,a,b,.034,.046,n=5)
        yield f'collar_{seg}',v[:,[1,2,0]],2700.
    def block(name,ctr,size,rho):
        half=np.asarray(size)/2
        yield name,box_pts(np.asarray(ctr)-half,np.asarray(ctr)+half),rho
    # Aluminium clamping ears on both sides of the split; threaded hardware is
    # represented by the closed joint between the halves.
    for x in (-.034,.034):
        for y in (-.003,.003):
            yield from block(f'ear_{x}_{y}'.replace('-','m').replace('.','p'),[x,y,.040],[.006,.0056,.012],2700.)
    yield from block('steel_shank',[0.,-.0615,.040],[.006,.063,.005],7850.)
    yield from block('steel_nose',[0.,-.09775,.040],[.003,.0105,.003],7850.)
    def end_block(name,u0,u1,v0,v1):
        centre=TIP+AXIAL*(u0+u1)/2+LATERAL*(v0+v1)/2
        yield from block(name,centre,[v1-v0,u1-u0,.003],7850.)
    yield from end_block('shoe_back',-.0045,-.0008,-.0028,.0018)
    yield from end_block('shoe_side',-.0008,.0009,-.0028,-.0008)

def mass_properties():
    from scipy.spatial import ConvexHull
    mass=0.
    first=np.zeros(3)
    raw=np.zeros((3,3))
    for _,verts,rho in components():
        centre=verts.mean(axis=0)
        for face in ConvexHull(verts).simplices:
            corners=np.vstack([centre,verts[face]])
            a,b,c=corners[1:]-centre
            m=rho*abs(a@np.cross(b,c))/6
            total=corners.sum(axis=0)
            mass+=m;first+=m*total/4
            raw+=m*(corners.T@corners+np.outer(total,total))/20
    centre=first/mass
    central=raw-mass*np.outer(centre,centre)
    return mass,centre,np.eye(3)*np.trace(central)-central
