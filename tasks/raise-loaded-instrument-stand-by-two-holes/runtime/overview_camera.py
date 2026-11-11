"""Static 640x360 overview camera for Root capture (NOT a model observation; SDK feeds exactly 3 images).
Pure CPU: projects frozen-source geometry incl. moving extremes (legs 25..75mm, plate_y +-30mm, pins -12..0mm)."""
import json,math
from pathlib import Path
import numpy as np
SRC=Path(__file__).resolve().parent.parent/'source'
MOVING={'plate','instrument','legL','legR','pinL','pinR','knobbL','knobbR'}
PINS={'pinL','pinR','knobbL','knobbR'}
W,H=640,360
def _rz(a):c,s=math.cos(a),math.sin(a);return np.array([[c,-s,0],[s,c,0],[0,0,1.]])
def _half(g):
 k,s=g['kind'],g['size']
 if k=='box':return np.array(s[:3])
 r=s[0];h=s[1]+(r if k=='capsule' else 0.);return np.array([max(r,h)]*3)  # orientation-free bound
def source_points():
 f=json.loads((SRC/'fixture.json').read_text());t=f['transform'];R=_rz(t['yaw']);T=np.array(t['translation'])
 byname={b['name']:b for b in f['bodies']};pts=[]
 for b in f['bodies']:
  dzs=(.025,.075) if b['name'] in MOVING else (0.,)
  dys=(-.03,.03) if b['name'] in MOVING else (0.,)
  dxs=(-.012,0.) if b['name'] in PINS else (0.,)
  for g in b['geoms']:
   c=np.array(b['pos'])+np.array(g['pos']);h=_half(g)
   for sx in (-1,1):
    for sy in (-1,1):
     for sz in (-1,1):
      for dz in dzs:
       for dy in dys:
        for dx in dxs:pts.append(R@(c+h*[sx,sy,sz]+[dx,dy,dz])+T)
 return np.array(pts)
def robot_points():
 # Source kit: pedestals (.32,+-.305,.9475) size .22x.24x.40; YAM bases raised +.32x,+.40z onto them.
 # Manipulation volume: pedestal tops up to 0.45 m above (arm reach above stand handles).
 p=[]
 for y in (.305,-.305):
  for sx in (-.11,.11):
   for sy in (-.12,.12):
    for z in (.7475,1.1475,1.5975):p.append((.32+sx,y+sy,z))
 return np.array(p)
def look_at(eye,target,up=(0,0,1.)):
 f=np.asarray(target,float)-eye;f/=np.linalg.norm(f);r=np.cross(f,up);r/=np.linalg.norm(r);u=np.cross(r,f)
 return np.stack([r,u,-f],1)  # OpenGL: columns x right, y up, z back (look along -z)
def project(pts,pos,Rcw,fx,fy,cx,cy):
 c=(np.asarray(pts)-pos)@Rcw;z=-c[:,2]
 return np.stack([cx+fx*c[:,0]/z,cy-fy*c[:,1]/z,z],1)
def coverage(uvz,w=W,h=H,margin=0):
 u,v,z=uvz.T;ok=(z>0)&(u>=margin)&(u<=w-margin)&(v>=margin)&(v<=h-margin)
 return dict(n=len(ok),inside=int(ok.sum()),all_inside=bool(ok.all()),u=[float(u.min()),float(u.max())],v=[float(v.min()),float(v.max())],z_min=float(z.min()))
def mat_to_xyzw(R):
 w=math.sqrt(max(0.,1+R[0,0]+R[1,1]+R[2,2]))/2;x=math.copysign(math.sqrt(max(0.,1+R[0,0]-R[1,1]-R[2,2]))/2,R[2,1]-R[1,2])
 y=math.copysign(math.sqrt(max(0.,1-R[0,0]+R[1,1]-R[2,2]))/2,R[0,2]-R[2,0]);z=math.copysign(math.sqrt(max(0.,1-R[0,0]-R[1,1]+R[2,2]))/2,R[1,0]-R[0,1])
 return (x,y,z,w)
# Front-left elevated static view, opposite the robots (+x), looking at the stand/robot volume centre.
EYE=np.array([1.55,-0.85,1.75]);FOCAL_MM=15.5;APERTURE_MM=20.955
def overview():
 pts=np.vstack([source_points(),robot_points()]);tgt=(pts.min(0)+pts.max(0))/2;R=look_at(EYE,tgt)
 f=FOCAL_MM*W/APERTURE_MM;uvz=project(pts,EYE,R,f,f,W/2,H/2)
 return dict(pos=EYE.tolist(),target=tgt.tolist(),rot_xyzw_opengl=mat_to_xyzw(R),convention='opengl',focal_mm=FOCAL_MM,horizontal_aperture_mm=APERTURE_MM,width=W,height=H,
  fx=f,coverage=coverage(uvz,margin=8),source_coverage=coverage(uvz[:len(source_points())],margin=8),model_observation=False)
