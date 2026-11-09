"""Exact station static primitive inventory and panel/furniture overlap.
No dynamics or object writes. Metadata comes from the installed scene's station
module and the actual TaskFile objects; validation compares live USD colliders.
"""
from __future__ import annotations
import math,itertools,numpy as np
from yam_arena import station
from yam_arena.ik import quat_xyzw_to_matrix
from yam_arena.task_file import Box
from telemetry_math import box_overlap,segment_box_distance

def box(name,centre,size,R=None):
 c=np.asarray(centre,float);h=np.asarray(size,float)/2;R=np.eye(3) if R is None else np.asarray(R,float)
 if c.shape!=(3,) or h.shape!=(3,) or R.shape!=(3,3) or not np.isfinite(np.r_[c,h,R.ravel()]).all() or np.any(h<=0):raise ValueError('invalid box '+name)
 return {'name':name,'kind':'box','centre':c,'R':R,'half':h}

def capsule(name,centre,radius,height,R):
 c=np.asarray(centre,float);R=np.asarray(R,float);axis=R[:,2];radius=float(radius);half=float(height)/2
 if not np.isfinite(np.r_[c,R.ravel(),radius,half]).all() or radius<=0 or half<0:raise ValueError('invalid capsule '+name)
 return {'name':name,'kind':'capsule','centre':c,'R':R,'radius':radius,'half_height':half,'P':c-axis*half,'Q':c+axis*half}

def station_furniture(top_camera,task_objects):
 result=[box('floor',(0.,0.,station.FLOOR_Z-.01),(200.,200.,.02)),box('table',station.TABLE_CENTER,station.TABLE_SIZE)]
 for name,(kind,size,pos,q) in station.camera_stand(top_camera).items():
  R=quat_xyzw_to_matrix(np.array(q))
  if kind=='box':result.append(box(name,pos,size,R))
  elif kind=='capsule':result.append(capsule(name,pos,size[0],size[1],R))
  else:raise ValueError('unsupported station collider '+name+' '+kind)
 for spec in task_objects:
  if spec.name=='monitor_stand':continue
  if not isinstance(spec,Box) or spec.mass is not None:raise ValueError('untracked additional task object '+spec.name)
  a=spec.yaw;R=np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1.]])
  result.append(box(spec.name,spec.pos,spec.size,R))
 if len({o['name'] for o in result})!=len(result):raise ValueError('duplicate furniture name')
 return result

def panel_contacts(panels,obstacles):
 hits=[]
 for p in panels:
  if p['kind']!='box':raise ValueError('unsupported panel primitive '+p['name'])
  extent=abs(p['R'])@p['half']
  for o in obstacles:
   if o['kind']=='box':oe=abs(o['R'])@o['half']
   elif o['kind']=='capsule':oe=abs(o['R'][:,2])*o['half_height']+o['radius']
   else:raise ValueError('unsupported furniture '+o['name'])
   if np.any(p['centre']+extent<o['centre']-oe) or np.any(o['centre']+oe<p['centre']-extent):continue
   if o['kind']=='box':hit=box_overlap(p['centre'],p['R'],p['half'],o['centre'],o['R'],o['half'])
   else:hit=segment_box_distance(o['P'],o['Q'],p['centre'],p['R'],p['half'])<=o['radius']
   if hit:hits.append((p['name'],o['name']))
 return hits

def serializable(shapes):
 return [{k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in o.items()} for o in shapes]

def validate_live_usd(stage,envroot,shapes,origin=(0,0,0),tolerance=1e-6):
 from pxr import Usd,UsdGeom,UsdPhysics,Gf
 expected={o['name']:o for o in shapes};groups={n:[] for n in expected};unknown=[];origin=np.asarray(origin,float)
 excluded=[envroot+'/'+name for name in ('LeftArm','RightArm','monitor_stand')]
 for p in stage.Traverse(Usd.TraverseInstanceProxies()):
  if not p.HasAPI(UsdPhysics.CollisionAPI) or UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() is False:continue
  path=str(p.GetPath())
  if any(path==x or path.startswith(x+'/') for x in excluded):continue
  name=path[len(envroot)+1:].split('/')[0] if path.startswith(envroot+'/') else None
  if name not in groups:unknown.append(path)
  else:groups[name].append(p)
 if unknown:raise ValueError('untracked external colliders '+str(unknown))
 cache=UsdGeom.XformCache();checks={}
 for name,o in expected.items():
  if len(groups[name])!=1:raise ValueError('expected one collider '+name+' '+str(len(groups[name])))
  p=groups[name][0];M=cache.GetLocalToWorldTransform(p)
  if o['kind']=='box':
   if p.IsA(UsdGeom.Cube):
    size=float(UsdGeom.Cube(p).GetSizeAttr().Get());local=np.array(list(itertools.product((-size/2,size/2),repeat=3)))
   elif p.IsA(UsdGeom.Mesh):local=np.array(UsdGeom.Mesh(p).GetPointsAttr().Get(),float)
   else:raise ValueError('not a live box primitive '+str(p.GetPath()))
   actual=np.array([list(M.Transform(Gf.Vec3d(*v))) for v in local])-origin
   corners=np.array(list(itertools.product((-1.,1.),repeat=3)))*o['half'];wanted=corners@o['R'].T+o['centre'];dist=np.linalg.norm(actual[:,None,:]-wanted[None,:,:],axis=2);error=max(float(dist.min(0).max()),float(dist.min(1).max()))
  else:
   if not p.IsA(UsdGeom.Capsule):raise ValueError('not a live capsule '+str(p.GetPath()))
   g=UsdGeom.Capsule(p);axis=str(g.GetAxisAttr().Get()).upper();assert axis=='Z',axis
   centre=np.array(M.ExtractTranslation())-origin;basis=np.array([list(M.TransformDir(Gf.Vec3d(*v))) for v in np.eye(3)]).T;scales=np.linalg.norm(basis,axis=0)
   if np.ptp(scales)>tolerance:raise ValueError('nonuniform capsule scaling '+name)
   a=basis[:,2]/scales[2];half=float(g.GetHeightAttr().Get())*scales[2]/2;radius=float(g.GetRadiusAttr().Get())*scales[0];P=centre-a*half;Q=centre+a*half
   endpoints=min(max(np.linalg.norm(P-o['P']),np.linalg.norm(Q-o['Q'])),max(np.linalg.norm(P-o['Q']),np.linalg.norm(Q-o['P'])));error=max(float(endpoints),abs(radius-o['radius']))
  if error>tolerance:raise ValueError('live furniture geometry mismatch '+name+' '+str(error))
  checks[name]={'collider':str(p.GetPath()),'max_geometry_error_m':error}
 return {'all_external_colliders_covered':True,'collider_count':len(checks),'checks':checks,'tolerance_m':tolerance}
