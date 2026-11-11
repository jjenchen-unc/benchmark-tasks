"""Read-only full contact vector packet; never supplied to v21 behavior."""
import math
import numpy as np

def force_packet(normal_contacts,force,point,count,start,capacity,sensor_paths,filter_paths):
 sensors=list(sensor_paths);filters=list(filter_paths);shape=(len(sensors),len(filters));counts=np.asarray(count);starts=np.asarray(start)
 if counts.shape!=shape or starts.shape!=shape or not np.issubdtype(counts.dtype,np.integer) or not np.issubdtype(starts.dtype,np.integer):raise ValueError('friction pair tables must be integral sensor/filter grid')
 if np.any(counts<0) or int(counts.sum())>=capacity:raise ValueError('friction capacity invalid')
 f=np.asarray(force);p=np.asarray(point)
 if f.ndim!=2 or p.ndim!=2 or f.shape[1:]!=(3,) or p.shape[1:]!=(3,):raise ValueError('friction world vectors Nx3 required')
 pairs={};used=set()
 def pair(s,t):
  if type(s) is not int or type(t) is not int or not 0<=s<shape[0] or not 0<=t<shape[1]:raise ValueError('normal sensor/filter index invalid')
  return pairs.setdefault((s,t),{'normal':np.zeros(3),'friction':np.zeros(3),'normal_points':0,'friction_patches':[]})
 for row in normal_contacts:
  s=row['sensor_index'];t=row['filter_index'];normal=np.asarray(row['normal_w'],float);value=row['normal_force_n']
  if normal.shape!=(3,) or not np.isfinite(normal).all() or type(value) not in (int,float) or not math.isfinite(value):raise ValueError('invalid normal vector')
  d=pair(s,t);d['normal']+=value*normal;d['normal_points']+=1
 for s,t in np.ndindex(shape):
  n=int(counts[s,t]);i=int(starts[s,t])
  if not n:continue
  if i<0 or i+n>len(f) or i+n>len(p):raise ValueError('friction patch range invalid')
  indices=set(range(i,i+n))
  if used&indices:raise ValueError('friction pair ranges overlap')
  used|=indices
  if not np.isfinite(f[i:i+n]).all() or not np.isfinite(p[i:i+n]).all():raise ValueError('nonfinite active friction patch')
  d=pair(s,t);d['friction']+=np.sum(f[i:i+n],axis=0)
  d['friction_patches']=[{'force_world_N':[float(x) for x in fv],'point_world_m':[float(x) for x in pv]} for fv,pv in zip(f[i:i+n],p[i:i+n])]
 normal=np.zeros(3);friction=np.zeros(3);rows=[]
 for (s,t),d in sorted(pairs.items()):
  normal+=d['normal'];friction+=d['friction'];rows.append({'sensor_index':s,'filter_index':t,'sensor_path':sensors[s],'filter_path':filters[t],'normal_world_N':d['normal'].tolist(),'friction_world_N':d['friction'].tolist(),'total_world_N':(d['normal']+d['friction']).tolist(),'normal_points':d['normal_points'],'friction_patches':d['friction_patches']})
 return {'sensor_paths':sensors,'filter_paths':filters,'normal_world_N':normal.tolist(),'friction_world_N':friction.tolist(),'total_world_N':(normal+friction).tolist(),'pairs':rows,'active_friction_patches':len(used),'friction_pair_counts':counts.tolist(),'friction_pair_starts':starts.tolist()}

def read_packet(contact,dt):
 if type(dt) not in (int,float) or not math.isfinite(dt) or dt<=0:raise ValueError('positive samephysics dt required')
 from contacts import arr
 force,point,count,start=contact.view.get_friction_data(dt)
 return force_packet(contact.last_contacts,arr(force).reshape(-1,3),arr(point).reshape(-1,3),arr(count),arr(start),contact.capacity,contact.selection['sensors'],contact.selection['filters'])
