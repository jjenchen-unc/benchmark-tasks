"""Offline source-bound check of production full-contact-vector sidecar."""
from pathlib import Path
import json,math
import numpy as np

def verify(bundle,vector_path,physics_path,start_control=0):
 bundle=Path(bundle);fixture=json.load(open(bundle/'candidate/fixture.json'));names=('lens_orange','lens_blue','lens_purple');expected={n:{b['name'] for b in fixture['bodies'] if b['name']==n or b['name'].startswith(n+'_')} for n in names};registries={};samples=0;peak_input=0.;previous=None
 def vec(x):
  if type(x) is not list or len(x)!=3 or any(type(y) not in (int,float) or not math.isfinite(y) for y in x):raise ValueError('ownedfinite WORLDVec3 required')
  return np.array(x,float)
 def integer(x):
  if type(x) is not int:raise ValueError('integerforcepairindex required')
  return x
 with Path(physics_path).open() as physics:
  for line in Path(vector_path).open():
   row=json.loads(line);raw=json.loads(next(physics));samples+=1
   if integer(row['control'])!=start_control+(samples-1)//25 or row['control']!=raw['control'] or row['sim_time']!=raw['sim_time'] or type(row['dt']) not in (int,float) or row['dt']!=.002:raise ValueError('rawforcephysicsclock mismatch')
   if previous is not None and abs(row['sim_time']-previous-.002)>1e-8:raise ValueError('forceclock gap')
   previous=row['sim_time']
   if set(row['objects'])!=set(names) or set(row['input_wrenches_used_by_rules_N'])!=set(names):raise ValueError('alllens forcevectors required')
   for name,p in row['objects'].items():
    sensors=p['sensor_paths'];filters=p['filter_paths']
    if type(sensors) is not list or {x.rsplit('/',1)[-1] for x in sensors}!=expected[name] or len(sensors)!=3 or len(set(sensors))!=3 or type(filters) is not list or len(filters)!=18 or len(set(filters))!=18:raise ValueError('actual descendant/robot forcepair registry mismatch')
    registry=(tuple(sensors),tuple(filters))
    if name in registries and registries[name]!=registry:raise ValueError('forceactorregistry changed')
    registries[name]=registry;counts=p['friction_pair_counts'];starts=p['friction_pair_starts']
    if type(counts) is not list or len(counts)!=3 or type(starts) is not list or len(starts)!=3 or any(type(x) is not list or len(x)!=18 for x in counts+starts):raise ValueError('forcegrid3x18 required')
    for x in counts+starts:
     for y in x:integer(y)
    normal=np.zeros(3);friction=np.zeros(3);seen=set();ranges=set();patchcount=0
    for pair in p['pairs']:
     s=integer(pair['sensor_index']);f=integer(pair['filter_index'])
     if not 0<=s<3 or not 0<=f<18 or (s,f) in seen or pair['sensor_path']!=sensors[s] or pair['filter_path']!=filters[f]:raise ValueError('forcepair identity/index mismatch')
     seen.add((s,f));patches=pair['friction_patches'];n=counts[s][f];begin=starts[s][f]
     if type(patches) is not list or len(patches)!=n or n<0 or (n and (begin<0 or begin+n>8192)):raise ValueError('forcepatch range mismatch')
     current=set(range(begin,begin+n))
     if ranges&current:raise ValueError('forcepatchranges overlap')
     ranges|=current;patchcount+=n
     actual=np.sum(np.asarray([vec(x['force_world_N']) for x in patches],dtype=np.float32),axis=0) if patches else np.zeros(3)
     for patch in patches:vec(patch['point_world_m'])
     if not np.allclose(actual,vec(pair['friction_world_N']),rtol=0,atol=1e-10) or not np.allclose(vec(pair['normal_world_N'])+vec(pair['friction_world_N']),vec(pair['total_world_N']),rtol=0,atol=1e-10):raise ValueError('forcepatch WORLDsum mismatch')
     normal+=vec(pair['normal_world_N']);friction+=vec(pair['friction_world_N'])
    if any(n<0 for x in counts for n in x) or sum(sum(x) for x in counts)!=patchcount or patchcount!=integer(p['active_friction_patches']) or patchcount>=8192:raise ValueError('missing/invalidforcepair patches')
    if not np.allclose(normal,vec(p['normal_world_N']),rtol=0,atol=1e-10) or not np.allclose(friction,vec(p['friction_world_N']),rtol=0,atol=1e-10) or not np.allclose(normal+friction,vec(p['total_world_N']),rtol=0,atol=1e-10) or not np.array_equal(vec(p['total_world_N']),vec(row['input_wrenches_used_by_rules_N'][name])):raise ValueError('actualrules forcecache differsfrompairdata')
    peak_input=max(peak_input,float(np.linalg.norm(normal+friction)))
   for field in ['pull_after_rules_N','lift_time_after_rules_s']:
    if set(row[field])!={'body_left','body_right'} or any(type(x) not in (int,float) or not math.isfinite(x) for x in row[field].values()):raise ValueError('force-ruleauditcache invalid')
   if row['captured_after_rules']!={m:d['lens'] for m,d in raw['row']['mounts'].items()}:raise ValueError('force/nativecapturecache mismatch')
  try:next(physics)
  except StopIteration:pass
  else:raise ValueError('force sidecar shorterthannativephysics')
 if samples==0 or samples%25:raise ValueError('completeforcecontrols required')
 return {'status':'PASS_ACTUAL_FULL_VECTOR_RAW_CACHE_BINDINGS','samples':samples,'controls':samples//25,'first_control_index':start_control,'peak_full_robot_force_N':peak_input,'all_lens_descendants_and18robot_filters':True,'own_patch_tables_and_float32_worldsums':True,'input_wrenches_exact_to_recorded_forcepacket':True,'nativecapturecache_agrees':True}
