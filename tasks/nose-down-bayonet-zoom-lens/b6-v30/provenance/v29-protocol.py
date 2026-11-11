"""Source-bound ROOT receipt, finite JSON and fresh actual whole-goal evidence."""
from pathlib import Path
import hashlib,json,math,sys
O=Path(__file__).resolve().parent
CHANNELS={'clock','robot_contacts','seat_push','mount_reaction','wrist_force','lock_capture','zoom_helix','reset','motor_limits'}
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def loads(s):
 def pairs(xs):
  d={}
  for k,v in xs:
   if k in d:raise ValueError('duplicate JSON key')
   d[k]=v
  return d
 def ff(x):
  v=float(x)
  if not math.isfinite(v):raise ValueError('nonfinite float')
  return v
 return json.loads(s,object_pairs_hook=pairs,parse_float=ff,parse_constant=lambda x:(_ for _ in ()).throw(ValueError('nonfinite')))
def load(p):return loads(Path(p).read_text())
def number(v):
 if type(v) not in (int,float) or not math.isfinite(v):raise ValueError('finite numeric field required')
 return v
def validate_physics_row(row):
 """Strict JSON ownership before any native History conversion or mutation."""
 from numeric_score import validate_measurement, LENSES
 if type(row) is not dict or set(row)!={'wrist_n','stop_nm','drop_velocity_z','held','captured','robot_touch','measurement','push_n','mounts'}:
  raise ValueError('physics row structure')
 wrist=row['wrist_n']
 if type(wrist) is not list or len(wrist)!=2:raise ValueError('wrist_n list2 required')
 for v in wrist:number(v)
 mounts={'body_left','body_right'};lenses=set(LENSES)
 for field,keys,boolean in [('stop_nm',mounts,False),('drop_velocity_z',lenses,False),('held',lenses,True),('captured',lenses,True)]:
  values=row[field]
  if type(values) is not dict or set(values)!=keys:raise ValueError(field+' mapping structure')
  for v in values.values():
   if boolean:
    if type(v) is not bool:raise ValueError(field+' boolean required')
   else:number(v)
 if type(row['robot_touch']) is not bool:raise ValueError('robot_touch boolean required')
 validate_measurement(row['measurement'])
 pushes=row['push_n'];push_keys={'body_left__lens_orange','body_right__lens_blue','body_right__lens_purple'}
 if type(pushes) is not dict or set(pushes)!=push_keys:raise ValueError('push_n mapping structure')
 for v in pushes.values():number(v)
 if type(row['mounts']) is not dict or set(row['mounts'])!=mounts:raise ValueError('mounts mapping structure')
 for mount in row['mounts'].values():
  if type(mount) is not dict or set(mount)!={'lens','pin','theta'}:raise ValueError('mount structure')
  if mount['lens'] is not None and (type(mount['lens']) is not str or mount['lens'] not in lenses|{'adapter_grey'}):raise ValueError('mount lens')
  if type(mount['pin']) is not bool:raise ValueError('mount pin boolean required')
  number(mount['theta'])
 return row

def relative(base,n):
 if type(n) is not str or not n or Path(n).is_absolute():raise ValueError('relative path required')
 base=Path(base).resolve();p=(base/n).resolve()
 if not p.is_relative_to(base) or not p.is_file():raise ValueError('missing/escaped file')
 return p
def check():
 m=load(O/'manifest.json')
 if hashlib.sha256(json.dumps(m['files'],sort_keys=True,separators=(',',':')).encode()).hexdigest()!=m['fingerprint']:raise ValueError('fingerprint')
 for n,h in m['files'].items():
  if sha(relative(O,n))!=h:raise ValueError('source mismatch '+n)
 native=load(O/'provenance/v14-candidate-sha256.json')
 derivation=load(O/'controller-derivation-v17.json')
 measurement=load(O/'measurement-derivation-v22.json')
 if set(measurement['changed_candidate_sha256'])!={'contacts.py','b6_physics.py'} or set(measurement['added_candidate_sha256'])!={'force_vectors.py'} or measurement['parent_candidate_sha256']!={n:native[n] for n in ['contacts.py','b6_physics.py']}:raise ValueError('v22 measurement lineage scope')
 if derivation['old_candidate_sha256']!=native or set(derivation['changed_candidate_sha256'])!={'reference.py','embedded-input-manifest.json'} or set(derivation['added_candidate_sha256'])!={'endpoint_control.py'}:raise ValueError('v17 controller lineage scope')
 if sha(O/'provenance/v16-reference.py')!=native['reference.py'] or derivation['old_reference_sha256']!=native['reference.py']:raise ValueError('historical controller changed')
 actual={str(p.relative_to(O/'candidate')) for p in (O/'candidate').rglob('*') if p.is_file() and '__pycache__' not in p.parts}
 if actual!=set(native)|{'endpoint_control.py','force_vectors.py'}:raise ValueError('candidate file-set changed')
 for n,h in native.items():
  expected=measurement['changed_candidate_sha256'].get(n,derivation['changed_candidate_sha256'].get(n,h))
  if sha(relative(O/'candidate',n))!=expected:raise ValueError('v17 lineage bytes changed '+n)
 for n,h in (derivation['added_candidate_sha256']|measurement['added_candidate_sha256']).items():
  if sha(relative(O/'candidate',n))!=h:raise ValueError('v17 helper changed '+n)
 embedded=load(O/'candidate/embedded-input-manifest.json')['sha256']
 for n,h in embedded.items():
  if sha(relative(O/'candidate',n))!=h:raise ValueError('embedded source mismatch')
 return m

def fresh(folder,expected_phase=None):
 m=check();f=Path(folder).resolve();r=load(f/'result.json')
 if r.get('kind')!='ACTUAL_B6_PORTABLE' or r.get('phase') not in ('reference','replay') or (expected_phase and r['phase']!=expected_phase):raise ValueError('actual whole reference/replay required')
 if r.get('error') is not None or r.get('close_error') is not None or r.get('source_before')!=m or r.get('source_after')!=m or r.get('completed') is not True:raise ValueError('actual clean source-bound completion required')
 sys.path.insert(0,str(O/'candidate'))
 from mechanics import History
 hist=History();samples=0
 for line in (f/'physics.jsonl').open():
  row=loads(line)
  if type(row) is not dict or set(row)!={'sim_time','control','row','status'}:raise ValueError('trace structure')
  number(row['sim_time'])
  if type(row['control']) is not int:raise ValueError('trace control integer required')
  validate_physics_row(row['row'])
  hist.update(row['row'],.002);samples+=1
  for k,v in hist.result().items():
   if row['status'].get(k)!=v:raise ValueError('native trace mismatch '+k)
  s=row['status'];c=s['controls']
  if type(s['samples']) is not int or s['samples']!=samples or abs(number(s['sim_elapsed_s'])-samples*.002)>1e-8:raise ValueError('numeric native clock')
  number(s['measured_rubric']['score'])
  if type(c) is not int or c!=(samples-1)//25 or row['control']!=c:raise ValueError('physics/control clock mismatch')
  if s['measurement_error'] is not None or s['failure'] is not None or number(s['score'])!=0 or s['missing_runtime_validation']!=sorted(CHANNELS):raise ValueError('invalid/unproved channels')
 if samples==0 or samples%25:raise ValueError('incomplete physics controls')
 actions=load(f/'actions.json');journal=[loads(x) for x in (f/'journal.jsonl').open()];controls=len(actions)
 if type(r['controls']) is not int or r['controls']!=controls or not 0<controls<=3034 or samples!=controls*25 or len(journal)!=controls+1:raise ValueError('budget/clock mismatch')
 for i,row in enumerate(journal):
  if row['control']!=i or type(row['control']) is not int:raise ValueError('journal discontinuity')
  if i:
   a=actions[i-1]
   if a['control']!=i or a['action']!=row['action'] or len(a['action'])!=14:raise ValueError('genuine action mismatch')
   for x in a['action']:number(x)
  if row['status']['samples']!=25*i or row['status']['controls']!=i:raise ValueError('journal clock')
  if row['status']['failure'] is not None or row['status']['measurement_error'] is not None:raise ValueError('failed measurement')
 final=journal[-1]['status']
 if final!=r['final_status'] or number(final['measured_rubric']['score'])!=1 or number(final['history']['hold_full_s'])<20 or hist.result()['measured_rubric']['score']!=1:raise ValueError('fresh complete native goal/20s absent')
 if r['phase']=='reference':
  if r['native_result'].get('raw_measured_goal') is not True or any('failure' in p or 'termination' in p for p in load(f/'phases.json')):raise ValueError('whole controller failed/stopped')
 media=load(f/'media-finalization.json')
 if media['all_encoded'] is not True or type(media['frames_per_camera']) is not int or media['frames_per_camera']!=controls+1:raise ValueError('media continuity')
 for n in ['top_cam.mp4','left_cam.mp4','right_cam.mp4']:
  p=relative(f,n);b=media['files'][n]['bytes']
  if type(b) is not int or b!=p.stat().st_size or not 0<b<=50_000_000:raise ValueError('media size')
 return dict(controls=controls,physics_samples=samples,hold_s=final['history']['hold_full_s'],measured_score=1.,public_score=0.)

def receipt(path):
 m=check();p=Path(path).resolve();r=load(p)
 if r.get('schema')!='b6-v16-root-physical-review' or r.get('approved') is not True or r.get('reviewer')!='ROOT' or r.get('manifest')!=m:raise ValueError('matching ROOT review required')
 for phase in ('reference','replay'):
  name=r.get(phase)
  if type(name) is not str or Path(name).is_absolute():raise ValueError('relative actual evidence required')
  f=(p.parent/name).resolve()
  if not f.is_relative_to(p.parent):raise ValueError('evidence escape')
  if fresh(f,phase)!=r.get(phase+'_summary'):raise ValueError('fresh summary mismatch')
  hashes={str(x.relative_to(f)):sha(x) for x in f.rglob('*') if x.is_file()}
  if hashes!=r.get(phase+'_sha256'):raise ValueError('actual evidence changed')
 if load(p.parent/r['reference']/'actions.json')!=load(p.parent/r['replay']/'actions.json'):raise ValueError('replay differs from genuine reference')
 if r.get('mounted_support_approved') is not True:raise ValueError('fresh support review required')
 decisions=r.get('channels',{})
 if set(decisions)!=CHANNELS:raise ValueError('all nine ROOT channel decisions required')
 for c,row in decisions.items():
  if row.get('approved') is not True or type(row.get('rationale')) is not str or not row['rationale'].strip() or not row.get('proofs'):raise ValueError('unproved channel '+c)
  for proof in row['proofs']:
   if sha(relative(p.parent,proof['path']))!=proof['sha256']:raise ValueError('channel proof changed')
 return r
