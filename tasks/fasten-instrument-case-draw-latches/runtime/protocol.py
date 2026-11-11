from pathlib import Path
import sys,json,hashlib,math
O=Path(__file__).resolve().parent
from strict_json import dumps
CHANNELS={'clock','joint_state','wrist_force','robot_contact','hook_capture','hook_tension','catch_lock','damage','total_motor_limits'}
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def loads(s):
 def pairs(xs):
  d={}
  for k,v in xs:
   if k in d:raise ValueError('duplicate JSON key')
   d[k]=v
  return d
 def floating(x):
  v=float(x)
  if not math.isfinite(v):raise ValueError('nonfinite')
  return v
 return json.loads(s,object_pairs_hook=pairs,parse_float=floating,parse_constant=lambda x:(_ for _ in ()).throw(ValueError('nonfinite')))
def load(p):return loads(Path(p).read_text())
def number(v):
 if type(v) not in (int,float) or not math.isfinite(v):raise ValueError('strict numeric required')
 return v
def relative(base,n):
 if type(n) is not str or not n or Path(n).is_absolute():raise ValueError('relative path required')
 p=(Path(base)/n).resolve()
 if not p.is_relative_to(Path(base).resolve()) or not p.is_file():raise ValueError('missing/escaped file')
 return p
def check():
 m=load(O/'manifest.json')
 if sha_text(dumps(m['files'],sort_keys=True,separators=(',',':')))!=m['fingerprint']:raise ValueError('fingerprint')
 for n,h in m['files'].items():
  if sha(relative(O,n))!=h:raise ValueError('source mismatch '+n)
 from installed_contract import verify_installed
 verify_installed(load(O/'runtime-source-pins.json'))
 return m
def sha_text(s):return hashlib.sha256(s.encode()).hexdigest()
def fresh(f):
 m=check();f=Path(f);r=load(f/'result.json')
 if r.get('kind')!='ACTUAL_S2_RUNTIME' or r.get('phase')!='replay' or r.get('manifest')!=m or r.get('completed') is not True or r.get('error') is not None or r.get('close_error') is not None:raise ValueError('fresh actual completion required')
 if r.get('source_after')!=m or r.get('promoted_channels')!=[]:raise ValueError('source or credit mismatch')
 rows=[loads(x) for x in (f/'journal.jsonl').read_text().splitlines()]
 if len(rows)<2:raise ValueError('actual journal missing')
 for i,row in enumerate(rows):
  s=row['status']
  if type(s['controls']) is not int or s['controls']!=i or type(s['samples']) is not int or s['samples']!=25*i or abs(number(s['seconds'])-.05*i)>1e-6:raise ValueError('clock/control continuity')
  if number(s['score'])!=0 or s['credit_gate']!=sorted(CHANNELS) or s['measurement_error'] is not None or s['failure'] is not None:raise ValueError('uncredited healthy channels required')
  if number(row['physics_dt'])!=.002 or number(row['control_dt'])!=.05:raise ValueError('native dt')
  if i:
   if type(row['action']) is not list or len(row['action'])!=14:raise ValueError('14D action')
   for x in row['action']:number(x)
 s=rows[-1]['status']
 if not 0<s['controls']<=7964 or number(s['measured_score'])!=1 or number(s['hold_s'])<20-1e-9:raise ValueError('native fullgoal/20s hold absent')
 if r['final_status']!=s or r['controller_result'].get('completed_sequence') is not True:raise ValueError('terminal mismatch')
 media=load(f/'media-finalization.json')
 if media.get('all_encoded') is not True or type(media.get('frames_per_camera')) is not int or media['frames_per_camera']!=len(rows):raise ValueError('actual frame continuity')
 if set(media.get('files',{}))!={'top_cam.mp4','left_cam.mp4','right_cam.mp4'}:raise ValueError('camera inventory')
 for n,row in media['files'].items():
  p=relative(f,n)
  if type(row['bytes']) is not int or row['bytes']!=p.stat().st_size or not 0<row['bytes']<=50_000_000:raise ValueError('media bytes')
 return {'controls':s['controls'],'hold_s':s['hold_s'],'measured_score':s['measured_score']}
def receipt(path):
 m=check();p=Path(path).resolve();r=load(p)
 if r.get('schema')!='s2-v19-root-physical-review' or r.get('approved') is not True or r.get('reviewer')!='ROOT' or r.get('manifest')!=m:raise ValueError('matching ROOT review required')
 if type(r.get('replay')) is not str or Path(r['replay']).is_absolute():raise ValueError('relative replay required')
 f=(p.parent/r['replay']).resolve()
 if not f.is_relative_to(p.parent) or fresh(f)!=r.get('actual_summary'):raise ValueError('fresh matching full replay required')
 hashes={str(x.relative_to(f)):sha(x) for x in f.rglob('*') if x.is_file()}
 if r.get('evidence_sha256')!=hashes:raise ValueError('evidence mismatch')
 if set(r.get('channels',{}))!=CHANNELS:raise ValueError('all native channels required')
 for channel,row in r['channels'].items():
  if row.get('approved') is not True or type(row.get('rationale')) is not str or not row['rationale'].strip() or not row.get('proofs'):raise ValueError('channel review absent '+channel)
  for proof in row['proofs']:
   if sha(relative(p.parent,proof['path']))!=proof['sha256']:raise ValueError('channel proof changed')
 return r
