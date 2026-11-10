"""Portable source/evidence contract. A local ROOT review is not a signature service."""
from pathlib import Path
import json,hashlib,math
from strict_json import dumps
O=Path(__file__).resolve().parent
CHANNELS={'geometry','mechanism','clock','contacts','wrist_force','motor_caps','reset'}
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def loads(s):
 def pairs(xs):
  d={}
  for k,v in xs:
   if k in d:raise ValueError('duplicate JSON key')
   d[k]=v
  return d
 def finite_float(x):
  v=float(x)
  if not math.isfinite(v):raise ValueError('nonfinite JSON float')
  return v
 return json.loads(s,object_pairs_hook=pairs,parse_float=finite_float,parse_constant=lambda x:(_ for _ in ()).throw(ValueError('nonfinite JSON')))
def load(p):return loads(Path(p).read_text())
def number(v):
 if type(v) not in (int,float) or not math.isfinite(v):raise ValueError('strict finite number required')
 return v
def relative(base,n):
 if type(n) is not str or not n or Path(n).is_absolute():raise ValueError('relative file required')
 p=(Path(base)/n).resolve()
 if not p.is_relative_to(Path(base).resolve()) or not p.is_file():raise ValueError('missing/escaped file')
 return p
def check():
 m=load(O/'manifest.json')
 expected=hashlib.sha256(dumps(m['files'],sort_keys=True,separators=(',',':')).encode()).hexdigest()
 if m['fingerprint']!=expected:raise ValueError('fingerprint mismatch')
 for n,h in m['files'].items():
  if sha(relative(O,n))!=h:raise ValueError('source mismatch '+n)
 for n,h in load(O/'provenance/native-exact.json').items():
  if sha(relative(O/'candidate',n))!=h:raise ValueError('native/core changed '+n)
 return m

def fresh(folder):
 m=check();f=Path(folder).resolve();launch=load(f/'launcher.json');result=load(f/'result.json')
 if result.get('synthetic',False) is not False or launch.get('synthetic',False) is not False:raise ValueError('synthetic evidence forbidden')
 if launch.get('phase')!='replay' or type(launch.get('exitcode')) is not int or launch['exitcode']!=0:raise ValueError('actual clean replay required')
 if launch.get('source_before')!=m or launch.get('source_after')!=m:raise ValueError('fresh source mismatch')
 if result.get('kind')!='ACTUAL_S5_RUNTIME' or result.get('phase')!='replay' or result.get('completed') is not True or result.get('error') is not None or result.get('close_error') is not None:raise ValueError('actual completion absent')
 if result.get('manifest')!=m or result.get('promoted_channels')!=[]:raise ValueError('uncredited/source exact replay required')
 rows=[]
 with (f/'journal.jsonl').open() as stream:
  for line in stream:
   r=loads(line);s=r['status'];i=len(rows)
   if r.get('kind')!='ACTUAL_RUNTIME_OBSERVER' or type(s['controls']) is not int or s['controls']!=i:raise ValueError('control discontinuity')
   if type(s['samples']) is not int or s['samples']!=25*i or abs(number(s['time_s'])-i*.05)>1e-6:raise ValueError('native clock mismatch')
   if number(s['score'])!=0 or s['missing_runtime_validation']!=sorted(CHANNELS):raise ValueError('unproved channel credit')
   if s['measurement_error'] is not None or s['failure'] is not None:raise ValueError('failed actual sample')
   if number(r['physics_dt'])!=.002 or number(r['control_dt'])!=.05:raise ValueError('wrong clock')
   if i:
    if type(r['action']) is not list or len(r['action'])!=14:raise ValueError('14D action missing')
    for x in r['action']:number(x)
   if type(r['physics_tick']) is not int or (i and r['physics_tick']!=rows[0]['physics_tick']+25*i):raise ValueError('physical tick discontinuity')
   for side in ('left','right'):
    rob=r['robots'][side]
    if len(rob['raw8'])!=8 or number(rob['tcp_fk_measured_error_m'])>.001:raise ValueError('actual robot measurement missing')
    for x in rob['raw8']:number(x)
   rows.append(r)
 if len(rows)<2:raise ValueError('actual journal absent')
 s=rows[-1]['status']
 if not 0<s['controls']<=3646 or number(s['diagnostic_score'])!=1 or number(s['hold_s'])<20-1e-8:raise ValueError('native full success/20s hold absent')
 if rows[-1]['observer']['box']['robot_contact'] is not False or rows[-1]['observer']['box']['table_contact'] is not True:raise ValueError('native hands-off/table condition absent')
 if result.get('final_status')!=s:raise ValueError('terminal result mismatch')
 # Recompute the complete native physical history from actual substep measurements.
 import importlib.util
 spec=importlib.util.spec_from_file_location('_s5_fresh_native_rules',O/'candidate/s5_rules.py');rules=importlib.util.module_from_spec(spec);spec.loader.exec_module(rules)
 history=rules.History();begins=0
 with (f/'physics.jsonl').open() as stream:
  for line in stream:
   tick=loads(line)
   if tick.get('event')=='begin':
    begins+=1
    if begins!=1 or history.samples or number(tick['clock_dt'])!=.002:raise ValueError('reset/clock discontinuity')
    continue
   if begins!=1 or number(tick['dt'])!=.002:raise ValueError('native substeps absent')
   row=tick['measurement']
   if len(row['wrist_force_n'])!=2 or set(row['door_q'])!={'AL','AR','BL','BR'} or set(row['door_v'])!=set(row['door_q']):raise ValueError('native measured inventory')
   for value in row['wrist_force_n']+list(row['door_q'].values())+list(row['door_v'].values()):number(value)
   box=row['box']
   for key in ('outside_cabinet','inside_table','table_contact','robot_contact'):
    if type(box[key]) is not bool:raise ValueError('native boolean ownership')
   for key in ('z','upright_cos','spatial_speed'):number(box[key])
   history.add(row,.002)
   if tick['status']!=history.result() or abs(number(tick['physics_s'])-history.time)>1e-8:raise ValueError('native substep history mismatch')
 if begins!=1 or history.samples!=s['samples'] or history.hold<20-1e-8 or history.failure is not None:raise ValueError('whole physical hold trace absent')
 for key,value in history.result().items():
  if s[key]!=value:raise ValueError('native terminal history mismatch '+key)
 media=load(f/'media-finalization.json')
 if media.get('all_encoded') is not True or type(media.get('frames_per_camera')) is not int or media['frames_per_camera']!=len(rows):raise ValueError('actual frame continuity absent')
 if set(media.get('files',{}))!={'top_cam.mp4','left_cam.mp4','right_cam.mp4'}:raise ValueError('camera inventory')
 for n,row in media['files'].items():
  p=relative(f,n)
  if type(row['bytes']) is not int or row['bytes']!=p.stat().st_size or not 0<row['bytes']<=50_000_000:raise ValueError('media bytes')
 return dict(controls=s['controls'],hold_s=s['hold_s'],raw_score=s['diagnostic_score'],public_score=s['score'])

def receipt(path):
 m=check();p=Path(path).resolve();r=load(p)
 if r.get('schema')!='s5-v10-root-physical-review' or r.get('approved') is not True or r.get('reviewer')!='ROOT' or r.get('manifest')!=m:raise ValueError('ROOT matching review required')
 if r.get('historical_scores_transferred') is not False:raise ValueError('old score transfer forbidden')
 if type(r.get('replay')) is not str or Path(r['replay']).is_absolute():raise ValueError('relative replay required')
 f=(p.parent/r['replay']).resolve()
 if not f.is_relative_to(p.parent):raise ValueError('replay escape')
 if fresh(f)!=r.get('actual_summary'):raise ValueError('stale actual summary')
 expected={str(x.relative_to(f)):sha(relative(f,str(x.relative_to(f)))) for x in f.rglob('*') if x.is_file()}
 if r.get('evidence_sha256')!=expected:raise ValueError('replay evidence changed')
 decisions=r.get('channels',{})
 if set(decisions)!=CHANNELS:raise ValueError('all native channel decisions required')
 for channel,row in decisions.items():
  if row.get('approved') is not True or type(row.get('rationale')) is not str or not row['rationale'].strip():raise ValueError('unproved channel '+channel)
  if not row.get('proofs'):raise ValueError('source/actual proof required '+channel)
  for proof in row['proofs']:
   if sha(relative(p.parent,proof['path']))!=proof['sha256']:raise ValueError('proof binding changed')
 if r.get('mounted_support_approved') is not True:raise ValueError('fresh mounted support review required')
 return r
