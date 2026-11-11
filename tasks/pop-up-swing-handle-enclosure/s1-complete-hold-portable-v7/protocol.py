from pathlib import Path
import json,hashlib,math,importlib.util
O=Path(__file__).resolve().parent
CHANNELS=['clock','joint_state','motor_total_limits','passive_mechanisms','robot_all_box_contacts','wrist_force']
def load(p):
 return json.loads(Path(p).read_text(),parse_constant=lambda x:(_ for _ in ()).throw(ValueError(x)))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def check():
 m=load(O/'manifest.json')
 for rel,h in m['files_sha256'].items():
  p=(O/rel).resolve()
  if not p.is_relative_to(O) or sha(p)!=h:raise ValueError('source mismatch '+rel)
 return m['fingerprint_sha256']
def filemap(root):return {str(p.relative_to(root)):sha(p) for p in sorted(Path(root).rglob('*')) if p.is_file()}
def fresh(root):
 root=Path(root).resolve();r=load(root/'result.json')
 if r.get('kind')!='ACTUAL_S1_RUNTIME' or r.get('manifest')!=check() or r.get('phase')!='replay' or r.get('completed') is not True or r.get('error') is not None:raise ValueError('fresh actual replay required')
 if r.get('promoted_channels')!=[]:raise ValueError('replay credit must remain disabled')
 spec=importlib.util.spec_from_file_location('independent_s1_rubric',O/'candidate/rubric.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
 fixture=load(O/'candidate/fixture.json');hist=mod.History(fixture['fixtures']['SH-T1']['target']);last=None;count=0;prev=None
 for line in (root/'physics.jsonl').open():
  x=json.loads(line,parse_constant=lambda x:(_ for _ in ()).throw(ValueError(x)));elapsed=x['status']['elapsed_s'];dt=elapsed-(prev or 0.)
  if type(elapsed) not in (int,float) or not math.isfinite(elapsed) or not 0<dt<.01:raise ValueError('physics clock discontinuity')
  hist.sample(x['measurement'],dt);prev=elapsed;last=x;count+=1
 if not last or hist.raw_score()!=1. or hist.hold<20.-1e-9 or hist.failure:raise ValueError('native whole goal/20s hold absent')
 rows=[json.loads(x) for x in (root/'controller-stream.jsonl').read_text().splitlines()]
 if count!=len(rows)*50:raise ValueError('native physics/control ratio mismatch')
 if not rows or len(rows)>4198 or [x['control'] for x in rows]!=list(range(1,len(rows)+1)):raise ValueError('control stream invalid')
 for x in rows:
  a=x['action']
  if len(a)!=14 or any(type(v) not in (int,float) or not math.isfinite(v) for v in a):raise ValueError('finite 14D action required')
 media=load(root/'media-finalization.json')
 if media.get('all_encoded') is not True or media.get('frames_per_camera')!=len(rows)+1:raise ValueError('media incomplete')
 for name in ['top_cam','left_cam','right_cam']:
  if not (root/(name+'.png')).is_file() or not (root/(name+'.mp4')).is_file():raise ValueError('actual media missing')
 if r['final_status']['measured_candidate_score']!=1. or r['final_status']['score']!=0.:raise ValueError('uncredited native result mismatch')
 return dict(kind='SOURCE_BOUND_S1_PHYSICAL_REVIEW',manifest=check(),replay_files=filemap(root),native_status=hist.status(),controls=len(rows),samples=count,reviewer=None,approved=False,channels={n:{'approved':False,'evidence':[],'rationale':''} for n in CHANNELS},historical_43761_is_full_success=False)
def receipt(path):
 path=Path(path).resolve();r=load(path)
 if r.get('kind')!='SOURCE_BOUND_S1_PHYSICAL_REVIEW' or r.get('manifest')!=check() or r.get('reviewer')!='ROOT' or r.get('approved') is not True or r.get('historical_43761_is_full_success') is not False:raise ValueError('ROOT source-bound approval required')
 expected=fresh(path.parent/'replay')
 for key in ['replay_files','native_status','controls','samples']:
  if r.get(key)!=expected[key]:raise ValueError('replay binding mismatch '+key)
 if set(r.get('channels',{}))!=set(CHANNELS):raise ValueError('channel set mismatch')
 for n,row in r['channels'].items():
  if row.get('approved') is not True or not row.get('rationale') or not row.get('evidence'):raise ValueError('channel unproved '+n)
  for e in row['evidence']:
   p=(path.parent/e['file']).resolve()
   if not p.is_relative_to(path.parent) or sha(p)!=e['sha256']:raise ValueError('evidence binding failed')
 return r
def loads(s):return json.loads(s,parse_constant=lambda x:(_ for _ in ()).throw(ValueError(x)))
def number(x):
 if type(x) not in (int,float) or not math.isfinite(x):raise ValueError('finite JSON number required')
 return x
