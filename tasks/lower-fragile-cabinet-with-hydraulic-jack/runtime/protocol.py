"""Source-bound local ROOT review boundary; CPU fixtures cannot establish physics."""
from pathlib import Path
import json,hashlib,math
O=Path(__file__).resolve().parent
CHANNELS=('clock','drive_limits','glass_force','hydraulics','load_pose','pad_force','rail_contact','robot_load_contact','wrist_force')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def pairs(items):
 d={}
 for k,v in items:
  if k in d:raise ValueError('duplicate JSON key')
  d[k]=v
 return d
def load(p):
 def number(s):
  v=float(s)
  if not math.isfinite(v):raise ValueError('nonfinite JSON')
  return v
 return json.loads(Path(p).read_text(),object_pairs_hook=pairs,parse_float=number,parse_constant=lambda s:(_ for _ in ()).throw(ValueError(s)))
def numeric(v):
 if type(v) not in (int,float) or not math.isfinite(v):raise ValueError('finite owned number required')
 return v
def check():
 m=load(O/'manifest.json')
 if hashlib.sha256(json.dumps(m['sha256'],sort_keys=True,separators=(',',':')).encode()).hexdigest()!=m['fingerprint']:raise ValueError('manifest fingerprint mismatch')
 for rel,h in m['sha256'].items():
  p=(O/rel).resolve()
  if not p.is_relative_to(O) or sha(p)!=h:raise ValueError('source changed '+rel)
 native=load(O/'provenance/candidate-SHA256.json')
 for rel,h in native.items():
  if sha(O/'candidate'/rel)!=h:raise ValueError('native changed '+rel)
 return m['fingerprint']
def bound(base,entry):
 p=(base/entry['path']).resolve()
 if Path(entry['path']).is_absolute() or not p.is_relative_to(base) or sha(p)!=entry['sha256']:raise ValueError('unbound evidence')
 return p
def fresh(out):
 out=Path(out).resolve();r=load(out/'result.json');l=load(out/'launcher.json');fingerprint=check()
 if r['kind']!='ACTUAL_HYDRAULIC_RUNTIME' or r['phase']!='replay' or r['completed'] is not True or r.get('error') is not None:raise ValueError('incomplete replay')
 if l['exitcode']!=0 or type(l['exitcode']) is not int or l['source_before']!=fingerprint or l['source_after']!=fingerprint or r['manifest']!=fingerprint:raise ValueError('process/source mismatch')
 s=r['final'];n=numeric(s['controls'])
 if type(n) is not int or not 1<=n<=3210 or s['failure'] is not None or s['measurement_error'] is not None or numeric(s['diagnostic_candidate_score'])!=1 or numeric(s['hold_s'])<3:raise ValueError('native whole/3s hold absent')
 if numeric(s['score'])!=0 or s['gate']!='unvalidated_channels' or sorted(s['missing_channels'])!=list(CHANNELS):raise ValueError('uncredited native gate changed')
 rows=[load_line(x) for x in (out/'trace.jsonl').read_text().splitlines()]
 after=[x for x in rows if x['phase']=='after_action'];before=[x for x in rows if x['phase']=='before_action']
 if len(after)!=n or len(before)!=n:raise ValueError('action coverage')
 for i,(a,b) in enumerate(zip(after,before),1):
  if type(a['tick']) is not int or type(b['tick']) is not int or a['tick']!=i or b['tick']!=i-1 or a['action']!=b['action'] or len(a['action'])!=14:raise ValueError('action clock')
  for v in a['action']:numeric(v)
  for side in ('left','right'):
   if len(a['robots'][side]['raw8'][0])!=8:raise ValueError('independent fingers missing')
 samples=[load_line(x) for x in (out/'physics.jsonl').read_text().splitlines()];active=[x for x in samples if x['settling'] is False]
 if len(active)!=25*n or type(s['physics_samples']) is not int or numeric(s['physics_samples'])!=25*n or abs(numeric(s['physics_s'])-.05*n)>1e-6:raise ValueError('native sample coverage')
 for x in active:
  if abs(numeric(x['dt'])-.002)>1e-12 or x['history']['failure'] is not None:raise ValueError('physics failure/clock')
  for key in ('height','vz','tilt','glass_force','flow_lowered','speed','spin'):numeric(x['row'][key])
  for v in x['row']['wrist_force']:numeric(v)
  for key in ('on_rails','on_pad','robot_contact'):
   if type(x['row'][key]) is not bool:raise ValueError('contact boolean ownership')
 import importlib.util
 spec=importlib.util.spec_from_file_location('_hydraulic_native_scoring',O/'candidate/scoring.py');mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod);history=mod.History()
 for x in active:history.update(x['row'],x['dt'])
 recomputed=history.result()
 for key in ('hold_s','diagnostic_candidate_score','failure','physics_samples','physics_s','landing_speed_m_s','max_jack_descent_m','peak_glass_N'):
  if recomputed[key]!=s[key]:raise ValueError('native history mismatch '+key)
 if after[-1]['status']!=s:raise ValueError('journal/final mismatch')
 media=load(out/'media-finalization.json')
 if media['all_encoded'] is not True or type(media['frames_per_camera']) is not int or media['frames_per_camera']!=n+1:raise ValueError('actual video coverage')
 for name in ('left_cam','right_cam','top_cam'):
  if type(media['encoder_returncodes'][name]) is not int or media['encoder_returncodes'][name]!=0 or not 0<(out/(name+'.mp4')).stat().st_size<=50000000:raise ValueError('media failed')
 return {'manifest':fingerprint,'controls':n,'hold_s':s['hold_s'],'public_score':0,'native_diagnostic':1,'proofs':{name:{'path':name,'sha256':sha(out/name)} for name in ('result.json','launcher.json','trace.jsonl','physics.jsonl','media-finalization.json','left_cam.mp4','right_cam.mp4','top_cam.mp4')}}
def load_line(s):
 # Same strict decoder without temporary files.
 d=json.loads(s,object_pairs_hook=pairs,parse_constant=lambda x:(_ for _ in ()).throw(ValueError(x)))
 def walk(x):
  if type(x) is float:numeric(x)
  elif isinstance(x,dict):
   for v in x.values():walk(v)
  elif isinstance(x,list):
   for v in x:walk(v)
 walk(d);return d
def receipt(path):
 p=Path(path).resolve();r=load(p)
 if r['kind']!='HYDRAULIC_ROOT_PHYSICAL_REVIEW' or r['approved'] is not True or r['reviewer']!='ROOT' or r['manifest']!=check():raise ValueError('ROOT review absent')
 replay=bound(p.parent,r['replay_result']).parent;f=fresh(replay)
 if r['fresh_replay']!=f:raise ValueError('fresh proof changed')
 if sorted(r['channels'])!=list(CHANNELS):raise ValueError('channel coverage')
 for key in (*CHANNELS,'supported_mount'):
  d=r['decisions'][key]
  if d['approved'] is not True or not isinstance(d['scope'],str) or len(d['scope'])<12 or not d['proofs']:raise ValueError('scoped actual evidence missing '+key)
  for e in d['proofs']:bound(p.parent,e)
 return r
