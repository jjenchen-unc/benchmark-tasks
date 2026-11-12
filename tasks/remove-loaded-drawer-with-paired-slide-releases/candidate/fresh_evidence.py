"""Strict fresh replay evidence checks. Numeric observations are not ROOT approval."""
from pathlib import Path
import math
from runtime_credit import load,loads,sha
NAMES=('launcher.json','result.json','journal.jsonl','sourcehashes.json','credit-binding.json','media-finalization.json')
def relative_file(base,value):
 if not isinstance(value,str) or not value or Path(value).is_absolute():raise ValueError('portable relative evidence path required')
 p=(Path(base)/value).resolve()
 if not p.is_relative_to(Path(base).resolve()) or not p.is_file():raise ValueError('evidence path escapes/missing')
 return p
def number(v):
 if type(v) not in (int,float) or not math.isfinite(v):raise ValueError('strict finite numeric required')
 return v

def reviewed_physical_manifest(core,current):
 proof=load(core/'proofs/v21-interface-derivative.json')
 if proof.get('schema')!='drawers-root-yam-interface-derivative-v21' or proof.get('reviewer')!='ROOT' or proof.get('approved') is not True:raise ValueError('explicit interface derivative review required')
 old=proof['original_manifest'];oldsha=proof['original_manifest_sha256']
 if __import__('hashlib').sha256(proof['original_manifest_text'].encode()).hexdigest()!=oldsha or loads(proof['original_manifest_text'])!=old:raise ValueError('original manifest proof changed')
 allowed={'drawers_kit.py','task.py','model_entry.py','task.draft.json','credit-contract.json','fresh_evidence.py'}
 if set(proof['current_interface_sha256'])!=allowed or set(proof['original_interface_text'])!=allowed:raise ValueError('interface derivative file set mismatch')
 for n,d in old['sha256'].items():
  expected=proof['current_interface_sha256'][n] if n in allowed else d
  if sha(core/n)!=expected:raise ValueError('unreviewed physical source derivative: '+n)
 for n,text in proof['original_interface_text'].items():
  if __import__('hashlib').sha256(text.encode()).hexdigest()!=old['sha256'][n]:raise ValueError('original interface proof changed: '+n)
 if set(current['sha256'])!=set(old['sha256'])|{'proofs/v21-interface-derivative.json'}:raise ValueError('unexpected derivative candidate file')
 oldkit=proof['original_interface_text']['drawers_kit.py'];removed=proof['removed_requested_fk_guard']
 if oldkit.count(removed)!=1 or (core/'drawers_kit.py').read_text()!=oldkit.replace(removed,''):raise ValueError('interface repair differs from reviewed removal')
 clause=' Tools must remain vertical, yaw free.'
 for n in ('task.py','model_entry.py','task.draft.json'):
  before=proof['original_interface_text'][n]
  if before.count(clause)!=1 or (core/n).read_text()!=before.replace(clause,''):raise ValueError('public instruction changed beyond API clause')
 oldcontract=loads(proof['original_interface_text']['credit-contract.json']);oldcontract['core_sha256']['drawers_kit.py']=sha(core/'drawers_kit.py')
 if load(core/'credit-contract.json')!=oldcontract:raise ValueError('native credit contract changed beyond interface binding')
 return old,oldsha

def check(core,folder):
 core=Path(core).resolve();folder=Path(folder).resolve();manifest=load(core/'manifest.json');expected=manifest['sha256']
 for n,d in expected.items():
  if sha(relative_file(core,n))!=d:raise ValueError('current source changed: '+n)
 observed,observed_sha=reviewed_physical_manifest(core,manifest)
 observed_expected=observed['sha256']
 launch=load(folder/'launcher.json')
 # Newly reproduced v21 evidence may bind the exact current manifest directly.
 if launch.get('source_before')==expected and launch.get('source_after')==expected and launch.get('manifest_sha256')==sha(core/'manifest.json'):
  observed,observed_sha,observed_expected=manifest,sha(core/'manifest.json'),expected
 if type(launch.get('exitcode')) is not int or launch['exitcode']!=0 or launch.get('phase')!='replay' or launch.get('source_unchanged') is not True:raise ValueError('fresh replay launcher not clean')
 if launch.get('source_before')!=observed_expected or launch.get('source_after')!=observed_expected or launch.get('manifest_sha256')!=observed_sha:raise ValueError('fresh replay source mismatch')
 if load(folder/'sourcehashes.json')!=observed:raise ValueError('fresh embedded source mismatch')
 credit=load(folder/'credit-binding.json')
 if credit.get('runtime_channels_promoted')!=[] or credit.get('status')!='PENDING_ROOT_PHYSICAL_REVIEW':raise ValueError('fresh replay must be uncredited')
 result=load(folder/'result.json')
 if result.get('status')!='foundation_observed' or result.get('first_termination') is not None:raise ValueError('fresh replay failed/terminated')
 rows=[loads(s) for s in (folder/'journal.jsonl').read_text().splitlines()]
 if len(rows)<2:raise ValueError('fresh journal empty')
 for i,r in enumerate(rows):
  s=r['status'];c=s['controls'];ns=s['samples']
  if type(c) is not int or c!=i or type(ns) is not int or ns!=12*c or abs(number(s['physics_s'])-c/20)>1e-6:raise ValueError('fresh clock mismatch')
  if s['failure'] is not None or s['measurement_error'] is not None or r.get('first_termination') is not None:raise ValueError('fresh safety/measurement failure')
  if number(s['score'])!=0:raise ValueError('uncredited replay public score must stay zero')
 s=rows[-1]['status'];p=rows[-1]['physics']
 if not 0<s['controls']<=3586 or number(s['candidate_score'])!=1 or number(s['hold_s'])<1:raise ValueError('native goal/hold absent')
 if result.get('status_channels')!=s:raise ValueError('fresh result/journal mismatch')
 if p['contents'] is not True or p['drawer_touch'] is not False or p['drop'] is not False or number(p['max_x'])>=.12 or abs(number(p['drawer_p'][2])-.835)>=.006 or number(p['drawer_p'][0])>=0 or number(p['drawer_R'][2][2])<=math.cos(math.radians(3)):raise ValueError('fresh native terminal predicate absent')
 media=load(folder/'media-finalization.json')
 if media.get('all_encoded') is not True or type(media.get('frames_per_camera')) is not int or media['frames_per_camera']!=s['controls']+1:raise ValueError('replay frames/encoding invalid')
 if set(media.get('files',{}))!={'top_cam.mp4','left_cam.mp4','right_cam.mp4'}:raise ValueError('replay cameras missing')
 for name,row in media['files'].items():
  f=relative_file(folder,name)
  if type(row['bytes']) is not int or row['bytes']!=f.stat().st_size or not 0<row['bytes']<=50_000_000:raise ValueError('replay video size invalid')
 return {'controls':s['controls'],'hold_s':s['hold_s'],'candidate_score':s['candidate_score'],'public_score':s['score'],'physical_approval':False,'review_required':True}

def validate_review(core,path,affected):
 base=Path(path).resolve().parent;root=Path(core).resolve().parent
 if affected.get('schema')!='drawers-fresh-pad-replay-v19':raise ValueError('fresh v19 replay receipt required')
 rows=affected.get('evidence',[])
 if not isinstance(rows,list) or not rows:raise ValueError('affected evidence absent')
 paths={}
 for row in rows:
  name=row.get('name');value=row.get('path')
  if not isinstance(value,str) or Path(value).is_absolute():raise ValueError('relative affected path required')
  p=(base/value).resolve()
  if not p.is_relative_to(root) or not p.is_file() or sha(p)!=row.get('sha256') or name in paths:raise ValueError('affected evidence binding invalid')
  paths[name]=p
 if not set(NAMES)<=set(paths):raise ValueError('complete fresh evidence missing')
 folder=paths['launcher.json'].parent
 if any(paths[n]!=folder/n for n in NAMES):raise ValueError('mixed replay evidence')
 summary=check(core,folder)
 if type(affected.get('controls')) is not int or affected['controls']!=summary['controls'] or type(affected.get('hold_s')) not in (int,float) or affected['hold_s']!=summary['hold_s']:raise ValueError('affected numeric ownership mismatch')
 return summary
