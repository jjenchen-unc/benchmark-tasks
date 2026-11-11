"""Verify one actual continuous S2 Dream run, then encode its original RGB frames."""
from pathlib import Path
import sys,json,math,re,urllib.parse,hashlib,argparse
O=Path(__file__).resolve().parents[1]/'s2-v30';sys.path.insert(0,str(O));sys.path.insert(0,str(O/'candidate'))
from protocol import check,load,loads,number,receipt
from strict_json import dumps
FINGERPRINT='f14b6578e9d24ab65e2ba991d9894f4eb82a3fca9dec481d9883adcff43f8542'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def verify(out,review):
 out=Path(out).resolve();m=check()
 if m['fingerprint']!=FINGERPRINT:raise ValueError('frozen core mismatch')
 summaries=list((out/'inspect').glob('adhoc_*.json'));records=list((out/'inspect/records').rglob('record.json'))
 if len(summaries)!=1 or len(records)!=1:raise ValueError('one actual summary/record required')
 summary=load(summaries[0]);record=load(records[0]);reg=load(out/'registration.json');after=load(out/'entry-envelope-after.json')
 receipt(review)
 if sha(review)!=reg['ROOT_review_sha256']:raise ValueError('physical review changed')
 if reg['manifest']!=m or load(out/'source-after.json')!=m or reg['budget']!=7964 or reg['control_hz']!=20 or reg['physical_seed']!=42:raise ValueError('model source/registration mismatch')
 if reg['entry_envelope_sha256']!=after['entry_envelope_sha256'] or after['core_fingerprint']!=FINGERPRINT or reg['entry_envelope_sha256']!=sha(Path(__file__).with_name('model_entry.py')):raise ValueError('model entry changed')
 if summary['status']!='success' or summary['error'] is not None or summary['results']['errored_trials']!=0 or record['status']!='success' or record['error'] is not None:raise ValueError('model framework error')
 config=record['configuration']
 if config['max_steps']!=7964 or config['scene']['instruction']!=reg['instruction'] or summary['eval']['policy']!='dream' or config['policy']['name']!='dream':raise ValueError('instruction/budget/policy mismatch')
 from yam_arena.task_file import load as load_task
 if reg['instruction']!=load_task(O/'candidate/task.py').docs:raise ValueError('native instruction changed')
 if record['termination']['reason'] in ('measurement_unavailable','measurement_error','initialization_error'):raise ValueError('measurement/init failure')
 seed=load(out/'seed-contract.json')
 if type(seed['inspect_seed']) is not int or seed['physical_seed']!=42 or seed['passed_unchanged'] is not True:raise ValueError('seed contract')
 steps=record['steps']
 if type(steps) is not int or not 0<steps<=7964:raise ValueError('model steps invalid')
 rows=[loads(s) for s in (out/'actions.jsonl').read_text().splitlines()]
 if len(rows)!=steps:raise ValueError('action journal length mismatch')
 physical_failures={'instrument_tipped','case_tipped','wrist_force_abort'}
 for i,row in enumerate(rows,1):
  s=row['status'];failure=s['failure'];samples=s['samples']
  if type(s['controls']) is not int or s['controls']!=i or type(samples) is not int or s['measurement_error'] is not None or s['credit_gate']!=[]:raise ValueError('measured channels unavailable')
  if failure is None:
   if samples!=25*i:raise ValueError('measured clock discontinuity')
  elif failure not in physical_failures or i!=steps or not 25*(i-1)<samples<=25*i or number(s['score'])!=0 or record['termination']['reason']!=failure:raise ValueError('invalid terminal physical failure')
  # Native History freezes at its first latched physical failure, possibly
  # inside the final control. Such a genuine scored0 is not a provider error.
  if abs(number(s['seconds'])-.002*samples)>1e-6:raise ValueError('native sample clock mismatch')
  if len(row['action'])!=14 or any(not math.isfinite(number(x)) for x in row['action']):raise ValueError('finite14D action required')
  if number(s['score'])!=number(s['measured_score']):raise ValueError('public/native score mismatch')
 score=number(summary['results']['metrics']['task_score'])
 if not 0<=score<=1 or abs(number(rows[-1]['status']['score'])-score)>1e-9:raise ValueError('actual score mismatch')
 continuity=[loads(s) for s in(out/'continuity.jsonl').read_text().splitlines()];chunks=[r for r in continuity if r['kind']=='chunk']
 if sum(r['kind']=='episode_start' for r in continuity)!=1 or not chunks or len({r['session_sha256'] for r in chunks})!=1 or any(r['continuous'] is not True or type(r['attempts']) is not int or r['attempts']<1 or not r['session_sha256'] for r in chunks):raise ValueError('continuous session invalid')
 frames={n:{} for n in ('top_cam','left_cam','right_cam')};trials=set()
 for p in(out/'inspect/frames').rglob('*.npy'):
  match=re.fullmatch(r'~f1~([^~]*)~([^~]*)_(-?[0-9]+)\.npy',p.name)
  if match is None:raise ValueError('unrecognized Inspect frame name')
  trial,camera=[urllib.parse.unquote_to_bytes(x).decode('utf8') for x in match.group(1,2)];tick=int(match.group(3));trials.add(trial)
  if camera not in frames or tick in frames[camera]:raise ValueError('unexpected/duplicate frame')
  frames[camera][tick]=p
 if len(trials)!=1 or any(set(v)!=set(range(steps+1)) for v in frames.values()):raise ValueError('frame continuity mismatch')
 import numpy as np
 from video_capture import Capture
 media=out/'final-model-media';media.mkdir(exist_ok=False);capture=Capture(media,{n:np.load(v[0],allow_pickle=False) for n,v in frames.items()})
 try:
  for tick in range(1,steps+1):capture.append({n:np.load(v[tick],allow_pickle=False) for n,v in frames.items()})
 finally:proof=capture.close()
 if proof['all_encoded'] is not True or proof['frames_per_camera']!=steps+1:raise ValueError('encoded media mismatch')
 for n,row in proof['files'].items():
  if not 0<row['bytes']<=50_000_000:raise ValueError('model video size invalid')
 report={'status':'ACTUAL_MODEL_COMPLETE_PENDING_ROOT_MEDIA_REVIEW','score':score,'steps':steps,'sessions':1,'request_count':len(chunks),'transport_retries':sum(r['kind']=='transport_retry' for r in continuity),'frames_per_camera':steps+1,'source_fingerprint':FINGERPRINT,'summary_sha256':sha(summaries[0]),'record_sha256':sha(records[0]),'media':{n:sha(media/n) for n in proof['files']},'official_acceptance_claim':False};(out/'model-verification.json').write_text(dumps(report,indent=2)+'\n');return report
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('out',type=Path);p.add_argument('--review',type=Path,required=True);a=p.parse_args();print(dumps(verify(a.out,a.review)))
