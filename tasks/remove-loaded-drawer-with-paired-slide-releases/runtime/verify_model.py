"""Offline postrun Inspect/Dream verification and actual stored RGB finalization."""
from pathlib import Path
import sys,re,json,math,urllib.parse,hashlib,argparse
O=Path(__file__).resolve().parent;C=O/'candidate';sys.path.insert(0,str(C))
from runtime_credit import load,loads,sha
from strict_json import dumps
from fresh_evidence import number

def verify(out):
 out=Path(out).resolve();summaries=list((out/'inspect').glob('adhoc_*.json'));records=list((out/'inspect/records').rglob('record.json'))
 if len(summaries)!=1 or len(records)!=1:raise ValueError('one actual summary/record required')
 summary=load(summaries[0]);record=load(records[0]);reg=load(out/'registration.json')
 if reg['source_manifest']!=load(C/'manifest.json') or reg['budget']!=3586 or reg['control_hz']!=20 or reg['physical_seed']!=42:raise ValueError('model source/registration mismatch')
 if summary['status']!='success' or summary['error'] is not None or summary['results']['errored_trials']!=0 or record['status']!='success' or record['error'] is not None:raise ValueError('model evaluation error')
 if record['configuration']['max_steps']!=3586 or record['configuration']['scene']['instruction']!=load(C/'task.draft.json')['instruction'] or reg['instruction']!=record['configuration']['scene']['instruction']:raise ValueError('model native instruction/budget mismatch')
 if summary['eval']['policy']!='dream' or record['configuration']['policy']['name']!='dream':raise ValueError('wrong actual policy')
 steps=record['steps']
 if type(steps) is not int or not 0<steps<=3586:raise ValueError('model steps invalid')
 if record['termination']['reason'] in ('measurement_error','measurement_unavailable','initialization_error'):raise ValueError('model measurement/init failure')
 rows=[loads(s) for s in (out/'action-journal.jsonl').read_text().splitlines()]
 if len(rows)!=steps:raise ValueError('action journal length mismatch')
 for i,row in enumerate(rows,1):
  s=row['status']
  if type(s['controls']) is not int or s['controls']!=i or type(s['samples']) is not int or s['samples']!=12*i or abs(number(s['physics_s'])-i/20)>1e-6 or s['measurement_error'] is not None:raise ValueError('model measured clock unavailable')
  if s.get('missing_channels')!=[] or s.get('credit_gate')!='validated':raise ValueError('actual model channel credit unavailable')
  if len(row['action'])!=14 or any(not math.isfinite(number(v)) for v in row['action']):raise ValueError('model action invalid')
 score=number(summary['results']['metrics']['task_score'])
 if not 0<=score<=1 or abs(number(rows[-1]['status']['score'])-score)>1e-9:raise ValueError('actual scorer/status mismatch')
 continuity=[loads(s) for s in (out/'continuity.jsonl').read_text().splitlines()];chunks=[r for r in continuity if r['kind']=='chunk']
 if sum(r['kind']=='episode_start' for r in continuity)!=1 or not chunks or len({r['session_sha256'] for r in chunks})!=1 or any(r['continuous'] is not True or type(r['attempts']) is not int or r['attempts']<1 or not r['session_sha256'] for r in chunks):raise ValueError('model continuity invalid')
 # Decode actual pinned Inspect FrameStore names; reject guessed/legacy names.
 frames={n:{} for n in ('top_cam','left_cam','right_cam')};trials=set()
 for p in (out/'inspect/frames').rglob('*.npy'):
  m=re.fullmatch(r'~f1~([^~]*)~([^~]*)_(-?[0-9]+)\.npy',p.name)
  if m is None:raise ValueError('unrecognized Inspect frame name')
  trial,camera=[urllib.parse.unquote_to_bytes(s).decode('utf-8') for s in m.group(1,2)];t=int(m.group(3));trials.add(trial)
  if camera not in frames or t in frames[camera]:raise ValueError('unexpected/duplicate frame')
  frames[camera][t]=p
 if len(trials)!=1 or any(set(r)!=set(range(steps+1)) for r in frames.values()):raise ValueError('actual frame continuity mismatch')
 overview=load(out/'overview-capture.json')
 if overview['frames']!=steps+1 or overview['encoder_returncode']!=0 or overview['policy_camera_names_unchanged']!=['left_cam','right_cam','top_cam'] or overview['extra_physics_steps']!=0 or overview['source_helper_sha256']!=sha(O/'overview_capture.py') or overview['sha256']!=sha(out/'overview_cam.mp4'):raise ValueError('recording-only overview evidence invalid')
 import numpy as np
 from video_capture import Capture
 media=out/'final-model-media';media.mkdir(exist_ok=False)
 capture=Capture(media,{n:np.load(frames[n][0],allow_pickle=False) for n in frames})
 try:
  for t in range(1,steps+1):capture.append({n:np.load(frames[n][t],allow_pickle=False) for n in frames})
 finally:proof=capture.close()
 if proof['all_encoded'] is not True or proof['frames_per_camera']!=steps+1 or any(not 0<r['bytes']<=50_000_000 for r in proof['files'].values()):raise ValueError('final model media invalid')
 report={'status':'ACTUAL_COMPLETED_DREAM_TRIAL_PENDING_ROOT_MEDIA_REVIEW','score':score,'steps':steps,'sessions':1,'request_count':len(chunks),'transport_retry_events':sum(r['kind']=='transport_retry' for r in continuity),'frames_per_camera':steps+1,'overview_media':overview,'source_manifest_sha256':sha(C/'manifest.json'),'summary_sha256':sha(summaries[0]),'record_sha256':sha(records[0]),'media_sha256':{n:sha(media/n) for n in proof['files']},'submission_ready':False}
 (out/'model-verification.json').write_text(dumps(report,indent=2)+'\n');print(dumps(report));return report
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('out',type=Path);a=p.parse_args();verify(a.out)
