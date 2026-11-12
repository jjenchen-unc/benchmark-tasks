"""Independent replay of saved genuine14D20Hz robot Actions; no state actuation."""
from c1_launch import bootstrap
bootstrap(__file__)
import argparse,hashlib,json,os,sys
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--public-case',action='store_true');ap.add_argument('--actions',type=Path,required=True);ap.add_argument('--source-hashes',type=Path,required=True);ap.add_argument('--target',default='C1-T1');ap.add_argument('--seed',type=int,default=42);ap.add_argument('--out',type=Path,required=True);a=ap.parse_args()
 __import__('c1_launch').output_directory(a.out);
 if a.public_case and a.target!='C1-T1':raise ValueError('Public case requires C1-T1')
 from c1_validation import strict,number
 expected=strict(a.source_hashes);mismatch=[]
 for name,h in expected.items():
  p=HERE/name
  if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=h:mismatch.append(name)
 if mismatch:raise RuntimeError('Replay source mismatch '+str(mismatch))
 raw_actions=strict(a.actions)
 if type(raw_actions) is not list or any(type(row) is not list or len(row)!=14 for row in raw_actions):raise ValueError('strict14D rows required')
 actions=np.asarray([[number(x) for x in row] for row in raw_actions],float)
 if actions.ndim!=2 or actions.shape[1]!=14 or len(actions)>2268 or not np.isfinite(actions).all():raise ValueError('Actions must be N<=2268 by14 finite joint/gripper targets')
 os.environ['C1_TARGET']=a.target;os.environ['C1_TRACE']=str(a.out/'measurements.jsonl')
 from inspect_robots import Scene
 from inspect_robots.types import Action
 from yam_arena.inspect_embodiment import yam_arena_embodiment
 emb=yam_arena_embodiment(task=str(HERE/('public_entry.py' if a.public_case else 'task.py')),device='cpu',object_jitter=0,control_hz=20,cam_width=320,cam_height=176)
 report={'public_case':a.public_case,'trial_seed':a.seed,'physical_seed':42 if a.public_case else a.seed,'kind':'Independent genuine14D20Hz replay','model_calls':0,'submission_ready':False,'actions_sha256':hashlib.sha256(a.actions.read_bytes()).hexdigest(),'controls':0,'instruction':__import__('c1_public').INSTRUCTION}
 capture=None
 try:
  from c1_public import INSTRUCTION
  from video_capture import Capture
  obs=emb.reset(Scene(id='C1-reference-replay',instruction=INSTRUCTION),seed=a.seed)
  capture=Capture(a.out,obs.images)
  for action in actions:
   result=emb.step(Action(data=action));report['controls']+=1
   capture.append(result.observation.images)
   if result.terminated:
    report['termination_reason']=result.termination_reason;break
  import c1_kit
  report['status']=c1_kit.ENGINE.status();report['physical_raw_fullscore']=report['status']['raw_score']==1.;report['verified_fullscore']=report['status']['score']==1.
 except BaseException as e:report['error']=type(e).__name__+': '+str(e);raise
 finally:
  try:
   if capture is not None:
    report['media']=capture.close()
    if not report['media']['all_encoded'] or report['media']['frames_per_camera']!=report['controls']+1 or any(v['bytes']>50000000 for v in report['media']['files'].values()):raise RuntimeError('Actual camera encoding/continuity/size failure')
  finally:
   (a.out/'result.json').write_text(json.dumps(report,indent=2)+'\n');emb.close()
if __name__=='__main__':main()
