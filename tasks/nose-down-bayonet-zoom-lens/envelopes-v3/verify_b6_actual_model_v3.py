"""Offline actualB6 model verifier. Native journal score0 is NOT public task_score."""
from pathlib import Path
import sys,json,math,re,hashlib,urllib.parse,argparse
CAMERAS=('top_cam','left_cam','right_cam')
def bind(bundle):
 root=Path(bundle).resolve();sys.path[:0]=[str(root),str(root/'candidate')]
 import protocol
 if Path(protocol.__file__).resolve().parent!=root:raise ValueError('sourceboundprotocolmoduleorigin mismatch')
 from mechanics import History
 return root,protocol,History

def public_score(history,channels):return history.result(channels)['score']

def raw_history(bundle,path,journal=None):
 root,P,History=bind(bundle);h=History();count=0
 for line in Path(path).open():
  row=P.loads(line)
  if type(row) is not dict or set(row)!={'sim_time','control','row','status'}:raise ValueError('actual physics trace structure')
  P.number(row['sim_time']);P.validate_physics_row(row['row']);h.update(row['row'],.002);count+=1
  native=h.result()
  if type(row['control']) is not int or row['control']!=(count-1)//25:raise ValueError('raw control clock')
  for k,v in native.items():
   if row['status'].get(k)!=v:raise ValueError('native History contradiction '+k)
  if row['status']['measurement_error'] is not None:raise ValueError('model physical measurement unavailable')
  if journal is not None and count%25==0:
   logged=journal[count//25-1]
   if logged['physics']!=row['row']:raise ValueError('action callback differs from last actual substep')
   for k,v in native.items():
    if logged['status'].get(k)!=v:raise ValueError('action/native History contradiction '+k)
 return h,count,public_score(h,P.CHANNELS)

def validate_external_helper(P,root,out,manifest,model_helper):
 if not (out/'external-model-envelope.json').exists():return None
 envelope=P.load(out/'external-model-envelope.json')
 if model_helper is None:raise ValueError('source path for externalmodelhelper required')
 external=P.sha(Path(model_helper));kind=envelope.get('kind')
 allowed={'B6_external_same_session_transport_v2':'1e98de7ba34fecf9f6cc7cac68be8eae593670b665e6e8b0da28e6345b03e9b8','B6_external_same_session_transport_gripper_v3':'6699ef55f27bf42c859befd28047c4186148365e6c5f69c2a317927930228356'}
 if allowed.get(kind)!=external or envelope.get('wrapper_sha256')!=external or envelope.get('original_model_child_sha256')!=P.sha(root/'model_child.py') or envelope.get('source_manifest')!=manifest:raise ValueError('external modelhelper source/envelope scope mismatch')
 if kind.endswith('gripper_v3'):
  adapter=Path(model_helper).with_name('gripper_saturation.py')
  if P.sha(adapter)!='b525f6c889d0accbfa7ac3183371150138d4b717a6ea5814f45e4dc2f8eeac06' or envelope.get('gripper_adapter_sha256')!=P.sha(adapter) or envelope.get('policy_conversion')!='Dream-1 + public gripper saturation [0,1] on all returned rows':raise ValueError('versioned shared gripper adapter mismatch')
 return external


def verify_public_projection(P,out,chunks,actions,reg,model_helper):
 adapter_sha=P.sha(Path(model_helper).with_name('gripper_saturation.py'))
 label='Dream-1 + public gripper saturation [0,1] on all returned rows'
 if adapter_sha!='b525f6c889d0accbfa7ac3183371150138d4b717a6ea5814f45e4dc2f8eeac06' or reg.get('gripper_adapter_sha256')!=adapter_sha or reg.get('policy_conversion')!=label:raise ValueError('registered public gripper adapter mismatch')
 projections=[P.loads(x) for x in (out/'gripper-projection.jsonl').read_text().splitlines()]
 if len(projections)!=len(chunks):raise ValueError('projection/chunk count mismatch')
 for index,row in enumerate(projections,1):
  if row.get('kind')!='public_gripper_projection' or row['chunk_index']!=index or type(row['rows']) is not int or not 1<=row['rows']<=48 or row['rows']!=chunks[index-1].get('rows') or row['bounds']!=[0.,1.] or row['all_rows_preserved'] is not True or row['arm_columns_sha256_before']!=row['arm_columns_sha256_after'] or re.fullmatch('[0-9a-f]{64}',row['arm_columns_sha256_before']) is None:raise ValueError('model conversion row/arm preservation mismatch')
  counts={'left':0,'right':0};seen=set()
  for item in row['clipped_rows']:
   side=item['side'];tick=item['row_index'];before=P.number(item['before']);after_value=P.number(item['after']);key=(side,tick)
   if side not in counts or type(tick) is not int or not 0<=tick<row['rows'] or key in seen or not (before<0 or before>1) or after_value!=(0. if before<0 else 1.):raise ValueError('projection clip journal invalid')
   seen.add(key);counts[side]+=1
  if counts!=row['clipped_by_side'] or len(seen)!=row['clipped_values']:raise ValueError('projection count mismatch')
  for side in counts:
   lo,hi=map(P.number,row['preprojection_grip_minmax'][side])
   if lo>hi:raise ValueError('projection preprojection range invalid')
   values=[P.number(x['before']) for x in row['clipped_rows'] if x['side']==side]
   if any(not lo<=value<=hi for value in values) or bool(values)!=(lo<0 or hi>1):raise ValueError('projection range/clip coverage mismatch')
   if lo<0 and min(values)!=lo or hi>1 and max(values)!=hi:raise ValueError('projection extreme not present in clip journal')
 for row in actions:
  if not all(0<=P.number(row['action'][i])<=1 for i in [6,13]):raise ValueError('applied public gripper outside original endpoints')
 return {'policy_conversion':label,'adapter_sha256':adapter_sha,'projection_chunks':len(projections),'projected_gripper_values':sum(x['clipped_values'] for x in projections),'all_returned_rows_preserved':True,'arm_bytes_preserved':True,'projection_journal_sha256':P.sha(out/'gripper-projection.jsonl')}

def verify(bundle,out,review,encode=False,model_helper=None):
 root,P,History=bind(bundle);out=Path(out).resolve();P.receipt(review);manifest=P.check();reg=P.load(out/'registration.json');seed=P.load(out/'seed-contract.json')
 if P.sha(Path(review))!=reg['review_sha256'] or reg['source_manifest']!=manifest or P.load(out/'source-after.json')!=manifest:raise ValueError('model source or review mismatch')
 external_helper_sha256=validate_external_helper(P,root,out,manifest,model_helper)
 if type(reg['budget']) is not int or type(reg['control_hz']) is not int or type(reg['physical_seed']) is not int or reg['budget']!=3034 or reg['control_hz']!=20 or reg['physical_seed']!=42:raise ValueError('registration native case mismatch')
 summaries=list((out/'inspect').glob('adhoc_*.json'));records=list((out/'inspect/records').rglob('record.json'))
 if len(summaries)!=1 or len(records)!=1:raise ValueError('one actual summary/record required')
 s=P.load(summaries[0]);r=P.load(records[0]);cfg=r['configuration'];steps=r['steps'];instruction=P.load(root/'candidate/task.draft.json')['instruction']
 if s['status']!='success' or s['error'] is not None or (type(s['results']['errored_trials']) is not int or s['results']['errored_trials']!=0) or r['status']!='success' or r['error'] is not None:raise ValueError('actual model evaluation error, not score0')
 if type(steps) is not int or not 0<steps<=3034 or cfg['max_steps']!=3034 or cfg['scene']['instruction']!=instruction or reg['instruction']!=instruction:raise ValueError('model budget/instruction mismatch')
 if cfg['seed']!=42 or cfg['embodiment']['name']!='yam_arena' or cfg['embodiment']['control_hz']!=20 or cfg['embodiment']['is_simulated'] is not True:raise ValueError('actual model embodiment/20Hz/native seed mismatch')
 if s['eval']['policy']!='dream' or s['eval']['policy_checkpoint']!='dreamlabs/dream-1-beta' or cfg['policy']['name']!='dream' or cfg['policy']['checkpoint']!='dreamlabs/dream-1-beta':raise ValueError('actual Dream1 required')
 if type(seed['inspect_trial_seed']) is not int or seed['passed_unchanged'] is not True or seed['physical_seed']!=42 or cfg['trial_seed']!=seed['inspect_trial_seed']:raise ValueError('actual seed boundary mismatch')
 if r['termination']['reason'] in ('measurement_unavailable','measurement_error','initialization_error'):raise ValueError('invalid physical model trial')
 J=[P.loads(x) for x in (out/'action-journal.jsonl').read_text().splitlines()]
 if len(J)!=steps:raise ValueError('model action count')
 for i,row in enumerate(J,1):
  state=row['status'];action=row['action']
  if len(action)!=14 or any(not math.isfinite(P.number(v)) for v in action):raise ValueError('genuine finite14D action required')
  if state['controls']!=i or type(state['controls']) is not int or state['samples']!=25*i or abs(P.number(state['sim_elapsed_s'])-i/20)>1e-8 or state['measurement_error'] is not None:raise ValueError('model physical clock')
  if state['score']!=0 or state['missing_runtime_validation']!=sorted(P.CHANNELS):raise ValueError('native journal/source scope changed')
 h,count,expected=raw_history(root,out/'physics.jsonl',J)
 if count!=steps*25:raise ValueError('full raw/control count mismatch')
 vectors=None;force_verifier_sha256=None
 if (root/'measurement-derivation-v22.json').exists():
  import importlib.util
  spec=importlib.util.spec_from_file_location('b6_full_vector_verifier',Path(__file__).with_name('verify_b6_force_trace.py'));v=importlib.util.module_from_spec(spec);spec.loader.exec_module(v);force_verifier_sha256=P.sha(Path(__file__).with_name('verify_b6_force_trace.py'));vectors=v.verify(root,out/'force-vectors.jsonl',out/'physics.jsonl')
  if vectors['samples']!=count:raise ValueError('actualforce/native samplecount mismatch')
 score=P.number(s['results']['metrics']['task_score'])
 # Public model_task.score opens approved channels; ENGINE.status stays native0.
 if score!=expected or not 0<=score<=1:raise ValueError('Inspect score differs from approved-channel native History')
 continuity=[P.loads(x) for x in (out/'continuity.jsonl').read_text().splitlines()];chunks=[x for x in continuity if x['kind']=='chunk']
 if sum(x['kind']=='episode_start' for x in continuity)!=1 or not chunks or len({x['session_sha256'] for x in chunks})!=1:raise ValueError('single continuous Dream session required')
 for i,x in enumerate(chunks,1):
  if x['index']!=i or x['continuous'] is not True or type(x['attempts']) is not int or x['attempts']<1 or not re.fullmatch('[0-9a-f]{64}',x['session_sha256']):raise ValueError('invalid session/transport receipt')
 projection=None
 if external_helper_sha256=='6699ef55f27bf42c859befd28047c4186148365e6c5f69c2a317927930228356':projection=verify_public_projection(P,out,chunks,J,reg,model_helper)
 frames={n:{} for n in CAMERAS};trials=set()
 for p in (out/'inspect/frames').rglob('*.npy'):
  m=re.fullmatch(r'~f1~([^~]*)~([^~]*)_(-?[0-9]+)\.npy',p.name)
  if m is None:raise ValueError('actual FrameStore name required')
  trial,camera=[urllib.parse.unquote_to_bytes(x).decode() for x in m.group(1,2)];tick=int(m.group(3));trials.add(trial)
  if camera not in frames or tick in frames[camera]:raise ValueError('unexpected or duplicate actual frame')
  frames[camera][tick]=p
 if len(trials)!=1 or any(set(x)!=set(range(steps+1)) for x in frames.values()):raise ValueError('actual frame continuity mismatch')
 import numpy as np
 def rgb(n,t):
  a=np.load(frames[n][t],allow_pickle=False)
  if a.dtype!=np.uint8 or a.shape!=(176,320,3):raise ValueError('actual frame RGB contract')
  return a
 media=None
 if encode:
  from video_capture import Capture
  dest=out/'final-model-media';dest.mkdir(exist_ok=False);capture=Capture(dest,{n:rgb(n,0) for n in CAMERAS})
  try:
   for t in range(1,steps+1):capture.append({n:rgb(n,t) for n in CAMERAS})
  finally:media=capture.close()
  if not media['all_encoded'] or media['frames_per_camera']!=steps+1 or any(not 0<(dest/n).stat().st_size<=50_000_000 for n in media['files']):raise ValueError('actual model encode incomplete')
 else:
  for n in CAMERAS:
   for t in range(steps+1):rgb(n,t)
 report={'status':'ACTUAL_B6_DREAM_COMPLETED_PENDING_ROOT_MEDIA_REVIEW','score':score,'steps':steps,'physics_samples':count,'native_status_score':0,'public_score_rederived_from_approved_channels':expected,'request_count':len(chunks),'sessions':1,'frames_per_camera':steps+1,'manifest_sha256':P.sha(root/'manifest.json'),'summary_sha256':P.sha(summaries[0]),'record_sha256':P.sha(records[0]),'media':media,'submission_ready':False,'external_model_helper_sha256':external_helper_sha256,'full_vector_audit':vectors,'force_verifier_sha256':force_verifier_sha256,'verifier_sha256':P.sha(Path(__file__)),'public_gripper_projection':projection,'policy_conversion':reg.get('policy_conversion','Dream-1 original decoder')};(out/'model-verification.json').write_text(json.dumps(report,indent=2)+'\n');return report
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--bundle',required=True,type=Path);p.add_argument('--out',required=True,type=Path);p.add_argument('--review',required=True,type=Path);p.add_argument('--encode',action='store_true');p.add_argument('--model-helper',type=Path);a=p.parse_args();print(json.dumps(verify(a.bundle,a.out,a.review,a.encode,a.model_helper)))
