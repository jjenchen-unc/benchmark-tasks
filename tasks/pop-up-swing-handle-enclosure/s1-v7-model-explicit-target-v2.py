"""Apply native embodiment-equivalent gripper saturation before Dream decoding; verify in a separate parent."""
from pathlib import Path
import argparse,sys,os,json,hashlib,runpy,datetime
import numpy as np
EXPECTED='6d47a7ff48fe7349ef8bba5768f77a371d00394308d652586a7ff95320ec152f'
ROWS=4
def consume_rows(reply,station,original):
 arrays={}
 for side in station._ARMS:
  key='q_'+side+'_chunk'
  if key not in reply:raise ValueError('Dream reply missing '+key)
  a=np.array(reply[key],dtype=np.float64)
  if a.ndim!=2 or a.shape[1]!=station.ARM_DIM or not 1<=len(a)<=station.MAX_ROWS:raise ValueError('Dream fullchunk shape invalid '+key)
  if not np.isfinite(a).all():raise ValueError('Dream fullchunk nonfinite '+key)
  arrays[key]=a
 values=list(arrays.values())
 if len(values)!=2 or values[0].shape!=values[1].shape:raise ValueError('Dream left/right chunks differ')
 limited=dict(reply)
 for key,a in arrays.items():
  bounded=a.copy();bounded[:,station.ARM_JOINTS]=np.clip(a[:,station.ARM_JOINTS],0.,1.);limited[key]=bounded
 rows=original(limited)
 return rows,arrays


def declared_input(B,path):
 data=json.loads(path.read_text());old=json.loads((B/'candidate/task.draft.json').read_text())['instruction']
 expected=old.replace("Press box 1's red cap,","Box 1 is the enclosure on the robot's right (negative Y). Press its red cap,",1)
 if data['instruction']!=expected or expected==old or data['source_instruction_sha256']!=hashlib.sha256(old.encode()).hexdigest():raise ValueError('input is not exact spatial clarification')
 if data['physical_core_fingerprint']!=EXPECTED or data['physical_target']!='SH-T1' or data['target_box']!=1 or data['target_shell_center_m']!=[.25,-.32,.7975]:raise ValueError('physical target changed')
 return data
def patch_input(B,out,path):
 data=declared_input(B,path);text=data['instruction'];digest=hashlib.sha256(text.encode()).hexdigest()
 import inspect_robots.cli as cli
 from inspect_robots_dream.session import DreamSession
 original_main=cli.main;original_observe=DreamSession.observe
 def main():
  if sys.argv.count('--instruction')!=1:raise ValueError('one instruction argument required')
  i=sys.argv.index('--instruction')+1
  old=json.loads((B/'candidate/task.draft.json').read_text())['instruction']
  if sys.argv[i]!=old:raise ValueError('unexpected original CLI instruction')
  reg=json.loads((out/'registration.json').read_text())
  if reg['instruction']!=old:raise ValueError('unexpected original registration')
  sys.argv[i]=text;reg['instruction']=text
  (out/'registration.json').write_text(json.dumps(reg,indent=2)+'\n')
  (out/'declared-input.json').write_text(json.dumps(data,indent=2)+'\n')
  return original_main()
 def observe(self,frames,state,prompt,timestamp):
  if prompt!=text:raise ValueError('actual Dream prompt differs from declared input')
  with (out/'actual-prompt-sha256.jsonl').open('a') as f:f.write(json.dumps({'instruction_sha256':digest,'timestamp':timestamp})+'\n')
  return original_observe(self,frames,state,prompt,timestamp)
 cli.main=main;DreamSession.observe=observe
def final_verify(B,out,path):
 import importlib.util,ast
 data=declared_input(B,path)
 tree=ast.parse((B/'verify_model.py').read_text())
 uses=[n for n in ast.walk(tree) if isinstance(n,ast.Name) and n.id=='C' and isinstance(n.ctx,ast.Load)]
 if len(uses)!=2:raise ValueError('unexpected verifier metadata dependencies')
 spec=importlib.util.spec_from_file_location('_s1_final_input_verifier',B/'verify_model.py')
 module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
 module.C=path.parent
 prompts=[json.loads(x) for x in (out/'actual-prompt-sha256.jsonl').read_text().splitlines()]
 digest=hashlib.sha256(data['instruction'].encode()).hexdigest()
 if not prompts or any(x['instruction_sha256']!=digest for x in prompts):raise ValueError('actual prompt evidence missing/mismatch')
 if json.loads((out/'declared-input.json').read_text())!=data:raise ValueError('actual input receipt mismatch')
 return module.verify(out)

def child(B,review,out,input_file):
 patch_input(B,out,input_file)
 import inspect_robots_dream._station as station
 original=station.rows;count=0
 def rows(reply):
  nonlocal count
  try:decoded,arrays=consume_rows(reply,station,original)
  except ValueError as error:
   def safe(x):
    if isinstance(x,list):return [safe(v) for v in x]
    if isinstance(x,(float,np.floating)) and not np.isfinite(x):return str(x)
    return x
   raw={}
   for key in ['q_l_chunk','q_r_chunk']:
    if key in reply:
     try:raw[key]=safe(np.array(reply[key],dtype=np.float64).tolist())
     except (TypeError,ValueError):raw[key]='unparseable numeric chunk'
   with (out/'rejected-prefix-decoder.jsonl').open('a') as f:f.write(json.dumps({'error':str(error),'raw_prediction':raw},allow_nan=False)+'\n')
   raise
  count+=1
  with (out/'consumed-prefix-decoder.jsonl').open('a') as f:f.write(json.dumps({'request':count,'consumed_rows':min(ROWS,len(decoded)),'raw_prediction_rows':len(next(iter(arrays.values()))),'raw_prediction':{k:v.tolist() for k,v in arrays.items()},'decoded_consumed_rows':decoded[:ROWS].tolist(),'projected_gripper_values':{k:int(np.count_nonzero((v[:ROWS,-1]<0)|(v[:ROWS,-1]>1))) for k,v in arrays.items()},'extra_over_native_decoder_slack':{k:int(np.count_nonzero((v[:ROWS,-1]<-station._GRIPPER_SLACK)|(v[:ROWS,-1]>1+station._GRIPPER_SLACK))) for k,v in arrays.items()}},separators=(',',':'))+'\n')
  return decoded
 station.rows=rows
 sys.argv=[str(B/'model_child.py'),'--review',str(review),'--out',str(out)]
 runpy.run_path(str(B/'model_child.py'),run_name='__main__')
def main():
 import subprocess
 p=argparse.ArgumentParser();p.add_argument('--core',required=True,type=Path);p.add_argument('--review',required=True,type=Path);p.add_argument('--out',required=True,type=Path);p.add_argument('--input-file',required=True,type=Path);p.add_argument('--preflight-only',action='store_true');p.add_argument('--child',action='store_true',help=argparse.SUPPRESS);a=p.parse_args()
 B=a.core.resolve();review=a.review.resolve();out=a.out.resolve();sys.path.insert(0,str(B))
 input_file=a.input_file.resolve();declared_input(B,input_file);input_sha=hashlib.sha256(input_file.read_bytes()).hexdigest()
 from protocol import check,receipt
 from verify_runtime import verify
 assert check()==EXPECTED;receipt(review);runtime=verify()
 if a.preflight_only:
  print(json.dumps({'status':'S1_EXPLICIT_TARGET_PARENT_CHILD_PREFLIGHT_PASS','source':EXPECTED,'rows_per_request':ROWS,'runtime':runtime,'simulator_initializations':0,'model_calls':0}));return
 if out.exists() or out.is_relative_to(B):raise ValueError('fresh external output required')
 if a.child:child(B,review,out,input_file);return
 entry=Path(__file__).resolve();before=hashlib.sha256(entry.read_bytes()).hexdigest()
 cmd=[sys.executable,'-I','-B',str(entry),'--child','--core',str(B),'--review',str(review),'--out',str(out),'--input-file',str(input_file)]
 rc=subprocess.run(cmd,env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1'),check=False).returncode
 assert check()==EXPECTED and hashlib.sha256(entry.read_bytes()).hexdigest()==before and hashlib.sha256(input_file.read_bytes()).hexdigest()==input_sha
 if not out.exists():raise RuntimeError('child produced no actual trial')
 writer='child'
 if not (out/'source-after.json').exists():
  (out/'source-after.json').write_text(json.dumps(check())+'\n');writer='parent_after_child_exit'
 envelope={'kind':'S1_EXPLICIT_TARGET_PARENT_CHILD_MODEL','source':EXPECTED,'entry_sha256':before,'input_sha256':input_sha,'instruction_sha256':hashlib.sha256(declared_input(B,input_file)['instruction'].encode()).hexdigest(),'review_sha256':hashlib.sha256(review.read_bytes()).hexdigest(),'child_exitcode':rc,'source_after_writer':writer,'source_after_verified_by_parent':True,'rows_per_request':ROWS,'native_decoder_after_embodiment_equivalent_gripper_saturation':True,'gripper_bounds':[0.,1.],'joint_values_unmodified':True,'automatic_retry':False,'session_recovery':False,'evaluation_valid':False}
 (out/'model-execution-envelope.json').write_text(json.dumps(envelope,indent=2)+'\n')
 if rc:raise RuntimeError('model child exit '+str(rc))
 result=final_verify(B,out,input_file)
 envelope.update(evaluation_valid=True,actual_score=result['score'],actual_steps=result['steps'])
 (out/'model-execution-envelope.json').write_text(json.dumps(envelope,indent=2)+'\n')
if __name__=='__main__':main()
