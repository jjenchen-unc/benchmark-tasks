"""Read-only, fail-closed selected-path ROOT review binding; no task-state writes."""
from pathlib import Path
import hashlib,json,math,ast
REQUIRED=frozenset(('geometry','clock','reset','contacts','jar_normal_speed','pad_identity','wrist','stop_masks','motor_total','finger_40N'))
def task_code_sha(p):
 tree=ast.parse(Path(p).read_text())
 if tree.body and isinstance(tree.body[0],ast.Expr) and isinstance(tree.body[0].value,ast.Constant) and isinstance(tree.body[0].value.value,str):tree.body=tree.body[1:]
 # Python3.12 adds empty type_params fields; normalize source-equivalent ASTs.
 for node in ast.walk(tree):
  if hasattr(node,'type_params') and node.type_params==[]:delattr(node,'type_params')
 return hashlib.sha256(ast.dump(tree,include_attributes=False).encode()).hexdigest()
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def loads(text):
 def pairs(items):
  d={}
  for k,v in items:
   if k in d:raise ValueError('duplicate JSON key: '+k)
   d[k]=v
  return d
 def bad(s):raise ValueError('nonfinite number '+s)
 data=json.loads(text,object_pairs_hook=pairs,parse_constant=bad)
 def finite(x):
  if type(x) is float and not math.isfinite(x):raise ValueError('nonfinite numeric ownership')
  if isinstance(x,dict):
   for v in x.values():finite(v)
  if isinstance(x,list):
   for v in x:finite(v)
 finite(data);return data

def load(p):return loads(Path(p).read_text())

def validate(core,review_path):
 core=Path(core);review=load(review_path)
 if review.get('schema')!='drawers-root-selected-43920-v1' or review.get('reviewer')!='ROOT' or review.get('approved') is not True:raise ValueError('missing explicit ROOT approval')
 if type(review.get('job_id')) is not int or review['job_id']!=43920:raise ValueError('wrong actual job')
 contract=load(core/'credit-contract.json')
 if review.get('core_sha256')!=contract['core_sha256']:raise ValueError('review core map mismatch')
 for n,d in contract['core_sha256'].items():
  if sha(core/n)!=d:raise ValueError('portable core changed: '+n)
 if review.get('proof_sha256')!=contract['proof_sha256']:raise ValueError('selected proof maps mismatch')
 for n,d in contract['proof_sha256'].items():
  if sha(core/n)!=d:raise ValueError('portable proof changed: '+n)
 if task_code_sha(core/'task.py')!=contract['task_code_sha256']:raise ValueError('task executable code changed')
 decisions=review.get('channels',{})
 if set(decisions)!=REQUIRED:raise ValueError('explicit ten-channel ROOT decisions required')
 for name,row in decisions.items():
  if row.get('approved') is not True or not isinstance(row.get('evidence'),list) or not row['evidence'] or not all(isinstance(v,str) and v for v in row['evidence']):raise ValueError('unapproved or unsupported channel: '+name)
 allowed={'actual:'+k for k in ('launcher','result','journal','sourcehashes')}|{'proof:'+k for k in contract['proof_sha256']}
 if any(e not in allowed for row in decisions.values() for e in row['evidence']):raise ValueError('unknown evidence reference')
 # ROOT binds exact actual inputs. Relative paths are relative to the review.
 base=Path(review_path).resolve().parent
 def resolve(s):
  if not isinstance(s,str) or Path(s).is_absolute():raise ValueError('actual evidence requires portable relative path')
  p=(base/s).resolve()
  if not p.is_relative_to(core.resolve().parent):raise ValueError('actual evidence escapes portable bundle')
  return p
 inputs=review.get('actual_inputs',{})
 if set(inputs)!={'launcher','result','journal','sourcehashes'}:raise ValueError('actual full run inputs missing')
 paths={}
 for k,v in inputs.items():
  paths[k]=resolve(v['path'])
  if sha(paths[k])!=v['sha256']:raise ValueError('actual input changed: '+k)
 launch=load(paths['launcher']);result=load(paths['result']);source=load(paths['sourcehashes'])
 expected=contract['selected_source_hashes']
 if launch.get('exitcode')!=0 or type(launch.get('exitcode')) is not int or launch.get('source_unchanged') is not True or launch.get('source_hashes_before')!=expected or launch.get('source_hashes_after')!=expected:raise ValueError('actual launcher/source mismatch')
 argv=launch.get('argv',[])
 if not any('job-43920' in x for x in argv) or '--mode' not in argv or argv[argv.index('--mode')+1]!='reference':raise ValueError('wrong selected actual run')
 if source.get('sha256')!={k:v for k,v in expected.items() if k!='manifest.json'}:raise ValueError('actual embedded source map mismatch')
 if result.get('status')!='foundation_observed' or result.get('first_termination') is not None:raise ValueError('actual did not complete cleanly')
 rows=[]
 for line in paths['journal'].read_text().splitlines():
  # Reuse strict parser without writing temporary files.
  row=loads(line)
  rows.append(row)
 if len(rows)<2:raise ValueError('actual observations absent')
 last=rows[-1];status=last['status'];p=last['physics']
 def num(x):
  if type(x) not in (int,float) or not math.isfinite(x):raise ValueError('number required; bool is not numeric')
  return x
 for i,row in enumerate(rows):
  s=row['status'];c=num(s['controls']);samples=num(s['samples']);t=num(s['physics_s'])
  if type(c) is not int or c!=i or type(samples) is not int or samples!=12*c or abs(t-c/20)>1e-6:raise ValueError('actual row/clock mismatch')
  if s['failure'] is not None or s['measurement_error'] is not None:raise ValueError('actual safety/measurement failure')
  if row.get('first_termination') is not None:raise ValueError('early termination')
 if num(status['controls'])>3586 or num(status['candidate_score'])!=1 or num(status['hold_s'])<1-1e-9:raise ValueError('native full goal/1s hold absent')
 if result.get('status_channels')!=status:raise ValueError('terminal status/journal mismatch')
 if p['contents'] is not True or p['drawer_touch'] is not False or p['drop'] is not False or num(p['max_x'])>=.12 or abs(num(p['drawer_p'][2])-.835)>=.006 or num(p['drawer_p'][0])>=0 or num(p['drawer_R'][2][2])<=math.cos(math.radians(3)):raise ValueError('terminal native predicate fails')
 # Full substep/contact semantics are explicitly ROOT-reviewed, not inferred
 # from control-rate rows. No raw contact-rate or pad exemption claim is invented.
 return {'status':'ROOT_REVIEWED_SELECTED_PATH','job_id':43920,'channels':sorted(REQUIRED),'actual_controls':status['controls'],'actual_samples':status['samples'],'actual_physics_s':status['physics_s'],'native_hold_s':status['hold_s'],'candidate_score':status['candidate_score'],'review_sha256':sha(review_path),'physical_scope':'selected fixed DR-T1/S seed42 only; no all-variant claim','submission_ready':False,'model_score':None}

def configure(core,review_path):
 import drawers_physics
 if review_path is None:
  if drawers_physics.VALIDATED_CHANNELS:raise ValueError('unexpected ungated credit')
  return {'status':'PENDING_ROOT_PHYSICAL_REVIEW','runtime_channels_promoted':[]}
 receipt=validate(core,review_path)
 drawers_physics.VALIDATED_CHANNELS=REQUIRED
 return receipt
