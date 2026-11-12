"""Fail-closed split proof; ROOT review attestations are not cryptographic signatures."""
from pathlib import Path
import functools,hashlib,json,math
CHANNELS=frozenset('geometry explicit_pair_filters dry_slip_hold wet_transition contacts_bracing wrist_force link_guard total_motor_limits genuine_robot_fullgoal'.split())
def strict(path):
 def pairs(rows):
  d={}
  for k,v in rows:
   if k in d:raise ValueError('duplicate JSON key')
   d[k]=v
  return d
 v=json.loads(Path(path).read_text(),object_pairs_hook=pairs,parse_constant=lambda s:(_ for _ in ()).throw(ValueError(s)))
 def finite(x):
  if type(x) is float and not math.isfinite(x):raise ValueError('nonfinite/overflow')
  if isinstance(x,dict):
   for a in x.values():finite(a)
  if isinstance(x,list):
   for a in x:finite(a)
 finite(v);return v
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def number(v):
 if type(v) not in (int,float) or not math.isfinite(v):raise ValueError('finite JSON number required; boolean forbidden')
 return v
def integer(v):
 if type(v) is not int:raise ValueError('integer required; boolean forbidden')
 return v
def vector(v):
 if type(v) is not list or len(v)!=3:raise ValueError('3-vector required')
 return [number(x) for x in v]
def norm(v):return math.sqrt(sum(x*x for x in v))
def force_rows(basis,axes,contact_free):
 expected={(a,i,s) for a in ('left','right') for i in axes for s in (-1,1)};seen=set();out=[]
 for row in basis['rows']:
  axis=integer(row['axis']);sign=integer(row['sign']);key=(row['arm'],axis,sign)
  if axis not in axes:continue
  if key not in expected or key in seen:raise ValueError('duplicate/invalid case')
  seen.add(key);f=[5.*sign if i==axis else 0. for i in range(3)]
  if vector(row['applied_world_n'])!=f:raise ValueError('not signed5N')
  means=[];ids_both=[]
  for name in ('before','after'):
   window=row[name]
   if type(window) is not list or len(window)!=40:raise ValueError('requires40controls')
   ids=[];values=[]
   for sample in window:
    ids.append(integer(sample['samples']));world=vector(sample['world']);sensor=vector(sample['sensor']);rot=sample['R']
    if type(rot) is not list or len(rot)!=3:raise ValueError('rotation shape')
    rot=[vector(v) for v in rot];reconstructed=[sum(rot[i][j]*sensor[j] for j in range(3)) for i in range(3)]
    if norm([world[i]-reconstructed[i] for i in range(3)])>1e-5:raise ValueError('frame mismatch')
    if contact_free and sample.get('outside_distal_contacts')!=[]:raise ValueError('external contact/missing guard')
    values.append(world)
   if any(b-a!=50 for a,b in zip(ids,ids[1:])):raise ValueError('nonconsecutive samples')
   means.append([sum(v[i] for v in values[-20:])/20 for i in range(3)]);ids_both.append(ids)
  if ids_both[1][0]-ids_both[0][-1]!=50:raise ValueError('window gap')
  delta=[means[1][i]-means[0][i] for i in range(3)];ve=norm([delta[i]+f[i] for i in range(3)]);ne=abs(norm(delta)-5.)
  if ve>1. or ne>1.:raise ValueError('exceeds unchanged1N')
  out.append(dict(arm=key[0],axis=axis,sign=sign,mean_world_delta_n=delta,vector_error_n=ve,norm_error_n=ne,sample_ranges=[[v[0],v[-1]] for v in ids_both]))
 if seen!=expected:raise ValueError('incomplete cases')
 return out
def split_proof(root,basis,result,source):
 root=Path(root);policy=strict(root/'evidence/split-force-policy.json')
 if source!=policy['new_source_sha256']:raise ValueError('actual43944 source differs from dispatchedv5')
 if basis.get('scope')!='FOUR_X_ONLY' or len(basis['rows'])!=4:raise ValueError('not fourX')
 for k,v in [('tolerance_n',1.),('expected_delta_norm_n',5.),('window_controls',40),('mean_tail_controls',20)]:
  if (integer(basis[k]) if k in ('window_controls','mean_tail_controls') else number(basis[k]))!=v:raise ValueError('changed force semantics')
 if result.get('error') or result.get('suite')!='selected-force' or result.get('all_checks_passed') is not True:raise ValueError('new actual not terminal passing')
 for k in ('known_5N_four_X_both_arms_magnitude','known_5N_four_X_both_arms_reaction_basis'):
  if result['checks'].get(k) is not True:raise ValueError('missing actual check')
 old=root/'evidence/actual-43824-known-force-basis.json'
 if sha(old)!=policy['old_basis_sha256']:raise ValueError('oldYZ changed')
 return dict(new_job=43944,reused_job=43824,new_X=force_rows(basis,(0,),True),reused_YZ=force_rows(strict(old),(1,2),False),old_X_failed=True,all_12_new_tests_passed=False,tolerance_n=1.)
def source_map(root):
 return {str(p.relative_to(root)):sha(p) for p in sorted(Path(root).rglob('*')) if p.is_file() and '__pycache__' not in p.parts and p.name not in ('manifest.json','c1-review-receipt.json')}
def verify(root):
 root=Path(root)
 try:
  r=strict(root/'c1-review-receipt.json')
  if r['kind']!='ROOT_APPROVED_SOURCEBOUND_SPLIT_C1' or r['root_approved'] is not True or r['channels']!=sorted(CHANNELS):return False
  if source_map(root)!=r['source_sha256']:return False
  evidence=r['evidence_sha256']
  if not evidence:return False
  for n,h in evidence.items():
   p=(root/n).resolve()
   if not p.is_relative_to(root.resolve()) or sha(p)!=h:return False
  review=strict(root/'evidence/ROOT-split-review.json')
  if review['kind']!='ROOT_APPROVED_SPLIT_FORCE_C1' or review['root_approved'] is not True or integer(review['new_job'])!=43944 or integer(review['reused_job'])!=43824:return False
  if review['evidence_sha256']!={n:h for n,h in evidence.items() if n!='evidence/ROOT-split-review.json'}:return False
  proof=split_proof(root,strict(root/'evidence/new-X-basis.json'),strict(root/'evidence/new-X-result.json'),strict(root/'evidence/new-X-source-sha256.json'))
  if json.dumps(proof,sort_keys=True,allow_nan=False)!=json.dumps(r['split_proof'],sort_keys=True,allow_nan=False):return False
  if review['channels']!=sorted(CHANNELS) or review['limitations_acknowledged'] is not True:return False
  from runtime_binding import verify_runtime
  verify_runtime(root)
  return True
 except (OSError,ValueError,KeyError,TypeError,IndexError,OverflowError,RuntimeError,ImportError):return False
@functools.lru_cache(maxsize=1)
def validated_channels():return CHANNELS if verify(Path(__file__).resolve().parent) else frozenset()
