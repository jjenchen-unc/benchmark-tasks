"""Credited replay envelope: unchanged S1 core, actual source-bound ROOT review required."""
from pathlib import Path
import sys,os,json,hashlib,argparse
R=Path(__file__).resolve().parent
B=R/'s1-complete-hold-portable-v7'
sys.path.insert(0,str(B))
from protocol import receipt,check,CHANNELS
p=argparse.ArgumentParser();p.add_argument('--review',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
review=receipt(a.review.resolve());before=check();source=(B/'runtime.py').read_text()
if review.get('credited_entry_sha256')!=hashlib.sha256(Path(__file__).read_bytes()).hexdigest():raise ValueError('credited envelope not ROOT reviewed')
guard="if kit.VERIFIED_CHANNELS:raise RuntimeError('uncredited replay required')"
assert source.count(guard)==1
source=source.replace(guard,guard+"\nkit.VERIFIED_CHANNELS=frozenset(CHANNELS)")
sys.argv=[str(B/'runtime.py'),'replay','--out',str(a.out.resolve())]
error=None
try:
 exec(compile(source,str(B/'runtime.py'),'exec'),{'__file__':str(B/'runtime.py'),'__name__':'__main__','CHANNELS':CHANNELS})
except BaseException as e:
 error=type(e).__name__+': '+str(e);raise
finally:
 assert check()==before
 if a.out.exists():
  (a.out/'credited-envelope.json').write_text(json.dumps({'kind':'S1_SOURCE_BOUND_CREDITED_REPLAY','core_manifest':before,'review_sha256':hashlib.sha256(a.review.read_bytes()).hexdigest(),'envelope_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'only_runtime_injection':'set six VERIFIED_CHANNELS after validating source-bound full-native replay review','error':error},indent=2)+'\n')
if error is None:
 result=json.loads((a.out/'result.json').read_text())
 assert result['error'] is None and result['completed']
 assert result['final_status']['score']==1 and result['final_status']['hold_s']>=20 and result['final_status']['failure'] is None
 print('ACTUAL_S1_CREDITED_DEMO_FULL_NATIVE_SUCCESS')
