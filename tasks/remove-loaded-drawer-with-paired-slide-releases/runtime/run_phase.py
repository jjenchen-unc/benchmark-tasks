"""Operator-only phases. Does nothing on import; no machine/service management."""
from pathlib import Path
import argparse,sys,subprocess,os,time,datetime
O=Path(__file__).resolve().parent;C=O/'candidate';sys.path.insert(0,str(O));sys.path.insert(0,str(C))
from runtime_credit import load,sha
from strict_json import dumps

def main():
 p=argparse.ArgumentParser();p.add_argument('phase',choices=['replay','setup','model']);p.add_argument('--out',type=Path,required=True);p.add_argument('--review');p.add_argument('--preflight-only',action='store_true');a=p.parse_args()
 from package_checks import check
 before=check();out=a.out.resolve()
 if out==O or out.is_relative_to(C) or out.exists():raise ValueError('fresh output outside candidate required')
 if a.phase=='model':
  from model_credit import validate_model
  if not a.preflight_only:validate_model(C,a.review)
 cmd=[sys.executable,'-I','-B',str(C/'pad_foundation.py'),'--legacy-rtx-override','--mode','reference' if a.phase=='replay' else 'setup','--out',str(out)]
 if a.phase=='model':cmd=[sys.executable,'-I','-B',str(O/'model_child.py'),'--out',str(out),'--review',a.review or '']
 if a.preflight_only:
  print(dumps({'status':'SOURCE_PREFLIGHT_ONLY','phase':a.phase,'command':cmd,'source_files':len(before),'model_calls':0,'simulator_initializations':0,'review_required_for_model':True}));return
 env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1')
 started=datetime.datetime.now(datetime.timezone.utc).isoformat();t=time.monotonic()
 rc=None
 try:rc=subprocess.run(cmd,env=env,check=False).returncode
 finally:
  after=check();out.mkdir(parents=True,exist_ok=True)
  (out/'launcher.json').write_text(dumps({'phase':a.phase,'argv':cmd,'exitcode':rc,'source_before':before,'source_after':after,'source_unchanged':before==after,'manifest_sha256':sha(C/'manifest.json'),'started_utc':started,'ended_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'wall_s':time.monotonic()-t},indent=2)+'\n')
 if rc:raise SystemExit(rc)
 if a.phase=='model':
  from verify_model import verify
  verify(out)
 if a.phase=='replay':
  from fresh_evidence import check as check_fresh
  print(dumps(check_fresh(C,out)))
if __name__=='__main__':main()
