"""Local operator process only. No service/scheduler operations."""
from pathlib import Path
import sys,os,argparse,subprocess,time
O=Path(__file__).resolve().parent;sys.path.insert(0,str(O))
from protocol import check,receipt,fresh
from strict_json import dumps
p=argparse.ArgumentParser();p.add_argument('phase',choices=['replay','setup','model']);p.add_argument('--out',type=Path,required=True);p.add_argument('--review',type=Path);p.add_argument('--preflight-only',action='store_true');a=p.parse_args()
before=check();out=a.out.resolve()
if out.exists() or out.is_relative_to(O/'candidate'):raise ValueError('fresh output required')
cmd=[sys.executable,'-B','-I',str(O/'runtime.py'),a.phase,'--out',str(out)]
if a.phase=='model':
 if not a.review:raise ValueError('ROOT physical review required')
 receipt(a.review);cmd=[sys.executable,'-B','-I',str(O/'model_child.py'),'--review',str(a.review.resolve()),'--out',str(out)]
if a.preflight_only:print(dumps({'command':cmd,'simulator_initializations':0,'model_calls':0}));raise SystemExit()
t=time.monotonic();rc=None
try:rc=subprocess.run(cmd,env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1')).returncode
finally:
 after=check();out.mkdir(parents=True,exist_ok=True);(out/'launcher.json').write_text(dumps({'phase':a.phase,'exitcode':rc,'source_before':before,'source_after':after,'wall_s':time.monotonic()-t,'argv':cmd},indent=2)+'\n')
if rc:raise SystemExit(rc)
if a.phase=='replay':print(dumps(fresh(out)))

if a.phase=='model':
 from verify_model import verify
 verify(out)
