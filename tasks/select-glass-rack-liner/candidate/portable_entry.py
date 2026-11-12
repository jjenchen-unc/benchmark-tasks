"""Standalone entry; real runtime owns its task-local import path."""
import argparse,runpy,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--mode',choices=['reference','replay','targeted','selected-force','setup','model'],required=True)
p.add_argument('--preflight-only',action='store_true');p.add_argument('--fresh-review',type=Path)
a,rest=p.parse_known_args()
sys.path.insert(0,str(HERE))
if a.mode=='model':
 from model_preflight import check
 if a.fresh_review is None:raise ValueError('ROOT fresh replay review required before model imports')
 import json
 readiness=check(HERE,a.fresh_review)
 rest += ['--fresh-review',str(a.fresh_review.resolve())]
 if a.preflight_only:print(json.dumps(readiness));raise SystemExit(0)
elif a.preflight_only:raise ValueError('preflight-only is a model option')
runner={'reference':'reference.py','replay':'replay.py','targeted':'evaluator.py','selected-force':'evaluator.py','setup':'setup.py','model':'model.py'}[a.mode]
extra=['--suite',a.mode] if a.mode in ('targeted','selected-force') else ['--public-case','--target','C1-T1','--seed','3867854786']
sys.path.insert(0,str(HERE));sys.argv=[str(HERE/runner),*extra,*rest]
runpy.run_path(sys.argv[0],run_name='__main__')
