"""Run existing public entry in-process; no installation/service/process spawning."""
from pathlib import Path
import argparse,sys,runpy
R=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('phase',choices=['replay','setup','model']);p.add_argument('--candidate',type=Path,default=R/'candidate');p.add_argument('--out',type=Path,required=True);p.add_argument('--fresh-review',type=Path);p.add_argument('--preflight-only',action='store_true');a=p.parse_args()
sys.argv=[str(a.candidate.resolve()/'portable_entry.py'),'--mode',a.phase,'--out',str(a.out.resolve())]
if a.phase=='replay':sys.argv+=['--actions',str(R/'historical43748-actions.json'),'--source-hashes',str(R/'replay-source-hashes.json')]
if a.fresh_review:sys.argv+=['--fresh-review',str(a.fresh_review.resolve())]
if a.preflight_only:sys.argv+=['--preflight-only']
runpy.run_path(sys.argv[0],run_name='__main__')
