"""Launch a single bounded requested suite, without any driver/SDK edits."""
import argparse,runpy,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
ap=argparse.ArgumentParser();ap.add_argument('--suite',required=True,choices=['foundation','friction','wet','guard','reset','reference','wet-reference','targeted']);ap.add_argument('--out',required=True);a=ap.parse_args()
if a.suite in ['reference','wet-reference']:
 runner='reference.py';extra=['--target','C1-T2' if a.suite=='wet-reference' else 'C1-T1']
else:runner='evaluator.py';extra=['--suite',a.suite]
sys.path.insert(0,str(HERE));sys.argv=[str(HERE/runner),'--out',a.out,*extra];runpy.run_path(str(HERE/runner),run_name='__main__')
