"""Observability probe: trial-fit one lens on the overhead mount and, if it locks, watch its zoom after release."""
from pathlib import Path
import sys,argparse,json
O=Path(__file__).resolve().parent;C=O/'scene';sys.path.insert(0,str(O));sys.path.insert(0,str(C))
p=argparse.ArgumentParser();p.add_argument('--lens',choices=['lens_orange','lens_blue','lens_purple'],required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
out=a.out.resolve()
from yam_arena.task_file import load
load(C/'task.py')
from isaaclab.app import AppLauncher
previous=AppLauncher.__init__
def launch(self,*args,**kw):
 kw['kit_args']=(kw.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip();return previous(self,*args,**kw)
AppLauncher.__init__=launch
import reference
out.mkdir(parents=True,exist_ok=False)
r=reference.run(out,'probe:'+a.lens)
(out/'probe.json').write_text(json.dumps(r,indent=2,default=str)+'\n')
print('B6_PROBE '+json.dumps(dict(lens=a.lens,decisions=r['decisions'],last_phase=r['last_phase'],controls=r['controls'])),flush=True)
