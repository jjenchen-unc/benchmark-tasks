"""Public entry: prepare | setup | demo | model. Outputs must be fresh and outside the package."""
import argparse,json,subprocess,sys
from pathlib import Path
R=Path(__file__).resolve().parent
ap=argparse.ArgumentParser();ap.add_argument('command',choices=['prepare','setup','demo','model']);ap.add_argument('--out',type=Path);ap.add_argument('--seed',type=int,default=42);a=ap.parse_args()
if a.out:
 o=a.out.resolve()
 if o.exists():sys.exit('out must be fresh')
 if o==R or R in o.parents:sys.exit('out must be outside the package')
py=sys.executable
if a.command in('prepare','setup'):
 sys.path.insert(0,str(R/'runtime'));import preflight;r=preflight.run(credit=True)
 if a.command=='setup':
  sys.path.insert(0,str(R/'runtime'));from overview_camera import overview;r['overview']=overview()
 print(json.dumps(r,indent=1));sys.exit(0)
script={'demo':'runtime/demo.py','model':'runtime/model_child.py'}[a.command]
if not a.out:sys.exit('--out required')
sys.exit(subprocess.call([py,'-I','-B',str(R/script),'--out',str(a.out.resolve()),'--seed',str(a.seed)]))
