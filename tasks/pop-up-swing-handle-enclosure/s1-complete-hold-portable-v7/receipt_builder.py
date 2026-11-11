from pathlib import Path
import sys,argparse,shutil
O=Path(__file__).resolve().parent;sys.path.insert(0,str(O))
from protocol import fresh,receipt
from strict_json import dumps
p=argparse.ArgumentParser();p.add_argument('--replay',type=Path);p.add_argument('--out',type=Path);p.add_argument('--validate',type=Path);a=p.parse_args()
if a.validate:print(dumps(receipt(a.validate)));raise SystemExit()
r=fresh(a.replay)
a.out.mkdir(parents=True,exist_ok=False);shutil.copytree(a.replay,a.out/'replay');(a.out/'ROOT-REVIEW-UNAPPROVED.json').write_text(dumps(r,indent=2)+'\n')
print('Unapproved review form only; ROOT must review fresh physical channels.')
