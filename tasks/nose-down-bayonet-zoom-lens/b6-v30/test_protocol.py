from pathlib import Path
import sys,json,time
O=Path(__file__).resolve().parent;sys.path.insert(0,str(O))
from protocol import check,receipt,loads,number,fresh
from strict_json import dumps
start=time.monotonic();check();tests=[]
for value in ['NaN','Infinity','1e400','{"x":1,"x":2}']:
 try:loads(value)
 except ValueError:tests.append('reject '+value)
 else:raise AssertionError(value)
for v in [True,'1',float('nan')]:
 try:number(v)
 except ValueError:tests.append('numeric ownership '+repr(v))
 else:raise AssertionError(v)
assert loads(dumps({'score':1.,'controls':3034}))=={'score':1.,'controls':3034}
for p in [O/'checks/absent-review.json',O/'provenance/B6-43897-reviewed.json',O/'provenance/B6-v14-reviewed.json']:
 try:receipt(p)
 except (ValueError,FileNotFoundError):tests.append('receipt failclosed '+p.name)
 else:raise AssertionError('old/missing receipt accepted')
try:fresh(O/'provenance')
except (ValueError,KeyError):tests.append('historical CPU result rejected as actual whole')
else:raise AssertionError('old CPU result accepted')
r=dict(passed=True,tests=tests,synthetic=True,physical_proof=False,wall_s=time.monotonic()-start)
(O/'checks/protocol.json').write_text(json.dumps(r,indent=2,allow_nan=False)+'\n');print(json.dumps(r))
