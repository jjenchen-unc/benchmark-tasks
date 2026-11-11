from pathlib import Path
import sys,argparse,os
O=Path(__file__).resolve().parent;sys.path.insert(0,str(O))
from protocol import check,fresh,receipt,CHANNELS,sha
from strict_json import dumps
p=argparse.ArgumentParser();p.add_argument('--replay',type=Path);p.add_argument('--out',type=Path);p.add_argument('--validate',type=Path);a=p.parse_args()
if a.validate:print(dumps(receipt(a.validate)));raise SystemExit(0)
if a.replay is None or a.out is None:raise ValueError('replay and out required')
f=a.replay.resolve();summary=fresh(f);out=a.out.resolve()
if out.exists() or not f.is_relative_to(out.parent):raise ValueError('fresh receipt beside replay required')
r={'schema':'s2-v19-root-physical-review','reviewer':None,'approved':False,'manifest':check(),'replay':str(f.relative_to(out.parent)),'actual_summary':summary,'evidence_sha256':{str(p.relative_to(f)):sha(p) for p in f.rglob('*') if p.is_file()},'channels':{n:{'approved':False,'rationale':'','proofs':[]} for n in sorted(CHANNELS)}}
out.write_text(dumps(r,indent=2)+'\n');print(str(out))
