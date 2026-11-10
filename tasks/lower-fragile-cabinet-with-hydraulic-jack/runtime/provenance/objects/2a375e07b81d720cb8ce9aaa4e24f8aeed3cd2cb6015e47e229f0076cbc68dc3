"""Source-equivalent verification; no simulator imports or task writes."""
from pathlib import Path
import ast,hashlib,json
R=Path(__file__).resolve().parent;C=R/'candidate';OLD=R.parent/'hydraulic-actual-contact-whole-v3-20261110-0840'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
m=json.loads((R/'candidate-SHA256.json').read_text());assert {str(p.relative_to(C)):sha(p) for p in C.iterdir() if p.is_file()}==m
for n,h in json.loads((C/'sourcehashes.json').read_text()).items():assert sha(C/n)==h
for p in (OLD/'candidate').iterdir():
 if p.is_file() and p.name not in ('task.py','kit.py','reference.py','sourcehashes.json'):assert sha(C/p.name)==sha(p)
logs=[json.loads(s) for s in (R/'guard.jsonl').read_text().splitlines()];assert [l['event'] for l in logs]==['guard_installed','actual_task_constructed','boundary'];assert logs[-1]['module']=='isaaclab.app' and logs[0]['isolated']==1
assert not any(str(R) in p for p in logs[0]['path'])
a=(R/'guard.py').read_text().replace(R.name,OLD.name);assert a==(OLD/'guard.py').read_text();assert all(x['pass'] for x in json.loads((OLD/'guard-regressions.json').read_text()))
for x in ast.walk(ast.parse((C/'reference.py').read_text())):
 if isinstance(x,ast.Call):assert not any(w in ast.unparse(x.func) for w in ('write_root','write_joint','set_joint','set_external','set_pose'))
print('PASS: frozen candidate, source-equivalent unchanged physics, actual clean -I task/launcher, reused identical guard escape logic, controller write-call audit')
