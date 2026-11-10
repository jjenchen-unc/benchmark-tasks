"""Actual operator parent + isolated chosen child under persistent cold audit."""
from pathlib import Path
import subprocess,sys,json,time,hashlib
O=Path(__file__).resolve().parent;out=O/'work'/('CPU-GUARD-'+str(time.time_ns()));log=O/'checks'/('entry-'+str(time.time_ns())+'.jsonl')
cmd=[sys.executable,'-B','-I',str(O/'import_audit.py'),str(log),str(O/'run_phase.py'),'replay','--out',str(out)]
t=time.monotonic();r=subprocess.run(cmd,capture_output=True,text=True);events=[json.loads(s) for s in log.read_text().splitlines()]
proof=dict(passed=r.returncode==86 and any(e['event']=='task_constructed' and e['objects']==8 for e in events) and any(e['event']=='sdk_boundary' and e['module']=='isaaclab.app' and e['isolated']==1 for e in events),argv=cmd,returncode=r.returncode,events=events,stdout=r.stdout,stderr=r.stderr,wall_s=time.monotonic()-t,physical_proof=False,artificial_path_injection=False)
(O/'checks/actual-entry.json').write_text(json.dumps(proof,indent=2)+'\n');print(json.dumps({k:v for k,v in proof.items() if k not in ('events','stdout','stderr')}));assert proof['passed']
