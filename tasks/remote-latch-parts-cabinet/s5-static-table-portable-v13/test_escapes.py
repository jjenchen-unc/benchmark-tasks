"""Targeted persistent audit escape checks; no SDK initialization."""
from pathlib import Path
import sys,subprocess,json,tempfile,time
O=Path(__file__).resolve().parent;d=Path(tempfile.mkdtemp(prefix='CPU-ESCAPES-',dir=O/'work'));rows=[]
cases={'execv':("import os,sys;os.execv(sys.executable,[sys.executable])",87),'execve':("import os,sys;os.execve(sys.executable,[sys.executable],{})",87),'Popen_raw':("import subprocess;subprocess.Popen(['/bin/true'])",87),'spawn':("import os;os.posix_spawn('/bin/true',['/bin/true'],{})",87),'fork':("import os;os.fork()",87),'cleanup':("import sys;sys.meta_path[:]=[];sys.path_hooks[:]=[];sys.path_importer_cache.clear();import isaaclab",86),'loader':("exec(compile('pass','/synthetic/isaaclab/escape.py','exec'))",86)}
t=time.monotonic()
for n,(body,expected) in cases.items():
 p=d/(n+'.py');p.write_text(body+'\n');log=d/(n+'.jsonl');r=subprocess.run([sys.executable,'-B','-I',str(O/'import_audit.py'),str(log),str(p)],capture_output=True,text=True);rows.append(dict(name=n,expected=expected,returncode=r.returncode,passed=r.returncode==expected))
report=dict(passed=all(r['passed'] for r in rows),checks=rows,physics_proof=False,wall_s=time.monotonic()-t);(O/'checks/escapes.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report));assert report['passed']
