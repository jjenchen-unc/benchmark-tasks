"""Chosen whole reference launcher, source-bound; no child process or simulator setup."""
import runpy,sys,hashlib,json
from pathlib import Path
R=Path(__file__).resolve().parent;C=R/'candidate'
expected=json.loads((R/'candidate-SHA256.json').read_text())
assert {str(p.relative_to(C)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(C.rglob('*')) if p.is_file()}==expected
sys.path.insert(0,str(C));sys.argv=[str(C/'foundation.py'),'--mode','reference','--out',sys.argv[1],'--legacy-rtx-override'];runpy.run_path(str(C/'foundation.py'),run_name='__main__')
