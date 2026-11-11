from pathlib import Path
import json,hashlib,shutil,ast
O=Path(__file__).resolve().parent;R=Path.cwd();S=R/'artifacts/submission-readiness/s1-actual-pop-contact-whole-v5-20261110-0900';B=R/'artifacts/submission-readiness/b1-fullportable-v8-20261110-1440'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
cases=json.loads((R/'artifacts/submission-readiness/root-reviewed-next-20261110-1005/cases.json').read_text())
c=next(x for x in cases if x['source']==str(S/'source'))
actual={str(p.relative_to(S/'source')):sha(p) for p in (S/'source').rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc'}
assert actual==c['source_hashes'], 'dispatch source mismatch'
shutil.copytree(S/'source',O/'candidate',ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
(O/'provenance').mkdir()
for name in ['REPORT.md','result.json','candidate-checks.json','actual-contact-analysis.json','grasp-geometry.json','consumed-source-sha256.json','evidence-reuse.json','pre-sdk-check.json']:
 shutil.copy2(S/name,O/'provenance'/name)
(O/'provenance/dispatch-source-receipt.json').write_text(json.dumps(c,indent=2)+'\n')
for name in ['video_capture.py','strict_json.py','verify_runtime.py','runtime-source-pins.json']:
 shutil.copy2(B/name,O/name)
# Reuse exact independently implemented all-link/FK observer functions from native runner.
tree=ast.parse((S/'source/run.py').read_text());main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
functions=[ast.unparse(n) for n in main.body if isinstance(n,ast.Try) for n in n.body if isinstance(n,ast.FunctionDef) and n.name in ['robot_snapshot','fixture_snapshot','geometry_proxies']]
assert len(functions)==3
(O/'observer.py').write_text('import numpy as np\nimport s1_kit as kit\nfrom contacts import arr\nemb=None\n'+'\n\n'.join(functions)+'\n\ndef observe(obs):\n r=robot_snapshot(obs);f=fixture_snapshot();return dict(robot=r,fixture=f,geometry_proxies=geometry_proxies(r,f))\n')
print(json.dumps({'dispatch_source_verified':True,'core_files':len(actual),'controller_sha256':actual['reference.py']}))
