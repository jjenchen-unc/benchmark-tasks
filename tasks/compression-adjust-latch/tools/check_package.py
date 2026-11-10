"""Pure checks and integrity negatives; never imports simulator/SDK."""
from pathlib import Path
import ast,json,sys,shutil,tempfile,hashlib
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from package_checks import preflight,dumps,digest

def main():
 checks=[]
 rows=preflight(ROOT);checks.append('29 runtime files; 10 calibration-bound files; 7 channels; exact1959 actions and samples')
 for p in ROOT.rglob('*.py'):ast.parse(p.read_text(),filename=str(p))
 checks.append('all Python AST parses')
 import numpy as np
 roundtrip=json.loads(dumps({'scalar':np.float32(.25),'array':np.array([1.,2.]),'tuple':(np.float64(.5),)}))
 assert roundtrip=={'scalar':.25,'array':[1.,2.],'tuple':[.5]}
 for v in [object(),np.float32('nan'),float('inf'),np.array([float('-inf')])]:
  try:dumps(v)
  except (TypeError,ValueError):pass
  else:raise AssertionError('Invalid telemetry accepted')
 checks.append('NumPy numeric roundtrip; unknown/NaN/infinity rejected')
 with tempfile.TemporaryDirectory(dir=ROOT.parent,prefix='integrity-negative-',ignore_cleanup_errors=True) as tmp:
  clone=Path(tmp)/'task';shutil.copytree(ROOT,clone)
  for target in ['runtime/observer.py','demo-actions.jsonl','runtime/calibration.json']:
   p=clone/target;original=p.read_bytes();p.chmod(0o644);p.write_bytes(original+b'\n')
   try:preflight(clone)
   except ValueError:pass
   else:raise AssertionError('Tamper accepted: '+target)
   p.write_bytes(original)
  # Updating general manifest cannot bypass the measurement calibration binding.
  p=clone/'runtime/mechanism.py';p.chmod(0o644);p.write_text(p.read_text()+'\n')
  m=json.loads((clone/'runtime-manifest.json').read_text());m['mechanism.py']=digest(p);(clone/'runtime-manifest.json').write_text(json.dumps(m))
  try:preflight(clone)
  except ValueError as e:assert 'Calibration source' in str(e)
  else:raise AssertionError('Source receipt bypass')
 checks.append('runtime/action/calibration tamper and calibration-manifest bypass rejected')
 task=json.loads((ROOT/'task.json').read_text());assert len(task['scores'])<=300 and len(task['fidelity'])<=500
 assert task['instruction']==json.loads((ROOT/'task.draft.json').read_text())['instruction']
 assert isinstance(task['media'],str) and task['media'].startswith('M-')
 proof=json.loads((ROOT/'evidence/final-brev-model-verification.json').read_text());assert proof['valid_completed_trial'] and proof['score']==0. and proof['sessions']==1
 checks.append('final task metadata, exact instruction, real media ID and completed model evidence')
 for filename in ('run_demo.py','capture_setup.py'):
  t=ast.parse((ROOT/filename).read_text());calls=[n.func.attr for n in ast.walk(t) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)]
  assert not any(n.startswith(('write_joint','write_root','set_joint','set_external')) for n in calls)
 checks.append('new replay/setup helpers contain no task-state setter calls')
 print(dumps(dict(passed=True,checks=checks,actions=len(rows),derived_physics_s=rows[-1]['status']['samples']*.002,scores_chars=len(task['scores']),fidelity_chars=len(task['fidelity']),simulation_starts=0,model_calls=0)))
if __name__=='__main__':main()
