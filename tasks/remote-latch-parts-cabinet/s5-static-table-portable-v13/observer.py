"""Read-only strict S5 measured journal, using preserved native telemetry."""
import json
from telemetry import snapshot,dumps,numeric

def observe(emb,obs,phase,action=None):
 import s5_kit
 row=snapshot(emb,obs);E=s5_kit.ENGINE
 row.update(kind='ACTUAL_RUNTIME_OBSERVER',phase=phase,action=action,physics_tick=int(E.base.sim.get_physics_step_count()),physics_dt=float(E.base.sim.get_physics_dt()),control_dt=float(E.base.step_dt))
 if row['status']['measurement_error'] is not None:raise RuntimeError(row['status']['measurement_error'])
 if abs(row['physics_dt']-.002)>1e-9 or abs(row['control_dt']-.05)>1e-9:raise RuntimeError('native clock mismatch')
 for side,r in row['robots'].items():
  if r['tcp_fk_measured_error_m']>.001:raise RuntimeError('actual FK mismatch '+side)
 return json.loads(dumps(row))
def emit(path,row):
 line=dumps(row)
 with path.open('a') as f:f.write(line+'\n');f.flush()
 return json.loads(line)
