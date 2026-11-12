"""Read-only diagnostic snapshots. No control, reset, close or mechanism writes."""
import json,os,traceback
from pathlib import Path
import numpy as np

def numeric(value):
 if isinstance(value,np.ndarray):return numeric(value.tolist())
 if isinstance(value,np.generic):return numeric(value.item())
 if value is None or isinstance(value,(str,bool,int,float)):return value
 if isinstance(value,(tuple,list)):return [numeric(v) for v in value]
 if isinstance(value,dict):
  if not all(isinstance(k,str) for k in value):raise TypeError('JSON keys must be strings')
  return {k:numeric(v) for k,v in value.items()}
 raise TypeError('Unsupported telemetry type: '+type(value).__name__)

def strict_json(value):return json.dumps(numeric(value),indent=2,allow_nan=False)

def write_snapshot(path,E,emb,q,*,phase,control,result,error=None):
 from kit import arr
 row={'phase':phase,'control':control,'command':q,'terminated':None if result is None else result.terminated,'termination_reason':None if result is None else result.termination_reason,'info':None if result is None else result.info,'status':E.status(),'measurement_error':E.error,'substeps':getattr(emb,'_first_cause_substeps',None),'history_samples':E.history.samples,'last_row':E.history.last,'engine_done':E.done}
 # Actual robot buffers and caps, read before any later action/reset/close.
 row['raw_joints']={s:arr(emb._base.scene[s+'_robot'].data.joint_pos) for s in ['left','right']}
 row['raw_joint_caps']={s:arr(emb._base.scene[s+'_robot'].root_view.get_dof_max_forces()) for s in ['left','right']}
 if error is not None:row['exception']={'type':type(error).__name__,'message':str(error),'traceback':traceback.format_exc()}
 if phase=='pre_action':
  bodies,joints,velocities=E.states()
  row['actual_numpy_row_preflight']={'joints':joints,'velocities':velocities,'body_positions':{n:p for n,(p,R) in bodies.items()},'body_rotations':{n:R for n,(p,R) in bodies.items()}}
 from settling_diagnostic import snapshot
 row['instruments_supports']=snapshot(E)
 text=strict_json(row)+'\n'
 path=Path(path)
 with path.open('x') as f:f.write(text);f.flush();os.fsync(f.fileno())
 return row

def count_substeps(emb):
 # Count actual existing scene-update callbacks; delegate unchanged, no physics writes.
 emb._first_cause_substeps=0
 original=getattr(emb,'_first_cause_original_update',emb._base.scene.update)
 emb._first_cause_original_update=original
 def update(*args,**kwargs):
  emb._first_cause_substeps+=1
  return original(*args,**kwargs)
 emb._base.scene.update=update
