"""Task-owned hinge constraint property only; no state/default/effort writes."""
import numpy as np

def write_limits(asset, limits):
 import warp as wp
 from kit import arr
 view=asset.root_view
 before=view.get_dof_limits()
 expected=arr(before)
 value=np.asarray(limits,dtype=expected.dtype)
 if value.shape!=expected.shape or value.ndim!=3 or value.shape[0]!=1 or value.shape[2]!=2:raise RuntimeError('Unexpected hinge limit shape')
 if not np.isfinite(value).all() or np.any(value[:,:,0]>value[:,:,1]):raise RuntimeError('Invalid hinge limits')
 # Tensor getter uses scalar float32 (count,max_dofs,2); Lab also passes vec2f.
 # bypassing its documented mutation of default joint positions.
 if not ((before.dtype==wp.float32 and before.shape==value.shape) or (before.dtype==wp.vec2f and before.shape==value.shape[:2])):raise RuntimeError('Unsupported actual Warp limit layout: '+repr((before.dtype,before.shape)))
 defaults=arr(asset.data.default_joint_pos).copy()
 def record(phase):
  import os,json
  from pathlib import Path
  trace=os.environ.get("B6_TRACE")
  if trace:
   row=dict(phase=phase,dtype=repr(before.dtype),shape=list(before.shape),strides=list(before.strides),device=str(before.device),stored_limits_before=expected.tolist(),requested_limits=value.tolist(),native_default_joint_pos=defaults.tolist(),readback_limits=arr(view.get_dof_limits()).tolist(),default_joint_pos=arr(asset.data.default_joint_pos).tolist())
   with (Path(trace).parent/"limit-api.jsonl").open("a") as f:f.write(json.dumps(row,allow_nan=False)+"\n")
 record("before_write")
 backend=wp.array(value,dtype=before.dtype,device='cpu')
 indices=wp.array([0],dtype=wp.int32,device='cpu')
 view.set_dof_limits(backend,indices=indices)
 if not np.array_equal(arr(view.get_dof_limits()),value):raise RuntimeError('Backend limits did not roundtrip')
 if not np.array_equal(arr(asset.data.default_joint_pos),defaults):raise RuntimeError('Backend limit write changed native defaults')
 record("after_verified_write")
