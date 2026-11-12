"""Strict journal boundary: preserve numeric types, reject unknowns/nonfinite."""
import json
import numpy as np
def convert(v):
 if isinstance(v,np.ndarray):return convert(v.tolist())
 if isinstance(v,np.generic):return convert(v.item())
 if v is None or isinstance(v,(str,bool,int,float)):return v
 if isinstance(v,dict):
  if not all(isinstance(k,str) for k in v):raise TypeError('non-string journal key')
  return {k:convert(x) for k,x in v.items()}
 if isinstance(v,(tuple,list)):return [convert(x) for x in v]
 raise TypeError('unsupported journal type '+type(v).__name__)
def dumps(v,**kw):return json.dumps(convert(v),allow_nan=False,**kw)
def preflight(emb,obs,out):
 import kit
 v=dict(status=kit.current_status(),observer_last=kit._OBSERVER.last,observer_rows=kit._OBSERVER.rows,joints=obs.state['joint_pos'])
 data=dumps(v);decoded=json.loads(data)
 assert np.asarray(decoded['joints']).shape==(14,)
 if kit._OBSERVER.error:raise RuntimeError(kit._OBSERVER.error)
 (out/'first-action-numeric-preflight.json').write_text(data+'\n')
