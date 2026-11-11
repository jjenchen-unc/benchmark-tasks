"""Strict finite numeric JSON boundary; unknown objects fail before serialization."""
import json,math
import numpy as np
def plain(x):
 if isinstance(x,np.ndarray):return plain(x.tolist())
 if isinstance(x,np.generic):return plain(x.item())
 if x is None or isinstance(x,(str,bool,int)):return x
 if isinstance(x,float):
  if not math.isfinite(x):raise ValueError('nonfinite journal value')
  return x
 if isinstance(x,(list,tuple)):return [plain(v) for v in x]
 if isinstance(x,dict):
  if not all(isinstance(k,str) for k in x):raise TypeError('JSON keys must be strings')
  return {k:plain(v) for k,v in x.items()}
 raise TypeError('unsupported journal value: '+type(x).__name__)
def dumps(x,**kw):return json.dumps(plain(x),allow_nan=False,**kw)
