"""Task-local strict numeric telemetry serialization; no simulator imports."""
import json
import numpy as np

def numeric(value):
 if isinstance(value,np.ndarray):return numeric(value.tolist())
 if isinstance(value,np.generic):return numeric(value.item())
 if value is None or isinstance(value,(str,bool,int,float)):return value
 if isinstance(value,(list,tuple)):return [numeric(x) for x in value]
 if isinstance(value,dict):
  if not all(isinstance(k,str) for k in value):raise TypeError('Telemetry keys must be strings')
  return {k:numeric(v) for k,v in value.items()}
 raise TypeError('Unsupported telemetry type: '+type(value).__name__)

def dumps(value,**kwargs):
 if 'allow_nan' in kwargs:raise TypeError('allow_nan is fixed false')
 return json.dumps(numeric(value),allow_nan=False,**kwargs)
