import json,math
import numpy as np
def plain(x):
 if isinstance(x,np.ndarray):return plain(x.tolist())
 if isinstance(x,np.generic):return plain(x.item())
 if isinstance(x,dict):return {str(k):plain(v) for k,v in x.items()}
 if isinstance(x,(list,tuple)):return [plain(v) for v in x]
 if isinstance(x,float) and not math.isfinite(x):raise ValueError("nonfinite telemetry")
 if x is None or isinstance(x,(str,bool,int,float)):return x
 raise TypeError(type(x).__name__)
def dumps(x,**kw):
 kw.pop("default",None);kw["allow_nan"]=False
 return json.dumps(plain(x),**kw)
