"""Strict primitive JSON boundary: preserve numeric ownership; reject unknowns/NaNs."""
import json,math

def primitive(x):
    if x is None or type(x) in (str,bool,int):return x
    if type(x) is float:
        if not math.isfinite(x):raise ValueError('nonfinite JSON number')
        return x
    if isinstance(x,dict):
        if any(type(k) is not str for k in x):raise TypeError('JSON key must be str')
        return {k:primitive(v) for k,v in x.items()}
    if isinstance(x,(tuple,list)):return [primitive(v) for v in x]
    if type(x).__module__.split('.')[0]=='numpy':
        return primitive(x.tolist() if hasattr(x,'tolist') else x.item())
    raise TypeError('unsupported JSON type: '+type(x).__name__)
def dumps(x,**kw):return json.dumps(primitive(x),allow_nan=False,**kw)
