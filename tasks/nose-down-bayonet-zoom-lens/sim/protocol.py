"""Strict finite JSON loading."""
from pathlib import Path
import json,math
def loads(s):
 def pairs(xs):
  d={}
  for k,v in xs:
   if k in d:raise ValueError('duplicate JSON key')
   d[k]=v
  return d
 def ff(x):
  v=float(x)
  if not math.isfinite(v):raise ValueError('nonfinite float')
  return v
 return json.loads(s,object_pairs_hook=pairs,parse_float=ff,parse_constant=lambda x:(_ for _ in ()).throw(ValueError('nonfinite')))
def load(p):return loads(Path(p).read_text())
def number(v):
 if type(v) not in (int,float) or not math.isfinite(v):raise ValueError('finite numeric field required')
 return v
