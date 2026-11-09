"""D3 mobile-contact graph semantics for an actual backend contact observer.
Inputs must be a verified complete snapshot, not policy-reported contact flags.
No actual SDK stream or scoring validation is claimed by this pure module.
"""
from dataclasses import dataclass
import math
import numpy as np

def reachable(edges, sources):
    """Items reachable from `sources` over undirected contact `edges` (pairs). Pure helper, unit-tested."""
    adj = {}
    for a, b in edges:
        adj.setdefault(a, set()).add(b); adj.setdefault(b, set()).add(a)
    seen, todo = set(), list(sources)
    while todo:
        x = todo.pop()
        for y in adj.get(x, ()):
            if y not in seen:
                seen.add(y); todo.append(y)
    return seen - set(sources)

def per_arm_force(records, narms=2):
    """records: iterable of (arm, force_vector_world applied BY that arm TO the package). Returns |net| per arm (N).
    Vectors of different arms are never summed together (no cross-arm cancellation)."""
    net = np.zeros((narms, 3))
    for arm, f in records:
        net[int(arm)] += np.asarray(f, float)
    return [float(np.linalg.norm(v)) for v in net]

@dataclass(frozen=True)
class BodyRole:
 kind:str
 item:str|None=None
 arm:int|None=None

def frame_contacts(points, roles, dt):
 """points=(body0,body1,separation_m,world_impulse_vector) from the actual physics frame.
 Static furniture never bridges mobile items; grouped carton/lid/face share one item.
 Unknown physically touching bodies fail closed instead of silently dropping a possible brace.
 """
 if not math.isfinite(dt) or not 0<dt<=.002:raise ValueError('Current resolved contact dt required')
 edges=set();direct=set();records=[];active_pairs=set()
 for body0,body1,separation,impulse in points:
  impulse=np.asarray(impulse,dtype=float)
  if not math.isfinite(separation) or impulse.shape!=(3,) or not np.isfinite(impulse).all():raise ValueError('Incomplete/nonfinite contact point')
  if separation>0 and float(np.linalg.norm(impulse))<=1e-12:continue
  if body0 not in roles or body1 not in roles:raise ValueError('Unregistered touching body may carry indirect robot contact')
  ra,rb=roles[body0],roles[body1]
  for role in [ra,rb]:
   if role.kind not in ['robot','mobile','static']:raise ValueError('Unknown contact body role')
   if role.kind=='mobile' and not role.item:raise ValueError('Mobile body needs an owner item')
   if role.kind=='robot' and role.arm not in [0,1]:raise ValueError('Robot body needs its measured arm')
  active_pairs.add(tuple(sorted([body0,body1])))
  if ra.kind=='mobile' and rb.kind=='mobile' and ra.item!=rb.item:edges.add(tuple(sorted([ra.item,rb.item])))
  if ra.kind=='robot' and rb.kind=='mobile':direct.add(rb.item)
  if rb.kind=='robot' and ra.kind=='mobile':direct.add(ra.item)
 touched=direct|reachable(edges,direct)
 return {'direct':direct,'indirect':touched-direct,'touched':touched,'mobile_edges':edges,'active_pairs':active_pairs}
