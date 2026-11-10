"""Cord solid-history guard. Geometry matches the spawned tote boxes.
Exact segment/OBB distances and conservative swept interpolation are reused from
the frozen native D1 fixture; this validates the analytic cord, not a FEM rope.
Stationary tangency is allowed. Uncertified moving intervals fail closed.
"""
from __future__ import annotations
import numpy as np
import geometry as geo
GEOM_EPS=1e-8
CORD_RADIUS=.004
def segment_box_distance(P, Q, c, R, hs):
    """Exact segment/AABB Euclidean distance, after OBB frame transform."""
    p, v = R.T@(P-c), R.T@(Q-P)
    cuts = [0., 1.]
    for a in range(3):
        if abs(v[a]) > 1e-15:
            cuts += [float(t) for t in ((-hs[a]-p[a])/v[a], (hs[a]-p[a])/v[a]) if 0. < t < 1.]
    cuts = sorted(cuts)
    def f(t): return float(np.linalg.norm(np.maximum(abs(p+t*v)-hs, 0.)))
    best = min(map(f, cuts))
    for lo, hi in zip(cuts[:-1], cuts[1:]):
        mid=(lo+hi)/2; a=p+mid*v; active=abs(a)>hs
        vv=v[active]; pp=p[active]-np.sign(a[active])*hs[active]
        if vv@vv > 1e-20: best=min(best, f(np.clip(-(pp@vv)/(vv@vv),lo,hi)))
    return best

def continuous_clear(previous, current, clearance, depth=0):
    """Certify linearly swept segment clearance by a distance Lipschitz bound.

    Uncertified intervals fail closed. Tangency is allowed for stationary geometry;
    moving trajectories need positive clearance. This is a conservative surrogate
    between observed poses, not a claim that the physical rope follows that path.
    """
    mid=(previous+current)/2
    bound=float(np.max(np.linalg.norm(current-previous,axis=1)))/2
    c=clearance(mid)
    if c < -GEOM_EPS: return False
    if c+GEOM_EPS >= bound: return True
    if depth >= 12: return False
    return continuous_clear(previous,mid,clearance,depth+1) and continuous_clear(mid,current,clearance,depth+1)

def path_partition(points):
    # Fixed material correspondence surrogate: normalized arc length. The union
    # of old/new vertex fractions ensures no corner is omitted at a wrap change.
    lengths=np.linalg.norm(np.diff(points,axis=0),axis=1); total=float(sum(lengths))
    return np.r_[0.,np.cumsum(lengths)/total] if total>1e-12 else np.linspace(0.,1.,len(points))

def sample_path(points,knots,at):
    return np.array([np.interp(at,knots,points[:,j]) for j in range(3)]).T

class RouteGuard:
    def __init__(self):
        self.previous=None
        self.previous_gaps=None
        self.failure=None
        self.detail=None
        self.failure_sample=None
        self.specs={name:geo.tote(name) for name in ("tote_tall","tote_low")}

    def check(self, points, gaps):
        points=np.asarray(points,float)
        if (points.ndim!=2 or points.shape[1]!=3 or len(points)<2 or
            not np.isfinite(points).all() or
            set(gaps)!=set(self.specs) or not np.isfinite(list(gaps.values())).all()):
            self.failure=self.failure or "cord_geometry_invalid"
            return self.failure
        previous=points if self.previous is None else self.previous
        old_gaps=gaps if self.previous_gaps is None else self.previous_gaps
        pk,ck=path_partition(previous),path_partition(points)
        at=np.unique(np.r_[pk,ck])
        old,new=sample_path(previous,pk,at),sample_path(points,ck,at)
        for name,t in self.specs.items():
            R=geo.rot_z(t["yaw"])
            old_local=(old-t["origin"])@R
            new_local=(new-t["origin"])@R
            # These are exactly the five physical wall/bottom boxes, two
            # bracket boxes and translating lid collider used by kit.Tote.
            parts=[(*part,False) for part in t["walls"]+t["brackets"]]
            parts.append(("lid",(0.,0.,t["lid_shut_z"]),t["lid_half"],True))
            for part,centre,half,moving in parts:
                centre=np.asarray(centre,float);half=np.asarray(half,float)
                c0=centre+np.array([0.,0.,old_gaps[name] if moving else 0.])
                c1=centre+np.array([0.,0.,gaps[name] if moving else 0.])
                a,b=old_local-c0,new_local-c1
                for i in range(len(at)-1):
                    aa,bb=a[i:i+2],b[i:i+2]
                    # Swept AABB clearance lower bound avoids exact work for
                    # distant solids, including a swept hull tangent to a box face.
                    # Unlike a scalar Lipschitz bound this certifies sliding tangency.
                    hull=np.vstack((aa,bb))
                    lo,hi=hull.min(axis=0),hull.max(axis=0)
                    separation=np.maximum(np.maximum(lo-half,-half-hi),0.)
                    if np.linalg.norm(separation)>=CORD_RADIUS-GEOM_EPS:
                        continue
                    def clearance(segment):
                        return segment_box_distance(segment[0],segment[1],np.zeros(3),np.eye(3),half)-CORD_RADIUS
                    if not continuous_clear(aa,bb,clearance):
                        if self.failure_sample is None:
                            self.failure_sample=dict(previous_path=previous.tolist(),current_path=points.tolist(),
                                previous_gaps=dict(old_gaps),current_gaps=dict(gaps),
                                segment_previous_local=aa.tolist(),segment_current_local=bb.tolist(),
                                box_half=half.tolist(),box_previous_centre=c0.tolist(),
                                box_current_centre=c1.tolist(),previous_clearance_m=clearance(aa),
                                current_clearance_m=clearance(bb))
                        self.failure=self.failure or "cord_through_solid"
                        self.detail=self.detail or dict(tote=name,part=part,segment=i)
                        break
                if self.failure:break
            if self.failure:break
        self.previous=points.copy()
        self.previous_gaps=dict(gaps)
        return self.failure
