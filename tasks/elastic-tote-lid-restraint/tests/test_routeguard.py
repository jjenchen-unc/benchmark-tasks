"""Regressions for solid coverage, sweeps, and permitted free/wrapped paths."""
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"d1r_arena"))
import geometry as geo
import strap
from routeguard import RouteGuard,segment_box_distance
gaps={"tote_tall":.02,"tote_low":.02}
def world(name,points):
    t=geo.tote(name)
    return np.asarray(points)@geo.rot_z(t["yaw"]).T+t["origin"]

# The original unused-tote gap: crossing a low-tote wall must fail.
t=geo.tote("tote_low")
guard=RouteGuard()
p=world("tote_low",[(t["h"][0]-.02,-.03,.04),(t["h"][0]+.02,.03,.04)])
assert guard.check(p,gaps)=="cord_through_solid"
assert guard.detail["tote"]=="tote_low"
# Each bracket is a real solid, independent of the tote hull.
for name in gaps:
    t=geo.tote(name)
    for part,c,h in t["brackets"]:
        c=np.asarray(c)
        p=world(name,[c+[-.01,0,0],c+[.01,0,0]])
        guard=RouteGuard()
        assert guard.check(p,gaps)=="cord_through_solid"
# Endpoint-only checks miss this substep sweep through a low-tote wall:
# both complete paths are exterior and clear, but their interpolation is not.
t=geo.tote("tote_low")
x=t["h"][0]
guard=RouteGuard()
a=world("tote_low",[(x-.02,.03,.04),(x-.02,.05,.04)])
b=world("tote_low",[(x+.02,.03,.04),(x+.02,.05,.04)])
assert guard.check(a,gaps) is None
assert RouteGuard().check(b,gaps) is None
assert guard.check(b,gaps)=="cord_through_solid"
# A translating lid sweeps through a stationary segment; endpoints are clear.
t=geo.tote("tote_low");z=t["lid_shut_z"]+.01
p=world("tote_low",[(-.03,0,z),(.03,0,z)])
guard=RouteGuard()
assert guard.check(p,dict(gaps,tote_low=.025)) is None
assert RouteGuard().check(p,dict(gaps,tote_low=0.)) is None
assert guard.check(p,dict(gaps,tote_low=0.))=="cord_through_solid"
# All nominal tray cords are clear and may remain stationary.
hooks={h["name"]:h for h in geo.hooks()}
for i in range(4):
    h=[hooks[f"strap_{i+1}_{e}"] for e in "ab"]
    p=[strap.hook_attach_world(x["pos"],x["quat_xyzw"]) for x in h]
    guard=RouteGuard()
    assert guard.check(p,gaps) is None
    assert guard.check(p,gaps) is None
# Legal wrapped top path must not be rejected merely for a route corner.
frame=strap.ToteFrame()
for gap in (0.,.002,.02):
    cord=strap.Cord(3,frame);cord.route="over_lid"
    p=[]
    for eye in (0,1):
        pos,q=strap.seated_hook_pose(frame,eye,gap)
        p.append(strap.hook_attach_world(pos,q))
    guard=RouteGuard()
    assert guard.check(cord.path(*p,gap),dict(gaps,tote_tall=gap)) is None,guard.detail
# Sliding at exact lid tangency has a swept halfspace certificate.
t=geo.tote("tote_low")
z=t["lid_shut_z"]+.02+t["lid_half"][2]+.004
guard=RouteGuard()
assert guard.check(world("tote_low",[(-.04,-.01,z),(.04,-.01,z)]),gaps) is None
assert guard.check(world("tote_low",[(-.04,.01,z),(.04,.01,z)]),gaps) is None
# Actual free-to-wrap transition while lowering opposite ends from above.
frame=strap.ToteFrame()
cord=strap.Cord(3,frame)
_,_,top=frame.box(.02)
for delta in np.linspace(.025,-.10,60):
    a=frame.world((-.23,0,top+delta));b=frame.world((.23,0,top+delta))
    cord.update_route(a,b,.02)
    assert cord.failure is None
    assert cord.route_guard.check(cord.path(a,b,.02),gaps) is None,cord.route_guard.detail
assert cord.route=="over_lid"
# Euclidean corner distance allows an 8mm cord tangent to a box corner.
assert abs(segment_box_distance(np.array([1.003,1.004,-.5]),np.array([1.003,1.004,.5]),np.zeros(3),np.eye(3),np.ones(3))-.005)<1e-10
print("route history and coverage regressions passed")
