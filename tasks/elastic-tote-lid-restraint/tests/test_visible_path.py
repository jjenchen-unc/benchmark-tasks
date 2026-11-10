from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"d1r_arena"))
import strap
import geometry as geo
from routeguard import RouteGuard
f=strap.ToteFrame()
gap=.02
_,_,top=f.box(gap)
# Side endpoints match the asymmetry from actual43311, with different lateral y.
ax,bx=-.276,.223
ay,by=-.003,-.023
bz=top+.060
# At the near4mm corner, a straight segment just touches the inflated box.
cx=-f.hx-.004
fraction=(cx-ax)/(bx-ax)
az=(top+.004-fraction*bz)/(1-fraction)
cord=strap.Cord(3,f)
guard=RouteGuard();lengths=[];counts=[]
for dz in np.linspace(.003,-.003,121):
 a=f.world((ax,ay,az+dz));b=f.world((bx,by,bz))
 cord.update_route(a,b,gap)
 assert cord.failure is None
 pts=cord.path(a,b,gap)
 assert guard.check(pts,{"tote_tall":gap,"tote_low":gap}) is None,guard.detail
 length=sum(float(np.linalg.norm(q-p)) for p,q in zip(pts[:-1],pts[1:]))
 lengths.append(length);counts.append(len(pts))
 if len(pts)>2:
  # Above far endpoint sees the near edge; forcing the far edge is a detour.
  assert len(pts)==3
  assert np.allclose(pts[1],pts[0]+np.linalg.norm(pts[1]-pts[0])/max(length,1e-12)*(pts[-1]-pts[0]),atol=.003)
assert 2 in counts and 3 in counts
assert max(abs(np.diff(lengths)))<.00010,max(abs(np.diff(lengths)))
# The contact-activation limit is continuous even with unequal lateral positions.
def forced(dz):
 c=strap.Cord(3,f);c.route="over_lid"
 pts=c.path(f.world((ax,ay,az+dz)),f.world((bx,by,bz)),gap)
 return sum(float(np.linalg.norm(q-p)) for p,q in zip(pts[:-1],pts[1:]))
assert abs(forced(1e-7)-forced(-1e-7))<3e-7
# Both ends below the lid require both corners; static legacy load unchanged.
c=strap.Cord(3,f);c.route="over_lid"
poses=[strap.seated_hook_pose(f,i,0.) for i in (0,1)]
a,b=[strap.hook_attach_world(p,q) for p,q in poses]
pts=c.path(a,b,0.)
assert len(pts)==4
T,fa,fb,lid=c.forces(a,b,0.,1/240)
assert np.linalg.norm(fa+fb+lid)<1e-10
# One-corner force conservation and genuine corner force on the lid.
c=strap.Cord(3,f);c.route="over_lid"
a=f.world((ax,ay,az-.005));b=f.world((bx,by,bz))
assert len(c.path(a,b,gap))==3
T,fa,fb,lid=c.forces(a,b,gap,1/240)
assert T>0 and np.linalg.norm(lid)>0
assert np.linalg.norm(fa+fb+lid)<1e-10
# A side/below entry remains a permanent failure; selecting a path cannot clear it.
c=strap.Cord(3,f)
a=f.world((-.23,0,top-.10));b=f.world((.23,0,top-.10))
c.update_route(a,b,gap)
assert c.failure=="cord_through_solid"
c.update_route(f.world((-.23,0,top+.10)),f.world((.23,0,top+.10)),gap)
assert c.failure=="cord_through_solid"
print("visible corner continuity, swept history, both-sided baseline and force balance passed")

# One endpoint may move above the lid into its footprint without crossing a solid.
# Establish the route from a continuously clear top approach, in both directions.
for swap in (False,True):
 c=strap.Cord(3,f);guard=RouteGuard()
 for height in np.linspace(top+.050,top-.150,161):
  a=f.world((.230,0,height));b=f.world((-.100,0,top+.050))
  if swap:a,b=b,a
  c.update_route(a,b,gap)
  assert c.failure is None,(swap,height,c.failure)
  assert guard.check(c.path(a,b,gap),{"tote_tall":gap,"tote_low":gap}) is None,guard.detail
 assert c.route=="over_lid"
 # Slide the upper endpoint out across the edge, and back into the footprint.
 for bx in list(np.linspace(-.100,-.230,81))+list(np.linspace(-.230,-.100,81)):
  a=f.world((.230,0,top-.150));b=f.world((bx,0,top+.050))
  if swap:a,b=b,a
  c.update_route(a,b,gap)
  assert c.failure is None
  assert guard.check(c.path(a,b,gap),{"tote_tall":gap,"tote_low":gap}) is None,guard.detail
 # Endpoint-below-solid remains invalid, including after a legitimate wrap.
 a=f.world((.230,0,top-.150));b=f.world((-.100,0,top-.050))
 if swap:a,b=b,a
 c.update_route(a,b,gap)
 assert c.failure=="cord_through_solid"
# An upper endpoint cannot legalize an initial side entry through the tote.
c=strap.Cord(3,f)
c.update_route(f.world((.230,0,top-.150)),f.world((-.100,0,top+.050)),gap)
assert c.failure=="cord_through_solid"
print("above-lid endpoint transitions: legal swept paths kept; initial/solid entries rejected")
