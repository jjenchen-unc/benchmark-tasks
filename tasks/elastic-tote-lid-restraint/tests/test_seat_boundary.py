from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"d1r_arena"))
import strap
import geometry as geo
from routeguard import segment_box_distance
f=strap.ToteFrame()
p,q=strap.seated_hook_pose(f,0,0.)
R=strap.quat_to_R(q);eye=f.world(f.eyes[0])
# A valid centred bar and a small endpoint overlap remain credited.
for shift in (0.,.010,.020):
 assert strap.seat_check(p+shift*f.bar_axis_world,q,0.,f)==0
# Counterexample: closest finite bar endpoint is >5mm from all hook surfaces.
shift=.025;hp=p+shift*f.bar_axis_world
a=R.T@(eye-geo.BAR_HALF*f.bar_axis_world-hp)
b=R.T@(eye+geo.BAR_HALF*f.bar_axis_world-hp)
gap=min(segment_box_distance(a,b,np.array(c),np.eye(3),np.array(h))-geo.BAR_RADIUS
    for c,h in [((0,0,0),geo.HANDLE),(geo.SHANK_POS,geo.SHANK_HALF),(strap.FOOT_POS,strap.FOOT_HALF)])
assert gap>.005
assert strap.seat_check(hp,q,0.,f) is None
assert strap.seat_check(p+.035*f.bar_axis_world,q,0.,f) is None
print("finite bar endpoint overlap: true seats kept and separated false seat rejected")
