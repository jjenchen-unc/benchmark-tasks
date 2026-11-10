"""Rigid20cm mount risers from frozen compact_yam_rear adapter.
Transforms arm bases and fixed top camera together; wrist cameras retain their
link-local calibrations. Payload geometry and robot dynamics are unchanged.
"""
from dataclasses import replace
RISE=.20
def up(p):
    return (p[0],p[1],p[2]+RISE)
def raised_rig(rig):
    if rig.name.endswith("-d1r-riser20"):return rig
    return replace(rig,name=rig.name+"-d1r-riser20",
        bases={side:up(p) for side,p in rig.bases.items()},
        top_cam=replace(rig.top_cam,pos=up(rig.top_cam.pos)))
