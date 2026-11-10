from pathlib import Path
import sys
from dataclasses import dataclass
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"d1r_arena"))
from rigging import raised_rig
@dataclass(frozen=True)
class Camera:pos:tuple
@dataclass(frozen=True)
class Rig:
 name:str
 bases:dict
 top_cam:Camera
 wrist_cams:dict
r=Rig("station",{"left":(0,.305,.75),"right":(0,-.305,.75)},Camera((.1,0,1.5)),{"left":Camera((0,.08,.07)),"right":Camera((0,.08,.07))})
q=raised_rig(r)
assert q.bases=={"left":(0,.305,.95),"right":(0,-.305,.95)}
assert q.top_cam.pos==(.1,0,1.7)
assert q.wrist_cams is r.wrist_cams
assert raised_rig(q) is q
assert r.bases["left"][2]==.75
print("rig20cm rigid translation, wrist-local calibration and idempotence passed")
