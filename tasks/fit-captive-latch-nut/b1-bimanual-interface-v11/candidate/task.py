"""Fit a rack wrench to the unmarked captive latch nut, loosen it at least 60
degrees without rounding, open the swing arm at least 85 degrees (90 degree stop),
return tools and leave both hands and all tools off the nut and arm for 20 seconds.

Rack order in source front-to-back: 12 mm, 1/2 in, 13 mm, 14 mm, preset adjustable.
Left arm works; right stays parked. Limits: total27/7Nm including gravity,40N per
finger. Wrist80N for100ms abort; default30N/20ms and nonfinger50N/20ms safety
guards end the port episode. This is a development port with unverified channels.
"""
import sys,math
from pathlib import Path
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
from kit import WrenchPart,F,install,current_score
install()
import numpy as np
SCENE='station'
R=np.asarray(F['rotation']);T=np.asarray(F['translation'])
OBJECTS=[WrenchPart(name='wrench_fixture',pos=tuple(T),yaw=math.pi,size=(.4,.6,.2),mass=None,articulation=True)]
for b in F['bodies']:
    if b['free']:
        mass=sum(float(g.get('mass',0)) for g in b['geoms'])
        OBJECTS.append(WrenchPart(name=b['name'],pos=tuple(R@b['pos']+T),yaw=math.pi+2*math.atan2(b['quat'][3],b['quat'][0]),size=(.22,.04,.007),mass=mass))
SCORE_OVER='end'
def score(objects):return current_score()
