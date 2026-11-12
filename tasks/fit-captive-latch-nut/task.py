"""A steel latch is fixed on the table: a swing arm with a black knob, clamped shut by an unmarked hex nut with a
yellow paint mark. A rack beside it holds five open-end wrenches (12 mm, 1/2 in, 13 mm, 14 mm and an adjustable
one set to 13.55 mm). The clamp lets the arm go once the nut has been turned more than 60 degrees; a wrench that
does not fit closely rounds the nut's corners under load, and then the nut only spins and the clamp never lets go.
"""
import sys, math
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from kit import WrenchPart, F, install, current_score
install()
import numpy as np

SCENE = 'station'
R = np.asarray(F['rotation']); T = np.asarray(F['translation'])
OBJECTS = [WrenchPart(name='wrench_fixture', pos=tuple(T), yaw=math.pi, size=(.4, .6, .2), mass=None, articulation=True)]
for b in F['bodies']:
    if b['free']:
        mass = sum(float(g.get('mass', 0)) for g in b['geoms'])
        OBJECTS.append(WrenchPart(name=b['name'], pos=tuple(R @ b['pos'] + T), yaw=math.pi + 2 * math.atan2(b['quat'][3], b['quat'][0]), size=(.22, .04, .007), mass=mass))
SCORE_OVER = 'end'


def score(objects):
    return current_score()
