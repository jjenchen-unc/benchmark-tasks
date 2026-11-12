"""A grey steel equipment enclosure is fixed on the table in front of the arms. Its lid lies on a black rubber
gasket and has an adjustable compression latch: a black wing handle on a round plate, and a brass knob with a
white mark that sets the latch's grip length. The latch is unlocked and the lid is not sealed.
"""
import math, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import latch
from latch import Latch, Box
latch.install()

SCENE = 'station'
OBJECTS = [Latch(name='compression_latch', pos=latch.POS, size=(.324, .40, .21))]
from yam_arena.station import BASES
for side, base in BASES.items():
    top = base[2] + latch.BASE_RISE + latch.BASE_MESH_BOTTOM
    OBJECTS.append(Box(name=side + '_robot_pedestal', pos=(base[0], base[1], (.7475 + top) / 2), size=(.24, .12, top - .7475), color=(.35, .35, .37)))

# Everything below uses only what a judge can measure: handle angle (protractor), knob turns from the start
# mark (counted on the video), lid-to-flange gap (feeler gauge) and whether a robot touches the enclosure.
LOCKED = math.radians(175)       # handle at its locked stop
OVER_CENTRE = math.radians(172)  # the cam passes over centre here
KNOB_LIMIT = 2.0                 # turns from the start mark; locking past over-centre with more is forcing
GAPS = (.001, .002, .0055)       # gap limits for 1.0, 0.5 and 0.25
HOLD = 10000                     # 20 s of 2 ms physics steps
SUCCESS_HOLD_S = 0
SCORE_OVER = 'end'
_S = {}


def _update():
    samples = latch.EPISODE['samples']
    if _S.get('samples') is not samples:
        _S.update(samples=samples, i=0, run=[0, 0, 0], forced=False, unreadable=False)
    for rec in samples[_S['i']:]:
        if rec is None:
            _S['unreadable'] = True; _S['run'] = [0, 0, 0]; continue
        handle, knob, gap, touch = rec
        if handle >= OVER_CENTRE and knob / (2 * math.pi) > KNOB_LIMIT: _S['forced'] = True
        ok = handle >= LOCKED and not touch
        _S['run'] = [n + 1 if ok and gap < g else 0 for n, g in zip(_S['run'], GAPS)]
    _S['i'] = len(samples)


def score(objects):
    _update()
    if not _S['i'] or _S['unreadable'] or _S['forced']: return 0.
    for n, s in zip(_S['run'], (1., .5, .25)):
        if n >= HOLD: return s
    return 0.


def success(objects):
    return score(objects) == 1.
