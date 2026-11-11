"""Inspect task entry; requires ROOT's fresh actual full-goal/channel review."""
from pathlib import Path
import sys,os,runpy
O=Path(__file__).resolve().parent
sys.path.insert(0,str(O));sys.path.insert(0,str(O/'candidate'))
from protocol import receipt,CHANNELS
REVIEW=receipt(os.environ.get('S2_ROOT_REVIEW',''))
native=runpy.run_path(str(O/'candidate/task.py'))
__doc__=native['__doc__']
for n in ('SCENE','OBJECTS','SCORE_OVER','score'):globals()[n]=native[n]
import physics
if physics.REQUIRED!=CHANNELS:raise RuntimeError('channel contract')
physics.VALIDATED_CHANNELS=frozenset(CHANNELS)
