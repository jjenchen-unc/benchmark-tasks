"""Native task exported through an explicit source-bound ROOT review gate."""
from pathlib import Path
import sys,os,runpy
O=Path(__file__).resolve().parent;sys.path.insert(0,str(O));sys.path.insert(0,str(O/'candidate'))
from protocol import receipt,CHANNELS
review=receipt(os.environ.get('HYDRAULIC_ROOT_REVIEW',''))
native=runpy.run_path(str(O/'candidate/task.py'));__doc__=native['__doc__']
for name in ('SCENE','OBJECTS','SCORE_OVER','score'):globals()[name]=native[name]
import physics
if not getattr(physics.Physics,'_root_review_adapter',False):
 original=physics.Physics.status
 def status(self):
  r=original(self)
  if r['missing_channels']!=list(CHANNELS):raise RuntimeError('native gate changed')
  actual=self.history.result(frozenset(CHANNELS));r.update(actual);return r
 physics.Physics.status=status;physics.Physics._root_review_adapter=True
