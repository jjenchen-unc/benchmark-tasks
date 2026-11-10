"""Recompute diagnostic numeric history from saved per-physics-step measurements.
Does not enable runtime credit, load a scene, or transfer native scores.
"""
import argparse,json
from s5_rules import History
ap=argparse.ArgumentParser();ap.add_argument('trace');a=ap.parse_args();h=History();episodes=[]
with open(a.trace) as f:
 for line in f:
  r=json.loads(line)
  if r.get('event')=='begin':
   if h.samples:episodes.append(h.result())
   h=History()
  elif 'measurement' in r:h.add(r['measurement'],r['dt'])
if h.samples:episodes.append(h.result())
print(json.dumps(dict(episodes=episodes,credit_enabled=False,model_score=None),indent=2))
