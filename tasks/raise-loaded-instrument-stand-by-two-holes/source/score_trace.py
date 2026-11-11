"""Evaluator replay of recorded substep measurements, not backend verification."""
import argparse,json
from pathlib import Path
from contract import History
ap=argparse.ArgumentParser();ap.add_argument('trace',type=Path);ap.add_argument('--episode',type=int);a=ap.parse_args()
rows=[json.loads(x) for x in a.trace.read_text().splitlines() if x.strip()]
if not rows:raise ValueError('empty trace')
episode=max(r['episode'] for r in rows) if a.episode is None else a.episode
h=History();last=0.;count=0
for r in rows:
 if r['episode']!=episode:continue
 dt=float(r['measurement']['actual_dt'])
 if abs(r['time']-last-dt)>1e-8:raise ValueError('missing/nonmonotonic physical substep')
 last=float(r['time']);h.update(dt,r['measurement']);count+=1
if not count:raise ValueError('episode not found')
if count%25:raise ValueError('incomplete final20Hz control; review raw failure before any score')
h.controls=count//25
if h.controls>4572:raise ValueError('episode exceeds time budget')
print(json.dumps(dict(h.result(),episode=episode,samples=count,evidence='recomputation only; actual runtime channels remain gated'),indent=2))
