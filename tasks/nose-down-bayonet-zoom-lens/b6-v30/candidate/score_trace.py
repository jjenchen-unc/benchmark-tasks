"""Recompute numeric history from measured substep records, no SDK or simulation.
Raw measured rubric is diagnostic until B6 runtime channels are calibrated.
"""
import argparse,json
from pathlib import Path
from mechanics import History,REQUIRED
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('trace',type=Path);a=p.parse_args();h=History();episodes=[];previous=None
 for line in a.trace.open():
  row=json.loads(line);t=row['sim_time']
  if previous is not None and row['control']<previous['control']:
   episodes.append(h.result(REQUIRED));h=History();previous=None
  dt=.002 if previous is None else t-previous['sim_time']
  h.update(row['row'],dt);previous=row
 episodes.append(h.result(REQUIRED));print(json.dumps({'evaluator_only_recomputed_episodes':episodes,'runtime_credit_activated':False},indent=2))
