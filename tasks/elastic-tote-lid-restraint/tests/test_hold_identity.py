# Pending identity-hold regression; run after applying next-scoring.py.
from pathlib import Path
import sys,copy
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"d1r_arena"))
from scoring import Scorer
names=("lid",*tuple(f"strap_{i}_{e}" for i in range(1,5) for e in ("a","b")))
def row(name,gap,both):
 return dict(wrist_force_measured=True,wrist_forces_n={"left":1.,"right":1.},
 robot_contacts_measured=True,robot_contacts={n:False for n in names},
 guard_failure=None,totes={"tote_tall":{"lid_gap_m":gap}},hooks={},hands_off_lid=True,
 straps={name:dict(seated_a=0,seated_b=1 if both else None,hands_off=True,route="over_lid" if both else "free",failure=None)})
for n,gap,both,score in [(400,.001,True,1.),(100,.005,True,.5),(100,.020,False,.25)]:
 s=Scorer()
 for _ in range(n-1):s.update(row("strap_1",gap,both))
 s.update(row("strap_2",gap,both))
 assert s.score==0.,(n,s.score)
 for _ in range(n-1):s.update(row("strap_2",gap,both))
 assert s.score==score,(n,s.score)
print("strap identity hold regressions passed")
