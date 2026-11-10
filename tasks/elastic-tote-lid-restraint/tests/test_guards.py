# Regression cases for declared80N/0.1s abort, absent channels and actual contact.
from pathlib import Path
import sys,copy
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"d1r_arena"))
from guards import GuardState,HOOKS
from scoring import Scorer
DT=1/240
clear={n:False for n in (*HOOKS,"lid")}
g=GuardState()
for _ in range(23):g.sample({"left":81.,"right":1.},clear,DT)
assert g.failure is None
g.sample({"left":81.,"right":1.},clear,DT);assert g.failure=="wrist_force_abort"
g.sample({"left":1.,"right":1.},clear,DT);assert g.failure=="wrist_force_abort"
g=GuardState()
for _ in range(200):g.sample({"left":80.,"right":80.},clear,DT)
assert g.failure is None
g=GuardState()
for _ in range(23):g.sample({"left":81.,"right":1.},clear,DT)
g.sample({"left":0.,"right":1.},clear,DT)
for _ in range(23):g.sample({"left":81.,"right":1.},clear,DT)
assert g.failure is None
g=GuardState();g.sample(None,clear,DT);assert g.failure=="missing_wrist_measurement"
g=GuardState();g.sample({"left":float("nan"),"right":0.},clear,DT);assert g.failure=="missing_wrist_measurement"
g=GuardState();g.sample({"left":1.,"right":1.},None,DT);assert g.failure=="missing_contact_measurement"
g=GuardState();contact=dict(clear);contact["strap_4_a"]=True
g.sample({"left":1.,"right":1.},contact,DT);g.sample({"left":1.,"right":1.},clear,DT)
assert not g.hooks_off("strap_4_a")
g.finish_control();assert g.hooks_off("strap_4_a")
def good():
 return dict(wrist_force_measured=True,wrist_forces_n={"left":1.,"right":1.},
 robot_contacts_measured=True,robot_contacts=dict(clear),guard_failure=None,
 totes={"tote_tall":{"lid_gap_m":0.001}},hooks={},hands_off_lid=True,
 straps={"strap_4":dict(seated_a=0,seated_b=1,hands_off=True,route="over_lid",failure=None)})
s=Scorer()
for _ in range(400):s.update(good())
assert s.score==1.
for name in ("lid","strap_4_a","strap_2_b"):
 s=Scorer();r=good();r["robot_contacts"][name]=True
 for _ in range(400):s.update(copy.deepcopy(r))
 assert s.score==0.,(name,s.score)
s=Scorer();r=good();r.pop("wrist_forces_n");s.update(r);assert s.score==0. and s.failure
s=Scorer();r=good();r["guard_failure"]="wrist_force_abort";s.update(r)
for _ in range(500):s.update(good())
assert s.score==0. and s.failure=="wrist_force_abort"
print("guard regressions passed")
