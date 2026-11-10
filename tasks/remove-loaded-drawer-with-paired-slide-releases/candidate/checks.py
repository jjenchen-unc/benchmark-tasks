"""Pure scorer/safety counterexamples; never physics or runtime certification."""
from pathlib import Path
import json,math,copy,numpy as np
from drawers_math import History,contents,extent,world,native,A
checks={}
def row(x=-.03,maxx=.11):return dict(drawer_p=np.array([x,-.17,.835]),drawer_R=np.eye(3),max_x=maxx,contents=True,drawer_touch=False,wrist=[0.,0.],jar_force=[2.55,2.55],jar_speed=[0.,0.],jar_cos=[1.,1.],drop=False)
def advance(r,n=240):
 h=History()
 for _ in range(n):h.update(r,1/240)
 return h
r=row(-.01,.13);h=advance(r);checks['runner_tail_inside_rejects_every_tier']=h.result(True)['score']==0 and h.hold==0
h=advance(row());checks['full_clear_hands_off_exact1s']=h.result(True)['score']==1
checks['unvalidated_credit_zero_despite_candidate_fullscore']=h.result(False)['score']==0 and h.result(False)['candidate_score']==1
r=row();r['drawer_p'][2]=.90;checks['airborne_clear_partial025']=advance(r).result(True)['score']==.25
r=row();r['drawer_R']=np.array([[1,0,0],[0,math.cos(.07),-math.sin(.07)],[0,math.sin(.07),math.cos(.07)]]);checks['table_tilt4deg_partial05']=advance(r).result(True)['score']==.5
h=advance(row(),180);r=row();r['drawer_touch']=True;h.update(r,1/240);h.update(row(),1/240);checks['contact_interrupts_finalhold']=h.hold<.01
for name,key,val in [('jar_force','jar_force',[15.01,0]),('jar_speed','jar_speed',[.301,0]),('second_jar_tilt','jar_cos',[1.,.8]),('drop','drop',True),('contents_removed','contents',False)]:
 r=row();r[key]=val;h=advance(r);checks[name+'_no_credit']=h.result(True)['score']==0
h=History();r=row();r['wrist']=[80.01,0]
for _ in range(23):h.update(r,1/240)
checks['wrist_95ms_not_abort']=h.failure is None
h.update(r,1/240);checks['wrist100ms_abort']=h.failure=='wrist_80N_100ms' and h.result(True)['score']==0
h.update(row(),1/240);checks['wrist_abort_sticky']=h.result(True)['score']==0
h=advance(row());h.__init__();checks['reset_history_zero']=h.hold==0 and h.samples==0 and h.result(True)['score']==0
for p in [[.2,-.17,.86],[-.03,-.1,.835]]:checks['rigid_transform_roundtrip_'+str(p)]=bool(np.allclose(native(world(p)),p))
# Oblique runner tail must use oriented primitive support, not centre+drawer halfdepth.
R=np.array([[0.,-1.,0.],[1.,0.,0.],[0.,0.,1.]])
checks['rotated_box_support']=np.allclose(extent('box',np.array([.1,.13,.035]),R),[.13,.1,.035])
print(json.dumps({'scope':'pure numeric contract only; no Arena physics proof','checks':checks,'passed':all(checks.values())},indent=2))
assert all(checks.values())
