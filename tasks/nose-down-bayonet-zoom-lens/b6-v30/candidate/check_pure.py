"""Pure adversarial numeric/history/passivity checks; no simulation execution."""
import copy,json,math
from pathlib import Path
import numpy as np
from numeric_score import score
from mechanics import History,REQUIRED,helix,H

def run():
 checks={}
 m=dict(lens_on_mount='lens_blue',axial_gap_m=.0006,radial_m=.0,tilt_deg=0.,family_ok=True,captured=True,pin_engaged=True,lock_angle_err_deg=.8,focal_mm=50.)
 h=dict(overload=False,dropped=False,invalid_measurement_seen=False,hands_off_end_s=20.,hold_full_s=20.,hold_locked_s=20.)
 checks['full_numeric']=score(m,h)['score']==1.
 for f in ['radial_m','tilt_deg','lock_angle_err_deg']:
  a=dict(m);a[f]=-1.;checks['negative_'+f]=score(a,h)['score']==0.
 for f in m:
  a=dict(m);del a[f];checks['missing_'+f]=score(a,h)['score']==0.
 a=dict(m);a['focal_mm']=55.;checks['partial_retained']=score(a,h)['score']==.5
 a=dict(m);a['pin_engaged']=False;checks['unlocked_zero']=score(a,h)['score']==0.
 a=dict(m);a['family_ok']=False;checks['wrong_family_zero']=score(a,h)['score']==0.
 a=dict(m);a['focal_mm']=float('nan');checks['nan_zero']=score(a,h)['score']==0.
 for f in ['overload','dropped','invalid_measurement_seen']:
  a=dict(h);a[f]=True;checks['veto_'+f]=score(m,a)['score']==0.
 row=dict(wrist_n=[0.,0.],stop_nm={'body_right':0.},drop_velocity_z={'lens_blue':0.},held={'lens_blue':False},captured={'lens_blue':True},robot_touch=False,measurement=m)
 hist=History()
 for _ in range(10000):hist.update(row,.002)
 checks['20s_continuous']=hist.result(REQUIRED)['score']==1.;checks['unvalidated_runtime_zero']=hist.result()['score']==0.
 touch=copy.deepcopy(row);touch['robot_touch']=True;hist.update(touch,.002);checks['contact_resets_hold']=hist.result(REQUIRED)['score']==0. and hist.hands==0.
 hist=History();over=copy.deepcopy(row);over['stop_nm']['body_right']=1.5001
 for _ in range(24):hist.update(over,.002)
 checks['under50ms_not_overloaded']=not hist.overload
 hist.update(over,.002);checks['50ms_damage']=hist.overload
 hist.update(row,.002);checks['damage_sticky']=hist.overload
 hist=History();force=copy.deepcopy(row);force['wrist_n']=[80.0001,0]
 for _ in range(49):hist.update(force,.002)
 checks['under100ms_wrist']=hist.failure is None
 hist.update(force,.002);checks['100ms_wrist_abort']=hist.failure=='wrist_force_80N_100ms'
 hist=History();drop=copy.deepcopy(row);drop['drop_velocity_z']['lens_blue']=-.6001;drop['captured']['lens_blue']=False;hist.update(drop,.002);checks['free_drop']=hist.dropped
 hist=History();drop['held']['lens_blue']=True;hist.update(drop,.002);checks['held_not_drop']=not hist.dropped
 hist=History();invalid=copy.deepcopy(row);invalid['wrist_n']=[float('nan'),0];hist.update(invalid,.002);checks['invalid_sensor_fails_closed']=hist.invalid
 # Mechanical virtual work: P+Udot=-C*edot^2, never energy creation.
 rng=np.random.default_rng(5);errors=[]
 for _ in range(200):
  th,x,w,v=rng.normal(size=4);F,T=helix(th,x,w,v);udot=20000*(x-H*th)*(v-H*w);errors.append(abs(F*v+T*w+udot+20*(v-H*w)**2))
 checks['helix_balanced_passive_power']=max(errors)<1e-8
 hist=History()
 for _ in range(10000):hist.update(row,.002)
 for _ in range(50):hist.update(force,.002)
 checks['wrist_veto_even_after_full_hold']=hist.result(REQUIRED)['score']==0. and hist.failure=='wrist_force_80N_100ms'
 checks={k:bool(v) for k,v in checks.items()}
 return dict(state='pass' if all(checks.values()) else 'failed',checks=checks,physics_steps=0,max_passive_power_residual=max(errors),runtime_channels_activated=[])
if __name__=='__main__':
 import argparse
 p=argparse.ArgumentParser();p.add_argument('--out',type=Path);a=p.parse_args();r=run();print(json.dumps(r,indent=2))
 if a.out:a.out.write_text(json.dumps(r,indent=2)+'\n')
 if r['state']!='pass':raise SystemExit(2)
