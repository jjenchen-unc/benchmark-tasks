"""Pure boundary and adversarial-cancellation checks; no physics."""
import json,numpy as np
from c1_force_bound import *
checks={};mu=effective_mu_bound([.3,1.,1.5]);factor=np.sqrt(1+mu*mu)
checks['multiply_combine_covered']=mu==2.25
checks['exact50N_no_stop']=contact_norm_upper_bound([50/factor],mu)<=50+1e-12
checks['above50N_stops']=contact_norm_upper_bound([50.000001/factor],mu)>50
checks['opposing_tangents_cannot_hide_load']=contact_norm_upper_bound([20,20],mu)+1e-10>=2*np.hypot(20,20*mu)
rng=np.random.default_rng(42)
for i in range(1000):
 n=rng.uniform(0,80,8);t=rng.uniform(-1,1,8)*mu*n
 if contact_norm_upper_bound(n,mu)+1e-10<float(np.hypot(n,t).sum()):raise AssertionError('Bound violated')
checks['1000_independent_contact_decompositions_bounded']=True
for values in [[-1],[float('nan')]]:
 try:effective_mu_bound(values);checks['invalid_material_fails_closed']=False
 except ValueError:checks['invalid_material_fails_closed']=True
checks={k:bool(v) for k,v in checks.items()}
print(json.dumps({'kind':'PURE_CONSERVATIVE_COULOMB_BOUND_TEST','checks':checks,'all_pass':all(checks.values()),'assumption':'Each actual contact obeys the solver Coulomb cone; no measured per-contact tangential mapping or exact calibration is claimed.'}));assert all(checks.values())
