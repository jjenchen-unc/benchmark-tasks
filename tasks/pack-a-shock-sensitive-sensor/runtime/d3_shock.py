"""D3 proper-acceleration history: exact2ms time weights across varying physics dt.
This math helper is evaluator-only; actual sensor/force calibration is separate.
"""
from collections import deque
import math
import numpy as np

G=9.81
WINDOW_S=.002
LIMIT_G=50.


def proper_acceleration(velocity, previous_velocity, dt, gravity=(0.,0.,-G)):
    """World linear-velocity derivative minus gravity: stationary1g, freefall0g."""
    v,p,g=[np.asarray(x,dtype=float) for x in (velocity,previous_velocity,gravity)]
    if (not np.isfinite(dt) or not 0<dt<=WINDOW_S or any(x.shape!=(3,) or not np.isfinite(x).all() for x in [v,p,g])):
        raise ValueError('Finite3Dvelocity and positive physics dt<=2ms required')
    return (v-p)/dt-g


class ShockWindow:
    def __init__(self):
        self.samples=deque();self.coverage=0.;self.peak_abs_g=0.;self.peak_vector_g=0.;self.invalid_reason=None

    def add(self, dt, acceleration):
        if self.invalid_reason:raise ValueError(self.invalid_reason)
        a=np.asarray(acceleration,dtype=float)
        if not np.isfinite(dt) or not 0<dt<=WINDOW_S or a.shape!=(3,) or not np.isfinite(a).all():
            self.invalid_reason='missing_nonfinite_or_underresolved_shock_sample'
            raise ValueError(self.invalid_reason)
        dt=float(dt);self.samples.append([dt,a.copy(),float(np.linalg.norm(a))]);self.coverage+=dt
        while self.samples and self.coverage-self.samples[0][0]>=WINDOW_S-1e-14:
            self.coverage-=self.samples.popleft()[0]
        if self.coverage>WINDOW_S:
            excess=self.coverage-WINDOW_S;self.samples[0][0]-=excess;self.coverage=WINDOW_S
        out=self.status()
        if out['valid']:
            self.peak_abs_g=max(self.peak_abs_g,out['mean_abs_g']);self.peak_vector_g=max(self.peak_vector_g,out['norm_mean_g'])
        return self.status()

    def status(self):
        mean_abs=math.fsum((dt/WINDOW_S)*(mag/G) for dt,a,mag in self.samples)
        vector=np.array([math.fsum((dt/WINDOW_S)*(a[i]/G) for dt,a,mag in self.samples) for i in range(3)])
        valid=bool(self.invalid_reason is None and self.coverage>=WINDOW_S-1e-12)
        return {'valid':valid,'coverage_s':float(self.coverage),'mean_abs_g':float(mean_abs),'norm_mean_g':float(np.linalg.norm(vector)),
                'peak_abs_g':float(self.peak_abs_g),'peak_vector_g':float(self.peak_vector_g),
                'over_rating':bool(max(self.peak_abs_g,self.peak_vector_g)>LIMIT_G),
                'within_rating':bool(valid and max(self.peak_abs_g,self.peak_vector_g)<LIMIT_G),
                'invalid_reason':self.invalid_reason,'max_sample_dt_s':max((dt for dt,a,mag in self.samples),default=None)}
