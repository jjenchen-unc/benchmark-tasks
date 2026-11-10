"""Evaluator-only histories and original W1 scoring; no simulator dependencies."""
import math
from collections import deque
import numpy as np
from source_rubric import SOURCE as S

REQUIRED = frozenset({'clock','joint_geometry','tool_nut_torque','robot_tool_contacts',
                      'wrist_force','nonfinger_contacts','passive_release','damage','drive_caps','reset'})
# Deliberately empty: only root's actual runtime evidence may qualify channels.
VALIDATED_RUNTIME_CHANNELS = frozenset()

class Mechanism:
    def __init__(self, params):
        self.params=params; self.af={'13mm':.013,'1/2in':.0127}[params['nut_size_yellow']]
        self.broken=False; self.released=False; self.rounded=False; self.over=0.
        self.gaps=[.01205,.01275,.01305,.01405,.013+params['adjustable_slack']*1e-6]

    def friction(self, angle):
        if angle>math.radians(2.):self.broken=True
        return .03 if self.broken else self.params['breakaway_yellow']*.001

    def update(self, dt, angle, torques, centres_xy):
        values=np.r_[dt,angle,torques,np.asarray(centres_xy).ravel()]
        if not np.isfinite(values).all() or dt<=0:raise ValueError('nonfinite mechanism measurement')
        damaging=False
        for gap,t,offset in zip(self.gaps,torques,centres_xy):
            c=gap-self.af; chi=.155*self.af
            if .00015<=c<chi and abs(t)>self.params['strip_torque']*.001*(1-c/chi):damaging=True
            if c<0 and abs(t)>.05 and np.linalg.norm(offset)<.009:damaging=True
        self.over=self.over+dt if damaging else 0.
        events=[]
        if not self.rounded and self.over>=.010-1e-10:
            self.rounded=True;events.append('nut_rounded')
        if not self.released and not self.rounded and angle>math.radians(60):
            self.released=True;events.append('arm_released')
        return events

class History:
    def __init__(self):
        self.h=dict(rounded_any=False,invalid_measurement_seen=False,hold_full_s=0.,hold_release_s=0.)
        self.time=0.;self.samples=0;self.measurement=None;self.failure=None
        self.wrist_above=np.zeros(2);self.guard_above=np.zeros(2);self.link_window=deque()
        self.max_wrist=np.zeros(2);self.max_link=0.
        self.motion_guard=None
        self.full_samples=0;self.release_samples=0

    def update(self, dt, measurement, wrist, link_loads, dropped=False):
        try:
            if not math.isclose(dt,.002,abs_tol=1e-9):raise ValueError('W1 requires actual 2ms physics samples')
            m=S.validate_measurement(measurement);w=np.asarray(wrist,float)
            if w.shape!=(2,) or not np.isfinite(w).all() or (w<0).any():raise ValueError('invalid wrist force')
            if any(not math.isfinite(v) or v<0 for v in link_loads.values()):raise ValueError('invalid nonfinger contact load')
            self.samples+=1;self.time=self.samples*.002;self.measurement=m
            tier=S.tier_now(m)[1];self.h['rounded_any']|=m['latch_rounded']
            self.full_samples=self.full_samples+1 if tier=='full' else 0
            self.release_samples=self.release_samples+1 if tier in ('full','released') else 0
            self.h['hold_full_s']=self.full_samples*.002
            self.h['hold_release_s']=self.release_samples*.002
            self.wrist_above=np.where(w>80.,self.wrist_above+dt,0.)
            self.guard_above=np.where(w>30.,self.guard_above+dt,0.)
            self.max_wrist=np.maximum(self.max_wrist,w)
            self.link_window.append((self.time,dict(link_loads)))
            while self.link_window and self.link_window[0][0]<self.time-.020+dt/2:self.link_window.popleft()
            sums={}
            for _,loads in self.link_window:
                for name,value in loads.items():sums[name]=sums.get(name,0.)+value*dt/.020
            link=max(sums.values(),default=0.);self.max_link=max(self.max_link,link)
            if np.any(self.wrist_above>=.1-1e-10):self.failure='wrist_force_abort'
            elif dropped:self.failure='object_out_of_bounds'
            # Source guards end the current tool call. Arena's continuous joint
            # Actions have no tool-call boundary; this port ends the episode at
            # the first reporting control boundary, a disclosed stricter choice.
            self.motion_guard='nonfinger_link_guard' if link>50. else ('wrist_force_guard' if np.any(self.guard_above>=.020-1e-10) else None)
            if self.motion_guard and self.failure is None:self.failure=self.motion_guard
        except (S.SchemaError,ValueError,TypeError,KeyError) as e:
            self.h['invalid_measurement_seen']=True;self.failure='measurement_unavailable: '+str(e)

    def result(self):
        out=S.score(self.measurement,self.h)
        if self.failure:out=dict(score=0.,tier='veto',reasons=[self.failure])
        return dict(out,measurement=self.measurement,history=dict(self.h),physics_s=self.time,physics_samples=self.samples)
