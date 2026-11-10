"""Environment-owned passive mechanism and terminal history, independent of SDK.
No controller imports this state machine or writes its fields.
"""
import math
import numpy as np
TH_OC=math.radians(172);LEAD=.0064/(TH_OC-math.pi/2);REV=.00015/(math.pi-TH_OC);PITCH=.001/(2*math.pi)
DT=.002;CONTROL_LIMIT=3848;REQUIRED=frozenset(('joint_state','pawl_contact','robot_enclosure_contact','wrist_force','substeps','cam_force','motor_limits'))

def cam(th,grip,kn):
    a0=-LEAD*math.pi/2-grip+PITCH*kn;slope=LEAD
    if th>TH_OC:a0+=(LEAD+REV)*TH_OC;slope=-REV
    return a0+slope*th,slope

def knob_for(spec):
    lo=spec['g0']-.001+.0001;hi=min(spec['g0']-.0005,(3.-.12-.03)/(spec['k']*LEAD)-.00015)
    c=(lo+hi)/2 if hi>lo else lo
    return (spec['grip']-(.0064-.00015-c))/PITCH

class Mechanism:
    def __init__(self,spec):
        self.spec=spec;self.lock=None;self.stripped=False;self.over_steps=0;self.force=0.;self.torque=0.;self.cam_error=0.
    def efforts(self,q,v,clamp,dt):
        """Balanced compliant cam forces from backward-Euler scalar constraint.
        No task poses/velocities are written. Armature is numerical inertia.
        """
        if clamp>2 and self.lock is None:self.lock=q['knob']
        if clamp<.5:self.lock=None
        kn=q['knob'] if self.lock is None else self.lock
        c,s=cam(q['handle'],self.spec['grip'],kn);e=q['pawl']-c;self.cam_error=e
        J=np.array([1.,-s,-PITCH if self.lock is None else 0.]);vel=np.array([v['pawl'],v['handle'],v['knob']]);inv=np.array([1/250.03,1/.00502,1/.005004])
        kc,dc=1e7,1e4
        self.force=0. if self.stripped else -(kc*e+(dc+kc*dt)*float(J@vel))/(1+dt*(dc+kc*dt)*float((J*J)@inv))
        self.torque=abs(self.force*s)
        self.over_steps=self.over_steps+1 if self.torque>3 else 0
        if self.over_steps*dt>=.1-1e-12:self.stripped=True;self.force=0.;self.torque=0.
        out=dict(zip(('pawl','handle','knob'),self.force*J));out['lid_slide']=-8*self.spec['k']*max(0.,q['lid_slide']-(self.spec['g0']-.0005))
        if self.lock is not None:
            # Loaded screw restraint is a finite passive equality approximation.
            k,d=1e5,100.;iv=1/.005004
            out['knob']+=-(k*(q['knob']-self.lock)+(d+k*dt)*v['knob'])/(1+dt*(d+k*dt)*iv)
        return out

class History:
    def __init__(self):
        self.counts=[0,0,0];self.samples=0;self.damage=False;self.failure=None;self.wrist_steps=[0,0];self.last=None;self.dt=None
    def update(self,row,dt):
        self.samples+=1;self.last=row
        try:
            vals=[row[k] for k in ('handle','gap','clamp','cam_torque','cam_error')]+list(row['wrist_force'])
            if not np.isfinite(vals).all() or not math.isclose(dt,DT,abs_tol=1e-9):raise ValueError('invalid substep telemetry')
            self.dt=dt;self.damage |= bool(row['stripped'])
            for i,f in enumerate(row['wrist_force']):
                self.wrist_steps[i]=self.wrist_steps[i]+1 if f>80 else 0
                if self.wrist_steps[i]>=50:self.failure='wrist_force_80N_100ms'
            locked=(not self.damage and row['handle']>=math.radians(175) and row['clamp']>20 and not row['touch'])
            conditions=[locked and row['gap']<.001,locked and row['gap']<.002,locked]
            self.counts=[n+1 if condition else 0 for n,condition in zip(self.counts,conditions)]
        except (KeyError,ValueError,TypeError):self.failure='measurement_unavailable';self.counts=[0,0,0]
    def result(self,channels=frozenset()):
        missing=sorted(REQUIRED-set(channels));score=0.
        if not missing and not self.failure and not self.damage and self.last is not None:
            for n,grade in zip(self.counts,(1.,.5,.25)):
                if n>=10000:score=grade;break
        return dict(score=score,missing=missing,failure=self.failure,damage=self.damage,hold_seconds=[n*DT for n in self.counts],samples=self.samples,last=self.last)
