"""Physical-step B5 evaluator history, independent of robot policy observations.

All inputs are measured runtime quantities. Geometry/contacts/wrenches are read
only; this module never actuates or writes a fixture, joint, flag or score.
"""
from __future__ import annotations
import math
import numpy as np
from numeric_score import (score,TOL_H_MM,TOL_DEG,HOLD_S,STILL,ABORT_N,ABORT_S,LOCK_PEN_MM,PIN_IN_MM,OFF_INDEX_MM,OVERTRAVEL_MM,OVERTRAVEL_DEG,BUTTON_PIN_OVERTRAVEL_MM,BASE_MU,BASE_SUSTAIN_S,TARGETS)

def box_overlap(ca,Ra,ha,cb,Rb,hb):
    # Complete separating-axis test, including all nine cross-product axes.
    delta=np.asarray(cb)-ca
    axes=[*np.asarray(Ra).T,*np.asarray(Rb).T]
    axes += [np.cross(a,b) for a in np.asarray(Ra).T for b in np.asarray(Rb).T]
    for axis in axes:
        if np.linalg.norm(axis)<1e-12:continue
        reach=np.sum(abs(np.asarray(Ra).T@axis)*ha)+np.sum(abs(np.asarray(Rb).T@axis)*hb)
        if abs(delta@axis)>reach:return False
    return True

def segment_box_distance(P,Q,c,R,half):
    # Exact piecewise-quadratic segment/AABB distance in the box frame.
    p=np.asarray(R).T@(np.asarray(P)-c);v=np.asarray(R).T@(np.asarray(Q)-P);half=np.asarray(half)
    cuts=[0.,1.]
    for a in range(3):
        if abs(v[a])>1e-15:cuts += [float(t) for t in ((-half[a]-p[a])/v[a],(half[a]-p[a])/v[a]) if 0<t<1]
    cuts=sorted(set(cuts))
    def f(t):return float(np.linalg.norm(np.maximum(abs(p+t*v)-half,0.)))
    best=min(map(f,cuts))
    for lo,hi in zip(cuts[:-1],cuts[1:]):
        at=p+(lo+hi)/2*v;mask=abs(at)>half;vv=v[mask];pp=p[mask]-np.sign(at[mask])*half[mask]
        if vv@vv>1e-20:best=min(best,f(float(np.clip(-(pp@vv)/(vv@vv),lo,hi))))
    return best

def geometric_pose(car_pos,panel_R,lowest_z):
    n,w,u=np.asarray(panel_R).T
    tilt=math.degrees(math.asin(float(np.clip(n[2],-1,1))))
    v=np.array([0.,0.,1.])-n[2]*n
    if np.linalg.norm(v)<1e-9:raise ValueError('screen-normal/vertical degeneracy')
    v/=np.linalg.norm(v);horizontal=np.cross(v,n)
    pivot=math.degrees(math.atan2(float(u@horizontal),float(u@v)))
    return (float(car_pos[2]-lowest_z)*1000,pivot,tilt)

def base_metrics(force,torque,half=(.14,.11)):
    F=np.asarray(force,float);M=np.asarray(torque,float)
    if not np.isfinite(np.r_[F,M]).all():raise ValueError('non-finite base wrench')
    fz=float(F[2])
    if fz<=1e-6:return dict(normal_n=fz,cop_ratio=float('inf'),slide_ratio=float('inf'),yaw_ratio=float('inf'),translation_ratio=float('inf'))
    hx,hy=half
    # Same uniform-pressure foot-radius estimate as the source rubric.
    if not hasattr(base_metrics,'radius'):
        x,y=np.meshgrid(np.linspace(-hx,hx,201),np.linspace(-hy,hy,201));base_metrics.radius=float(np.mean(np.hypot(x,y)))
    rx,ry=-M[1]/fz,M[0]/fz;cop=max(abs(rx)/hx,abs(ry)/hy)
    trans=float(np.hypot(*F[:2]))/(BASE_MU*fz);yaw=abs(M[2]-(rx*F[1]-ry*F[0]))/(BASE_MU*fz*base_metrics.radius)
    return dict(normal_n=fz,cop_ratio=float(cop),translation_ratio=trans,yaw_ratio=float(yaw),slide_ratio=float(np.hypot(trans,yaw)))

class History:
    def __init__(self,target='portrait_side',foot_R=None):
        # The source foot rectangle belongs to the fixture, including after an
        # explicitly declared scene yaw. Wrenches remain world-frame in traces.
        self.foot_R=np.eye(3) if foot_R is None else np.array(foot_R,float,copy=True)
        if self.foot_R.shape!=(3,3) or not np.isfinite(self.foot_R).all() or not np.allclose(self.foot_R.T@self.foot_R,np.eye(3),atol=1e-10) or not np.isclose(np.linalg.det(self.foot_R),1.) or not np.allclose(self.foot_R[:,2],[0.,0.,1.],atol=1e-10):raise ValueError('foot frame must be a finite proper yaw rotation')
        self.target=target;self.samples=0;self.time_s=0.;self.hold_s=0.;self.best_hold_s=0.;self.abort_t=np.zeros(2);self.wrist_abort=False;self.max_wrist=np.zeros(2)
        self.base_bad_s=0.;self.base_bad=False;self.integrity=set();self.furniture_steps=0;self.robot_box_steps=0;self.max_furniture_force=0.;self.nonfinite=False;self.row=None;self.failure=None;self.max_lock_pen=0.;self.source_succeeded=False
    def update(self,row,dt):
        if not(math.isfinite(dt) and 0<dt<=.05):raise ValueError('invalid physics dt')
        required=('pose','joint_pose','joint_vel','joint_q','joint_ranges','wrist_force_n','base_force_w','base_torque_at_foot_w','robot_stand_contact','panel_furniture_contact','robot_box_contact','furniture_force_n','lock_penetration_mm')
        if any(k not in row for k in required):raise ValueError('missing physical measurement')
        nums=np.r_[row['pose'],row['joint_pose'],row['joint_vel'],row['wrist_force_n'],row['base_force_w'],row['base_torque_at_foot_w'],row['furniture_force_n'],row['lock_penetration_mm'],list(row['joint_q'].values())]
        if not np.isfinite(nums).all():self.nonfinite=True;self.failure='nonfinite_measurement';return
        if any(type(row[k]) is not bool for k in ('robot_stand_contact','panel_furniture_contact','robot_box_contact')):raise ValueError('invalid contact flag')
        if min(row['wrist_force_n'])<0 or row['lock_penetration_mm']<0 or row['furniture_force_n']<0:raise ValueError('negative magnitude')
        self.samples+=1;self.time_s+=dt;self.row=row
        raw=np.array(row['wrist_force_n']);self.max_wrist=np.maximum(self.max_wrist,raw);self.abort_t=np.where(raw>ABORT_N,self.abort_t+dt,0.);self.wrist_abort |= bool(np.any(self.abort_t>=ABORT_S-1e-10))
        if self.wrist_abort:self.failure='wrist_force_limit'
        self.furniture_steps+=int(row['panel_furniture_contact']);self.robot_box_steps+=int(row['robot_box_contact']);self.max_furniture_force=max(self.max_furniture_force,row['furniture_force_n'])
        h,p,t=row['joint_pose'];q=row['joint_q']
        off={'H':abs(h-round(h/5.)*5.),'T':math.radians(min(abs(t-x) for x in (-5,0,5,10,15,20)))*.070*1000,'P':math.radians(min(abs(p-x) for x in (0,90)))*.045*1000}
        for axis in 'HTP':
            if q['pin_'+axis]*1000<PIN_IN_MM and off[axis]>OFF_INDEX_MM:self.integrity.add('lock_forced_'+axis)
        pen=row['lock_penetration_mm'];self.max_lock_pen=max(self.max_lock_pen,pen)
        if pen>LOCK_PEN_MM:self.integrity.add('lock_pin_penetration')
        for name,(lo,hi) in row['joint_ranges'].items():
            over=max(lo-q[name],q[name]-hi,0.)
            limit=OVERTRAVEL_DEG if name in ('tilt','pivot') else BUTTON_PIN_OVERTRAVEL_MM if name.startswith(('button_','pin_')) else OVERTRAVEL_MM
            if (math.degrees(over) if name in ('tilt','pivot') else over*1000)>limit:self.integrity.add('overtravel_'+name)
        bm=base_metrics(self.foot_R.T@np.asarray(row['base_force_w']),self.foot_R.T@np.asarray(row['base_torque_at_foot_w']));self.base_last=bm
        bad=bm['cop_ratio']>1 or bm['slide_ratio']>1;self.base_bad_s=self.base_bad_s+dt if bad else 0.;self.base_bad |= self.base_bad_s>=BASE_SUSTAIN_S-1e-10
        goal,_=TARGETS[self.target];within=all(abs(a-b)<tol for a,b,tol in zip(row['pose'],goal,(TOL_H_MM,TOL_DEG,TOL_DEG)))
        ok=within and max(abs(np.array(row['joint_vel'])))<STILL and not row['robot_stand_contact'] and not row['panel_furniture_contact']
        self.hold_s=self.hold_s+dt if ok else 0.;self.best_hold_s=max(self.best_hold_s,self.hold_s);self.source_succeeded |= self.hold_s>=HOLD_S-1e-9
    def measurement(self):
        if self.row is None:return None
        r=self.row;h,p,t=r['pose'];jh,jp,jt=r['joint_pose']
        return dict(target=self.target,height_mm=h,pivot_deg=p,tilt_deg=t,joint_height_mm=jh,joint_pivot_deg=jp,joint_tilt_deg=jt,hands_on_stand_at_end=r['robot_stand_contact'],hold_s=self.hold_s,max_furniture_contact_n=self.max_furniture_force,wrist_force_abort=self.wrist_abort,nonfinite=self.nonfinite,samples=self.samples,disallowed_damage=sorted(self.integrity),source_success_final=bool(self.source_succeeded),furniture_contact_steps=self.furniture_steps,robot_box_contact_steps=self.robot_box_steps,mechanism_damage=sorted(self.integrity),base_tip_or_slide=self.base_bad)
