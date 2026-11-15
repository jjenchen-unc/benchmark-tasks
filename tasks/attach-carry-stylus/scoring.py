"""End-state retention scoring, independent of magnet/controller state."""
import itertools
import numpy as np
from scipy.spatial.transform import Rotation
from magnet import IPAD_SIZE, LENGTH, pencil_vertices
TOP = .7475
CORNERS = np.array(list(itertools.product(*[[-h,h] for h in IPAD_SIZE/2])))

def geometry(ip, iq, pp, pq):
    ri=Rotation.from_quat(iq).as_matrix();rp=Rotation.from_quat(pq).as_matrix()
    rel=ri.T@(np.asarray(pp)-ip);rr=ri.T@rp
    gap=float((pencil_vertices()@rr.T+rel)[:,1].min()-IPAD_SIZE[1]/2)
    face=float(np.degrees(np.arccos(np.clip(rr[1,1],-1,1))))
    axis=float(np.degrees(np.arccos(np.clip(abs(rr[0,0]),-1,1))))
    low=float((CORNERS@ri.T+ip)[:,2].min()-TOP)
    seated=bool(abs(rel[0]) <= (IPAD_SIZE[0]-LENGTH)/2+.002 and abs(rel[2])<=.004
                and -.0005<=gap<=.001 and face<=15 and axis<=15)
    return dict(relative=rel.tolist(),gap_m=gap,face_deg=face,axis_deg=axis,
                clearance_m=low,seated=seated)

def support_flags(contacts, ipad_path, pencil_path, hand_paths, desk_paths=()):
    """Classify actual body contacts; a fixed robot base is not a holding hand."""
    pen={p for p,f in contacts['pencil'].items() if f>.01}
    tablet={p for p,f in contacts['ipad'].items() if f>.01}
    hands=set(hand_paths)
    return dict(pen_tablet_contact=ipad_path in pen,
                pen_external_contact=bool(pen-{ipad_path}),
                pen_illegal_support=bool(pen-{ipad_path}-set(desk_paths)),
                tablet_robot_contact=bool(tablet & hands),
                tablet_environment_contact=bool(tablet-hands-{pencil_path}))

class Scorer:
    def __init__(self):self.reset()
    def reset(self):
        self.partial_s=0.;self.full_s=0.;self.score=0.;self.steps=0;self.seated_s=0.;self.attached_before_lift=False
    def update(self, m, dt=.05):
        self.steps+=1
        attached=m['seated'] and m['pen_tablet_contact'] and not m['pen_illegal_support']
        if not attached:self.seated_s=0.;self.attached_before_lift=False
        elif m['clearance_m']<=.002:
            self.seated_s+=dt
            self.attached_before_lift |= self.seated_s>=.1-1e-8
        valid=(self.attached_before_lift and attached and not m['pen_external_contact']
               and m['tablet_robot_contact'] and not m['tablet_environment_contact'])
        self.partial_s=self.partial_s+dt if valid and m['clearance_m']>=.020 else 0.
        self.full_s=self.full_s+dt if valid and m['clearance_m']>=.050 else 0.
        self.score=1. if self.full_s>=5.-1e-8 else .5 if self.partial_s>=1.-1e-8 else 0.
        return self.score
