"""Read-only D1 planning scene built from a measured snapshot and task modules.
No physics writes, scoring writes, SDK monkeypatches or historical-trial reads.
"""
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from yam_arena import YAM_URDF
from yam_arena.urdf import forward_kinematics
from yam_arena.ik import tcp_pose,LOWER,UPPER
from yam_arena.constants import ARM_JOINTS,FINGER_JOINTS,FINGER_OPEN,READY,TCP_OFFSET
from yam_arena.station import rig,camera_stand
from yam_arena.analysis import world_hulls
from planning_geometry import sat,sepbox,segment_inside,exact_segment_hull,hull_set_clearance

class MeasuredScene:
    def __init__(self,snapshot,geo,strap):
        self.snapshot=snapshot;self.row=snapshot['row'];self.geo=geo;self.strap=strap
        self.raw={s:np.array(self.row['robot_raw_joint_pos'][s],dtype=np.float64) for s in ('left','right')}
        self.bases={s:np.array(snapshot['bases'][s],dtype=np.float64) for s in self.raw}
        assert all(q.shape==(8,) and np.isfinite(q).all() for q in self.raw.values())
        self.frame=strap.ToteFrame()
        self.hookparts=[('handle',np.zeros(3),np.array(geo.HANDLE)),('shank',np.array(geo.SHANK_POS),np.array(geo.SHANK_HALF)),('foot',np.array(strap.FOOT_POS),np.array(strap.FOOT_HALF))]
        self.bars=[]
        for fixture in ("tote_tall","tote_low"):
            t=geo.tote(fixture);BR=geo.rot_z(t["yaw"]);axis=BR@np.array([0.,1.,0.])
            for part,c in t["eyes"]:
                centre=t["origin"]+BR@np.array(c)
                self.bars.append((fixture+"/"+part,centre-geo.BAR_HALF*axis,centre+geo.BAR_HALF*axis,geo.BAR_RADIUS))
        self.boxes=[]
        for name in ('tote_tall','tote_low'):
            t=geo.tote(name);R=geo.rot_z(t['yaw']);gap=self.row['totes'][name]['lid_gap_m']
            parts=t['walls']+t['brackets']+[('lid',(0,0,t['lid_shut_z']+gap),t['lid_half'])]
            for part,c,h in parts:self.boxes.append((name+'/'+part,t['origin']+R@np.array(c),R,np.array(h)))
        tr=geo.tray();R=geo.rot_z(geo.YAW)
        self.boxes.append(('tray/plate',tr['origin'],R,np.array(tr['plate_half'])))
        for name,c,h in tr['holders']:
            c=np.array(c);c[2]-=geo.TRAY_T
            self.boxes.append(('tray/'+name,tr['origin']+R@c,R,np.array(h)))
        for side in self.raw:
            bp=self.bases[side][:3,3]
            self.boxes.append((side+'/riser',np.array([bp[0],bp[1],geo.TABLE_TOP+.1]),np.eye(3),np.array([.08,.08,.1])))
        foot=camera_stand(rig().top_cam)['stand_foot'][2]
        self.boxes.append(('camera/riser',np.array([foot[0],foot[1],geo.TABLE_TOP+.1]),np.eye(3),np.array([.055,.055,.1])))
        self.boxes.append(('table',np.array([.36,0,.7175]),np.eye(3),np.array([.55,.65,.03])))
        for name,h in self.row['hooks'].items():
            self.add_model_hook(name,np.array(h['pos']),strap.quat_to_R(h['quat_xyzw']))
        self.rng=np.random.default_rng(912)
    def add_model_hook(self,name,p,R):
        self.boxes=[b for b in self.boxes if not b[0].startswith(name+'/')]
        self.boxes.extend((name+'/'+n,p+R@c,R,h) for n,c,h in self.hookparts)
    def link_poses(self,side,q,fingers=None):
        finger=self.raw[side][6:] if fingers is None else np.asarray(fingers,float)
        joints=dict(zip(ARM_JOINTS+FINGER_JOINTS,np.r_[np.asarray(q,float),finger]))
        return {n:self.bases[side]@T for n,T in forward_kinematics(YAM_URDF,joints).items()}
    def robot_hulls(self,side,q,fingers=None,all_links=False):
        poses=self.link_poses(side,q,fingers)
        return world_hulls(poses,None if all_links else [n for n in poses if n not in ('base','link1')])
    def tcp(self,side,q):
        T=self.link_poses(side,q)['gripper'];R=T[:3,:3]
        return T[:3,3]+R@np.array(TCP_OFFSET),R
    def grasp(self,side,hook_name):
        p,R=self.tcp(side,self.raw[side][:6]);h=self.row['hooks'][hook_name]
        hp=np.array(h['pos']);Rh=self.strap.quat_to_R(h['quat_xyzw'])
        return dict(offset=R.T@(hp-p),relative=R.T@Rh,hook_pos=hp,hook_R=Rh,tcp=p,Rhand=R)
    def fit_pose(self,side,p,R,seed,multistart=False):
        def residual(q):
            pp,RR=self.tcp(side,q)
            return np.r_[pp-p,.2*Rotation.from_matrix(R@RR.T).as_rotvec()]
        starts=[seed]
        if multistart:starts+=[np.array(READY)]+[self.rng.uniform(LOWER+.001,UPPER-.001) for _ in range(10)]
        fits=[]
        for initial in starts:
            f=least_squares(residual,np.clip(initial,LOWER+1e-8,UPPER-1e-8),bounds=(LOWER,UPPER),max_nfev=130,gtol=1e-8,ftol=1e-8,xtol=1e-8)
            error=residual(f.x)
            if np.linalg.norm(error[:3])<.001 and np.linalg.norm(error[3:])/.2<.01:fits.append((np.linalg.norm(f.x-seed),f.x))
        return min(fits,key=lambda v:v[0])[1] if fits else None
    def fit_hook_free(self,side,root,offset,seed):
        def residual(q):
            p,R=self.tcp(side,q)
            return np.r_[p+R@offset-root,.0003*(q-seed)]
        f=least_squares(residual,np.clip(seed,LOWER+1e-7,UPPER-1e-7),bounds=(LOWER,UPPER),max_nfev=100,ftol=1e-8,xtol=1e-8,gtol=1e-8)
        return f.x if np.linalg.norm(residual(f.x)[:3])<.005 else None
    def check_robot(self,side,q,points,held_hook=None,other_q=None,allow_lid=False,fingers=None):
        other='right' if side=='left' else 'left'
        oh=self.robot_hulls(other,self.raw[other][:6] if other_q is None else other_q,all_links=True)
        hulls=self.robot_hulls(side,q,fingers);minimum=1.;nearest=None;contacts=[];hits=[];exact=[]
        for ln,hull in hulls:
            for bn,c,R,half in self.boxes:
                if held_hook and bn.startswith(held_hook+'/'):continue
                v=(hull.vertices-c)@R;g=np.maximum(v.min(0)-half,-half-v.max(0)).max()
                if g>.025:continue
                d=sat(hull.vertices,c,R,half)
                if bn=='tote_tall/lid' and allow_lid and ln in ('tip_left','tip_right'):
                    if d<.001:contacts.append(dict(link=ln,separation_m=d))
                    continue
                if d<minimum:minimum=d;nearest=(ln,bn)
            for i,(P,Q) in enumerate(zip(points[:-1],points[1:])):
                if segment_inside(P,Q,hull,margin=.004):
                    hits.append(dict(link=ln,segment=i));exact.append(dict(link=ln,segment=i,**exact_segment_hull(P,Q,hull)))
        bar_checks=[];bar_minimum=1.
        for ln,hull in hulls:
            hlo,hhi=hull.vertices.min(0),hull.vertices.max(0)
            for bn,P,Q,radius in self.bars:
                lo,hi=np.minimum(P,Q),np.maximum(P,Q)
                lower=float(np.linalg.norm(np.maximum(np.maximum(lo-hhi,hlo-hi),0.)))
                if lower-radius>.025:
                    gap=lower-radius;certificate=dict(method="swept_AABB_lower_bound",distance_lower_m=lower)
                else:
                    certificate=exact_segment_hull(P,Q,hull)
                    gap=float(certificate["distance_lower_m"])-radius
                bar_checks.append(dict(link=ln,bar=bn,gap_lower_m=gap,certificate=certificate))
                bar_minimum=min(bar_minimum,gap)
                if gap<minimum:minimum=gap;nearest=(ln,bn)
        allh=self.robot_hulls(side,q,fingers,True)
        ds,ns=hull_set_clearance([(n,h) for n,h in allh if n in ('gripper','tip_left','tip_right')],[(n,h) for n,h in allh if n in ('base','link1','link2','link3')])
        cross,_=hull_set_clearance(hulls,oh)
        return dict(fixture_separation_m=minimum,nearest=nearest,other_arm_separation_m=cross,self_hand_separation_m=ds,self_nearest=ns,cord_radius_hits=hits,exact_cord_flags=exact,intended_lid_contacts=contacts,fixture_bar_count=len(self.bars),bar_capsule_checks=bar_checks,bar_minimum_separation_m=bar_minimum)
    def check_held_hook(self,name,p,R,bar_eye=None):
        from routeguard import segment_box_distance
        minimum=1.;nearest=None;hits=[];bar=[];all_bar=[]
        for pn,pc,ph in self.hookparts:
            centre=p+R@pc
            for bn,c,BR,half in self.boxes:
                if bn.startswith(name+'/'):continue
                extent=np.abs(R)@ph+np.abs(BR)@half
                if np.max(np.abs(centre-c)-extent)>.015:continue
                d=sepbox(centre,R,ph,c,BR,half)
                if d<minimum:minimum=d;nearest=(pn,bn)
                if d<-.0001:hits.append(dict(part=pn,obstacle=bn,separation_m=d))
            for bn,P,Q,radius in self.bars:
                d=segment_box_distance(P,Q,centre,R,ph)-radius
                all_bar.append(dict(part=pn,bar=bn,gap_m=float(d)))
                if d<minimum:minimum=d;nearest=(pn,bn)
                if d<-.0001:hits.append(dict(part=pn,obstacle=bn,separation_m=float(d)))
            if bar_eye is not None:
                axis=self.frame.bar_axis_world
                bar.append(dict(part=pn,gap_m=segment_box_distance(bar_eye-.013*axis,bar_eye+.013*axis,centre,R,ph)-.003))
        return dict(minimum_fixture_m=minimum,nearest=nearest,fixture_hits=hits,bar_capsule_gaps=bar,fixture_bar_count=len(self.bars),all_bar_capsule_gaps=all_bar)
    def verify_fk(self):
        errors={}
        for side in self.raw:
            expected=self.link_poses(side,self.raw[side][:6]);values=[]
            for name,T in self.snapshot.get('links',{}).get(side,{}).items():
                if name not in expected:continue
                T=np.array(T);ref=expected[name]
                values.append(dict(link=name,position_m=float(np.linalg.norm(T[:3,3]-ref[:3,3])),rotation_rad=float(np.linalg.norm(Rotation.from_matrix(T[:3,:3]@ref[:3,:3].T).as_rotvec()))))
            errors[side]=values
        return errors
