"""Disclosed state-aware W1 teacher. Sole mutation: emb.step(Action(14D)).

This is a complete unvalidated attempt, not a successful demo. It reads actual
tool/robot poses for payload planning; there are no task state setters here.
"""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from contacts import arr
from yam_arena import ik
from yam_arena.constants import TCP_OFFSET

class ReferenceComplete(Exception):
    """A real, validated full-score termination completed the robot attempt."""

class Teacher:
    def __init__(self,emb,engine,log):
        self.emb=emb;self.E=engine;self.log=log
        self.cmd=np.array(emb._observation().state['joint_pos'],dtype=float,copy=True)
        self.R=engine.R;self.T=engine.T+engine.origin
        self.payload_bias=np.zeros(3)
        self.payload_rotation_bias=np.zeros(3)
        self.continuation_grasp=None
        self.continuation_active=False

    def action(self,label):
        from inspect_robots.types import Action
        if self.E.controls>=3870:raise RuntimeError('original3870-action budget exhausted')
        result=self.emb.step(Action(data=self.cmd.copy()))
        self.log.append(dict(step=self.E.controls,phase=label,action=self.cmd.tolist(),status=self.E.status()))
        if result.terminated:
            if result.termination_reason=='success' and self.E.status()['score']==1.:raise ReferenceComplete()
            raise RuntimeError('terminated: '+str(result.termination_reason))
        return result

    def hand(self):
        a=self.E.base.scene['left_robot'];s=arr(a.data.body_link_state_w)[0,a.body_names.index('gripper')]
        R=Rotation.from_quat(s[3:7]).as_matrix();return s[:3]+R@TCP_OFFSET,R

    def tool(self,name):
        s=arr(self.E.base.scene[name].data.root_link_state_w)[0]
        return s[:3].copy(),Rotation.from_quat(s[3:7]).as_matrix()

    def jaw(self,fraction,n=20):
        initial=self.cmd[6]
        for k in range(n):self.cmd[6]=initial+(fraction-initial)*(k+1)/n;self.action('jaw')

    def move(self,p,R,n=60,label='move',payload=None):
        p0,R0=self.hand();rv=Rotation.from_matrix(R@R0.T).as_rotvec()
        robot=self.E.base.scene['left_robot'];q=arr(robot.data.joint_pos)[0,:6].copy()
        base=arr(robot.data.root_pos_w)[0].copy()
        for k in range(n+(20 if payload is not None and n>1 else 0)):
            f=min((k+1)/n,1.);f=f*f*(3-2*f);pt=p0+(np.asarray(p)-p0)*f;Rt=Rotation.from_rotvec(rv*f).as_matrix()@R0
            wanted=None
            if payload is not None:
                name,offset,relative=payload;wanted=pt+Rt@offset
                pt=pt+self.payload_bias
                desired_tool_R=Rt@relative
                if label in ("fit_wrench","refit") or self.continuation_active:
                    Rt=Rotation.from_rotvec(self.payload_rotation_bias).as_matrix()@Rt
            q,pe,re=ik.solve(pt,ik.matrix_to_quat_xyzw(Rt),q,base_pos=base,iters=160,damping=.015)
            if pe>.003 or re>.05:raise RuntimeError(f'{label} SDK IK failed: {pe}m/{re}rad')
            native=self.R.T@(pt-self.T)
            if np.any(native<[-.30,-.50,.825]) or np.any(native>[.50,.65,1.27]):raise RuntimeError('original TCP workspace exceeded')
            self.cmd[:6]=q;self.action(label)
            if payload is not None:
                actual,actualR=self.tool(name);error=wanted-actual
                # Bounded integral compensates observed loaded tracking sag; no task writes.
                self.payload_bias=np.clip(self.payload_bias+.25*error,-.008,.008)
                rotation_error=Rotation.from_matrix(desired_tool_R@actualR.T).as_rotvec()
                angle=float(np.linalg.norm(rotation_error))
                if label in ("fit_wrench","refit") or self.continuation_active:
                    # Measured angular servo lag dominates 43764; the grasp is rigid.
                    # World-frame SO(3) feedback, 0.5 degree/control maximum update.
                    delta=.25*rotation_error
                    norm=float(np.linalg.norm(delta))
                    if norm>math.radians(.5):delta*=math.radians(.5)/norm
                    candidate=self.payload_rotation_bias+delta
                    norm=float(np.linalg.norm(candidate))
                    if norm>math.radians(7):candidate*=math.radians(7)/norm
                    self.payload_rotation_bias=candidate
                if k>=n-1 and np.linalg.norm(error)<=.001 and angle<=.025:break
                if k==n+19:raise RuntimeError(f'{label} actual payload tracking failed: {np.linalg.norm(error)}m/{angle}rad')

    def carry(self,name,p,R,n=60,label='carry'):
        ph,Rh=self.hand();pk,Rk=self.tool(name)
        offset=Rh.T@(pk-ph);relative=Rh.T@Rk
        if self.continuation_grasp is not None:
            offset,relative=self.continuation_grasp
        handR=R@relative.T
        self.move(np.asarray(p)-handR@offset,handR,n,label,payload=(name,offset,relative))

    def refit_phase(self,name,N,deg):
        # Choose a hex-equivalent phase only after checking the next complete
        # 30-degree stroke with installed IK and the measured hand/tool relation.
        ph,Rh=self.hand();pk,Rk=self.tool(name)
        offset=Rh.T@(pk-ph);relative=Rh.T@Rk
        robot=self.E.base.scene['left_robot'];base=arr(robot.data.root_pos_w)[0].copy()
        current=arr(robot.data.joint_pos)[0,:6].copy();candidates=[]
        for j in range(-4,5):
            psi=deg+60*j
            if not -180<=psi<=180:continue
            q=current.copy();cost=0.;valid=True
            for delta in np.linspace(0.,30.,13):
                R=self.R@Rotation.from_euler('z',psi+delta,degrees=True).as_matrix();handR=R@relative.T
                q,pe,re=ik.solve(N-handR@offset,ik.matrix_to_quat_xyzw(handR),q,base_pos=base,iters=220,damping=.015)
                if pe>.003 or re>.05:valid=False;break
                cost+=float(np.linalg.norm(q-current))
            if valid:candidates.append((cost,psi))
        if not candidates:raise RuntimeError('no SDK IK-feasible hex-equivalent full refit stroke')
        return min(candidates)[1]

    def engage(self,name,N,psi):
        # A correctly seated open wrench can have clearance at rest. Take up at
        # most two degrees through robot actions, checking the actual nut view.
        if self.E.last and any(self.E.last['tool_nut_contact'][0]):return psi
        p,R=self.tool(name)
        if np.linalg.norm(p-N)>.002:raise RuntimeError('actual tool-nut engagement absent: head not seated')
        for k in range(8):
            angle=psi+.25*(k+1)
            R=self.R@Rotation.from_euler('z',angle,degrees=True).as_matrix()
            self.carry(name,N,R,1,'engagement_probe')
            if self.E.last and any(self.E.last['tool_nut_contact'][0]):return angle
        raise RuntimeError('actual tool-nut engagement absent after bounded two-degree take-up')

    def run(self):
        # Correct tool is private teacher knowledge, not public observation.
        name='w1_2' if self.E.f['params']['nut_size_yellow']=='1/2in' else 'w13'
        home,homeR=self.tool(name);grasp=home+homeR@[0,-.10,0]
        down=homeR@np.diag([1.,-1.,-1.]);p0,R0=self.hand()
        # Symmetric fingers permit the pi-equivalent top grasp; choose real SDK IK.
        robot=self.E.base.scene['left_robot'];base=arr(robot.data.root_pos_w)[0].copy();q=arr(robot.data.joint_pos)[0,:6].copy()
        candidates=[]
        for flip in (0.,math.pi):
            r=down@Rotation.from_euler('z',flip).as_matrix()
            qq,pe,re=ik.solve(grasp+[0,0,.08],ik.matrix_to_quat_xyzw(r),q,base_pos=base,iters=220,damping=.015)
            if pe<.003 and re<.05:candidates.append((np.linalg.norm(qq-q),r))
        if not candidates:raise RuntimeError('no actual SDK top-grasp solution')
        down=min(candidates,key=lambda a:a[0])[1]
        self.jaw(.22);self.move(grasp+[0,0,.08],down,90,'rack_approach')
        self.move(grasp,down,60,'handle_grasp');self.jaw(0.,30)
        self.carry(name,home+[0,0,.12],homeR,70,'extract')
        N=self.R@np.array([.12,-.02,.894])+self.T
        psi=-28.;targetR=self.R@Rotation.from_euler('z',psi,degrees=True).as_matrix()
        A=N-targetR[:,1]*.035
        self.carry(name,A+[0,0,.055],targetR,100,'nut_approach')
        self.carry(name,A,targetR,70,'nut_lower')
        self.carry(name,N,targetR,100,'fit_wrench')
        return self.continue_goal(name,home,homeR,N,psi)

    def continue_goal(self,name,home,homeR,N,psi):
        # Freeze actual loaded grasp only after the verified prefix fit returns.
        ph,Rh=self.hand();pk,Rk=self.tool(name)
        self.continuation_grasp=(Rh.T@(pk-ph),Rh.T@Rk)
        self.continuation_active=True
        psi=self.engage(name,N,psi)
        # Real contact transmits every stroke; no latch/joint/history write.
        for stroke in range(4):
            if self.E.mechanism.released:break
            for k in range(60):
                R=self.R@Rotation.from_euler('z',psi+30*(k+1)/60,degrees=True).as_matrix()
                self.carry(name,N,R,1,'loosen_contact')
                if self.E.mechanism.released:break
            p,R=self.tool(name);off=N-R[:,1]*.045
            self.carry(name,off,R,50,'withdraw')
            self.carry(name,off+[0,0,.055],R,50,'withdraw_lift')
            if not self.E.mechanism.released:
                # The loaded stroke changes the measured held-tool transform.
                # Refresh only after the tool has physically withdrawn and lifted;
                # all subsequent movement still uses genuine robot Actions.
                ph,Rh=self.hand();pk,Rk=self.tool(name)
                self.continuation_grasp=(Rh.T@(pk-ph),Rh.T@Rk)
                self.payload_bias=np.zeros(3)
                self.payload_rotation_bias=np.zeros(3)
                # Refit to actual head phase after each genuine turning stroke.
                q=arr(self.E.asset.data.joint_pos)[0,self.E.qids['latch_turn']]
                deg=math.degrees(q)+32.;psi=self.refit_phase(name,N,deg)
                targetR=self.R@Rotation.from_euler('z',psi,degrees=True).as_matrix();A=N-targetR[:,1]*.035
                self.carry(name,A+[0,0,.055],targetR,70,'refit_above')
                self.carry(name,A,targetR,50,'refit_lower');self.carry(name,N,targetR,70,'refit')
                psi=self.engage(name,N,psi)
        if not self.E.mechanism.released:raise RuntimeError('physical nut release not achieved')
        self.carry(name,home+[0,0,.12],homeR,100,'return_above')
        self.carry(name,home+[0,0,.0008],homeR,70,'return_rack');self.jaw(.22)
        self.continuation_grasp=None;self.continuation_active=False
        p,R=self.hand();self.move(p+[0,0,.08],R,60,'tool_clear')
        def knob(a):return self.R@np.array([.12-.08*math.sin(a),-.02+.08*math.cos(a),.9197])+self.T
        down=self.R@np.diag([1.,-1.,-1.]);self.jaw(.4)
        self.move(knob(0)+[0,0,.06],down,90,'knob_approach');self.move(knob(0),down,60,'knob_grasp');self.jaw(0.,30)
        for k in range(150):
            a=-math.pi/2*(k+1)/150;rot=down # cylindrical knob: translate along arc without wrist co-rotation
            self.move(knob(a),rot,1,'open_arm_contact')
        self.jaw(.4);p,R=self.hand();self.move(p+[0,0,.08],R,60,'arm_clear')
        for k in range(420):self.action('final_20s_hands_off')
        return self.E.status()
