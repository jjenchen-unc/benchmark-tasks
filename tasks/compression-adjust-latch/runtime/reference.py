"""Privileged read-only planning; ALL state changes via genuine 14D20Hz Actions.
Unvalidated reference candidate. No simulator/task/force/scorer writes here.
"""
import json,math
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from mechanism import knob_for,CONTROL_LIMIT

def json_scalar(value):
    if isinstance(value,np.generic):return value.item()
    if isinstance(value,np.ndarray):return value.tolist()
    raise TypeError("Unsupported telemetry JSON type: "+type(value).__name__)

def run(emb,observation,output,mode='reference'):
    from inspect_robots.types import Action
    from yam_arena import ik
    from yam_arena.constants import TCP_OFFSET
    import kit
    from contacts import arr,rot_xyzw
    desired=np.asarray(observation.state['joint_pos'],float).copy();actual=desired.copy();robot=emb._base.scene['left_robot'];base=arr(robot.data.body_pos_w)[0,0]-arr(emb._base.scene.env_origins)[0];rng=np.random.default_rng(42);n=0;log=open(output/'actions.jsonl','w',buffering=1);latest=observation
    json.dumps(kit.current_status(),default=json_scalar,allow_nan=False) # preflight before the first Action
    def step(label):
        nonlocal actual,n,latest
        if n>=CONTROL_LIMIT:raise RuntimeError('control budget exhausted')
        if desired.shape!=(14,) or not np.isfinite(desired).all():raise ValueError('invalid14D')
        r=emb.step(Action(data=desired.copy()));n+=1;latest=r.observation;actual=np.asarray(latest.state['joint_pos'],float)
        log.write(json.dumps({'control':n,'phase':label,'action':desired.tolist(),'status':kit.current_status()},default=json_scalar,allow_nan=False)+'\n')
        if r.terminated and n<CONTROL_LIMIT:raise RuntimeError(str(r.termination_reason))
    def solve(p,R):
        def err(q):
            pp,RR=ik.tcp_pose(q,base);return np.r_[pp-p,.25*ik._rot_error(R,RR)]
        best=None
        for q0 in [actual[:6],desired[:6]]+[rng.uniform(ik.LOWER,ik.UPPER) for _ in range(3)]:
            fit=least_squares(err,np.clip(q0,ik.LOWER+1e-8,ik.UPPER-1e-8),bounds=(ik.LOWER,ik.UPPER),max_nfev=100)
            e=err(fit.x);value=np.linalg.norm(e)
            if best is None or value<best[0]:best=(value,fit.x)
            if np.linalg.norm(e[:3])<.0005 and np.linalg.norm(e[3:])<.002:break
        if best[0]>.005:raise RuntimeError('unreachable reference waypoint '+str((p.tolist(),best[0])))
        return best[1]
    def tcp():
        i=robot.body_names.index('gripper');R=rot_xyzw(arr(robot.data.body_quat_w)[0,i]);return arr(robot.data.body_pos_w)[0,i]-arr(emb._base.scene.env_origins)[0]+R@np.array(TCP_OFFSET),R
    def orientation(yaw):
        yaw+=kit.FIX.get('arena_yaw_rad',0.);c,s=math.cos(yaw),math.sin(yaw);return np.array([[c,s,0],[s,-c,0],[0,0,-1.]])
    def target(part):
        a=emb._base.scene['compression_latch'];i=a.body_names.index(part);p=arr(a.data.body_pos_w)[0,i]-arr(emb._base.scene.env_origins)[0]
        return p+np.array([0,0,.006+.025/2+.012 if part=='handle' else .018/2+.012])
    def move(p,yaw,jaw,count,label):
        p0,R0=tcp();j0=desired[6];R=orientation(yaw);rv=Rotation.from_matrix(R@R0.T).as_rotvec()
        count=max(count,math.ceil(1.5*np.linalg.norm(rv)/1.2*20),math.ceil(1.5*np.linalg.norm(p-p0)/.25*20))
        for i in range(count):
            t=(i+1)/count;f=t*t*(3-2*t);Ri=Rotation.from_rotvec(rv*f).as_matrix()@R0
            desired[:6]=solve(p0+(p-p0)*f,Ri);desired[6]=j0+(jaw-j0)*f;step(label)
        if np.linalg.norm(tcp()[0]-p)>.012:raise RuntimeError('physical tracking error >12mm')
    def grip(part,yaw):
        move(target(part)+[0,0,.045],yaw,1.,50,'approach_'+part);move(target(part),yaw,1.,30,'descend_'+part)
        for i in range(30):desired[6]=1-(i+1)/30;step('grasp_'+part)
    def release(part,yaw):
        for i in range(15):desired[6]=(i+1)/15;step('release_'+part)
        move(target(part)+[0,0,.06],yaw,1.,30,'retreat_'+part)
    def turn(part,amount,startyaw,rate):
        start=float(kit._OBSERVER.last[part]);count=math.ceil(1.5*abs(amount)/rate*20)+24
        for i in range(count):
            t=min(1.,(i+1)/(count-24));f=t*t*(3-2*t);yaw=startyaw-amount*f;desired[:6]=solve(target(part),orientation(yaw));desired[6]=0.;step('turn_'+part)
        end=float(kit._OBSERVER.last[part])
        if abs(end-start-amount)>.12:raise RuntimeError('physical part failed to follow gripper '+part)
    try:
        if mode=='noop':
            for _ in range(500):step('noop_negative')
        else:
            adjustment=knob_for(kit.FIX['spec']) if mode=='reference' else 0.
            remaining=adjustment
            while abs(remaining)>.001:
                amount=np.clip(remaining,-math.radians(150),math.radians(150));yaw=math.radians(80) if amount>0 else math.radians(-80)
                grip('knob',yaw);turn('knob',amount,yaw,math.radians(90));release('knob',yaw-amount);remaining-=amount
            yaw=math.pi/2;grip('handle',yaw);turn('handle',math.pi,yaw,math.radians(40));release('handle',yaw-math.pi)
            angle=kit.FIX.get('arena_yaw_rad',0.);c,s=math.cos(angle),math.sin(angle);Rz=np.array([[c,-s,0],[s,c,0],[0,0,1.]])
            move(np.array(kit.FIX['arena_root'])+Rz@np.array([-.20,-.10,.30]),yaw-math.pi,1.,50,'park_hands_off')
            for _ in range(420):step('retention_21s')
        return dict(kind='genuine14D20Hz robot reference candidate',mode=mode,model_calls=0,controls=n,status=kit.current_status(),full_goal_demonstrated=kit.current_score()==1.)
    finally:log.close()
