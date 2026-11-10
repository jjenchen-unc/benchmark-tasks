"""Passive contact-driven W1 environment. Only reset writes task joint state.

During actions this module changes passive thread friction and opens angle-triggered
locks. It never places a tool, advances a task joint, adds an external wrench, or
changes a policy observation. Scoring history is evaluator-owned.
"""
import json, math, os
import numpy as np
from scipy.spatial.transform import Rotation
from contacts import Contacts, arr
from history import History, Mechanism, REQUIRED, VALIDATED_RUNTIME_CHANNELS

class Physics:
    def __init__(self, emb, fixture):
        self.emb=emb;self.base=emb._base;self.f=fixture
        self.asset=self.base.scene['wrench_fixture'];self.settling=True
        self.qids={n:i for i,n in enumerate(self.asset.joint_names)}
        self.bids={n:i for i,n in enumerate(self.asset.body_names)}
        self.R=np.asarray(fixture['rotation']);self.T=np.asarray(fixture['translation'])
        self.origin=arr(self.base.scene.env_origins)[0]
        self.names=['w12','w1_2','w13','w14','wadj']
        self.mechanism=Mechanism(fixture['params']);self.history=History()
        self.done=False;self.error=None;self.controls=0;self.events=[];self.last=None
        from isaaclab_physx.physics import PhysxManager
        sim=PhysxManager.get_physics_sim_view()
        links=list(self.asset.root_view.link_paths[0])
        self.tools=[str(self.base.scene[n].root_view.prim_paths[0]) for n in self.names]
        robots=sum([list(self.base.scene[s+'_robot'].root_view.link_paths[0]) for s in ['left','right']],[])
        self.nut_path=links[self.bids['latch_head']];self.arm_path=links[self.bids['swing_arm']]
        self.nut_contact=Contacts(sim,[self.nut_path],self.tools)
        self.hands_off=Contacts(sim,[self.nut_path,self.arm_path],robots+self.tools)
        # Each nonfinger moving body is sampled separately; loads cannot cancel
        # across links. All station collider targets are explicit, including fixed furniture.
        nonfinger=[p for p in robots if p.rsplit('/',1)[-1] in ['link1','link2','link3','link4','link5','gripper']]
        env=self.nut_path.split('/wrench_fixture/',1)[0]
        furniture=[env+'/'+n for n in ['table','floor','stand_foot','stand_mast','stand_arm','camera_housing','lamp']]
        self.nonfinger=Contacts(sim,nonfinger,robots+self.tools+links+furniture)
        self.trace=open(os.environ['B1_TRACE'],'w',buffering=1) if os.environ.get('B1_TRACE') else None

    def limits(self, name, values):
        import torch
        self.asset.write_joint_position_limit_to_sim_index(limits=torch.tensor([[values]],dtype=torch.float32,device=self.base.device),env_ids=[0],full_data=False,joint_ids=[self.qids[name]],warn_limit_violation=False)

    def reset_mechanisms(self):
        import torch
        self.settling=True;self.mechanism=Mechanism(self.f['params']);self.history=History()
        self.done=False;self.error=None;self.controls=0;self.events=[];self.last=None
        self.limits('arm_hinge',[0.,0.]);self.limits('latch_slip',[0.,0.])
        z=torch.zeros_like(self.asset.data.joint_pos.torch)
        self.asset.write_joint_state_to_sim_index(position=z,velocity=z)
        self.asset.set_joint_effort_target(z)
        self.rounded_visual(False)

    def begin(self):
        self.history=History();self.settling=False;self.controls=0;self.done=False
        self.right_park=np.asarray(self.emb._observation().state['joint_pos'],float)[7:].copy()

    def pre_step(self, dt):
        import torch
        if self.error:return
        try:
            q=float(arr(self.asset.data.joint_pos)[0,self.qids['latch_turn']])
            tau=self.mechanism.friction(q)
            self.asset.write_joint_friction_coefficient_to_sim_index(joint_friction_coeff=torch.tensor([[float(tau)]],dtype=torch.float32,device=self.base.device),joint_dynamic_friction_coeff=torch.tensor([[float(tau)]],dtype=torch.float32,device=self.base.device),joint_viscous_friction_coeff=torch.tensor([[.0005]],dtype=torch.float32,device=self.base.device),env_ids=[0],full_data=False,joint_ids=[self.qids['latch_turn']])
        except Exception as e:self.error=type(e).__name__+': '+str(e)

    def post_step(self, dt):
        if self.settling or self.error:return
        try:
            q=arr(self.asset.data.joint_pos)[0];states=arr(self.asset.data.body_link_state_w)[0]
            if not np.isfinite(np.r_[q,states.ravel()]).all():raise ValueError('nonfinite fixture state')
            head=states[self.bids['latch_head']];axis=Rotation.from_quat(head[3:7]).as_matrix()[:,2]
            hit,F,M=self.nut_contact.read(dt,head[:3])
            torques=M[0]@axis
            tool_states={n:arr(self.base.scene[n].data.root_link_state_w)[0] for n in self.names}
            offset=[(self.R.T@(tool_states[n][:3]-head[:3]))[:2] for n in self.names]
            turn=float(q[self.qids['latch_turn']]);arm=float(q[self.qids['arm_hinge']])
            events=self.mechanism.update(dt,turn,torques,offset)
            for event in events:
                if event=='nut_rounded':
                    self.limits('latch_slip',[-1e6,1e6]);self.rounded_visual(True)
                if event=='arm_released':self.limits('arm_hinge',[-math.pi/2,0.])
                self.events.append(dict(event=event,physics_s=self.history.time+dt,turn_rad=turn,tool_torques_nm=torques.tolist()))
            touch,_,_=self.hands_off.read(dt)
            _,linkF,_=self.nonfinger.read(dt)
            # Per-contact force magnitudes (normal and friction), not the norm of
            # the net force, preserve the source's no-cancellation link guard.
            link_loads=self.nonfinger.magnitude_by_sensor
            wrist=[]
            for side in ['left','right']:
                sensor=self.base.scene[side+'_b1_wrench']
                wrist.append(float(np.linalg.norm(arr(sensor.data.force)[0,sensor.body_names.index('gripper')])))
            dropped=False
            for state in tool_states.values():
                native=self.R.T@(state[:3]-self.origin-self.T)
                if not np.isfinite(state).all():raise ValueError('nonfinite tool state')
                dropped|=bool(np.any(native<self.f['source_objects_bounds'][0]) or np.any(native>self.f['source_objects_bounds'][1]))
            m=dict(nut_backoff_deg=math.degrees(turn),arm_open_deg=-math.degrees(arm),latch_rounded=self.mechanism.rounded,arm_free=self.mechanism.released,contact_free=not bool(np.any(touch)))
            self.history.update(dt,m,wrist,link_loads,dropped)
            self.last=dict(measurement=m,tool_torques_nm=torques.tolist(),tool_nut_contact=hit.tolist(),wrist_force_n=wrist,link_loads_n=link_loads,tool_states={n:v.tolist() for n,v in tool_states.items()})
            if self.trace:self.trace.write(json.dumps(dict(dt=dt,row=self.last,status=self.status()))+'\n')
        except Exception as e:
            self.error=type(e).__name__+': '+str(e);self.history.h['invalid_measurement_seen']=True

    def status(self):
        candidate=self.history.result();missing=sorted(REQUIRED-VALIDATED_RUNTIME_CHANNELS)
        return dict(candidate,score=0. if missing or self.error else candidate['score'],
                    candidate_score=candidate['score'],gate='runtime_channels_unverified' if missing else None,
                    missing_runtime_channels=missing,error=self.error,controls=self.controls,events=self.events)

    def reason(self):
        if self.error:return 'measurement_unavailable'
        if self.history.failure:return self.history.failure
        if self.controls>=3870:return 'time_limit'
        if self.status()['score']>=1.:return 'success'

    def close(self):
        if self.trace:self.trace.close();self.trace=None

    def rounded_visual(self, rounded):
        from isaaclab.sim.utils import get_current_stage
        from pxr import Gf
        stage=get_current_stage();path=self.nut_path+'/latch_hex'
        color=(.85,.85,.88) if rounded else (.62,.60,.55)
        stage.GetPrimAtPath(path).GetAttribute('primvars:displayColor').Set([Gf.Vec3f(*color)])
        stage.GetPrimAtPath(path+'_material/Shader').GetAttribute('inputs:diffuseColor').Set(Gf.Vec3f(*color))
