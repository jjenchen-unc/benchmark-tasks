"""Physical substep measurements; calibration is bound to the source manifest."""
import hashlib,json
import numpy as np
from pathlib import Path
from mechanism import Mechanism,History,REQUIRED,DT
from contacts import arr,body_paths,Contacts
HERE=Path(__file__).resolve().parent
RUNTIME_FILES=('fixture.json','compiled-fixture.json','latch.usda','kit.py','mechanism.py','observer.py','contacts.py','motor_limits.py','step_guard.py','task.py')
CALIBRATION_CHECKS=('joint_state','pawl_contact','robot_enclosure_contact','wrist_force','substeps','cam_force','motor_limits')
class Observer:
    def __init__(self,emb):
        from isaaclab_physx.physics import PhysxManager
        self.emb=emb;self.base=emb._base;self.asset=self.base.scene['compression_latch'];self.spec=json.loads((HERE/'fixture.json').read_text())['spec'];self.mechanism=Mechanism(self.spec);self.history=History();self.error=None;self.last=None;self.clamp=0.;self.efforts={};self.motor_tables={}
        paths=body_paths(self.asset);robots=body_paths(self.base.scene['left_robot'])+body_paths(self.base.scene['right_robot']);view=PhysxManager.get_physics_sim_view()
        self.touch=Contacts(view,paths,robots);self.pawl=Contacts(view,[next(p for p in paths if p.endswith('/pawl'))],[next(p for p in paths if p.endswith('/enclosure'))])
        self.channels=frozenset();self.calibration=None
        receipt=HERE/'calibration.json'
        if receipt.exists():
            r=json.loads(receipt.read_text());hashes={n:hashlib.sha256((HERE/n).read_bytes()).hexdigest() for n in RUNTIME_FILES}
            if (hashes==r.get('source_sha256') and r.get('passed') is True and r.get('physics_device')=='cpu' and r.get('physics_dt')==DT and r.get('control_hz')==20 and all(r.get('checks',{}).get(n) is True for n in CALIBRATION_CHECKS)):
                self.channels=REQUIRED;self.calibration=r
    def joints(self):return dict(zip(self.asset.joint_names,arr(self.asset.data.joint_pos)[0])),dict(zip(self.asset.joint_names,arr(self.asset.data.joint_vel)[0]))
    def apply_passive(self):
        if self.error:return
        import torch
        try:
            q,v=self.joints();self.efforts=self.mechanism.efforts(q,v,self.clamp,DT)
            effort=torch.tensor([[self.efforts[n] for n in self.asset.joint_names]],dtype=self.asset.data.joint_pos.torch.dtype,device=self.asset.data.joint_pos.torch.device)
            if not torch.isfinite(effort).all():raise ValueError('nonfinite passive effort')
            self.asset.set_joint_effort_target(effort)
        except Exception as exc:self.error=str(exc);self.history.failure='mechanism_unavailable'
    def sample(self,dt):
        if self.error:return
        try:
            q,v=self.joints();hit,_,_=self.touch.read(dt);self.pawl.read(dt)
            self.clamp=sum(abs(c['normal_force_n']) for c in self.pawl.last_contacts)
            wrist=[];motors=[]
            for side in ('left','right'):
                s=self.base.scene[side+'_s4_wrench'];wrist.append(float(np.linalg.norm(arr(s.data.force)[0,s.body_names.index('gripper')])))
                robot=self.base.scene[side+'_robot']
                if side not in self.motor_tables:
                    table=arr(robot.root_view.get_dof_max_forces())[0];names=robot.joint_names
                    expected={**{f'joint{i}':27. if i<4 else 7. for i in range(1,7)},'joint7':50. if side=='left' else 40.,'joint8':50. if side=='left' else 40.}
                    measured={n:float(table[names.index(n)]) for n in expected}
                    if any(abs(measured[n]-limit)>1e-5 for n,limit in expected.items()):raise ValueError('actual arm/finger drive ceiling mismatch '+str(measured))
                    self.motor_tables[side]=measured
                motors.append(bool(getattr(robot,'_s4_total_effort_checked',False)))
            row=dict(handle=float(q['handle']),knob=float(q['knob']),pawl=float(q['pawl']),gap=self.spec['g0']-float(q['lid_slide']),clamp=float(self.clamp),touch=bool(hit),wrist_force=wrist,stripped=self.mechanism.stripped,cam_torque=self.mechanism.torque,cam_force=self.mechanism.force,cam_error=self.mechanism.cam_error,joints={k:float(x) for k,x in q.items()},velocities={k:float(x) for k,x in v.items()},motor_limits_checked=motors,robot_contacts=self.touch.last_contacts,pawl_contacts=self.pawl.last_contacts)
            self.last=row;self.history.update(row,dt)
            if not all(motors):self.channels=frozenset()
        except Exception as exc:self.error=f'{type(exc).__name__}: {exc}';self.history.failure='measurement_unavailable';self.channels=frozenset()
    def result(self):return dict(self.history.result(self.channels),measurement_error=self.error,physics_device=str(self.emb.device),calibration_verified=self.calibration is not None,motor_tables=self.motor_tables)
