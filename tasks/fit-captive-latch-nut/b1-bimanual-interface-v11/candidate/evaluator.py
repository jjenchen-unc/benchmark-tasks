"""Evaluator-only known-load/negative driver. Never imported by robot teacher.

All placements and direct wrenches in this module are diagnostic privileges;
none is a model/reference action or a feasibility demonstration.
"""
import math
import numpy as np
from scipy.spatial.transform import Rotation
from contacts import arr

def run_negative(emb,out):
    import torch
    import kit
    from inspect_robots import Scene
    from inspect_robots.types import Action
    rows=[]
    def reset(label):
        obs=emb.reset(Scene(id='b1-evaluator-'+label,instruction='Evaluator-only diagnostics, not robot performance.'),seed=123)
        return np.asarray(obs.state['joint_pos'],float)
    def step(q,n,label,allow_termination=False):
        result=None
        for _ in range(n):
            result=emb.step(Action(data=q.copy()))
            rows.append(dict(phase=label,status=kit.ENGINE.status(),last=kit.ENGINE.last))
            if result.terminated:
                if allow_termination:break
                raise RuntimeError(label+': '+str(result.termination_reason))
        return result
    checks={}
    # Direct wrist load: actual JointWrenchSensor signal and duration, with no
    # fabricated sample/history. Default30N guard is intentionally also active.
    q=reset('wrist');step(q,10,'wrist_baseline');E=kit.ENGINE
    robot=E.base.scene['left_robot'];force=torch.tensor([[[0.,0.,-120.]]],device=E.base.device)
    robot.permanent_wrench_composer.reset()
    robot.permanent_wrench_composer.add_forces_and_torques(force,torch.zeros_like(force),body_ids=robot.find_bodies('gripper')[0],is_global=True)
    r=step(q,10,'EVALUATOR_ONLY_120N_wrist',True)
    checks['measured_wrist_safety_terminates']=r.terminated and r.termination_reason in ['wrist_force_guard','wrist_force_abort']
    checks['wrist_negative_no_credit']=E.status()['score']==0.
    robot.permanent_wrench_composer.reset()
    # Contact torque calibration and wrong-size damage. Tool positions below
    # are evaluator setup; only the robot teacher must perform its own pickup.
    for name,torque in [('w1_2',.10),('w13',.40),('w12',.15)]:
        q=reset('contact-'+name);E=kit.ENGINE;tool=E.base.scene[name]
        native=np.array([.12,-.02,.894]);p=E.R@native+E.T+E.origin
        R=E.R@Rotation.from_euler('z',32,degrees=True).as_matrix();quat=Rotation.from_matrix(R).as_quat()
        pose=torch.tensor([np.r_[p,quat]],dtype=torch.float32,device=E.base.device)
        tool.write_root_pose_to_sim_index(root_pose=pose)
        tool.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1,6),device=E.base.device))
        step(q,10,'EVALUATOR_ONLY_placed_'+name)
        t=torch.tensor([[[0.,0.,torque]]],device=E.base.device);z=torch.zeros_like(t)
        tool.permanent_wrench_composer.reset();tool.permanent_wrench_composer.add_forces_and_torques(z,t,is_global=True)
        step(q,8,'EVALUATOR_ONLY_contact_torque_'+name)
        tool.permanent_wrench_composer.reset()
        rows.append(dict(phase='EVALUATOR_ONLY_torque_summary_'+name,applied_nm=torque,measured=E.last,status=E.status()))
        if name=='w1_2':checks['correct_fit_does_not_round']=not E.mechanism.rounded
        elif name=='w13':checks['oversize_contact_rounds']=E.mechanism.rounded
    q=reset('final-reset');E=kit.ENGINE
    checks['reset_restores_damage_and_locks']=not E.mechanism.rounded and not E.mechanism.released and E.history.samples==0
    return dict(kind='evaluator-only direct setup/loads; NOT feasibility',checks=checks,rows=rows)
