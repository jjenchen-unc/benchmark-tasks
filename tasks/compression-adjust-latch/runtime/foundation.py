"""EVALUATOR ONLY: prepared joint states and known wrenches diagnose mechanics.
These interventions are never reference-policy controls or feasibility evidence.
No calibration is silently promoted; all measured evidence is written for review.
"""
import json,math,traceback
import numpy as np
from mechanism import History,Mechanism,DT,cam,knob_for

def evaluate(emb,obs,out):
    import torch,kit
    from contacts import arr,rot_xyzw
    from inspect_robots import Scene
    from inspect_robots.types import Action
    report={'kind':'EVALUATOR_ONLY foundation and negatives; no robot feasibility/model score','model_calls':0,'full_goal_demonstrated':False,'checks':{},'phases':[],'calibration_promoted':False};trace=open(out/'foundation.jsonl','w',buffering=1);qrobot=np.array(obs.state['joint_pos'],float);actions=[]
    def checkpoint(stage):
        report['completed_actions']=completed[0];report['attempted_actions']=len(actions)
        (out/'foundation-progress.json').write_text(json.dumps({'stage':stage,'active_phase':report.get('active_phase'),'completed_actions':completed[0],'attempted_actions':len(actions)},indent=2)+'\n')
        (out/'foundation-partial.json').write_text(json.dumps(report,indent=2,default=str)+'\n')
    completed=[0]
    def tick(label,n=1,allow_termination=False):
        report['active_phase']=label;checkpoint('phase_start')
        print('S4 foundation phase:',label,'controls:',completed[0],flush=True)
        rows=[];termination=None
        for _ in range(n):
            actions.append(qrobot.tolist());trace.write(json.dumps({'phase':label,'attempt':len(actions),'action':qrobot.tolist()})+'\n')
            r=emb.step(Action(data=qrobot.copy()));completed[0]+=1;row=kit._OBSERVER.last
            trace.write(json.dumps({'phase':label,'measurement':row,'status':kit.current_status()},default=str)+'\n');rows.append(row)
            if len(rows)%20==0:checkpoint('running')
            if r.terminated:
                termination=r.termination_reason
                if not allow_termination:raise RuntimeError(str(termination))
                break
        report['phases'].append({'label':label,'samples':len(rows),'first':rows[0],'last':rows[-1],'termination':termination});checkpoint('phase_complete');return rows,termination
    def prepared(label,th,kn):
        nonlocal qrobot
        ob=emb.reset(Scene(id=label,instruction='Evaluator-only prepared-state mechanism diagnostic.'),seed=42);qrobot=np.asarray(ob.state['joint_pos'],float);a=kit._OBSERVER.asset;q=torch.zeros_like(a.data.joint_pos.torch);draw=cam(th,kit.FIX['spec']['grip'],kn)[0]
        values={'lid_slide':min(kit.FIX['spec']['g0'],max(0.,draw)),'pawl':draw,'handle':th,'knob':kn}
        for name,x in values.items():q[:,a.joint_names.index(name)]=x
        # Explicit evaluator-only prepared pose, not a robot action/demo.
        a.write_joint_state_to_sim(q,torch.zeros_like(q));kit._OBSERVER.history=History();kit._OBSERVER.mechanism=Mechanism(kit.FIX['spec']);return values
    try:
        rows,_=tick('neutral_20Hz500Hz',40);report['checks']['action14']=qrobot.shape==(14,);report['checks']['25_substeps_per_action']=kit._OBSERVER.history.samples==1000;report['checks']['open_reset_no_credit']=kit.current_status()['score']==0 and not rows[-1]['touch'] and rows[-1]['clamp']<.5
        a=kit._OBSERVER.asset;i=a.body_names.index('enclosure');origin=arr(emb._base.scene.env_origins)[0];root=arr(a.data.body_pos_w)[0,i]-origin
        report['checks']['root_position_m']=np.linalg.norm(root-kit.FIX['arena_root'])<1e-5
        report['checks']['three_nonblack_RGB']=set(obs.images)=={'top_cam','left_cam','right_cam'} and all(float(im.std())>5 for im in obs.images.values())
        # Validate the actual raised physical mounts and loaded SDK FK.
        from yam_arena import ik
        from yam_arena.station import BASES
        from yam_arena.constants import TCP_OFFSET
        report['mounts']=[]
        for side in ('left','right'):
            robot=emb._base.scene[side+'_robot'];base=arr(robot.data.body_pos_w)[0,0]-origin;expected=np.asarray(BASES[side])+[0.,0.,kit.FIX.get('robot_base_rise_m',0.)]
            B=rot_xyzw(arr(robot.data.body_quat_w)[0,0]);gi=robot.body_names.index('gripper');G=rot_xyzw(arr(robot.data.body_quat_w)[0,gi]);tcp=arr(robot.data.body_pos_w)[0,gi]-origin+G@np.array(TCP_OFFSET)
            ids=robot.find_joints(['joint'+str(i) for i in range(1,7)],preserve_order=True)[0];fp,fR=ik.tcp_pose(arr(robot.data.joint_pos)[0,ids]);pe=float(np.linalg.norm(base+B@fp-tcp));re=float(np.linalg.norm(ik._rot_error(B@fR,G)))
            report['mounts'].append({'side':side,'actual_base':base.tolist(),'expected_base':expected.tolist(),'FK_position_error_m':pe,'FK_rotation_error_rad':re})
            report['checks'][side+'_physical_mount_and_FK']=np.linalg.norm(base-expected)<1e-5 and np.linalg.norm(B-np.eye(3))<1e-5 and pe<.002 and re<.01
        root_R=rot_xyzw(arr(a.data.body_quat_w)[0,i]);yaw=kit.FIX.get('arena_yaw_rad',0.);c,s=math.cos(yaw),math.sin(yaw);fixture_R=np.array([[c,-s,0],[s,c,0],[0,0,1.]])
        report['checks']['declared_whole_fixture_yaw']=np.linalg.norm(root_R-fixture_R)<1e-5
        from PIL import Image
        for name,frame in obs.images.items():Image.fromarray(frame).save(out/('initial-'+name+'.png'))
        kn=knob_for(kit.FIX['spec']);prepared('EVALUATOR_ONLY_adjusted_locked',math.pi,kn);rows,_=tick('EVALUATOR_ONLY_retention_22s',440)
        last=rows[-1];report['checks']['prepared_physical_retention']=last['handle']>=math.radians(175) and last['gap']<.001 and last['clamp']>20 and not last['stripped']
        report['checks']['cam_error_below_0p1mm']=max(abs(r['cam_error']) for r in rows)<.0001
        report['checks']['substep_hold_20s']=kit._OBSERVER.history.counts[0]>=10000
        report['checks']['prepared_gasket_load']=abs(last['clamp']-(kit.FIX['spec']['k']*(kit.FIX['spec']['g0']-last['gap'])-9.81*1.6))<max(5.,.05*last['clamp'])
        # Physical loaded and unloaded adjustment diagnostics. This wrapper is
        # evaluator-only and cannot be reached through reference Actions.
        def known_joint_torque(joint,torque,label,n):
            ob=kit._OBSERVER;original=ob.apply_passive
            def apply():
                original();effort=torch.tensor([[ob.efforts[name]+(torque if name==joint else 0.) for name in ob.asset.joint_names]],device=ob.asset.data.joint_pos.torch.device,dtype=ob.asset.data.joint_pos.torch.dtype);ob.asset.set_joint_effort_target(effort)
            ob.apply_passive=apply
            try:return tick(label,n)
            finally:ob.apply_passive=original
        start=kit._OBSERVER.last['knob'];rows,_=known_joint_torque('knob',.5,'EVALUATOR_ONLY_loaded_screw_0p5Nm',20)
        report['checks']['loaded_screw_holds']=abs(rows[-1]['knob']-start)<.01 and rows[-1]['clamp']>20
        prepared('EVALUATOR_ONLY_open_screw',0.,0.);tick('open_screw_settle',20);start=kit._OBSERVER.last['knob'];rows,_=known_joint_torque('knob',.5,'EVALUATOR_ONLY_unloaded_screw_0p5Nm',20)
        report['checks']['unloaded_screw_rotates']=abs(rows[-1]['knob']-start)>.05
        # Short grip bottoms the gasket before cam peak. Actual contact plus
        # passive coupling must produce stripping; do not write damage flags.
        prepared('EVALUATOR_ONLY_short_grip_damage',math.radians(150),6*math.pi)
        report['cam_overload_intervention']={'kind':'EVALUATOR_ONLY finite handle joint torque; no state/score/damage writes','torque_nm':4.,'max_controls':160,'source_damage_threshold_nm':3.,'source_damage_duration_s':.1}
        rows,_=known_joint_torque('handle',4.,'EVALUATOR_ONLY_sustained_cam_load_4Nm',160)
        report['checks']['physical_cam_strips']=any(r['stripped'] for r in rows)
        rows,_=tick('EVALUATOR_ONLY_after_strip_hands_off',440)
        report['checks']['strip_sticky_no_credit']=rows[-1]['stripped'] and kit.current_score()==0. and kit._OBSERVER.history.counts==[0,0,0]
        # Delivered setting must not satisfy full-gap criterion after release.
        prepared('EVALUATOR_ONLY_no_adjust',math.pi,0.);rows,_=tick('EVALUATOR_ONLY_no_adjust_retention',440)
        report['checks']['no_adjust_fails_full_gap']=rows[-1]['gap']>=.001 or rows[-1]['handle']<math.radians(175) or rows[-1]['clamp']<=20
        # Known wrist load is physically applied; expected 80N/100ms abort.
        ob=emb.reset(Scene(id='evaluator-wrist-load',instruction='Evaluator-only known load.'),seed=42);qrobot=np.asarray(ob.state['joint_pos'],float);robot=emb._base.scene['right_robot'];force=torch.tensor([[[0.,0.,-120.]]],device=emb._base.device)
        robot.permanent_wrench_composer.reset();robot.permanent_wrench_composer.add_forces_and_torques(force,torch.zeros_like(force),body_ids=robot.find_bodies('gripper')[0],is_global=True)
        try:rows,termination=tick('EVALUATOR_ONLY_120N_wrist',8,True)
        finally:robot.permanent_wrench_composer.reset()
        report['checks']['physical_80N100ms_abort']=termination=='wrist_force_80N_100ms'
        report['checks']['motor_limits_active']=all(rows[-1]['motor_limits_checked'])
        # Physical robot/enclosure contact positive, isolated from the policy.
        ob=emb.reset(Scene(id='evaluator-contact',instruction='Evaluator-only contact sensor check.'),seed=42);qrobot=np.asarray(ob.state['joint_pos'],float);robot=emb._base.scene['left_robot']
        from yam_arena import ik
        from scipy.optimize import least_squares
        base=arr(robot.data.body_pos_w)[0,0]-arr(emb._base.scene.env_origins)[0];target=np.array(kit.FIX['arena_root'])+fixture_R@np.array([-.10,.10,.1368+.004+.010]);R=fixture_R@np.diag([1.,-1.,-1.])
        def err(q):
            p,r=ik.tcp_pose(q,base);return np.r_[p-target,.25*ik._rot_error(R,r)]
        rng=np.random.default_rng(42);best=None
        for seed in [qrobot[:6]]+[rng.uniform(ik.LOWER,ik.UPPER) for _ in range(8)]:
            fit=least_squares(err,np.clip(seed,ik.LOWER+1e-8,ik.UPPER-1e-8),bounds=(ik.LOWER,ik.UPPER),max_nfev=300)
            error=err(fit.x);value=float(np.linalg.norm(error))
            if best is None or value<best[0]:best=(value,fit,error)
            if np.linalg.norm(error[:3])<.0005 and np.linalg.norm(error[3:])/.25<.008:break
        _,fit,error=best
        report['contact_test_IK']={'position_error_m':float(np.linalg.norm(error[:3])),'rotation_error_rad':float(np.linalg.norm(error[3:])/.25),'q':fit.x.tolist(),'target':target.tolist(),'R':R.tolist()}
        report['checks']['contact_test_IK']=np.linalg.norm(error[:3])<.0005 and np.linalg.norm(error[3:])/.25<.008
        if report['checks']['contact_test_IK']:
            q=robot.data.joint_pos.torch.clone();v=torch.zeros_like(q)
            for i,name in enumerate(('joint1','joint2','joint3','joint4','joint5','joint6')):q[:,robot.joint_names.index(name)]=float(fit.x[i])
            for name in ('joint7','joint8'):q[:,robot.joint_names.index(name)]=0.
            robot.write_joint_state_to_sim(q,v);qrobot[:6]=fit.x;qrobot[6]=0.
            rows,_=tick('EVALUATOR_ONLY_prepared_robot_touch',4,True)
            report['checks']['physical_robot_enclosure_contact']=any(r['touch'] for r in rows)
        else:report['checks']['physical_robot_enclosure_contact']=False
        report['remaining_calibration']=['cam-force/joint-wrench load and sign agreement','runtime total arm/finger effort table','reset reproducibility after loaded/damaged phases','review actual per-substep raw telemetry and all physical check outcomes before calibration promotion']
        report['checks']={k:bool(v) for k,v in report['checks'].items()}
        report['passed_observed_checks']=all(report['checks'].values());return report
    except BaseException as exc:
        report['error']={'type':type(exc).__name__,'message':str(exc),'traceback':traceback.format_exc()};checkpoint('exception');raise
    finally:
        checkpoint('finished' if 'error' not in report else 'exception')
        trace.close();(out/'actions.json').write_text(json.dumps(actions)+'\n');(out/'foundation-partial.json').write_text(json.dumps(report,indent=2,default=str)+'\n')
