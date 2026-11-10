"""Cold, immutable-copy runner. GPU execution belongs to root's serial queue."""
import argparse,functools,hashlib,json,os,shutil,sys,time,traceback
from pathlib import Path

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--mode',choices=['foundation','reference','negative'],default='foundation')
    ap.add_argument('--snapshot',action='store_true');ap.add_argument('--legacy-rtx-override',action='store_true')
    args=ap.parse_args();args.out=args.out.resolve();here=Path(__file__).resolve().parent
    if not args.snapshot:
        args.out.mkdir(parents=True,exist_ok=False)
        copied=args.out/'executed-source'
        shutil.copytree(here,copied,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
        hashes={str(p.relative_to(copied)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(copied.rglob('*')) if p.is_file()}
        (args.out/'executed-sourcehashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
        command=[sys.executable,'-B',str(copied/'foundation.py'),'--out',str(args.out),'--mode',args.mode,'--snapshot']
        if args.legacy_rtx_override:command.append('--legacy-rtx-override')
        (args.out/'executed-command.json').write_text(json.dumps(dict(argv=command,chdir=str(copied)),indent=2)+'\n')
        os.chdir(copied);os.execv(sys.executable,command)
    expected=args.out/'executed-source'
    if here!=expected:raise RuntimeError('--snapshot is internal: execute the recorded immutable copy')
    hashes=json.loads((args.out/'executed-sourcehashes.json').read_text())
    actual={str(p.relative_to(here)):hashlib.sha256(p.read_bytes()).hexdigest() for p in here.rglob('*') if p.is_file() and '__pycache__' not in p.parts}
    if actual!=hashes:raise RuntimeError('copied runtime source differs from recorded hashes')
    # This exact copied rubric actually executes BEFORE cold SDK imports.
    import source_rubric
    import kit
    kit.install()
    if args.legacy_rtx_override:
        from isaaclab.app import AppLauncher
        old=AppLauncher.__init__
        @functools.wraps(old)
        def launch(self,*a,**kw):
            kw['kit_args']=(kw.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip()
            return old(self,*a,**kw)
        AppLauncher.__init__=launch
    os.environ['B1_TRACE']=str(args.out/'physics-measurements.jsonl')
    report=dict(kind=args.mode,source_rubric_executed=source_rubric.EXECUTED,physics_verified=False,model_calls=0,media_verified=False,checks={},actions=[],exception=None)
    emb=None;start=time.monotonic()
    try:
        import numpy as np
        from inspect_robots import Scene
        from inspect_robots.types import Action
        from yam_arena.inspect_embodiment import yam_arena_embodiment
        from PIL import Image
        emb=yam_arena_embodiment(task=str(here/'task.py'),device='cpu',object_jitter=0,control_hz=20,cam_width=320,cam_height=176)
        obs=emb.reset(Scene(id='b1-fixed-case-foundation',instruction=__import__('task').__doc__),seed=987654321)
        E=kit.ENGINE;q=np.asarray(obs.state['joint_pos'],float)
        report['checks']['genuine14D']=q.shape==(14,)
        report['checks']['fixed_case_accepts_harness_derived_seed']=E.f['source_seed']==42
        report['checks']['actual2ms_20Hz']=abs(emb._base.physics_dt-.002)<1e-10 and abs(emb._base.step_dt-.05)<1e-10
        report['cameras']={n:dict(shape=list(im.shape),std=float(im.std())) for n,im in obs.images.items()}
        report['checks']['three_rgb']=set(obs.images)=={'top_cam','left_cam','right_cam'} and all(im.std()>5 for im in obs.images.values())
        for n,im in obs.images.items():Image.fromarray(im).save(args.out/('initial-'+n+'.png'))
        report['drive_limits']={s:kit.arr(emb._base.scene[s+'_robot'].root_view.get_dof_max_forces()).tolist() for s in ['left','right']}
        report['checks']['actual27_7_40_caps']=all(np.allclose(report['drive_limits'][s][0],[27,27,27,7,7,7,40,40]) for s in ['left','right'])
        report['bases']={s:kit.arr(emb._base.scene[s+'_robot'].data.body_pos_w)[0,0].tolist() for s in ['left','right']}
        def step(target,label,n=1):
            nonlocal obs
            for _ in range(n):
                r=emb.step(Action(data=np.asarray(target,float)));obs=r.observation
                report['actions'].append(dict(phase=label,action=np.asarray(target).tolist(),info=r.info))
                if r.terminated:raise RuntimeError(label+': '+str(r.termination_reason))
            return r
        step(q,'neutral',10)
        report['checks']['all_physics_samples_observed']=E.history.samples==250
        report['checks']['initial_locked_undamaged_zero']=not E.mechanism.released and not E.mechanism.rounded and E.history.result()['score']==0.
        if args.mode=='foundation':
            moved=q.copy();moved[0]+=.04;step(moved,'genuine_left_joint_action',20)
            report['checks']['actual_left_joint_moves']=abs(float(obs.state['joint_pos'][0])-moved[0])<.03
            step(q,'return',20)
            before=E.status();obs=emb.reset(Scene(id='b1-reset-other-episode',instruction='Reset foundation'),seed=17)
            report['checks']['reset_clears_clock_damage_locks']=E.history.samples==0 and not E.mechanism.rounded and not E.mechanism.released
            report['before_second_reset']=before
        elif args.mode=='reference':
            from reference import Teacher,ReferenceComplete
            log=[]
            try:report['reference_result']=Teacher(emb,E,log).run()
            except ReferenceComplete:report['reference_result']=E.status()
            finally:(args.out/'reference-actions.json').write_text(json.dumps(log,indent=2)+'\n')
        else:
            from evaluator import run_negative
            report['evaluator_only']=run_negative(emb,args.out)
            report['checks']['evaluator_negatives_pass']=all(report['evaluator_only']['checks'].values())
        report['final']=E.status()
        report['checks']['no_unvalidated_credit']=report['final']['score']==0. and report['final']['gate']=='runtime_channels_unverified'
        # Deliberately force only evaluator-owned episode termination for this
        # API negative, without advancing physics or fabricating task state.
        E.done=True;count=E.history.samples
        try:emb.step(Action(data=q));report['checks']['posttermination_rejected']=False
        except RuntimeError:report['checks']['posttermination_rejected']=E.history.samples==count
    except Exception as exc:
        report['exception']=type(exc).__name__+': '+str(exc);(args.out/'traceback.txt').write_text(traceback.format_exc())
        if kit.ENGINE is not None:report['final']=kit.ENGINE.status()
    finally:
        report['wall_s']=time.monotonic()-start
        (args.out/'result.json').write_text(json.dumps(report,indent=2,default=lambda x:x.tolist() if hasattr(x,'tolist') else str(x))+'\n')
        if emb is not None:emb.close()
    return 1 if report['exception'] or not all(report['checks'].values()) else 0

if __name__=='__main__':raise SystemExit(main())
