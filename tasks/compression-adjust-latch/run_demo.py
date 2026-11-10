"""Replay bundled real14D20Hz actions. Never writes task state."""
from pathlib import Path
import argparse,json,shutil,subprocess,sys
from package_checks import preflight,dumps,digest
HERE=Path(__file__).resolve().parent

def execute(out,override):
    rows=preflight(HERE) # before importing SDK or launching simulation
    rt=HERE/'runtime';sys.path.insert(0,str(rt))
    import driver
    sys.meta_path.insert(0,driver.FrozenImports())
    driver.install_override(override)
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    import numpy as np
    emb=yam_arena_embodiment(task=str(rt/'task.py'),device='cpu',object_jitter=0,control_hz=20,cam_width=640,cam_height=360)
    capture=None;report={'model_calls':0,'full_goal_demonstrated':False};error=None
    try:
        instruction=json.loads((HERE/'task.draft.json').read_text())['instruction']
        obs=emb.reset(Scene(id='compression-latch-saved-action-replay',instruction=instruction),seed=42)
        import kit
        from video_capture import Capture
        dumps(kit.current_status()) # fail closed for unknown/nonfinite telemetry before first Action
        if kit.current_status()['samples']!=0:raise RuntimeError('Reset must have zero episode samples')
        capture=Capture(out,obs.images)
        with (out/'actions.jsonl').open('x',buffering=1) as log:
            for n,row in enumerate(rows,1):
                if n>3848:raise RuntimeError('Budget exceeded')
                r=emb.step(Action(data=np.asarray(row['action'],dtype=float)))
                status=kit.current_status();capture.append(r.observation.images)
                log.write(dumps(dict(control=n,phase=row['phase'],action=row['action'],status=status))+'\n')
                if status['samples']!=25*n:raise RuntimeError('Substep count mismatch')
                if status['failure'] or status['damage'] or status['missing']:raise RuntimeError('Task guard failure')
                if r.terminated:
                    if n!=len(rows):raise RuntimeError('Early termination: '+str(r.termination_reason))
                    break
        report.update(controls=n,status=status,physics_s_derived=status['samples']*.002,full_goal_demonstrated=status['score']==1.)
        if status['score']!=1.:raise RuntimeError('Actual replay score must be1.0')
    except BaseException as exc:
        error=exc;report['error']=dict(type=type(exc).__name__,message=str(exc))
    finally:
        try:
            if capture is not None:
                report['media']=capture.close()
                if not report['media']['all_encoded'] or any(v['bytes']>50_000_000 for v in report['media']['files'].values()):raise RuntimeError('Media encoding/size failure')
            preflight(HERE)
        except BaseException as exc:
            if error is None:error=exc
            report['finalization_error']=str(exc)
        report['success']=error is None and report['full_goal_demonstrated']
        (out/'result.json').write_text(dumps(report)+'\n')
        emb.close()
    if error:raise error

def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--preflight-only',action='store_true');p.add_argument('--driver-check-override',action='store_true');p.add_argument('--_child',action='store_true',help=argparse.SUPPRESS);a=p.parse_args()
    rows=preflight(HERE)
    if a.preflight_only:
        print(dumps(dict(valid=True,controls=len(rows),runtime_files=29,model_calls=0,simulation_starts=0)));return
    out=a.out.resolve()
    if a._child:return execute(out,a.driver_check_override)
    if out==HERE or HERE in out.parents:raise ValueError('Output must be outside task folder')
    out.mkdir(parents=True,exist_ok=False);snapshot=out/'package'
    shutil.copytree(HERE,snapshot,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    for f in snapshot.rglob('*'):f.chmod(0o555 if f.is_dir() else 0o444)
    snapshot.chmod(0o555)
    cmd=[sys.executable,'-I',str(snapshot/'run_demo.py'),'--_child','--out',str(out)]
    # -I excludes script directory: explicitly add only this copied package.
    cmd=[sys.executable,'-I','-c','import sys,runpy;p=sys.argv.pop(1);sys.path.insert(0,p);runpy.run_path(p+"/run_demo.py",run_name="__main__")',str(snapshot),'--_child','--out',str(out)]
    if a.driver_check_override:cmd+=['--driver-check-override']
    subprocess.run(cmd,cwd=out,check=True)
    preflight(HERE)
if __name__=='__main__':main()
