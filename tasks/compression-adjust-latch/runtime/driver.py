"""Freeze first, then run ONLY the copied source in an isolated child process.
Runtime needs root's allocated GPU for RTX and CPU PhysX. No job submission.
"""
import argparse,functools,hashlib,importlib.abc,importlib.util,json,os,shutil,signal,subprocess,sys,traceback
from pathlib import Path
HERE=Path(__file__).resolve().parent
LOCAL_NAMES={p.stem for p in HERE.glob('*.py')}
def hashes(root):
    return {str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(root.rglob('*')) if p.is_file() and '__pycache__' not in p.parts}
def save(path,value):
    tmp=path.with_suffix(path.suffix+'.tmp');tmp.write_text(json.dumps(value,indent=2,default=str)+'\n');tmp.replace(path)
class FrozenImports(importlib.abc.MetaPathFinder):
    def find_spec(self,fullname,path=None,target=None):
        if fullname in LOCAL_NAMES:
            return importlib.util.spec_from_file_location(fullname,HERE/(fullname+'.py'))
def install_override(enabled):
    if not enabled:return
    # Process-local proven UNC RTX550 launcher argument, no SDK/driver edits.
    from isaaclab.app import AppLauncher
    original=AppLauncher.__init__
    @functools.wraps(original)
    def init(self,*a,**kw):
        kw['kit_args']=(kw.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip()
        return original(self,*a,**kw)
    AppLauncher.__init__=init

def frozen_run(args):
    def timeout_signal(signum,frame):raise TimeoutError('Bounded foundation launcher sent signal '+str(signum))
    signal.signal(signal.SIGTERM,timeout_signal)
    sys.dont_write_bytecode=True
    # -I child ignores PYTHONPATH/cwd. Bind every task-local module explicitly.
    sys.path.insert(0,str(HERE));sys.meta_path.insert(0,FrozenImports())
    before=hashes(HERE);save(args.out/'source-before-sha256.json',before)
    report={'mode':args.mode,'model_calls':0,'runtime_completed':False,'full_goal_demonstrated':False,'frozen_source':str(HERE),'driver_check_override':args.driver_check_override};emb=None;error=None;capture=None;original_public_step=None
    save(args.out/'progress.json',dict(stage='cold_start',**report))
    try:
        instruction=json.loads((HERE/'task.draft.json').read_text())['instruction']
        report['instruction']=instruction
        install_override(args.driver_check_override)
        # Simulator imports follow the optional cold-launch override. Actual
        # Kit-dependent helpers are imported only after reset launches Kit.
        from inspect_robots import Scene
        from yam_arena.inspect_embodiment import yam_arena_embodiment
        emb=yam_arena_embodiment(task=str(HERE/'task.py'),device='cpu',object_jitter=0,control_hz=20,cam_width=640 if args.record else 320,cam_height=360 if args.record else 176)
        save(args.out/'progress.json',{'stage':'reset_start','frozen_source':str(HERE)})
        obs=emb.reset(Scene(id='compression-latch-'+args.mode,instruction=instruction),seed=42)
        save(args.out/'progress.json',{'stage':'reset_complete','frozen_source':str(HERE)})
        if args.record:
            from video_capture import Capture
            capture=Capture(args.out,obs.images);original_public_step=emb.step
            def record_step(action):
                result=original_public_step(action);capture.append(result.observation.images);return result
            emb.step=record_step
        if args.mode=='foundation':
            from foundation import evaluate
            report.update(evaluate(emb,obs,args.out))
        else:
            from reference import run
            report.update(run(emb,obs,args.out,args.mode))
        report['runtime_completed']=True
    except BaseException as exc:
        error=exc;report['error']={'type':type(exc).__name__,'message':str(exc),'traceback':traceback.format_exc()}
    finally:
        if capture is not None:
            try:
                report['media']=capture.close()
                if not report['media']['all_encoded']:
                    report['runtime_completed']=False
                    if error is None:error=RuntimeError('Actual camera encoding failed')
            except BaseException as exc:
                report['media_error']=str(exc)
                if error is None:error=exc
            if emb is not None and original_public_step is not None:emb.step=original_public_step
        kit=sys.modules.get('kit')
        if kit is not None:
            try:report['terminal_status']=kit.current_status()
            except Exception as exc:report['status_error']=str(exc)
        after=hashes(HERE);save(args.out/'source-after-sha256.json',after)
        consumed={'driver':str(Path(__file__).resolve()),'task':str(HERE/'task.py')}
        for name,module in tuple(sys.modules.items()):
            if name in LOCAL_NAMES and getattr(module,'__file__',None):consumed[name]=str(Path(module.__file__).resolve())
        foreign={n:p for n,p in consumed.items() if Path(p).parent!=HERE}
        report.update(source_unchanged=before==after,source_sha256=before,consumed_source=consumed,foreign_local_modules=foreign)
        save(args.out/'consumed-source.json',{'files_sha256':before,'module_paths':consumed,'foreign_local_modules':foreign})
        if before!=after or foreign:
            report['runtime_completed']=False
            report['source_integrity_failure']=True;report['full_goal_demonstrated']=False
            if error is None:error=RuntimeError('Frozen consumed-source integrity failed')
        save(args.out/'result.json',report);save(args.out/'progress.json',{'stage':'failed' if error else 'complete','runtime_completed':report['runtime_completed'],'error':report.get('error')})
        if emb is not None:
            try:emb.close()
            except Exception as exc:report['close_error']=str(exc)
        final_after=hashes(HERE);save(args.out/'source-after-sha256.json',final_after)
        if final_after!=before:
            report.update(source_unchanged=False,source_integrity_failure=True,runtime_completed=False,full_goal_demonstrated=False)
            if error is None:error=RuntimeError('Source integrity changed during close')
        save(args.out/'result.json',report)
    if error:raise error

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,required=True);ap.add_argument('--mode',choices=['reference','noop','no-adjust','foundation'],default='foundation');ap.add_argument('--driver-check-override',action='store_true',help='Process-local UNC RTX550 cold-launch verification override');ap.add_argument('--record',action='store_true',help='Record every actual step from all three cameras');ap.add_argument('--_execute-frozen',action='store_true',help=argparse.SUPPRESS);args=ap.parse_args();args.out=args.out.resolve()
    if args._execute_frozen:
        if HERE!=args.out/'source':raise RuntimeError('Frozen child path mismatch')
        return frozen_run(args)
    if args.out==HERE or HERE in args.out.parents:raise ValueError('Output must be outside source tree')
    args.out.mkdir(parents=True,exist_ok=False);source=args.out/'source';original=hashes(HERE)
    shutil.copytree(HERE,source,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    copied=hashes(source)
    if original!=copied:raise RuntimeError('Source changed during snapshot copy')
    save(args.out/'live-source-at-copy-sha256.json',original);save(args.out/'source-sha256.json',copied)
    # The child cannot create caches or output in its consumed source.
    for p in source.rglob('*'):p.chmod(0o555 if p.is_dir() else 0o444)
    source.chmod(0o555)
    command=[sys.executable,'-I',str(source/'driver.py'),'--_execute-frozen','--out',str(args.out),'--mode',args.mode]
    if args.driver_check_override:command.append('--driver-check-override')
    if args.record:command.append('--record')
    save(args.out/'execution.json',{'argv':command,'cwd':str(args.out),'source':str(source),'instruction':json.loads((source/'task.draft.json').read_text())['instruction']})
    save(args.out/'progress.json',{'stage':'source_frozen','source':str(source)})
    result=subprocess.run(command,cwd=args.out)
    after=hashes(source);save(args.out/'parent-source-after-sha256.json',after)
    if copied!=after:raise RuntimeError('Frozen source mutated in child')
    if result.returncode:raise SystemExit(result.returncode)
if __name__=='__main__':main()
