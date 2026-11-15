"""Real Dream-1 evaluation through the official Inspect adapter, with read-only audit."""
import argparse,functools,hashlib,json,os,sys,time
from pathlib import Path


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('out',type=Path,nargs='?')
    parser.add_argument('--task',type=Path,default=Path(__file__).resolve().parent.parent/'src'/'task.py')
    parser.add_argument('--check-imports',action='store_true',help='Import pinned adapters only; never create a client, simulator or model session')
    parser.add_argument('--max-steps',type=int,default=3600)
    parser.add_argument('--rows-per-request',type=int,default=4)
    parser.add_argument('--driver-check-override',action='store_true')
    a=parser.parse_args()
    if not 1<=a.max_steps<=3600:parser.error('max-steps must be in [1,3600]')
    if not a.check_imports and a.out is None:parser.error('out is required for an actual evaluation')
    if a.rows_per_request<1:parser.error('rows-per-request must be positive')
    task_path=a.task.resolve()
    if task_path.name!='task.py' or not task_path.is_file():parser.error('--task must name the frozen task.py')
    here=task_path.parent
    sys.path.insert(0,str(here))
    os.environ['STAND_PHYSICS_DT']='.001'
    import runtime,task
    from inspect_robots_dream.session import DreamSession
    from inspect_robots.cli import main as inspect_main
    if task.MAX_CONTROL_STEPS!=3600:raise RuntimeError('task action limit differs from the validated 3600-step configuration')
    if a.check_imports:
        import importlib.metadata as metadata
        versions={}
        for package in ('inspect-robots','inspect-robots-dream','yam-arena'):
            try:versions[package]=metadata.version(package)
            except metadata.PackageNotFoundError:versions[package]='editable distribution name unavailable'
        print(json.dumps(dict(status='IMPORT_CHECK_ONLY_MODEL_NOT_RUN',physics_dt=.001,max_steps=3600,
            task=str(task_path),session_methods=all(callable(getattr(DreamSession,n,None)) for n in ('start','act')),versions=versions),indent=2))
        return
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    import dream
    if not dream.DreamClient()._api_key:raise RuntimeError('Dream credentials are not configured')
    if a.driver_check_override:
        from isaaclab.app import AppLauncher
        old=AppLauncher.__init__
        @functools.wraps(old)
        def init(self,*args,**kwargs):
            kwargs['kit_args']=(kwargs.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip()
            return old(self,*args,**kwargs)
        AppLauncher.__init__=init
    start0,act0=DreamSession.start,DreamSession.act
    trace=(out/'measurements.jsonl').open('x',buffering=1)
    continuity=(out/'continuity.jsonl').open('x',buffering=1)
    measure0=runtime.Runtime.measure
    def measure(self):
        row=measure0(self);trace.write(json.dumps(row,allow_nan=False)+'\n');return row
    runtime.Runtime.measure=measure
    def start(self,*args,**kwargs):
        result=start0(self,*args,**kwargs)
        self._stand_session=dict(hash=None,count=0,finished=None)
        continuity.write(json.dumps(dict(kind='episode_start'))+'\n');return result
    def act(self,*args,**kwargs):
        s=self._stand_session;began=time.monotonic();chunk=act0(self,*args,**kwargs);ended=time.monotonic()
        fingerprint=hashlib.sha256(chunk.session_id.encode()).hexdigest() if chunk.session_id else None
        ok=bool(fingerprint) and s['hash'] in (None,fingerprint)
        continuity.write(json.dumps(dict(kind='chunk',index=s['count']+1,session_sha256=fingerprint,
            continuous=ok,idle_s=None if s['finished'] is None else began-s['finished'],
            request_s=ended-began,attempts=chunk.attempts))+'\n')
        if not ok:raise RuntimeError('Dream server session changed or is missing; trial invalid')
        s.update(hash=fingerprint,count=s['count']+1,finished=ended);return chunk
    DreamSession.start,DreamSession.act=start,act
    args=['run','--embodiment','yam_arena','--policy','dream','-P','model=dreamlabs/dream-1-beta',
          '-P',f'rows_per_request={a.rows_per_request}','-E',f'task={task_path}',
          '-E','object_jitter=0','-E','control_hz=20','-E','device=cpu',
          '-E','cam_width=320','-E','cam_height=176','--instruction',task.INSTRUCTION,
          '--scorer','task_score','--max-steps',str(a.max_steps),'--seed','0','--store-frames','--log-dir',str(out)]
    registration=dict(policy='dream',model='dreamlabs/dream-1-beta',max_steps=a.max_steps,
      rows_per_request=a.rows_per_request,control_hz=20,physics_dt=.001,seed=0,object_jitter=0,args=args,
      runner_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
      source={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(here.iterdir()) if p.is_file()},
      scope='Actual official Dream adapter. Read-only physics measurements and session continuity check.')
    (out/'registration.json').write_text(json.dumps(registration,indent=2))
    sys.argv=['inspect-robots',*args]
    try:return inspect_main()
    finally:
        trace.close();continuity.close()
        if runtime.ACTIVE:
            (out/'last-measurement.json').write_text(json.dumps(runtime.ACTIVE.latest,indent=2,allow_nan=False))
if __name__=='__main__':main()
