"""Real Dream-1 evaluation through the official Inspect adapter, with read-only audit."""
import argparse,functools,hashlib,json,sys,time
from pathlib import Path


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('out',type=Path)
    parser.add_argument('--max-steps',type=int,default=2400)
    parser.add_argument('--rows-per-request',type=int,default=4)
    parser.add_argument('--driver-check-override',action='store_true')
    a=parser.parse_args()
    if not 1<=a.max_steps<=2400:parser.error('max-steps must be in [1,2400]')
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=False)
    here=Path(__file__).resolve().parent
    sys.path.insert(0,str(here))
    import runtime,task
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
    from inspect_robots_dream.session import DreamSession
    start0,act0=DreamSession.start,DreamSession.act
    trace=(out/'measurements.jsonl').open('x',buffering=1)
    continuity=(out/'continuity.jsonl').open('x',buffering=1)
    measure0=runtime.Runtime.measure
    def measure(self):
        row=measure0(self);trace.write(json.dumps(row,allow_nan=False)+'\n');return row
    runtime.Runtime.measure=measure
    def start(self,*args,**kwargs):
        result=start0(self,*args,**kwargs)
        self._pencil_session=dict(hash=None,count=0,finished=None)
        continuity.write(json.dumps(dict(kind='episode_start'))+'\n');return result
    def act(self,*args,**kwargs):
        s=self._pencil_session;began=time.monotonic();chunk=act0(self,*args,**kwargs);ended=time.monotonic()
        fingerprint=hashlib.sha256(chunk.session_id.encode()).hexdigest() if chunk.session_id else None
        ok=bool(fingerprint) and s['hash'] in (None,fingerprint)
        continuity.write(json.dumps(dict(kind='chunk',index=s['count']+1,session_sha256=fingerprint,
            continuous=ok,idle_s=None if s['finished'] is None else began-s['finished'],
            request_s=ended-began,attempts=chunk.attempts))+'\n')
        if not ok:raise RuntimeError('Dream server session changed or is missing; trial invalid')
        s.update(hash=fingerprint,count=s['count']+1,finished=ended);return chunk
    DreamSession.start,DreamSession.act=start,act
    args=['run','--embodiment','yam_arena','--policy','dream','-P','model=dreamlabs/dream-1-beta',
          '-P',f'rows_per_request={a.rows_per_request}','-E',f'task={here/"task.py"}',
          '-E','object_jitter=0','-E','control_hz=20','-E','device=cpu',
          '-E','cam_width=320','-E','cam_height=176','--instruction',task.INSTRUCTION,
          '--scorer','task_score','--max-steps',str(a.max_steps),'--seed','0','--store-frames','--log-dir',str(out)]
    registration=dict(policy='dream',model='dreamlabs/dream-1-beta',max_steps=a.max_steps,
      rows_per_request=a.rows_per_request,control_hz=20,seed=0,object_jitter=0,args=args,
      source={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(here.iterdir()) if p.is_file()},
      scope='Actual official Dream adapter. Read-only physics measurements and session continuity check.')
    (out/'registration.json').write_text(json.dumps(registration,indent=2))
    sys.argv=['inspect-robots',*args]
    from inspect_robots.cli import main as inspect_main
    try:return inspect_main()
    finally:
        trace.close();continuity.close()
        if runtime.ACTIVE:
            (out/'last-measurement.json').write_text(json.dumps(runtime.ACTIVE.latest,indent=2,allow_nan=False))
if __name__=='__main__':main()
