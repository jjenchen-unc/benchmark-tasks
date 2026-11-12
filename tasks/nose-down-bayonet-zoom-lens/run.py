"""Run the nose-down zoom lens task: setup image, demo replay, no-op, any action log, the scripted demo controller, or the lens probe."""
from pathlib import Path
import argparse,json,os,subprocess,sys,tempfile
HERE=Path(__file__).resolve().parent
READY=[0.,1.,1.,-.6,0.,0.,1.]
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('phase',choices=['setup','demo','noop','replay','reference','probe']);p.add_argument('--out',type=Path,required=True);p.add_argument('--actions',type=Path);p.add_argument('--lens',choices=['lens_orange','lens_blue','lens_purple']);a=p.parse_args()
 env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1')
 if a.phase=='probe':
  if a.lens is None:p.error('--lens required')
  return subprocess.run([sys.executable,'-I','-B',str(HERE/'sim/probe.py'),'--lens',a.lens,'--out',str(a.out.resolve())],cwd=HERE/'sim',env=env,check=True)
 mode={'setup':'setup','reference':'reference'}.get(a.phase,'replay')
 cmd=[sys.executable,'-I','-B',str(HERE/'sim/runtime.py'),mode,'--out',str(a.out.resolve()),'--rtx-driver-check-override']
 actions={'demo':HERE/'demo-actions.json','replay':a.actions}.get(a.phase)
 if a.phase=='noop':
  actions=Path(tempfile.mkstemp(suffix='-noop.json')[1]);actions.write_text(json.dumps([dict(control=i+1,action=READY*2) for i in range(3034)]))
 if mode=='replay':
  if actions is None:p.error('--actions required')
  cmd+=['--actions',str(Path(actions).resolve())]
 subprocess.run(cmd,cwd=HERE/'sim',env=env,check=True)
if __name__=='__main__':main()
