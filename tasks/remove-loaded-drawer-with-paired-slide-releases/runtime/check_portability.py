"""Extract verified portable inputs; forbid reads of other canonical task roots."""
from pathlib import Path
import sys,argparse,tarfile,json,hashlib,subprocess,time
O=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args();dest=a.out.resolve()
if dest.exists():raise SystemExit('fresh extraction directory required')
archive=O/'drawers-v19-final-validation.tgz';expected=json.loads((O/'archive-sha256.json').read_text())[archive.name]
if hashlib.sha256(archive.read_bytes()).hexdigest()!=expected:raise SystemExit('archive SHA mismatch')
t=time.monotonic();dest.mkdir(parents=True)
with tarfile.open(archive) as tf:
 for m in tf.getmembers():
  f=(dest/m.name).resolve()
  if not f.is_relative_to(dest) or m.issym() or m.islnk() or not m.isfile():raise ValueError('unsafe archive member')
  tf.extract(m,dest)
root=dest/'drawers-v19'
code='''import sys,os,runpy
from pathlib import Path
base=Path(sys.argv[1]).resolve()
def audit(event,args):
 if event=='open' and isinstance(args[0],(str,bytes)):
  p=Path(os.fsdecode(args[0])).resolve()
  if str(p).startswith('/mnt/bum/jjenchen/robocurve') and not p.is_relative_to(base):raise RuntimeError('old canonical source read blocked: '+str(p))
sys.addaudithook(audit)
sys.argv=[str(base/'package_checks.py')]
runpy.run_path(sys.argv[0],run_name='__main__')
'''
subprocess.run([sys.executable,'-I','-B','-c',code,str(root)],check=True)
print(json.dumps({'status':'PASS_RELOCATED_ARCHIVE_SOURCE_CHECK','canonical_reads_outside_extracted_root':'DENIED','root':str(root),'wall_s':time.monotonic()-t,'physical_evidence':False}))
