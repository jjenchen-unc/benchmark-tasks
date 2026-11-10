"""Portable path binding around the exact evaluated native bundle and model helper."""
from pathlib import Path
import argparse,hashlib,json,os,sys,tarfile,subprocess,runpy,tempfile,urllib.request,shutil
sys.dont_write_bytecode=True
P=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('phase',choices=['prepare','setup','replay','model']);p.add_argument('--cache',type=Path,required=True);p.add_argument('--out',type=Path);p.add_argument('--evidence-archive',type=Path);p.add_argument('--preflight-only',action='store_true');a=p.parse_args();C=a.cache.resolve();S=P/'runtime'
if C==P or C.is_relative_to(P):raise ValueError('Cache must be outside the submission directory')
m=json.loads((P/'publication-manifest.json').read_text());expected=m['review_evidence_archive_sha256']
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
if not C.exists():
 C.parent.mkdir(parents=True,exist_ok=True)
 archive=a.evidence_archive.resolve() if a.evidence_archive else C.parent/(expected+'.tar.gz')
 if not archive.exists():
  if a.evidence_archive:raise FileNotFoundError('Provided review evidence archive does not exist')
  url=m['review_evidence_url']
  if url!='https://github.com/jjenchen-unc/benchmark-tasks/releases/download/hydraulic-review-20261110/review-evidence.tar.gz':raise ValueError('Unexpected review evidence URL')
  with tempfile.NamedTemporaryFile(dir=C.parent,prefix='review-download-',delete=False) as f:
   tmp=Path(f.name);h=hashlib.sha256();size=0
   try:
    with urllib.request.urlopen(url,timeout=90) as response:
     while True:
      block=response.read(1024*1024)
      if not block:break
      size+=len(block)
      if size>m['review_evidence_bytes']:raise ValueError('Review download exceeds declared size')
      h.update(block);f.write(block)
    if size!=m['review_evidence_bytes'] or h.hexdigest()!=expected:raise ValueError('Review download size/SHA mismatch')
   except BaseException:tmp.unlink(missing_ok=True);raise
  tmp.replace(archive)
 if archive.stat().st_size!=m['review_evidence_bytes'] or sha(archive)!=expected:raise ValueError('Review archive size/SHA changed')
 inventory={x['path']:x for x in m['review_evidence_members']};names=set()
 stage=Path(tempfile.mkdtemp(dir=C.parent,prefix='review-extract-'))
 try:
  with tarfile.open(archive,'r:gz') as t:
   for x in t.getmembers():
    if x.name in names or x.name not in inventory or x.name.startswith('/') or '..' in Path(x.name).parts or '\\' in x.name or not (x.isfile() or x.isdir()):raise ValueError('Unexpected/unsafe evidence member')
    names.add(x.name);declared=inventory[x.name]
    if x.isfile():
     if declared['type']!='file' or x.size!=declared['bytes']:raise ValueError('Evidence member size/type changed')
     h=hashlib.sha256()
     with t.extractfile(x) as f:
      for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
     if h.hexdigest()!=declared['sha256']:raise ValueError('Evidence member SHA changed')
    elif declared['type']!='directory':raise ValueError('Evidence directory type changed')
   if names!=set(inventory):raise ValueError('Evidence inventory incomplete')
   t.extractall(stage,filter='data')
  (stage/'.archive-sha256').write_text(expected+'\n');stage.rename(C)
 except BaseException:
  import shutil;shutil.rmtree(stage);raise
elif not (C/'.archive-sha256').is_file() or (C/'.archive-sha256').read_text().strip()!=expected:raise ValueError('Use a fresh cache or one extracted from the exact reviewed archive')
# Restore the exact evaluated native tree outside the source checkout. Historical
# provenance is packed losslessly to keep the public source change manageable.
packed=m['packed_native_provenance'];bundle=C/'native-runtime'
expected_native=m['executed_native_file_sha256']
def verify_native(folder):
 actual={str(f.relative_to(folder)):sha(f) for f in folder.rglob('*') if f.is_file() and '__pycache__' not in f.parts}
 if actual!=expected_native:raise ValueError('Restored native source differs from evaluated207-file tree')
if not bundle.exists():
 archive=P/packed['archive']
 if archive.stat().st_size!=packed['bytes'] or sha(archive)!=packed['sha256']:raise ValueError('Packed source archive changed')
 inventory={x['path']:x for x in packed['members']};seen=set()
 stage=Path(tempfile.mkdtemp(dir=C,prefix='native-source-'))
 try:
  for item in S.rglob('*'):
   rel=item.relative_to(S)
   if item.is_symlink():raise ValueError('Published runtime source must not contain symlinks')
   if item.is_file():
    dst=stage/rel;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(item,dst)
  with tarfile.open(archive,'r:gz') as t:
   for member in t.getmembers():
    n=member.name
    if not member.isfile() or n not in inventory or n in seen or Path(n).is_absolute() or '..' in Path(n).parts or '\\' in n:raise ValueError('Unsafe packed native source member')
    declared=inventory[n];seen.add(n)
    if member.size!=declared['bytes']:raise ValueError('Packed native source size changed')
    content=t.extractfile(member).read()
    if hashlib.sha256(content).hexdigest()!=declared['sha256']:raise ValueError('Packed native source member SHA changed')
    dst=stage/n;dst.parent.mkdir(parents=True,exist_ok=True)
    if dst.exists():raise ValueError('Packed native source duplicates a published file')
    dst.write_bytes(content)
   if seen!=set(inventory):raise ValueError('Packed native source incomplete')
  verify_native(stage);stage.rename(bundle)
 except BaseException:shutil.rmtree(stage);raise
verify_native(bundle);S=bundle
sys.path[:0]=[str(S),str(S/'candidate')]
from protocol import receipt,check
review=C/'ROOT-physical-review.json';receipt(review)
if check()!=m['native_bundle_fingerprint']:raise ValueError('Native task changed')
print(json.dumps({'native_bundle':str(S),'review':str(review),'native_bundle_fingerprint':check(),'phase':a.phase,'preflight_only':a.preflight_only}),flush=True)
if a.phase=='prepare' or a.preflight_only:raise SystemExit(0)
if a.out is None:raise ValueError('--out is required for an actual phase')
O=a.out.resolve()
if O.exists() or O.is_relative_to(P):raise ValueError('Output must be fresh and outside the submission directory')
os.environ.update(HYDRAULIC_NATIVE_BUNDLE=str(S),HYDRAULIC_ROOT_REVIEW=str(review),OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1')
if a.phase=='model':
 cmd=[sys.executable,'-B',str(P/'execution/run_model_transport.py'),'--bundle',str(S),'--workflow',str(P/'execution/workflow'),'--review',str(review),'--out',str(O)]
 raise SystemExit(subprocess.run(cmd,env=os.environ).returncode)
# This wrapper only binds paths; the unchanged production adapter supplies credit.
# Actual published replay execution used public_replay.py, shipped separately.
if a.phase=='replay':
 from yam_arena.task_file import load
 loaded=load(S/'model_task.py')
 if len(loaded.objects)!=14:raise ValueError('Task object inventory changed')
sys.argv=[str(S/'runtime.py'),a.phase,'--out',str(O)]
runpy.run_path(str(S/'runtime.py'),run_name='__main__')
