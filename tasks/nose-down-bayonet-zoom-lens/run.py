"""Pinned B6 review download/cache and unchanged actual entry commands; stdlib only."""
from pathlib import Path,PurePosixPath
import argparse,hashlib,json,os,shutil,subprocess,sys,tarfile,tempfile,urllib.request
ROOT=Path(__file__).resolve().parent
ASSET='B6-v30-frozen-physical-review.tgz'
URL='https://github.com/jjenchen-unc/benchmark-tasks/releases/download/b6-review-20261111/'+ASSET
BYTES=54318811
SHA256='1f942cbe68b9f10fb3ffeda3e3819223340128cca79bf4513c32846b00a34f2e'
CORE=ROOT/'b6-v30'
FINGERPRINT='f7db7f019989b30f88921430caeded1aed0df1f108061b72c0c21afd17a43994'
RECEIPT_SHA='92ea414e4c251ebb1963a6276597fd4afecda1dc8789899d658cfb55f6b2c585'
def sha(path):
 h=hashlib.sha256()
 with Path(path).open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()
def within(p,root):
 try:p.relative_to(root);return True
 except ValueError:return False
def verify_sources():
 pins=json.loads((ROOT/'published-source-sha256.json').read_text())
 for name,digest in pins.items():
  p=ROOT/name
  if p.is_symlink() or not p.is_file() or not within(p.resolve(),ROOT) or sha(p)!=digest:raise ValueError('submitted source mismatch: '+name)
 for top in ['b6-v30','envelopes-v3']:
  actual={str(p.relative_to(ROOT)) for p in (ROOT/top).rglob('*') if p.is_file() and '__pycache__' not in p.parts}
  if actual!={n for n in pins if n.startswith(top+'/')}:raise ValueError('submitted source file set mismatch: '+top)
 return pins
def prepare(cache,offline):
 verify_sources();inventory=json.loads((ROOT/'evidence/review-asset-inventory.json').read_text())
 if inventory['sha256']!=SHA256 or inventory['bytes']!=BYTES:raise ValueError('inventory archive identity mismatch')
 expected=inventory['members'];filemap={n:r for n,r in expected.items() if r['type']=='file'};cache=cache.resolve()
 if within(cache,ROOT):raise ValueError('cache must be outside submitted source')
 cache.parent.mkdir(parents=True,exist_ok=True)
 if not cache.exists():
  stage=Path(tempfile.mkdtemp(prefix=cache.name+'.partial-',dir=cache.parent))
  try:
   archive=stage/ASSET
   if offline:shutil.copyfile(offline,archive)
   else:
    request=urllib.request.Request(URL,headers={'User-Agent':'robocurve-b6-evidence/1.0'})
    with urllib.request.urlopen(request,timeout=600) as response,archive.open('xb') as out:
     size=0
     for b in iter(lambda:response.read(1024*1024),b''):
      size+=len(b)
      if size>BYTES:raise ValueError('archive exceeds pinned size')
      out.write(b)
   if archive.stat().st_size!=BYTES or sha(archive)!=SHA256:raise ValueError('archive size/SHA256 mismatch')
   seen=set();total=0
   with tarfile.open(archive,'r:gz') as tar:
    for member in tar:
     pure=PurePosixPath(member.name)
     if pure.is_absolute() or '..' in pure.parts or pure.parts[0]!='credit-review' or member.name in seen or not (member.isfile() or member.isdir()):raise ValueError('unsafe or duplicate archive member')
     actual={'type':'file' if member.isfile() else 'directory','bytes':member.size};declared=expected.get(member.name)
     if declared is None or actual!={k:declared[k] for k in ['type','bytes']}:raise ValueError('archive member inventory mismatch')
     seen.add(member.name);destination=stage/Path(*pure.parts)
     if member.isdir():destination.mkdir(parents=True,exist_ok=True);continue
     total+=member.size
     if member.size>600_000_000 or total>800_000_000:raise ValueError('archive expansion limit')
     destination.parent.mkdir(parents=True,exist_ok=True);h=hashlib.sha256();size=0
     with tar.extractfile(member) as source,destination.open('xb') as out:
      for b in iter(lambda:source.read(1024*1024),b''):h.update(b);size+=len(b);out.write(b)
     if size!=member.size or h.hexdigest()!=declared['sha256']:raise ValueError('member SHA/size mismatch')
   if seen!=set(expected):raise ValueError('archive member set mismatch')
   archive.unlink();(stage/'CACHE-VERIFIED.json').write_text(json.dumps({'sha256':SHA256,'bytes':BYTES,'files':filemap},sort_keys=True)+'\n');stage.rename(cache)
  except BaseException:shutil.rmtree(stage,ignore_errors=True);raise
 marker=json.loads((cache/'CACHE-VERIFIED.json').read_text())
 if marker!={'sha256':SHA256,'bytes':BYTES,'files':filemap}:raise ValueError('cache marker mismatch')
 actual={str(p.relative_to(cache)) for p in cache.rglob('*') if p.is_file()}
 if actual!=set(filemap)|{'CACHE-VERIFIED.json'}:raise ValueError('cache file set mismatch')
 for p in cache.rglob('*'):
  if p.is_symlink() or not within(p.resolve(),cache):raise ValueError('unsafe cached path')
 for name,row in filemap.items():
  p=cache/name
  if p.stat().st_size!=row['bytes'] or sha(p)!=row['sha256']:raise ValueError('cached file changed: '+name)
 review=cache/'credit-review/ROOT-APPROVED.json'
 if sha(review)!=RECEIPT_SHA:raise ValueError('approval receipt changed')
 return review

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('phase',choices=['prepare','setup','demo','model','verify-model','verify-demo']);p.add_argument('--cache',type=Path,required=True);p.add_argument('--archive',type=Path,help='Optional exact offline physical-review archive');p.add_argument('--out',type=Path);p.add_argument('--preflight-only',action='store_true');a=p.parse_args();review=prepare(a.cache,a.archive)
 command=[sys.executable,'-I','-B','-c',"import sys;sys.path.insert(0,sys.argv[1]);from protocol import check,receipt;assert check()['fingerprint']==sys.argv[3];receipt(sys.argv[2]);print('B6 source and nine-channel receipt PASS')",str(CORE),str(review),FINGERPRINT]
 subprocess.run(command,cwd=CORE,check=True)
 if a.phase=='prepare' or a.preflight_only:return
 if a.out is None or not a.out.is_absolute() or within(a.out.resolve(),ROOT) or within(a.out.resolve(),a.cache.resolve()):p.error('--out must be absolute and outside source/cache')
 if a.phase not in ['verify-model','verify-demo'] and a.out.exists():p.error('--out must be fresh')
 E=ROOT/'envelopes-v3'
 if a.phase=='setup':command=[sys.executable,'-I','-B',str(CORE/'runtime.py'),'setup','--out',str(a.out),'--rtx-driver-check-override']
 elif a.phase=='demo':command=[sys.executable,'-I','-B',str(CORE/'runtime.py'),'reference','--out',str(a.out),'--rtx-driver-check-override']
 elif a.phase=='model':command=[sys.executable,'-I','-B',str(E/'b6_model_same_session_gripper_v3.py'),'--out',str(a.out),'--review',str(review)]
 elif a.phase=='verify-model':command=[sys.executable,'-I','-B',str(E/'verify_b6_actual_model_v3.py'),'--bundle',str(CORE),'--out',str(a.out),'--review',str(review),'--encode','--model-helper',str(E/'b6_model_same_session_gripper_v3.py')]
 else:command=[sys.executable,'-I','-B',str(E/'verify_demo.py'),'--bundle',str(CORE),'--out',str(a.out),'--review',str(review)]
 if any('\n' in v or '\r' in v for v in command):raise ValueError('OS argv must contain no line breaks')
 env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1',B6_FROZEN_BUNDLE=str(CORE),B6_ROOT_REVIEW=str(review),B6_TRACE=str(a.out/'physics.jsonl'))
 print(json.dumps({'phase':a.phase,'review_asset_sha256':SHA256,'core_fingerprint':FINGERPRINT,'public_model_conversion':'Dream-1 + public gripper saturation [0,1] on all returned rows'}),flush=True);subprocess.run(command,cwd=CORE,env=env,check=True)
 if a.phase=='demo':subprocess.run([sys.executable,'-I','-B',str(E/'verify_demo.py'),'--bundle',str(CORE),'--out',str(a.out),'--review',str(review)],cwd=CORE,env=env,check=True)
if __name__=='__main__':main()
