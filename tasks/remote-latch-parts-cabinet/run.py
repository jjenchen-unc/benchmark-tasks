"""Pinned S5 review download/cache and unchanged actual entry commands; stdlib only."""
from pathlib import Path,PurePosixPath
import argparse,hashlib,json,os,shutil,subprocess,sys,tarfile,tempfile,urllib.request
ROOT=Path(__file__).resolve().parent
ASSET='S5-v13-frozen-physical-review.tgz'
URL='https://github.com/jjenchen-unc/benchmark-tasks/releases/download/s5-review-20261110/'+ASSET
BYTES=50050668
SHA256='36da215d9317404b93a513dc04398555020fcd89cde334e82b02ea403f48087c'
CORE=ROOT/'s5-static-table-portable-v13'
FINGERPRINT=json.loads((CORE/'manifest.json').read_text())['fingerprint']
RECEIPT_SHA='f18b132c9ba098c4c437580ed4f7c481bdf4b05cd233bd26fe0a45618b9442ad'
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
 for top in ['s5-static-table-portable-v13','s5-v13-credited-envelope']:
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
    request=urllib.request.Request(URL,headers={'User-Agent':'robocurve-s5-evidence/1.0'})
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
     if pure.is_absolute() or '..' in pure.parts or pure.parts[0]!='physical-review' or member.name in seen or not (member.isfile() or member.isdir()):raise ValueError('unsafe or duplicate archive member')
     actual={'type':'file' if member.isfile() else 'directory','bytes':member.size};declared=expected.get(member.name)
     if declared is None or actual!={k:declared[k] for k in ['type','bytes']}:raise ValueError('archive member inventory mismatch')
     seen.add(member.name);destination=stage/Path(*pure.parts)
     if member.isdir():destination.mkdir(parents=True,exist_ok=True);continue
     total+=member.size
     if member.size>100_000_000 or total>300_000_000:raise ValueError('archive expansion limit')
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
 review=cache/'physical-review/ROOT-REVIEW.json'
 if sha(review)!=RECEIPT_SHA:raise ValueError('approval receipt changed')
 return review

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('phase',choices=['prepare','setup','demo','model']);p.add_argument('--cache',type=Path,required=True);p.add_argument('--archive',type=Path,help='Optional exact offline review archive');p.add_argument('--out',type=Path);p.add_argument('--preflight-only',action='store_true');a=p.parse_args();review=prepare(a.cache,a.archive)
 # Actual pinned receipt gate before App/model imports, using only immutable source.
 command=[sys.executable,'-I','-B','-c',"import sys;sys.path.insert(0,sys.argv[1]);from protocol import check,receipt;assert check()['fingerprint']==sys.argv[3];receipt(sys.argv[2]);print('S5 source and ROOT receipt PASS')",str(CORE),str(review),FINGERPRINT]
 subprocess.run(command,cwd=CORE,check=True)
 if a.phase=='prepare' or a.preflight_only:return
 if a.out is None or not a.out.is_absolute() or within(a.out.resolve(),ROOT) or within(a.out.resolve(),a.cache.resolve()):p.error('--out must be an absolute path outside source/cache')
 if a.out.exists():p.error('--out must be fresh')
 if a.phase=='setup':command=[sys.executable,'-I','-B',str(CORE/'run_phase.py'),'setup','--out',str(a.out)]
 elif a.phase=='demo':command=[sys.executable,'-I','-B',str(ROOT/'s5-v13-credited-envelope/credited_demo.py'),'demo','--review',str(review),'--out',str(a.out)]
 elif a.phase=='model':command=[sys.executable,'-I','-B',str(CORE/'run_phase.py'),'model','--review',str(review),'--out',str(a.out)]
 if any('\n' in v or '\r' in v for v in command):raise ValueError('OS argv must contain no line breaks')
 env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1')
 print(json.dumps({'phase':a.phase,'review_archive_sha256':SHA256,'core_fingerprint':FINGERPRINT,'unchanged_entry':str(command[3])}),flush=True);subprocess.run(command,cwd=CORE,env=env,check=True)
if __name__=='__main__':main()
