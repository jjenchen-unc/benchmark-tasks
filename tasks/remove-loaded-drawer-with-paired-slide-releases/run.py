"""Verified external evidence cache and exact frozen entry commands. Stdlib only."""
from pathlib import Path
import argparse,hashlib,json,os,shutil,subprocess,sys,tarfile,tempfile,urllib.request
ROOT=Path(__file__).resolve().parent
ASSET='Drawers-v21-frozen-model-inputs-and-zero-setup.tgz'
URL='https://github.com/jjenchen-unc/benchmark-tasks/releases/download/drawers-review-20261110/'+ASSET
BYTES=30856983
SHA256='f5c473b9aafb4a5c754a245cfc9812b10680a56392d3ed81931832b2ae770e19'
REVIEW='../work/review/ROOT-general-pad-APPROVED.json'
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
 return h.hexdigest()
def verified_archive(p):
 if p.stat().st_size!=BYTES or sha(p)!=SHA256:raise ValueError('immutable review archive size/SHA256 mismatch')
def verify_sources(payload):
 pins=json.loads((ROOT/'published-source-sha256.json').read_text())
 for relative,digest in pins.items():
  public=ROOT/relative
  if public.is_symlink() or sha(public)!=digest:raise ValueError('submitted source changed: '+relative)
  cached=payload/(relative[len('runtime/'):] if relative.startswith('runtime/') else relative)
  if cached.is_symlink() or sha(cached)!=digest:raise ValueError('cached runtime differs from submitted source: '+relative)
 candidate_files={str(p.relative_to(ROOT)) for p in (ROOT/'candidate').rglob('*') if p.is_file() and '__pycache__' not in p.parts}
 if candidate_files!={n for n in pins if n.startswith('candidate/')}:raise ValueError('submitted candidate file set differs')
 return pins
def prepare(cache,offline):
 inventory=json.loads((ROOT/'evidence/verification-cache-inventory.json').read_text())
 if inventory['sha256']!=SHA256 or inventory['bytes']!=BYTES:raise ValueError('published cache inventory identity mismatch')
 expected_members={n:row['sha256'] for n,row in inventory['members'].items()}
 cache=cache.resolve()
 if cache.is_relative_to(ROOT):raise ValueError('cache must be outside the submitted task directory')
 cache.parent.mkdir(parents=True,exist_ok=True)
 if not cache.exists():
  stage=Path(tempfile.mkdtemp(prefix=cache.name+'.partial-',dir=cache.parent))
  try:
   a=stage/ASSET
   if offline:shutil.copyfile(offline,a)
   else:
    request=urllib.request.Request(URL,headers={'User-Agent':'robocurve-drawers-evidence/1.0'})
    with urllib.request.urlopen(request,timeout=600) as response,a.open('xb') as out:
     count=0
     for chunk in iter(lambda:response.read(1024*1024),b''):
      count+=len(chunk)
      if count>BYTES:raise ValueError('archive response exceeds pinned size')
      out.write(chunk)
   verified_archive(a);seen=set();rows={};total=0;backup=None
   with tarfile.open(a,'r:gz') as t:
    for member in t:
     p=Path(member.name)
     if not member.isfile() or p.is_absolute() or '..' in p.parts or member.name in seen:raise ValueError('unsafe/duplicate archive member')
     if p.parts[0] not in {'drawers-v21','zero-action-setup','BACKUP-MANIFEST.json'}:raise ValueError('unexpected archive root')
     total+=member.size
     if member.size>100_000_000 or total>150_000_000:raise ValueError('archive expansion exceeds bounds')
     seen.add(member.name);data=t.extractfile(member).read()
     if len(data)!=member.size:raise ValueError('truncated archive member')
     rows[member.name]=hashlib.sha256(data).hexdigest()
     destination=stage/p;destination.parent.mkdir(parents=True,exist_ok=True)
     with destination.open('xb') as out:out.write(data)
     if member.name=='BACKUP-MANIFEST.json':backup=json.loads(data)
   if backup is None or rows!=expected_members or {k:v for k,v in rows.items() if k!='BACKUP-MANIFEST.json'}!=backup['files_sha256']:raise ValueError('archive member SHA manifest mismatch')
   a.unlink();verify_sources(stage/'drawers-v21')
   (stage/'CACHE-VERIFIED.json').write_text(json.dumps({'archive':ASSET,'bytes':BYTES,'sha256':SHA256,'member_sha256':rows},indent=2)+'\n')
   stage.rename(cache)
  except BaseException:
   shutil.rmtree(stage,ignore_errors=True);raise
 receipt=json.loads((cache/'CACHE-VERIFIED.json').read_text())
 if receipt.get('sha256')!=SHA256 or receipt.get('bytes')!=BYTES or receipt.get('member_sha256')!=expected_members:raise ValueError('cache identity/member map mismatch')
 actual_files={str(p.relative_to(cache)) for p in cache.rglob('*') if p.is_file()}
 if actual_files!=set(expected_members)|{'CACHE-VERIFIED.json'}:raise ValueError('cache file set changed')
 for name,digest in receipt['member_sha256'].items():
  p=cache/name
  if p.is_symlink() or not p.resolve().is_relative_to(cache) or sha(p)!=digest:raise ValueError('cached evidence changed: '+name)
 payload=cache/'drawers-v21';verify_sources(payload)
 return payload
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('phase',choices=['prepare','setup','replay','model','fresh-physical']);p.add_argument('--cache',type=Path,required=True);p.add_argument('--archive',type=Path,help='Optional offline copy of the exact pinned archive');p.add_argument('--out',type=Path);p.add_argument('--preflight-only',action='store_true');a=p.parse_args()
 payload=prepare(a.cache,a.archive)
 if a.phase=='prepare':command=[sys.executable,'-I','-B',str(payload/'package_checks.py')]
 else:
  if a.out is None or not a.out.is_absolute() or a.out.resolve().is_relative_to(a.cache.resolve()) or a.out.resolve().is_relative_to(ROOT):p.error('--out must be a fresh absolute path outside source/cache')
  if a.phase in ['setup','replay']:
   if a.preflight_only:command=[sys.executable,'-I','-B',str(payload/'package_checks.py')]
   else:command=[sys.executable,'-I','-B',str(payload/'run_public.py'),'--mode','setup' if a.phase=='setup' else 'demo','--review',REVIEW,'--out',str(a.out)]
  else:
   command=[sys.executable,'-I','-B',str(payload/'run_phase.py'),'model' if a.phase=='model' else 'replay','--out',str(a.out)]
   if a.phase=='model':command+=['--review',REVIEW]
   if a.preflight_only:command+=['--preflight-only']
 env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1')
 print(json.dumps({'phase':a.phase,'evidence_archive_sha256':SHA256,'submitted_sources_match_cached_payload':True,'command':command,'model_score_claimed':None}),flush=True)
 subprocess.run(command,cwd=payload,env=env,check=True)
if __name__=='__main__':main()
