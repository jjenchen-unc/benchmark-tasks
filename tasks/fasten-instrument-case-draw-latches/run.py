"""Verified external evidence cache and exact frozen entry commands. Stdlib only."""
from pathlib import Path
import argparse,hashlib,json,os,shutil,subprocess,sys,tarfile,tempfile,urllib.request
ROOT=Path(__file__).resolve().parent
ASSET='S2-v30-frozen-reviewed-inputs.tgz'
URL='https://github.com/jjenchen-unc/benchmark-tasks/releases/download/s2-review-20261111/'+ASSET
BYTES=208633430
SHA256='9bb80c8153e4b10149b27f0fd521a3e6132cb8f24c991a859757ef41b5f53b42'
REVIEW='ROOT-S2-v30-physical-review.json'
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
  cached=payload/('s2-v30/'+relative[len('runtime/'):] if relative.startswith('runtime/') else 'model-boundary-review/'+relative[len('reproduction/original_'):] if relative.startswith('reproduction/') else 's2-v30/'+relative)
  if cached.is_symlink() or sha(cached)!=digest:raise ValueError('cached runtime differs from submitted source: '+relative)
 candidate_files={str(p.relative_to(ROOT)) for p in (ROOT/'candidate').rglob('*') if p.is_file() and '__pycache__' not in p.parts}
 if candidate_files!={n for n in pins if n.startswith('candidate/')}:raise ValueError('submitted candidate file set differs')
 overlay=json.loads((ROOT/'model-envelope-sha256.json').read_text())
 if set(overlay)!={'reproduction/model_entry.py','reproduction/verify_model.py','reproduction/gripper_saturation.py'}:raise ValueError('model envelope file set changed')
 for name,digest in overlay.items():
  if (ROOT/name).is_symlink() or sha(ROOT/name)!=digest:raise ValueError('public model envelope changed: '+name)
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
    request=urllib.request.Request(URL,headers={'User-Agent':'robocurve-s2-evidence/1.0'})
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
     if p.parts[0] not in {'s2-v30','full-replay','ROOT-S2-v30-physical-review.json','review-evidence','model-boundary-review','CACHE-MANIFEST.json'}:raise ValueError('unexpected archive root')
     total+=member.size
     if member.size>256_000_000 or total>512_000_000:raise ValueError('archive expansion exceeds bounds')
     seen.add(member.name);data=t.extractfile(member).read()
     if len(data)!=member.size:raise ValueError('truncated archive member')
     rows[member.name]=hashlib.sha256(data).hexdigest()
     destination=stage/p;destination.parent.mkdir(parents=True,exist_ok=True)
     with destination.open('xb') as out:out.write(data)
     if member.name=='CACHE-MANIFEST.json':backup=json.loads(data)
   if backup is None or rows!=expected_members or {k:v for k,v in rows.items() if k!='CACHE-MANIFEST.json'}!={n:row['sha256'] for n,row in backup['files'].items()}:raise ValueError('archive member SHA manifest mismatch')
   a.unlink();verify_sources(stage)
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
 payload=cache;verify_sources(payload)
 return payload

def default_cache():
 return Path(os.environ.get('S2_EVIDENCE_CACHE',str(Path.home()/'.cache/robocurve-s2-v30')))
def preflight(payload):
 core=payload/'s2-v30';sys.path.insert(0,str(core));sys.path.insert(0,str(core/'candidate'))
 from protocol import receipt,check
 review=payload/REVIEW;r=receipt(review);os.environ['S2_ROOT_REVIEW']=str(review)
 from yam_arena.task_file import load
 task=load(core/'model_task.py')
 return {'source_fingerprint':check()['fingerprint'],'channels_approved':sorted(r['channels']),'objects':len(task.objects),'instruction':task.docs,'simulation_starts':0,'model_calls':0}
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('phase',choices=['prepare','setup','replay','model','verify-model']);p.add_argument('--cache',type=Path,default=default_cache());p.add_argument('--archive',type=Path);p.add_argument('--out',type=Path);p.add_argument('--preflight-only',action='store_true');a=p.parse_args()
 payload=prepare(a.cache,a.archive)
 if a.phase=='prepare' or a.preflight_only:print(json.dumps(preflight(payload)));return
 if a.out is None or not a.out.is_absolute() or a.out.resolve().is_relative_to(a.cache.resolve()) or a.out.resolve().is_relative_to(ROOT):p.error('--out must be a fresh absolute path outside source/cache')
 core=payload/'s2-v30';review=payload/REVIEW
 if a.phase in ['setup','replay']:command=[sys.executable,'-I','-B',str(core/'runtime.py'),a.phase,'--out',str(a.out),'--legacy-rtx-override']
 elif a.phase=='model':command=[sys.executable,'-I','-B',str(ROOT/'reproduction/model_entry.py'),'--review',str(review),'--out',str(a.out),'--legacy-rtx-override']
 else:command=[sys.executable,'-I','-B',str(ROOT/'reproduction/verify_model.py'),str(a.out),'--review',str(review)]
 env=dict(os.environ,OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',NUMEXPR_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1');env.pop('S2_EVALUATOR_DIAGNOSTICS',None);env['S2_REVIEWED_SOURCE']=str(core)
 print(json.dumps({'phase':a.phase,'evidence_archive_sha256':SHA256,'submitted_sources_match_cached_payload':True,'command':command}),flush=True)
 subprocess.run(command,cwd=core,env=env,check=True)
if __name__=='__main__':main()
