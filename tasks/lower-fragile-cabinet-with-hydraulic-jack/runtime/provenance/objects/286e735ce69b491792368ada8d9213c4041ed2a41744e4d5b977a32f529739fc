from pathlib import Path
import hashlib,json
R=Path(__file__).resolve().parent;C=R/'candidate'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def put(p,x):p.write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
put(C/'sourcehashes.json',{str(p.relative_to(C)):sha(p) for p in sorted(C.rglob('*')) if p.is_file() and p.name!='sourcehashes.json'})
m={str(p.relative_to(C)):sha(p) for p in sorted(C.rglob('*')) if p.is_file()};put(R/'candidate-SHA256.json',m)
put(R/'fingerprint.json',{'sha256':hashlib.sha256(json.dumps(m,sort_keys=True,separators=(',',':')).encode()).hexdigest(),'native_copy_byte_exact':False,'changes':['mount.py: upright opposite-side right base on steel plate','kit.py: robot initial transform only','task.py: fixed physical mount plate only','reference.py: opposing jaw branch, raised TCP, bounded command subdivision'],'physics_changed':False})
