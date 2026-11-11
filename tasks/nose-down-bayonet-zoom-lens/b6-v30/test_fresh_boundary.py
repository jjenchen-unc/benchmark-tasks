"""CPU diagnostic through real fresh parsing, using actual43967 typed rows."""
from pathlib import Path
import sys,json,copy,time
O=Path(__file__).resolve().parent;sys.path.insert(0,str(O));sys.path.insert(0,str(O/'candidate'))
import protocol,mechanics
start=time.monotonic();manifest=protocol.check()
rows=[protocol.loads(s) for s in (O/'provenance/actual43967-first25.jsonl').read_text().splitlines()]
folder=O/'checks/fresh-boundary';folder.mkdir(exist_ok=True)
(folder/'result.json').write_text(json.dumps(dict(kind='ACTUAL_B6_PORTABLE',phase='reference',source_before=manifest,source_after=manifest,completed=True,error=None,close_error=None)))
original=mechanics.History.update;calls=[]
def observed(self,row,dt):
 calls.append(copy.deepcopy(row));return original(self,row,dt)
mechanics.History.update=observed
results=[]
def run(name,mutate=None):
 data=copy.deepcopy(rows);calls.clear()
 if mutate:mutate(data[0])
 (folder/'physics.jsonl').write_text(''.join(json.dumps(r,allow_nan=False)+'\n' for r in data))
 try:protocol.fresh(folder,'reference')
 except (ValueError,FileNotFoundError) as e:
  if mutate:
   assert isinstance(e,ValueError) and not calls,(name,str(e),len(calls))
  else:
   # All 25 measured rows passed validation AND native status comparison.
   assert type(e) is FileNotFoundError and Path(e.filename).name=='actions.json' and len(calls)==25,(str(e),len(calls))
  results.append(dict(name=name,passed=True,native_updates=len(calls),boundary=str(e)))
 else:raise AssertionError('partial diagnostic claimed whole success')
run('actual typed first25 accepted at parsing boundary')
def setfield(k,v):return lambda r:r['row'].__setitem__(k,v)
for k,v in [('wrist_n',['1',False]),('wrist_n',[1,False]),('wrist_n',[1]),('wrist_n',{'0':1,'1':2}),('stop_nm',{'body_left':False,'body_right':0}),('stop_nm',{'body_left':'0','body_right':0}),('drop_velocity_z',dict.fromkeys(['lens_orange','lens_blue','lens_purple'],'0')),('held',dict.fromkeys(['lens_orange','lens_blue','lens_purple'],0)),('captured',dict.fromkeys(['lens_orange','lens_blue','lens_purple'],1)),('robot_touch',0),('stop_nm',{}),('held',[]),('measurement',{'lens_on_mount':'lens_orange'}),('push_n',{}),('mounts',{})]:run(k+' '+repr(v),setfield(k,v))
run('missing wrist',lambda r:r['row'].pop('wrist_n'))
run('extra row field',lambda r:r['row'].__setitem__('extra',0))
run('boolean trace control',lambda r:r.__setitem__('control',False))
run('string trace time',lambda r:r.__setitem__('sim_time','0.502'))
mechanics.History.update=original
consumed={str(Path(m.__file__).resolve()):protocol.sha(Path(m.__file__)) for m in list(sys.modules.values()) if getattr(m,'__file__',None) and Path(m.__file__).is_file()}
r=dict(consumed_source_sha256=consumed,passed=True,synthetic=True,physical_proof=False,actual_excerpt_sha256=protocol.sha(O/'provenance/actual43967-first25.jsonl'),tests=results,wall_s=time.monotonic()-start)
(O/'checks/fresh-boundary.json').write_text(json.dumps(r,indent=2,allow_nan=False)+'\n')
print(json.dumps(dict(passed=True,tests=len(results),wall_s=r['wall_s'])))
