"""Pure, deterministic XML primitive export. Exact native geometry executes locally."""
import json,math,hashlib,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from source_geometry import N,xml,HERE

def vector(s):return np.array([float(v) for v in s.split()])
def geometry(g):
 d=dict(g.attrib);p=vector(d.get('pos','0 0 0'));q=vector(d.get('quat','1 0 0 0'));size=vector(d['size'])
 if 'fromto' in d:
  pts=vector(d['fromto']).reshape(2,3);v=pts[1]-pts[0];L=np.linalg.norm(v);p=pts.mean(0);axis=v/L
  if axis[2]<-1+1e-8:q=np.array([0.,1.,0.,0.])
  else:q=np.r_[1+axis[2],-axis[1],axis[0],0.];q=q/np.linalg.norm(q)
  size=np.r_[size[0],L/2]
 return dict(name=d['name'],type=d['type'],pos=p.tolist(),quat=q.tolist(),size=size.tolist(),rgba=vector(d.get('rgba','.6 .6 .6 1')).tolist(),collision=bool(int(d.get('contype','1')) or int(d.get('conaffinity','1'))),friction=float(d.get('friction','.8').split()[0]),native=d)
def compile_scene():
 w,p=xml();(HERE/'fixture.xml').write_text(ET.tostring(w,encoding='unicode')+'\n');bodies=[]
 def body(b,parent=None):
  info=dict(name=b.get('name'),parent=parent,pos=vector(b.get('pos','0 0 0')).tolist(),geoms=[geometry(g) for g in b.findall('geom')],joint=dict(b.find('joint').attrib) if b.find('joint') is not None else None,inertial=dict(b.find('inertial').attrib) if b.find('inertial') is not None else None)
  bodies.append(info)
  for sub in b.findall('body'):body(sub,info['name'])
 for b in w.findall('body'):body(b)
 data=dict(seed=42,target='remove_C',parameters=p,bodies=bodies,native_source_sha256=hashlib.sha256((HERE/'native_source.py').read_bytes()).hexdigest(),translation=[.23,.14,-.0525],rotation_wxyz=[1,0,0,0],robot_base_z=.90)
 (HERE/'fixture.json').write_text(json.dumps(data,indent=2)+'\n');return data
if __name__=='__main__':print(json.dumps({'bodies':len(compile_scene()['bodies'])}))
