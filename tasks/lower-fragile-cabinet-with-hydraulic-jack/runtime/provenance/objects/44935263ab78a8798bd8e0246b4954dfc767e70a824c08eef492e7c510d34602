"""Offline geometry compiler. Executes the immutable native source, no mj_step/build/reference.
Native helpers are authoring inputs only; Arena runtime consumes fixture.json/USD.
"""
import argparse,hashlib,importlib.util,json,math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco

def extract(source,out):
 spec=importlib.util.spec_from_file_location('hydraulic_frozen',source);s=importlib.util.module_from_spec(spec);spec.loader.exec_module(s)
 p=s.parameters(42)
 root=ET.Element('mujoco',model='hydraulic_only');ET.SubElement(root,'compiler',angle='radian',autolimits='true');root.append(ET.fromstring(s.portable_scene(42,'test')[0]).find('default'));asset=ET.SubElement(root,'asset');w=ET.SubElement(root,'worldbody');eq=ET.SubElement(root,'equality')
 colors={'steel':'.55 .57 .6 1','dark':'.12 .12 .13 1','walnut':'.36 .22 .13 1','crate':'.62 .47 .30 1','glass':'.75 .88 .95 .25','red_paint':'.72 .08 .07 1','blue_paint':'.12 .27 .62 1','rail':'.40 .42 .45 1'}
 for n,c in colors.items():ET.SubElement(asset,'material',name=n,rgba=c)
 # Same source rail geometry including supports and anti-slide lips.
 for x in s.RAIL_X:
  s.sub(w,'geom',name='rail_'+('b' if x>s.JX else 'a'),type='box',pos=(x,s.RAIL_Y0,s.RAIL_TOP-.015),size=(.015,s.RAIL_HALF_Y,.015),material='rail',friction='.35 .01 .001')
  s.sub(w,'geom',name='lip_'+('b' if x>s.JX else 'a'),type='box',pos=(x+np.sign(x-s.JX)*.0195,s.RAIL_Y0,s.RAIL_TOP+.006),size=(.0025,s.RAIL_HALF_Y,.006),material='rail',friction='.35 .01 .001')
  for yy in (s.RAIL_Y0-s.RAIL_HALF_Y+.02,.30,s.RAIL_Y0+s.RAIL_HALF_Y-.02):s.sub(w,'geom',type='box',pos=(x,yy,(.8+s.RAIL_TOP-.03)/2),size=(.012,.012,(s.RAIL_TOP-.03-.8)/2),material='rail')
 for j in ('S','T'):s._jack(w,eq,j,p)
 s._load(w,'cabinet','S',p,.050)
 # Compilation computes mass/inertia and capsule fromto frames; it never advances dynamics.
 m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
 types={int(mujoco.mjtGeom.mjGEOM_BOX):'box',int(mujoco.mjtGeom.mjGEOM_CYLINDER):'cylinder',int(mujoco.mjtGeom.mjGEOM_CAPSULE):'capsule'}
 bodies=[];positions={0:np.zeros(3)}
 for b in range(m.nbody):
  n=m.body(b).name or 'rails';parent=int(m.body_parentid[b]);positions[b]=positions.get(parent,np.zeros(3))+m.body_pos[b]
  geoms=[]
  for gi in range(m.body_geomadr[b],m.body_geomadr[b]+m.body_geomnum[b]):
   geoms.append(dict(name=m.geom(gi).name or 'shape_'+str(gi),kind=types[int(m.geom_type[gi])],pos=m.geom_pos[gi].tolist(),quat_wxyz=m.geom_quat[gi].tolist(),size=m.geom_size[gi].tolist(),rgba=m.geom_rgba[gi].tolist(),friction=m.geom_friction[gi].tolist(),collision=bool(m.geom_contype[gi] or m.geom_conaffinity[gi])))
  joints=[]
  for ji in range(m.body_jntadr[b],m.body_jntadr[b]+m.body_jntnum[b]):
   if int(m.jnt_type[ji])==int(mujoco.mjtJoint.mjJNT_FREE):continue
   di=int(m.jnt_dofadr[ji]);qi=int(m.jnt_qposadr[ji]);joints.append(dict(name=m.joint(ji).name,kind='slide' if int(m.jnt_type[ji])==int(mujoco.mjtJoint.mjJNT_SLIDE) else 'hinge',axis_local=m.jnt_axis[ji].tolist(),anchor_local=m.jnt_pos[ji].tolist(),range=m.jnt_range[ji].tolist() if m.jnt_limited[ji] else None,stiffness=float(m.jnt_stiffness[ji]),springref=float(m.qpos_spring[qi]),damping=float(m.dof_damping[di]),frictionloss=float(m.dof_frictionloss[di]),armature=float(m.dof_armature[di])))
  bodies.append(dict(name=n,parent=m.body(parent).name if b else None,pos=m.body_pos[b].tolist(),world_pos=positions[b].tolist(),quat_wxyz=m.body_quat[b].tolist(),mass=float(m.body_mass[b]),com=m.body_ipos[b].tolist(),inertia=m.body_inertia[b].tolist(),inertia_quat_wxyz=m.body_iquat[b].tolist(),joints=joints,geoms=geoms))
 out.write_text(json.dumps(dict(schema=1,target='J2_lower_cabinet',source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),seed=42,parameters=p,native_origin=[.05,.15,.8],arena_origin=[.17,-.15,.7475],translation=[.12,-.3,-.0525],bodies=bodies,initial_joints={'ram_S':.055,'ram_T':0.,'lever_S':.3,'lever_T':.3},compile_only=True),indent=2)+'\n')
 print(json.dumps({'compiled_bodies':len(bodies),'geoms':sum(len(b['geoms']) for b in bodies),'physics_steps':0,'source_executed':str(source)}))
if __name__=='__main__':
 a=argparse.ArgumentParser();a.add_argument('source',type=Path);a.add_argument('--out',type=Path,default=Path(__file__).with_name('fixture.json'));v=a.parse_args();extract(v.source,v.out)
