"""Optional rebuild tool: static native-fixture compilation only, NO simulation.
Requires MuJoCo3.3.7 and numpy; runtime USD authoring does not need MuJoCo.
"""
import hashlib,json,xml.etree.ElementTree as ET
from pathlib import Path
import mujoco
HERE=Path(__file__).resolve().parent

def compile_fixture():
    tree=ET.fromstring((HERE/'fixture.xml').read_text());root=ET.Element('mujoco');ET.SubElement(root,'compiler',angle='radian');default=ET.SubElement(root,'default');ET.SubElement(default,'geom',friction='1.1 .01 .001');ET.SubElement(default,'joint',armature='.02');world=ET.SubElement(root,'worldbody');world.extend([b for b in tree if b.tag=='body' and b.get('name')!='overview_target']);m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
    fields='body_parentid body_pos body_quat body_mass body_ipos body_inertia body_iquat geom_bodyid geom_size geom_type geom_pos geom_quat geom_rgba geom_contype geom_conaffinity geom_friction jnt_bodyid jnt_type jnt_pos jnt_axis jnt_range dof_armature dof_frictionloss jnt_stiffness dof_damping'.split()
    result={n:getattr(m,n).tolist() for n in fields};result.update({n:int(getattr(m,n)) for n in ('nbody','ngeom','njnt')});result['names']={kind:[getattr(m,kind)(i).name for i in range(getattr(m,{'body':'nbody','geom':'ngeom','joint':'njnt'}[kind]))] for kind in ('body','geom','joint')};result['provenance']={'fixture_xml_sha256':hashlib.sha256((HERE/'fixture.xml').read_bytes()).hexdigest(),'mujoco_version':mujoco.__version__,'simulation_steps':0,'source_defaults':'dual_arm_demo.env geom friction1.1/.01/.001, joint armature.02; portable_demo overrides solver only'}
    (HERE/'compiled-fixture.json').write_text(json.dumps(result,indent=2)+'\n');return result
if __name__=='__main__':r=compile_fixture();print(json.dumps({k:r[k] for k in ('nbody','ngeom','njnt','provenance')},indent=2))
