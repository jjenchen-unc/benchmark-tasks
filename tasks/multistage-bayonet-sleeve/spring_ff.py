"""Read-only spring/payload load estimate converted to public joint targets.
The existing implicit actuator remains the sole force producer with unchanged
27/7 Nm total limits and gravity compensation. No simulator write/API here.
"""
import numpy as np
import json
from pathlib import Path
FIX=json.loads((Path(__file__).resolve().parent/"fixture.json").read_text())
STAND=next(b for b in FIX["bodies"] if b["name"]=="stand_C")
from yam_arena import ik
from source_geometry import N
GAINS=np.array([80.,80.,80.,10.,10.,10.])
def compensate(q,base,row):
 q=np.asarray(q,float);p,_=ik.tcp_pose(q,base);eps=1e-6
 J=np.column_stack([(ik.tcp_pose(q+np.eye(6)[i]*eps,base)[0]-p)/eps for i in range(6)])
 # Contact with the shaft alone does not prove plunger contact: the native
 # cap underside must coincide with the measured pad top before using spring.
 sleeve_z=float(row['pos'][2]);plunger=float(row['plunger_q'])
 stand_z=float(STAND['pos'][2])+float(FIX['translation'][2])
 pad_top=stand_z+float(N['PLUNGER_TOP0'])+plunger
 cap_under=sleeve_z+float(N['SLEEVE_L'])-float(N['CAP_T'])
 spring_contact=bool(row['post_contact']) and abs(pad_top-cap_under)<.002
 force_z=float(N['SLEEVE_MASS'])*9.81
 if spring_contact:force_z+=float(FIX['parameters']['plunger_k'])*plunger
 delta=J.T@np.array([0.,0.,force_z])/GAINS
 command=q+delta
 if not np.isfinite(command).all() or np.any(command<ik.LOWER) or np.any(command>ik.UPPER):raise RuntimeError('load feedforward joint limits')
 return command,dict(force_z_N=force_z,spring_contact=spring_contact,delta_joint_rad=delta.tolist(),pad_cap_error_m=pad_top-cap_under)
