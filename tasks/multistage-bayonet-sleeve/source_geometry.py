"""Execute selected EXACT frozen native constants/helpers, without MuJoCo/robot imports.
The complete original is native_source.py. No source regex substitutions.
"""
import ast,hashlib,math,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent
CONSTANTS='HOLD_S DT TICK_BUDGET HAND_SALT TABLE_Z STAND_H POST_R POST_TOP ZP PIN_R PIN_IN PIN_OUT S0 WS WPHI DA R_IN R_OUT R_SHELL SLEEVE_L CAP_T SLEEVE_MASS MOUTH0 PLUNGER_K PLUNGER_PRE PLUNGER_TOP0 OUT JUNCTION_DEG DECOY_H DECOY_DEG NOTCH_H STANDS DEPLOY_XY TRAY_XY TRAY_HALF TRAY_FLOOR TRAY_SLOTS COLOURS MARKS BASE_PATHS FOOT_R FOOT_L'.split()
FUNCTIONS='mirrored handedness path_D hands_of parameters dead_ends track_segments free_intervals solid_intervals f3 zquat add_coupling add_tray sleeve_footprint sleeve_inside_tray'.split()
def load():
 path=HERE/'native_source.py';tree=ast.parse(path.read_text());selected=[]
 for node in tree.body:
  if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id in CONSTANTS for root in node.targets for t in ast.walk(root)):selected.append(node)
  if isinstance(node,ast.FunctionDef) and node.name in FUNCTIONS:selected.append(node)
 env=dict(np=np,math=math,ET=ET)
 exec(compile(ast.Module(body=selected,type_ignores=[]),str(path),'exec'),env)
 missing=set(CONSTANTS+FUNCTIONS)-env.keys()
 if missing:raise ValueError('missing frozen definitions '+str(missing))
 return env
N=load()
def xml():
 world=ET.Element('worldbody');p=N['parameters'](42)
 N['add_coupling'](world,'C',p['path_C'],N['DEPLOY_XY'],p['plunger_k']);N['add_tray'](world)
 return world,p
if __name__=='__main__':
 w,p=xml();(HERE/'fixture.xml').write_text(ET.tostring(w,encoding='unicode')+'\n');print(p)
