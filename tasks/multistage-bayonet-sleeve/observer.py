"""Read physical substeps, rigid-body contacts and robot wrist reaction history.
Calibration receipts are optional evidence; missing/unmatched receipts gate score.
"""
import hashlib,json
from pathlib import Path
import numpy as np
from contacts import arr,body_paths,Contacts,rot_xyzw
from scoring import History,REQUIRED,DT
HERE=Path(__file__).resolve().parent
RUNTIME_FILES=('fixture.json','fixture.xml','native_source.py','source_geometry.py','coupling-fixture.usda','coupling-sleeve.usda','kit.py','scoring.py','observer.py','contacts.py','motor_limits.py','step_guard.py','task.py')
def source_hashes():return {n:hashlib.sha256((HERE/n).read_bytes()).hexdigest() for n in RUNTIME_FILES}
class Observer:
 def __init__(self,emb):
  from isaaclab_physx.physics import PhysxManager
  import kit
  self.emb=emb;self.base=emb._base;self.sleeve=self.base.scene['coupling_sleeve'];self.asset=self.base.scene['coupling_fixture'];self.history=History();self.error=None;self.last=None;self.shift=np.array(kit.FIX['translation']);self.motor_tables={};self.rows=[]
  view=PhysxManager.get_physics_sim_view();paths=body_paths(self.asset);robots=body_paths(self.base.scene['left_robot'])+body_paths(self.base.scene['right_robot']);sensors=[str(self.sleeve.cfg.prim_path).replace('{ENV_REGEX_NS}','/World/envs/env_0')]
  # Resolve free-body path from SDK view where available; avoid wildcard guesses.
  if hasattr(self.sleeve.root_view,'prim_paths'):sensors=list(self.sleeve.root_view.prim_paths)
  self.touch=Contacts(view,sensors,robots);self.post=Contacts(view,sensors,[p for p in paths if p.endswith(('/stand','/plunger'))]);self.floor=Contacts(view,sensors,[p for p in paths if p.endswith('/tray_floor')]);self.selections=dict(robot=self.touch.selection,post=self.post.selection,floor=self.floor.selection)
  self.channels=frozenset();self.calibration=None
  from validation_receipt import verify_receipt
  if verify_receipt():self.channels=REQUIRED;self.calibration=True
 def apply_passive(self):
  # Environment-only source linear spring/damper. No release latches or pose writes.
  if self.error:return
  try:
   import torch,kit
   q=self.asset.data.joint_pos.torch;v=self.asset.data.joint_vel.torch
   self.asset.set_joint_position_target(torch.zeros_like(q));self.asset.set_joint_effort_target(-kit.FIX['parameters']['plunger_k']*q-1.5*v)
  except Exception as exc:self.error=str(exc);self.history.failure='passive_spring_unavailable'
 def sample(self,dt):
  if self.error:return
  try:
   origin=arr(self.base.scene.env_origins)[0];p=arr(self.sleeve.data.root_pos_w)[0]-origin;q=arr(self.sleeve.data.root_quat_w)[0];v=arr(self.sleeve.data.root_vel_w)[0];R=rot_xyzw(q)
   robot,_,_=self.touch.read(dt);post,_,_=self.post.read(dt);floor,_,_=self.floor.read(dt);wrist=[];checked=[];raw={}
   for side in ('left','right'):
    s=self.base.scene[side+'_b7_wrench'];wrist.append(float(np.linalg.norm(arr(s.data.force)[0,s.body_names.index('gripper')])));robot_asset=self.base.scene[side+'_robot']
    if side not in self.motor_tables:
     table=arr(robot_asset.root_view.get_dof_max_forces())[0];names=robot_asset.joint_names;expected={**{f'joint{i}':27. if i<4 else 7. for i in range(1,7)},'joint7':40.,'joint8':40.};measured={n:float(table[names.index(n)]) for n in expected}
     if any(abs(measured[n]-limit)>1e-5 for n,limit in expected.items()):raise ValueError('actual drive ceilings mismatch '+str(measured))
     self.motor_tables[side]=measured
    checked.append(bool(getattr(robot_asset,'_b7_total_effort_checked',False)));raw[side]=arr(robot_asset.data.joint_pos)[0].tolist()
   row=dict(pos=p.tolist(),quat_xyzw=q.tolist(),R=R.tolist(),velocity=v.tolist(),translation=self.shift.tolist(),robot_contact=bool(robot),post_contact=bool(post),floor_contact=bool(floor),wrist_force=wrist,motor_limits_checked=checked,raw_robot_joints=raw,plunger_q=float(arr(self.asset.data.joint_pos)[0,self.asset.joint_names.index('plunger_slide_C')]),robot_contacts=self.touch.last_contacts,post_contacts=self.post.last_contacts,floor_contacts=self.floor.last_contacts)
   self.last=row;self.rows.append(row);self.rows=self.rows[-25:];self.history.update(row,dt)
   if not all(checked):self.channels=frozenset()
  except Exception as exc:self.error=type(exc).__name__+': '+str(exc);self.history.failure='measurement_unavailable';self.channels=frozenset()
 def result(self):return dict(self.history.result(self.channels),measurement_error=self.error,motor_tables=self.motor_tables,physics_device=str(self.emb.device),calibration_receipt_verified=self.calibration is not None)
