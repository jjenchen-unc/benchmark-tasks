"""Read-only scoring observer and passive wet material transition.
Only wetting changes cup-pair material parameters/appearance; never body poses,
forces or scorer state from a policy. Credit requires exact-source verification.
"""
from collections import deque
from pathlib import Path
import json,os
import numpy as np
from c1_contract import *
from c1_history import History
from c1_kit import arr,startup_mark
HERE=Path(__file__).resolve().parent
REQUIRED=frozenset(['geometry','explicit_pair_filters','dry_slip_hold','wet_transition','contacts_bracing','wrist_force','link_guard','total_motor_limits','genuine_robot_fullgoal'])
# Intentionally empty until root returns matching-source physical evidence.
from c1_validation import validated_channels
VERIFIED_CHANNELS=validated_channels()

def author_filters(stage,env):
 """Preserve explicit shape-pair exclusions, changing each relationship once.
 Existing complete relations are audited without reauthoring a playing scene.
 """
 from pxr import UsdPhysics,Sdf
 colliders=[p for p in stage.Traverse() if p.HasAPI(UsdPhysics.CollisionAPI)]
 tasks=[p for p in colliders if any(str(p.GetPath()).startswith(env+'/'+n+'/') for n in [*[f'liner_{i}' for i in range(1,7)],'cup_1','cup_2'])]
 role=lambda p:'liner' if '/liner_' in str(p.GetPath()) else 'cup' if '/cup_' in str(p.GetPath()) else 'other'
 pair=lambda p:str(p.GetPath()).endswith('/pair_contact')
 count=0;updates=0
 for p in tasks:
  required=[]
  for q in colliders:
   if p==q:continue
   opposite={role(p),role(q)}=={'liner','cup'}
   allow=(opposite and pair(q)) if pair(p) else not opposite
   if not allow:required.append(q.GetPath());count+=1
  if pair(p):
   for name in ['LeftArm','RightArm']:required.append(Sdf.Path(env+'/'+name));count+=1
  api=UsdPhysics.FilteredPairsAPI(p) if p.HasAPI(UsdPhysics.FilteredPairsAPI) else UsdPhysics.FilteredPairsAPI.Apply(p)
  relation=api.GetFilteredPairsRel()
  if not relation:relation=api.CreateFilteredPairsRel()
  current=list(relation.GetTargets());seen=set(current);missing=[]
  for target in required:
   if target not in seen:missing.append(target);seen.add(target)
  if missing:
   relation.SetTargets(current+missing);updates+=1
 return {'task_colliders':len(tasks),'all_colliders':len(colliders),'exclusions_authored':count,'relationship_updates':updates}

class Physics:
 def __init__(self,emb,specs):
  self.emb=emb;self.base=emb._base;self.specs=specs;self.active=False;self.done=False;self.controls=0;self.error=None;self.trace=None
  self.target=os.environ.get('C1_TARGET','C1-T1')
  from isaaclab.sim.utils import get_current_stage
  self.stage=get_current_stage();env=str(self.base.scene['liner_1'].root_view.prim_paths[0]).rsplit('/',1)[0];self.env=env
  startup_mark('existing_stage_filter_author_start');self.filter_audit=author_filters(self.stage,env);startup_mark('existing_stage_filter_author_complete')
  self.history=History(self.target);self.wet=[False]*6;self.guard_n=float(os.environ.get('C1_FORCE_GUARD_N','30'));
  if not np.isfinite(self.guard_n) or not 5.<=self.guard_n<=60.:raise ValueError('Source force guard requires5..60N')
  self.guard_streak=np.zeros(2);self.freeze_arms=False;self.links=deque();self.link_max=0.;self.max_wrist=np.zeros(2);self.examples=[]
  from c1_contacts import Contacts
  startup_mark('contact_view_create_start');self.contacts=Contacts(self.base);startup_mark('contact_view_create_complete')
 def set_mu(self,i,mu,wet=False):
  from pxr import UsdPhysics,UsdShade,Usd,Gf
  path=self.env+f'/liner_{i+1}/pair_material';mat=UsdPhysics.MaterialAPI(self.stage.GetPrimAtPath(path))
  if not mat:raise RuntimeError('Missing liner pair-only material '+path)
  mat.GetStaticFrictionAttr().Set(float(mu));mat.GetDynamicFrictionAttr().Set(float(mu))
  # The dry/wet shade changes once; gloss is not a friction-value cue.
  root=self.stage.GetPrimAtPath(self.env+f'/liner_{i+1}')
  for p in Usd.PrimRange(root):
   if p.IsA(UsdShade.Shader):
    shader=UsdShade.Shader(p);rough=shader.GetInput('roughness')
    if rough:rough.Set(.05 if wet else .8)
 def reset_materials(self,seed):
  self.params=parameters(seed,transfer=self.target=='C1-T4');self.wet=[False]*6
  for i,mu in enumerate(self.params['mu_dry']):self.set_mu(i,mu)
 def begin_episode(self,seed):
  if self.trace:self.trace.close()
  self.trace=open(os.environ['C1_TRACE'],'w',buffering=1) if os.environ.get('C1_TRACE') else None
  self.history=History(self.target);self.controls=0;self.done=False;self.error=None;self.guard_streak=np.zeros(2);self.freeze_arms=False;self.links.clear();self.link_max=0.;self.max_wrist=np.zeros(2);self.samples=0;self.active=True
 def state(self,name):
  obj=self.base.scene[name];p=arr(obj.data.root_link_state_w)[0];origin=arr(self.base.scene.env_origins)[0];return p[:3]-origin,rotation_xyzw(p[3:7])
 def sample(self,dt):
  if self.error:return
  try:
   if abs(dt-DT)>1e-8:raise ValueError('C1 physics dt must remain exactly1ms')
   lp,lR=zip(*(self.state(f'liner_{i}') for i in range(1,7)));cp,cR=zip(*(self.state(f'cup_{i}') for i in range(1,3)))
   for i,p in enumerate(lp):
    n=native(p)
    if not self.wet[i] and abs(n[0]-.05)<.07 and abs(n[1]-.035)<.085 and p[2]<WATER_TOP:
     self.wet[i]=True;self.set_mu(i,self.params['mu_wet'][i],True)
   touch,links,self.examples=self.contacts.sample(dt)
   forces=[]
   for side in ['left','right']:
    sensor=self.base.scene[side+'_c1_wrench'];forces.append(float(np.linalg.norm(arr(sensor.data.force)[0,sensor.body_names.index('gripper')])))
   row={'contacts':touch,'cup_pos':[x.tolist() for x in cp],'cup_R':[x.tolist() for x in cR],'liner_pos':[x.tolist() for x in lp],'liner_R':[x.tolist() for x in lR],'wet':list(self.wet),'wrist_force_n':forces}
   self.history.update(row,dt);self.samples+=1;self.max_wrist=np.maximum(self.max_wrist,forces)
   self.guard_streak=np.where(np.asarray(forces)>self.guard_n,self.guard_streak+dt,0.)
   self.links.append((self.history.seconds,links))
   while self.links and self.links[0][0]<self.history.seconds-.020+dt/2:self.links.popleft()
   sums={}
   for t,loads in self.links:
    for b,f in loads.items():sums[b]=sums.get(b,0.)+f*dt/.020
   self.link_max=max(sums.values(),default=0.)
   if self.link_max>50. or np.any(self.guard_streak>=.020-1e-10):self.freeze_arms=True
   if self.controls<=400 or self.freeze_arms:
    from diagnostic_observer import record
    record(self,dt,forces,links,sums)
   if self.trace and self.samples%50==0:self.trace.write(json.dumps({'controls':self.controls,'row':row,'raw_score':self.history.score(),'guard_frozen':self.freeze_arms,'link20ms_force_n':self.link_max})+'\n')
  except Exception as e:self.error=type(e).__name__+': '+str(e);self.history.failure='measurement_unavailable'
 def validate_action(self,action):
  a=np.asarray(action.data,float)
  if a.shape!=(14,) or not np.isfinite(a).all():raise ValueError('C1 requires finite14D absolute YAM joint Actions')
  # Every public action starts a fresh motion call; automatic substep force
  # stop persists to its end. Hard80N/100ms history never resets here.
  self.guard_streak[:]=0.;self.freeze_arms=False
  for side in ['left','right']:self.base.scene[side+'_robot']._c1_guard_hold=None
 def termination_reason(self):
  if self.error:return 'measurement_unavailable'
  if self.history.failure:return self.history.failure
  if self.controls>=BUDGET:return 'time_limit'
  return None
 def status(self):
  raw=self.history.score();missing=sorted(REQUIRED-VERIFIED_CHANNELS)
  return dict(raw,score=0. if missing else raw['score'],raw_score=raw['score'],gate='unverified_runtime_channels' if missing else None,missing=missing,measurement_error=self.error,controls=self.controls,wet=list(self.wet),wrist_max_n=self.max_wrist.tolist(),force_guard_frozen=self.freeze_arms,link20ms_force_n=self.link_max,filter_audit=self.filter_audit,submission_ready=False)
 def close(self):
  if hasattr(self,'diagnostic_trace'):self.diagnostic_trace.close()
  if self.trace:self.trace.close();self.trace=None
  self.contacts.close()
