"""D3 passive materials, measured contact/latch/harness and read-only final scoring.
The latch/drop channels stay score-gated until actual integrated verification.
Policies issue genuine robot Actions; only reset initializes object state.
"""
import math,numpy as np
from d3_foundation_physics import Physics as FoundationPhysics,arr
from d3_material import rotation_xyzw
from d3_history import MeasuredHistory,geometry
from d3_mechanisms import LidLatch,Harness
VALIDATED_RUNTIME_CHANNELS=frozenset(['clock','shock','contact','wrist','package_force','latch','drop'])
# Evidence:43478/43479/43480 plus actual consistency assertions. Latch/drop remain gated.
class Physics(FoundationPhysics):
 def initialize_episode_state(self):
  super().initialize_episode_state();self.done=False;self.error=None;self.history=None;self.broken={n:False for n in 'AB'};self.drop_contact_veto=False;self.body_frames=None;self._post_packet=None;self.lid_events=[]
  for c in self.materials:
   n=c['number'];c['latch']=LidLatch();c['harness']=Harness(n,float(self.fixture['cartons'][n-1]['position_world_m'][2]));c['tester_force_peak_n']=0.;c['lid_normal_push_n']=0.;c['lid_reaction_nm']=0.
 def begin_episode(self):
  self.broken={n:False for n in 'AB'};super().begin_episode();self.body_frames=self.read_bodies();rest={c['number']:float(self.fixture['cartons'][c['number']-1]['position_world_m'][2]) for c in self.materials};self.history=MeasuredHistory(rest,required=('A',));self.history.validated={k:k in VALIDATED_RUNTIME_CHANNELS for k in self.history.validated};self.done=False;self.error=None;self.drop_contact_veto=False;self.lid_events=[]
 def read_packet(self):
  import torch
  def tensor(value):
   if hasattr(value,'torch'):return value.torch
   if type(value).__module__.startswith('warp'):
    import warp as wp
    return wp.to_torch(value)
   return torch.as_tensor(value,device=self.base.device)
  entries=[]
  for c in self.materials:
   n=c['number'];entries.extend([(f'link{n}',tensor(c['carton'].data.body_link_state_w)),(f'com{n}',tensor(c['carton'].data.body_com_state_w)),(f'q{n}',tensor(c['carton'].data.joint_pos)),(f'face{n}',tensor(c['face'].data.root_com_state_w))])
  for name in 'AB':entries.append(('box'+name,tensor(self.base.scene['box_'+name].data.root_link_state_w)))
  for side in ['left','right']:
   sensor=self.base.scene[side+'_d3_wrench'];entries.append(('wrist'+side,tensor(sensor.data.force)[0,sensor.body_names.index('gripper')]))
  values=[value for name,value in entries]
  if any(value.dtype!=torch.float32 for value in values):raise ValueError('Unexpected live-state buffer dtype')
  if values[0].is_cuda:torch.cuda.synchronize(values[0].device)
  host=torch.cat([value.reshape(-1) for value in values]).detach().cpu().numpy().copy();parts={};offset=0
  for name,value in entries:
   count=value.numel();parts[name]=host[offset:offset+count].reshape(tuple(value.shape));offset+=count
  packet={'cartons':{},'carton_com':{},'faces':{},'boxes':{name:parts['box'+name][0] for name in 'AB'},'wrist_vectors':{side:parts['wrist'+side] for side in ['left','right']}}
  for c in self.materials:
   n=c['number'];states=parts[f'link{n}'][0];c['link_states']=states;packet['cartons'][n]={'state':states[c['index']],'lid_q':float(parts[f'q{n}'][0,0]),'latched':bool(c['latch'].latched)};packet['carton_com'][n]=parts[f'com{n}'][0];packet['faces'][n]=parts[f'face{n}'][0]
  return packet
 def read_bodies(self):
  if self._post_packet is None:self._post_packet=self.read_packet()
  return {'cartons':self._post_packet['cartons'],'boxes':self._post_packet['boxes']}
 def pre_step(self,dt):
  super().pre_step(dt)
  if self.settling or self.history is None or self.error:return
  import torch
  touched=set(self.tensor_contacts.latest['touched_items']) if self.tensor_contacts.latest else set()
  for c in self.materials:
   n=c['number'];h=c['harness'];latch=c['latch'];obj=c['carton'];root=self.body_frames['cartons'][n]['state'];b=c['pre_base_com'];B=rotation_xyzw(root[3:7]);inside=[x for x in 'AB' if geometry(self.body_frames['boxes'][x],self.body_frames['cartons'][n],h.rest_z)['inside']];mass=.27+sum(float(self.fixture['parameters']['box_mass']['AB'.index(x)]) for x in inside)
   result=h.pre(dt,self.physics_s,root[:3],b[:3],B,b[7:10],b[10:13],latch.latched,latch.armed,touched,inside,mass);latch.armed=result['armed'];c['phase']=h.phase
   if h.phase in ['lift','hold']:
    force=np.zeros((1,len(obj.body_names),3),np.float32);moment=np.zeros_like(force);force[0,c['index']]=c['last_wrench']['base_force']+result['force_w_n'];moment[0,c['index']]=c['last_wrench']['base_torque']+result['moment_w_nm'];obj.set_external_force_and_torque(torch.as_tensor(force,device=self.base.device),torch.as_tensor(moment,device=self.base.device),is_global=True);c['tester_force_peak_n']=max(c['tester_force_peak_n'],float(np.linalg.norm(result['force_w_n'])))
 def post_step(self,dt):
  if not self.settling and self.history is not None and not self.error:
   try:self._post_packet=self.read_packet()
   except Exception as e:self.error=type(e).__name__+': '+str(e);self.history.invalid_reason=self.error
  super().post_step(dt)
  if self.settling or self.history is None or self.error:return
  try:
   if self.contact_error:raise ValueError(self.contact_error)
   frame=self.tensor_contacts.latest
   if frame is None:raise ValueError('No actual contact frame')
   self.body_frames=self.read_bodies();touched=set(frame['touched_items']);direct=set(frame['direct_items']);normal=np.asarray(frame['normal_magnitude_by_filter_n']);vectors=np.asarray(frame['normal_force_matrix_n'])+np.asarray(frame['friction_force_by_filter_n']);arms=np.zeros((2,3))
   for j,path in enumerate(frame['filter_patterns']):
    owners=[i for i,prefix in enumerate(self.tensor_contacts.robot_roots) if path.startswith(prefix+'/')]
    if len(owners)!=1:raise ValueError('Unresolved robot force filter')
    arms[owners[0]]+=vectors[:,j].sum(0)
   self.package_forces=np.linalg.norm(arms,axis=1).tolist();self.wrist_forces=[]
   for side in ['left','right']:
    F=self._post_packet['wrist_vectors'][side]
    if not np.isfinite(F).all():raise ValueError('Nonfinite raw wrist force')
    self.wrist_forces.append(float(np.linalg.norm(F)))
   for x in 'AB':self.broken[x]=self.broken[x] or self.shocks[x].peak_abs_g>50. or self.shocks[x].peak_vector_g>50.
   for c in self.materials:
    n=c['number'];obj=c['carton'];latch=c['latch'];h=c['harness'];q=self.body_frames['cartons'][n]['lid_q'];index=frame['sensor_paths'].index(self.tensor_contacts.env+f'/carton_{n}/lid');push=float(normal[index].sum());lid_contact=bool(np.any(np.asarray(frame['contact_present_by_filter'])[index]))
    #43491 resolves the installed raw incoming-wrench frame; use its actualhinge-X component.
    # Source _lids reads the reaction only for a latched, robot-touched lid.
    # In every other branch the value cannot affect the transition; reportNone.
    required=latch.latched and lid_contact;reaction=0.
    if required:
     lid=obj.body_names.index('lid');raw=arr(obj.root_view.get_link_incoming_joint_force())[0,lid]
     if raw.shape!=(6,) or not np.isfinite(raw).all():raise ValueError('Invalid actual lid joint reaction')
     reaction=float(raw[3])
    c['lid_normal_push_n']=push;c['lid_reaction_nm']=reaction if required else None;event=latch.update(dt,q,lid_contact,push,reaction)
    if event:
     import torch
     limits=[0.,0.] if event=='lock' else [0.,1.75];obj.write_joint_position_limit_to_sim_index(limits=torch.tensor([[limits]],device=self.base.device),warn_limit_violation=False);self.lid_events.append({'t':self.physics_s,'carton':n,'event':event,'normal_push_n':push,'hinge_reaction_nm':c['lid_reaction_nm'],'q':q})
    c['latched']=latch.latched;self.body_frames['cartons'][n]['latched']=bool(latch.latched)
    if h.phase in ['lift','hold','drop','window'] and h.touched(touched):h.contact=True;self.drop_contact_veto=True
    inside=[x for x in 'AB' if geometry(self.body_frames['boxes'][x],self.body_frames['cartons'][n],h.rest_z)['inside']]
    h.post(self.physics_s,float(self.body_frames['cartons'][n]['state'][2]),latch.latched,inside,{x:self.shocks[x].status()['mean_abs_g'] for x in 'AB'},self.broken);c['phase']=h.phase
    # The rigidface represents the moving control corners; watch theiractualworldvelocities.
    f=c['post_face_com'];F=rotation_xyzw(f[3:7]);points=np.array([[x,y,0.] for x in [-.0805,.0805] for y in [-.0605,.0605]]);vel=f[7:10]+np.cross(f[10:13],points@F.T)
    if not np.isfinite(vel).all() or np.max(abs(vel))>30.:self.history.diverged=True
   source=[d for c in self.materials for d in c['harness'].records];opened=[d for c in self.materials for d in [c['harness'].open_record()] if d is not None]
   self.history.add(dt,self.body_frames['cartons'],self.body_frames['boxes'],touched,direct,self.wrist_forces,self.package_forces,{x:self.shocks[x].status() for x in 'AB'},source,opened,bool(self.drop_contact_veto))
  except Exception as e:
   self.error=type(e).__name__+': '+str(e)
   if self.history is not None:self.history.invalid_reason=self.error
 def score(self):
  if self.history is None:return {'score':0.,'stage':'initializing'}
  return self.history.score(self.controls)
 def termination_reason(self):
  if self.error or self.contact_error:return 'measurement_error'
  if self.history is not None:
   if any(g.tripped for g in self.history.wrist):return 'wrist_force_abort'
   if any(g.tripped for g in self.history.package):return 'crush_force_abort'
   if self.history.diverged or any(self.broken.values()):return 'source_damage'
  if self.controls>=1604:return 'time_limit'
  return None
 def status(self):
  status=super().status();status.update(stage='integrated_latch_drop_channels_unverified',measurement_error=self.error,runtime_score=self.score(),validated_channels=sorted(VALIDATED_RUNTIME_CHANNELS),wrist_force_n=getattr(self,'wrist_forces',None),package_force_n=getattr(self,'package_forces',None),broken=dict(self.broken),lid_events=list(self.lid_events),source_drops=[d for c in self.materials for d in c['harness'].records])
  for row,c in zip(status['cartons'],self.materials):row.update(normal_push_n=c['lid_normal_push_n'],hinge_reaction_nm=c['lid_reaction_nm'],tester_force_peak_n=c['tester_force_peak_n'],armed=c['latch'].armed)
  if self.history is not None:status.update(final=self.history.final,independent_drops=self.history.drops,history_valid=self.history.invalid_reason is None)
  return status

 def update_visuals(self):
  super().update_visuals()
  if not hasattr(self,'broken'):return
  from pxr import UsdShade,Gf
  from isaaclab.sim.utils import get_current_stage
  stage=get_current_stage()
  for name,broken in self.broken.items():
   path=self.tensor_contacts.env+'/box_'+name+'/box_'+name+'_indicator_look'
   material=UsdShade.Material(stage.GetPrimAtPath(path))
   if not material:raise ValueError('Missing actual shock-indicator material')
   shader=material.ComputeSurfaceSource()[0];color=(.9,.1,.1) if broken else (.97,.97,.97);shader.GetInput('diffuseColor').Set(Gf.Vec3f(*color))
