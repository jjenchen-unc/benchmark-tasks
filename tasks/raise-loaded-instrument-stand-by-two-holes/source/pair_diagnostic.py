"""Read-only primary actor-pair telemetry, separate from native scorer/contact reader.
No collision enable/filter/contact-report/physics writes. No task or robot state writes.
"""
from pathlib import Path
import os
import numpy as np
from numeric_json import plain,dumps

def decode(data, sensor, filters, dt):
 from kit import arr
 force,point,normal,separation,count,start=data
 count=arr(count).astype(int);start=arr(start).astype(int)
 if count.shape!=(1,len(filters)) or start.shape!=count.shape:raise ValueError('unresolved pair matrix shape')
 f=arr(force).reshape(-1);p=arr(point).reshape(-1,3);n=arr(normal).reshape(-1,3);d=arr(separation).reshape(-1)
 if np.any(count<0) or int(count.sum())>=32768:raise ValueError('contact buffer overflow')
 out=[]
 for j,target in enumerate(filters):
  size=int(count[0,j]);offset=int(start[0,j])
  if size and (offset<0 or offset+size>min(len(f),len(p),len(n),len(d))):raise ValueError('invalid contact range')
  for k in range(offset,offset+size):
   out.append(dict(sensor=sensor,other=target,normal_force_n=float(f[k]),point_w=p[k],normal_w=n[k],separation_m=float(d[k]),dt=dt))
 return plain(out)

def usd_value(v):
 """Keep USD scalar/vector metadata typed; only path-valued metadata is text."""
 if isinstance(v,(float,np.floating)):
  import math
  if math.isnan(float(v)):raise ValueError('NaN USD metadata')
  if math.isinf(float(v)):return {'usd_schema_bound':'+infinity' if v>0 else '-infinity'}
 if v is None or isinstance(v,(str,bool,int,float)):return plain(v)
 if isinstance(v,np.generic):return usd_value(v.item())
 if hasattr(v,'GetReal') and hasattr(v,'GetImaginary'):return {'real':float(v.GetReal()),'imaginary':list(v.GetImaginary())}
 from pxr import Sdf
 if isinstance(v,(Sdf.Path,Sdf.AssetPath)):return str(v)
 try:return [usd_value(x) for x in v]
 except TypeError:raise TypeError('unsupported USD metadata: '+type(v).__name__)

def scene_geometry(stage):
 from pxr import Usd,UsdGeom,UsdPhysics
 cache=UsdGeom.XformCache();rows=[];owners=set()
 for prim in stage.Traverse():
  if not prim.HasAPI(UsdPhysics.CollisionAPI):continue
  ancestor=prim;owner=None
  while ancestor and not ancestor.IsPseudoRoot():
   if ancestor.HasAPI(UsdPhysics.RigidBodyAPI):owner=str(ancestor.GetPath());break
   ancestor=ancestor.GetParent()
  owner=owner or str(prim.GetPath());owners.add(owner)
  attrs={}
  for a in prim.GetAttributes():
   name=a.GetName();v=a.Get()
   if v is None:continue
   if name.startswith(('physics:','physxCollision:','physxRigidBody:')):attrs[name]=usd_value(v)
   elif name in ('size','radius','height','axis','points','faceVertexCounts','faceVertexIndices','extent'):
    # Numeric geometry stays numeric; axis is a token.
    def value(x):
     if isinstance(x,(str,int,float,bool)):return x
     return [value(y) for y in x]
    attrs[name]=value(v)
  rel={r.GetName():[str(x) for x in r.GetTargets()] for r in prim.GetRelationships() if r.GetName().startswith('physics:')}
  rows.append(dict(schemas=list(prim.GetAppliedSchemas()),path=str(prim.GetPath()),actor=owner,type=prim.GetTypeName(),world_matrix=[list(row) for row in cache.GetLocalToWorldTransform(prim)],attributes=attrs,relationships=rel))
 return plain(rows),sorted(owners)

class PairDiagnostic:
 def __init__(self,emb):
  import omni.usd
  from kit import ENGINE
  from isaaclab_physx.physics import PhysxManager
  self.engine=ENGINE;self.phase='initial';self.samples=0;self.views=[];self.error=None;self.events=[]
  base=Path(os.environ['PINS_ACTION_TRACE']).parent
  self.file=(base/'primary-pairs.jsonl').open('w',buffering=1)
  stage=omni.usd.get_context().get_stage()
  geometry,filters=scene_geometry(stage)
  topology=[dict(path=str(p.GetPath()),schemas=list(p.GetAppliedSchemas()),attributes={a.GetName():usd_value(a.Get()) for a in p.GetAttributes() if a.GetName().startswith(('physics:','physx'))},relationships={r.GetName():[str(v) for v in r.GetTargets()] for r in p.GetRelationships() if r.GetName().startswith('physics:')}) for p in stage.Traverse() if p.GetAppliedSchemas()]
  sensors=[p for side in ('left','right') for p in emb._base.scene[side+'_robot'].root_view.link_paths[0]]
  sim=PhysxManager.get_physics_sim_view()
  # One exact sensor per view removes backend sensor ordering ambiguity.
  for sensor in sensors:
   view=sim.create_rigid_contact_view([sensor],filter_patterns=[filters],max_contact_data_count=32768)
   if view.sensor_count!=1 or view.filter_count!=len(filters):raise RuntimeError('incomplete primary-pair coverage: '+sensor)
   self.views.append((sensor,view))
  from omni.physx import get_physx_simulation_interface
  self.subscription=get_physx_simulation_interface().subscribe_full_contact_report_events(self.on_report)
  self.filters=filters
  (base/'primary-pair-coverage.json').write_text(dumps(dict(sensors=sensors,filters=filters,geometry=geometry,topology=topology,dt=.002,read_only=True),indent=2))
  self.original=ENGINE.post
  def post(dt):
   # Existing task mechanism/scorer executes unchanged first.
   try:return self.original(dt)
   finally:self.read(dt)
  self.hook=post;ENGINE.post=post
 def on_report(self,headers,data,friction):
  from pxr import PhysicsSchemaTools
  try:
   for h in headers:
    actor0=str(PhysicsSchemaTools.intToSdfPath(h.actor0));actor1=str(PhysicsSchemaTools.intToSdfPath(h.actor1))
    if not any('/LeftArm/' in x or '/RightArm/' in x for x in (actor0,actor1)):continue
    points=[]
    for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data):
     c=data[i];points.append(dict(position=list(c.position),normal=list(c.normal),impulse=list(c.impulse),separation=float(c.separation),face0=int(c.face_index0),face1=int(c.face_index1)))
    anchors=[]
    if friction is not None:
     for i in range(h.friction_anchors_offset,h.friction_anchors_offset+h.num_friction_anchors_data):
      a=friction[i];anchors.append(dict(position=list(a.position),impulse=list(a.impulse)))
    self.events.append(plain(dict(type=str(h.type),actor0=actor0,actor1=actor1,collider0=str(PhysicsSchemaTools.intToSdfPath(h.collider0)),collider1=str(PhysicsSchemaTools.intToSdfPath(h.collider1)),points=points,friction_anchors=anchors)))
  except Exception as e:self.error='contact callback: '+type(e).__name__+': '+str(e)
 def read(self,dt):
  if self.engine.settling:return
  self.samples+=1
  from approach_diagnostic import capture
  try:
   if self.error:raise RuntimeError(self.error)
   pairs=[]
   for sensor,view in self.views:pairs.extend(decode(view.get_contact_data(dt),sensor,self.filters,dt))
   row=dict(substep=self.samples,time_s=self.engine.time,control=self.engine.history.controls+1,phase=self.phase,pairs=pairs,primary_shape_events=self.events)
   self.events=[]
   # Preserve actual independent fingers/all link poses at every relevant sample.
   if self.phase=='L_knob_approach' or pairs:row['snapshot']=capture(self.engine.emb)
   self.file.write(dumps(row)+'\n')
  except Exception as e:
   self.error=type(e).__name__+': '+str(e)
   self.file.write(dumps(dict(substep=self.samples,diagnostic_error=self.error))+'\n')
   raise
 def close(self):
  if self.engine.post is self.hook:self.engine.post=self.original
  self.subscription=None
  self.file.close()
