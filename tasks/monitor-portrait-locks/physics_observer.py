"""Read-only PhysX observer for B5, sampled after every physical scene update.

Joint-wrench convention is child-side incoming joint frame at its joint anchor.
Root-weld forces are transformed to world and shifted to the foot centre before
checking the source's freestanding-foot wrench limits. No fixture poses, forces,
attachment/score flags or observations are altered by this observer.
"""
from __future__ import annotations
import json,math
from pathlib import Path
import numpy as np
from telemetry_math import History,geometric_pose,box_overlap,segment_box_distance
from contract import evaluate,REQUIRED_CHANNELS
from room_geometry import station_furniture,panel_contacts,box as furniture_box,validate_live_usd,serializable
# Populated only after this runtime adapter's sensor/calibration checks pass.
VALIDATED_RUNTIME_CHANNELS=frozenset(REQUIRED_CHANNELS)  # Actual43361knownloads,43383standpositive/neutral/release,43386box/furniture/HPT/reset negatives.

def arr(x):
    x=getattr(x,'torch',x)
    if hasattr(x,'detach'):return x.detach().cpu().numpy()
    if hasattr(x,'numpy'):return x.numpy()
    return np.asarray(x)

def rot(q):
    w,x,y,z=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def rot_xyzw(q):
    return rot(np.r_[q[3],q[:3]])

def body_paths(asset):
    # Resolve the bodies actually registered with PhysX, including links behind
    # instance proxies. USD subtree traversal can omit those robot links.
    paths=asset.root_view.link_paths
    if len(paths)!=1 or not paths[0]:raise ValueError('B5 observer requires one resolved articulation')
    return list(paths[0])


class Contacts:
    def __init__(self,simulation_view,sensors,filters):
        if not sensors or not filters:raise ValueError('empty contact body selection')
        self.capacity=8192
        self.view=simulation_view.create_rigid_contact_view(sensors,filter_patterns=[filters]*len(sensors),max_contact_data_count=self.capacity)
        if self.view.sensor_count<1 or self.view.filter_count<1:raise ValueError('unresolved contact sensors/filters')
        self.selection={'sensors':sensors,'filters':filters,'sensor_count':self.view.sensor_count,'filter_count':self.view.filter_count}
    def read(self,dt):
        force,point,normal,separation,count,start=self.view.get_contact_data(dt)
        count=arr(count).astype(int);start=arr(start).astype(int)
        if np.any(count<0) or int(count.sum())>=self.capacity:raise ValueError('contact buffer capacity/measurement invalid')
        f=arr(force).reshape(-1);d=arr(separation).reshape(-1);indices=[];pairs=[]
        for pair,(n,i) in enumerate(zip(count.ravel(),start.ravel())):
            if n:
                if i<0 or i+n>len(d):raise ValueError('contact indices invalid')
                indices.extend(range(i,i+n));pairs.extend([pair]*n)
        if not indices:self.last_contacts=[];return False,0.,0
        ids=np.array(indices,int)
        if not np.isfinite(np.r_[f[ids],d[ids]]).all():raise ValueError('nonfinite contact data')
        # Count active solver support even when separation is rounded slightly
        # positive.43237 measured38.6N at+0.094micrometres with old hit=False.
        hit=(d[ids]<=0.)|(abs(f[ids])>0.)
        pts=arr(point).reshape(-1,3);normals=arr(normal).reshape(-1,3)
        if not np.isfinite(np.r_[pts[ids].ravel(),normals[ids].ravel()]).all():raise ValueError('nonfinite contact vectors')
        self.last_contacts=[{'sensor_index':int(pair//count.shape[-1]),'filter_index':int(pair%count.shape[-1]),'separation_m':float(d[i]),'normal_force_n':float(f[i]),'point_w':pts[i].tolist(),'normal_w':normals[i].tolist()} for pair,i in zip(pairs,ids)]
        return bool(np.any(hit)),float(np.max(abs(f[ids][hit]))) if np.any(hit) else 0.,len(indices)

class Observer:
    def __init__(self,emb,fixture,spec):
        if str(emb.device)!='cpu':raise ValueError('Only CPU PhysX has passed B5 base-wrench calibration')
        import carb.settings
        if carb.settings.get_settings().get('/physics/disableContactProcessing') is not False:raise ValueError('CPU contact processing must be enabled by the standard contact sensor')
        self.emb,self.base,self.fixture,self.spec=emb,emb._base,fixture,spec
        self.asset=self.base.scene['monitor_stand'];self.history=None;self.channels=set();self.last=None;self.error=None
        self.qranges={j['name']:j['range'] for b in fixture['bodies'] for j in b['joints']}
        self.lowest_z=spec.pos[2]+next(b for b in fixture['bodies'] if b['name']=='stand_carriage')['native_world_pos'][2]-fixture['native_origin'][2]
        self.rootR=rot([math.cos(spec.yaw/2),0,0,math.sin(spec.yaw/2)]);self.foot=np.array(spec.pos)+self.rootR@np.array([.005,0,0])
        self.history=History(foot_R=self.rootR)
        from isaaclab_physx.physics import PhysxManager
        self.simview=PhysxManager.get_physics_sim_view()
        stand=body_paths(self.asset)
        robots=body_paths(self.base.scene['left_robot'])+body_paths(self.base.scene['right_robot'])
        if '/monitor_stand/' not in stand[0]:raise ValueError('unexpected monitor body namespace '+stand[0])
        envroot=stand[0].split('/monitor_stand/',1)[0]
        import os
        if os.environ.get('B5_TRACE'):
            Path(os.environ['B5_TRACE']).with_name('observer-selection.json').write_text(json.dumps({'stand':stand,'robots':robots,'environment':envroot},indent=2)+'\n')
        self.stand_contact=Contacts(self.simview,stand,robots)
        self.box_contact=Contacts(self.simview,robots,[envroot+'/monitor_stand/links/side_box'])
        self.selections={'robot_stand':self.stand_contact.selection,'robot_box':self.box_contact.selection}
        self.furniture_shapes=station_furniture(emb.rig.top_cam,emb.task.objects)
        import omni.usd
        self.furniture_validation=validate_live_usd(omni.usd.get_context().get_stage(),envroot,self.furniture_shapes,arr(self.base.scene.env_origins)[0])
        external=[v['collider'] for v in self.furniture_validation['checks'].values()]+[envroot+'/monitor_stand/links/side_box']
        panel_bodies=[p for p in stand if p.split('/')[-1] in ('stand_pivot','stand_tilt')]
        self.furniture_contact=Contacts(self.simview,panel_bodies,external)
        if self.furniture_contact.view.filter_count!=len(external):raise ValueError('furniture contact filters unresolved')
        self.selections['panel_furniture']=self.furniture_contact.selection
        self.steps_trace=None
    def world_geoms(self):
        origin=arr(self.base.scene.env_origins)[0]
        poses=dict(zip(self.asset.body_names,zip(arr(self.asset.data.body_pos_w)[0]-origin,arr(self.asset.data.body_quat_w)[0])))
        out=[]
        for b in self.fixture['bodies']:
            if b['name'] in poses:p,q=poses[b['name']];R=rot_xyzw(q)
            else:p=np.array(self.spec.pos)+self.rootR@(np.array(b['native_world_pos'])-self.fixture['native_origin']);R=self.rootR@rot(b['native_world_quat_wxyz'])
            for g in b['geoms']:
                if not(g['contype'] or g['conaffinity']):continue
                gp=np.array(g['pos'])-(np.array(self.fixture['native_origin']) if b['name']=='monitor_stand' else 0.)
                out.append(dict(g,name=g['name'],body=b['name'],centre=p+R@gp,R=R@rot(g['quat_wxyz']),half=np.array(g['size'])))
        return out,poses
    def read(self,dt):
        geoms,poses=self.world_geoms();q=dict(zip(self.asset.joint_names,arr(self.asset.data.joint_pos)[0].tolist()));v=dict(zip(self.asset.joint_names,arr(self.asset.data.joint_vel)[0].tolist()))
        car=poses['stand_carriage'][0];panelR=rot_xyzw(poses['stand_pivot'][1]);pose=geometric_pose(car,panelR,self.lowest_z)
        panel=[g for g in geoms if g['name'].startswith('monitor_') or g['name']=='tilt_housing']
        side=next(g for g in geoms if g['name']=='side_box_geom')
        obstacles=[*self.furniture_shapes,furniture_box('side_box',side['centre'],2*side['half'],side['R'])]
        furniture_pairs=panel_contacts(panel,obstacles)
        furniture_touch,furniture_force,furniture_count=self.furniture_contact.read(dt)
        furniture=bool(furniture_pairs) or furniture_touch
        pins=[g for g in geoms if g['body'].startswith('pin_')];teeth=[g for g in geoms if g['contype']==2 and not g['body'].startswith('pin_')];penetration=0.
        for pin in pins:
            axis=pin['R'][:,2];P=pin['centre']-axis*pin['half'][1];Q=pin['centre']+axis*pin['half'][1];radius=pin['half'][0]
            lo=np.minimum(P,Q)-radius;hi=np.maximum(P,Q)+radius
            for tooth in teeth:
                extent=abs(tooth['R'])@tooth['half']
                if np.any(hi<tooth['centre']-extent) or np.any(lo>tooth['centre']+extent):continue
                penetration=max(penetration,1000*(radius-segment_box_distance(P,Q,tooth['centre'],tooth['R'],tooth['half'])))
        robot_touch,_,stand_count=self.stand_contact.read(dt);box_touch,_,box_count=self.box_contact.read(dt)
        forces=[]
        for side in ('left','right'):
            sensor=self.base.scene[side+'_b5_wrench'];forces.append(float(np.linalg.norm(arr(sensor.data.force)[0,sensor.body_names.index('gripper')])))
        sensor=self.base.scene['b5_base_wrench'];i=sensor.body_names.index('monitor_stand');rootp,rootq=poses['monitor_stand'];R=rot_xyzw(rootq)
        F=R@arr(sensor.data.force)[0,i];M=R@arr(sensor.data.torque)[0,i]+np.cross(rootp-self.foot,F)
        row=dict(foot_R_world_from_native=self.rootR.tolist(),pose=pose,joint_pose=(q['height']*1000,math.degrees(q['pivot']),math.degrees(q['tilt'])),joint_vel=(v['height'],v['pivot'],v['tilt']),joint_q=q,joint_ranges=self.qranges,wrist_force_n=forces,base_force_w=F.tolist(),base_torque_at_foot_w=M.tolist(),robot_stand_contact=robot_touch,panel_furniture_contact=bool(furniture),robot_box_contact=box_touch,furniture_force_n=furniture_force,panel_furniture_pairs=furniture_pairs,lock_penetration_mm=max(0.,penetration),contact_counts={'robot_stand':stand_count,'robot_box':box_count,'panel_furniture':furniture_count},contact_details={'robot_stand':self.stand_contact.last_contacts,'robot_box':self.box_contact.last_contacts,'panel_furniture':self.furniture_contact.last_contacts})
        # Exact primitive tangency and actual active solver contact both count,
        # including zero-force grazes and positive-separation solver support.
        self.last=row;self.channels.update(REQUIRED_CHANNELS);return row
    def sample(self,dt):
        if self.error:return
        try:self.history.update(self.read(dt),dt)
        except Exception as exc:
            self.error=f'{type(exc).__name__}: {exc}';self.channels.clear();self.history.failure='measurement_unavailable'
    def result(self):
        result=evaluate(self.history.measurement(),(self.channels&VALIDATED_RUNTIME_CHANNELS) if self.error is None else frozenset())
        return dict(result,measurement_error=self.error,physics_samples=self.history.samples,physics_device=str(self.base.device))
