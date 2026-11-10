"""Portable passive spring vises, free carrier and crosswise hidden-ballast bar.

The rubber-footed clamp is a free body, not attached to the table. Only the
robot has actuators. Known pickup geometry is a demo assumption, not perception.
"""
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from dual_arm_demo.env import DualArmEnv, scene as robot_scene

CASES=('train_a','train_b','train_c','test','wrong_clamp','wrong_com')
COLORS=('.12 .38 .70 1','.46 .24 .65 1')
TOOL_START=np.array([[.03,-.26,.80],[.13,.13,.80]])
STATION=np.array([.16,.02,.80])
CARRY=np.array([.085,.065,.097])
OPEN=np.array([-.10,-.10,.085])
PANEL_START=np.array([.18,.40,.812])
PANEL_WIDTH=.040
PANEL_MIN=-.045
PANEL_END=.255
PANEL_Z=.890
PANEL_GRASP=np.array([0,.145,.016])
LOAD_START=np.array([-.04,-.18,.825])
LOAD_OFFSET=.010
EDGE_Y=.230
PAD_MU=1.2


def tool_parameters(seed):
    r=np.random.default_rng(np.random.SeedSequence([seed,7301]))
    values=[r.uniform(650,700),r.uniform(110,130)];r.shuffle(values)
    return values


def describe(case):
    ids=[0] if case=='train_a' else [1] if case=='train_b' else [] if case=='train_c' else [0,1]
    pickups=TOOL_START.copy()
    if not case.startswith('train'):pickups[0,:2]=[.03,-.37]  # Clear the shared two-arm pickup zone.
    return dict(training=case.startswith('train'),balance_training=case=='train_c',tools=ids,
                tool_pickups=pickups.tolist(),station=STATION.tolist(),carry_offset=CARRY.tolist(),
                right_yaw_offset=(0. if case in ('train_a','train_b') else float(np.pi)),
                observation_park=([-.20,.35,1.17] if case in ('train_a','train_b') else [.42,.45,1.17]),
                parking_station=[.40,.57,.8],opening_offset=OPEN.tolist(),panel_pickup=PANEL_START.tolist(),panel_grasp=PANEL_GRASP.tolist(),
                panel_width=PANEL_WIDTH,workpiece_width=.024 if case.startswith('train') else PANEL_WIDTH,initial_gap=.012,panel_z=PANEL_Z,panel_top=PANEL_Z+.012,
                payload_pickup=LOAD_START.tolist(),grasp_half_span=.110,grasp_offset=LOAD_OFFSET,payload_half_height=.025,
                target_xy=[.16,.02+EDGE_Y] if case!='train_c' else [.16,.14],
                support_top=PANEL_Z+.012 if case!='train_c' else .96,
                friction=PAD_MU,base_edge=.080)


def geom(body,name,pos,size,rgba,mass,**attrs):
    return ET.SubElement(body,'geom',name=name,type='box',pos=' '.join(map(str,pos)),size=' '.join(map(str,size)),rgba=rgba,mass=str(mass),**attrs)


def finish_castings(root):
    """Small visible chamfers; the original boxes remain contact proxies.

    These meshes add no mass or contacts. The flat gripping/support surfaces
    coincide with the physical boxes; only the millimetre-scale edges differ.
    """
    from itertools import product
    from scipy.spatial import ConvexHull
    asset=root.find('asset')
    ET.SubElement(asset,'material',name='cast_finish',specular='.35',shininess='.3')
    for body in root.iter('body'):
        for g in list(body.findall('geom')):
            name=g.get('name','')
            if not name.startswith(('foot_','base_','fixed_frame_','fixed_','pad_','open_','rubber_')):continue
            if g.get('type')!='box' or g.get('contype')=='0':continue
            size=np.fromstring(g.get('size'),sep=' ');bevel=min(.003,float(size.min())*.2)
            vertices=[]
            for signs in product((-1,1),repeat=3):
                for axis in range(3):
                    v=size-bevel;v[axis]=size[axis];vertices.append(v*np.array(signs))
            vertices=np.array(vertices);hull=ConvexHull(vertices);faces=[]
            for face,equation in zip(hull.simplices,hull.equations):
                a,b,c=vertices[face]
                if np.dot(np.cross(b-a,c-a),equation[:3])<0:face=face[::-1]
                faces.extend(face.tolist())
            ET.SubElement(asset,'mesh',name=name+'_mesh',vertex=' '.join(map(str,vertices.flatten())),face=' '.join(map(str,faces)))
            color=g.get('rgba');g.set('rgba',' '.join(color.split()[:3])+ ' 0')
            ET.SubElement(body,'geom',name=name+'_finish',type='mesh',mesh=name+'_mesh',pos=g.get('pos'),
                          rgba=color,material='cast_finish',mass='0',contype='0',conaffinity='0')


def scene(seed,case):
    if case not in CASES:raise ValueError(case)
    xml,_=robot_scene(0,'balance');root=ET.fromstring(xml);world=root.find('worldbody')
    root.set('model','portable_clamps_and_crosswise_bar');root.find('option').set('impratio','10')
    root.find('default/geom').set('solref','.004 1');root.find('default/geom').set('solimp','.99 .999 .0001')
    for a in root.find('actuator'):
        if '_finger_' in a.get('name',''):a.set('kp','1200');a.set('forcerange','-40 40')
    for j in root.iter('joint'):
        if '_finger_' in j.get('name',''):j.set('damping','20')
    world.find("body[@name='right_base']").set('pos','-.32 .30 .8')
    if case not in ('train_a','train_b'):
        # Opposed arms keep their elbows apart during crosswise two-hand placement.
        if case!='train_c':
            world.find("body[@name='left_base']").set('pos','-.40 -.15 .8')
        right=world.find("body[@name='right_base']")
        right.set('pos','.55 0 .8');right.set('quat','0 0 0 1')
    for n in ('bar','post','cradle_left','cradle_right'):world.remove(world.find(f"body[@name='{n}']"))
    spec=describe(case)
    for i in spec['tools']:
        color=COLORS[i]
        body=ET.SubElement(world,'body',name=f'tool_{i}',pos=' '.join(map(str,spec['tool_pickups'][i])))
        ET.SubElement(body,'freejoint',name=f'tool_joint_{i}')
        # Steel sole and four visible rubber feet; the whole vise is free.
        geom(body,f'foot_{i}',[-.01,0,.016],[.15,.10,.008],color,2.6,friction='3 .08 .002',condim='6',priority='1')
        for n,(x,y) in enumerate(((-.13,-.075),(-.13,.075),(.11,-.075),(.11,.075))):
            geom(body,f'rubber_{i}_{n}',[x,y,.004],[.025,.025,.004],'.10 .12 .14 1',.10,friction='3 .08 .002',condim='6',priority='1')
        geom(body,f'base_{i}',[0,0,.051],[.045,.080,.027],color,.15,friction='.04 .001 .0001',priority='1')
        geom(body,f'fixed_{i}',[.032,0,.097],[.012,.022,.019],color,.07,friction=f'{PAD_MU} .001 .0001',priority='1')
        ET.SubElement(root.find('asset'),'material',name=f'fiducial_mat_{i}',emission='.5')
        geom(body,f'fiducial_{i}',[.032,0,.1166],[.009,.017,.0006],'.10 .75 .01 1' if i==0 else '.75 .01 .05 1',0,material=f'fiducial_mat_{i}',contype='0',conaffinity='0')
        # Fixed frame is structural: the robot grasps this housing itself.
        geom(body,f'fixed_frame_{i}',[.085,.065,.070],[.020,.024,.046],color,.18,
             friction='1.5 .10 .15',condim='6',priority='2',solref='.008 1')
        steel='.48 .53 .59 1';rubber='.12 .14 .16 1'
        for n,y in enumerate((-.025,.025)):
            ET.SubElement(body,'geom',name=f'guide_{i}_{n}',type='cylinder',
                fromto=f'-.12 {y} .06 .065 {y} .06',size='.005',rgba=steel,mass='.025',contype='0',conaffinity='0')
        geom(body,f'fixed_liner_{i}',[.0195,0,.097],[.0005,.022,.018],rubber,0,contype='0',conaffinity='0')
        # A flush cover contains the return spring; no decorative grasp posts.
        geom(body,f'spring_cover_{i}',[-.04,.060,.075],[.075,.014,.006],steel,.015,contype='0',conaffinity='0')
        jaw=ET.SubElement(body,'body',name=f'jaw_{i}',pos='-.004 0 0')
        ET.SubElement(jaw,'joint',name=f'spring_{i}',type='slide',axis='1 0 0',range='-.065 0',stiffness=str(tool_parameters(seed)[i]),springref='0',damping='2',armature='.001')
        ET.SubElement(root.find('contact'),'exclude',body1=f'tool_{i}',body2=f'jaw_{i}')
        geom(jaw,f'pad_{i}',[0,0,.097],[.012,.022,.019],color,.08,friction=f'{PAD_MU} .001 .0001',priority='1')
        geom(jaw,f'link_{i}',[0,-.05,.039],[.008,.06,.007],steel,.02)
        geom(jaw,f'cross_{i}',[-.048,-.10,.039],[.056,.010,.007],steel,.015)
        geom(jaw,f'pull_link_{i}',[-.096,-.10,.055],[.01,.012,.013],steel,.015)
        geom(jaw,f'open_{i}',[-.096,-.10,.074],[.020,.025,.020],rubber,.04,
             friction='1.5 .08 .06',condim='6',priority='2',solref='.008 1')
        geom(jaw,f'moving_liner_{i}',[.0125,0,.097],[.0005,.022,.018],rubber,0,contype='0',conaffinity='0')
    # The inherited robot API requires a bar joint. In C it is the measured
    # object itself; in other scenes it is the free workpiece/carrier.
    if case=='train_c':
        add_load(world,'bar','bar_joint',seed,'C')
        post=ET.SubElement(world,'body',name='post',pos='.16 .14 .88')
        geom(post,'post_geom',[0,0,0],[.012,.050,.08],'.12 .17 .22 1',0)
    else:
        board=ET.SubElement(world,'body',name='bar',pos=' '.join(map(str,PANEL_START)))
        ET.SubElement(board,'freejoint',name='bar_joint')
        end=.17 if case.startswith('train') else PANEL_END
        geom(board,'bar_geom',[0,(PANEL_MIN+end)/2,0],[.012 if case.startswith('train') else PANEL_WIDTH/2,(end-PANEL_MIN)/2,.012],'.16 .68 .60 1',.20)

        if not case.startswith('train'):
            # Public visual stripe moves with panel. No collision/support.
            geom(board,'edge_zone',[0,EDGE_Y,.0125],[.020,.015,.0005],'.95 .88 .20 1',0,contype='0',conaffinity='0')
            add_load(world,'payload','payload_joint',seed,'D')
    finish_castings(root)
    return ET.tostring(root,encoding='unicode'),None


def add_load(world,name,joint,seed,instance):
    r=np.random.default_rng(np.random.SeedSequence([seed,7302 if instance=='C' else 7303]))
    offset=float(r.choice([-1.,1.])*r.uniform(.068,.085))
    body=ET.SubElement(world,'body',name=name,pos=' '.join(map(str,LOAD_START)))
    ET.SubElement(body,'freejoint',name=joint)
    geom(body,'load_shell',[0,0,0],[.16,.025,.025],'.96 .56 .14 1',.08,friction='1.2 .02 .003',condim='6',priority='2')
    geom(body,'load_ballast',[offset,0,0],[.015,.020,.020],'0 0 0 0',.60,contype='0',conaffinity='0')



class PortableEnv(DualArmEnv):
    build_scene=staticmethod(scene)
    def __init__(self,seed=42,case='test',render=False,resolution=640):
        super().__init__(seed,render,resolution,case)
        self.case=case;self.stage='ready';self.phase='Ready — independent training / portable clamp testing'
        self.tool_observations=[];self.estimates=[];self.selected=None;self.payload_estimate=None;self.placement=None
        self.metrics=dict(tool_probes=0,tool_pickups=0,assemblies=0,load_pickups=0)
        self.observed=dict(panel_tilt_deg=0.,relative_tilt_deg=0.,edge_error_m=None,perpendicular_error_deg=None)
        self.failure_mode=None;self.released_at=None;self.max_panel_tilt=0.;self.max_relative_tilt=0.
        self.load_id=self.model.body('bar' if case=='train_c' else 'payload').id if case not in ('train_a','train_b') else None

    def public_spec(self):return describe(self.case)
    def public(self):
        return dict(estimates=self.estimates,selected=self.selected,estimate=self.payload_estimate,metrics=self.metrics,
                    observed=self.observed,outcome=self.reason,placement=self.placement,case=self.case)
    def probe_position(self,arm):
        qa,qb=self.data.qpos[self.finger_qids[arm]]
        axis=self.data.site_xmat[self.tips[arm]].reshape(3,3)[:,0]
        return self.data.site_xpos[self.tips[arm]].copy()+axis*(qb-qa)/2

    def check(self):
        if any(w.number for w in self.data.warning) or not np.isfinite(self.data.qpos).all():self.reason='numerical_instability';return
        if min(self.data.qpos[np.concatenate(self.finger_qids)])<-.002:self.reason='finger_limit_abort';return
        overload=any(np.linalg.norm(self.data.sensor(s+'_force').data)>80 for s in ('left','right'))
        self.force_time=self.force_time+.002 if overload else 0.
        if self.force_time>.1:self.reason='force_abort';return
        if self.data.time>360:self.reason='timeout';return
        if getattr(self, 'exploration_only', False):return
        if not hasattr(self,'stage') or self.stage!='released':return
        if self.released_at is None:self.released_at=float(self.data.time)
        elapsed=self.data.time-self.released_at
        robot={g for g in range(self.model.ngeom) if self.model.geom(g).name.startswith(('left_','right_'))}
        def touching(ids):
            out=set()
            for c in self.data.contact:
                for a,b in ((c.geom1,c.geom2),(c.geom2,c.geom1)):
                    if a in ids and b not in ids:out.add(b)
            return out
        panel={self.model.geom('bar_geom').id} if self.case!='train_c' else {self.model.geom('load_shell').id}
        pt=touching(panel)
        R=self.data.xmat[self.bar_id].reshape(3,3)
        tilt=float(np.arccos(np.clip(R[2,2],-1,1)))
        if self.case in ('train_a','train_b'):
            pads={self.model.geom(f'{n}_{self.selected}').id for n in ('fixed','pad')}
            allowed=pads|{self.model.geom(f'base_{self.selected}').id}
            good=pads.issubset(pt) and not pt-allowed and tilt<.06
        else:
            Q=self.data.xmat[self.load_id].reshape(3,3)
            load={self.model.geom('load_shell').id}
            lt=touching(load)
            if self.case=='train_c':
                relative=tilt;good=lt=={self.model.geom('post_geom').id} and tilt<.06
            else:
                relative=float(np.arccos(np.clip((R.T@Q)[2,2],-1,1)))
                local=R.T@(self.data.xpos[self.load_id]-self.data.xpos[self.bar_id])
                edge_error=abs(float(local[1])-EDGE_Y)
                perpendicular=float(np.arcsin(np.clip(abs(np.dot(R[:,1],Q[:,0])),0,1)))
                self.observed.update(edge_error_m=edge_error,perpendicular_error_deg=float(np.degrees(perpendicular)))
                pads={self.model.geom(f'{n}_{self.selected}').id for n in ('fixed','pad')}
                allowed=pads|{self.model.geom(f'base_{self.selected}').id}|load
                good=(pads.issubset(pt) and not pt-allowed and lt.issubset(panel) and bool(lt) and tilt<.06 and relative<.06 and edge_error<.015 and perpendicular<.08)
                self.observed['panel_tilt_deg']=float(np.degrees(tilt))
                if not (pt|lt)&robot:
                    self.max_panel_tilt=max(self.max_panel_tilt,tilt)
                    if self.failure_mode is None and tilt>.12:self.failure_mode='fixture_slip'
            self.observed['relative_tilt_deg']=float(np.degrees(relative))
            if not (pt|lt)&robot:
                self.max_relative_tilt=max(self.max_relative_tilt,relative)
                if self.failure_mode is None and relative>.25:self.failure_mode='object_tip'
            load_dof=self.model.jnt_dofadr[self.model.joint('bar_joint' if self.case=='train_c' else 'payload_joint').id]
            good=good and np.linalg.norm(self.data.qvel[load_dof:load_dof+6])<.025
        if self.selected is not None:
            bodies={self.model.body(f'tool_{self.selected}').id,self.model.body(f'jaw_{self.selected}').id}
            tool_geoms={g for g in range(self.model.ngeom) if self.model.geom_bodyid[g] in bodies}
            good=good and not touching(tool_geoms)&robot
        dof=self.model.jnt_dofadr[self.model.joint('bar_joint').id]
        good=good and not pt&robot and np.linalg.norm(self.data.qvel[dof:dof+6])<.025
        self.stable=self.stable+.002 if good else 0.
        if elapsed>5 and self.stable>2:self.success=True;self.reason='success';self.phase='Success — stable, hands clear, target geometry satisfied'
        elif elapsed>10 and self.failure_mode:self.reason=self.failure_mode;self.phase='Physical failure — '+self.failure_mode.replace('_',' ')
