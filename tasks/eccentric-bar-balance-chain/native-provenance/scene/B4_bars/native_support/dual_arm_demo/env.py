"""Articulated Panda arms supporting a free bar through contact only.

Actuated parallel fingers grasp through frictional contact. Native wrist sensors
measure child/parent reaction wrenches. Hidden CoM is evaluator-only.
"""
from copy import deepcopy
from pathlib import Path
import importlib.util
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from . import end_effectors

HOME = np.array([0., -.45, 0., -2.15, 0., 1.7, .7854])
DOWN = np.diag([1., -1., -1.])
START = np.array([[0., -.17, 1.06], [0., .17, 1.06]])
POST = np.array([.20, 0., 1.015])
BAR_Z = .964


def scene(seed, scene_kind="balance"):
    if scene_kind not in ("balance", "table_random", "table_visual"):
        raise ValueError("Unknown dual-arm scene")
    rng = np.random.default_rng(seed)
    offset = float(rng.choice([-1, 1]) * rng.uniform(.035, .085))
    root = ET.fromstring('''<mujoco model="dual_panda_balance">
      <compiler angle="radian" autolimits="true"/>
      <option timestep="0.002" integrator="implicitfast" cone="elliptic" iterations="100"/>
      <size nconmax="500" njmax="3000"/>
      <visual><global offwidth="960" offheight="720"/><quality shadowsize="2048"/></visual>
      <default><geom friction="1.1 .01 .001" solref=".008 1" solimp=".95 .99 .001"/><joint armature=".02"/></default>
      <asset><texture name="floor_tex" type="2d" builtin="checker" rgb1=".12 .15 .19" rgb2=".16 .19 .23" width="256" height="256"/><material name="floor_mat" texture="floor_tex" texrepeat="5 5"/></asset>
      <worldbody>
        <light pos="0 -1 3" dir="0 0 -1" diffuse=".8 .8 .8"/>
        <light pos="1 1 2" dir="-1 -1 -1" diffuse=".5 .5 .5"/>
        <geom name="floor" type="plane" size="3 3 .1" material="floor_mat"/>
        <geom name="table" type="box" pos="0 0 .76" size=".72 .82 .04" rgba=".28 .32 .38 1"/>
        <body name="post" pos=".20 0 .9075"><geom name="post_geom" type="box" size=".07 .012 .1075" rgba=".08 .12 .17 1"/></body>
        <body name="cradle_left" pos="0 -.25 .875"><geom name="cradle_left_geom" type="box" size=".07 .025 .075" rgba=".4 .45 .5 1"/></body>
        <body name="cradle_right" pos="0 .25 .875"><geom name="cradle_right_geom" type="box" size=".07 .025 .075" rgba=".4 .45 .5 1"/></body>
      </worldbody><actuator/><sensor/><contact/>
    </mujoco>''')
    bar_pos=np.array([0.,0.,BAR_Z]); yaw=0.
    if scene_kind in ('table_random','table_visual'):
        # Separate random stream: visible placement does not determine the hidden CoM.
        pose_rng=np.random.default_rng(np.random.SeedSequence([seed, 8127]))
        bar_pos=np.array([pose_rng.uniform(-.08,.01),pose_rng.uniform(-.045,.045),.814])
        yaw=pose_rng.uniform(-np.pi/7.2,np.pi/7.2)  # ±25 degrees, reachable workspace.
        world=root.find('worldbody')
        for side in ('left','right'): world.remove(world.find(f"body[@name='cradle_{side}']"))
    spec = importlib.util.find_spec('robosuite')
    assets = Path(spec.origin).parent/'models/assets/robots/panda'
    for side, y, color in [('left', -.42, '.1 .55 .95 1'), ('right', .42, '.95 .5 .1 1')]:
        robot = ET.parse(assets/'no_texture_robot.xml').getroot()
        for node in robot.iter():
            for attr in ('name', 'joint', 'mesh', 'material', 'site', 'body'):
                if attr in node.attrib:
                    node.set(attr, side+'_'+node.get(attr))
            if node.tag == 'mesh': node.set('file', str(assets/node.get('file')))
        for geom in robot.iter('geom'):
            if geom.get('name', '').endswith('_collision'): geom.set('rgba', '0 0 0 0')
        base = robot.find('worldbody/body')
        base.set('pos', f'-.40 {y} .8')
        hand = base.find(f'.//body[@name="{side}_right_hand"]')
        ET.SubElement(hand, 'site', name=side+'_wrist', size='.008', rgba=color)
        ET.SubElement(hand, 'geom', name=side+'_palm', type='box', pos='0 0 .045', size='.055 .025 .02', rgba=color)
        ET.SubElement(hand, 'site', name=side+'_tip', pos='0 0 .12', size='.004', rgba=color)
        for finger, sign in [('a', -1), ('b', 1)]:
            name=f'{side}_finger_{finger}'
            body=ET.SubElement(hand, 'body', name=name)
            ET.SubElement(body, 'joint', name=name, type='slide', axis=f'{sign} 0 0', range='0 .045', damping='4', armature='.001')
            ET.SubElement(body, 'geom', name=name+'_pad', type='box', pos=f'{sign*.008} 0 .11', size='.008 .016 .035', mass='.04', rgba='.18 .2 .23 1', friction='1.5 .01 .001')
            ET.SubElement(body, 'site', name=name+'_touch', type='box', pos=f'{sign*.001} 0 .11', size='.003 .017 .036', rgba='0 0 0 0')
            ET.SubElement(root.find('sensor'), 'touch', name=name+'_normal', site=name+'_touch')
        # Wrist inertia retained; fingers add real moving mass. No grasp weld.
        for typ in ('force', 'torque'):
            ET.SubElement(root.find('sensor'), typ, name=side+'_'+typ, site=side+'_wrist')
        root.find('worldbody').append(base)
        root.find('asset').extend(robot.find('asset'))
        root.find('actuator').extend(robot.find('actuator'))
    # Append after all fourteen arm motors so their existing indices stay stable.
    for side in ('left', 'right'):
        for finger in ('a', 'b'):
            name=f'{side}_finger_{finger}'
            ET.SubElement(root.find('actuator'), 'position', name=name, joint=name, kp='400', ctrlrange='0 .045', forcerange='-12 12')
    # A 0.1 kg shell plus concealed 0.5 kg ballast, merged analytically.
    mass, shell, ballast = .6, .1, .5
    b = offset*mass/ballast
    inertia_shell = shell/3*np.array([.30**2+.014**2, .035**2+.014**2, .035**2+.30**2])
    inertia_ballast = ballast/3*np.array([.015**2+.01**2, .025**2+.01**2, .025**2+.015**2])
    I = inertia_shell + inertia_ballast + np.array([1,0,1])*(shell*offset**2+ballast*(b-offset)**2)
    bar = ET.SubElement(root.find('worldbody'), 'body', name='bar', pos=' '.join(map(str,bar_pos)), quat=f'{np.cos(yaw/2)} 0 0 {np.sin(yaw/2)}')
    ET.SubElement(bar, 'freejoint', name='bar_joint')
    ET.SubElement(bar, 'inertial', mass=str(mass), pos=f'0 {offset} 0', diaginertia=' '.join(map(str,I)))
    ET.SubElement(bar, 'geom', name='bar_geom', type='box', size='.035 .30 .014', rgba='.72 .76 .8 1')
    ET.SubElement(bar, 'geom', name='center_mark', type='box', pos='0 0 .0145', size='.035 .002 .0005', rgba='.15 .2 .25 1', contype='0', conaffinity='0', mass='0')
    from scipy.spatial.transform import Rotation
    for name, p, look in [('agentview',[1.3,-1.55,2.0],[-.1,0,1.05]), ('wristview',[.65,0,2.4],[0,0,.95])]:
        z=np.array(p)-look; z/=np.linalg.norm(z)
        x=np.cross([0,0,1],z); x/=np.linalg.norm(x); yy=np.cross(z,x)
        q=Rotation.from_matrix(np.column_stack([x,yy,z])).as_quat()[[3,0,1,2]]
        ET.SubElement(root.find('worldbody'),'camera',name=name,pos=' '.join(map(str,p)),quat=' '.join(map(str,q)),fovy='48')
    return ET.tostring(root,encoding='unicode'), offset


class DualArmEnv:
    build_scene = staticmethod(scene)

    def __init__(self, seed=0, render=False, resolution=640, scene_kind="balance", end_effector="parallel_gripper"):
        self.scene_kind=scene_kind
        self.xml, self.true_offset = self.build_scene(seed, scene_kind)
        self.xml = end_effectors.configure(self.xml, end_effector)
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        self.ik = mujoco.MjData(self.model)
        self.resolution, self.renderer = resolution, None
        self.balance_estimate=None; self.balance_history=[]
        self.visual_observation=None; self.visual_history=[]; self.visual_frame=None
        self.visual_selection=None; self.visual_initial_frame=None; self.visual_captures=[]
        self.render_enabled = render
        self.ids = [[self.model.joint(f'{s}_joint{i}').id for i in range(1,8)] for s in ('left','right')]
        self.qids = [self.model.jnt_qposadr[j] for j in self.ids]
        self.dids = [self.model.jnt_dofadr[j] for j in self.ids]
        self.tips = [self.model.site(s+'_tip').id for s in ('left','right')]
        self.wrists = [self.model.site(s+'_wrist').id for s in ('left','right')]
        self.bar_id = self.model.body('bar').id
        self.bar_q = self.model.jnt_qposadr[self.model.joint('bar_joint').id]
        self.targets = START.copy()
        if scene_kind=="table_visual": self.targets=np.array([[-.23,-.42,1.13],[-.23,.42,1.13]])
        if end_effector == 'allegro':
            # The longer open hand must clear the bar/cradles before empty-tool tare.
            self.targets = np.array([[-.18, -.30, 1.15], [-.18, .30, 1.15]])
        self.yaw_targets = np.zeros(2)
        self.grip_targets = np.full(2, .045)
        end_effectors.initialize(self, end_effector)
        self.qtargets = []
        for q in self.qids: self.data.qpos[q] = HOME
        mujoco.mj_forward(self.model,self.data)
        for a in range(2):
            q = self.solve_ik(a, self.targets[a], iterations=300)
            self.data.qpos[self.qids[a]] = q
            self.qtargets.append(q)
            mujoco.mj_forward(self.model,self.data)
        self.success=False; self.reason=None; self.stable=0.; self.steps=0; self.force_time=0.
        self.phase='Ready — grasp the bar and measure wrist force / torque'
        self.estimate=None; self.samples=[]; self.baseline=np.zeros((2,6))
        self.history=[]
        for _ in range(80): self.step_targets(self.targets, check=False)
        self.baseline=self.wrenches_world().copy()
        self.data.time=0.; self.steps=0; self.history=[]

    def solve_ik(self, arm, target, iterations=12):
        from scipy.spatial.transform import Rotation
        # IK needs only robot proprioception. Keep free-object coordinates out
        # of its working state, even though they cannot affect arm kinematics.
        for qi in self.qids+self.finger_qids:
            self.ik.qpos[qi]=self.data.qpos[qi]
        qi, di, site=self.qids[arm],self.dids[arm],self.tips[arm]
        jp=np.zeros((3,self.model.nv)); jr=jp.copy()
        for _ in range(iterations):
            mujoco.mj_forward(self.model,self.ik)
            rot=self.ik.site_xmat[site].reshape(3,3)
            err=np.r_[target-self.ik.site_xpos[site], .5*Rotation.from_matrix(Rotation.from_euler("z",self.yaw_targets[arm]).as_matrix()@DOWN@rot.T).as_rotvec()]
            if np.linalg.norm(err)<1e-5: break
            mujoco.mj_jacSite(self.model,self.ik,jp,jr,site)
            J=np.vstack([jp[:,di], .5*jr[:,di]])
            dq=J.T@np.linalg.solve(J@J.T+np.eye(6)*.0001,err)
            self.ik.qpos[qi]+=np.clip(dq,-.08,.08)
            self.ik.qpos[qi]=np.clip(self.ik.qpos[qi],self.model.jnt_range[self.ids[arm],0]+.01,self.model.jnt_range[self.ids[arm],1]-.01)
        return self.ik.qpos[qi].copy()

    def wrenches_world(self):
        result=[]
        for side,site in zip(('left','right'),self.wrists):
            R=self.data.site_xmat[site].reshape(3,3)
            result.append(np.r_[R@self.data.sensor(side+'_force').data,R@self.data.sensor(side+'_torque').data])
        return np.array(result)


    def step_targets(self, targets, check=True, grips=None, yaws=None):
        if self.reason: return
        targets=np.asarray(targets,dtype=float)
        if targets.shape!=(2,3) or not np.isfinite(targets).all():
            raise ValueError('Expected finite (2,3) Cartesian targets')
        if grips is not None:
            grips=np.asarray(grips, dtype=float)
            if grips.shape!=(2,) or not np.isfinite(grips).all() or np.any(grips<0) or np.any(grips>.045):
                raise ValueError('Expected two jaw targets in [0, .045] metres')
            self.grip_targets=grips.copy()
        if yaws is not None:
            yaws=np.asarray(yaws,dtype=float)
            if yaws.shape!=(2,) or not np.isfinite(yaws).all():
                raise ValueError('Expected two finite yaw targets in radians')
            self.yaw_targets=yaws.copy()
        self.targets=targets.copy()
        self.qtargets=[self.solve_ik(a,t) for a,t in enumerate(self.targets)]
        for _ in range(25):
            for a in range(2):
                qi,di=self.qids[a],self.dids[a]
                tau=550*(self.qtargets[a]-self.data.qpos[qi])-45*self.data.qvel[di]+self.data.qfrc_bias[di]
                lim=np.array([80]*5+[12]*2)
                self.data.ctrl[a*7:(a+1)*7]=np.clip(tau,-lim,lim)
            end_effectors.apply(self)
            mujoco.mj_step(self.model,self.data)
            if check: self.check()
            if self.reason: break
        mujoco.mj_forward(self.model,self.data)
        mujoco.mj_rnePostConstraint(self.model,self.data)
        self.steps+=1
        if check:
            self.history.append(dict(time=round(self.data.time,3), loads=self.loads().tolist()))
            self.history=self.history[-160:]

    def finger_forces(self):
        if self.hands: return self.hands.forces()
        return np.array([[float(self.data.sensor(f'{s}_finger_{f}_normal').data[0]) for f in ('a','b')] for s in ('left','right')])

    def openings(self):
        if self.hands: raise ValueError('Articulated hands have joint angles, not a scalar jaw opening')
        return np.array([self.data.qpos[qi].sum() for qi in self.finger_qids])

    def loads(self):
        # Reaction points from child to parent: world +Z reports supported load.
        return (self.wrenches_world()-self.baseline)[:,2]

    def check(self):
        if any(w.number for w in self.data.warning) or not np.isfinite(self.data.qpos).all():
            self.reason='numerical_instability'; return
        overloaded=any(np.linalg.norm(self.data.sensor(s+'_force').data)>80 for s in ('left','right'))
        self.force_time=self.force_time+.002 if overloaded else 0.
        if self.force_time>=.1: self.reason='force_abort'; return
        if self.data.time>90: self.reason='timeout'; return
        if self.data.xpos[self.bar_id,2]<.79: self.reason='bar_fell'; return
        if getattr(self, 'exploration_only', False):return
        g=self.model.geom('bar_geom').id; post=self.model.geom('post_geom').id
        touched=[]
        for c in self.data.contact:
            if g in (c.geom1,c.geom2): touched.append(c.geom2 if c.geom1==g else c.geom1)
        free=self.model.jnt_dofadr[self.model.joint('bar_joint').id]
        flat=self.data.xmat[self.bar_id].reshape(3,3)[2,2]>.995
        good=(post in touched and all(t==post for t in touched) and flat and np.linalg.norm(self.data.qvel[free:free+6])<.02)
        self.stable=self.stable+.002 if good else 0.
        if self.stable>=2.: self.success=True; self.reason='success'; self.phase='Balanced — hands released for 2 seconds'

    def wrist_positions(self):
        return self.data.site_xpos[self.wrists].copy()

    def rgbd(self,camera_name="wristview"):
        # Fixed calibrated camera only; no object IDs or segmentation buffer.
        rgb=self.image(camera_name)
        self.renderer.enable_depth_rendering()
        try: depth=self.renderer.render().copy()
        finally: self.renderer.disable_depth_rendering()
        camera=self.model.camera(camera_name).id
        return dict(rgb=rgb,depth=depth,fovy=float(self.model.cam_fovy[camera]),
                    camera_position=self.data.cam_xpos[camera].copy(),
                    camera_rotation=self.data.cam_xmat[camera].reshape(3,3).copy())

    def anchor_visual_grasp(self):
        """Associate the last camera pose with current wrist FK at grasp time."""
        self._visual_wrists=self.wrist_positions().mean(axis=0)

    def observe_bar(self):
        if self.scene_kind!='table_visual':
            return self.data.xpos[self.bar_id].copy(),self.data.xmat[self.bar_id].reshape(3,3).copy()
        from .vision import perceive_bar
        frame=self.rgbd()
        wrists=self.wrist_positions().mean(axis=0)
        translation=None if self.visual_observation is None else wrists-self._visual_wrists
        frame["tracking_translation"]=np.zeros(3) if translation is None else translation.copy()
        observation,overlay=perceive_bar(frame,self.visual_observation,translation)
        self._visual_wrists=wrists
        self.visual_captures.append(frame)
        if self.visual_selection is None:
            self.visual_selection=observation;self.visual_initial_frame=overlay.copy()
        self.visual_observation=observation;self.visual_frame=overlay
        self.visual_history.append(dict(step=self.steps,**observation))
        return np.array(observation['center']),np.array(observation['rotation'])

    def image(self,camera='agentview'):
        if self.renderer is None: self.renderer=mujoco.Renderer(self.model,height=self.resolution,width=self.resolution)
        self.renderer.update_scene(self.data,camera=camera)
        return self.renderer.render().copy()

    def close(self):
        if self.renderer is not None: self.renderer.close()
