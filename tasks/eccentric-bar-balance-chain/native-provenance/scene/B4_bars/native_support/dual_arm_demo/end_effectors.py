"""Optional, pinned Allegro assets and robot-only hand control.

The default XML is returned unchanged. The Allegro mount uses tool +Z along
the fingers, +X along finger flexion. Sensor sites cover the distal capsules;
MuJoCo touch reports scalar normal contact force, not a tactile image or shear.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

ASSETS = Path(__file__).parent / 'assets' / 'wonik_allegro'
SIDES = ('left', 'right')
FINGERS = ('index', 'middle', 'ring', 'thumb')
CODES = ('ff', 'mf', 'rf', 'th')
OPEN = np.array([0., 0., 0., 0.] * 3 + [.3, 0., 0., 0.])
PRESETS = {
    'open': OPEN,
    'curl': np.array([0., .8, .8, .6] * 3 + [1.2, .4, .6, .5]),
    'pinch': np.array([0., .6, .6, .4] + [0., 0., 0., 0.] * 2 + [1.2, .3, .5, .5]),
}


def configure(xml, kind):
    if kind == 'parallel_gripper':
        return xml
    if kind != 'allegro':
        raise ValueError('Unknown end effector')
    root = ET.fromstring(xml)
    for side in SIDES:
        mount = root.find(f".//body[@name='{side}_right_hand']")
        removed = set()
        for node in list(mount):
            if node.get('name') in (side+'_palm', side+'_finger_a', side+'_finger_b'):
                removed.update(n.get('name') for n in node.iter() if n.get('name'))
                mount.remove(node)
        # Original jaw-specific contact pairs have no physical counterpart on
        # this hand. Use native hand/object collision instead of remapping them.
        for section in ('actuator', 'sensor', 'contact'):
            for node in list(root.find(section)):
                if any(value in removed for value in node.attrib.values()):
                    root.find(section).remove(node)
        donor = ET.parse(ASSETS / f'{side}_hand.xml').getroot()
        prefix = side+'_allegro_'
        for mesh in donor.findall('asset/mesh'):
            mesh.set('name', mesh.get('name', Path(mesh.get('file')).stem))
            mesh.set('file', str((ASSETS/'assets'/mesh.get('file')).resolve()))
        for node in donor.iter():
            for attr in ('name', 'class', 'childclass', 'mesh', 'material', 'joint', 'body1', 'body2'):
                if attr in node.attrib:
                    node.set(attr, prefix+node.get(attr))
        defaults = donor.find('default/default')
        # Explicitly isolate inherited robot defaults from the imported hand.
        defaults.find('joint').set('armature', '.001')
        defaults.find('geom').set('friction', '1 .005 .0001')
        defaults.find('geom').set('solref', '.008 1')
        defaults.find('geom').set('conaffinity', '3')
        defaults.find('position').set('forcerange', '-.5 .5')
        root.find('default').append(defaults)
        root.find('asset').extend(donor.find('asset'))
        palm = donor.find('worldbody/body')
        # Palm extends 95 mm behind the finger bases; mount above the flange
        # instead of burying that palm in the Panda wrist.
        palm.set('pos', '0 0 .10')
        palm.set('quat', '1 0 0 0')
        for i, geom in enumerate(palm.iter('geom')):
            geom.set('name', prefix+f'geom_{i}')
        for code in CODES:
            tip = palm.find(f".//body[@name='{prefix}{code}_tip']")
            thumb = code == 'th'
            ET.SubElement(tip, 'site', name=prefix+code+'_touch', type='capsule',
                          pos='0 0 .035' if thumb else '0 0 .019',
                          size='.0121 .0081' if thumb else '.0121 .0101', rgba='0 0 0 0')
            ET.SubElement(root.find('sensor'), 'touch', name=prefix+code+'_normal', site=prefix+code+'_touch')
        mount.append(palm)
        # Fixed grasp reference: does not move when individual fingers bend.
        mount.find(f"site[@name='{side}_tip']").set('pos', '.035 0 .19')
        root.find('actuator').extend(donor.find('actuator'))
        root.find('contact').extend(donor.find('contact'))
    # Keep the first fourteen motors stable; imported actuators follow them.
    return ET.tostring(root, encoding='unicode')


class AllegroHands:
    def __init__(self, model, data):
        self.model, self.data = model, data
        self.qids = [[model.jnt_qposadr[model.joint(f'{s}_allegro_{f}j{k}').id]
                      for f in CODES for k in range(4)] for s in SIDES]
        self.aids = [[model.actuator(f'{s}_allegro_{f}a{k}').id
                      for f in CODES for k in range(4)] for s in SIDES]
        self.ranges = model.actuator_ctrlrange[self.aids[0]].copy()
        self.targets = np.tile(OPEN, (2, 1))
        for ids in self.qids:
            data.qpos[ids] = OPEN

    def set(self, arm, values):
        values = np.asarray(values, dtype=float)
        if (arm not in (0, 1) or values.shape != (16,) or not np.isfinite(values).all()
                or np.any(values < self.ranges[:, 0]) or np.any(values > self.ranges[:, 1])):
            raise ValueError('Expected 16 finite Allegro joint targets within their joint limits')
        self.targets[arm] = values

    def apply(self):
        for i in range(2):
            self.data.ctrl[self.aids[i]] = self.targets[i]

    def forces(self):
        return np.array([[float(self.data.sensor(f'{s}_allegro_{f}_normal').data[0])
                          for f in CODES] for s in SIDES])

    def state(self, arm):
        return dict(kind='allegro', fingers=list(FINGERS),
                    joint_names=[f'{finger}_{k}' for finger in FINGERS for k in range(4)],
                    joint_positions_rad=self.data.qpos[self.qids[arm]].tolist(),
                    joint_targets_rad=self.targets[arm].tolist(),
                    joint_limits_rad=self.ranges.tolist(),
                    fingertip_normal_n=self.forces()[arm].tolist(),
                    presets={k: v.tolist() for k, v in PRESETS.items()},
                    sensor_description='Simulated distal-tip normal contact force (N); no shear or tactile image')


def initialize(env, kind):
    env.end_effector = kind
    env.hands = AllegroHands(env.model, env.data) if kind == 'allegro' else None
    if env.hands:
        env.finger_qids = env.hands.qids
    else:
        env.finger_qids = [[env.model.jnt_qposadr[env.model.joint(f'{s}_finger_{f}').id]
                            for f in ('a', 'b')] for s in SIDES]
        for ids in env.finger_qids:
            env.data.qpos[ids] = .045


def apply(env):
    if getattr(env, 'hands', None):
        env.hands.apply()
    else:
        env.data.ctrl[14:] = np.repeat(env.grip_targets, 2)
