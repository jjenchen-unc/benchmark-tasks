# Card: cards/B1_wrench.md (B1 wrench + thumb bolt), revised 2026-10-28; rulings in cards/DECISIONS-2026-10-28.md.
# Robot: the repo's two-Panda B4 robot (portable_demo.env.scene(.., 'test') stripped to the robot, as examples/toy_drawer.py),
# stock Franka hand (fingers kp 1200, +-40 N). Actions are data.ctrl rows (nu = 18): 14 arm joint torques (residual;
# step() adds the arm gravity/bias term, so zero control floats the arms) and 4 finger position targets (0 .. .045 m).
# The left arm does all the work; the right arm holds its start pose.
# One sentence: the robot tightens or loosens hex nuts with stamped open-end wrenches and screws knurled thumb bolts into an
# aluminium cover; whether a nut is 13 mm or 1/2 in and a bolt M8x1.25 or 5/16-18 cannot be seen (G), and the wrong wrench
# or bolt turned hard rounds the nut or galls the aluminium thread (M/P), so an agent that feels the fit first picks right
# and breaks nothing.
# Mechanisms (all read only simulator state; no teleports, no object actuators):
#  - nut: fixed base -> `turn` hinge (frictionloss written each tick: run torque + seat stiffness, or breakaway when seized)
#    -> `head` hinge held by a joint equality. nut_strip: wrench clearance in [0.15 mm, 0.155 AF) and wrench->nut contact
#    torque above tau_strip = tau_s0 (1 - c / (0.155 AF)) for 5 ticks releases the equality (head spins; sticky).
#  - thumb bolt: per hole a follower (hinge + slide, joint equality lift = -p turn / 2 pi). thread_engage welds the bolt to
#    the follower once its tip is in the hole, upright, and turned 15 deg clockwise; backing out past the start releases it.
#    thread_bind/gall: a mismatched pitch raises the thread friction after th_bind turns; thread torque above tau_dmg for
#    5 ticks galls the hole (sticky, then jams).
#  - W1 swing arm is clamped by its nut (joint equality) until the nut is backed off 60 deg; W2 cover likewise (weld).
# Torque family scale s_tau ~ 0.01 (all fastener torques are ~1/100 of a real 13 mm nut's); see DEVIATIONS.md.
import os, sys, struct, zlib, json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation

REPO = next(p / 'native_support' for p in Path(__file__).resolve().parents if (p / 'native_support').is_dir())
sys.path[:0] = [str(REPO), str(REPO / 'mujoco-box-demo')]
from dual_arm_demo.env import HOME, DOWN
from portable_demo.env import scene as portable_scene

BENCH_ID = 'B1_wrench'
SEEDS = (42, 43, 44)
LIM = np.array([80.] * 5 + [12.] * 2)
KP, KD = 550., 45.
POSTURE, POSTURE_W, NULL_GAIN = HOME.copy(), np.array([1., 1., 1., 1., 1., .3, 0.]), .05
VMAX = np.array([2.0] * 4 + [2.4] * 3)     # rad/s, below the real Panda limits (2.175 / 2.61)
TLIM = np.array([80.] * 4 + [12.] * 3)    # total joint torque incl. gravity compensation (real j5-j7: 12 N m)
IK_EVERY = 4                 # IK update period (ticks); joint targets are ramped linearly in between (no torque steps)
DT = .002
HOLD_S = 20.
TASK_A = 'A_seat_tighten'
RIGHT_PARK = np.array([-.3619, -.9369, -.635, -2.6051, -.5071, 1.7492, .1108])   # right hand parked at (.42, .30, 1.15)

# ---------------------------------------------------------------- geometry constants (metres)
TABLE_Z = .80
SEAT_Z = .89                 # nut bearing surface on every nut fixture
NUT_H = .0068                # ISO 4032 M8 nut height 6.8 mm (5/16 SAE nut 6.75 mm)
WR_DZ = .0040                # wrench mid-plane above the bearing surface when on a nut
WR_HT = .0025                # wrench half thickness (5 mm head)
RACK_Z = .8375               # wrench mid-plane on the rack
RG = .10                     # grasp radius on the handle (from the jaw centre)
AF = {'13mm': .0130, '1/2in': .0127}
SIZE_LABELS = ('13mm', '1/2in')
FIT = .00005                 # open-end opening above nominal (ISO 691 class, b13 smoke value)
WRENCH_DEFS = [('w12', '12 mm', .012 + FIT), ('w1_2', '1/2 in', .0127 + FIT), ('w13', '13 mm', .013 + FIT),
               ('w14', '14 mm', .014 + FIT), ('wadj', 'adjustable', None)]
RACK_Y = [-.40, -.35, -.30, -.25, -.20]
RACK_X = .10
WR_YAW = -np.pi / 2          # jaws open toward +x, handle toward -x
STRIP_CMIN, STRIP_CHI = .00015, .155
STRIP_TICKS = 5
FORCED_TAU, FORCED_OFF = .05, .009    # undersized wrench jammed on: N m and centre offset (blocked wrenches stop ~12.8 mm out)
HEX_SOLREF, HEX_SOLIMP = '.004 1', '.99 .999 .0001'   # steel-on-steel jaw contact (see DEVIATIONS: stiffness test)
TAU_RUN = .02                # free-running nut torque, N m
K_SEAT = .35 / np.radians(30.)   # seat stiffness: 0.35 N m reached 30 deg past snug
SNUG = .05                   # seat nuts start hand-snug (N m)
TAU_LOOSE = .03              # running torque after a seized nut breaks loose
BREAK_DEG = 2.               # rotation that counts as broken loose
RELEASE_DEG = 60.            # back-off that frees the W1 arm / W2 cover
CLAMP_RUN_DEG = 90.          # W3 clamp: free run before the jaw seats
WIN = (.30, .40)             # public tightening window (N m)

NUT_PHASE = dict(orange=33., purple=26., yellow=35., cyan=27., latch=32., fl1=28., fl2=34., clamp=31.)   # visible nut phases
MARKS = dict(S=dict(orange='.95 .45 .05 1', purple='.55 .2 .8 1', yellow='.95 .85 .1 1', cyan='.1 .8 .85 1'),
             T=dict(orange='.9 .1 .5 1', purple='.35 .75 .15 1', yellow='.2 .3 .9 1', cyan='.6 .4 .2 1'))

# deployment nut identity: S reuses the exploration nuts (same hidden values); T uses new nuts (transfer values)
# strip_torque is listed only for task A (lead #184, as B3): the rounding rule matters on W1-W3, but its value decides nothing there.
TARGETS = [
    dict(id=TASK_A, settings=['A'], cells=['nut_size_orange', 'nut_size_purple', 'strip_torque'], rules=['nut_strip']),
    dict(id='W1_latch_release', settings=['S', 'T-new'], cells=['nut_size_yellow'],
         rules=['nut_strip', 'arm_release'], target_objects=['swing_arm']),
    dict(id='W2_mixed_flange', settings=['S', 'T-new'],
         cells=['nut_size_yellow', 'nut_size_cyan'],
         rules=['nut_strip', 'cover_release'], target_objects=['flange_cover', 'shelf']),
    dict(id='W3_clamp_tighten', settings=['S', 'T-new'], cells=['nut_size_orange'],
         rules=['nut_strip'], target_objects=['clamp_jaw']),
    dict(id='W4_thumb_bolt_aluminium', settings=['S', 'T-new'], cells=['bolt_threads'],
         rules=['thread_engage', 'thread_gall', 'cover_clamp'], target_objects=['al_cover']),
]
TIDS = [t['id'] for t in TARGETS]
SCENE_OF = {None: 'explore', TASK_A: 'explore', 'W1_latch_release': 'W1', 'W2_mixed_flange': 'W2',
            'W3_clamp_tighten': 'W3', 'W4_thumb_bolt_aluminium': 'W4'}
OBJECT_BOX = ((-.75, -.85, .70), (.75, .85, 1.60))
WORKSPACE = dict(x=(-.30, .50), y=(-.50, .65), z=(.825, 1.27))    # left-hand tip reach box (start pose Z 1.259 m is inside)
TOOLS = [dict(name='move_hand', frame='world', orientation='fingers straight down, yaw only', workspace_m=WORKSPACE),
         dict(name='set_jaw', range_m=(0., .045)), dict(name='read_wrist_force_torque'), dict(name='read_finger_force')]


def other(v, pair): return pair[1 - pair.index(v)]


def v3(a): return ' '.join(f'{float(x):.6g}' for x in a)


# ---------------------------------------------------------------- parameters
BOLT_COLOURS = ('red', 'blue', 'green', 'white')
BOLT_RGBA = dict(red='.85 .15 .12 1', blue='.15 .35 .85 1', green='.15 .7 .25 1', white='.92 .92 .9 1')
PERMS = [('M8', 'M8', 'UNC', 'UNC'), ('M8', 'UNC', 'M8', 'UNC'), ('M8', 'UNC', 'UNC', 'M8'),
         ('UNC', 'M8', 'M8', 'UNC'), ('UNC', 'M8', 'UNC', 'M8'), ('UNC', 'UNC', 'M8', 'M8')]


def parameters(seed):
    r = np.random.default_rng(np.random.SeedSequence([seed, 1101]))
    # Binary size cells use seed bits so the three dev seeds cover both values (as the toy does); values are labels
    # ('13mm' / '1/2in'), the tool stamps in the public text being the same words.
    orange = SIZE_LABELS[seed % 2]
    p = dict(nut_size_orange=orange, nut_size_purple=other(orange, SIZE_LABELS),
             nut_size_yellow=SIZE_LABELS[(seed // 2) % 2], nut_size_cyan=other(SIZE_LABELS[(seed // 2) % 2], SIZE_LABELS),
             breakaway_yellow=round(float(r.uniform(350, 500)), 1), breakaway_cyan=round(float(r.uniform(350, 500)), 1),
             strip_torque=round(float(r.uniform(180, 250)), 1),          # tau_s0, mN m
             adjustable_slack=round(float(r.uniform(450, 650)), 1),      # um above 13 mm
             bolt_threads=list(PERMS[(seed * 5 + int(r.integers(6))) % 6]),
             al_gall_torque=round(float(r.uniform(120, 140)), 1),        # aluminium tau_dmg, mN m
             thread_run=round(float(r.uniform(10, 30)), 1), bind_turns=round(float(r.uniform(.7, .9)), 3),
             steel_gall_torque=round(float(r.uniform(500, 700)), 1), instance='S')
    return p


def transfer_parameters(params, seed):
    r = np.random.default_rng(np.random.SeedSequence([seed, 2202]))
    p = dict(params)
    for c in ('nut_size_orange', 'nut_size_purple', 'nut_size_yellow', 'nut_size_cyan'):
        p[c] = other(params[c], SIZE_LABELS)
    def resample(v, lo, hi, gap):
        while True:
            x = round(float(r.uniform(lo, hi)), 1)
            if abs(x - v) >= gap: return x
    p['breakaway_yellow'] = resample(params['breakaway_yellow'], 350, 500, 40)
    p['breakaway_cyan'] = resample(params['breakaway_cyan'], 350, 500, 40)
    p['strip_torque'] = resample(params['strip_torque'], 180, 250, 15)
    p['adjustable_slack'] = resample(params['adjustable_slack'], 450, 650, 50)
    p['al_gall_torque'] = resample(params['al_gall_torque'], 120, 140, 6)
    cur = PERMS.index(tuple(params['bolt_threads']))
    p['bolt_threads'] = list(PERMS[(cur + 1 + int(r.integers(5))) % 6])
    p['instance'] = 'T'
    return p


# ---------------------------------------------------------------- XML helpers
def sub(parent, tag, **kw): return ET.SubElement(parent, tag, **{k: (v3(v) if isinstance(v, (list, tuple, np.ndarray)) else str(v)) for k, v in kw.items()})


def box(b, name, pos, size, rgba='.45 .48 .52 1', **kw): return sub(b, 'geom', name=name, type='box', pos=pos, size=size, rgba=rgba, **kw)


def hexmesh(root, name, af, h, phase=30.):
    R = af / np.sqrt(3); pts = []
    for z in (0., h):
        for k in range(6):
            a = np.radians(phase + 60 * k); pts += [R * np.cos(a), R * np.sin(a), z]
    sub(root.find('asset'), 'mesh', name=name, vertex=pts)


def wrench_gap(name, p):
    g = dict((n, gap) for n, _, gap in WRENCH_DEFS)[name]
    return .013 + p['adjustable_slack'] * 1e-6 if g is None else g


def add_wrench(root, world, name, gap, origin, yaw, adjustable=False):
    """Single open-end wrench lying flat; origin = jaw centre (nut axis when engaged); jaws open toward body +y."""
    b = sub(world, 'body', name=name, pos=origin, euler=[0, 0, yaw]); sub(b, 'freejoint', name=name + '_free')
    steel = '.72 .73 .76 1'; kw = dict(contype='3', conaffinity='3', friction='.2 .005 .0001')
    tj = .0055 if adjustable else .0045
    Rg = gap / np.sqrt(3); y0, y1 = -Rg - .0015, Rg + .0015
    for s in (-1, 1):
        box(b, f'{name}_jaw{"p" if s > 0 else "m"}', [s * (gap / 2 + tj), (y0 + y1) / 2, 0], [tj, (y1 - y0) / 2, WR_HT], steel, mass='.02', **kw)
        a = -s * np.radians(15.); P = np.array([s * gap / 2, y1])
        c = P + np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]) @ np.array([s * tj, .002])
        box(b, f'{name}_flare{"p" if s > 0 else "m"}', [c[0], c[1], 0], [tj, .002, WR_HT], steel, euler=[0, 0, a], mass='.002', **kw)
    box(b, name + '_throat', [0, y0 - .002, 0], [gap / 2 + 2 * tj, .002, WR_HT], steel, mass='.01', **kw)
    L = .20 if adjustable else .16
    box(b, name + '_handle', [0, (y0 - .004 - L) / 2, 0], [.006, (L + y0 - .004) / 2, WR_HT], steel, mass='.16' if adjustable else '.06',
        friction='1 .005 .0001', contype='3', conaffinity='3')
    if adjustable:  # worm housing + worm (visual): the worm is not operable in this build (see DEVIATIONS)
        box(b, name + '_housing', [-(gap / 2 + 2 * tj) + .004, y0 - .008, 0], [.006, .006, WR_HT * 1.4], '.6 .6 .64 1', mass='.03', **kw)
        sub(b, 'geom', name=name + '_worm', type='cylinder', pos=[-(gap / 2 + 2 * tj) + .004, y0 - .008, .0045], size=[.004, .001],
            rgba='.5 .5 .55 1', contype='0', conaffinity='0', mass='0')
    # stamped size band (visual): colour coded by stamp so images can tell the tools apart
    band = {'w12': '.25 .25 .28 1', 'w1_2': '.85 .7 .2 1', 'w13': '.2 .45 .8 1', 'w14': '.75 .25 .2 1', 'wadj': '.3 .3 .3 1'}[name]
    box(b, name + '_stamp', [0, -.05, WR_HT + .0002], [.005, .012, .0002], band, contype='0', conaffinity='0', mass='0')
    return b


def nut_fixture(root, world, name, xy, af, mark, pillar=True):
    """Captive hex nut on a fixed stud: base -> turn hinge -> head hinge (held by `<name>_hold` until rounded)."""
    x, y = xy
    if pillar:
        # post top 6 mm below the nut and a 12 mm boss under the nut: a sagging wrench head clears the fixture
        box(world, name + '_pillar', [x, y, (TABLE_Z + SEAT_Z - .006) / 2], [.011, .011, (SEAT_Z - .006 - TABLE_Z) / 2], '.33 .36 .40 1')
        sub(world, 'geom', name=name + '_washer', type='cylinder', pos=[x, y, SEAT_Z - .003], size=[.006, .003], rgba='.7 .7 .72 1')
    sub(world, 'geom', name=name + '_stud', type='cylinder', pos=[x, y, SEAT_Z + .006], size=[.004, .006], rgba='.62 .62 .64 1',
        contype='0', conaffinity='0')
    hexmesh(root, name + '_hexm', af, NUT_H)
    base = sub(world, 'body', name=name + '_base', pos=[x, y, SEAT_Z], euler=[0, 0, np.radians(NUT_PHASE[name])])
    turn = sub(base, 'body', name=name + '_turn')
    sub(turn, 'joint', name=name + '_turn', type='hinge', axis='0 0 1', frictionloss='.02', damping='.0005', armature='.001',
        solreffriction='.002 1', solimpfriction='.9999 .9999 .0001')
    sub(turn, 'geom', name=name + '_core', type='sphere', size='.001', mass='.005', contype='0', conaffinity='0')
    head = sub(turn, 'body', name=name + '_head')
    sub(head, 'joint', name=name + '_slip', type='hinge', axis='0 0 1', frictionloss='.005', damping='.0002', armature='.001',
        solreffriction='.002 1', solimpfriction='.9999 .9999 .0001')
    sub(head, 'geom', name=name + '_hex', type='mesh', mesh=name + '_hexm', mass='.006', rgba='.62 .6 .55 1', friction='.15 .005 .0001',
        priority='1', contype='2', conaffinity='3', solref=HEX_SOLREF, solimp=HEX_SOLIMP)
    box(head, name + '_mark', [af * .3, 0, NUT_H + .0002], [.0018, .0018, .0002], mark, contype='0', conaffinity='0', mass='0')
    sub(root.find('equality'), 'joint', name=name + '_hold', joint1=name + '_slip', polycoef='0 0 0 0 0', solref='.002 1',
        solimp='.9999 .9999 .0001')
    sub(root.find('contact'), 'exclude', body1=name + '_head', body2='world')
    return base


def wrench_rack(root, world, p):
    for (name, _, _), y in zip(WRENCH_DEFS, RACK_Y):
        add_wrench(root, world, name, wrench_gap(name, p), [RACK_X, y, RACK_Z], WR_YAW, adjustable=name == 'wadj')
        for dx, tag in ((0., 'h'), (-.155, 't')):
            box(world, f'rack_{name}_{tag}', [RACK_X + dx, y, (TABLE_Z + RACK_Z - WR_HT) / 2], [.008, .016, (RACK_Z - WR_HT - TABLE_Z) / 2], '.22 .29 .33 1')


# ---------------------------------------------------------------- scenes
EXPLORE_NUTS = dict(orange=(.12, -.12), purple=(.12, -.04), yellow=(.12, .04), cyan=(.12, .12))
W1_NUT, W2_NUTS, W3_NUT = (.12, -.02), dict(fl1=(.12, -.07), fl2=(.12, .07)), (.12, 0.)
SHELF = (-.13, .22, .84)     # W2 shelf centre (top surface z)
ARM_LEN = .08


def robot_root(model_name):
    root = ET.fromstring(portable_scene(42, 'test')[0]); world = root.find('worldbody')
    for body in list(world.findall('body')):
        if not body.get('name', '').startswith(('left_', 'right_')): world.remove(body)
    for sec in ('contact', 'equality', 'tendon'):
        node = root.find(sec)
        if node is None: node = ET.SubElement(root, sec)
        for c in list(node): node.remove(c)
    root.find('visual/global').attrib.update(offwidth='1280', offheight='720')
    root.find('size').attrib.update(njmax='12000', nconmax='3000')
    root.set('model', model_name)
    return root


def nut_identity(scene, p):
    """name -> (size label, kind, exploration identity whose hidden values it carries)"""
    if scene == 'explore':
        return dict(orange=(p['nut_size_orange'], 'seat', 'orange'), purple=(p['nut_size_purple'], 'seat', 'purple'),
                    yellow=(p['nut_size_yellow'], 'seized', 'yellow'), cyan=(p['nut_size_cyan'], 'seized', 'cyan'))
    if scene == 'W1': return dict(latch=(p['nut_size_yellow'], 'seized', 'yellow'))
    if scene == 'W2': return dict(fl1=(p['nut_size_yellow'], 'seized', 'yellow'), fl2=(p['nut_size_cyan'], 'seized', 'cyan'))
    if scene == 'W3': return dict(clamp=(p['nut_size_orange'], 'clamp', 'orange'))
    return {}


def nut_xy(scene, name):
    if scene == 'explore': return EXPLORE_NUTS[name]
    return dict(latch=W1_NUT, clamp=W3_NUT, **W2_NUTS)[name]


def build(seed, target=None, params=None):
    p = params or parameters(seed); scene = SCENE_OF[target]
    root = robot_root(BENCH_ID + ('' if scene == 'explore' else '_' + target)); world = root.find('worldbody')
    marks = MARKS[p.get('instance', 'S')]
    sub(world, 'body', name='overview_target', pos=[.04, -.02, .86])
    sub(world, 'camera', name='overview', mode='targetbody', target='overview_target', pos=[.32, -.72, 1.22], fovy='50')
    if scene != 'W4': wrench_rack(root, world, p)
    for name, (lab, kind, ident) in nut_identity(scene, p).items():
        nut_fixture(root, world, name, nut_xy(scene, name), AF[lab], marks[ident], pillar=scene != 'W2')
    if scene == 'W1': add_swing_arm(root, world)
    if scene == 'W2': add_flange(root, world)
    if scene == 'W3': add_clamp(root, world)
    if scene in ('explore', 'W4'): add_bolts(root, world, p, scene)
    m = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode')); d = mujoco.MjData(m)
    k = ids(m)
    d.qpos[k['q'][0]] = HOME; d.qpos[k['q'][1]] = RIGHT_PARK
    d.qpos[k['fingers']] = .045; d.ctrl[14:] = .045
    st = init_state(seed, target, p)
    apply_rules(m, d, st, k, init=True)
    mujoco.mj_forward(m, d)
    return m, d


def add_swing_arm(root, world):
    x, y = W1_NUT
    # pillar for W1 is the nut pillar; the arm sits between pillar hub and washer (its hinge is the stud axis)
    arm = sub(world, 'body', name='swing_arm', pos=[x, y, SEAT_Z - .0045])
    sub(arm, 'joint', name='arm_hinge', type='hinge', axis='0 0 1', range=f'{-np.pi / 2} 0', damping='.05', frictionloss='.02',
        armature='.001')
    box(arm, 'arm_bar', [0, ARM_LEN / 2, 0], [.009, ARM_LEN / 2 + .009, .0022], '.25 .55 .35 1', mass='.08')
    sub(arm, 'geom', name='arm_knob', type='cylinder', pos=[0, ARM_LEN, .0202], size=[.006, .018], rgba='.2 .2 .2 1', mass='.01',
        friction='1 .005 .0001')
    sub(root.find('equality'), 'joint', name='arm_lock', joint1='arm_hinge', polycoef='0 0 0 0 0', solref='.004 1')
    sub(root.find('contact'), 'exclude', body1='swing_arm', body2='world')
    box(world, 'arm_stop', [x + ARM_LEN + .006, y, SEAT_Z - .0045], [.004, .01, .006], '.3 .3 .32 1', contype='0', conaffinity='0')


def add_flange(root, world):
    xs = W2_NUTS['fl1'][0]; ys = [W2_NUTS['fl1'][1], W2_NUTS['fl2'][1]]
    zt = SEAT_Z - .0023; zc = zt - .002            # 2 mm washer under each nut (visual, captive on the nut)
    box(world, 'flange_base', [xs - .0025, 0, (TABLE_Z + zc - .002) / 2], [.0275, .095, (zc - .002 - TABLE_Z) / 2], '.33 .36 .40 1')
    for i, y in enumerate(ys):  # collidable studs (the nut fixture's stud is visual)
        sub(world, 'geom', name=f'fl_stud{i}', type='cylinder', pos=[xs, y, zc], size=[.004, .003], rgba='.6 .6 .62 1')
        sub(world, 'geom', name=f'fl_washer{i}', type='cylinder', pos=[xs, y, SEAT_Z - .001], size=[.006, .001], rgba='.7 .7 .72 1')
    cov = sub(world, 'body', name='flange_cover', pos=[xs, 0, zc]); sub(cov, 'freejoint', name='cover_free')
    al = '.78 .8 .83 1'; kw = dict(mass='.03', friction='.6 .005 .0001')
    box(cov, 'cov_back', [-.0175, 0, 0], [.0125, .085, .002], al, **kw)
    edges = [-.085, ys[0] - .005, ys[0] + .005, ys[1] - .005, ys[1] + .005, .085]
    for j in range(3):
        a, b = edges[2 * j], edges[2 * j + 1]
        box(cov, f'cov_strip{j}', [.0075, (a + b) / 2, 0], [.0125, (b - a) / 2, .002], al, **kw)
    box(cov, 'cov_tab', [.014, 0, .020], [.004, .025, .018], '.7 .72 .76 1', mass='.02', friction='1 .005 .0001')
    sub(root.find('equality'), 'weld', name='cover_clamp', body1='flange_cover', solref='.004 1')
    sx, sy, sz = SHELF
    box(world, 'shelf', [sx, sy, (TABLE_Z + sz) / 2], [.05, .10, (sz - TABLE_Z) / 2], '.5 .42 .3 1')
    sub(world, 'body', name='shelf', pos=[sx, sy, sz])


def add_clamp(root, world):
    x, y = W3_NUT
    jaw = sub(world, 'body', name='clamp_jaw', pos=[x + .02, y, SEAT_Z - .004])
    sub(jaw, 'joint', name='jaw_hinge', type='hinge', axis='0 1 0', damping='.01', armature='.001')
    box(jaw, 'jaw_bar', [.03, 0, 0], [.03, .012, .003], '.8 .45 .1 1', mass='.05', contype='0', conaffinity='0')
    box(jaw, 'jaw_pad', [.058, 0, -.008], [.006, .012, .006], '.15 .15 .15 1', mass='.01', contype='0', conaffinity='0')
    box(world, 'clamp_work', [x + .078, y, SEAT_Z - .035], [.012, .02, .008], '.55 .4 .25 1', contype='0', conaffinity='0')
    # jaw rotation follows the nut thread (0.1 rad per rad of nut turn): 90 deg of free run closes the jaw onto the work
    sub(root.find('equality'), 'joint', name='jaw_thread', joint1='jaw_hinge', joint2='clamp_turn', polycoef='0 -.1 0 0 0',
        solref='.004 1')


# ---------------------------------------------------------------- thumb bolts (knob screws) and tapped holes
PITCH = dict(M8=.00125, UNC=.0254 / 18)
SHANK_R = dict(M8=.0040, UNC=.00397)
LS = .00925                  # shank length under the knob: 4 mm clearance layer + 1.5 mm start + 3.75 mm (3 turns M8)
KNOB_H, KNOB_HW = .030, .010
CLEAR_D, CLEAR_HW, TAP_D, TAP_HW = .004, .0043, .012, .00415
RAIL = dict(y=.24, x=(-.02, .02, .06, .10), top=.86)
BLOCK_HOLES = dict(explore=[('blk_m8', (-.14, .14), 'M8', 'steel'), ('blk_unc', (-.10, .14), 'UNC', 'steel'),
                            ('scrap_m8', (-.12, .26), 'M8', 'al')],
                   W4=[('cov_a', (.04, .02), 'M8', 'al'), ('cov_b', (.10, .02), 'M8', 'al')])
BLOCK_TOP = .86
SNUG_GAP, K_BSEAT, TAU_STALL = .00025, 1000., .5
CLAMP_GAP = .00021          # cover clamped: knob inside the seat zone with >= 0.04 N m preload on the thread


def holed_layer(parent, name, x0, x1, y0, y1, z0, z1, holes, rgba, **kw):
    """Tile [x0,x1]x[y0,y1]x[z0,z1] with boxes leaving square holes (hx, hy, half) in one row along x."""
    hz, cz = (z1 - z0) / 2, (z1 + z0) / 2; xs = x0; n = 0
    for hx, hy, h in sorted(holes):
        if hx - h > xs + 1e-6: box(parent, f'{name}_{n}', [(xs + hx - h) / 2, (y0 + y1) / 2, cz], [(hx - h - xs) / 2, (y1 - y0) / 2, hz], rgba, **kw); n += 1
        if hy - h > y0 + 1e-6: box(parent, f'{name}_{n}', [hx, (y0 + hy - h) / 2, cz], [h, (hy - h - y0) / 2, hz], rgba, **kw); n += 1
        if y1 > hy + h + 1e-6: box(parent, f'{name}_{n}', [hx, (hy + h + y1) / 2, cz], [h, (y1 - hy - h) / 2, hz], rgba, **kw); n += 1
        xs = hx + h
    if x1 > xs + 1e-6: box(parent, f'{name}_{n}', [(xs + x1) / 2, (y0 + y1) / 2, cz], [(x1 - xs) / 2, (y1 - y0) / 2, hz], rgba, **kw)


def tapped_block(world, name, xy, holes, rgba, top, clear_in_block=True):
    """Block with blind holes: clearance layer (counterbore) on top, tapped bore below; holes = [(hx, hy)]."""
    x, y = xy; x0, x1, y0, y1 = x - .035, x + .035, y - .025, y + .025
    zc = top - (CLEAR_D if clear_in_block else 0.)
    box(world, name + '_solid', [x, y, (TABLE_Z + zc - TAP_D) / 2], [.035, .025, (zc - TAP_D - TABLE_Z) / 2], rgba)
    holed_layer(world, name + '_tap', x0, x1, y0, y1, zc - TAP_D, zc, [(hx, hy, TAP_HW) for hx, hy in holes], rgba)
    if clear_in_block: holed_layer(world, name + '_clr', x0, x1, y0, y1, zc, top, [(hx, hy, CLEAR_HW) for hx, hy in holes], rgba)


def add_bolts(root, world, p, scene):
    eq = root.find('equality')
    # bolt rail: four knob screws hang by their knobs in clearance pockets
    xs = RAIL['x']; ry, rt = RAIL['y'], RAIL['top']
    box(world, 'rail_solid', [np.mean(xs), ry, (TABLE_Z + rt - .015) / 2], [.085, .014, (rt - .015 - TABLE_Z) / 2], '.3 .3 .34 1')
    holed_layer(world, 'rail_pk', xs[0] - .025, xs[-1] + .025, ry - .014, ry + .014, rt - .015, rt, [(x, ry, .0052) for x in xs], '.3 .3 .34 1')
    for c, x, th in zip(BOLT_COLOURS, xs, p['bolt_threads']):
        b = sub(world, 'body', name='bolt_' + c, pos=[x, ry, rt]); sub(b, 'freejoint', name=f'bolt_{c}_free')
        sub(b, 'geom', name=f'bolt_{c}_shank', type='cylinder', pos=[0, 0, -LS / 2], size=[SHANK_R[th], LS / 2], rgba='.7 .7 .72 1',
            mass='.004', friction='.2 .005 .0001')
        box(b, f'bolt_{c}_knob', [0, 0, KNOB_H / 2], [KNOB_HW, KNOB_HW, KNOB_H / 2], BOLT_RGBA[c], mass='.012', friction='1 .005 .0001')
    holes = BLOCK_HOLES[scene]
    if scene == 'explore':
        tapped_block(world, 'steel_block', (-.12, .14), [h[1] for h in holes[:2]], '.55 .57 .6 1', BLOCK_TOP)
        for (hn, (hx, hy), th, _), lab in zip(holes[:2], ('.95 .95 .95 1', '.15 .15 .15 1')):  # engraved labels (public text)
            box(world, hn + '_label', [hx, hy - .016, BLOCK_TOP + .0002], [.008, .004, .0002], lab, contype='0', conaffinity='0')
        tapped_block(world, 'al_scrap', (-.12, .26), [holes[2][1]], '.82 .84 .87 1', BLOCK_TOP)
        sub(world, 'body', name='al_scrap', pos=[-.12, .26, BLOCK_TOP])     # named handle for the offcut (geoms are static)
    else:
        # W4: aluminium base with two tapped holes; a 4 mm aluminium cover with two clearance holes sits in a pocket
        tapped_block(world, 'al_base', (.07, .02), [h[1] for h in holes], '.8 .82 .85 1', BLOCK_TOP - CLEAR_D, clear_in_block=False)
        zt = BLOCK_TOP - CLEAR_D
        for sx in (-1, 1): box(world, f'pocket_x{sx}', [.07 + sx * .0405, .02, zt + .0015], [.0045, .03, .0015], '.7 .72 .75 1')
        for sy in (-1, 1): box(world, f'pocket_y{sy}', [.07, .02 + sy * .0265, zt + .0015], [.036, .0015, .0015], '.7 .72 .75 1')
        cov = sub(world, 'body', name='al_cover', pos=[.07, .02, zt] if not p.get('test_no_cover') else [.07, -.20, TABLE_Z])
        sub(cov, 'freejoint', name='al_cover_free')      # test_no_cover (rule test only): cover lies on the table, off the base
        holed_layer(cov, 'alc', -.0357, .0357, -.0247, .0247, 0., CLEAR_D, [(hx - .07, hy - .02, CLEAR_HW) for _, (hx, hy), _, _ in holes],
                    '.86 .87 .9 1', mass='.006', friction='.6 .005 .0001')
    for hn, (hx, hy), th, mat in holes:
        f = sub(world, 'body', name='fol_' + hn, pos=[hx, hy, BLOCK_TOP - CLEAR_D])
        sub(f, 'joint', name=f'fol_{hn}_turn', type='hinge', axis='0 0 1', range='-40 .2', frictionloss='.02', damping='.0002',
            armature='.00002', solreffriction='.002 1', solimpfriction='.9999 .9999 .0001')
        sub(f, 'joint', name=f'fol_{hn}_lift', type='slide', axis='0 0 1', range='-.012 .0005', damping='.01', armature='.0005')
        sub(f, 'geom', name=f'fol_{hn}_g', type='sphere', size='.001', mass='.002', contype='0', conaffinity='0')
        # first thread turn: a crest ring 1.5 mm below the thread start that a bolt tip rests on until its thread catches
        # (it rides down with the follower, i.e. with the engaged bolt)
        for j, (cx, cy, hx_, hy_) in enumerate([(.0039, 0, .0003, .0042), (-.0039, 0, .0003, .0042), (0, .0039, .0036, .0003), (0, -.0039, .0036, .0003)]):
            box(f, f'fol_{hn}_crest{j}', [cx, cy, -.0015 - .0005], [hx_, hy_, .0005], '.5 .5 .52 1', mass='.0005', friction='.2 .005 .0001')
        sub(root.find('contact'), 'exclude', body1='fol_' + hn, body2='world')
        sub(eq, 'joint', name=f'fol_{hn}_thread', joint1=f'fol_{hn}_lift', joint2=f'fol_{hn}_turn', polycoef=f'0 {PITCH[th] / (2 * np.pi):.8g} 0 0 0',
            solref='.002 1', solimp='.9999 .9999 .0001')
        for c in BOLT_COLOURS:
            sub(eq, 'weld', name=f'thr_{hn}_{c}', body1='fol_' + hn, body2='bolt_' + c, active='false', solref='.002 1',
                solimp='.9999 .9999 .0001')


# ---------------------------------------------------------------- ids, state, rules
def _has(m, kind, name): return mujoco.mj_name2id(m, kind, name) >= 0


def ids(m):
    B, J, G, E = mujoco.mjtObj.mjOBJ_BODY, mujoco.mjtObj.mjOBJ_JOINT, mujoco.mjtObj.mjOBJ_GEOM, mujoco.mjtObj.mjOBJ_EQUALITY
    arms = [[m.joint(f'{s}_joint{i}').id for i in range(1, 8)] for s in ('left', 'right')]
    def under(b, root):
        while b > 0:
            if b == root: return True
            b = m.body_parentid[b]
        return False
    robot = [g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name.startswith(('left_', 'right_'))]
    k = dict(q=[m.jnt_qposadr[a].tolist() for a in arms], v=[m.jnt_dofadr[a].tolist() for a in arms],
             fingers=[int(m.jnt_qposadr[m.joint(f'{s}_finger_{f}').id]) for s in ('left', 'right') for f in 'ab'],
             tip=[int(m.site('left_tip').id), int(m.site('right_tip').id)], robot=robot, nuts={}, wrenches={}, bolts={}, holes={})
    for n in ('orange', 'purple', 'yellow', 'cyan', 'latch', 'fl1', 'fl2', 'clamp'):
        if _has(m, B, n + '_head'):
            k['nuts'][n] = dict(turn_q=int(m.jnt_qposadr[m.joint(n + '_turn').id]), turn_v=int(m.jnt_dofadr[m.joint(n + '_turn').id]),
                                hex=int(m.geom(n + '_hex').id), hold=int(m.equality(n + '_hold').id), base=int(m.body(n + '_base').id))
    for n, _, _ in WRENCH_DEFS:
        if _has(m, B, n):
            b = m.body(n).id
            k['wrenches'][n] = dict(body=int(b), geoms=[g for g in range(m.ngeom) if m.geom_bodyid[g] == b])
    for c in BOLT_COLOURS:
        if _has(m, B, 'bolt_' + c):
            b = m.body('bolt_' + c).id
            k['bolts'][c] = dict(body=int(b), geoms=[g for g in range(m.ngeom) if m.geom_bodyid[g] == b])
    for sc in ('explore', 'W4'):
        for hn, _, _, _ in BLOCK_HOLES[sc]:
            if _has(m, B, 'fol_' + hn):
                k['holes'][hn] = dict(turn_q=int(m.jnt_qposadr[m.joint(f'fol_{hn}_turn').id]), turn_v=int(m.jnt_dofadr[m.joint(f'fol_{hn}_turn').id]),
                                      thread=int(m.equality(f'fol_{hn}_thread').id),
                                      lift_q=int(m.jnt_qposadr[m.joint(f'fol_{hn}_lift').id]), body=int(m.body('fol_' + hn).id),
                                      weld={c: int(m.equality(f'thr_{hn}_{c}').id) for c in k['bolts']})
    if _has(m, J, 'arm_hinge'):
        k['arm'] = dict(q=int(m.jnt_qposadr[m.joint('arm_hinge').id]), lock=int(m.equality('arm_lock').id),
                        geoms=[g for g in range(m.ngeom) if m.geom_bodyid[g] == m.body('swing_arm').id])
    if _has(m, B, 'flange_cover'):
        cb = m.body('flange_cover').id
        k['cover'] = dict(body=int(cb), clamp=int(m.equality('cover_clamp').id), geoms=[g for g in range(m.ngeom) if m.geom_bodyid[g] == cb])
    if _has(m, B, 'al_cover'):
        cb = m.body('al_cover').id
        k['alcover'] = dict(body=int(cb), geoms=[g for g in range(m.ngeom) if m.geom_bodyid[g] == cb])
    k['tool_geoms'] = sorted({g for w in k['wrenches'].values() for g in w['geoms']})
    return k


def init_state(seed, target, params):
    p = params or parameters(seed); scene = SCENE_OF[target]
    nuts = {}
    for name, (lab, kind, ident) in nut_identity(scene, p).items():
        nuts[name] = dict(af=AF[lab], kind=kind, tb=(p['breakaway_' + ident] / 1000. if kind == 'seized' else None), broke=False,
                          rounded=False, n=0, released=False, practice=scene == 'explore' and kind == 'seized', tau_tight=0.)
    holes = {}
    for hn, xy, th, mat in BLOCK_HOLES.get(scene, []):
        holes[hn] = dict(kind=th, mat=mat, xy=list(xy), tdmg=(p['al_gall_torque'] if mat == 'al' else p['steel_gall_torque']) / 1000.,
                         bolt=None, galled=False, n=0, tmax=0., zoff=0.)
    bolts = {c: dict(thread=th, hole=None, cand=None, acc=0., yaw=None) for c, th in zip(BOLT_COLOURS, p['bolt_threads'])} if holes else {}
    gaps = {n: wrench_gap(n, p) for n, _, _ in WRENCH_DEFS} if scene != 'W4' else {}
    return dict(scene=scene, target=target, tau_s0=p['strip_torque'] / 1000., run=p['thread_run'] / 1000., bind=p['bind_turns'],
                kbind=(TAU_STALL - p['thread_run'] / 1000.) / .25, nuts=nuts, gaps=gaps, holes=holes, bolts=bolts, hold=0.,
                rounded_event=False, arm_released=False, cover_released=False, engaged_event=False, galled_event=False, clamped_event=False,
                flags=dict(nut_rounded=False, practice_nut_rounded=False, thread_galled=False, practice_thread_galled=False))


def nut_angle(d, kn): return float(d.qpos[kn['turn_q']])   # + = counter-clockwise from above = loosening


def nut_friction(n, q):
    """Thread friction for this nut at turn angle q (rad, + = loosening); also updates the tightening torque state."""
    tight = -q
    if n['kind'] == 'seat':
        n['tau_tight'] = max(0., K_SEAT * tight + SNUG); return TAU_RUN + n['tau_tight']
    if n['kind'] == 'clamp':
        n['tau_tight'] = max(0., K_SEAT * (tight - np.radians(CLAMP_RUN_DEG))); return 2 * TAU_RUN + n['tau_tight']
    if not n['broke'] and q > np.radians(BREAK_DEG): n['broke'] = True
    return TAU_LOOSE if n['broke'] else n['tb']


def contact_torques(m, d, k, st):
    """{(nut, wrench): torque about the nut axis exerted by the wrench on the nut head} from native contacts."""
    out = {}; nc = d.ncon
    if not nc or not k['nuts'] or not k['wrenches']: return out
    g1, g2 = d.contact.geom1[:nc], d.contact.geom2[:nc]
    hexes = {kn['hex']: n for n, kn in k['nuts'].items()}; tools = {g: w for w, kw in k['wrenches'].items() for g in kw['geoms']}
    hx = np.fromiter(hexes, int)
    sel = np.nonzero(np.isin(g1, hx) | np.isin(g2, hx))[0]
    buf = np.zeros(6)
    for i in sel:
        a, b = int(g1[i]), int(g2[i])
        if a in hexes and b in tools: nut, w, s = hexes[a], tools[b], -1.
        elif b in hexes and a in tools: nut, w, s = hexes[b], tools[a], 1.
        else: continue
        mujoco.mj_contactForce(m, d, int(i), buf)
        c = d.contact[int(i)]; f = c.frame.reshape(3, 3).T @ buf[:3] * s
        ax = d.xpos[k['nuts'][nut]['base']]; r = c.pos - ax
        out[(nut, w)] = out.get((nut, w), 0.) + float(r[0] * f[1] - r[1] * f[0])
    return out


def apply_rules(m, d, st, k, init=False):
    # nuts: thread friction + strip rule
    tq = {} if init else contact_torques(m, d, k, st)
    for name, n in st['nuts'].items():
        kn = k['nuts'][name]; q = nut_angle(d, kn)
        m.dof_frictionloss[kn['turn_v']] = nut_friction(n, q)
        if n['kind'] == 'seized' and not n['released'] and q > np.radians(RELEASE_DEG): n['released'] = True
        if n['rounded'] or init: continue
        over = False
        for w, gap in st['gaps'].items():
            c = gap - n['af']; chi = STRIP_CHI * n['af']; t = abs(tq.get((name, w), 0.))
            if STRIP_CMIN <= c < chi and t > st['tau_s0'] * (1. - c / chi): over = True
            # an undersized wrench forced over the corners (centre well inside the corner circle) crushes them under torque
            if c < 0 and t > FORCED_TAU and np.hypot(*(d.xpos[k['wrenches'][w]['body']][:2] - d.xpos[k['nuts'][name]['base']][:2])) < FORCED_OFF:
                over = True
        n['n'] = n['n'] + 1 if over else 0
        if n['n'] >= STRIP_TICKS:
            n['rounded'] = True; st['rounded_event'] = True; d.eq_active[kn['hold']] = 0
            m.geom_rgba[kn['hex']] = [.85, .85, .88, 1.]          # bright, rounded corners are visible
            st['flags']['practice_nut_rounded' if n['practice'] else 'nut_rounded'] = True
    if 'arm' in k and not st['arm_released'] and st['nuts']['latch']['released'] and not st['nuts']['latch']['rounded']:
        d.eq_active[k['arm']['lock']] = 0; st['arm_released'] = True
    if 'cover' in k and not st['cover_released'] and all(st['nuts'][n]['released'] and not st['nuts'][n]['rounded'] for n in ('fl1', 'fl2')):
        d.eq_active[k['cover']['clamp']] = 0; st['cover_released'] = True
    if st['holes']: bolt_rules(m, d, st, k, init)


def thread_gap(d, st, k, hn):
    """knob-to-surface gap set by the thread advance (gap at engagement minus pitch x turns of the follower); independent of how
    hard the hand presses the knob (the soft thread coupling gives a little), so a released screw keeps the reading"""
    h = st['holes'][hn]; return float(h['zoff'] + PITCH[h['kind']] / (2 * np.pi) * d.qpos[k['holes'][hn]['turn_q']])


def bolt_yaw(d, b): R = d.xmat[b].reshape(3, 3); return float(np.arctan2(R[1, 0], R[0, 0]))


def wrap(a): return (a + np.pi) % (2 * np.pi) - np.pi


def bolt_rules(m, d, st, k, init):
    FR = mujoco.mjtConstraint.mjCNSTR_FRICTION_DOF
    for hn, h in st['holes'].items():
        kh = k['holes'][hn]; turn = float(d.qpos[kh['turn_q']]); c = h['bolt']
        if c is not None and not init and turn > .05:            # backed out past the thread start: bolt is free again
            d.eq_active[kh['weld'][c]] = 0; st['bolts'][c]['hole'] = None; h['bolt'] = None; c = None
        fl = .02
        if c is not None:
            b = st['bolts'][c]; turns = -turn / (2 * np.pi); mis = b['thread'] != h['kind']
            if h['galled']: fl = st['run']                     # stripped aluminium thread: the bolt spins without advancing
            elif mis: fl = min(st['run'] + st['kbind'] * max(0., turns - st['bind']), TAU_STALL)
            else: fl = st['run']
            gap = thread_gap(d, st, k, hn)
            fl += K_BSEAT * max(0., SNUG_GAP - gap)
            if not init:
                t = 0.
                rows = np.nonzero((d.efc_type[:d.nefc] == FR) & (d.efc_id[:d.nefc] == kh['turn_v']))[0]
                if len(rows): t = abs(float(d.efc_force[rows[0]]))
                h['tmax'] = max(h['tmax'], t); thr = h['tdmg'] if mis else 3. * h['tdmg']
                h['n'] = h['n'] + 1 if (not h['galled'] and t > thr) else 0
                if h['n'] >= STRIP_TICKS:
                    h['galled'] = True; st['galled_event'] = True
                    m.eq_data[kh['thread'], 0] = d.qpos[kh['lift_q']]; m.eq_data[kh['thread'], 1] = 0.   # lift frozen, turn free
                    if h['mat'] == 'al': st['flags']['practice_thread_galled' if st['scene'] == 'explore' else 'thread_galled'] = True
        m.dof_frictionloss[kh['turn_v']] = fl
    if init: return
    free_holes = [hn for hn, h in st['holes'].items() if h['bolt'] is None]
    for c, b in st['bolts'].items():
        if b['hole'] is not None: continue
        kb = k['bolts'][c]['body']; R = d.xmat[kb].reshape(3, 3); tip = d.xpos[kb] - R[:, 2] * LS; yaw = bolt_yaw(d, kb)
        cand = None
        for hn in free_holes:
            hx, hy = st['holes'][hn]['xy']
            if np.hypot(tip[0] - hx, tip[1] - hy) <= .0007 and (BLOCK_TOP - CLEAR_D) - tip[2] >= .001 and R[2, 2] >= np.cos(np.radians(3.)): cand = hn
        if cand != b['cand']: b.update(cand=cand, acc=0., yaw=yaw); continue
        if cand is None: continue
        b['acc'] += wrap(yaw - b['yaw']); b['yaw'] = yaw
        if b['acc'] <= -np.radians(15.):                            # thread caught: weld bolt to the follower at the current pose
            kh = k['holes'][cand]; fb = kh['body']
            q1, q2 = d.xquat[fb].copy(), d.xquat[kb].copy(); qi = np.zeros(4); mujoco.mju_negQuat(qi, q1); rq = np.zeros(4)
            mujoco.mju_mulQuat(rq, qi, q2)
            m.eq_data[kh['weld'][c], 3:6] = d.xmat[fb].reshape(3, 3).T @ (d.xpos[kb] - d.xpos[fb]); m.eq_data[kh['weld'][c], 6:10] = rq
            d.eq_active[kh['weld'][c]] = 1; b.update(hole=cand, cand=None, acc=0.); st['holes'][cand]['bolt'] = c
            st['holes'][cand]['zoff'] = float(d.xpos[kb][2] - BLOCK_TOP - PITCH[st['holes'][cand]['kind']] / (2 * np.pi) * d.qpos[kh['turn_q']])
            st['engaged_event'] = True; free_holes.remove(cand)


# ---------------------------------------------------------------- step / success / damage
def contacts_between(d, A, B):
    nc = d.ncon
    if not nc: return False
    g1, g2 = d.contact.geom1[:nc], d.contact.geom2[:nc]
    return bool(np.any((np.isin(g1, A) & np.isin(g2, B)) | (np.isin(g2, A) & np.isin(g1, B))))


def task_done(m, d, st, k):
    sc = st['scene']; N = st['nuts']; actors = k['robot'] + k['tool_geoms']
    if sc == 'explore':
        if st['target'] != TASK_A: return False
        ok = all(WIN[0] <= N[n]['tau_tight'] <= WIN[1] and not N[n]['rounded'] for n in ('orange', 'purple'))
        return ok and not contacts_between(d, actors, [k['nuts'][n]['hex'] for n in ('orange', 'purple')])
    if sc == 'W1':
        ok = d.qpos[k['arm']['q']] <= -np.radians(85.) and st['arm_released'] and not N['latch']['rounded']
        return ok and not contacts_between(d, actors, k['arm']['geoms'] + [k['nuts']['latch']['hex']])
    if sc == 'W2':
        if not (st['cover_released'] and not N['fl1']['rounded'] and not N['fl2']['rounded']): return False
        cb = k['cover']['body']; x, y, z = d.xpos[cb]; sx, sy, sz = SHELF
        on = abs(x + .005 - sx) <= .035 and abs(y - sy) <= .03 and sz <= z <= sz + .01 and d.xmat[cb][8] >= np.cos(np.radians(10.))
        return on and not contacts_between(d, k['robot'], k['cover']['geoms'])
    if sc == 'W3':
        n = N['clamp']; ok = WIN[0] <= n['tau_tight'] <= WIN[1] and not n['rounded']
        return ok and not contacts_between(d, actors, [k['nuts']['clamp']['hex']])
    if sc == 'W4':
        H = st['holes']
        for hn in ('cov_a', 'cov_b'):
            c = H[hn]['bolt']
            if c is None or H[hn]['galled'] or st['bolts'][c]['thread'] != H[hn]['kind']: return False
            if thread_gap(d, st, k, hn) > CLAMP_GAP: return False   # thread preload >= 0.04 N m (knob torque >= 0.05 N m)
        cb = k['alcover']['body']; cp = d.xpos[cb]                    # the cover must sit in its pocket, flat
        if np.hypot(cp[0] - .07, cp[1] - .02) > .001 or abs(cp[2] - (BLOCK_TOP - CLEAR_D)) > .0003 or d.xmat[cb][8] < np.cos(np.radians(2.)):
            return False
        st['clamped_event'] = True
        bg = [g for c in k['bolts'] for g in k['bolts'][c]['geoms']]
        return not contacts_between(d, k['robot'], bg + k['alcover']['geoms'])
    return False


def step(m, d, state):
    k = state.get('_k') or state.setdefault('_k', ids(m))
    dofs = k['v'][0] + k['v'][1]
    d.qfrc_applied[dofs] = d.qfrc_bias[dofs]
    mujoco.mj_step(m, d)
    apply_rules(m, d, state, k)
    state['hold'] = state['hold'] + m.opt.timestep if task_done(m, d, state, k) else 0.


def success(model, data, state, target): return state['hold'] >= HOLD_S


def damage(model, data, state):
    f = state['flags']; sc = state['scene']
    if sc == 'explore': return dict(f)
    if sc == 'W4': return dict(thread_galled=f['thread_galled'])
    return dict(nut_rounded=f['nut_rounded'])


def wrist_wrench(model, data):
    out = []   # no flex in this bench, so the native wrist F/T is complete
    for s in ('left', 'right'):
        R = data.site(s + '_wrist').xmat.reshape(3, 3)
        out += [R @ data.sensor(s + '_force').data, R @ data.sensor(s + '_torque').data]
    return np.concatenate(out)


def observation(model, data, state):
    k = state.get('_k') or state.setdefault('_k', ids(model))
    q, v = k['q'][0] + k['q'][1], k['v'][0] + k['v'][1]
    return dict(joint_pos=data.qpos[q].copy(), joint_vel=data.qvel[v].copy(), jaw_opening=data.qpos[k['fingers']].copy(),
                wrist_wrench=wrist_wrench(model, data),
                finger_force=np.array([data.sensor(f'{s}_finger_{f}_normal').data[0] for s in ('left', 'right') for f in 'ab']),
                time=float(data.time))


# ---------------------------------------------------------------- privileged scripted controller (references, probes)
def smooth(f): f = min(1., max(0., f)); return f * f * (3 - 2 * f)


def Rz2(a): c, s = np.cos(a), np.sin(a); return np.array([[c, -s], [s, c]])


class Drive:
    """Closed-loop scripted left-arm controller (joint PD on IK targets, the agent's gains). Records ctrl rows; may read
    privileged object state. step() is the bench step, so a replay of the recorded rows from reset is bit-identical."""
    def __init__(s, seed, target, params, max_ticks=200000):
        s.p = params or parameters(seed); s.target = target
        s.m, s.d = build(seed, target, s.p); s.st = init_state(seed, target, s.p); s.k = ids(s.m)
        s.scr = mujoco.MjData(s.m); s.acts = []; s.max_ticks = max_ticks
        s.qt = [s.d.qpos[s.k['q'][a]].copy() for a in (0, 1)]
        s.qg = [q.copy() for q in s.qt]          # latest IK goal
        s.qp = [q.copy() for q in s.qt]; s.kr = [IK_EVERY, IK_EVERY]   # ramp start and ramp counter
        s.grip = [.045, .045]
        s.cmd = s.tip(0).copy(); s.cyaw = s.hyaw(0); s.e = np.zeros(3); s.ey = 0.
        s.log = []

    # --- state readers
    def tip(s, a=0): return s.d.site_xpos[s.k['tip'][a]].copy()
    def hyaw(s, a=0): R = s.d.site_xmat[s.k['tip'][a]].reshape(3, 3); return float(np.arctan2(R[1, 0], R[0, 0]))
    def bpose(s, b): R = s.d.xmat[b].reshape(3, 3); return s.d.xpos[b].copy(), float(np.arctan2(R[1, 0], R[0, 0]))

    # --- low level
    def tick(s, n=1):
        m, d, k = s.m, s.d, s.k
        for _ in range(n):
            u = np.zeros(m.nu)
            for a in (0, 1):
                s.kr[a] += 1; ramp = s.qp[a] + (s.qg[a] - s.qp[a]) * min(1., s.kr[a] / IK_EVERY)
                s.qt[a] = s.qt[a] + np.clip(ramp - s.qt[a], -VMAX * DT, VMAX * DT)       # real Panda joint speed limits
                bias = d.qfrc_bias[k['v'][a]]                                            # step() adds this compensation
                u[7 * a:7 * a + 7] = np.clip(KP * (s.qt[a] - d.qpos[k['q'][a]]) - KD * d.qvel[k['v'][a]], -TLIM - bias, TLIM - bias)
            u[14:16] = s.grip[0]; u[16:18] = s.grip[1]
            d.ctrl[:] = u; s.acts.append(u); step(m, d, s.st)
            if len(s.acts) > s.max_ticks: raise RuntimeError('reference too long')

    def ik(s, pos, yaw, a=0, iters=20):
        q = s.solve(pos, yaw, a, iters, s.qg[a]); s.qp[a] = s.qt[a].copy(); s.qg[a] = q; s.kr[a] = 0

    def solve(s, pos, yaw, a=0, iters=20, q0=None):
        m, scr, k = s.m, s.scr, s.k
        q, v, tip = k['q'][a], k['v'][a], k['tip'][a]
        scr.qpos[:] = s.d.qpos; scr.qpos[q] = s.qg[a] if q0 is None else q0
        R = Rotation.from_euler('z', yaw).as_matrix() @ DOWN
        jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv)); lo, hi = m.jnt_range[[m.dof_jntid[i] for i in v]].T
        for _ in range(iters):
            mujoco.mj_kinematics(m, scr); mujoco.mj_comPos(m, scr)
            err = np.r_[pos - scr.site_xpos[tip], .5 * Rotation.from_matrix(R @ scr.site_xmat[tip].reshape(3, 3).T).as_rotvec()]
            if np.linalg.norm(err) < 1e-6: break
            mujoco.mj_jacSite(m, scr, jp, jr, tip)
            J = np.vstack([jp[:, v], .5 * jr[:, v]])
            Jp = J.T @ np.linalg.inv(J @ J.T + 1e-5 * np.eye(6))
            null = (np.eye(7) - Jp @ J) @ (NULL_GAIN * (POSTURE - scr.qpos[q]) * POSTURE_W)   # keep the elbow branch near HOME
            scr.qpos[q] = np.clip(scr.qpos[q] + np.clip(Jp @ err + null, -.08, .08), lo + .01, hi - .01)
        return scr.qpos[q].copy()

    def j7_margin(s, pos, yaws):
        """smallest distance of joint 7 from its limits along a sequence of hand yaws at pos (IK from HOME-like posture)"""
        q0 = s.qg[0].copy(); worst = 9.
        for y in yaws:
            for _ in range(4): q0 = s.solve(pos, y, 0, 30, q0)
            worst = min(worst, 2.8973 - abs(q0[6]))
        return worst

    def go(s, pos, yaw=None, grip=None, n=300, every=4, settle=60, unwrap=False):
        """Smooth open-loop Cartesian move of the left tip (with tracking-offset compensation frozen). Yaw takes the short
        way unless unwrap=True (then `yaw` is an absolute, continuous wrist angle)."""
        p0, y0, g0 = s.cmd.copy(), s.cyaw, s.grip[0]
        p1 = p0 if pos is None else np.asarray(pos, float); g1 = g0 if grip is None else grip
        y1 = y0 if yaw is None else (yaw if unwrap else y0 + wrap(yaw - y0))
        for i in range(n + settle):
            if i % every == 0:
                f = smooth((i + 1) / n); s.cmd = p0 + (p1 - p0) * f; s.cyaw = y0 + (y1 - y0) * f
                s.ik(s.cmd + s.e, s.cyaw + s.ey)
            s.grip[0] = g0 + (g1 - g0) * smooth((i + 1) / n)
            s.tick()

    def servo(s, body, traj, n, every=4, gain=.03, emax=.006, eymax=np.radians(8.), stop=None, settle=0, rel=None):
        """Move the grasped object `body` so that its pose follows traj(f) -> (pos*, yaw*), f in [0,1]. The hand target is
        derived from the hand->object transform (live, or the fixed unloaded `rel` while the object is constrained); an
        integrator on the hand's own tracking error removes the PD offset (it pushes the hand, it never twists the grip)."""
        for i in range(n + settle):
            if i % every == 0:
                ps, ys = traj(smooth((i + 1) / n))
                ph, yh = s.tip(), s.hyaw()
                rxy, rz, ry = s.rel(body) if rel is None else rel
                yt = s.cyaw + wrap((ys - ry) - s.cyaw); pt = np.r_[np.asarray(ps)[:2] - Rz2(yt) @ np.asarray(rxy), ps[2] - rz]
                s.e = np.clip(s.e + gain * (pt - ph), -emax, emax); s.ey = float(np.clip(s.ey + gain * wrap(yt - yh), -eymax, eymax))
                s.cmd, s.cyaw = pt, yt
                s.ik(s.cmd + s.e, s.cyaw + s.ey)
                if stop is not None and stop(): return True
            s.tick()
        return False

    def relax(s):
        """Forget the tracking-offset integrator smoothly (after letting go of an object)."""
        e0, ey0 = s.e.copy(), s.ey
        for i in range(60):
            if i % 4 == 0:
                f = smooth((i + 1) / 60); s.e = e0 * (1 - f); s.ey = ey0 * (1 - f); s.ik(s.cmd + s.e, s.cyaw + s.ey)
            s.tick()

    def rel(s, body):
        o, yo = s.bpose(body); ph, yh = s.tip(), s.hyaw()
        return (Rz2(-yh) @ (o - ph)[:2]).tolist(), float(o[2] - ph[2]), float(wrap(yo - yh))

    def unload(s, body, rel, n=150):
        """Move the hand to where it would sit on the object's current pose with the unloaded grasp transform `rel`
        (releases the elastic wind-up stored in the fingers without moving the object)."""
        o, yo = s.bpose(body); rxy, rz, ry = rel
        yt = s.cyaw + wrap((yo - ry) - s.cyaw); pt = np.r_[o[:2] - Rz2(yt) @ np.asarray(rxy), o[2] - rz]
        p0, y0, e0, ey0 = s.cmd + s.e, s.cyaw + s.ey, s.e.copy(), s.ey
        for i in range(n):
            if i % 4 == 0:
                f = smooth((i + 1) / n); s.ik(p0 + (pt - p0) * f, y0 + (yt - y0) * f)
            s.tick()
        s.cmd, s.cyaw, s.e, s.ey = pt, yt, np.zeros(3), 0.

    def freeze(s):
        """Hold the current hand pose: drop the integrator and accumulated push (e.g. once a nut reached its torque)."""
        s.e = np.zeros(3); s.ey = 0.; s.cmd = s.tip(); s.cyaw = s.hyaw(); s.ik(s.cmd, s.cyaw); s.tick(40)

    def wait(s, seconds):
        for i in range(int(round(seconds / DT))):
            if i % 20 == 0: s.ik(s.cmd + s.e, s.cyaw + s.ey)
            s.tick()

    def out(s): return np.array(s.acts)


# ---------------------------------------------------------------- reference skills
Z_TRAVEL = SEAT_Z + WR_DZ + .045
APPROACH = .035


def wpose(dr, w): return dr.bpose(dr.k['wrenches'][w]['body'])


def wrist_const(dr, pos):
    """joint 7 + hand yaw is (locally) constant along a continuous path; its value with the tip at pos"""
    q = dr.solve(np.asarray(pos, float), dr.cyaw, 0, 40, dr.qg[0].copy()); return float(q[6] + dr.cyaw)


def pick_wrench(dr, w, work=None):
    """Grasp the handle from above. The jaw is symmetric and the wrist yaw is continuous, so the hand yaw is psi + j pi + 2 k pi;
    choose the one keeping joint 7 farthest from its limits at the rack and, with `work` = (nut xy, wrench yaws), over the job."""
    o, psi = wpose(dr, w); gp = np.r_[o[:2] + Rz2(psi) @ [0., -RG], o[2]]
    checks = [(wrist_const(dr, gp + [0, 0, .05]), 0.)]
    if work is not None:
        N, ys = work; c_work = wrist_const(dr, np.r_[np.asarray(N)[:2], SEAT_Z + WR_DZ + .05])
        checks += [(c_work, y - psi) for y in ys]
    base = dr.cyaw + wrap(psi - dr.cyaw); best = None
    for Y in [base + j * np.pi + 2 * np.pi * kk for j in (0, 1) for kk in (-1, 0, 1)]:
        mg = min(2.8973 - abs(c - (Y + dy)) for c, dy in checks)
        if best is None or mg > best[0]: best = (mg, Y)
    yaw = best[1]
    dr.go(gp + [0, 0, .08], yaw, .012, n=700, unwrap=True)
    dr.go(gp + [0, 0, .0], None, .012, n=300)
    dr.go(None, None, 0., n=150, settle=100)
    dr.go(gp + [0, 0, Z_TRAVEL - o[2] + .02], None, 0., n=350)


def nut_yaw(dr, n):
    R = dr.d.xmat[dr.m.body(n + '_head').id].reshape(3, 3); return float(np.arctan2(R[1, 0], R[0, 0]))


def aligned_yaw(dr, n, want):
    phi = nut_yaw(dr, n); return want + wrap(((phi - want) + np.pi / 6) % (np.pi / 3) - np.pi / 6)


def lin(a, b): a, b = np.asarray(a, float), np.asarray(b, float); return lambda f: a + (b - a) * f


def engage(dr, w, n, want):
    """Bring wrench w onto nut n from the -y_w side; returns the engaged wrench yaw."""
    wb = dr.k['wrenches'][w]['body']; N = dr.d.xpos[dr.k['nuts'][n]['base']].copy(); N[2] = SEAT_Z + WR_DZ
    psi = aligned_yaw(dr, n, want); yhat = np.array([-np.sin(psi), np.cos(psi), 0.]); A = N - APPROACH * yhat
    o, y0 = wpose(dr, w); y1 = y0 + wrap(psi - y0)
    P = lambda a, b, ya, yb: (lambda f: (a + (b - a) * f, ya + (yb - ya) * f))
    dr.servo(wb, P(o, np.r_[A[:2], Z_TRAVEL], y0, y1), 500)
    dr.servo(wb, P(np.r_[A[:2], Z_TRAVEL], A, y1, y1), 400, settle=100)
    psi = aligned_yaw(dr, n, want); y1b = y1 + wrap(psi - y1)
    dr.servo(wb, P(A, N, y1, y1b), 600, settle=150)
    for _ in range(3):        # seated check (privileged): jaws fully on the flats; otherwise back off, re-align and slide again
        o1 = dr.d.xpos[wb]
        if np.hypot(*(o1[:2] - N[:2])) < .002 and abs(o1[2] - N[2]) < .0015: break
        dr.engage_retries = getattr(dr, 'engage_retries', 0) + 1
        dr.servo(wb, P(o1, np.r_[A[:2], Z_TRAVEL], dr.bpose(wb)[1], y1b), 500)
        psi = aligned_yaw(dr, n, want); y1b = y1 + wrap(psi - y1); yhat = np.array([-np.sin(psi), np.cos(psi), 0.]); A = N - APPROACH * yhat
        dr.servo(wb, P(np.r_[A[:2], Z_TRAVEL], A, y1b, y1b), 500, settle=150)
        dr.servo(wb, P(A, N, y1b, y1b), 700, settle=150)
    return y1b, N


def turn(dr, w, N, y0, dy, n, stop=None, rel=None):
    wb = dr.k['wrenches'][w]['body']
    return dr.servo(wb, lambda f: (N, y0 + dy * f), n, stop=stop, rel=rel)


def disengage(dr, w, N):
    wb = dr.k['wrenches'][w]['body']; o, y = wpose(dr, w)
    yhat = np.array([-np.sin(y), np.cos(y), 0.]); A = N - (APPROACH + .01) * yhat
    dr.servo(wb, lambda f: (o + (A - o) * f, y), 500, settle=50)
    dr.servo(wb, lambda f: (A + (np.r_[A[:2], Z_TRAVEL] - A) * f, y), 300)


def return_wrench(dr, w):
    i = [x[0] for x in WRENCH_DEFS].index(w); home = np.array([RACK_X, RACK_Y[i], RACK_Z + .0008])
    wb = dr.k['wrenches'][w]['body']; o, y = wpose(dr, w); y1 = y + wrap(WR_YAW - y)
    up = np.r_[home[:2], Z_TRAVEL + .01]
    dr.servo(wb, lambda f: (o + (up - o) * f, y + (y1 - y) * f), 700)
    dr.servo(wb, lambda f: (up + (home - up) * f, y1), 450, settle=80)
    dr.go(None, None, .012, n=120, settle=40); dr.relax()
    dr.go(dr.cmd + [0, 0, .07], None, .012, n=300, settle=20)


def wrench_for(lab): return 'w13' if lab == '13mm' else 'w1_2'


def park(dr):
    dr.go([-.05, -.12, 1.08], 0., .045, n=600)


def finish(dr, extra=1.):
    dr.wait(HOLD_S + extra)
    return dr.out()




def _png(path, img):
    chunk = lambda t, b: struct.pack('>I', len(b)) + t + b + struct.pack('>I', zlib.crc32(t + b) & 0xffffffff)
    raw = b''.join(b'\x00' + row.tobytes() for row in img)
    h, w = img.shape[:2]
    Path(path).write_bytes(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0))
                           + chunk(b'IDAT', zlib.compress(raw, 6)) + chunk(b'IEND', b''))


def render(model, data, path, camera='overview'):
    r = mujoco.Renderer(model, 720, 1280); r.update_scene(data, camera=camera); img = r.render(); r.close(); _png(path, img)


def tighten_nut(dr, w, n, start_deg=18., limit_deg=45., target=.345, n_ticks=2200):
    y0, N = engage(dr, w, n, WR_YAW + np.radians(start_deg))
    nut = dr.st['nuts'][n]; wb = dr.k['wrenches'][w]['body']; rel = dr.rel(wb)
    hit = turn(dr, w, N, y0, -np.radians(limit_deg), n_ticks, stop=lambda: nut['tau_tight'] >= target, rel=rel)
    dr.unload(wb, rel); disengage(dr, w, N)
    return hit


LOOSEN_START = dict(latch=(-70., 80.), fl1=(-70., 80.), fl2=(-140., 45.))   # (start wrench yaw deg, stroke deg): handle off arm/tab


def loosen_nut(dr, w, n, deg=None, stop_deg=RELEASE_DEG + 8., max_strokes=4):
    start, stroke = LOOSEN_START.get(n, (-110., 60.)); stroke = stroke if deg is None else deg
    kn = dr.k['nuts'][n]; wb = dr.k['wrenches'][w]['body']; hit = False
    for _ in range(max_strokes):
        y0, N = engage(dr, w, n, np.radians(start)); rel = dr.rel(wb)
        hit = turn(dr, w, N, y0, np.radians(stroke), int(2600 * stroke / 80.) + 400,
                   stop=lambda: nut_angle(dr.d, kn) >= np.radians(stop_deg), rel=rel)
        dr.unload(wb, rel); disengage(dr, w, N)
        if hit or nut_angle(dr.d, kn) >= np.radians(stop_deg): return True
    return hit


def work_tighten(dr, n, start_deg=18., limit_deg=45.):
    N = dr.d.xpos[dr.k['nuts'][n]['base']][:2].copy(); w0 = WR_YAW + np.radians(start_deg)
    return N, [w0 + np.radians(a) for a in (30., 0., -limit_deg / 2, -limit_deg - 30.)]


def work_loosen(dr, n):
    start, stroke = LOOSEN_START.get(n, (-110., 60.)); N = dr.d.xpos[dr.k['nuts'][n]['base']][:2].copy()
    return N, [np.radians(start + a) for a in (-30., 0., stroke / 2, stroke + 30.)]


def ref_w1(seed, params):
    dr = Drive(seed, 'W1_latch_release', params)
    w = wrench_for(params['nut_size_yellow'])
    pick_wrench(dr, w, work=work_loosen(dr, 'latch')); loosen_nut(dr, w, 'latch'); return_wrench(dr, w)
    # swing the arm to its open stop by its knob
    x, y = W1_NUT; ztip = SEAT_Z - .0045 + .0022 + .036 - .004
    def knob(a): return np.array([x - ARM_LEN * np.sin(a), y + ARM_LEN * np.cos(a), ztip])
    dr.go(knob(0.) + [0, 0, .06], 0., .02, n=500)
    dr.go(knob(0.), None, .02, n=300)
    dr.go(None, None, 0., n=150, settle=60)
    a_end = -np.radians(92.)
    for i in range(1200):
        if i % 4 == 0:
            f = smooth((i + 1) / 1000); a = a_end * f
            dr.cmd, dr.cyaw = knob(a), a; dr.ik(dr.cmd, dr.cyaw)
        dr.tick()
    dr.go(None, None, .02, n=120, settle=30)
    dr.go(dr.cmd + [0, 0, .07], None, .02, n=300)
    park(dr)
    return finish(dr), dr


def ref_w2(seed, params):
    dr = Drive(seed, 'W2_mixed_flange', params)
    for n, lab in zip(('fl1', 'fl2'), (params['nut_size_yellow'], params['nut_size_cyan'])):
        w = wrench_for(lab); pick_wrench(dr, w, work=work_loosen(dr, n)); loosen_nut(dr, w, n); return_wrench(dr, w)
    # slide the freed cover out of its slots, lift it and set it on the shelf
    cb = dr.k['cover']['body']; o = dr.d.xpos[cb].copy(); tab = o + [.014, 0, .038 - .004]
    dr.go(tab + [0, 0, .06], 0., .02, n=500); dr.go(tab, None, .02, n=300); dr.go(None, None, 0., n=150, settle=80)
    sx, sy, sz = SHELF; dest = np.array([sx + .005, sy, sz + .002])
    P = lambda a, b: (lambda f: (a + (b - a) * f, 0.))
    o1 = o + [-.045, 0, 0]; o2 = o1 + [0, 0, .04]; o3 = np.r_[dest[:2], o2[2]]
    dr.servo(cb, P(o, o1), 700); dr.servo(cb, P(o1, o2), 400); dr.servo(cb, P(o2, o3), 900)
    dr.servo(cb, P(o3, dest + [0, 0, .0006]), 500, settle=80, emax=.001)
    dr.go(None, None, .02, n=120, settle=40); dr.relax()
    dr.go(dr.cmd + [0, 0, .07], None, .02, n=300)
    park(dr)
    return finish(dr), dr


def ref_w3(seed, params):
    dr = Drive(seed, 'W3_clamp_tighten', params)
    w = wrench_for(params['nut_size_orange']); pick_wrench(dr, w, work=work_tighten(dr, 'clamp', 35., 70.))
    for _ in range(3):
        if tighten_nut(dr, w, 'clamp', start_deg=35., limit_deg=70., n_ticks=2600): break
    return_wrench(dr, w); park(dr)
    return finish(dr), dr


def ref_task_a(seed, params):
    dr = Drive(seed, TASK_A, params)
    for n in ('orange', 'purple'):
        w = wrench_for(params['nut_size_' + n]); pick_wrench(dr, w, work=work_tighten(dr, n)); tighten_nut(dr, w, n); return_wrench(dr, w)
    park(dr)
    return finish(dr), dr


YHI, YLO = np.radians(195.), np.radians(-70.)      # wrist stroke for screwing (clockwise = decreasing yaw)
GAP_STOP = .00019                                   # stop screwing once the knob is snug (seated within 0.5 mm)


def near_mod(a, ref, period):
    """angle congruent to a (mod period) closest to ref"""
    return ref + ((a - ref + period / 2) % period) - period / 2


def bolt_body(dr, c): return dr.k['bolts'][c]['body']


def pick_bolt(dr, c, want=np.pi / 2):
    kb = bolt_body(dr, c); o, yb = dr.bpose(kb)
    gy = near_mod(yb, want, np.pi / 2); top = o + [0, 0, KNOB_H - .002]
    dr.go(top + [0, 0, .07], gy, .016, n=500, unwrap=True)
    dr.go(top, None, .016, n=300)
    dr.go(None, None, 0., n=150, settle=80)
    dr.go(top + [0, 0, .045], None, 0., n=300)


def bolt_tip(dr, kb): R = dr.d.xmat[kb].reshape(3, 3); return dr.d.xpos[kb] - R[:, 2] * LS


def insert_bolt(dr, c, hn, tries=4):
    """Centre the screw TIP over the hole (privileged pose; the screw may sit a degree tilted in the jaws), lower it onto the
    thread start; retry if it lands off."""
    kb = bolt_body(dr, c); hx, hy = dr.st['holes'][hn]['xy']; o, yb = dr.bpose(kb)
    axis = lambda: dr.d.xmat[kb].reshape(3, 3)[:, 2] * LS
    t_entry = np.array([hx, hy, BLOCK_TOP + .0015]); t_down = np.array([hx, hy, BLOCK_TOP - CLEAR_D - .0013])
    centred = lambda: np.hypot(*(bolt_tip(dr, kb)[:2] - [hx, hy])) < .00008
    seated = lambda: (BLOCK_TOP - CLEAR_D) - bolt_tip(dr, kb)[2] >= .0011 and np.hypot(*(bolt_tip(dr, kb)[:2] - [hx, hy])) < .0006
    above = t_entry + [0, 0, LS + .0045]
    dr.servo(kb, lambda f: (o + (above - o) * f, yb), 700, settle=100)
    for _ in range(tries):
        cur = bolt_tip(dr, kb)
        dr.servo(kb, lambda f: (cur + (t_entry - cur) * f + axis(), yb), 300)
        dr.servo(kb, lambda f: (t_entry + axis(), yb), 1, settle=1500, stop=centred)
        dr.servo(kb, lambda f: (t_entry + (t_down - t_entry) * f + axis(), yb), 500, settle=150)
        if seated(): return True
    return False


def screw_bolt(dr, c, hn, gap_stop=GAP_STOP, cap=None, max_strokes=8, speed=np.radians(60.)):
    """Turn the gripped bolt clockwise in wrist strokes with regrasps until the knob is snug (privileged gap), or until the
    hole's thread torque exceeds `cap` (privileged; used by probes). Returns the reason it stopped."""
    kb = bolt_body(dr, c); hx, hy = dr.st['holes'][hn]['xy']; h = dr.st['holes'][hn]
    for k in range(max_strokes):
        rel = dr.rel(kb); y0 = dr.cyaw; n = max(50, int(abs(y0 - YLO) / speed / DT))
        why = []
        def stop():
            if h['bolt'] == c and thread_gap(dr.d, dr.st, dr.k, hn) <= gap_stop: why.append('snug'); return True
            if dr.qg[0][6] > 2.85 or dr.qg[0][6] < -2.85: return 'wrist'
            if cap is not None and h['tmax'] > cap: why.append('cap'); return True
            return False
        zin = BLOCK_TOP + LS - CLEAR_D - .0013
        live = lambda f: (np.r_[hx, hy, min(dr.d.xpos[kb][2], zin) if h['bolt'] == c else zin], y0 + rel[2] + (YLO - y0) * f)
        if dr.servo(kb, live, n, stop=stop, rel=rel) and why: break
        # regrasp: open, swing the open jaws back, close on the flats
        dr.go(None, None, .016, n=100, settle=20); dr.relax()
        o, yb = dr.bpose(kb); gy = near_mod(yb - rel[2], YHI, np.pi / 2)
        dr.go(None, gy, .016, n=int(abs(gy - dr.cyaw) / np.radians(150.) / DT) + 50, settle=20, unwrap=True)
        tip = np.r_[hx, hy, dr.d.xpos[kb][2] + KNOB_H - .002]
        dr.go(tip, None, .016, n=80, settle=10); dr.go(None, None, 0., n=120, settle=40)
    else:
        why = ['strokes']
    dr.go(None, None, .016, n=100, settle=20); dr.relax()      # open first: unloading a gripped knob would back the screw off
    dr.go(dr.cmd + [0, 0, .06], None, .016, n=300, settle=20)
    return why[0] if why else 'none'


def ref_w4(seed, params):
    dr = Drive(seed, 'W4_thumb_bolt_aluminium', params)
    m8 = [c for c, th in zip(BOLT_COLOURS, params['bolt_threads']) if th == 'M8']
    for c, hn in zip(m8, ('cov_a', 'cov_b')):
        pick_bolt(dr, c); insert_bolt(dr, c, hn); screw_bolt(dr, c, hn)
    park(dr)
    return finish(dr), dr



# ---------------------------------------------------------------- references, probes, rule tests
REFS = {TASK_A: ref_task_a, 'W1_latch_release': ref_w1, 'W2_mixed_flange': ref_w2, 'W3_clamp_tighten': ref_w3,
        'W4_thumb_bolt_aluminium': ref_w4}


def reference(seed, target, params): return REFS[target](seed, params or parameters(seed))[0]


ROCK_T, RAMP_T, BOLT_T = 2500, 5000, 2900   # measurement windows (last ticks of each probe)


def arc_open(dr, N, rel, yaw_of, n):
    """Open-loop: hand follows the arc that puts the grasped wrench at yaw yaw_of(i) about axis N (fixed grasp transform)."""
    rxy, rz, ry = rel
    for i in range(n):
        if i % 4 == 0:
            yt = dr.cyaw + wrap((yaw_of(i) - ry) - dr.cyaw); pt = np.r_[N[:2] - Rz2(yt) @ np.asarray(rxy), N[2] - rz]
            dr.cmd, dr.cyaw = pt, yt; dr.ik(pt + dr.e, yt + dr.ey)
        dr.tick()


def _engaged(seed, params, nut, w, want):
    dr = Drive(seed, None, params); wb = dr.k['wrenches'][w]['body']
    N0 = dr.d.xpos[dr.k['nuts'][nut]['base']][:2].copy()
    pick_wrench(dr, w, work=(N0, [want + np.radians(a) for a in (-30., 0., 30.)]))
    _, N = engage(dr, w, nut, want); rel = dr.rel(wb); dr.unload(wb, rel)
    y1 = dr.bpose(wb)[1]; dr.servo(wb, lambda f: (N, y1), 1, settle=300, rel=rel)   # re-learn the sag offset, no turning load
    return dr, N, rel, dr.bpose(wb)[1]


def _rock(seed, params, nut, w, amp):
    dr, N, rel, y0 = _engaged(seed, params, nut, w, WR_YAW)
    arc_open(dr, N, rel, lambda i: y0 + np.radians(amp) * np.sin(2 * np.pi * i / (ROCK_T / 2)), ROCK_T)
    return dr.out()


def _ramp(seed, params, nut, w, deg):
    """slow open-loop loosening ramp (constant wrist speed) with wrench w on nut"""
    dr, N, rel, y0 = _engaged(seed, params, nut, w, WR_YAW - np.radians(20.))
    arc_open(dr, N, rel, lambda i: y0 + np.radians(deg) * i / RAMP_T, RAMP_T)
    return dr.out()


def half_inch_practice(p): return 'yellow' if p['nut_size_yellow'] == '1/2in' else 'cyan'


def first_unc(p): return [c for c, th in zip(BOLT_COLOURS, p['bolt_threads']) if th == 'UNC'][0]


def first_m8(p): return [c for c, th in zip(BOLT_COLOURS, p['bolt_threads']) if th == 'M8'][0]


def screw_open(dr, c, hn, strokes=2, stroke_ticks=3000):
    """Fixed-timing wrist strokes with regrasps (no stop condition): an open-loop screwing motion of BOLT_T ticks."""
    kb = bolt_body(dr, c); hx, hy = dr.st['holes'][hn]['xy']; zin = BLOCK_TOP + LS - CLEAR_D - .0013
    for k in range(strokes):
        rel = dr.rel(kb); y0 = dr.cyaw
        for i in range(stroke_ticks):
            if i % 4 == 0:   # (generated closed-loop on the screw height, then replayed open-loop for both contrast values)
                f = smooth((i + 1) / (stroke_ticks - 100)); dr.cyaw = y0 + (YLO - y0) * f
                zb = min(dr.d.xpos[kb][2], zin) if dr.st['holes'][hn]['bolt'] == c else zin
                dr.cmd = np.r_[np.array([hx, hy]) - Rz2(dr.cyaw) @ np.asarray(rel[0]), zb - rel[1]]; dr.ik(dr.cmd + dr.e, dr.cyaw + dr.ey)
            dr.tick()
        if k == strokes - 1: break
        dr.go(None, None, .016, n=100, settle=20)
        o, yb = dr.bpose(kb); gy = near_mod(yb - rel[2], YHI, np.pi / 2)
        dr.go(None, gy, .016, n=1200, settle=20, unwrap=True)
        dr.go(None, None, 0., n=140, settle=40)


def _bolt(seed, params, c, hn, strokes=2):
    dr = Drive(seed, None, params); pick_bolt(dr, c); insert_bolt(dr, c, hn); dr.tick(100); n0 = len(dr.acts)
    screw_open(dr, c, hn, strokes)
    return dr.out(), len(dr.acts) - n0


_CACHE = {}


def _cached(key, fn):
    if key not in _CACHE: _CACHE[key] = fn()
    return _CACHE[key]


def _canon(params, cell, value): p = dict(params); p[cell] = value; return p


def probe_factory(name, cell, canon, gen):
    """The arm is torque-driven, so a replayed torque sequence would not repeat the motion once contact differs. A probe is
    therefore the same scripted Cartesian plan for both contrast values (fixed targets from visible geometry, PD-tracked
    like the agent's move tool, no decision depends on the probed value); only the resulting torques differ."""
    return lambda seed, params: gen(seed, params)


def _bolt_actions(seed, p, c, hn):
    a, n = _bolt(seed, p, c, hn); return a


PROBES = {}
for _n in ('orange', 'purple', 'yellow', 'cyan'):   # seat nuts run free: +-5 deg rock; seized nuts: +-1 deg (below breakaway)
    PROBES['rock13_' + _n] = probe_factory('rock13_' + _n, 'nut_size_' + _n, '13mm',
                                           lambda s, p, _n=_n: _rock(s, p, _n, 'w13', 5. if _n in ('orange', 'purple') else 1.))
for _n in ('yellow', 'cyan'):
    PROBES['loosen_' + _n] = probe_factory('loosen_' + _n, 'breakaway_' + _n, 350.,
                                           lambda s, p, _n=_n: _ramp(s, p, _n, wrench_for(p['nut_size_' + _n]), 20.))
PROBES['strip13_half'] = probe_factory('strip13_half', 'strip_torque', 180., lambda s, p: _ramp(s, p, half_inch_practice(p), 'w13', 25.))
PROBES['rock_adjustable'] = probe_factory('rock_adjustable', 'adjustable_slack', 450., lambda s, p: _rock(s, p, 'orange', 'wadj', 10.))
PROBES['screw_red_block'] = probe_factory('screw_red_block', 'bolt_threads', list(PERMS[0]), lambda s, p: _bolt_actions(s, p, 'red', 'blk_m8'))
PROBES['screw_unc_offcut'] = probe_factory('screw_unc_offcut', 'al_gall_torque', 120., lambda s, p: _bolt_actions(s, p, first_unc(p), 'scrap_m8'))


def _ma(x, n=25): return np.convolve(x, np.ones(n) / n, 'valid') if len(x) > n else np.asarray(x)
def _fxy(tr, T): F = np.asarray(tr['wrench'])[-T:, 0:2]; return np.linalg.norm(F - F[0], axis=1)
def stat_rock_median(tr): return float(np.median(_fxy(tr, ROCK_T)))           # N, typical horizontal hand force while rocking
def stat_rock_contact(tr): return float(np.mean(_ma(_fxy(tr, ROCK_T)) > .3))  # fraction of the rock with jaw contact
def stat_ramp_peak(tr): return float(_ma(_fxy(tr, RAMP_T)).max())            # N, peak (25-tick mean) hand force on the ramp
def stat_bolt_peak(tr):
    """N m: largest thread-resisting wrist torque about the screw axis in the last stroke (screwing clockwise loads the wrist with
    negative Mz): 10-tick mean, minus the slow linear drift of the reading fitted on the free-running part of the stroke."""
    Mz = _ma(np.asarray(tr['wrench'])[-BOLT_T:, 5], 10); t = np.arange(len(Mz))
    a, b = np.polyfit(t[300:1100], Mz[300:1100], 1); return float(-(Mz - (a * t + b))[300:].min())


HIDDEN_CELLS = [dict(name='nut_size_' + n, type='G', object=n + '_head', contrast=('13mm', '1/2in'), probe='rock13_' + n, channel='force',
                     statistic=stat_rock_contact if n in ('orange', 'purple') else stat_rock_median) for n in ('orange', 'purple', 'yellow', 'cyan')]
HIDDEN_CELLS += [dict(name='breakaway_' + n, type='P', object=n + '_head', contrast=(350., 500.), probe='loosen_' + n, channel='force',
                      statistic=stat_ramp_peak) for n in ('yellow', 'cyan')]
HIDDEN_CELLS += [
    dict(name='strip_torque', type='M', object='yellow_head', contrast=(180., 250.), probe='strip13_half', channel='force', statistic=stat_ramp_peak),
    dict(name='adjustable_slack', type='P', object='wadj', contrast=(450., 650.), probe='rock_adjustable', channel='force', statistic=stat_rock_contact),
    dict(name='bolt_threads', type='G', object='bolt_red', contrast=(list(PERMS[0]), list(PERMS[3])), probe='screw_red_block', channel='force',
         statistic=stat_bolt_peak),
    dict(name='al_gall_torque', type='P', object='al_scrap', contrast=(120., 140.), probe='screw_unc_offcut', channel='force', statistic=stat_bolt_peak),
]


def _rt_strip(seed, ok):
    p = parameters(seed); n = half_inch_practice(p)
    if ok: return _ramp(seed, p, n, 'w13', 25.), p
    return _ramp(seed, p, n, 'w1_2', 25.), p


def _rt_arm(seed, ok):
    p = parameters(seed); dr = Drive(seed, 'W1_latch_release', p); w = wrench_for(p['nut_size_yellow'])
    pick_wrench(dr, w, work=work_loosen(dr, 'latch')); loosen_nut(dr, w, 'latch', stop_deg=RELEASE_DEG + 8. if ok else 35.); dr.tick(200)
    return dr.out(), p


def _rt_cover(seed, ok):
    p = parameters(seed); dr = Drive(seed, 'W2_mixed_flange', p)
    for n in (('fl1', 'fl2') if ok else ('fl1',)):
        w = wrench_for(p['nut_size_yellow' if n == 'fl1' else 'nut_size_cyan']); pick_wrench(dr, w, work=work_loosen(dr, n)); loosen_nut(dr, w, n); return_wrench(dr, w)
    dr.tick(200)
    return dr.out(), p


def _rt_engage(seed, ok):
    p = parameters(seed); c = first_m8(p); dr = Drive(seed, None, p); pick_bolt(dr, c)
    if ok: insert_bolt(dr, c, 'blk_m8'); screw_open(dr, c, 'blk_m8', strokes=1)
    else:
        hx, hy = dr.st['holes']['blk_m8']['xy']; kb = bolt_body(dr, c); o, yb = dr.bpose(kb); above = np.array([hx, hy, BLOCK_TOP + LS + .004])
        dr.servo(kb, lambda f: (o + (above - o) * f, yb), 700, settle=200); dr.servo(kb, lambda f: (above, yb - np.radians(150.) * f), 1200)
    dr.tick(200)
    return dr.out(), p


def _rt_gall(seed, ok):
    p = parameters(seed)
    if ok: a, _ = _bolt(seed, p, first_unc(p), 'scrap_m8'); return a, p
    c = first_m8(p); dr = Drive(seed, None, p); pick_bolt(dr, c); insert_bolt(dr, c, 'scrap_m8'); screw_bolt(dr, c, 'scrap_m8')
    return dr.out(), p


def _rt_clamp(seed, ok):
    p = parameters(seed) if ok else dict(parameters(seed), test_no_cover=True)
    return ref_w4(seed, p)[0], p


RULE_TESTS = [
    dict(name='cover_clamp', event_key='clamped_event', target='W4_thumb_bolt_aluminium', positive=lambda s: _rt_clamp(s, True),
         negative=lambda s: _rt_clamp(s, False)),
    dict(name='nut_strip', event_key='rounded_event', positive=lambda s: _rt_strip(s, True), negative=lambda s: _rt_strip(s, False)),
    dict(name='arm_release', event_key='arm_released', target='W1_latch_release', positive=lambda s: _rt_arm(s, True), negative=lambda s: _rt_arm(s, False)),
    dict(name='cover_release', event_key='cover_released', target='W2_mixed_flange', positive=lambda s: _rt_cover(s, True),
         negative=lambda s: _rt_cover(s, False)),
    dict(name='thread_engage', event_key='engaged_event', positive=lambda s: _rt_engage(s, True), negative=lambda s: _rt_engage(s, False)),
    dict(name='thread_gall', event_key='galled_event', positive=lambda s: _rt_gall(s, True), negative=lambda s: _rt_gall(s, False)),
]


# ---------------------------------------------------------------- public text
ROBOT_TXT = ('Robot: two Panda arms with parallel-jaw hands; the left arm reaches the work area, the right arm is parked. Joint torque limits '
             '80 N m on joints 1-4, 12 N m on joints 5-7. Each finger squeezes with at most 40 N. Your force guard stops the arm at 30 N by '
             'default (settable 5-60 N); a wrist force above 80 N for 0.1 s aborts the episode. Reach box of the hand: '
             'X -0.30..0.50 m, Y -0.50..0.65 m, Z 0.825..1.27 m (table top at Z 0.80 m; the hand starts at Z 1.26 m); everything on '
             'the bench lies inside it. The hand points straight down; only its yaw can be set.')
RACK_TXT = ('A wrench rack holds, from front to back, open-end wrenches stamped 12 mm, 1/2 in, 13 mm and 14 mm, and an adjustable wrench '
            '(its jaw is already set; it has no size stamp).')
SCENE_TXT = {
    'explore': (f'{RACK_TXT} Two practice seats each carry a hex nut on a stud (orange and purple paint marks; sizes not marked). '
                'Two old practice nuts sit on posts (yellow and cyan marks; scrap, they may be ruined). A steel tapped block has two holes, '
                'the white label reads M8x1.25 and the black label reads 5/16-18. An aluminium offcut '
                'has one M8x1.25 tapped hole. Four knob screws with red, blue, green and white knobs hang in a rail; they look identical apart '
                'from the colour.'),
    'W1': f'{RACK_TXT} A swing arm is clamped by a captive hex nut (paint mark on the nut, size not marked). A stop block marks its open position.',
    'W2': (f'{RACK_TXT} A cover plate is held by two captive hex nuts (paint marks, sizes not marked) in open-ended slots; there is a shelf '
           'to the left.'),
    'W3': f'{RACK_TXT} A clamp is closed by a captive hex nut (paint mark, size not marked).',
    'W4': ('An aluminium cover lies in a pocket on an aluminium base; the drawing marks both tapped holes M8x1.25. Four knob screws with red, '
           'blue, green and white knobs hang in a rail; they look identical apart from the colour.'),
}
GOAL_TXT = {
    None: 'Goal: explore the bench freely. The task is given after a reset.',
    TASK_A: ('Goal: tighten the nuts on both practice seats to between 0.30 and 0.40 N m. Then take the tools away and keep both hands off '
             'the nuts for 20 s.'),
    'W1_latch_release': ('Goal: loosen the nut and swing the arm to its open stop. Then take the tools away and keep both hands off the arm '
                         'and the nut for 20 s.'),
    'W2_mixed_flange': ('Goal: loosen both nuts, slide the cover out of its slots, lift it off and set it flat on the shelf. Then keep both '
                        'hands off the cover for 20 s.'),
    'W3_clamp_tighten': ('Goal: tighten the nut so that the clamp is closed and the nut is at 0.30 to 0.40 N m. Then take the tools away '
                         'and keep both hands off the nut for 20 s.'),
    'W4_thumb_bolt_aluminium': ('Goal: fasten the cover with two M8x1.25 knob screws, both seated and tightened to at least 0.05 N m of knob torque, so the cover is clamped in its pocket. Then '
                                'keep both hands off the screws and the cover for 20 s.'),
}


def public_text(target): return f'{SCENE_TXT[SCENE_OF[target]]} {ROBOT_TXT} {GOAL_TXT[target]}'
