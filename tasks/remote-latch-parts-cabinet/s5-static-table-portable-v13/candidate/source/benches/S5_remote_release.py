# Card: cards/S5_remote_release.md (S5 remote-release rotary latch, two double-door cabinets with side paddles).
# Robot: the repo's two-Panda B4 robot in its side-by-side layout (portable_demo.env.scene(seed, 'train_a'), non-robot
# bodies stripped; same Pandas/gains/sensors as 'test'), stock Franka hands (+-40 N per finger). Left arm works the
# cabinet on the -Y side (A, or the new cabinet N in T4/T5; it stands 0.13 m inboard of the left base's line, lead
# decision #136), right arm works cabinet B on the +Y side. Every robot action
# is top-down with yaw only (the agent tool API, DUAL_POSE). See DEVIATIONS.md for all changes from the card.
# Actions are data.ctrl rows (nu = 18): 14 arm joint torques (residual: step() adds the arm gravity/bias compensation) and
# 4 finger position targets (0 closed .. .045 open, per finger).
# Mechanism (no cable or rotary claw is simulated; all rules read simulator state only):
#  - door = hinge joint. A passive torque element on each hinge (written in step() as a function of the hinge angle only,
#    like a physical spring) gives: a kick-off spring (door pops open ~14 deg when released), on two-stage doors only a
#    claw detent bump at 1.4-2.8 deg (first force peak when closing, then the force drops; felt only while the claw is
#    armed), and a gasket spring over the last 1.2 deg whose force at the knob is the hidden second-stage force f2.
#  - latch state per door in {open, stage1, stage2} reads only the hinge angle: on a two-stage door, past the detent
#    (< 1.35 deg) the safety catch engages (hinge range upper limit -> 1.35 deg, near flush); at the gasket bottom
#    (< 0.10 deg) the claw engages (upper limit -> 0.15 deg). Single-stage doors have no detent and no first stage.
#    two_stage is drawn independently per door (parameters()).
#  - paddle = spring-loaded slide joint on the cabinet side; while a paddle is pulled past its release travel, every door
#    it is cabled to is released (range restored). A splitter paddle drives two cables, so its return spring is 2x.
import os, sys, struct, zlib, math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation

REPO = next(p / 'native_support' for p in Path(__file__).resolve().parents if (p / 'native_support').is_dir())
sys.path[:0] = [str(REPO), str(REPO / 'mujoco-box-demo')]
from dual_arm_demo.env import HOME, DOWN
from portable_demo.env import scene as portable_scene

BENCH_ID = 'S5_remote_release'
SEEDS = (42, 43, 44)
LIM = np.array([80.] * 5 + [12.] * 2)  # arm torque limits (dual_arm_demo/env.py step_targets)
KP, KD = 550., 45.
TABLE_Z = .80
DEG = math.pi / 180.

# ---- cabinet geometry (local frame: origin at the centre of the front plane at table level; the doors face local -X;
# local +Y is the viewer's left when facing the doors). Dimensions: see ASSETS.md (scaled benchtop double-door enclosure).
W, D, H = .36, .18, .50          # outer width (Y), depth (X), height (Z)
WALL = .01
DT = .012                        # door thickness
DW = W / 2 - .004                # door width (4 mm centre gap)
DZ0, DZ1 = .012, .490            # door bottom/top (local z)
DOOR_MASS = .75
RK = .136                        # hinge axis -> knob axis distance (force reference radius for f2)
ZK = .455                         # knob height (local z)
LK = .040                        # knob standoff length
PX = (.045, .110)                # paddle H1 (front), H2 (rear) local x on the +Y side face
PZ = .45                         # paddle height (local z)
P_TRAVEL, P_RELEASE = .012, .009 # paddle travel and release travel (m)
P_K1, P_PRE = 500., .004         # single-cable paddle spring N/m and preload offset (m); splitter = 2x
BOX = (.025, .025, .03)          # box half sizes
# Each box stands on the cabinet floor behind its door, with a grab tab (fin) on its front face that reaches to 3 mm
# behind the closed door; a top-down gripper pinches the tab at the door opening and slides the box out.
FIN = (.0185, .010, .022)        # tab half sizes (x along the depth, y thickness, z)
BOX_LOCAL = {'L': (.065, .088, WALL + BOX[2] + .0005), 'R': (.065, -.088, WALL + BOX[2] + .0005)}
TH_OPEN = 100 * DEG              # reference opening angle for box access (door folded out of the arm's way)
# per cabinet (lead decision #144): B's door is swung further (125 deg, hinge stop 140) so the right forearm, whose base
# sits on B's centre line, clears the open door while it reaches the box tab (at 100 deg it leaned on it at 60-420 N)
TH_OPEN_BOX = {'A': TH_OPEN, 'N': TH_OPEN, 'B': 125 * DEG}
BOX_MASS = .30
# hinge torque profile (angles in rad, opening angle theta >= 0)
TH_KICK, F_KICK = 12 * DEG, 3.0           # kick-off spring active below 12 deg, 3 N at the knob at theta = 0
TH_DET, W_DET, F_DET = 2.1 * DEG, .7 * DEG, 8.0    # first-stage claw detent: centre, half width, peak force at the knob
TH_GASK = 1.2 * DEG                        # gasket compression zone (last 1.2 deg = 2.8 mm at the knob)
TH_S1 = 1.35 * DEG                         # first (safety) stage: door held 1.35 deg open (3.2 mm at the knob, near flush)
TH_S2, TH_S2_LIM = .10 * DEG, .15 * DEG    # second stage: claw engages at 0.10 deg, door then held within 0.15 deg
TH_MIN, TH_MAX = -.15 * DEG, 140 * DEG     # frame stop (gasket fully compressed) / hinge stop
TH_ARM = TH_DET + W_DET                    # a released claw re-arms only after the striker has left it (door > 2.8 deg)
V_SLAM = .25                               # door_slam: knob speed at the frame stop above 0.25 m/s (estimated, see DEVIATIONS)
F_ABORT, ABORT_TICKS = 80., 50             # wrist_overload: untared wrist force above 80 N for 0.1 s (the agent abort level)
DOOR_FRICTION, DOOR_DAMPING = .08, .15
HOLD_S = 20.                               # card: released 20 s
# Start poses: both arms tucked toward their own bases (tips 28 cm in front of each base at z 1.12, 5 cm outboard)
# so the arms start clear of the cabinets.
PARK_Q = (np.array([-0.073, -0.892, -0.092, -2.797, -0.075, 1.907, 0.68]),
          np.array([-0.073, -0.892, -0.092, -2.797, -0.075, 1.907, 0.68]))

# cabinet placement: name -> (front-plane centre x, y, yaw). A/N for the left arm, B for the right arm.
# Layout (2026-10-29, lead decision #136): cabinet A/N stands 0.13 m toward the middle of the table (+Y) from the left
# base's line (was on it), so the left base sits 42 mm outboard of the RIGHT compartment's box (box R at world y -0.378,
# base y -0.42: the base is offset toward the compartment it works in T1/T4/T5). With the base on the cabinet
# centre line the forearm cleared the closed left door at the right box only in a twisted arm branch, which the stock IK
# left by an elbow flip at ~3x the joint-speed limit. Cabinet B is unchanged (right arm: no such branch).
PLACE = {'A': (.10, -.29, 0.), 'N': (.10, -.29, 0.), 'B': (.18, .30, 0.)}
# side face carrying the paddles (local +Y = 1 = the viewer's left, -1 = right): both on the side facing the middle of the
# table (A/N: world +Y side, B: world -Y side)
PSIDE = {'A': 1, 'N': 1, 'B': -1}
ARM_OF = {'A': 0, 'N': 0, 'B': 1}
MAPS = {'L/R': {1: 'L', 2: 'R'}, 'R/L': {1: 'R', 2: 'L'}, 'LR/R': {1: 'LR', 2: 'R'}, 'R/LR': {1: 'R', 2: 'LR'}}

# Robot layout: the repo's two-Panda B4 robot in its side-by-side base layout (portable_demo case 'train_a': left base
# (-.40,-.42), right base (-.32,.30), both facing +X). Each arm faces its own cabinet head-on (see DEVIATIONS).
ROBOT_CASE = 'train_a'

TASK_A = 'A_left_A'
RULES = ['paddle_release', 'stage1_catch', 'stage2_catch']
TARGETS = [
    # Lead decision #189 (strict #187 line: a cell stays on a target only if the informed plan beats the BEST uninformed
    # strategy by >= 10 % of the full task time, or by a success/damage gap; probes/p189/). f2_* / two_*: the default agent
    # push (8 mm per tick, 30 N guard) latches every door, any f2 and latch type, in 0.35-0.45 s with no slam or overload
    # (56/56), so knowing them saves nothing. map_A / map_B: pulling the front paddle first and looking which door popped
    # costs on average +8.8 % (T1), -0.5 % (T2), +7.9 % (T4) and +3.6 % (T5) of the informed full task time, no failures.
    # So no deployment target claims a cell (skill targets); the cells stay sampled, probed and listed in task A.
    dict(id=TASK_A, settings=['A'], cells=['map_A', 'f2_AL', 'two_AL'], rules=RULES),
    dict(id='RR-T1', settings=['S'], cells=[], rules=['paddle_release']),
    dict(id='RR-T2', settings=['S'], cells=[], rules=['paddle_release']),
    dict(id='RR-T3', settings=['S'], cells=[], rules=['stage1_catch', 'stage2_catch']),
    dict(id='RR-T4', settings=['T-new'], cells=[], rules=RULES, target_objects=['cab_N']),
    dict(id='RR-T5', settings=['T-combo'], cells=[], rules=RULES, target_objects=['cab_N']),
]
GOALS = {TASK_A: dict(cab='A', boxes=['L'], doors=['L']), 'RR-T1': dict(cab='A', boxes=['R'], doors=[]),
         'RR-T2': dict(cab='B', boxes=['R'], doors=[]), 'RR-T3': dict(cab='B', boxes=[], doors=['L', 'R']),
         'RR-T4': dict(cab='N', boxes=['R'], doors=['L', 'R']), 'RR-T5': dict(cab='N', boxes=['R'], doors=['L', 'R'])}


def other(v, pair): return pair[1 - pair.index(v)]


def parameters(seed):
    r = np.random.default_rng(np.random.SeedSequence([seed, 5505]))
    # per door: two-stage (safety catch first) or single-stage latch, each door an independent fair coin (own stream, so
    # the f2 draws above are unchanged). Stream key 5513 is the first key from 5507 up whose dev seeds 42/43/44 show both
    # values of every two_* flag (accept.py check 7) and both equal and unequal door pairs in each cabinet.
    t = np.random.default_rng(np.random.SeedSequence([seed, 5513])).random(4) < .5
    return dict(map_A=('L/R', 'R/L')[seed % 2], map_B=('LR/R', 'R/LR')[(seed // 2) % 2],
                f2_AL=float(r.uniform(15., 25.)), f2_AR=float(r.uniform(15., 25.)),
                f2_BL=float(r.uniform(34., 45.)), f2_BR=float(r.uniform(34., 45.)),
                two_AL=bool(t[0]), two_AR=bool(t[1]), two_BL=bool(t[2]), two_BR=bool(t[3]))


def transfer_parameters(params, seed):
    # T-new: a new cabinet in A's place with a resampled cable map (always different from A's) and new second-stage
    # forces from the large band (34-45 N; A's are 15-25 N).
    r = np.random.default_rng(np.random.SeedSequence([seed, 5506]))
    choices = [m for m in MAPS if m != params['map_A']]
    T = dict(params)
    T.update(map_A=choices[int(r.integers(len(choices)))], f2_AL=float(r.uniform(34., 45.)), f2_AR=float(r.uniform(34., 45.)),
             two_AL=not params['two_AL'], two_AR=not params['two_AR'])
    return T


def combo_cabinet(p):
    # T-combo: new cabinet; the splitter sits on the other paddle than on B, with B's large second-stage forces swapped.
    return dict(map=('R/LR' if p['map_B'] == 'LR/R' else 'LR/R'), f2L=p['f2_BR'], f2R=p['f2_BL'],
                twoL=bool(p['two_BR']), twoR=bool(p['two_BL']))


def cabinets_for(target, p):
    # name -> dict(map, f2L, f2R, twoL, twoR, init: 'latched' | 'open')
    A = dict(map=p['map_A'], f2L=p['f2_AL'], f2R=p['f2_AR'], twoL=bool(p['two_AL']), twoR=bool(p['two_AR']))
    if target in ('RR-T4',):
        return {'N': dict(A, init='latched')}
    if target == 'RR-T5':
        return {'N': dict(combo_cabinet(p), init='latched')}
    initB = 'open' if target == 'RR-T3' else 'latched'     # RR-T3: cabinet B's (high-force) doors stand open
    return {'A': dict(A, init='latched'),
            'B': dict(map=p['map_B'], f2L=p['f2_BL'], f2R=p['f2_BR'], twoL=bool(p['two_BL']), twoR=bool(p['two_BR']), init=initB)}


T3_OPEN = 40 * DEG


def door_init(c):
    return (T3_OPEN, TH_MAX) if c['init'] == 'open' else (TH_S2_LIM - .01 * DEG, TH_S2_LIM)


def splitter(c, h): return len(MAPS[c['map']][h]) == 2


def cabinet_xml(name, c):
    x0, y0, yaw = PLACE[name]
    q = f'{math.cos(yaw / 2):.6f} 0 0 {math.sin(yaw / 2):.6f}'
    grey, dark, steel = '.62 .64 .66 1', '.10 .10 .11 1', '.80 .80 .82 1'
    ps = PSIDE[name]
    s = [f'<body name="cab_{name}" pos="{x0} {y0} {TABLE_Z}" quat="{q}">',
         f'<geom name="{name}_back" type="box" pos="{D - WALL / 2} 0 {H / 2}" size="{WALL / 2} {W / 2} {H / 2}" rgba="{grey}"/>',
         f'<geom name="{name}_side_l" type="box" pos="{D / 2} {W / 2 - WALL / 2} {H / 2}" size="{D / 2} {WALL / 2} {H / 2}" rgba="{grey}"/>',
         f'<geom name="{name}_side_r" type="box" pos="{D / 2} {-W / 2 + WALL / 2} {H / 2}" size="{D / 2} {WALL / 2} {H / 2}" rgba="{grey}"/>',
         f'<geom name="{name}_top" type="box" pos="{D / 2} 0 {H - WALL / 2}" size="{D / 2} {W / 2} {WALL / 2}" rgba="{grey}"/>',
         f'<geom name="{name}_bottom" type="box" pos="{D / 2} 0 {WALL / 2}" size="{D / 2} {W / 2} {WALL / 2}" rgba="{grey}"/>',
         # low partition between the two compartments (keeps each box behind its own door; low enough for the palm)
         f'<geom name="{name}_divider" type="box" pos="{D / 2} 0 {WALL + .04}" size="{D / 2} .004 .04" rgba="{grey}"/>',
         # recessed paddle pockets on the paddle side face (visual only)
         *[f'<geom name="{name}_pocket{i}" type="box" pos="{PX[i - 1]} {ps * (W / 2 + .001)} {PZ}" size=".022 .001 .016" rgba=".25 .25 .27 1" contype="0" conaffinity="0" mass="0"/>' for i in (1, 2)]]
    for i in (1, 2):
        k = P_K1 * (2 if splitter(c, i) else 1)
        s += [f'<body name="{name}_paddle{i}" pos="{PX[i - 1]} {ps * W / 2} {PZ}">',
              f'<joint name="{name}_paddle{i}_slide" type="slide" axis="0 {ps} 0" range="0 {P_TRAVEL}" stiffness="{k}" springref="{-P_PRE}" damping="3" armature=".02"/>',
              f'<geom name="{name}_paddle{i}_tab" type="box" pos="0 {ps * .026} 0" size=".014 .022 .006" mass=".05" rgba="{dark}"/>',
              '</body>']
    for side, sgn in (('L', 1), ('R', -1)):
        lo, hi = door_init(c)
        hi_r = max(hi, lo + 1e-4)
        s += [f'<body name="{name}_door_{side}" pos="{-DT / 2} {sgn * (W / 2 - .002)} 0">',
              f'<joint name="{name}_hinge_{side}" type="hinge" axis="0 0 {-sgn}" range="{TH_MIN} {hi_r}" damping="{DOOR_DAMPING}" frictionloss="{DOOR_FRICTION}" armature=".01" solreflimit=".004 1"/>',
              f'<geom name="{name}_door_{side}_panel" type="box" pos="0 {-sgn * DW / 2} {(DZ0 + DZ1) / 2}" size="{DT / 2} {DW / 2} {(DZ1 - DZ0) / 2}" mass="{DOOR_MASS}" rgba="{grey}"/>',
              f'<geom name="{name}_door_{side}_knob" type="cylinder" pos="{-DT / 2 - LK / 2} {-sgn * RK} {ZK}" quat=".7071068 0 .7071068 0" size=".015 {LK / 2}" mass=".05" rgba="{steel}"/>',
              f'<geom name="{name}_door_{side}_cap" type="cylinder" pos="{-DT / 2 - LK - .003} {-sgn * RK} {ZK}" quat=".7071068 0 .7071068 0" size=".022 .003" mass=".01" rgba="{steel}"/>',
              '</body>']
    s.append('</body>')
    for side in ('L', 'R'):
        wx, wy, wz = to_world(name, BOX_LOCAL[side]); col = '.85 .55 .15 1' if side == 'L' else '.20 .50 .75 1'
        s += [f'<body name="{name}_box_{side}" pos="{wx:.5f} {wy:.5f} {wz:.5f}" quat="{q}"><freejoint name="{name}_box_{side}_joint"/>',
              f'<geom name="{name}_box_{side}_geom" type="box" size="{BOX[0]} {BOX[1]} {BOX[2]}" mass="{BOX_MASS}" rgba="{col}"/>',
              f'<geom name="{name}_box_{side}_tab" type="box" pos="{-BOX[0] - FIN[0]} 0 0" size="{FIN[0]} {FIN[1]} {FIN[2]}" mass=".01" rgba="{col}"/></body>']
    return '\n'.join(s)


def build(seed, target=None, params=None):
    p = params or parameters(seed)
    root = ET.fromstring(portable_scene(seed, ROBOT_CASE)[0]); world = root.find('worldbody')
    for body in list(world.findall('body')):
        if not body.get('name', '').startswith(('left_', 'right_')): world.remove(body)
    for sec in ('contact', 'equality', 'tendon'):
        node = root.find(sec)
        if node is None: node = ET.SubElement(root, sec)
        for c in list(node): node.remove(c)
    root.find('visual/global').attrib.update(offwidth='1280', offheight='720')
    root.find('size').attrib.update(njmax='6000', nconmax='1000')
    root.set('model', BENCH_ID + ('' if target in (None, TASK_A) else '_' + target))
    cabs = cabinets_for(target, p)
    world.extend(ET.fromstring('<w>' + ''.join(cabinet_xml(n, c) for n, c in cabs.items()) + '''
      <body name="overview_target" pos=".18 -.06 .98"/>
      <camera name="overview" mode="targetbody" target="overview_target" pos="-.80 -.02 2.15" fovy="58"/>
    </w>'''))
    for n in cabs:
        for side in ('L', 'R'):
            ET.SubElement(root.find('contact'), 'exclude', body1=f'cab_{n}', body2=f'{n}_door_{side}')
        for i in (1, 2):
            ET.SubElement(root.find('contact'), 'exclude', body1=f'cab_{n}', body2=f'{n}_paddle{i}')
        ET.SubElement(root.find('contact'), 'exclude', body1=f'{n}_door_L', body2=f'{n}_door_R')
    m = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode')); d = mujoco.MjData(m)
    k = ids(m)
    for q, q0 in zip(k['q'], PARK_Q): d.qpos[q] = q0
    d.qpos[k['fingers']] = .045; d.ctrl[14:] = .045
    for n, c in cabs.items():
        for side in ('L', 'R'):
            d.qpos[m.jnt_qposadr[m.joint(f'{n}_hinge_{side}').id]] = door_init(c)[0]
    mujoco.mj_forward(m, d)
    return m, d


def ids(m):
    arms = [[m.joint(f'{s}_joint{i}').id for i in range(1, 8)] for s in ('left', 'right')]
    robot = [g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name.startswith(('left_', 'right_'))]
    cabs = [n for n in ('A', 'B', 'N') if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f'cab_{n}') >= 0]
    k = dict(q=[m.jnt_qposadr[a].tolist() for a in arms], v=[m.jnt_dofadr[a].tolist() for a in arms],
             fingers=[int(m.jnt_qposadr[m.joint(f'{s}_finger_{f}').id]) for s in ('left', 'right') for f in 'ab'],
             tips=[int(m.site('left_tip').id), int(m.site('right_tip').id)], robot=robot, cabs=cabs, door={}, paddle={}, box={})
    for n in cabs:
        for side in ('L', 'R'):
            j = m.joint(f'{n}_hinge_{side}').id; b = m.body(f'{n}_door_{side}').id
            k['door'][f'{n}{side}'] = dict(jnt=int(j), q=int(m.jnt_qposadr[j]), v=int(m.jnt_dofadr[j]),
                                           geoms=[g for g in range(m.ngeom) if m.geom_bodyid[g] == b])
            bb = m.body(f'{n}_box_{side}').id
            k['box'][f'{n}{side}'] = dict(body=int(bb), geoms=[g for g in range(m.ngeom) if m.geom_bodyid[g] == bb])
        for i in (1, 2):
            j = m.joint(f'{n}_paddle{i}_slide').id
            k['paddle'][f'{n}{i}'] = int(m.jnt_qposadr[j])
    k['table'] = [g for g in range(m.ngeom) if m.geom(g).name == 'table']
    return k


def init_state(seed, target, params):
    p = params or parameters(seed)
    cabs = cabinets_for(target, p)
    st = dict(target=target, cabs={n: dict(map=c['map'], f2L=float(c['f2L']), f2R=float(c['f2R']), twoL=bool(c['twoL']),
                                           twoR=bool(c['twoR'])) for n, c in cabs.items()},
              latch={f'{n}{s}': ('open' if c['init'] == 'open' else 'stage2') for n, c in cabs.items() for s in 'LR'},
              armed={f'{n}{s}': c['init'] == 'open' for n, c in cabs.items() for s in 'LR'},
              paddle_release=False, stage1_catch=False, stage2_catch=False, events=[], hold=0., max_door_deg=0.,
              door_slam=False, wrist_overload=False, over80=[0, 0], max_impact=0.)
    return st


def door_torque(theta, f2, two, state, armed=True):
    # Passive hinge torque (opening positive), a function of the hinge angle only. The detent belongs to the claw of a
    # two-stage latch: it is felt only while the claw is armed (closed, waiting for the striker).
    tau = 0.
    if theta < TH_KICK: tau += F_KICK * RK * (TH_KICK - theta) / TH_KICK
    if theta < TH_GASK: tau += f2 * RK * (TH_GASK - theta) / TH_GASK
    if two and armed and state == 'open' and abs(theta - TH_DET) < W_DET:
        tau += F_DET * RK * math.sin(math.pi * (theta - TH_DET) / W_DET)
    return tau


def step(m, d, state):
    k = state.get('ids') or state.setdefault('ids', ids(m))
    dofs = k['v'][0] + k['v'][1]
    d.qfrc_applied[dofs] = d.qfrc_bias[dofs]
    for key, dd in k['door'].items():
        c = state['cabs'][key[0]]
        d.qfrc_applied[dd['v']] = door_torque(float(d.qpos[dd['q']]), c['f2' + key[1]], c['two' + key[1]], state['latch'][key],
                                              state['armed'][key])
    th_prev = {key: float(d.qpos[dd['q']]) for key, dd in k['door'].items()}
    mujoco.mj_step(m, d)
    # damage: door slammed into the frame stop; wrist force at the agent abort level held for 0.1 s
    for key, dd in k['door'].items():
        th = float(d.qpos[dd['q']])
        if th <= TH_MIN + .02 * DEG < th_prev[key]:
            v = abs(float(d.qvel[dd['v']])) * RK
            vi = max(v, (th_prev[key] - th) / m.opt.timestep * RK)
            state['max_impact'] = max(state['max_impact'], round(vi, 4))
            if vi > V_SLAM: state['door_slam'] = True
    for a, s in enumerate(('left', 'right')):
        fn = float(np.linalg.norm(d.sensor(s + '_force').data))
        state['over80'][a] = state['over80'][a] + 1 if fn > F_ABORT else 0
        if state['over80'][a] >= ABORT_TICKS: state['wrist_overload'] = True
    # paddles -> cables -> latches
    held = set()
    for key, qa in k['paddle'].items():
        if d.qpos[qa] >= P_RELEASE:
            for side in MAPS[state['cabs'][key[0]]['map']][int(key[1])]: held.add(key[0] + side)
    for key, dd in k['door'].items():
        th = float(d.qpos[dd['q']]); c = state['cabs'][key[0]]; ls = state['latch'][key]
        state['max_door_deg'] = max(state['max_door_deg'], th / DEG)
        if key in held:
            if ls != 'open':
                state['latch'][key] = 'open'; state['armed'][key] = False; m.jnt_range[dd['jnt'], 1] = TH_MAX
                state['paddle_release'] = True; state['events'].append([round(float(d.time), 3), key, 'release'])
            continue
        # a released claw stays open until the striker has left it (door past the detent zone), then it can catch again
        if ls == 'open' and not state['armed'][key]:
            if th > TH_ARM: state['armed'][key] = True
            continue
        if ls == 'open' and c['two' + key[1]] and th < TH_S1:
            state['latch'][key] = ls = 'stage1'; m.jnt_range[dd['jnt'], 1] = TH_S1
            state['stage1_catch'] = True; state['events'].append([round(float(d.time), 3), key, 'stage1'])
        if ls in ('open', 'stage1') and th < TH_S2:
            state['latch'][key] = 'stage2'; m.jnt_range[dd['jnt'], 1] = TH_S2_LIM
            state['stage2_catch'] = True; state['events'].append([round(float(d.time), 3), key, 'stage2'])
    # success bookkeeping: goal predicate true and no robot contact with the goal objects
    g = GOALS.get(state['target'])
    if g is None: return
    ok = True; watch = []
    for s in g['doors']:
        key = g['cab'] + s; dd = k['door'][key]
        ok &= state['latch'][key] == 'stage2' and abs(float(d.qpos[dd['q']])) < .5 * DEG
        watch += dd['geoms']
    for s in g['boxes']:
        ok &= box_on_table(m, d, k, g['cab'] + s); watch += k['box'][g['cab'] + s]['geoms']
    if ok:
        g1, g2 = d.contact.geom1[:d.ncon], d.contact.geom2[:d.ncon]
        touch = np.any((np.isin(g1, k['robot']) & np.isin(g2, watch)) | (np.isin(g2, k['robot']) & np.isin(g1, watch)))
        ok = not touch
    state['hold'] = state['hold'] + m.opt.timestep if ok else 0.


def box_on_table(m, d, k, key):
    b = k['box'][key]['body']; x, y, z = d.xpos[b]
    if abs(z - (TABLE_Z + BOX[2])) > .004 or d.xmat[b, 8] < math.cos(15 * DEG): return False
    if np.linalg.norm(d.cvel[b]) > .05: return False
    n = key[0]; x0, y0, yaw = PLACE[n]
    lx = math.cos(yaw) * (x - x0) + math.sin(yaw) * (y - y0); ly = -math.sin(yaw) * (x - x0) + math.cos(yaw) * (y - y0)
    return not (-.02 < lx < D + .02 and abs(ly) < W / 2 + .02)   # outside the cabinet footprint


def success(model, data, state, target): return state['hold'] >= HOLD_S


def damage(model, data, state): return dict(door_slam=state['door_slam'], wrist_overload=state['wrist_overload'])


WORKSPACE = ((-.20, -.75, .82), (.45, .65, 1.40))   # tool-point reach box for this bench (also stated in public_text)
OBJECT_BOX = ((-.75, -.85, .70), (.75, .85, 1.60))
TOOLS = [dict(name='move_arm / move_both_arms / touch / wait (standard dual-arm tools)', orientation='gripper points straight down, yaw only',
              workspace=dict(x=[WORKSPACE[0][0], WORKSPACE[1][0]], y=[WORKSPACE[0][1], WORKSPACE[1][1]], z=[WORKSPACE[0][2], WORKSPACE[1][2]]),
              max_force_n=[5, 60])]
PUBLIC_WORDS = []
ROBOT_TEXT = ('Robot: two Panda arms with parallel grippers side by side, both facing +X (left base at x -0.40, y -0.42; '
              'right base at x -0.32, y 0.30); the left arm reaches the cabinet on the -Y side, the right arm the cabinet '
              'on the +Y side. Grippers point straight down and turn about vertical only. Tool workspace for this bench: '
              'X -0.20 to 0.45 m, Y -0.75 to 0.65 m, Z 0.82 to 1.40 m (the cabinets are 0.50 m tall). Joint torque limits '
              '80 N m on joints 1-4, 12 N m on joints 5-7; each finger squeezes with at most 40 N; the arm force guard can '
              'be set up to 60 N; a wrist force above 80 N for 0.1 s aborts the episode.')
CAB_TEXT = ('Each cabinet has two hinged doors with a round knob near the top of each door and a box with a front grab '
            'tab behind each door. Two identical paddle handles sit on the side face of the cabinet that faces the middle '
            'of the table, a front paddle and a rear paddle.')
# lead decision #144 (review r4 L5): skill guidance only (geometry, no hidden value): the A/N rear paddle is at the edge of
# the left arm's reach (reachable only within about 3 cm above its own height; a top-down approach over the cabinet stalls)
REACH_TEXT = ("The rear paddle of the -Y cabinet is near the left arm's reach; approach it at paddle height from the front.")


def public_text(target):
    if target in ('RR-T4', 'RR-T5'):
        scene = ('One grey double-door tool cabinet stands on the -Y side of the table, doors facing -X (toward the arms). '
                 + CAB_TEXT + ' ' + REACH_TEXT)
    else:
        scene = ('Two grey double-door tool cabinets stand on the table, doors facing -X (toward the arms): cabinet A on the '
                 '-Y side and cabinet B on the +Y side. ' + CAB_TEXT + ' ' + REACH_TEXT)
    sides = 'Left and right are as seen when facing the doors.'
    closed = ('A door counts as closed when it is fully closed, flush with the cabinet front, so that it stays shut when '
              'its knob is pulled, with no hand touching it.')
    care = ('Damage ends success: a door must not hit the cabinet frame faster than 0.25 m/s measured at its knob, and '
            'a wrist force above 80 N held for 0.1 s counts as damage too.')
    placed = ('A box counts as on the table when it rests upright on the table top outside the cabinet with no hand '
              'touching it.')
    hold = 'The final state must hold for 20 s after the hands let go.'
    goal = {None: 'Goal: explore the cabinets. The deployment goal is given after reset.',
            TASK_A: 'Goal: open the left door of cabinet A, put the box behind it on the table, then close the left door.',
            'RR-T1': 'Goal: open the right door of cabinet A and put the box behind it on the table.',
            'RR-T2': 'Goal: open the right door of cabinet B and put the box behind it on the table.',
            'RR-T3': 'Both doors of cabinet B stand open. Goal: close both doors of cabinet B.',
            'RR-T4': 'Goal: open the right door of the cabinet, put the box behind it on the table, then close every door that is open.',
            'RR-T5': 'Goal: open the right door of the cabinet, put the box behind it on the table, then close every door that is open.'}[target]
    tol = ' '.join(t for t, need in ((placed, target in (TASK_A, 'RR-T1', 'RR-T2', 'RR-T4', 'RR-T5')),
                                     (closed, target in (TASK_A, 'RR-T3', 'RR-T4', 'RR-T5')), (hold, target is not None),
                                     (care, target is not None)) if need)
    return ' '.join(x for x in (scene, sides, ROBOT_TEXT, goal, tol) if x)


def wrist_wrench(model, data):
    out = []
    for s in ('left', 'right'):
        R = data.site(s + '_wrist').xmat.reshape(3, 3)
        out += [R @ data.sensor(s + '_force').data, R @ data.sensor(s + '_torque').data]
    return np.concatenate(out)


def observation(model, data, state):
    k = state.get('ids') or state.setdefault('ids', ids(model))
    q, v = k['q'][0] + k['q'][1], k['v'][0] + k['v'][1]
    return dict(joint_pos=data.qpos[q].copy(), joint_vel=data.qvel[v].copy(), jaw_opening=data.qpos[k['fingers']].copy(),
                wrist_wrench=wrist_wrench(model, data),
                finger_force=np.array([data.sensor(f'{s}_finger_{f}_normal').data[0] for s in ('left', 'right') for f in 'ab']),
                time=float(data.time))


# ------------------------------------------------------------------ privileged scripted controller
def rot(yaw): return Rotation.from_euler('z', yaw).as_matrix() @ DOWN


def ik(m, scratch, k, arm, target, yaw, q0, iters=15, nom=None):
    q, v, tip = k['q'][arm], k['v'][arm], k['tips'][arm]
    scratch.qpos[q] = q0
    R = rot(yaw)
    jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv)); lo, hi = m.jnt_range[[m.dof_jntid[i] for i in v]].T
    for _ in range(iters):
        mujoco.mj_kinematics(m, scratch); mujoco.mj_comPos(m, scratch)
        err = np.r_[target - scratch.site_xpos[tip], .5 * Rotation.from_matrix(R @ scratch.site_xmat[tip].reshape(3, 3).T).as_rotvec()]
        mujoco.mj_jacSite(m, scratch, jp, jr, tip)
        J = np.vstack([jp[:, v], .5 * jr[:, v]])
        Jp = J.T @ np.linalg.inv(J @ J.T + 1e-4 * np.eye(6))
        # null-space pull toward HOME keeps the elbow up and back (forearm clear of the closed neighbour door)
        dq = Jp @ err + (np.eye(7) - Jp @ J) @ (NULL_GAIN * ((HOME if nom is None else nom) - scratch.qpos[q]))
        scratch.qpos[q] = np.clip(scratch.qpos[q] + np.clip(dq, -.08, .08), lo + .01, hi - .01)
    return scratch.qpos[q].copy()


NULL_GAIN = .05


def tip_yaw(d, k, arm):
    R = d.site_xmat[k['tips'][arm]].reshape(3, 3) @ DOWN.T
    return math.atan2(R[1, 0], R[0, 0])


class Driver:
    """Scripted Cartesian plan tracked by joint PD. Records ctrl rows. Plans may read privileged state."""

    def __init__(self, seed, target, params):
        self.m, self.d = build(seed, target, params); self.st = init_state(seed, target, params); self.k = ids(self.m)
        self.scratch = mujoco.MjData(self.m); self.acts = []
        self.pos = [self.d.site_xpos[t].copy() for t in self.k['tips']]
        self.yaw = [tip_yaw(self.d, self.k, a) for a in (0, 1)]
        self.jaw = [.045, .045]; self.qt = [self.d.qpos[self.k['q'][a]].copy() for a in (0, 1)]; self.qs = [q.copy() for q in self.qt]
        self.n = 0; self.logd = {}; self.nom = [None, None]; self.mindist = (9.,)

    def tick(self, arm, pos, yaw, jaw):
        self.pos[arm], self.yaw[arm], self.jaw[arm] = np.asarray(pos, float), yaw, jaw
        if self.n % 10 == 0:
            for a in (0, 1): self.qt[a] = ik(self.m, self.scratch, self.k, a, self.pos[a], self.yaw[a], self.qt[a], nom=self.nom[a])
        u = np.zeros(self.m.nu)
        for a in (0, 1):
            # smoothed joint target (IK runs every 10 ticks; a raw step in the target gave single-tick wrist-joint spikes)
            self.qs[a] = self.qs[a] + .2 * (self.qt[a] - self.qs[a])
            ua = np.clip(KP * (self.qs[a] - self.d.qpos[self.k['q'][a]]) - KD * self.d.qvel[self.k['v'][a]], -LIM, LIM)
            # real Panda: joints 5-7 deliver at most 12 N m in total, including the gravity/bias term step() adds
            bias = self.d.qfrc_bias[self.k['v'][a][4:]]
            ua[4:] = np.clip(ua[4:], -12. - bias, 12. - bias)
            u[7 * a:7 * a + 7] = ua
        u[14:16] = self.jaw[0]; u[16:18] = self.jaw[1]
        self.d.ctrl[:] = u; self.acts.append(u.copy()); step(self.m, self.d, self.st); self.n += 1
        if self.debug and self.n % 5 == 0: self._log()

    debug, phase = False, ''

    def _log(self):
        m, d, rob = self.m, self.d, set(self.k['robot']); buf = np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            if (c.geom1 in rob) == (c.geom2 in rob): continue
            mujoco.mj_contactForce(m, d, i, buf)
            if c.dist < self.mindist[0]: self.mindist = (round(float(c.dist), 4), self.phase, m.geom(c.geom1).name, m.geom(c.geom2).name, round(float(d.time), 2))
            if buf[0] > 25:
                key = (self.phase, m.geom(c.geom1).name, m.geom(c.geom2).name)
                self.logd[key] = max(self.logd.get(key, (0, 0))[0], round(float(buf[0]), 1)), round(float(d.time), 2)

    def move(self, arm, goal=None, yaw=None, jaw=None, n=300, until=None):
        a0, y0, j0 = self.pos[arm].copy(), self.yaw[arm], self.jaw[arm]
        goal = a0 if goal is None else np.asarray(goal, float); yaw = y0 if yaw is None else yaw; jaw = j0 if jaw is None else jaw
        dy = (yaw - y0 + math.pi) % (2 * math.pi) - math.pi
        for i in range(n):
            f = min(1., (i + 1) / max(1, n - 50)); f = f * f * (3 - 2 * f)
            # commanded yaw wrapped into +-180 deg (the agent tool rejects |yaw| > 180; lead decision #164)
            self.tick(arm, a0 + (goal - a0) * f, wrap(y0 + dy * f), j0 + (jaw - j0) * f)
            if until is not None and until(self): return True
        return False

    def wait(self, n):
        for _ in range(n): self.tick(0, self.pos[0], self.yaw[0], self.jaw[0])

    def door(self, key): return float(self.d.qpos[self.k['door'][key]['q']])

    def result(self): return np.array(self.acts)


def door_to_local(side, theta, p):
    # door frame (origin on the hinge axis) -> cabinet local frame at door angle theta
    sgn = 1 if side == 'L' else -1
    a = -sgn * theta  # rotation about +z
    Rz = np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])
    return np.array([-DT / 2, sgn * (W / 2 - .002), 0.]) + Rz @ np.asarray(p, float)


def cab_frame(name):
    x0, y0, yaw = PLACE[name]; c, s = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])
    return np.array([x0, y0, TABLE_Z]), R, yaw


def to_world(name, p):
    o, R, _ = cab_frame(name); return o + R @ np.asarray(p, float)


def knob_point(name, side, theta, out=.022):
    # Grasp point on the knob shaft `out` m in front of the door face, at door angle theta (world).
    sgn = 1 if side == 'L' else -1
    return to_world(name, door_to_local(side, theta, [-DT / 2 - out, -sgn * RK, ZK]))


def door_yaw(name, side, theta):
    # Gripper yaw putting the jaw axis along the door width direction (pads straddle the knob shaft).
    sgn = 1 if side == 'L' else -1
    return cab_frame(name)[2] + math.pi / 2 - sgn * theta


def paddle_point(name, i, travel=0.):
    return to_world(name, [PX[i - 1], PSIDE[name] * (W / 2 + .030 + travel), PZ])


def wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi


def near_yaw(target, current):
    # the gripper is symmetric: pick target or target + pi, whichever is closer to the current yaw
    a = current + wrap(target - current); b = current + wrap(target + math.pi - current)
    return a if abs(a - current) <= abs(b - current) else b


def L(name, x, y, z): return to_world(name, [x, y, z])


SPEED = .12   # m/s for free-space transit


def goto(dr, arm, pts, yaw=None, jaw=None, speed=SPEED, settle=60):
    # straight segments through world waypoints; duration from distance (smoothstep per segment)
    for i, p in enumerate(pts):
        p = np.asarray(p, float); dist = np.linalg.norm(p - dr.pos[arm])
        n = int(max(80, dist / speed / .002)) + (settle if i == len(pts) - 1 else 0)
        dr.move(arm, p, yaw if i == 0 else None, jaw, n)


SAFE_Z = TABLE_Z + H + .065     # transit height: fingertips clear the cabinet top and the door tops


def transit(dr, arm, target, yaw=None, jaw=None, approach=.08, up_speed=SPEED):
    # up to the safe height, across, down to `approach` above the target, then slowly onto it
    target = np.asarray(target, float); cur = dr.pos[arm]
    goto(dr, arm, [np.r_[cur[:2], max(SAFE_Z, cur[2])]], None, jaw, speed=up_speed)   # straight up first, no wrist turn near objects
    goto(dr, arm, [np.r_[target[:2], max(SAFE_Z, target[2] + approach)], target + [0, 0, approach]], yaw, jaw)
    goto(dr, arm, [target], None, jaw, speed=.05)


def pull_paddle(dr, name, i, travel=None, hold=150):
    arm = ARM_OF[name]; y = near_yaw(cab_frame(name)[2], dr.yaw[arm])     # jaw axis along local x
    out = cab_frame(name)[1] @ np.array([0, PSIDE[name], 0.])
    tab = paddle_point(name, i); pre = None
    if name in ('A', 'N'):
        # A/N's paddle side lies toward the middle of the table, far from the left base. Over the cabinet top at SAFE_Z
        # the arm is nearly stretched (stock IK: joint 4 at ~1.6x its speed limit; reference controller: sags 8 cm), so
        # the hand comes at paddle height from in front of the cabinet, 0.10 m out from the side face (clear of both
        # paddle tabs), and slides sideways onto the tab (the open jaw straddles it along x)
        cur = dr.pos[arm]; yo = PSIDE[name] * (W / 2 + .10)
        pre = L(name, -.10, yo, PZ)
        goto(dr, arm, [np.r_[cur[:2], max(cur[2], PZ)], pre], y, .025)
        goto(dr, arm, [L(name, PX[i - 1], yo, PZ)], None, .025, speed=.05)    # slow: the arm is stretched here
        goto(dr, arm, [tab], None, .025, speed=.05)
    else:
        transit(dr, arm, tab, y, .025)
    dr.move(arm, None, None, 0., 200)
    if travel is None:
        # privileged closed loop: pull until the paddle is at its stop (tracking lag grows with the paddle spring)
        qa = dr.k['paddle'][f'{name}{i}']
        dr.move(arm, tab + out * .035, None, 0., 900, until=lambda r: r.d.qpos[qa] >= P_TRAVEL - .0005)
    else:
        dr.move(arm, tab + out * travel, None, 0., 400)
    dr.move(arm, None, None, 0., hold)
    dr.move(arm, tab, None, 0., 300)
    dr.move(arm, None, None, .025, 150)
    if pre is None:
        goto(dr, arm, [tab + out * .03, tab + out * .03 + [0, 0, .09]], None, .025, speed=.06)
    else:   # A/N: leave the way the hand came (above the paddle at SAFE_Z the arm cannot reach)
        goto(dr, arm, [L(name, PX[i - 1], yo, PZ), pre], None, .025, speed=.05)


def pick_knob_yaw(dr, name, side, th0, th1):
    # The jaw is symmetric, so the knob can be gripped at door_yaw or door_yaw + pi. Pick the one whose wrist (j7)
    # rotation over the planned swing th0 -> th1 stays furthest from the joint limits (privileged IK look-ahead).
    arm = ARM_OF[name]; m, k = dr.m, dr.k; sc = mujoco.MjData(m); sc.qpos[:] = dr.d.qpos
    lo, hi = m.jnt_range[m.joint(('left', 'right')[arm] + '_joint7').id]
    best = None
    for yo in (0., math.pi):
        y0 = dr.yaw[arm] + wrap(door_yaw(name, side, th0) + yo - dr.yaw[arm]); off = y0 - door_yaw(name, side, th0)
        q = dr.qt[arm].copy(); margin = 9.
        for th in np.linspace(th0, th1, 13):
            for _ in range(4): q = ik(m, sc, k, arm, knob_point(name, side, th), door_yaw(name, side, th) + off, q)
            margin = min(margin, q[6] - lo, hi - q[6])
        if best is None or margin > best[0]: best = (margin, y0)
    return best[1]


def grasp_knob(dr, name, side, th_end=None):
    arm = ARM_OF[name]; th = dr.door(name + side)
    p = knob_point(name, side, th)
    y = pick_knob_yaw(dr, name, side, th, th if th_end is None else th_end)
    transit(dr, arm, p, y, .03)
    dr.move(arm, None, None, 0., 250)
    return y - door_yaw(name, side, th)   # yaw offset to keep while swinging


def swing(dr, name, side, th1, yoff, rate=.35):
    arm = ARM_OF[name]; th0 = dr.door(name + side)
    n = max(50, int(abs(th1 - th0) / rate / .002))
    for i in range(n):
        f = (i + 1) / n; f = f * f * (3 - 2 * f); th = th0 + (th1 - th0) * f
        dr.tick(arm, knob_point(name, side, th), wrap(door_yaw(name, side, th) + yoff), 0.)
    dr.move(arm, None, None, 0., 100)


def release_knob(dr, name, side, back=.06):
    arm = ARM_OF[name]; th = dr.door(name + side); sgn = 1 if side == 'L' else -1
    dr.move(arm, None, None, .03, 150)
    n_out = cab_frame(name)[1] @ np.array([-math.cos(th), sgn * math.sin(th), 0.])
    p = dr.pos[arm] + n_out * back
    goto(dr, arm, [p, p + [0, 0, .06]], None, .045, speed=.08)


PUSH_END = -.06 / RK   # commanded knob angle at the end of a closing push (6 cm past closed at the knob)


def push_arc(dr, name, side, yoff, ticks=2000, hold=0, until=None):
    # Knob gripped: drive the commanded knob point along the door's own arc from the current angle to PUSH_END (past
    # closed, so the arm compliance pushes the door), wrist yaw following the door. Returns True if `until` fired.
    arm = ARM_OF[name]; th0 = dr.door(name + side)
    for i in range(ticks + hold):
        th = th0 + (PUSH_END - th0) * min(1., (i + 1) / ticks)
        dr.tick(arm, knob_point(name, side, th), wrap(door_yaw(name, side, max(th, 0.)) + yoff), 0.)
        if until is not None and until(dr): return True
    return False


def close_door(dr, name, side, yoff, max_ticks=4000):
    # Knob still gripped: swing to 12 deg, then push along the arc until the claw engages, then let go and back off.
    arm = ARM_OF[name]; key = name + side
    swing(dr, name, side, 12 * DEG, yoff, rate=.3)
    push_arc(dr, name, side, yoff, ticks=2000, hold=max_ticks - 2000, until=lambda r: r.st['latch'][key] == 'stage2')
    # latched: hold the hand where it is, open the jaw, then back off (pulling on the knob would load the latched door)
    dr.move(arm, dr.d.site_xpos[dr.k['tips'][arm]].copy(), None, 0., 100)
    release_knob(dr, name, side)


def open_door(dr, name, side, th_open=TH_OPEN):
    c = dr.st['cabs'][name]; mp = MAPS[c['map']]
    singles = [h for h in (1, 2) if mp[h] == side]
    h = singles[0] if singles else next(h for h in (1, 2) if side in mp[h])
    pull_paddle(dr, name, h)
    dr.wait(400)
    yoff = grasp_knob(dr, name, side, th_open)
    swing(dr, name, side, th_open, yoff)
    return yoff


# Wrist-posture check before reaching into a compartment (polish 2026-10-29, lead decision #115). A top-down Panda hand
# leaves one free arm posture: the wrist centre (link6 origin) circles the tool axis. A controller without a posture
# term (the agent's stock IK, dual_arm_demo solve_ik) lets it drift with the motion history; when it points inboard by
# about 30 deg the forearm (link5) hits the closed neighbouring door 30-56 mm before the tab. The check reads only joint
# angles (forward kinematics); an agent sees the same in its joint readings and cameras.
# wrist-centre direction window (deg, + = outboard) + aim. R boxes: inboard family -25..-47 deg fails, so lo = -8 (the
# reference controller sits at -0.5). L box (task A only): the reference sits at -26 and the agent emulation at -9..-14,
# both succeed, and no failing L posture was seen, so its window is wide.
POSTURE_WIN = {'R': (-8., 45., 25.), 'L': (-40., 45., 25.)}
# Cabinet A/N (left arm, layout of #136). The posture is checked twice: first at a check point 0.20 m above the cabinet
# floor in front of the compartment (world z 1.00, above the height where the stock IK twists), then at the low point.
# Measured (stock-IK emulation, all dev runs): at the check point the R-box posture is about +32 deg (on the way down
# the IK drops into the twisted branch, j1 > 1.5 rad, the forearm hits the OPEN right door, and under a joint-rate limit
# the arm cannot leave that branch later) or about +11 deg (descends to about -6 deg, forearm grazing the closed left
# door). Winding into +2..+9 at the check point gives +2..+7 at the low point: untwisted, clear of both doors.
# L box (task A): -25..-28 deg at the check point, -27..-44 at the low point; no failure seen.
POSTURE_WIN_AN = {'R': (-45., 22., 0.), 'L': (-55., 45., 25.)}
POSTURE_WIN_AN_HIGH = {'R': (-45., 8., 4.), 'L': (-55., 45., 25.)}
Z_CHECK = .20                      # check point height above the cabinet floor top (cabinet frame z, world z 1.00)
POSTURE_P = (-.22, .06, .595)      # where the posture is wound (cabinet frame; y toward the open door): in front of the
                                   # cabinet near the top of the declared workspace (world z 1.395 <= 1.40)
# Cabinet B (right arm, T2; lead decision #144, review r4 L6): the wind at POSTURE_P swung the right elbow (link4) into B's
# open right door (3.1 kN one-step impact, ~300 N for 1.2 s, tool pushed to z 1.44). B winds 0.30 m in front of the
# cabinet, 0.12 m toward the CLOSED door (negative = away from the open door), at world z 1.385; with the door opened to
# 125 deg (TH_OPEN_BOX) the wound posture's forearm also clears the open door while it reaches the tab.
POSTURE_P_B = (-.30, -.12, .585)


def wrist_dir_err(dr, arm, name, side):
    # horizontal direction of the wrist centre seen from the tool point, relative to straight back (cabinet -X), in
    # degrees, positive = toward the compartment's outer side wall
    pre = ('left', 'right')[arm]
    v = cab_frame(name)[1].T @ (dr.d.xpos[dr.m.body(f'{pre}_link6').id] - dr.d.site_xpos[dr.k['tips'][arm]])
    return math.degrees(wrap(math.atan2(v[1], v[0]) - math.pi)) * (-1 if side == 'L' else 1)


def wind_shift(dr, arm, p, s, extra):
    # In free space at p: turn the yaw (direction s) until joint 7 reaches its limit (a yaw increase drives joint 7
    # down), keep turning by `extra` rad (joint 7 can no longer take it, so the arm swings the wrist centre round the
    # tool), then turn back to the working yaw at the agent's 0.06 rad per control tick. Net effect: the wrist centre
    # turns by about 0.85 x extra, s > 0 = toward the outer wall.
    # As agent commands (lead decision #164): every yaw goal is wrapped into +-180 deg, and the agent tool turns toward a
    # yaw goal the shorter way, so each command must lie less than 180 deg from the previous one in the intended
    # direction. The turn out is one short command per control tick (0.06 rad, reading joint 7 after each), the extra
    # turn one command (<= 1.2 rad), and the turn back (often more than 180 deg) WIND_LEGS >= 3 commands of at most
    # 120 deg each; the per-tick path is the same as one uniform stream. The goals are logged in WIND_CMDS.
    m = dr.m; j7 = m.joint(('left', 'right')[arm] + '_joint7'); lo, hi = m.jnt_range[j7.id]; qa = m.jnt_qposadr[j7.id]
    y0 = dr.yaw[arm]; y = y0; cmds = []
    for _ in range(140):
        q7 = float(dr.d.qpos[qa])
        if (s > 0 and q7 <= lo + .03) or (s < 0 and q7 >= hi - .03): break
        y += s * .06; cmds.append(('out', y))
        for _ in range(25): dr.tick(arm, p, wrap(y), dr.jaw[arm])
    n = max(1, int(math.ceil(extra / .06)))
    for _ in range(n):
        y += s * extra / n
        for _ in range(25): dr.tick(arm, p, wrap(y), dr.jaw[arm])
    cmds.append(('extra', y))
    n = int(math.ceil(abs(y - y0) / .06)); y1 = y
    k = max(WIND_LEGS, int(math.ceil(abs(y1 - y0) / (2 * math.pi / 3))))
    ends = {int(round(n * j / k)) for j in range(1, k + 1)}
    for i in range(n):
        y = y1 + (y0 - y1) * (i + 1) / n
        if i + 1 in ends: cmds.append(('back', y))
        for _ in range(25): dr.tick(arm, p, wrap(y), dr.jaw[arm])
    for _ in range(100): dr.tick(arm, p, wrap(y0), dr.jaw[arm])
    WIND_CMDS.append(dict(arm=arm, s=s, start_deg=round(math.degrees(wrap(y0)), 2),
                          cmds=[(ph, round(math.degrees(wrap(c)), 2), round(math.degrees(c - y0), 2)) for ph, c in cmds]))


WIND_LEGS = 3        # minimum number of commands for the turn back
WIND_CMDS = []       # diagnostics only: (phase, commanded yaw in deg wrapped to +-180, unwrapped turn from the start) per wind


POSTURE_LOG = []     # diagnostics only: wrist-direction errors seen by posture_fix


def posture_fix(dr, name, side, front, yaw, tries=4, win=None):
    # At the low point in front of the compartment: if the wrist centre points too far inboard (or outboard), go back up
    # to free space, wind the posture over, come back down and check again. Never fires under the reference controller
    # (its IK pulls toward HOME in the null space); fires under the agent's stock IK when the history left a bad posture.
    arm = ARM_OF[name]; bl = BOX_LOCAL[side]; lo, hi, aim = (win or (POSTURE_WIN_AN if name in ('A', 'N') else POSTURE_WIN))[side]
    for _ in range(tries):
        e = wrist_dir_err(dr, arm, name, side); POSTURE_LOG.append(round(e, 1))
        if lo <= e <= hi: return
        PP = POSTURE_P_B if name == 'B' else POSTURE_P
        p = L(name, PP[0], PP[1] * (1 if side == 'L' else -1), PP[2])
        # slow rise (leaving the low point stays below the joint-speed limits); no approach overshoot above p (z <= 1.40)
        transit(dr, arm, p, None, dr.jaw[arm], approach=0., up_speed=.05)
        # measured: the low-point posture has two stable families, about -25..-47 deg (forearm hits the neighbouring door)
        # and +20..+41 deg (clear); a wind of about 1.1 rad moves it from the first to the second
        wind_shift(dr, arm, p, 1 if e < lo else -1, min(1.2, math.radians(abs(aim - e)) / .85))
        transit(dr, arm, front, yaw, .022)


def take_box(dr, name, side):
    # Door open to TH_OPEN_BOX: pinch the box's front tab top-down (jaw axis along local y), lift 8 mm, slide the box out
    # through the door opening, carry it clear of the door's sweep, set it on the table.
    arm = ARM_OF[name]; bl = BOX_LOCAL[side]; sgn = 1 if side == 'L' else -1
    # jaw axis along local y; of the two equivalent yaws this one keeps the forearm (link5/6) clear of both doors
    y = dr.yaw[arm] + wrap(cab_frame(name)[2] + math.pi / 2 - dr.yaw[arm])
    tab = L(name, .015, bl[1], bl[2])
    if name in ('A', 'N'):
        # posture check above the twist height first (see POSTURE_WIN_AN_HIGH), then straight down to the low point
        transit(dr, arm, L(name, -.09, bl[1], Z_CHECK), y, .022)
        posture_fix(dr, name, side, L(name, -.09, bl[1], Z_CHECK), y, win=POSTURE_WIN_AN_HIGH)
        goto(dr, arm, [L(name, -.09, bl[1], bl[2])], None, .022, speed=.05)
    else:
        transit(dr, arm, L(name, -.09, bl[1], bl[2]), y, .022)     # descend in front of the cabinet
    posture_fix(dr, name, side, L(name, -.09, bl[1], bl[2]), y)
    goto(dr, arm, [tab], None, .022, speed=.04)                    # slide in: pads pass either side of the tab
    dr.move(arm, None, None, 0., 250)
    goto(dr, arm, [tab + [0, 0, .008]], None, 0., speed=.03)
    # set-down point: centre front, 0.20 m in front of the doors, all cabinets. The box ends 14-17 mm outside both doors'
    # sweeps (at 0.17 m: 0.2-0.6 mm on B, measured; a door closed after the set-down, T4/T5, hit a box set down a few mm
    # off, layout trial #136). Review r4 O1 (B back to 0.17 m for T2's 1.11x joint speed) checked under the new B wind
    # (lead decision #144): T2 peak joint speed is the same at 0.17 / 0.185 / 0.20 m (0.67-1.02x), so B keeps 0.20 m.
    goto(dr, arm, [L(name, -.12, bl[1], bl[2] + .008), L(name, -.20, 0., bl[2] + .008)], None, 0., speed=.05)
    goto(dr, arm, [L(name, -.20, 0., bl[2] - WALL + .002)], None, 0., speed=.02)
    dr.move(arm, None, None, .022, 200)
    goto(dr, arm, [dr.pos[arm] + [0, 0, .10]], None, .022)


def reference(seed, target, params):
    p = params or parameters(seed)
    dr = Driver(seed, target, p)
    g = GOALS[target]; name = g['cab']; arm = ARM_OF[name]
    if target == 'RR-T3':
        for side in ('L', 'R'):
            yoff = grasp_knob(dr, name, side, 0.); close_door(dr, name, side, yoff)
    else:
        side = g['boxes'][0]
        open_door(dr, name, side, TH_OPEN_BOX[name])
        release_knob(dr, name, side)
        take_box(dr, name, side)
        for s in g['doors']:
            if dr.st['latch'][name + s] != 'stage2':
                yo = grasp_knob(dr, name, s, 0.); close_door(dr, name, s, yo)
    goto(dr, arm, [dr.pos[arm] + [0, 0, .03]], None, .045)
    for _ in range(int(HOLD_S / dr.m.opt.timestep) + 200): dr.tick(arm, dr.pos[arm], dr.yaw[arm], .045)
    return dr.result()


def render(model, data, path):
    r = mujoco.Renderer(model, 720, 1280); r.update_scene(data, camera='overview'); img = r.render(); r.close()
    chunk = lambda t, b: struct.pack('>I', len(b)) + t + b + struct.pack('>I', zlib.crc32(t + b) & 0xffffffff)
    raw = b''.join(b'\x00' + row.tobytes() for row in img)
    Path(path).write_bytes(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', 1280, 720, 8, 2, 0, 0, 0))
                           + chunk(b'IDAT', zlib.compress(raw, 6)) + chunk(b'IEND', b''))


# ------------------------------------------------------------------ probes (scripted, the same plan for both contrast values)
_MARKS = {}   # probe name -> tick index where the measured segment starts (same for both contrast values)


def _open_for_probe(dr, name, side):
    c = dr.st['cabs'][name]; mp = MAPS[c['map']]
    singles = [h for h in (1, 2) if mp[h] == side]
    h = singles[0] if singles else next(h for h in (1, 2) if side in mp[h])
    pull_paddle(dr, name, h)
    dr.wait(400)


def probe_close(name, side, ramp=3000, hold=300):
    # Release the door with its own paddle, grip the knob at the popped angle, push the door shut along its arc with
    # a fixed slow ramp and stop pushing the moment the door is fully latched (a person pushes until it latches; the
    # stop uses the latch state), hold, let go. The same plan for every contrast value.
    def run(seed, params):
        dr = Driver(seed, None, params); arm = ARM_OF[name]; key = name + side
        _open_for_probe(dr, name, side)
        yoff = grasp_knob(dr, name, side, 0.)
        _MARKS[f'close_{name}{side}'] = len(dr.acts)
        push_arc(dr, name, side, yoff, ticks=ramp, hold=1000, until=lambda r: r.st['latch'][key] == 'stage2')
        dr.move(arm, dr.d.site_xpos[dr.k['tips'][arm]].copy(), None, 0., hold)
        release_knob(dr, name, side)
        return dr.result()
    return run


def closing_force(tr, name, side):
    # Equivalent closing force at the knob from the working wrist: door torque applied by the hand about the hinge
    # axis = a.(r x f) + a.c, with f = -dF (tared wrist force) and the couple a.c = -a.dM (the wrist site sits on the
    # vertical through the knob, so the force's own moment about it has no component along the vertical hinge axis).
    arm = ARM_OF[name]; sgn = 1 if side == 'L' else -1; i0 = _MARKS[f'close_{name}{side}']
    o, R, _ = cab_frame(name)
    a = R @ np.array([0, 0, -sgn])
    r = knob_point(name, side, 0.) - to_world(name, [-DT / 2, sgn * (W / 2 - .002), ZK])
    w = tr['wrench'][:, 6 * arm:6 * arm + 6]
    if len(w) <= i0 + 10: return np.zeros(1)
    dF, dM = w[i0:, :3] - w[i0, :3], w[i0:, 3:] - w[i0, 3:]
    tau_open = np.cross(r, -dF) @ a - dM @ a
    return tau_open / RK   # the sensor reads the load on the hand: sign checked against the applied hinge torque


def closing_peak(name, side):
    def stat(tr): return float(closing_force(tr, name, side).max())
    return stat


def detent_drop(name, side):
    # First-stage signature: before the final (gasket) peak, the closing force rises over the claw detent and then
    # drops again. Statistic = the largest drop below a running maximum before the global peak (0 for a single-stage door).
    def stat(tr):
        f = closing_force(tr, name, side)
        if len(f) < 20: return 0.
        f = np.convolve(f, np.ones(10) / 10, mode='same')[:int(np.argmax(f)) + 1]
        return float((np.maximum.accumulate(f) - f).max())
    return stat


def probe_pull_light(name, i, travel=.008, hold=700):
    # Grip paddle i and pull it a little (commanded 8 mm, below the 9 mm release travel), hold, let go.
    def run(seed, params):
        dr = Driver(seed, None, params); arm = ARM_OF[name]
        y = near_yaw(cab_frame(name)[2], dr.yaw[arm]); out = cab_frame(name)[1] @ np.array([0, PSIDE[name], 0.])
        tab = paddle_point(name, i)
        transit(dr, arm, tab, y, .025)
        dr.move(arm, None, None, 0., 200)
        _MARKS[f'pull_{name}{i}'] = len(dr.acts)
        dr.move(arm, tab + out * travel, None, 0., 400)
        dr.move(arm, None, None, 0., hold)
        dr.move(arm, tab, None, 0., 300)
        return dr.result()
    return run


def pull_plateau(name, i, hold=700):
    def stat(tr):
        arm = ARM_OF[name]; out = cab_frame(name)[1] @ np.array([0, PSIDE[name], 0.]); i0 = _MARKS[f'pull_{name}{i}']
        w = tr['wrench'][:, 6 * arm:6 * arm + 3]
        if len(w) <= i0 + 400 + hold: return 0.
        f = (w[i0:] - w[i0]) @ out
        return float(abs(f[400 + hold - 400:400 + hold].mean()))
    return stat


PROBES = {f'close_{n}{s}': probe_close(n, s) for n in ('A', 'B') for s in 'LR'}
PROBES['pull_B1_light'] = probe_pull_light('B', 1)
HIDDEN_CELLS = [
    dict(name='map_A', type='M', object='cab_A', contrast=('L/R', 'R/L'), probe=None, channel='visual',
         note='which door pops when a paddle is pulled is seen, not felt; pull force is the same for both maps'),
    dict(name='map_B', type='M', object='cab_B', contrast=('LR/R', 'R/LR'), probe='pull_B1_light', channel='both',
         statistic=pull_plateau('B', 1), note='a splitter paddle (2 cables) pulls about 2x harder; plus which doors pop'),
    dict(name='f2_AL', type='P', object='A_door_L', contrast=(18., 40.), probe='close_AL', channel='force', statistic=closing_peak('A', 'L')),
    dict(name='f2_AR', type='P', object='A_door_R', contrast=(18., 40.), probe='close_AR', channel='force', statistic=closing_peak('A', 'R')),
    dict(name='f2_BL', type='P', object='B_door_L', contrast=(18., 40.), probe='close_BL', channel='force', statistic=closing_peak('B', 'L')),
    dict(name='f2_BR', type='P', object='B_door_R', contrast=(18., 40.), probe='close_BR', channel='force', statistic=closing_peak('B', 'R')),
    *[dict(name=f'two_{n}{d}', type='M', object=f'{n}_door_{d}', contrast=(True, False), probe=f'close_{n}{d}', channel='force',
           statistic=detent_drop(n, d), note='two-stage: a first force peak (safety stage) then a drop before the gasket peak')
      for n in ('A', 'B') for d in 'LR'],
]


# ------------------------------------------------------------------ rule tests (exploration scene, cabinet A)
def _rt_pull_full(seed):
    p = parameters(seed); dr = Driver(seed, None, p); pull_paddle(dr, 'A', 1); dr.wait(300); return dr.result(), p


def _rt_pull_light(seed):
    p = parameters(seed); return probe_pull_light('A', 1, travel=.004)(seed, p), p


def _rt_close(seed, mode):
    # mode: 'full' = push until stage 2; 'stage1' = push until the first stage catches, then let go; 'short' = swing the
    # popped door only to 9 deg (before the first stage), then let go.
    p = dict(parameters(seed), two_AL=True)      # the stage tests need a two-stage door
    dr = Driver(seed, None, p); name, side = 'A', 'L'; key = name + side
    _open_for_probe(dr, name, side)
    yoff = grasp_knob(dr, name, side, 0.)
    if mode == 'full':
        close_door(dr, name, side, yoff)
    elif mode == 'short':
        swing(dr, name, side, 9 * DEG, yoff, rate=.3); release_knob(dr, name, side)
    else:
        arm = ARM_OF[name]
        push_arc(dr, name, side, yoff, ticks=2000, hold=2000, until=lambda r: r.st['latch'][key] != 'open')
        dr.move(arm, dr.d.site_xpos[dr.k['tips'][arm]].copy(), None, 0., 100)
        release_knob(dr, name, side)
    dr.wait(500)
    return dr.result(), p


RULE_TESTS = [
    dict(name='paddle_release', event_key='paddle_release', positive=_rt_pull_full, negative=_rt_pull_light),
    dict(name='stage1_catch', event_key='stage1_catch', positive=lambda s: _rt_close(s, 'full'), negative=lambda s: _rt_close(s, 'short')),
    dict(name='stage2_catch', event_key='stage2_catch', positive=lambda s: _rt_close(s, 'full'), negative=lambda s: _rt_close(s, 'stage1')),
]
