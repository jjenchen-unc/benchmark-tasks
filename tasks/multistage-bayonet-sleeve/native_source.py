# Card: cards/B7_couplings.md (B7 聯軸器套筒, revised 2026-10-28). Deviations: ../DEVIATIONS.md. Assets: ../ASSETS.md.
# One sentence: the robot takes bayonet (J-slot) sleeves off fixed shaft posts by following each coupling's hidden
# push / turn / pull release path; the path (M) decides which way to turn so the pins do not jam. The path cells are listed
# only on task A: since lead decision #171 no remove_* target lists a cell (skill / control targets; informed beat the best
# uninformed probing rule by < 10 %), and the first-junction dead end + false-seat geometry is kept (DEVIATIONS 21/22).
# Robot: the repo's two-Panda B4 robot (portable_demo.env.scene(seed, 'test'), non-robot bodies stripped, as in
# examples/toy_drawer.py). The left arm does all the work; the right arm is folded back at RIGHT_PARK. Stock Franka hand.
# Actions are data.ctrl rows (nu = 18): 14 arm joint torques (residual; step() adds the arm bias/gravity term) and
# 4 finger position targets (0 closed .. .045 open per finger).
# Mechanism (native, no rules): each shaft post is fixed to its stand and carries two opposed radial pins. The track
# (axial and circumferential channels plus a blind seating pocket) is cut into the sleeve BORE, so the post itself hides
# it (card: 'track moved into the bore'). A spring plunger on the post top pushes the sleeve outward into the pocket.
# Wrong moves jam pin against track wall (force/torque rises, no motion); right moves are free. Payoff trial
# 2026-10-29 (DEVIATIONS 21): at the first junction a wrong-way turn is free into a dead end (90 deg behind a 30 deg
# turn, 60 deg behind a 60 deg turn) that ends in a blind false-seat notch (behind 90 deg with a short blind ledge),
# so a wrong first guess costs a full excursion and a deliberate back-out instead of a short jam.
# The shafts are vertical (deviation D1): push = press the sleeve down, turn = wrist yaw, pull = lift.
# Hidden cells: path_A, path_B, path_C (M), the release path of each practice coupling. Each practice coupling has its
# own handedness bit (8 configurations; polish 2026-10-29, DEVIATIONS 2): E follows B, F follows C, and D = C's first
# stage followed by a final turn in A's direction (T-combo of path_C and path_A).
# The finger servos follow their target through a first-order filter (FINGER_TAU, polish 2026-10-29, DEVIATIONS 17) so
# that a jumped jaw command closes the fingers at a realistic speed instead of slamming the free sleeve on its post.
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

BENCH_ID = 'B7_couplings'
SEEDS = (42, 43, 44)
# Repair round 3 (2026-11-09): the declared robot text (and the public pose controller, runtime motion.TORQUE) limit
# the TOTAL joint torque to 80 N m on joints 1-4 and 12 N m on joints 5-7 (Franka Panda: 87/12 N m). The source
# reference used 80 N m on joint 5 and clipped only the residual, leaving the bias/gravity term unlimited.
LIM = np.array([80.] * 4 + [12.] * 3)
KP, KD = 550., 45.
DT = .002
HOLD_S = 20.                     # card: release and hold 20 s
RIGHT_PARK = np.array([0., -1.6, 0., -3.0, 0., 1.6, .7854])  # idle right arm folded back over its base (hand at x = .40)
TICK_BUDGET = 6000 * 25          # card: 6000 agent ticks (0.05 s) per deployment target, in mj_step ticks (reported)
FINGER_TAU = .3                  # finger position-servo target filter time constant, s (repo hand: none). A jumped jaw
                                 # command then closes each finger at ~0.15 m/s falling to ~0.1 m/s at contact on the
                                 # 58 mm sleeve (repo hand: ~0.85 m/s, about 10x a real Franka Hand; LEAD_DECISIONS #101)
HAND_SALT = 7110                 # per-coupling handedness draw; seed 42 keeps the card's handedness for all three

# ---------------- geometry (metres, post frame: origin at the stand top on the post axis, z up) -------------------
TABLE_Z = .80
STAND_H = .016                   # stand plate thickness; post frame origin z = TABLE_Z + STAND_H
POST_R, POST_TOP = .0155, .085
ZP = .070                        # pin height on the post
PIN_R, PIN_IN, PIN_OUT = .003, .012, .025   # pin tip 2 mm inside the shell (radial play 1.5 mm)
S0 = .040                        # pin height in the sleeve frame (origin at the sleeve mouth) at the nominal seat
WS = .005                        # axial half-height of a channel (pin radius + 2 mm)
WPHI = 14.                       # sector centres within +-WPHI deg of a channel axis are cut (edge at +-16 deg)
DA = 4.                          # track sector angular pitch, deg
R_IN, R_OUT, R_SHELL = .017, .027, .029
SLEEVE_L, CAP_T = .080, .006
SLEEVE_MASS = .25
MOUTH0 = ZP - PIN_R - (S0 - WS)  # sleeve mouth height above the stand top when seated (pins on the pocket floor)
PLUNGER_K, PLUNGER_PRE = 320., .015   # card: plunger 320 N/m; 15 mm preload when seated
PLUNGER_TOP0 = MOUTH0 + SLEEVE_L - CAP_T + PLUNGER_PRE  # pad top height at zero spring load
OUT = .10                        # exit pull, sleeve fully off the post and plunger
JUNCTION_DEG = 120.              # the first (push-level) junction spans 120 deg: the true turn plus a dead end the other
                                 # way (60 for a 60 deg turn, 90 for a 30 deg turn; payoff trial, DEVIATIONS 21)
DECOY_H, DECOY_DEG = .018, 20.   # behind a 90 deg dead end: notch height and the blind ledge at its top
NOTCH_H = .012                   # false-seat notch at the end of a 60 deg dead end: height (= the real seat pocket)

STANDS = {'A': (.05, -.30), 'B': (.05, -.12), 'C': (.05, .06)}
DEPLOY_XY = (.05, -.12)
TRAY_XY, TRAY_HALF, TRAY_FLOOR = np.array([.05, .26]), np.array([.10, .075]), TABLE_Z + .012
TRAY_SLOTS = {'A': (-.045, 0.), 'B': (.045, 0.), 'X': (0., 0.)}
COLOURS = {'A': '.95 .35 .08 1', 'B': '.10 .55 .95 1', 'C': '.65 .20 .85 1',
           'D': '.15 .80 .40 1', 'E': '.95 .80 .10 1', 'F': '.90 .30 .65 1'}
MARKS = {'A': 1, 'B': 2, 'C': 3, 'D': 1, 'E': 2, 'F': 3}

# ---------------- release paths: list of (x outward displacement in m, accumulated clockwise turn seen from above, deg)
# Card values. x < 0 is a push toward the stand, x = OUT is off. Turns are clockwise-positive as seen from above.
BASE_PATHS = {
    'A': [(0., 0.), (-.012, 0.), (-.012, 60.), (OUT, 60.)],                                   # push 12, CW60, out
    'B': [(0., 0.), (-.012, 0.), (-.012, -60.), (OUT, -60.)],                                 # push 12, CCW60, out
    'C': [(0., 0.), (-.012, 0.), (-.012, 30.), (.024, 30.), (.024, -30.), (OUT, -30.)],       # push, CW30, pull 24, CCW60, out
    'D': [(0., 0.), (-.012, 0.), (-.012, 30.), (.024, 30.), (.024, 60.), (OUT, 60.)],         # push, CW30, pull 24, CW30, out
    'E': [(0., 0.), (-.012, 0.), (-.012, -30.), (.024, -30.), (.024, -60.), (OUT, -60.)],     # push, CCW30, pull 24, CCW30, out
    'F': [(0., 0.), (-.012, 0.), (-.012, 30.), (.012, 30.), (.012, -30.), (OUT, -30.)],       # push, CW30, pull 12, CCW60, out (D2)
}


def mirrored(path, m): return [[round(float(x), 4), round(float(t) * m, 1)] for x, t in path]


TASK_A = 'A_clear_AB'
TARGETS = [
    dict(id=TASK_A, settings=['A'], cells=['path_A', 'path_B'], rules=[], objects=['A', 'B']),
    # Deployment targets list no hidden cells (lead #152 / #168): the best fixed uninformed rule (probe30: turn 30 deg the
    # guessed way, test a short pull, else turn back) comes within 7.0-9.9 % of informed on every remove_* target over
    # all 8 handedness worlds (DEVIATIONS 21 / 22), so they are skill / control targets. The geometry still follows
    # the practice paths (C <- path_C; D <- C's first stage + A's last-turn direction; E <- B family; F <- C family).
    dict(id='remove_C', settings=['S'], cells=[], rules=[], objects=['C']),
    dict(id='remove_D', settings=['T-combo'], cells=[], rules=[], objects=['D'],
         target_objects=['sleeve_D', 'stand_D']),
    dict(id='remove_E', settings=['T-new'], cells=[], rules=[], objects=['E'],
         target_objects=['sleeve_E', 'stand_E']),
    dict(id='remove_F', settings=['T-new'], cells=[], rules=[], objects=['F'],
         target_objects=['sleeve_F', 'stand_F']),
]
TARGET_BY_ID = {t['id']: t for t in TARGETS}


def handedness(seed):
    """One independent handedness bit per practice coupling (+1 = the card's direction, -1 = mirrored): 8 worlds.
    Dev seeds: 42 (+,+,+) = card, 43 (+,-,-), 44 (-,+,-)."""
    r = np.random.default_rng(np.random.SeedSequence([seed, HAND_SALT]))
    return dict(zip('ABC', (float(x) for x in r.choice([1., -1.], 3))))


def path_D(hC, hA):
    """T-combo D: C's first stage (push 12, turn 30, pull 24, in C's direction), then a 30 deg turn in A's direction
    and out. With equal bits this is the card's D (push 12, CW30, pull 24, CW30, out); with opposite bits the last turn
    goes back to the start angle (still a closed J-slot: 14 mm of wall separates the seat from the exit channel)."""
    t1 = 30. * hC
    return [[0., 0.], [-.012, 0.], [-.012, t1], [.024, t1], [.024, t1 + 30. * hA], [OUT, t1 + 30. * hA]]


def hands_of(params):
    """Handedness bits read back from the path cells (S or T instance)."""
    return dict(A=math.copysign(1., params['path_A'][2][1]), B=-math.copysign(1., params['path_B'][2][1]),
                C=math.copysign(1., params['path_C'][2][1]))


def parameters(seed):
    # plunger_k: per-seed nuisance (not a hidden cell; same for every coupling in the instance), card 320 N/m +- 12 %.
    h = handedness(seed)
    k = float(np.random.default_rng(np.random.SeedSequence([seed, 7107])).uniform(280., 360.))
    return dict(path_A=mirrored(BASE_PATHS['A'], h['A']), path_B=mirrored(BASE_PATHS['B'], h['B']),
                path_C=mirrored(BASE_PATHS['C'], h['C']), path_D=path_D(h['C'], h['A']),
                plunger_k=round(k, 1))


def transfer_parameters(params, seed):
    # T instance: same kind of coupling with new path values. B-family -> E (CCW30, pull 24, CCW30 in B's direction);
    # C-family -> F (CW30, pull 12, CCW60 in C's direction). Each family keeps its exploration handedness.
    h = hands_of(params)
    return dict(params, path_B=mirrored(BASE_PATHS['E'], h['B']), path_C=mirrored(BASE_PATHS['F'], h['C']))


def deploy_path(target, p):
    """Path of the single deployment coupling."""
    return dict(remove_C=p['path_C'], remove_D=p['path_D'], remove_E=p['path_B'], remove_F=p['path_C'])[target]


def scene_objects(target, p):
    """[(label, path, stand xy)] for the scene."""
    if target in (None, TASK_A): return [(c, p['path_' + c], STANDS[c]) for c in 'ABC']
    return [(TARGET_BY_ID[target]['objects'][0], deploy_path(target, p), DEPLOY_XY)]


# ---------------- track construction ---------------------------------------------------------------------------
def dead_ends(path):
    """Dead-end branch (payoff trial 2026-10-29, DEVIATIONS 21). Derived from the path alone (no hidden value added).
    At the push level (the first junction) the circumferential channel also runs the other way, so the junction spans
    JUNCTION_DEG: 60 deg of dead end for a 60 deg turn, 90 deg for a 30 deg turn. The far end has a blind false-seat
    notch (NOTCH_H, like the real seat pocket); behind a 90 deg dead end the notch is DECOY_H tall and its top turns
    DECOY_DEG back toward the true side as a blind ledge (it looks like the true path's 'pull, then turn'). A wrong-way
    first turn is therefore free until the far end, the pull there rises into the notch and stops, and the way out is a
    deliberate back-out (push down, turn back). Upper junctions are unchanged (a wrong-way turn jams at once)."""
    out = []
    for (x0, t0), (x1, t1) in zip(path, path[1:]):
        if abs(t1 - t0) < 1e-9 or x0 >= 0: continue
        sg = math.copysign(1., t1 - t0)
        dead = JUNCTION_DEG - abs(t1 - t0)
        td = t0 - sg * dead
        out.append(((x0, t0), (x0, td)))
        if dead > 60.:
            top = x0 + DECOY_H
            out += [((x0, td), (top, td)), ((top, td), (top, td + sg * DECOY_DEG))]
        else:
            out.append(((x0, td), (x0 + NOTCH_H, td)))
    return out


def track_segments(path): return list(zip(path, path[1:])) + dead_ends(path)


def free_intervals(path, phi):
    """Sleeve-frame z intervals that are cut free at track angle phi (deg), periodic in 180 deg (opposed pins)."""
    out = []
    for (x0, t0), (x1, t1) in track_segments(path):
        if abs(t0 - t1) < 1e-9:
            if abs((phi - t0 + 90.) % 180. - 90.) <= WPHI + 1e-9:
                lo = -1. if max(x0, x1) >= OUT - 1e-9 else S0 - max(x0, x1) - WS
                out.append((lo, S0 - min(x0, x1) + WS))
        else:
            lo, hi = sorted((t0, t1))
            if any(lo - WPHI - 1e-9 <= phi + 180. * k <= hi + WPHI + 1e-9 for k in range(-3, 4)):
                out.append((S0 - x0 - WS, S0 - x0 + WS))
    return out


def solid_intervals(free, top=SLEEVE_L - CAP_T):
    at, out = 0., []
    for lo, hi in sorted(free):
        lo, hi = max(lo, 0.), min(hi, top)
        if hi <= at: continue
        if lo > at + 1e-6: out.append((at, lo))
        at = max(at, hi)
    if at < top - 1e-6: out.append((at, top))
    return out


def f3(v): return ' '.join(f'{x:.6g}' for x in v)


def zquat(deg): a = math.radians(deg) / 2; return f'{math.cos(a):.9g} 0 0 {math.sin(a):.9g}'


def add_coupling(world, label, path, xy, k_plunger=PLUNGER_K):
    ox, oy = xy; oz = TABLE_Z + STAND_H
    stand = ET.SubElement(world, 'body', name='stand_' + label, pos=f3([ox, oy, oz]))
    ET.SubElement(stand, 'geom', name=f'stand_plate_{label}', type='box', pos='0 0 -.008', size='.060 .055 .008',
                  rgba='.24 .28 .30 1')
    for x in (-.045, .045):
        for y in (-.037, .037):
            ET.SubElement(stand, 'geom', name=f'stand_bolt_{label}_{x:+.3f}_{y:+.3f}', type='cylinder', pos=f3([x, y, .002]),
                          size='.005 .002', rgba='.16 .18 .20 1', mass='0')
    ET.SubElement(stand, 'geom', name=f'post_{label}', type='cylinder', fromto=f3([0, 0, 0, 0, 0, POST_TOP]),
                  size=f'{POST_R}', rgba='.17 .65 .66 1', contype='3', conaffinity='3', friction='.12 .001 .0001')
    for sgn in (-1, 1):
        ET.SubElement(stand, 'geom', name=f'pin_{label}_{"pn"[sgn > 0]}', type='cylinder',
                      fromto=f3([sgn * PIN_IN, 0, ZP, sgn * PIN_OUT, 0, ZP]), size=f'{PIN_R}', rgba='.65 .68 .70 1',
                      contype='3', conaffinity='3', friction='.12 .001 .0001')
    plunger = ET.SubElement(stand, 'body', name='plunger_' + label, pos=f3([0, 0, PLUNGER_TOP0 - .003]))
    ET.SubElement(plunger, 'joint', name='plunger_slide_' + label, type='slide', axis='0 0 1', range='-.032 0',
                  stiffness=f'{k_plunger}', springref='0', damping='1.5', armature='.001')
    ET.SubElement(plunger, 'geom', name=f'plunger_pad_{label}', type='cylinder', size='.009 .003', mass='.02',
                  rgba='.18 .20 .22 1', contype='3', conaffinity='3', friction='.10 .001 .0001')
    ET.SubElement(plunger, 'geom', name=f'plunger_stem_{label}', type='cylinder', fromto='0 0 -.003 0 0 -.03', size='.004',
                  mass='.005', rgba='.62 .64 .66 1', contype='0', conaffinity='0')
    sleeve = ET.SubElement(world, 'body', name='sleeve_' + label, pos=f3([ox, oy, oz + MOUTH0]))
    ET.SubElement(sleeve, 'freejoint', name='sleeve_free_' + label)
    ri2, ro2 = R_IN ** 2, R_SHELL ** 2
    ET.SubElement(sleeve, 'inertial', pos=f3([0, 0, SLEEVE_L / 2]), mass=f'{SLEEVE_MASS}',
                  diaginertia=f3([SLEEVE_MASS / 12 * (3 * (ri2 + ro2) + SLEEVE_L ** 2)] * 2 + [SLEEVE_MASS / 2 * (ri2 + ro2)]))
    metal = '.60 .64 .67 1'
    n_shell = 48
    for i in range(n_shell):
        a = 360. * i / n_shell; t = math.radians(a); r = (R_OUT + R_SHELL) / 2
        ET.SubElement(sleeve, 'geom', name=f'shell_{label}_{i}', type='box', pos=f3([r * math.cos(t), r * math.sin(t), SLEEVE_L / 2]),
                      size=f3([(R_SHELL - R_OUT) / 2, r * math.tan(math.radians(180. / n_shell + .3)), SLEEVE_L / 2]),
                      quat=zquat(a), rgba=metal, friction='1.0 .01 .001')
    ET.SubElement(sleeve, 'geom', name=f'cap_{label}', type='cylinder', pos=f3([0, 0, SLEEVE_L - CAP_T / 2]),
                  size=f3([R_OUT + .0005, CAP_T / 2]), rgba=metal, contype='3', conaffinity='3', friction='.6 .01 .001')
    rm = (R_IN + R_OUT) / 2
    for j in range(int(360 / DA)):
        phi = j * DA; t = math.radians(phi)
        for i, (lo, hi) in enumerate(solid_intervals(free_intervals(path, phi))):
            ET.SubElement(sleeve, 'geom', name=f'track_{label}_{j}_{i}', type='box',
                          pos=f3([rm * math.cos(t), rm * math.sin(t), (lo + hi) / 2]),
                          size=f3([(R_OUT - R_IN) / 2, rm * math.tan(math.radians(DA / 2 + .1)), (hi - lo) / 2]),
                          quat=zquat(phi), rgba='.52 .55 .58 1', contype='3', conaffinity='3', friction='.12 .001 .0001')
    # identity: coloured band near the top and white bars on the cap (common geometry for every path)
    for i in range(n_shell):
        a = 360. * i / n_shell; t = math.radians(a); r = R_SHELL + .0002
        ET.SubElement(sleeve, 'geom', name=f'band_{label}_{i}', type='box', pos=f3([r * math.cos(t), r * math.sin(t), SLEEVE_L - .012]),
                      size=f3([.0002, r * math.tan(math.radians(180. / n_shell + .3)), .005]), quat=zquat(a),
                      rgba=COLOURS[label], contype='0', conaffinity='0', mass='0')
    for j in range(MARKS[label]):
        ET.SubElement(sleeve, 'geom', name=f'mark_{label}_{j}', type='box', pos=f3([-.008 + .008 * j, 0, SLEEVE_L + .0002]),
                      size='.0015 .010 .0002', rgba='.97 .97 .97 1', contype='0', conaffinity='0', mass='0')
    # reference grasp marker on the stand is not used; stands carry their letter as a coloured corner tab
    ET.SubElement(stand, 'geom', name=f'stand_tab_{label}', type='box', pos='-.050 -.045 .0005', size='.008 .008 .0005',
                  rgba=COLOURS[label], contype='0', conaffinity='0', mass='0')


def add_tray(world):
    tray = ET.SubElement(world, 'body', name='tray', pos=f3([TRAY_XY[0], TRAY_XY[1], TABLE_Z]))
    hx, hy = TRAY_HALF; wall = .004; h = .028
    ET.SubElement(tray, 'geom', name='tray_floor', type='box', pos=f3([0, 0, .006]), size=f3([hx + wall, hy + wall, .006]),
                  rgba='.20 .62 .30 1', friction='.8 .01 .001')
    for sx in (-1, 1):
        ET.SubElement(tray, 'geom', name=f'tray_wall_x{sx:+d}', type='box', pos=f3([sx * (hx + wall / 2), 0, .012 + h / 2]),
                      size=f3([wall / 2, hy + wall, h / 2]), rgba='.18 .55 .27 1')
    for sy in (-1, 1):
        ET.SubElement(tray, 'geom', name=f'tray_wall_y{sy:+d}', type='box', pos=f3([0, sy * (hy + wall / 2), .012 + h / 2]),
                      size=f3([hx, wall / 2, h / 2]), rgba='.18 .55 .27 1')


def build_xml(seed, target=None, params=None):
    p = params or parameters(seed)
    root = ET.fromstring(portable_scene(seed, 'test')[0]); world = root.find('worldbody')
    for body in list(world.findall('body')):
        if not body.get('name', '').startswith(('left_', 'right_')): world.remove(body)
    for sec in ('contact', 'equality', 'tendon'):
        node = root.find(sec)
        if node is None: node = ET.SubElement(root, sec)
        for c in list(node): node.remove(c)
    root.find('visual/global').attrib.update(offwidth='1280', offheight='720')
    root.find('option').set('iterations', '100')
    for a in root.find('actuator'):               # realistic finger closing speed (DEVIATIONS 17)
        if a.get('name', '').endswith(('_finger_a', '_finger_b')): a.set('timeconst', f'{FINGER_TAU:g}')
    size = root.find('size')
    if size is not None: size.attrib.update(nconmax='800', njmax='4000')
    root.set('model', BENCH_ID + ('' if target in (None, TASK_A) else '_' + target))
    for label, path, xy in scene_objects(target, p): add_coupling(world, label, path, xy, p.get('plunger_k', PLUNGER_K))
    add_tray(world)
    world.extend(ET.fromstring('''<w><body name="overview_target" pos=".02 -.05 .88"/>
      <camera name="overview" mode="targetbody" target="overview_target" pos=".95 -1.05 1.45" fovy="42"/></w>'''))
    return ET.tostring(root, encoding='unicode')


def build(seed, target=None, params=None):
    p = params or parameters(seed)
    m = mujoco.MjModel.from_xml_string(build_xml(seed, target, p)); d = mujoco.MjData(m)
    k = ids(m)
    d.qpos[k['q'][0]] = HOME; d.qpos[k['q'][1]] = RIGHT_PARK
    d.qpos[k['fingers']] = .045; d.ctrl[14:] = .045
    d.act[:] = d.ctrl[14:]           # finger filter state starts at the open target (no slam at reset; review M1)
    for label in k['labels']: d.qpos[m.jnt_qposadr[m.joint('plunger_slide_' + label).id]] = -PLUNGER_PRE
    mujoco.mj_forward(m, d)
    return m, d


def ids(m):
    arms = [[m.joint(f'{s}_joint{i}').id for i in range(1, 8)] for s in ('left', 'right')]
    labels = [m.body(b).name[len('sleeve_'):] for b in range(m.nbody) if m.body(b).name.startswith('sleeve_')]
    robot = [g for g in range(m.ngeom) if m.body(m.geom_bodyid[g]).name.startswith(('left_', 'right_'))]
    sleeve_geoms = {c: [g for g in range(m.ngeom) if m.geom_bodyid[g] == m.body('sleeve_' + c).id] for c in labels}
    stand_geoms = {c: [g for g in range(m.ngeom) if m.body_rootid[m.geom_bodyid[g]] == m.body('stand_' + c).id
                       or m.geom_bodyid[g] == m.body('stand_' + c).id] for c in labels}
    tray = [g for g in range(m.ngeom) if m.geom_bodyid[g] == m.body('tray').id]
    return dict(q=[m.jnt_qposadr[a].tolist() for a in arms], v=[m.jnt_dofadr[a].tolist() for a in arms],
                fingers=[int(m.jnt_qposadr[m.joint(f'{s}_finger_{f}').id]) for s in ('left', 'right') for f in 'ab'],
                labels=labels, robot=robot, sleeve_geoms=sleeve_geoms, stand_geoms=stand_geoms, tray=tray,
                tray_floor=int(m.geom('tray_floor').id),
                sleeve=[int(m.body('sleeve_' + c).id) for c in labels],
                sleeve_dof=[int(m.jnt_dofadr[m.joint('sleeve_free_' + c).id]) for c in labels],
                tip=int(m.site('left_tip').id))


def goal_labels(target): return ['A', 'B'] if target in (None, TASK_A) else TARGET_BY_ID[target]['objects']


def init_state(seed, target, params):
    return dict(target=target, goals=goal_labels(target), hold=0., done=False, success_at=None, removed={})


# Repair round 4 (2026-11-09): per-axis footprint of the sleeve's bounding cylinder. The previous bound applied
# R_SHELL + L/2*|axis_xy| to BOTH tray axes, so a horizontal sleeve lying along tray X with a 30 mm sideways offset was
# rejected although it fits. For a cylinder of radius R and length L with unit axis a, its half-extent along world
# axis e is L/2*|a.e| + R*sqrt(1-(a.e)^2). R = 0.0295 m bounds every shell/band box corner (max 0.02947 m); the axial span
# 0 .. L+0.4 mm includes the cap marks. Walls' inner faces sit at TRAY_HALF.
FOOT_R, FOOT_L = .0295, SLEEVE_L + .0004


def sleeve_footprint(pos, R):
    """(centre_xy, half_extent_xy) of the sleeve bounding cylinder for body pose (pos, 3x3 R)."""
    a = np.asarray(R)[:, 2]
    c = np.asarray(pos)[:2] + a[:2] * FOOT_L / 2
    half = FOOT_L / 2 * np.abs(a[:2]) + FOOT_R * np.sqrt(np.clip(1. - a[:2] ** 2, 0., 1.))
    return c, half


def sleeve_inside_tray(pos, R):
    c, half = sleeve_footprint(pos, R)
    return bool(np.all(np.abs(c - TRAY_XY) + half < TRAY_HALF))


def sleeve_status(m, d, k, label):
    """(in_tray_and_resting, robot_touch, post_touch) for one sleeve, from native contacts and poses."""
    i = k['labels'].index(label); b = k['sleeve'][i]; geoms = k['sleeve_geoms'][label]
    g1, g2 = d.contact.geom1, d.contact.geom2
    mine1, mine2 = np.isin(g1, geoms), np.isin(g2, geoms)
    rob = bool(np.any((mine1 & np.isin(g2, k['robot'])) | (mine2 & np.isin(g1, k['robot']))))
    post = bool(np.any((mine1 & np.isin(g2, k['stand_geoms'][label])) | (mine2 & np.isin(g1, k['stand_geoms'][label]))))
    floor = bool(np.any((mine1 & (g2 == k['tray_floor'])) | (mine2 & (g1 == k['tray_floor']))))
    inside = sleeve_inside_tray(d.xpos[b], d.xmat[b].reshape(3, 3))
    v = d.qvel[k['sleeve_dof'][i]:k['sleeve_dof'][i] + 6]
    slow = bool(np.linalg.norm(v[:3]) < .02 and np.linalg.norm(v[3:]) < .10)
    return inside and floor and slow and not post, rob, post


def step(m, d, state):
    k = state.get('ids') or state.setdefault('ids', ids(m))
    dofs = k['v'][0] + k['v'][1]
    d.qfrc_applied[dofs] = d.qfrc_bias[dofs]
    mujoco.mj_step(m, d)
    ok = True
    for label in state['goals']:
        rest, rob, _ = sleeve_status(m, d, k, label)
        ok &= rest and not rob
    state['hold'] = state['hold'] + m.opt.timestep if ok else 0.
    if state['hold'] >= HOLD_S and not state['done']:
        state['done'] = True; state['success_at'] = round(float(d.time), 4)


def success(model, data, state, target):
    # Repair round 1 (2026-11-08): success is the CURRENT unbroken hold (>= 20 s, still valid now), not the latched 'done'
    # event. A sleeve that is touched, lifted or moved out of the tray after the hold no longer scores. 'done'/'success_at'
    # remain as event records only.
    return bool(state['hold'] >= HOLD_S)


def damage(model, data, state): return {}


ROBOT_TEXT = ('Robot: two Panda arms with parallel-jaw hands; the left arm reaches every holder and the tray. Joint torque '
              'limits 80 N m on joints 1-4, 12 N m on joints 5-7. Each finger squeezes with at '
              'most 40 N. '
              'Working a sleeve may need a wrist force guard up to about 35 N. A wrist force above 80 N for 100 ms aborts the episode. Tool reach box: X -0.30 to 0.35 m, '
              'Y -0.40 to 0.40 m, Z 0.825 to 1.18 m (table top at 0.80 m).')


def public_text(target):
    if target in (None, TASK_A):
        scene = ('Three couplings stand on the table, each a metal sleeve on a fixed shaft post with its own holder, labelled '
                 'A (orange band, one white bar on the cap), B (blue band, two bars) and C (purple band, three bars), in a row '
                 'with B in the middle, plus a green tray.')
        if target is None: return f'{scene} {ROBOT_TEXT} Goal: explore the couplings. The deployment goal is given after reset.'
        return (f'{scene} {ROBOT_TEXT} Goal: take sleeves A and B off their shafts and put both in the green tray. '
                'Success: both shafts clear, both sleeves resting wholly inside the tray, no robot contact, held for 20 s after you let go.')
    c = TARGET_BY_ID[target]['objects'][0]
    names = dict(C='purple band, three bars', D='green band, one bar', E='yellow band, two bars', F='pink band, three bars')
    return (f'One coupling ({c}: {names[c]}) stands on its holder, a metal sleeve on a fixed shaft post, next to a green tray. '
            f'{ROBOT_TEXT} Goal: take sleeve {c} off its shaft and put it in the green tray. Success: the shaft is clear, '
            'the sleeve rests wholly inside the tray, no robot contact, held for 20 s after you let go.')


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


# ---------------- scripted controller (privileged geometry; never reads the path except where the caller passes it) ----
def ik(m, scratch, k, target, yaw, q0, iters=15):
    q, v, tip = k['q'][0], k['v'][0], k['tip']
    scratch.qpos[q] = q0
    R = Rotation.from_euler('z', yaw).as_matrix() @ DOWN
    jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv)); lo, hi = m.jnt_range[[m.dof_jntid[i] for i in v]].T
    for _ in range(iters):
        mujoco.mj_kinematics(m, scratch); mujoco.mj_comPos(m, scratch)
        err = np.r_[target - scratch.site_xpos[tip], .5 * Rotation.from_matrix(R @ scratch.site_xmat[tip].reshape(3, 3).T).as_rotvec()]
        if np.linalg.norm(err) < 1e-6: break
        mujoco.mj_jacSite(m, scratch, jp, jr, tip)
        J = np.vstack([jp[:, v], .5 * jr[:, v]])
        Jp = J.T @ np.linalg.inv(J @ J.T + 1e-4 * np.eye(6))
        # task step plus a null-space pull toward the rest posture (keeps the redundant elbow on one branch)
        rest = REST.copy(); rest[0] = math.atan2(target[1] - BASE_L[1], target[0] - BASE_L[0])
        dq = Jp @ err + (np.eye(7) - Jp @ J) @ (NULL_W * (rest - scratch.qpos[q]))
        scratch.qpos[q] = np.clip(scratch.qpos[q] + np.clip(dq, -.08, .08), lo + .01, hi - .01)
    return scratch.qpos[q].copy()


REST = np.array([0., .3, 0., -2.2, 0., 2.5, 0.])   # natural elbow-up branch; j1 is aimed at the target
NULL_W = np.array([.2, .2, .2, .1, .2, .1, 0.])
BASE_L = np.array([-.40, -.15])                    # left arm base (portable_demo.env)
YAW0 = math.pi / 2               # jaw axis along world Y at zero turn
SERVO_KI = .3
SERVO_MAX = .006
IK_EVERY = 10
AIM_ITERS = 6
GRASP_Z = .052                   # tool point height above the sleeve mouth when grasping


class Driver:
    """Cartesian tool-point + yaw targets tracked by joint PD (left arm); records ctrl rows. Deterministic."""

    def __init__(self, seed, target, params):
        self.m, self.d = build(seed, target, params); self.st = init_state(seed, target, params)
        self.k = ids(self.m); self.scratch = mujoco.MjData(self.m)
        # start from the measured pose (polish 2026-10-29: the old start set the IK target for YAW0 at once, so j7 spun
        # 90 deg at 4.3x its speed limit in the first 0.1 s); the first move turns the hand to YAW0 at a legal rate
        self.pos = self.d.site_xpos[self.k['tip']].copy(); self.jaw = .045
        Rz = self.d.site_xmat[self.k['tip']].reshape(3, 3) @ DOWN.T; self.yaw = math.atan2(Rz[1, 0], Rz[0, 0])
        self.qt = self.d.qpos[self.k['q'][0]].copy()
        self.acts = []; self.log = []; self.servo = None; self.corr = np.zeros(3)
        self.peak_total = np.zeros((2, 7))   # repair round 3: peak |total joint torque| per arm/joint, N m

    def servo_off(self):
        self.pos = self.pos + self.corr; self.corr = np.zeros(3); self.servo = None

    def tick(self):
        m, d, k = self.m, self.d, self.k
        u = np.zeros(m.nu)
        for arm, qd in ((0, self.qt), (1, RIGHT_PARK)):
            # total (residual + bias) torque clipped to the declared limits, as runtime motion.py does
            bias = d.qfrc_bias[k['v'][arm]]
            total = np.clip(KP * (qd - d.qpos[k['q'][arm]]) - KD * d.qvel[k['v'][arm]] + bias, -LIM, LIM)
            u[7 * arm:7 * arm + 7] = total - bias
            self.peak_total[arm] = np.maximum(self.peak_total[arm], np.abs(total))
        u[14:16] = self.jaw_now; u[16:] = .045
        d.ctrl[:] = u; self.acts.append(u.copy()); step(m, d, self.st)

    def move(self, goal=None, yaw=None, jaw=None, n=200, settle=40, monitor=None):
        a, y0, j0 = self.pos.copy(), self.yaw, self.jaw
        goal = a if goal is None else np.asarray(goal, float); yaw = y0 if yaw is None else yaw; jaw = j0 if jaw is None else jaw
        sf = lambda i: (lambda f: f * f * (3 - 2 * f))(min(1., (i + 1) / max(1, n - settle)))
        for i in range(n):
            f = sf(i)
            if i % IK_EVERY == 0:
                # IK for the target IK_EVERY ticks ahead; joint targets are interpolated per tick (no torque steps)
                if self.servo is not None:   # keep the held sleeve's axis on the post axis (integral lateral servo)
                    label, xy = self.servo
                    self.corr[:2] = np.clip(self.corr[:2] + SERVO_KI * (xy - self.sleeve(label)[0][:2]), -SERVO_MAX, SERVO_MAX)
                fa = sf(min(n - 1, i + IK_EVERY - 1))
                self.q0 = self.qt.copy()
                self.q1 = ik(self.m, self.scratch, self.k, a + (goal - a) * fa + self.corr, y0 + (yaw - y0) * fa, self.qt)
                self.qn = min(IK_EVERY, n - i)
            self.qt = self.q0 + (self.q1 - self.q0) * ((i % IK_EVERY) + 1) / self.qn
            self.jaw_now = j0 + (jaw - j0) * f
            self.tick()
            if monitor is not None and monitor(self, i):
                # stopped early: hold the target reached so far (joint target already interpolated to here)
                self.pos, self.yaw, self.jaw = a + (goal - a) * f, y0 + (yaw - y0) * f, self.jaw_now
                self.q1 = self.qt.copy(); return self
        self.pos, self.yaw, self.jaw = goal, yaw, jaw
        return self

    def wait(self, n):
        self.jaw_now = self.jaw
        for _ in range(n): self.tick()
        return self

    # privileged helpers
    def sleeve(self, label):
        b = self.m.body('sleeve_' + label).id
        return self.d.xpos[b].copy(), self.d.xmat[b].reshape(3, 3).copy()

    def post_origin(self, label): return self.d.xpos[self.m.body('stand_' + label).id].copy()


def grasp(dr, label):
    """Open hand above the sleeve, descend, close. Tool point on the sleeve axis."""
    p, _ = dr.sleeve(label)
    top = p + [0, 0, GRASP_Z]
    dr.move(top + [0, 0, .12], YAW0, .045, 700)            # 700 ticks: the first call also turns the hand from HOME yaw
    dr.move(top, YAW0, .045, 350)
    dr.move(None, None, 0., 200)
    dr.wait(60)


def follow_path(dr, label, path, push_ticks=300, turn_ticks_per_deg=10, pull_ticks_per_m=25000, seat_from=None, aim=True):
    """Move the grasped sleeve along a release path. The tool target keeps the sleeve axis on the post axis,
    using the measured sleeve-in-hand offset (privileged pose). seat_from: (x, turn) the sleeve is at now, if not seated."""
    post = dr.post_origin(label)
    if seat_from is None or not hasattr(dr, 'grip'):
        p, _ = dr.sleeve(label)
        tip = dr.d.site_xpos[dr.k['tip']].copy(); Rt = dr.d.site_xmat[dr.k['tip']].reshape(3, 3).copy()
        off = Rt.T @ (tip - p)                # tool point relative to the sleeve origin, hand frame
        seat = p.copy(); seat[:2] = post[:2]  # sleeve origin on the post axis ...
        seat[2] -= WS - PIN_R                 # ... at the nominal seat (pin centred in the channels), not on the pocket floor
        dr.grip = (off, seat)
    off, seat = dr.grip
    dr.servo = (label, post[:2].copy())
    bx = bth = 0.                                   # aim offsets (arm compliance), re-measured after every segment

    def target(x, t):
        yaw = YAW0 - math.radians(t + bth)
        return seat + [0, 0, x + bx] + (Rotation.from_euler('z', yaw).as_matrix() @ DOWN) @ off, yaw

    for (x0, t0), (x1, t1) in zip(path, path[1:]):
        x1c = min(x1, OUT)
        if abs(t1 - t0) > 1e-9: n = int(abs(t1 - t0) * turn_ticks_per_deg) + 80
        elif x1 < x0: n = push_ticks
        else: n = int(abs(x1c - x0) * pull_ticks_per_m) + 80
        if abs(t1 - t0) < 1e-9: bx = 0.             # axial compliance flips with the load direction
        dr.move(*target(x1c, t1), 0., n, settle=60)
        if aim and x1c < OUT:                       # aim: put the sleeve on the path vertex (privileged pose)
            for _ in range(AIM_ITERS):
                p, R = dr.sleeve(label)
                ex = x1c - (p[2] - seat[2]); et = t1 + math.degrees(math.atan2(R[1, 0], R[0, 0]))
                if abs(ex) < .0007 and abs(et) < 1.: break
                bx = float(np.clip(bx + ex, -.005, .005)); bth = float(np.clip(bth + et, -6., 6.))
                dr.move(*target(x1c, t1), 0., 80, settle=20)
        dr.log.append(dict(seg=((x0, t0), (x1, t1)), t=float(dr.d.time)))
    dr.servo_off()


def place_in_tray(dr, label, slot, final_yaw=None):
    p, _ = dr.sleeve(label)
    tip = dr.d.site_xpos[dr.k['tip']].copy()
    off_z = tip[2] - p[2]
    sx, sy = TRAY_XY + np.array(TRAY_SLOTS[slot])
    up = max(tip[2] + .03, TRAY_FLOOR + off_z + .10)
    dr.move([tip[0], tip[1], up], None, 0., 300)
    dr.move([sx, sy, up], YAW0 if final_yaw is None else final_yaw, 0., 900)   # 900: j1 swing + 60 deg yaw back stays < j7 limit
    dr.move([sx, sy, TRAY_FLOOR + off_z + .02], None, 0., 350)
    dr.move([sx, sy, TRAY_FLOOR + off_z + .001], None, 0., 400, settle=100)
    dr.move(None, None, .045, 150)
    dr.move([sx, sy, up + .06], None, .045, 300)


PARK = np.array([-.10, -.10, 1.10])


def reference_actions(seed, target, params, hold=True):
    p = params or parameters(seed)
    dr = Driver(seed, target, p)
    if target in (None, TASK_A):
        jobs = [('A', p['path_A'], 'A'), ('B', p['path_B'], 'B')]
    else:
        c = TARGET_BY_ID[target]['objects'][0]; jobs = [(c, deploy_path(target, p), 'X')]
    for label, path, slot in jobs:
        grasp(dr, label); follow_path(dr, label, path); place_in_tray(dr, label, slot)
    dr.move(PARK, YAW0, .045, 500)
    if hold: dr.wait(int(HOLD_S / DT) + 300)
    return dr


def reference(seed, target, params):
    return np.array(reference_actions(seed, target, params or parameters(seed)).acts)


# ---------------- probes (same scripted motion for both contrast values) ----------------------------------------
PROBE_PUSH = .0145               # probe push command: pins against the circumferential channel ceiling


def probe_motion(seed, params, label, turn_deg, pull):
    """Scripted probe: grasp, push until the pins bear on the channel ceiling (14.5 mm command), turn turn_deg (CW positive), then optionally pull. Same targets for any path;
    no aiming, so the tick windows are identical for every seed and contrast value."""
    dr = Driver(seed, None, params)
    grasp(dr, label)
    dr.mark = {}
    follow_path(dr, label, [(0., 0.), (-PROBE_PUSH, 0.)], aim=False); dr.wait(50)
    dr.mark['turn'] = len(dr.acts)
    follow_path(dr, label, [(-PROBE_PUSH, 0.), (-PROBE_PUSH, turn_deg)], seat_from=(-PROBE_PUSH, 0.), aim=False); dr.wait(50)
    dr.mark['turn_end'] = len(dr.acts)
    if pull:
        follow_path(dr, label, [(-PROBE_PUSH, turn_deg), (-PROBE_PUSH + pull, turn_deg)], seat_from=(-PROBE_PUSH, turn_deg), aim=False); dr.wait(50)
    dr.mark['end'] = len(dr.acts)
    return dr


# A / B: with the dead-end branches a wrong-way turn is free, so the probe turns the full 60 deg and then pulls 30 mm:
# at the true end the pull is free (exit channel); at the dead end it rises 12 mm into the false-seat notch and stops.
PROBE_SPECS = dict(turn_pull_cw=('A', 60., .030), turn_pull_ccw=('B', -60., .030), turn_pull=('C', 30., .015))
def probe_turn_pull_cw(seed, params): return np.array(probe_motion(seed, params, *PROBE_SPECS['turn_pull_cw']).acts)
def probe_turn_pull_ccw(seed, params): return np.array(probe_motion(seed, params, *PROBE_SPECS['turn_pull_ccw']).acts)
def probe_turn_pull(seed, params): return np.array(probe_motion(seed, params, *PROBE_SPECS['turn_pull']).acts)


PROBE_WINDOWS = {}


def _window(name):
    if name not in PROBE_WINDOWS:
        PROBE_WINDOWS[name] = probe_motion(42, parameters(42), *PROBE_SPECS[name]).mark
    return PROBE_WINDOWS[name]


def pull_force(name):
    """Plateau axial force while the hand pulls: median |Fz - Fz(start)| over the last half of the pull window."""
    def stat(tr):
        w = _window(name); s, e = w['turn_end'], w['end']
        fz = tr['wrench'][s:e, 2]
        if len(fz) < 10: return 0.
        return float(np.median(np.abs(fz[len(fz) // 2:] - fz[0])))
    return stat


PROBES = dict(turn_pull_cw=probe_turn_pull_cw, turn_pull_ccw=probe_turn_pull_ccw, turn_pull=probe_turn_pull)
HIDDEN_CELLS = [
    dict(name='path_A', type='M', object='sleeve_A', contrast=(mirrored(BASE_PATHS['A'], 1.), mirrored(BASE_PATHS['B'], 1.)),
         probe='turn_pull_cw', channel='force', statistic=pull_force('turn_pull_cw')),
    dict(name='path_B', type='M', object='sleeve_B', contrast=(mirrored(BASE_PATHS['B'], 1.), mirrored(BASE_PATHS['A'], 1.)),
         probe='turn_pull_ccw', channel='force', statistic=pull_force('turn_pull_ccw')),
    dict(name='path_C', type='M', object='sleeve_C', contrast=(mirrored(BASE_PATHS['C'], 1.), mirrored(BASE_PATHS['A'], 1.)),
         probe='turn_pull', channel='force', statistic=pull_force('turn_pull')),
]
RULE_TESTS = []   # the mechanism is native contact geometry (pins, track walls, spring plunger); there are no rules
OBJECT_BOX = ((-.75, -.85, .70), (.75, .85, 1.60))
WORKSPACE = ((-.30, -.40, .825), (.35, .40, 1.18))   # agent tool-point goal box (= public_text reach box; lead #158)
PUBLIC_WORDS = []


def render(model, data, path, camera='overview'):
    r = mujoco.Renderer(model, 720, 1280); r.update_scene(data, camera=camera); img = r.render(); r.close()
    chunk = lambda t, b: struct.pack('>I', len(b)) + t + b + struct.pack('>I', zlib.crc32(t + b) & 0xffffffff)
    raw = b''.join(b'\x00' + row.tobytes() for row in img)
    Path(path).write_bytes(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', 1280, 720, 8, 2, 0, 0, 0))
                           + chunk(b'IDAT', zlib.compress(raw, 6)) + chunk(b'IEND', b''))
