# Card: cards/B4_bars.md (B4 重心藏起來的長條), ported 2026-10-28 to the ACCEPTANCE.md v1 interface.
# Earlier implementation (geometry, sampling, success predicate): repo_snapshot/backend/playground/benches/b4.py.
# Robot: the repo's two-Panda B4 robot (portable_demo.env.scene(seed, 'test') with every non-robot body stripped, as in
# examples/toy_drawer.py). Stock Franka hand (per finger kp 1200, +-40 N). The left arm does all work with top-down grasps
# (jaw axis along world Y); the right arm only parks out of the way.
# Actions are data.ctrl rows (nu = 18): 14 arm joint torques (residual: step() adds the arm gravity/bias compensation, so
# zero control floats the arms in place) and 4 finger position targets (0 closed .. .045 open, per finger).
# Objects: three look-alike bars (50 x 50 mm aluminium square tube, 320 mm, with a hidden steel slug that sets the mass and
# the centre-of-mass offset along the long axis), plus one support per scene:
#   exploration / task A: a 2-4-6 steel riser block (practice pedestal);
#   deployment: 24 mm ridge (edge_A), 40 mm ridge (edge_B), point pedestal 28 x 30 mm (stack_ABC), 40 mm ridge pedestal (stack_CBA).
# Hidden cells: com_A/B/C (P, signed offset of the centre of mass from the bar's middle along its long axis, +x = away from the
# capped end) and mass_A/B/C (P, intermediate only: the agent needs it to turn the wrist torque into an offset).
# Native rigid contact only; no rules (card: "原生接觸，沒有規則").
import os, sys, struct, zlib
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation, Slerp

REPO = next(p / 'native_support' for p in Path(__file__).resolve().parents if (p / 'native_support').is_dir())
sys.path[:0] = [str(REPO), str(REPO / 'mujoco-box-demo')]
from dual_arm_demo.env import HOME, DOWN
from portable_demo.env import scene as portable_scene

BENCH_ID = 'B4_bars'
SEEDS = (42, 43, 44)
LIM = np.array([80.] * 5 + [12.] * 2)      # arm torque clip, as dual_arm_demo/env.py
KP, KD = 550., 45.
DT = .002
HOLD_S = 20.                               # card: 放手 20 s
LEVEL = .06                                # rad (3.4 deg), the earlier build's tilt tolerance
STILL = .025                               # norm of the bar's 6-dof velocity, the earlier build's "low speed" tolerance

# ---------------------------------------------------------------- objects
BAR_L, BAR_A = .32, .05                    # bar length and square side (m)
BAR_HL, BAR_HA = BAR_L / 2, BAR_A / 2
SHELL_M = .19                              # tube + two end caps, kg (6063 tube 50 x 50 x 1.0 mm: 0.53 kg/m x 0.32 m + 2 x 10 g caps)
COM_RANGE = (.010, .090)                   # |COM offset| (m), random sign; widened from the card's 45-75 mm (review L1, DEVIATIONS)
YAW_TOL = np.radians(10)                   # long axis along world X (across the supports), either end first
BARS = ('A', 'B', 'C')
BAR_RGBA = dict(A='.96 .56 .14 1', B='.12 .35 .90 1', C='.65 .14 .80 1')
BAR_Y = dict(A=-.30, B=-.19, C=-.08)       # reset: lying along world X, capped end towards -X, centre x = 0
BAR_X = 0.
TABLE_Z = .80
SUPPORT_XY = np.array([0., .18])
# name: (half sizes x (along the bar), y, z), top z, label. Tops sit 152-160 mm above the table.
SUPPORTS = dict(
    riser_246=((.0254, .0508, .0762), 'steel 2-4-6 riser block standing on its 2 x 4 in face'),
    ridge_24=((.012, .050, .080), 'steel ridge support, 24 mm wide'),
    ridge_40=((.020, .050, .080), 'steel ridge support, 40 mm wide'),
    point_28=((.014, .015, .080), 'steel point pedestal, 28 x 30 mm top'),
    ridge_ped_40=((.020, .050, .080), 'steel ridge pedestal, 40 mm wide'),
)
SUPPORT_TXT = dict(
    riser_246='A steel riser block stands on the table (a 2-4-6 block on its 2 x 4 in face: top 51 mm along the bars by 102 mm, 152 mm high).',
    ridge_24='A narrow steel ridge support stands on the table (flat top 24 mm wide along the bars by 100 mm, 160 mm high).',
    ridge_40='A steel ridge support stands on the table (flat top 40 mm wide along the bars by 100 mm, 160 mm high).',
    point_28='A small steel point pedestal stands on the table (flat top 28 mm along the bars by 30 mm, 160 mm high).',
    ridge_ped_40='A steel ridge pedestal stands on the table (flat top 40 mm wide along the bars by 100 mm, 160 mm high).',
)

TASK_A = 'A_stack_pedestal'
# Deployment targets list no cell (lead #152/#184): in-task wrist identification succeeds 27/27 and costs only 3.9-5.0 % (one bar)
# / 7.9-9.4 % (stacks) of the full task time incl. the 20 s hold (scratch/payoff2.jsonl). Task A keeps the COM cells (exploration).
TARGETS = [
    dict(id=TASK_A, settings=['A'], cells=['com_A', 'com_B', 'com_C'], support='riser_246', order='ABC', rules=[]),
    dict(id='edge_A', settings=['S', 'T-new'], cells=[], support='ridge_24', order='A', rules=[], target_objects=['ridge_24']),
    dict(id='edge_B', settings=['S', 'T-new'], cells=[], support='ridge_40', order='B', rules=[], target_objects=['ridge_40']),
    dict(id='stack_ABC', settings=['S', 'T-new'], cells=[], support='point_28', order='ABC', rules=[],
         target_objects=['point_28']),
    dict(id='stack_CBA', settings=['S', 'T-new'], cells=[], support='ridge_ped_40', order='CBA', rules=[],
         target_objects=['ridge_ped_40']),
]
TGT = {t['id']: t for t in TARGETS}


def spec(target): return TGT[TASK_A if target is None else target]


def parameters(seed):
    # Card: COM offset +-45..75 mm along the long axis, random direction; mass 0.55-0.85 kg for all three bars.
    p = {}
    for i, b in enumerate(BARS):
        r = np.random.default_rng(np.random.SeedSequence([seed, 4404, i]))
        sign = float(r.choice([-1., 1.]))
        p['mass_' + b] = float(r.uniform(.55, .85))
        p['com_' + b] = sign * float(r.uniform(*COM_RANGE))
    return p


def transfer_parameters(params, seed):
    # T-new: new bars of the same kind. Every value is resampled from the card range and kept away from the S value
    # (COM by more than the widest support half-width, 20 mm; mass by 0.05 kg), so stale notes do not solve T by luck.
    p = {}
    for i, b in enumerate(BARS):
        r = np.random.default_rng(np.random.SeedSequence([seed, 4405, i]))
        while True:
            m = float(r.uniform(.55, .85))
            if abs(m - params['mass_' + b]) >= .05: break
        while True:
            c = float(r.choice([-1., 1.])) * float(r.uniform(*COM_RANGE))
            if abs(c - params['com_' + b]) >= .025: break
        p['mass_' + b], p['com_' + b] = m, c
    return p


def bar_inertial(mass, com):
    # Uniform thin-walled square tube (SHELL_M) plus a steel slug on the axis at x_s: m_s x_s = mass * com.
    ms = mass - SHELL_M; xs = mass * com / ms
    ixx = SHELL_M * BAR_A ** 2 / 6 + ms * (.048 ** 2) / 6
    iyy = SHELL_M * (BAR_L ** 2 + BAR_A ** 2) / 12 + SHELL_M * com ** 2 + ms * (xs - com) ** 2 + ms * (.048 ** 2) / 12
    return f'{com:.6f} 0 0', f'{ixx:.7f} {iyy:.7f} {iyy:.7f}'


def _vec(v): return ' '.join(f'{x:.6g}' for x in v)


def build(seed, target=None, params=None):
    p = params or parameters(seed); sp = spec(target)
    root = ET.fromstring(portable_scene(seed, 'test')[0]); world = root.find('worldbody')
    for body in list(world.findall('body')):
        if not body.get('name', '').startswith(('left_', 'right_')): world.remove(body)
    for sec in ('contact', 'equality', 'tendon'):
        node = root.find(sec)
        if node is None: node = ET.SubElement(root, sec)
        for c in list(node): node.remove(c)
    root.find('visual/global').attrib.update(offwidth='1280', offheight='720')
    root.set('model', BENCH_ID + ('' if target in (None, TASK_A) else '_' + target))
    for b in BARS:
        pos, inertia = bar_inertial(p['mass_' + b], p['com_' + b])
        body = ET.SubElement(world, 'body', name='bar_' + b, pos=_vec([BAR_X, BAR_Y[b], TABLE_Z + BAR_HA]))
        ET.SubElement(body, 'freejoint', name=f'bar_{b}_joint')
        ET.SubElement(body, 'inertial', mass=f"{p['mass_' + b]:.6f}", pos=pos, diaginertia=inertia)
        # Anodised aluminium tube (mu 0.6, estimated); the black end cap marks the -x end and is part of the tube's length.
        ET.SubElement(body, 'geom', name=f'bar_{b}_tube', type='box', size=_vec([BAR_HL - .004, BAR_HA, BAR_HA]),
                      pos=_vec([.004, 0, 0]), rgba=BAR_RGBA[b], mass='0', friction='.6 .005 .0001')
        ET.SubElement(body, 'geom', name=f'bar_{b}_cap', type='box', size=_vec([.004, BAR_HA, BAR_HA]),
                      pos=_vec([-BAR_HL + .004, 0, 0]), rgba='.08 .08 .09 1', mass='0', friction='.6 .005 .0001')
    sname = sp['support']; hs = SUPPORTS[sname][0]
    sup = ET.SubElement(world, 'body', name=sname, pos=_vec([SUPPORT_XY[0], SUPPORT_XY[1], TABLE_Z + hs[2]]))
    ET.SubElement(sup, 'geom', name=sname + '_block', type='box', size=_vec(hs), rgba='.55 .58 .62 1', friction='.5 .005 .0001')
    ET.SubElement(sup, 'geom', name=sname + '_top', type='box', size=_vec([hs[0], hs[1], .0005]), pos=_vec([0, 0, hs[2] - .0005]),
                  rgba='.82 .84 .86 1', contype='0', conaffinity='0', mass='0')
    world.extend(ET.fromstring('''<w><body name="overview_target" pos="0 -.02 .90"/>
      <camera name="bench_overview" mode="targetbody" target="overview_target" pos=".30 -1.00 1.30" fovy="50"/></w>'''))
    m = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode')); d = mujoco.MjData(m)
    k = ids(m)
    for q in k['q']: d.qpos[q] = HOME
    d.qpos[k['fingers']] = .045; d.ctrl[14:] = .045
    mujoco.mj_forward(m, d)
    return m, d


def _support_name(m):
    return next(n for n in SUPPORTS if mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n) >= 0)


def ids(m):
    arms = [[m.joint(f'{s}_joint{i}').id for i in range(1, 8)] for s in ('left', 'right')]
    sname = _support_name(m); sb = m.body(sname).id
    group = []
    for g in range(m.ngeom):
        bn = m.body(m.geom_bodyid[g]).name
        group.append(-1 if bn.startswith(('left_', 'right_')) else BARS.index(bn[4]) if bn.startswith('bar_') else
                     10 if m.geom_bodyid[g] == sb else 11)
    return dict(q=[m.jnt_qposadr[a].tolist() for a in arms], v=[m.jnt_dofadr[a].tolist() for a in arms],
                fingers=[int(m.jnt_qposadr[m.joint(f'{s}_finger_{f}').id]) for s in ('left', 'right') for f in 'ab'],
                bars=[int(m.body('bar_' + b).id) for b in BARS],
                bar_dof=[int(m.jnt_dofadr[m.body_jntadr[m.body('bar_' + b).id]]) for b in BARS],
                group=group, support=sname, tips=[int(m.site('left_tip').id), int(m.site('right_tip').id)])


def init_state(seed, target, params):
    sp = spec(target)
    return dict(order=[BARS.index(c) for c in sp['order']], hold=0., complete=False, released_ok_s=0.)


def complete(m, d, state, k):
    G = np.asarray(k['group']); n = d.ncon
    a, b = G[d.contact.geom1[:n]], G[d.contact.geom2[:n]]
    order = state['order']
    for j, i in enumerate(order):
        touch = set(b[a == i].tolist()) | set(a[b == i].tolist())
        below = 10 if j == 0 else order[j - 1]
        allowed = {below} | ({order[j + 1]} if j + 1 < len(order) else set())
        if below not in touch or touch - allowed: return False        # robot (-1), table (11) or a wrong bar touches it
        R = d.xmat[k['bars'][i]].reshape(3, 3)
        # Level: long axis within LEVEL of horizontal, and a flat face down. The tube is square and the slug sits on its
        # axis, so a bar balanced level either way up (or on a side face) is physically the same and is accepted.
        if abs(R[2, 0]) > np.sin(LEVEL) or max(abs(R[2, 1]), abs(R[2, 2])) < np.cos(LEVEL): return False
        # Heading (review F1): the long axis must lie along world X, i.e. across the ridge supports (either end first).
        # A bar turned to lie along a 100 mm ridge would otherwise need only the sign of its offset.
        if abs(R[1, 0]) > np.sin(YAW_TOL): return False
        q = k['bar_dof'][i]
        if np.linalg.norm(d.qvel[q:q + 6]) >= STILL: return False
    return True


def step(m, d, state):
    k = state.get('ids') or state.setdefault('ids', ids(m))
    dofs = k['v'][0] + k['v'][1]
    d.qfrc_applied[dofs] = d.qfrc_bias[dofs]
    mujoco.mj_step(m, d)
    ok = complete(m, d, state, k)
    state['complete'] = ok
    state['hold'] = state['hold'] + DT if ok else 0.
    state['released_ok_s'] = max(state['released_ok_s'], state['hold'])


def success(model, data, state, target): return state['hold'] >= HOLD_S


def damage(model, data, state): return {}   # card: no damage flags (a toppled bar costs time only)


ROBOT_TXT = ('Robot: two Panda arms with parallel grippers (jaw opening 0-90 mm); grippers point straight down and turn about '
             'the vertical. Joint torque limits 80 N m on joints 1-4, 12 N m on joints 5-7. Each finger squeezes with at most '
             '40 N. Arm motions stop when the wrist force exceeds the force guard (30 N by default, settable up to 60 N); a wrist '
             'force above 80 N for 0.1 s aborts the episode. Tool reach box: X -300..350 mm, Y -400..400 mm, Z 825..1180 mm '
             '(table top at Z 800 mm); the arms start at about Z 1260 mm, above the reach box. The robot senses joint angles, jaw openings, wrist force/torque and finger forces.')
BARS_TXT = ('Three bars lie on the table side by side, each along the X axis: orange bar A, blue bar B and purple bar C. They '
            'look the same apart from colour: 320 mm long aluminium square tube, 50 x 50 mm, with a black cap on the end '
            'that points to -X. Colours only identify the bars.')
LEVEL_TXT = ('Each bar must lie across the support with its long axis along the X axis (within 10 deg, either end first) '
             'and level within 3.4 deg, lie flat, be still (below 25 mm/s), and touch only the '
             'support or bar directly below it and the bar directly above it; nothing else (no table, no robot). Let go and '
             'keep both hands off; the arrangement must stay like this for 20 s.')


def public_text(target):
    sp = spec(target); scene = f'{BARS_TXT} {SUPPORT_TXT[sp["support"]]} {ROBOT_TXT}'
    if target is None: return scene + ' Goal: explore the objects. The deployment goal is given after reset.'
    o = sp['order']
    if len(o) == 1: goal = f'Goal: set bar {o} on the support so that it rests there on its own. '
    else: goal = (f'Goal: stack the bars on the support from bottom to top in the order {", ".join(o)} (bar {o[0]} on the support, '
                  f'bar {o[1]} on bar {o[0]}, bar {o[2]} on bar {o[1]}), each lying along the one below. ')
    return f'{scene} {goal}{LEVEL_TXT}'


OBJECT_BOX = ((-.75, -.85, .70), (.75, .85, 1.60))


def wrist_wrench(model, data):
    # No flex in this scene, so the native wrist F/T sees every contact.
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


# ---------------------------------------------------------------- privileged reference controller
YAW_L = np.pi / 2            # left hand: tool down, jaw axis along world Y (across the bars)
GZ = .010                    # tip height above the bar's centre line when grasping: pad bottoms 15 mm above the bar's bottom
CARRY_Z = 1.15
GRIP = .012                  # finger target on a 50 mm bar: 1200 N/m x 13 mm = about 15 N per finger (gentle grasp: holds the
                             # worst 0.85 kg x 75 mm moment with 1.1 deg tilt; a full close at 30 N gave -2.4 mm pad bursts at set-down)
RETREAT_ZMAX = 1.175       # stay inside the 1.18 m reach box
PARK_L = np.array([-.20, -.05, 1.15])
PARK_R = np.array([.33, .22, 1.10])


def r_down(yaw): return Rotation.from_euler('z', yaw).as_matrix() @ DOWN


def ik(m, scratch, k, arm, target, q0, R, iters=15):
    q, v, tip = k['q'][arm], k['v'][arm], k['tips'][arm]
    scratch.qpos[q] = q0
    jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv)); lo, hi = m.jnt_range[[m.dof_jntid[i] for i in v]].T
    for _ in range(iters):
        mujoco.mj_kinematics(m, scratch); mujoco.mj_comPos(m, scratch)
        err = np.r_[target - scratch.site_xpos[tip], .5 * Rotation.from_matrix(R @ scratch.site_xmat[tip].reshape(3, 3).T).as_rotvec()]
        if np.linalg.norm(err) < 1e-5: break
        mujoco.mj_jacSite(m, scratch, jp, jr, tip)
        J = np.vstack([jp[:, v], .5 * jr[:, v]])
        scratch.qpos[q] = np.clip(scratch.qpos[q] + np.clip(J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), err), -.08, .08), lo + .01, hi - .01)
    return scratch.qpos[q].copy()


class Driver:
    # Scripted Cartesian plan per arm tracked by joint PD (toy_drawer / bike_pump pattern); records the ctrl rows.
    def __init__(self, seed, target, params):
        self.m, self.d = build(seed, target, params); self.st = init_state(seed, target, params); self.k = ids(self.m)
        self.scratch = mujoco.MjData(self.m); self.acts = []; self.goals = [[], []]
        self.pos = [self.d.site_xpos[t].copy() for t in self.k['tips']]
        self.qt = [self.d.qpos[self.k['q'][a]].copy() for a in (0, 1)]
        self.jaw = [.045, .045]
        self.R = [self.d.site_xmat[t].reshape(3, 3).copy() for t in self.k['tips']]
        self.Rt = [r_down(YAW_L), self.R[1].copy()]

    def tick(self):
        m, d, k = self.m, self.d, self.k; u = np.zeros(m.nu)
        for arm in (0, 1):
            u[7 * arm:7 * arm + 7] = np.clip(KP * (self.qt[arm] - d.qpos[k['q'][arm]]) - KD * d.qvel[k['v'][arm]], -LIM, LIM)
        u[14:16] = self.jaw[0]; u[16:18] = self.jaw[1]
        d.ctrl[:] = u; self.acts.append(u.copy()); step(m, d, self.st)

    def move(self, arm, goal=None, jaw=None, n=300):
        p0 = self.pos[arm].copy(); g = p0 if goal is None else np.asarray(goal, float)
        j0 = self.jaw[arm]; j1 = j0 if jaw is None else jaw
        sl = Slerp([0., 1.], Rotation.from_matrix([self.R[arm], self.Rt[arm]]))
        for i in range(n):
            f = min(1., (i + 1) / max(1, n - 40)); f = f * f * (3 - 2 * f)
            if i % 5 == 0 or i == n - 1:
                self.qt[arm] = ik(self.m, self.scratch, self.k, arm, p0 + (g - p0) * f, self.qt[arm], sl(f).as_matrix())
            self.jaw[arm] = j0 + (j1 - j0) * f
            self.tick()
        self.pos[arm] = g.copy(); self.R[arm] = self.Rt[arm].copy(); self.goals[arm].append(g.tolist())

    def jmove(self, arm, goal, n=600):
        q0 = self.qt[arm].copy(); q1 = q0.copy()
        for _ in range(20): q1 = ik(self.m, self.scratch, self.k, arm, np.asarray(goal, float), q1, self.Rt[arm])
        for i in range(n):
            f = min(1., (i + 1) / max(1, n - 40)); f = f * f * (3 - 2 * f)
            self.qt[arm] = q0 + (q1 - q0) * f
            self.tick()
        self.pos[arm] = np.asarray(goal, float).copy(); self.R[arm] = self.Rt[arm].copy(); self.goals[arm].append(list(goal))

    def wait(self, n):
        for _ in range(n): self.tick()


def ref_park_right(D): D.jmove(1, PARK_R, 500)


def ref_pick(D, i, first=False):
    # Travel at carry height, then straight down: the open fingers never sweep through a placed bar.
    c = D.d.xpos[D.k['bars'][i]].copy(); g = c + [0, 0, GZ]
    D.move(0, [g[0], g[1], CARRY_Z], .045, 700 if first else 500)
    D.move(0, g + [0, 0, .05], .045, 400)
    D.move(0, g, .045, 250)
    D.move(0, None, GRIP, 150); D.wait(100)
    D.move(0, [g[0], g[1], CARRY_Z], GRIP, 450)


def ref_place(D, j, i, com_xy=None):
    # Privileged: the bar's true COM relative to the tool tip after the grasp (com_xy=None) or an assumed offset.
    d, k = D.d, D.k; tip = d.site_xpos[k['tips'][0]]; b = k['bars'][i]
    rel = (d.xipos[b] - tip)[:2] if com_xy is None else com_xy
    top = TABLE_Z + 2 * SUPPORTS[k['support']][0][2]
    dz = tip[2] - d.xpos[b][2]
    tx, ty = SUPPORT_XY - rel
    zp = top + j * BAR_A + BAR_HA + dz
    D.move(0, [tx, ty, CARRY_Z], GRIP, 500)
    D.move(0, [tx, ty, zp + .012], GRIP, 400)
    D.move(0, [tx, ty, zp + .001], GRIP, 350); D.wait(150)     # slow set-down (about 30 mm/s): no impact spike
    D.move(0, None, .045, 200); D.wait(100)
    D.move(0, [tx, ty, min(zp + .08, RETREAT_ZMAX)], .045, 300)
    D.move(0, [tx, ty, CARRY_Z], .045, 150)


def reference(seed, target, params, com_guess=None):
    # com_guess: None = privileged true COM (the reference); a dict bar -> assumed offset is used by the payoff study.
    p = params or parameters(seed); sp = spec(target)
    D = Driver(seed, target, p)
    ref_park_right(D)
    for j, c in enumerate(sp['order']):
        i = BARS.index(c)
        ref_pick(D, i, first=(j == 0))
        ref_place(D, j, i, None if com_guess is None else np.array([com_guess[c], 0.]))
    D.move(0, PARK_L, .045, 500)
    D.wait(int(HOLD_S / DT) + 500)
    LAST_GOALS[:] = D.goals
    return np.array(D.acts)


LAST_GOALS = [[], []]   # commanded tool goals of the last reference run (audit of the reach box)


# ---------------------------------------------------------------- probes (lift a bar at its middle and hold it still)
PROBE_HOLD = 600


def probe_lift(bar):
    def run(seed, params):
        D = Driver(seed, None, params); i = BARS.index(bar)
        ref_park_right(D)
        c = D.d.xpos[D.k['bars'][i]].copy(); g = c + [0, 0, GZ]
        D.move(0, g + [0, 0, .12], .045, 700); D.wait(200)
        D.marks = [len(D.acts)]                                           # tare window ends here (hand free, still)
        D.move(0, g, .045, 350); D.move(0, None, GRIP, 150); D.wait(100)
        D.move(0, g + [0, 0, .10], GRIP, 400); D.wait(PROBE_HOLD)
        return np.array(D.acts)
    return run


def _probe_len():
    # Tare index of the lift probe: right park 500 + approach 700 + wait 200 ticks.
    return 500 + 700 + 200


def _tared(tr):
    w = np.asarray(tr['wrench']); t0 = _probe_len()
    if len(w) < t0 + PROBE_HOLD: return w[:, :6] * 0
    return w[:, :6] - w[t0 - 100:t0, :6].mean(0)


def stat_torque(tr):
    # Mean tared left-wrist torque about world Y over the last 400 ticks of the hold: = -weight x COM offset along world X
    # (bar along X, capped end at -X; measured -0.39 N m for +60 mm and +0.36 N m for -60 mm at 0.6-0.85 kg).
    w = _tared(tr); return float(w[-400:, 4].mean())


def stat_weight(tr):
    # Mean tared left-wrist world Fz over the same window: the world-frame wrist force reads +weight of the held bar (N).
    w = _tared(tr); return float(w[-400:, 2].mean())


PROBES = {f'lift_{b}': probe_lift(b) for b in BARS}
HIDDEN_CELLS = (
    [dict(name='com_' + b, type='P', object='bar_' + b, contrast=(-.06, .06), probe='lift_' + b, channel='force',
          statistic=stat_torque) for b in BARS] +
    [dict(name='mass_' + b, type='P', object='bar_' + b, contrast=(.60, .80), probe='lift_' + b, channel='force',
          statistic=stat_weight, note='intermediate (card): converts the torque into an offset; scored in no target')
     for b in BARS])
RULE_TESTS = []   # no rules: native rigid contact only


def render(model, data, path, camera='bench_overview'):
    r = mujoco.Renderer(model, 720, 1280); r.update_scene(data, camera=camera); img = r.render(); r.close()
    chunk = lambda t, b: struct.pack('>I', len(b)) + t + b + struct.pack('>I', zlib.crc32(t + b) & 0xffffffff)
    raw = b''.join(b'\x00' + row.tobytes() for row in img)
    Path(path).write_bytes(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', 1280, 720, 8, 2, 0, 0, 0))
                           + chunk(b'IDAT', zlib.compress(raw, 6)) + chunk(b'IEND', b''))
