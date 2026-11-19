"""recyclekit: the battery-recycle bench for yam_arena (Isaac Lab-Arena / PhysX), plus its runtime hooks.

The bench (9 V battery, two AA cells, the jar with eight loose used cells, the tape strip) is authored as ONE
custom-spawned assembly of PhysX bodies and joints, because yam_arena's task-file primitives are single rigid
bodies. Geometry and reset poses come from ``layout.py``; the scorer is the pure ``scoring.py``.

Tape: 20 rigid 3 x 19 mm segments in a chain. D6 joints between neighbours leave the bend about the width axis free
(hard limit +-150 deg) and allow +-20 deg of twist; the in-plane DOFs are locked. No bending stiffness: drives on
these non-articulation joints are not honoured by PhysX here, and an explicit spring is unstable at the segments'
inertia (README fidelity).

Adhesive (a sim rule standing in for the tape's pressure-sensitive adhesive; disclosed in the README):
* every physics step, PhysX contact reports between each tape segment and every other body (battery, cells, jar,
  table, robot fingers) are read; a contact counts as "on the adhesive face" when the body pushes the segment from
  its adhesive (+z) side, i.e. the contact normal points along the segment's -z within 45 deg (``BOND_FACE_COS``).
  The side is taken from the normal, not from the contact point: the strip is 0.4 mm thick, and a body pressing its
  plain back can have a contact point just past the mid-plane;
* a segment sticks to a body when the normal force pressing its adhesive face onto that body reaches
  ``BOND_PRESS_N`` (0.5 N per 3 mm segment, a deliberate press). Touching or resting does not stick it, and nothing
  is triggered by proximity;
* sticking creates a PhysX breakable fixed joint at the segment's CURRENT pose relative to the body (no snapping,
  no pose change), with break force ``BOND_BREAK_N`` (the tape's peel strength) and the matching break torque;
* a bond whose segment has drifted more than 1.5 mm from where it stuck (i.e. PhysX broke the joint) is removed;
  the segment sticks again only if it is pressed on again.

Runtime hooks are installed on import; nothing in yam_arena is edited (same hook pattern as earlier task kits):
after every physics step -> adhesive; after every control step -> scorer (``score()`` reads the cached result).
The scripted demo READS poses through ``bench_state()``; nothing here writes a body pose, a bond flag or a score
during a trial. ``Runtime.restore_authored`` puts every bench body back at its reset pose and removes all bonds
before yam_arena's own reset (environment reset only, never during a trial).
"""
from __future__ import annotations

import math
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from yam_arena.task_file import Box  # noqa: E402

import layout as L  # noqa: E402
from scoring import AA_NAMES, Scorer, tape_state  # noqa: E402

PHYSICS_DT = float(os.environ.get('RECYCLE_PHYSICS_DT', str(1 / 960)))
CONTACT_OFFSET = float(os.environ.get('RECYCLE_CONTACT_OFFSET', '0.0004'))
BOND_MODE = os.environ.get('RECYCLE_BOND_MODE', 'joint')  # 'joint' (breakable fixed joint) | 'drive' | 'off'
USD_SYNC = os.environ.get('RECYCLE_USD_SYNC', '1') == '1'

COL = dict(case=(.06, .06, .06), copper=(.72, .42, .16), snap=(.78, .78, .80), aa_body=(.08, .08, .08),
           aa_top=(.72, .42, .16), cell_b=(.15, .35, .75), cell_c=(.85, .70, .10), tape=(.85, .08, .08),
           jar=(.85, .92, .95), label=(.95, .95, .92), label_green=(.15, .55, .25), mat=(.80, .45, .30))


@dataclass
class Body:
    name: str
    pos: np.ndarray
    rot: np.ndarray
    mass: float
    friction: float
    boxes: list = field(default_factory=list)      # (centre, size, colour)
    cylinders: list = field(default_factory=list)  # (centre, radius, height, colour, axis)
    visuals: list = field(default_factory=list)    # (centre, size, colour)
    visual_cyl: list = field(default_factory=list)  # (centre, radius, height, colour, axis)
    static: bool = False
    opacity: float = 1.0
    damping: tuple = (0.0, 0.5)


def bench_spec():
    """Every body of the bench in world coordinates (no simulator)."""
    bodies = {}
    sx, sy, sz = L.NINE_V_SIZE
    nv = Body('nine_v', L.NINE_V_POS.copy(), L.NINE_V_ROT.copy(), L.NINE_V_MASS, L.MU['battery'])
    nv.boxes.append(((0, 0, 0), (sx, sy, sz), COL['case']))
    for name in ('positive', 'negative'):
        nv.cylinders.append(((L.SNAP_X[name], 0, sz / 2 + L.SNAP_HEIGHT / 2), L.SNAP_RADIUS[name], L.SNAP_HEIGHT,
                             COL['snap'], 'Z'))
    # Duracell-style copper band near the top and printed labels (visual only, 0.1 mm proud)
    nv.visuals += [((0, 0, sz / 2 - .006), (sx + .0002, sy + .0002, .012), COL['copper']),
                   ((0, sy / 2 + .0001, -.004), (.018, .0002, .016), COL['label'])]
    bodies['nine_v'] = nv
    for name, (p, r) in L.AA_CELLS.items():
        b = Body(name, p.copy(), r.copy(), L.AA_MASS, L.MU['cell'])
        b.cylinders.append(((0, 0, 0), L.AA_RADIUS, L.AA_LENGTH - .0015, COL['aa_body'], 'Z'))
        b.cylinders.append(((0, 0, L.AA_LENGTH / 2 - .00075), .0026, .0015, COL['snap'], 'Z'))  # + button
        b.visual_cyl.append(((0, 0, L.AA_LENGTH / 2 - .009), L.AA_RADIUS + .0001, .012, COL['aa_top'], 'Z'))
        bodies[name] = b
    for i, (name, (kind, p, r)) in enumerate(L.jar_contents().items()):
        ln, rad, m = (L.AA_LENGTH, L.AA_RADIUS, L.AA_MASS) if kind == 'AA' else (L.AAA_LENGTH, L.AAA_RADIUS, L.AAA_MASS)
        b = Body(name, p.copy(), r.copy(), m, L.MU['cell'], damping=(0.05, 0.5))
        b.cylinders.append(((0, 0, 0), rad, ln, COL['cell_b'] if i % 2 else COL['cell_c'], 'Z'))
        bodies[name] = b
    # jar: static wall slabs, floor and neck, clear PET
    jar = Body('jar', np.array([L.JAR_CENTER[0], L.JAR_CENTER[1], L.TOP]), np.eye(3), 0., L.MU['jar'], static=True,
               opacity=0.25)
    n = L.JAR_SEGMENTS
    wall_r = L.JAR_INNER_R + L.JAR_WALL / 2
    chord = 2 * (L.JAR_OUTER_R) * math.tan(math.pi / n) + .0005
    body_h = L.JAR_HEIGHT - L.JAR_NECK_H
    for k in range(n):
        a = 2 * math.pi * k / n
        jar.boxes.append(((wall_r * math.cos(a), wall_r * math.sin(a), body_h / 2), (L.JAR_WALL, chord, body_h),
                          COL['jar'], a))
        # neck: shoulder ring narrowing to the 89 mm mouth
        nr = (L.JAR_MOUTH_R + L.JAR_OUTER_R) / 2
        jar.boxes.append(((nr * math.cos(a), nr * math.sin(a), body_h + L.JAR_NECK_H / 2),
                          (L.JAR_OUTER_R - L.JAR_MOUTH_R, chord, L.JAR_NECK_H), COL['jar'], a))
    jar.cylinders.append(((0, 0, L.JAR_FLOOR / 2), L.JAR_INNER_R + .0005, L.JAR_FLOOR, COL['jar'], 'Z'))
    # paper label "USED BATTERIES" on the front (robot side), visual only
    jar.visuals.append(((-(L.JAR_OUTER_R + .0003), 0, .06), (.0004, .06, .045), COL['label']))
    jar.visuals.append(((-(L.JAR_OUTER_R + .0005), 0, .06), (.0004, .05, .008), COL['label_green']))
    bodies['jar'] = jar
    mat = Body('mat', L.MAT_CENTER.copy(), np.eye(3), 0., L.MU['mat'], static=True)
    mat.boxes.append(((0, 0, 0), tuple(L.MAT_SIZE), COL['mat']))
    bodies['mat'] = mat
    segs = []
    for i, (p, r) in enumerate(L.tape_segment_poses()):
        flap = i in L.FLAP_SEGMENTS  # carries the folded-under 3 mm end (double layer)
        b = Body(f'tape_{i:02d}', p, r, L.TAPE_SEG_MASS * (2 if flap else 1), L.MU['tape_back'], damping=(0.2, 0.2))
        b.boxes.append(((0, 0, 0), (L.TAPE_SEG_LEN, L.TAPE_WIDTH, L.TAPE_THICKNESS), COL['tape']))
        bodies[b.name] = b
        segs.append(b.name)
    return dict(bodies=bodies, tape=segs)


class Bench(Box):
    """The bench, as one task-file object (subclasses Box only to pass yam_arena's checks)."""


_SPEC: dict = {}


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])


def _spawn_bench(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade
    from yam_arena.station import linear_rgb

    spec = bench_spec()
    _SPEC.update(spec)
    _SPEC['prim_path'] = prim_path
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    ox, oy, oz, ow = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(ow, ox, oy, oz))
    looks, mats = {}, {}

    def look(color, opacity=1.0):
        key = (color, opacity)
        if key not in looks:
            path = f'{prim_path}/Looks/c{len(looks)}'
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=.4, opacity=opacity)
            vis.func(path, vis)
            looks[key] = path
        return looks[key]

    def material(friction):
        if friction not in mats:
            path = f'{prim_path}/Looks/phys{len(mats)}'
            pm = sim_utils.RigidBodyMaterialCfg(static_friction=friction, dynamic_friction=friction)
            pm.func(path, pm)
            mats[friction] = path
        return mats[friction]

    def xform(path, pos, rot):
        x = UsdGeom.Xform.Define(stage, path)
        x.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float, pos)))
        q = L.mat2quat_xyzw(rot)
        x.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(q[3]), *map(float, q[:3])))
        return x

    def collide(prim, body, color):
        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(CONTACT_OFFSET)
        cp.CreateRestOffsetAttr(0.)
        bind_physics_material(str(prim.GetPath()), material(body.friction), stage=stage)
        bind_visual_material(str(prim.GetPath()), look(color, body.opacity), stage=stage)

    def rigid(path, body):
        prim = stage.GetPrimAtPath(path)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(body.mass))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(.5)
        rb.CreateSolverPositionIterationCountAttr(32)
        rb.CreateSolverVelocityIterationCountAttr(4)
        rb.CreateLinearDampingAttr(float(body.damping[0]))
        rb.CreateAngularDampingAttr(float(body.damping[1]))
        rb.CreateSleepThresholdAttr(0.)  # never sleep: contact reports and the adhesive need awake bodies
        rb.CreateStabilizationThresholdAttr(0.)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)

    for b in spec['bodies'].values():
        path = f'{prim_path}/{b.name}'
        xform(path, b.pos, b.rot)
        if not b.static:
            rigid(path, b)
        for i, item in enumerate(b.boxes):
            c, s, col = item[:3]
            cube = UsdGeom.Cube.Define(stage, f'{path}/box_{i}')
            cube.CreateSizeAttr(1.)
            cube.AddTranslateOp().Set(Gf.Vec3d(*map(float, c)))
            if len(item) > 3:  # yaw about the body z
                q = L.mat2quat_xyzw(_rot_z(item[3]))
                cube.AddOrientOp().Set(Gf.Quatf(float(q[3]), *map(float, q[:3])))
            cube.AddScaleOp().Set(Gf.Vec3f(*map(float, s)))
            collide(cube.GetPrim(), b, col)
        for i, (c, r, h, col, axis) in enumerate(b.cylinders):
            g = UsdGeom.Cylinder.Define(stage, f'{path}/cyl_{i}')
            g.CreateAxisAttr(axis)
            g.CreateRadiusAttr(float(r))
            g.CreateHeightAttr(float(h))
            g.AddTranslateOp().Set(Gf.Vec3d(*map(float, c)))
            collide(g.GetPrim(), b, col)
        for i, (c, s, col) in enumerate(b.visuals):
            cube = UsdGeom.Cube.Define(stage, f'{path}/vis_{i}')
            cube.CreateSizeAttr(1.)
            cube.AddTranslateOp().Set(Gf.Vec3d(*map(float, c)))
            cube.AddScaleOp().Set(Gf.Vec3f(*map(float, s)))
            bind_visual_material(str(cube.GetPrim().GetPath()), look(col), stage=stage)
        for i, (c, r, h, col, axis) in enumerate(b.visual_cyl):
            g = UsdGeom.Cylinder.Define(stage, f'{path}/vcyl_{i}')
            g.CreateAxisAttr(axis)
            g.CreateRadiusAttr(float(r))
            g.CreateHeightAttr(float(h))
            g.AddTranslateOp().Set(Gf.Vec3d(*map(float, c)))
            bind_visual_material(str(g.GetPrim().GetPath()), look(col), stage=stage)
    # tape chain joints
    segs = spec['tape']
    for i in range(len(segs) - 1):
        a, b = spec['bodies'][segs[i]], spec['bodies'][segs[i + 1]]
        j = UsdPhysics.Joint.Define(stage, f'{prim_path}/{segs[i + 1]}/hinge')
        j.CreateBody0Rel().SetTargets([f'{prim_path}/{segs[i]}'])
        j.CreateBody1Rel().SetTargets([f'{prim_path}/{segs[i + 1]}'])
        j.CreateLocalPos0Attr(Gf.Vec3f(L.TAPE_SEG_LEN / 2, 0, 0))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(-L.TAPE_SEG_LEN / 2, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateExcludeFromArticulationAttr(True)
        prim = j.GetPrim()
        for dof in ('transX', 'transY', 'transZ', 'rotZ'):
            lim = UsdPhysics.LimitAPI.Apply(prim, dof)
            lim.CreateLowAttr(1.)
            lim.CreateHighAttr(-1.)
        for dof, lo in (('rotX', L.TAPE_TWIST_LIMIT_DEG), ('rotY', L.TAPE_BEND_LIMIT_DEG)):
            lim = UsdPhysics.LimitAPI.Apply(prim, dof)
            lim.CreateLowAttr(-float(lo))
            lim.CreateHighAttr(float(lo))
        _ = (a, b, UsdShade)
    if BOND_MODE == 'drive':
        _glue_joints(stage, prim_path, spec, root)
    return root.GetPrim()


GLUE_ANCHORS = (-0.0012, 0.0012)   # m along each segment: two glue points per segment
GLUE_K = float(os.environ.get('RECYCLE_GLUE_K', '1e4'))   # N/m implicit drive stiffness (0.4 mm at 4 N)
GLUE_C = float(os.environ.get('RECYCLE_GLUE_C', '10'))    # N s/m


def _glue_joints(stage, prim_path, spec, root):
    """Adhesive as pre-authored, initially inert D6 joints (drive mode): for every tape segment, two glue points,
    each a D6 joint to the 9 V (and, for the two flap segments, to the world = the silicone mat) with every DOF free
    and linear drives whose force cap is 0 until the segment is pressed on (then the targets are set to the point's
    current position in the 9 V's frame and the cap to the bond strength). Collision between the jointed bodies stays
    enabled. Nothing is pulled anywhere: a drive only holds a stuck point where it stuck, up to the cap."""
    from pxr import Gf, Sdf, UsdPhysics
    world = np.array(root.ComputeLocalToWorldTransform(0).ExtractTranslation(), float)
    _SPEC['glue'] = {}
    for i, seg in enumerate(spec['tape']):
        b = spec['bodies'][seg]
        targets = [('nine_v', f'{prim_path}/nine_v')] + ([('mat', None)] if i in L.FLAP_SEGMENTS else [])
        for name, body0 in targets:
            for ai, ax in enumerate(GLUE_ANCHORS):
                path = f'{prim_path}/{seg}/glue_{name}_{ai}'
                j = UsdPhysics.Joint.Define(stage, path)
                if body0:
                    j.CreateBody0Rel().SetTargets([Sdf.Path(body0)])
                    j.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
                else:  # world: the glue point's reset position
                    p0 = b.pos + b.rot @ np.array([ax, 0, 0]) + world
                    j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, p0)))
                j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
                j.CreateBody1Rel().SetTargets([Sdf.Path(f'{prim_path}/{seg}')])
                j.CreateLocalPos1Attr(Gf.Vec3f(float(ax), 0, 0))
                j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
                j.CreateCollisionEnabledAttr(True)
                j.CreateExcludeFromArticulationAttr(True)
                prim = j.GetPrim()
                for dof in ('transX', 'transY', 'transZ'):
                    d = UsdPhysics.DriveAPI.Apply(prim, dof)
                    d.CreateTypeAttr('force')
                    d.CreateStiffnessAttr(GLUE_K)
                    d.CreateDampingAttr(GLUE_C)
                    d.CreateMaxForceAttr(0.)
                    d.CreateTargetPositionAttr(0.)
                _SPEC['glue'].setdefault((i, name), []).append((path, ax))


def _arena_object(spec: Bench):
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class BenchSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_bench)

    return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=BenchSpawnerCfg(),
                  initial_pose=Pose(position_xyz=(0., 0., 0.), rotation_xyzw=(0., 0., 0., 1.)))


# ---------------------------------------------------------------------------------------------- runtime state
STATE: dict = dict(score=0., success=False, veto=None, steps=0, detail={})


class Runtime:
    """Reads the bench through PhysX tensor views; runs the adhesive and the scorer."""

    def __init__(self, environment):
        import omni.physics.tensors as tensors
        import torch
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdUtils
        import carb
        # PhysX contact reports (adhesive press force, robot touching a battery): Isaac Lab's physics set-up turns
        # contact processing off; the CPU pipeline then reports no contacts at all
        carb.settings.get_settings().set_bool('/physics/disableContactProcessing', False)
        self.base = environment.unwrapped
        self.env = environment
        self.torch = torch
        self.stage = get_current_stage()
        stage_id = UsdUtils.StageCache.Get().GetId(self.stage).ToLongInt()
        self.simview = tensors.create_simulation_view('torch', stage_id=stage_id)
        self.simview.set_subspace_roots('/')
        self.root = '/World/envs/env_0/bench'
        self.names = [n for n, b in _SPEC['bodies'].items() if not b.static]
        self.tape = list(_SPEC['tape'])
        self.view = self.simview.create_rigid_body_view([f'{self.root}/{n}' for n in self.names])
        self.index = {n: i for i, n in enumerate(self.names)}
        self.origin = self.base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        robot_links, self.robot_link_names = [], []
        for arm in ('left_robot', 'right_robot'):
            view = self.base.scene[arm].root_view
            paths = view.link_paths
            paths = list(paths[0] if paths and isinstance(paths[0], (list, tuple)) else paths)
            robot_links += paths
            self.robot_link_names += [f'{arm}:{p.rsplit("/", 1)[1]}' for p in paths]
        self.robot_links = robot_links
        # bodies a tape segment can stick to: every other bench body + robot links + table (+ jar, static)
        others = [n for n in self.names if n not in self.tape]
        self.stick_names = others + list(self.robot_link_names) + ['table', 'jar']
        self.stick_paths = ([f'{self.root}/{n}' for n in others] + robot_links
                            + ['/World/envs/env_0/table', f'{self.root}/jar'])
        self.stick_static = {'table', 'jar'}
        if os.environ.get('RECYCLE_STATIC_FILTERS', '0') != '1':  # static colliders as filters: see probe notes
            keep = [i for i, n in enumerate(self.stick_names) if n not in self.stick_static]
            self.stick_names = [self.stick_names[i] for i in keep]
            self.stick_paths = [self.stick_paths[i] for i in keep]
        self.tape_contacts = self.simview.create_rigid_contact_view(
            [f'{self.root}/{n}' for n in self.tape], filter_patterns=[self.stick_paths] * len(self.tape),
            max_contact_data_count=8192)
        self.tape_net = self.simview.create_rigid_contact_view([f'{self.root}/{n}' for n in self.tape])
        watched = ['nine_v'] + list(AA_NAMES)
        self.robot_contacts = self.simview.create_rigid_contact_view(
            [f'{self.root}/{n}' for n in watched], filter_patterns=[robot_links] * len(watched),
            max_contact_data_count=2048)
        self.watched = watched
        self.bond_seq = 0
        self.reset()

    # ---- reset
    def reset(self):
        self.bonds = {}            # (segment index, stick name) -> dict(path, rel_pos, rel_rot, step)
        self.bond_events = []
        self.scorer = Scorer()
        self.t = 0.
        self.n_phys = 0
        self.hook_s = 0.
        self.press_max = {}        # (seg, stick) -> max adhesive-face normal force over the control step
        self.poses = self.read()
        self.nonfinite = 0
        # the folded-under end flaps cling to the silicone mat (the strip's reset state; a light, breakable hold)
        if BOND_MODE == 'drive':
            self._glue_reset = {}
            for s in L.FLAP_SEGMENTS:
                for path, pw in self.glue_points(s, 'mat', self.poses):
                    self._glue_reset[path] = pw
        for s in L.FLAP_SEGMENTS:
            self.make_bond(s, -1 - s, self.poses, break_n=L.LINER_BREAK_N, target_override=('mat', None))
        self.bond_events = []
        STATE.update(score=0., success=False, veto=None, steps=0, detail={})

    def check_glue(self, poses):
        for (s, k), b in list(self.bonds.items()):
            name = b['name']
            cur = dict(self.glue_points(s, name, poses))
            if name == 'mat':
                ref = {path: np.array(self._glue_reset[path]) for path in cur}
            else:
                ref = b['glue']
            drift = max(float(np.linalg.norm(cur[p] - ref[p])) for p in cur)
            if drift > L.BOND_DRIFT_M:
                self.set_glue(s, name, 0.)
                del self.bonds[(s, k)]
                self.bond_events.append(dict(t=round(self.t, 4), event='let go', segment=s, to=name,
                                             drift_mm=round(drift * 1000, 2)))

    def remove_bonds(self):
        if BOND_MODE == 'drive':
            for (s, k), b in list(self.bonds.items()):
                self.set_glue(s, b['name'], 0.)
            self.bonds = {}
            return
        for key, b in list(self.bonds.items()):
            if self.stage.GetPrimAtPath(b['path']).IsValid():
                self.stage.RemovePrim(b['path'])
        self.bonds = {}

    def restore_authored(self):
        """Environment reset (never during a trial): remove all bonds, put every bench body back at its reset pose
        with zero velocity."""
        torch = self.torch
        self.remove_bonds()
        tf = self.view.get_transforms().clone()
        for n, i in self.index.items():
            b = _SPEC['bodies'][n]
            tf[i, :3] = torch.tensor(np.asarray(b.pos, float) + self.origin, dtype=tf.dtype)
            tf[i, 3:7] = torch.tensor(L.mat2quat_xyzw(b.rot), dtype=tf.dtype)
        idx = torch.arange(len(self.index), dtype=torch.int32, device=tf.device)
        self.view.set_transforms(tf, idx)
        self.view.set_velocities(torch.zeros_like(self.view.get_velocities()), idx)

    # ---- state
    def read(self):
        tf = self.view.get_transforms().detach().cpu().numpy().astype(float)
        if not np.isfinite(tf).all():
            self.nonfinite = getattr(self, 'nonfinite', 0) + 1
        return {n: (tf[i, :3] - self.origin, L.quat_xyzw2mat(tf[i, 3:7])) for n, i in self.index.items()}

    def speeds(self):
        v = self.view.get_velocities().detach().cpu().numpy().astype(float)
        return {n: float(np.linalg.norm(v[i, :3])) for n, i in self.index.items()}

    def stick_pose(self, k, poses):
        """World pose (env frame) of stick target k (index into stick_names); static targets return None."""
        name = self.stick_names[k]
        if name in poses:
            return poses[name]
        if name in self.stick_static:
            return None
        arm, link = name.split(':')
        robot = self.base.scene[arm]
        i = list(robot.body_names).index(link)
        p = robot.data.body_link_pose_w[0].detach().cpu().numpy().astype(float)[i]
        return p[:3] - self.origin, L.quat_xyzw2mat(p[3:7])

    # ---- probe fixtures (dev only: never used by the task, the scorer or the demo)
    fixture = None

    def probe_force(self, forces):
        """World forces (N) at body origins for one physics step, {body: force}."""
        torch = self.torch
        n = len(self.names)
        tf = self.view.get_transforms()
        f = torch.zeros((n, 3), device=tf.device)
        for name, force in forces.items():
            f[self.index[name]] = torch.tensor(np.asarray(force, float), dtype=f.dtype)
        idx = torch.arange(n, dtype=torch.int32, device=f.device)
        self.view.apply_forces_and_torques_at_position(f, torch.zeros_like(f), tf[:, :3].clone(), idx, True)

    def probe_place(self, name, pos, rot):
        torch = self.torch
        i = self.index[name]
        tf = self.view.get_transforms().clone()
        tf[i, :3] = torch.tensor(np.asarray(pos, float) + self.origin, dtype=tf.dtype)
        tf[i, 3:7] = torch.tensor(L.mat2quat_xyzw(rot), dtype=tf.dtype)
        idx = torch.tensor([i], dtype=torch.int32, device=tf.device)
        self.view.set_transforms(tf, idx)
        v = self.view.get_velocities().clone()
        v[i] = 0
        self.view.set_velocities(v, idx)

    def contact_debug(self, dt):
        """Raw contact numbers (diagnostics)."""
        out = dict(filter_count=int(getattr(self.tape_contacts, 'filter_count', -1)),
                   sensor_count=int(getattr(self.tape_contacts, 'sensor_count', -1)))
        fm = self.tape_contacts.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
        out['force_matrix_max'] = float(np.abs(fm).max()) if fm.size else 0.
        nz = np.argwhere(np.linalg.norm(fm, axis=-1) > 1e-6)
        out['force_matrix_pairs'] = [[self.tape[int(a)], self.stick_names[int(b)],
                                      np.round(fm[a, b], 3).tolist()] for a, b in nz[:12]]
        f, pts, nrm, sep, cnt, st = self.tape_contacts.get_contact_data(dt=dt)
        out['contact_count_total'] = int(cnt.sum())
        out['contact_force_max'] = float(f.max()) if f.numel() else 0.
        net = self.tape_net.get_net_contact_forces(dt=dt).detach().cpu().numpy()
        out['net'] = {self.tape[i]: np.round(net[i], 3).tolist() for i in range(len(self.tape)) if np.linalg.norm(net[i]) > 1e-4}
        return out

    # ---- adhesive
    def physics(self, dt):
        t0 = time.perf_counter()
        self.t += dt
        self.n_phys += 1
        poses = self.read()
        self.poses = poses
        if self.fixture is not None:  # probe fixtures only (never in the task or the demo)
            self.fixture(self, poses)
        if BOND_MODE != 'off':
            self.adhesive(dt, poses)
        self.hook_s += time.perf_counter() - t0

    def adhesive(self, dt, poses):
        forces, points, normals, seps, counts, starts = self.tape_contacts.get_contact_data(dt=dt)
        counts = counts.detach().cpu().numpy()
        if not counts.any():
            self.check_drift(poses)
            return
        starts = starts.detach().cpu().numpy()
        forces = forces.detach().cpu().numpy().reshape(-1)
        normals = normals.detach().cpu().numpy().reshape(-1, 3)
        for s, k in zip(*np.nonzero(counts)):
            seg = self.tape[s]
            sp, sr = poses[seg]
            a, n = int(starts[s, k]), int(counts[s, k])
            f_adh = 0.
            for c in range(a, a + n):
                nz = float(sr[:, 2] @ normals[c])  # -1: the body presses the adhesive (+z) face; +1: the plain back
                if nz <= -L.BOND_FACE_COS or (s in L.FLAP_SEGMENTS and nz >= L.BOND_FACE_COS):  # flaps: both faces
                    f_adh += float(forces[c])
            key = (int(s), int(k))
            self.press_max[key] = max(self.press_max.get(key, 0.), f_adh)
            if f_adh >= L.BOND_PRESS_N and key not in self.bonds:
                self.make_bond(int(s), int(k), poses)
        self.check_drift(poses)

    def make_bond(self, s, k, poses, break_n=L.BOND_BREAK_N, target_override=None):
        if BOND_MODE == 'drive':
            return self.make_glue(s, k, poses, break_n, target_override)
        from pxr import Gf, Sdf, UsdPhysics
        seg = self.tape[s]
        sp, sr = poses[seg]
        if target_override is not None:  # (name, None): a static target (world joint)
            name, target = target_override
        else:
            target = self.stick_pose(k, poses)
            name = self.stick_names[k]
        self.bond_seq += 1
        path = f'{self.root}/{seg}/bond_{self.bond_seq}'
        if USD_SYNC:
            self.sync_usd(seg, sp, sr)
            if target is not None and name in self.index:
                self.sync_usd(name, *target)
        j = UsdPhysics.FixedJoint.Define(self.stage, path)
        if target is None:  # static target: joint to the world frame
            rel_p, rel_r = sp + self.origin, sr
        else:
            tp, tr = target
            j.CreateBody0Rel().SetTargets([Sdf.Path(self.stick_paths[k])])
            rel_p, rel_r = tr.T @ (sp - tp), tr.T @ sr
        j.CreateBody1Rel().SetTargets([Sdf.Path(f'{self.root}/{seg}')])
        q = L.mat2quat_xyzw(rel_r)
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, rel_p)))
        j.CreateLocalRot0Attr(Gf.Quatf(float(q[3]), *map(float, q[:3])))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        scale = float(os.environ.get('RECYCLE_BREAK_SCALE', '1'))  # diagnostics only (never in evidence runs)
        # contacts between the stuck segment and the body stay on: pressing (compression) goes through contact, the
        # bond only carries pulling and shear, as an adhesive does
        j.CreateCollisionEnabledAttr(True)
        j.CreateBreakForceAttr(float(break_n) * scale)
        j.CreateBreakTorqueAttr(float(break_n * L.TAPE_WIDTH / 2) * scale)
        j.CreateExcludeFromArticulationAttr(True)
        self.bonds[(s, k)] = dict(path=path, rel_p=rel_p, rel_r=rel_r, t=self.t, name=name, static=target is None)
        self.bond_events.append(dict(t=round(self.t, 4), event='stick', segment=s, to=name,
                                     press_n=round(self.press_max.get((s, k), 0.), 3)))

    def glue_points(self, s, name, poses):
        """Current glue-point positions of segment s in the target's frame (world for the mat)."""
        sp, sr = poses[self.tape[s]]
        out = []
        for path, ax in _SPEC['glue'][(s, name)]:
            pw = sp + sr @ np.array([ax, 0, 0])
            if name == 'mat':
                out.append((path, pw + self.origin))
            else:
                bp, br = poses[name]
                out.append((path, br.T @ (pw - bp)))
        return out

    def set_glue(self, s, name, cap, targets=None):
        from pxr import UsdPhysics
        for path, ax in _SPEC['glue'][(s, name)]:
            prim = self.stage.GetPrimAtPath(path)
            tgt = None if targets is None else targets[path]
            for i, dof in enumerate(('transX', 'transY', 'transZ')):
                d = UsdPhysics.DriveAPI.Get(prim, dof)
                if tgt is not None:
                    d.GetTargetPositionAttr().Set(float(tgt[i]))
                d.GetMaxForceAttr().Set(float(cap))

    def make_glue(self, s, k, poses, break_n, target_override):
        name = target_override[0] if target_override is not None else self.stick_names[k]
        if (s, name) not in _SPEC['glue']:
            return  # drive mode: the tape sticks to the 9 V (and its flaps to the mat) only; see README
        pts = self.glue_points(s, name, poses)
        if name == 'mat':  # authored at the reset position: targets 0 (joint frame 0 = the reset glue point)
            targets = {path: np.zeros(3) for path, _ in pts}
        else:
            targets = {path: p for path, p in pts}
        self.set_glue(s, name, break_n / len(pts), targets)
        self.bonds[(s, k)] = dict(path=None, glue=targets, t=self.t, name=name, static=name == 'mat',
                                  rel_p=None)
        self.bond_events.append(dict(t=round(self.t, 4), event='stick', segment=s, to=name,
                                     press_n=round(self.press_max.get((s, k), 0.), 3)))

    def sync_usd(self, name, pos, rot):
        """Write a bench body's CURRENT simulated pose into its USD xform (bookkeeping only: the pose does not change).
        Isaac Lab keeps simulated poses in Fabric, so USD still holds the reset pose; PhysX parses a new joint against
        the USD poses of its two bodies, and with stale poses the new joint does not hold."""
        from pxr import Gf, UsdGeom
        prim = self.stage.GetPrimAtPath(f'{self.root}/{name}')
        parent = UsdGeom.Xformable(prim.GetParent()).ComputeLocalToWorldTransform(0)
        pw = np.asarray(pos, float) + self.origin
        m = np.eye(4)
        m[:3, :3], m[:3, 3] = rot, pw
        pm = np.array(parent, float).T  # Gf matrices are row-major with row vectors
        loc = np.linalg.inv(pm) @ m
        q = L.mat2quat_xyzw(loc[:3, :3])
        ops = {op.GetOpName(): op for op in UsdGeom.Xformable(prim).GetOrderedXformOps()}
        ops['xformOp:translate'].Set(Gf.Vec3d(*map(float, loc[:3, 3])))
        ops['xformOp:orient'].Set(Gf.Quatd(float(q[3]), *map(float, q[:3])))

    def check_drift(self, poses):
        if BOND_MODE == 'drive':
            return self.check_glue(poses)
        for (s, k), b in list(self.bonds.items()):
            sp, sr = poses[self.tape[s]]
            target = None if b.get('static') else self.stick_pose(k, poses)
            if target is None:
                cur = sp + self.origin
            else:
                tp, tr = target
                cur = tr.T @ (sp - tp)
            if np.linalg.norm(cur - b['rel_p']) > L.BOND_DRIFT_M:
                if self.stage.GetPrimAtPath(b['path']).IsValid():
                    self.stage.RemovePrim(b['path'])
                del self.bonds[(s, k)]
                self.bond_events.append(dict(t=round(self.t, 4), event='let go', segment=s, to=b['name'],
                                             drift_mm=round(float(np.linalg.norm(cur - b['rel_p'])) * 1000, 2)))

    # ---- scorer
    def touched(self, dt):
        f = self.robot_contacts.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
        mag = np.linalg.norm(f, axis=-1).sum(axis=-1)
        return {n: bool(mag[i] > 0.01) for i, n in enumerate(self.watched)}, {n: round(float(mag[i]), 3) for i, n in enumerate(self.watched)}

    def segments(self, poses=None):
        poses = poses or self.poses
        return [poses[n] for n in self.tape]

    def bonded_to(self, name):
        return {s for (s, k), b in self.bonds.items() if b['name'] == name}

    def bond_list(self):
        return sorted([s, b['name']] for (s, k), b in self.bonds.items())

    def tape_verdict(self, poses=None):
        poses = poses or self.poses
        bp, br = poses['nine_v']
        return tape_state(bp, br, self.segments(poses), self.bonded_to('nine_v'))

    def control(self, environment, dt=.05):
        STATE['steps'] += 1
        poses = self.poses
        taped = self.tape_verdict(poses)
        touched, touch_n = self.touched(dt)
        speeds = self.speeds()
        self.scorer.step(poses['nine_v'][0], {n: poses[n][0] for n in AA_NAMES}, taped, touched, speeds)
        STATE.update(score=self.scorer.score, success=self.scorer.success, veto=self.scorer.veto,
                     detail=dict(t=round(self.t, 4), last=self.scorer.last,
                                 bonds=self.bond_list(),
                                 n_bond_events=len(self.bond_events), touch_n=touch_n,
                                 press_max={f'{s}:{self.stick_names[k]}': round(v, 3)
                                            for (s, k), v in self.press_max.items() if v > .05},
                                 nine_v=np.round(poses['nine_v'][0] * 1000, 2).tolist(),
                                 hook_wall_s=round(self.hook_s, 2), nonfinite=self.nonfinite))
        self.press_max = {}


_RT: dict = {'rt': None, 'sim': None}


def runtime():
    return _RT['rt']


def bench_state():
    """Current poses {body: (pos, rot)} for the scripted demo (read-only)."""
    return _RT['rt'].read() if _RT['rt'] else None


def finger_tips(environment):
    """Pad-tip centres of all four fingers (world, env-relative)."""
    from yam_arena.constants import PAD_TIP_PAST_TCP, TCP_OFFSET
    base = environment.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
    pts = []
    for arm in ('left_robot', 'right_robot'):
        robot = base.scene[arm]
        pose = robot.data.body_link_pose_w[0].detach().cpu().numpy().astype(float)[list(robot.body_names).index('gripper')]
        jp = robot.data.joint_pos[0].detach().cpu().numpy().astype(float)
        jn = list(robot.joint_names)
        half = max(abs(jp[jn.index(j)]) for j in ('joint7', 'joint8'))
        r = L.quat_xyzw2mat(pose[3:7])
        tip = pose[:3] + r @ (np.array(TCP_OFFSET) + [0, 0, PAD_TIP_PAST_TCP]) - origin
        pts += [tip + r[:, 0] * s * (half + .0032) for s in (1., -1.)]
    return pts


def _patch_scene(scene) -> None:
    if getattr(scene, '_recycle_patched', False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Bench):
            return _arena_object(spec)
        return orig_task_object(spec)

    def object_states(environment, names):
        from yam_arena.task_file import ObjectState
        plain = [n for n in names if n != 'bench']
        out = orig_states(environment, plain) if plain else {}
        if 'bench' in names:
            rt = _RT['rt']
            p, r = rt.poses['nine_v'] if rt else (L.NINE_V_POS, L.NINE_V_ROT)
            z = np.zeros(3)
            out['bench'] = ObjectState(pos=np.asarray(p), quat_xyzw=L.mat2quat_xyzw(r), lin_vel=z, ang_vel=z,
                                       low=p - .1, high=p + .1)
        return {n: out[n] for n in names}

    orig_build = scene.build_task_environment

    def build(embodiment, task, control_hz=None):
        environment = orig_build(embodiment, task, control_hz)
        if any(isinstance(o, Bench) for o in task.objects):
            previous = environment.env_cfg_callback

            def configure(cfg):
                from yam_arena.constants import CONTROL_HZ
                cfg = previous(cfg)
                cfg.sim.dt = PHYSICS_DT
                cfg.decimation = round(1.0 / (PHYSICS_DT * (control_hz or CONTROL_HZ)))
                cfg.sim.render_interval = cfg.decimation
                if os.environ.get('RECYCLE_DEV_NO_RENDER') == '1':  # development runs only: skip camera renders
                    cfg.sim.render_interval = 10 ** 7
                return cfg

            environment.env_cfg_callback = configure
        return environment

    scene.build_task_environment = build
    scene.task_object = task_object
    scene.object_states = object_states
    scene._recycle_patched = True


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, '_recycle_patched', False):
        return
    base_task_step, base_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        if _RT['rt'] is not None:
            _RT['rt'].control(self._env, 1. / float(self.info.control_hz))
        return base_task_step(self, *a, **k)

    def reset(self, *a, **k):
        previous, _RT['rt'] = _RT['rt'], None  # no hooks while yam_arena resets and settles the arms
        if previous is not None:
            previous.restore_authored()
        obs = base_reset(self, *a, **k)
        if any(isinstance(o, Bench) for o in (self.task.objects if self.task is not None else ())):
            _wrap(self._env)
            rt = Runtime(self._env)
            # every trial (the first one too) starts from the authored poses at rest: the loose round cells can roll
            # a little while yam_arena settles the arms, differently from reset to reset
            rt.restore_authored()
            rt.reset()
            _RT['rt'] = rt
        return obs

    cls._task_step, cls.reset = _task_step, reset
    if os.environ.get('RECYCLE_DEV_NO_CAMERAS') == '1':  # development runs only: no camera sensors at all
        orig_obs = cls._observation

        def observation(self):
            from inspect_robots_yam.packing import STATE_KEY
            try:
                return orig_obs(self)
            except KeyError:
                pass
            from inspect_robots.types import Observation
            from yam_arena.constants import FINGER_OPEN
            per_arm = []
            for name in ('left_robot', 'right_robot'):
                arm_ids, finger_ids = self._ids[name]
                q = self._base.scene[name].data.joint_pos.torch[0]
                grip = float(np.clip(q[finger_ids].mean().item() / FINGER_OPEN, 0.0, 1.0))
                per_arm.append(np.append(q[arm_ids].cpu().numpy().astype(np.float64), grip))
            from inspect_robots_yam.packing import pack
            img = np.zeros((self.cam_height, self.cam_width, 3), np.uint8)
            return Observation(images={'top_cam': img}, state={STATE_KEY: pack(*per_arm)},
                               instruction=self._instruction, image_times={'top_cam': self._t}, state_time=self._t)
        cls._observation = observation
    cls._recycle_patched = True


def _patch_embodiment_class(E) -> None:
    """Development runs only (RECYCLE_DEV_NO_CAMERAS=1): build the arms without camera sensors."""
    if getattr(E.BimanualYamEmbodiment, '_recycle_patched', False):
        return
    orig_init = E.BimanualYamEmbodiment.__init__

    def init(self, *a, **k):
        k['enable_cameras'] = False
        orig_init(self, *a, **k)
    E.BimanualYamEmbodiment.__init__ = init
    E.BimanualYamEmbodiment._recycle_patched = True


def _wrap(environment) -> None:
    base = environment.unwrapped
    sim = base.sim
    if _RT['sim'] is sim:
        return
    orig = sim.step

    def step(*a, **k):
        out = orig(*a, **k)
        if _RT['rt'] is not None:
            _RT['rt'].physics(base.physics_dt)
        return out

    sim.step = step
    _RT['sim'] = sim


def install() -> None:
    import importlib.abc
    import importlib.util

    class _Finder(importlib.abc.MetaPathFinder):
        targets = {'yam_arena.scene': _patch_scene, 'yam_arena.inspect_embodiment': _patch_embodiment}
        if os.environ.get('RECYCLE_DEV_NO_CAMERAS') == '1':
            targets = dict(targets, **{'yam_arena.embodiment': _patch_embodiment_class})

        def find_spec(self, fullname, path, target=None):
            if fullname not in self.targets:
                return None
            sys.meta_path.remove(self)
            try:
                spec = importlib.util.find_spec(fullname)
            finally:
                sys.meta_path.insert(0, self)
            if spec is None or spec.loader is None:
                return None
            loader, patch = spec.loader, self.targets[fullname]
            orig_exec = loader.exec_module

            def exec_module(module):
                orig_exec(module)
                patch(module)

            loader.exec_module = exec_module
            return spec

    for name, patch in _Finder.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(type(f).__name__ == '_Finder' and getattr(f, 'targets', None) and 'yam_arena.scene' in f.targets
               for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
