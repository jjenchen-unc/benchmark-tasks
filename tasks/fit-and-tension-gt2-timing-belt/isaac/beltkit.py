"""beltkit: the timing-belt bench for yam_arena (Isaac Lab-Arena / PhysX), plus its runtime hooks.

The bench (the belt-reduction stage of a robot arm joint: fixed joint housing with the GT2 60T joint pulley, a NEMA 17
on a sliding motor plate with the GT2 20T motor pulley and a clamp lever; the 70-link GT2 belt, which starts hung over the joint pulley) is
authored as ONE custom-spawned assembly of PhysX rigid bodies and joints: yam_arena's task-file primitives are single
rigid bodies. Dimensions and poses come from ``../src/layout.py``; the scorer is ``../src/belt_scoring.py``.

Joints that exist (hard limits only: drives on maximal-coordinate joints are not honoured in this PhysX build, as
found in this and another task's calibration, so no joint carries a drive): joint pulley (revolute about x to the
world, turns freely), motor plate (prismatic along y to the world, 12 mm slot), motor shaft (fixed on the plate),
motor pulley (revolute about x on the shaft, turns freely), clamp lever (revolute about z on the plate, stops),
belt hinges (D6 at the pitch line between consecutive links, translations locked; twist, flat bending and edgewise
bending limited, no bending spring).

Belt tension is a model, not the chain: a PhysX chain of 0.1 g links stretches millimetres under newtons, so the links
give the belt's shape and contacts (the wrap is real) while the tension per span is computed from geometry
(``belt_scoring.belt_tension``: 10 N per mm of centre distance past 100.6 mm, where the simulated belt is just taut
with its teeth meshed; zero unless the belt is round both pulleys between their flanges). Friction, the belt's pull and the
clamp are explicit forces applied every physics step (``Runtime.bench_forces``): the plate's 4 N T-nut friction, the
belt's pull 2 T on the plate toward the joint pulley (so the robot pushes against the belt load), the clamp (once the
lever has been turned >= 75 deg clockwise, ``LEVER_LOCK_DEG``, the plate is held where it is, up to 300 N; turning
the lever back releases it) and the lever's 0.15 N m thread friction.

Runtime (hooks installed on import, nothing in yam_arena is edited):
* after every physics step: read every bench body and belt link, apply the lever clamp rule, and every
  ``SCORE_EVERY`` physics steps measure and step the scorer (arm contact from a PhysX contact view, fingertips);
* after every control step: publish the scorer state for ``score()``.
The scripted demo READS poses through ``bench_state()`` / ``belt_state()``; nothing here writes a body pose after
reset. In-process reset (environment reset only; never during a trial): ``Runtime.restore_authored`` puts every
bench body and link back at its authored pose before yam_arena's own reset and settle, and after it
``Runtime.restore_start`` writes back the settled start state recorded at the process's first reset (zero velocity),
so every trial starts from the same poses.
"""
from __future__ import annotations

import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SRC = HERE.parent / 'src'
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from yam_arena.task_file import Box  # noqa: E402
import layout as L  # noqa: E402
import belt_scoring as S  # noqa: E402

ALU, DARK, BLACK, STEEL = (.72, .74, .76), (.12, .12, .13), (.04, .04, .04), (.55, .56, .58)
BELT_COL, WOOD, WHITE, GREEN, RED, BRASS = (.06, .06, .06), (.55, .40, .25), (.92, .92, .9), (.1, .65, .2), (.8, .1, .08), (.75, .62, .3)
PHYSICS_DT = 0.0020833333  # 480 Hz (the start state and wrap were tuned and verified at this exact step)
SCORE_EVERY = 8
# explicit bench forces (see Runtime.bench_forces); gains chosen for stability of an explicit force at 480 Hz on the
# 0.34 kg plate and the ~1.5e-5 kg m^2 lever
SLIDE_K, SLIDE_C = 3.0e4, 100.0                     # plate friction anchor spring (N/m) and damping (N s/m), cap 4 N
CLAMP_KP, CLAMP_C, CLAMP_KI = 3.0e4, 150.0, 6.0e5   # clamped plate: PI hold (N/m, N s/m, N/(m s))
ROT_K = (4.0, 0.8, 0.02)   # friction anchor springs (N m/rad; stick window = friction / K: lever 4 deg, joint pulley
                           # 0.7 deg): lever (I ~1.5e-5), joint pulley (~1.1e-5), motor pulley
ROT_C = (0.003, 0.002, 5e-5)  # (~2.5e-7 kg m^2); damping (N m s)
ARM_CONTACT_N = .05
BELT_FRICTION = .8
CONTACT_OFFSET = .0002
LINK_INERTIA_SCALE = 4.0
LINK_MASS = L.LINK_MASS
POS_ITERS = 32           # solver position iterations (bench bodies)


def _rot(q_xyzw):
    x, y, z, w = q_xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _quat(r):
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(np.asarray(r, float)).as_quat()  # x, y, z, w


def _prism_x(x0, x1, outline):
    """Convex prism along x through a convex (y, z) outline."""
    n = len(outline)
    v = [np.r_[x, y, z] for x in (x0, x1) for y, z in outline]
    faces = [list(range(n))[::-1], list(range(n, 2 * n))] + [[i, (i + 1) % n, n + (i + 1) % n, n + i] for i in range(n)]
    return np.array(v), faces


def _prism_y(y0, y1, outline_xz):
    """Convex prism along y through a convex (x, z) outline."""
    n = len(outline_xz)
    v = [np.r_[x, y, z] for y in (y0, y1) for x, z in outline_xz]
    faces = [list(range(n)), list(range(n, 2 * n))[::-1]] + [[i, n + i, n + (i + 1) % n, (i + 1) % n] for i in range(n)]
    return np.array(v), faces


@dataclass
class Body:
    name: str
    pos: np.ndarray
    rot: np.ndarray
    mass: float
    friction: float = .8
    boxes: list = field(default_factory=list)      # (centre, size, colour) body frame
    cylinders: list = field(default_factory=list)  # (centre, radius, height, colour, axis)
    meshes: list = field(default_factory=list)     # (vertices, faces, colour)
    visuals: list = field(default_factory=list)    # (centre, size, colour) no collision
    visual_cylinders: list = field(default_factory=list)
    damping: tuple = (0., .5)
    static: bool = False
    inertia: tuple | None = None
    offset: float | None = None                     # contact offset override


def pulley_shapes(kind):
    """Teeth (trapezoid prisms), core and flanges of a GT2 pulley (20T motor / 60T joint), in its body frame (origin
    on the axis at the tooth-zone centre, axis = x)."""
    meshes = []
    hz = L.TOOTH_ZONE / 2
    n = L.TEETH[kind]
    r0, r1 = L.r_root(kind) - .0002, L.r_od(kind)
    w0, w1 = L.PULLEY_TOOTH_ROOT_W[kind] / 2, L.PULLEY_TOOTH_TOP_W[kind] / 2
    for k in range(n):
        a = 2 * math.pi * k / n
        u = np.array([math.cos(a), math.sin(a)])   # radial (y, z)
        t = np.array([-math.sin(a), math.cos(a)])  # tangential
        outline = [u * r0 - t * w0, u * r0 + t * w0, u * r1 + t * w1, u * r1 - t * w1]
        meshes.append((*_prism_x(-hz, hz, [tuple(p) for p in outline]), ALU))
    cyl = [((0, 0, 0), L.r_root(kind), L.TOOTH_ZONE, ALU, 'X'),
           ((-(hz + L.FLANGE_T / 2), 0, 0), L.R_FLANGE[kind], L.FLANGE_T, ALU, 'X'),
           ((hz + L.FLANGE_T / 2, 0, 0), L.R_FLANGE[kind], L.FLANGE_T, ALU, 'X')]
    if kind == 'motor':
        cyl.append(((hz + L.FLANGE_T + L.HUB_L / 2, 0, 0), L.HUB_R, L.HUB_L, ALU, 'X'))
    else:
        cyl.append(((hz + L.FLANGE_T + L.JOINT_HUB_L / 2, 0, 0), L.JOINT_HUB_R, L.JOINT_HUB_L, ALU, 'X'))
    return meshes, cyl


def link_shapes():
    """One belt link (link frame, see layout): backing collider, two trapezoid teeth, full-height visual backing."""
    hl, hw = L.LINK_LEN / 2, L.BELT_WIDTH / 2
    z0, z1 = L.BACK_COLLIDE_Z
    boxes = [((0, 0, (z0 + z1) / 2), (L.LINK_LEN * .98, L.BELT_WIDTH, z1 - z0), BELT_COL)]
    meshes = []
    for xc in L.TOOTH_X:
        a, b = L.TOOTH_ROOT_W / 2, L.TOOTH_TIP_W / 2
        outline = [(xc - a, z0 + .00002), (xc + a, z0 + .00002), (xc + b, L.TOOTH_TIP_Z), (xc - b, L.TOOTH_TIP_Z)]
        meshes.append((*_prism_y(-hw, hw, outline), BELT_COL))
    del hl
    return boxes, meshes


def bench_spec():
    """Every body of the bench in world coordinates (no simulator)."""
    T = L.TOP
    bodies = {}
    rz0, rz1 = L.RAIL_Z
    rail_top = rz1
    za = L.AXIS_Z
    px0, px1 = L.PLATE_FRONT, L.PLATE_FRONT + .006
    # --- static frame: rail, posts, joint housing plate with its bearing boss and shaft
    fr = Body('frame', np.zeros(3), np.eye(3), 1., .6, static=True)
    fr.boxes.append((((L.RAIL_X[0] + L.RAIL_X[1]) / 2, 0, (rz0 + rz1) / 2), (L.RAIL_X[1] - L.RAIL_X[0], L.RAIL_Y[1] - L.RAIL_Y[0], rz1 - rz0), ALU))
    for y in L.POST_Y:
        fr.boxes.append(((.41, y, (T + rz0) / 2), (.02, .02, rz0 - T), ALU))
    yj = L.JOINT_Y
    jp = L.JOINT_PLATE
    fr.boxes.append((((px0 + px1) / 2, yj, (jp['z'][0] + jp['z'][1]) / 2), (px1 - px0, jp['width'], jp['z'][1] - jp['z'][0]), ALU))
    fr.boxes.append((((px1 + .432) / 2, yj, rail_top + .0025), (.432 - px1, jp['width'], .005), ALU))
    fr.cylinders.append((((px1 + px1 + .012) / 2, yj, za), .014, .012, DARK, 'X'))  # bearing housing behind the plate
    hub_back = L.BELT_X + L.TOOTH_ZONE / 2 + L.FLANGE_T + L.JOINT_HUB_L
    fr.visual_cylinders.append((((px0 + hub_back) / 2, yj, za), L.JOINT_SHAFT_R, px0 - hub_back, STEEL, 'X'))
    bodies['frame'] = fr
    # --- joint pulley (60T) on the joint shaft
    m, c = pulley_shapes('joint')
    bodies['joint_pulley'] = Body('joint_pulley', np.array([L.BELT_X, yj, za]), np.eye(3), L.PULLEY_MASS['joint'], .6,
                                  meshes=m, cylinders=c, offset=CONTACT_OFFSET)
    # --- sliding motor plate with the NEMA 17 (origin: plate front face centre at axis height), axle, motor pulley
    c0 = L.C_START
    ym = L.motor_y(c0)
    blk = Body('motor_block', np.array([px0, ym, za]), np.eye(3), L.IDLER_BLOCK_MASS, .3)
    hw = L.IDLER_PLATE['width'] / 2
    pz0, pz1 = rail_top - za, L.IDLER_PLATE['z'][1] - za
    blk.boxes.append(((.003, 0, (pz0 + pz1) / 2), (.006, 2 * hw, pz1 - pz0), ALU))                   # upright plate
    fx0, fx1 = .006, L.IDLER_FOOT['x'][1] - px0
    blk.boxes.append((((fx0 + fx1) / 2, 0, rail_top - za + .0025), (fx1 - fx0, 2 * hw, .005), ALU))  # foot on the rail
    tb = L.LEVER_TAB
    blk.boxes.append((((tb['x'][0] + tb['x'][1]) / 2 - px0, (tb['y'][0] + tb['y'][1]) / 2, rail_top - za + .0025),
                      (tb['x'][1] - tb['x'][0], tb['y'][1] - tb['y'][0], .005), ALU))                # lever tab
    mb = L.MOTOR_BOX
    blk.boxes.append((((mb['x'][0] + mb['x'][1]) / 2 - px0, 0, 0), (mb['x'][1] - mb['x'][0], 2 * mb['half'], 2 * mb['half']), DARK))
    blk.visuals.append(((-.0003, 0, rail_top - za - .004), (.0006, .0012, .008), RED))                # index mark
    bodies['motor_block'] = blk
    shaft0 = L.BELT_X + L.TOOTH_ZONE / 2 + L.FLANGE_T + L.HUB_L
    bodies['motor_axle'] = Body('motor_axle', np.array([L.BELT_X, ym, za]), np.eye(3), L.AXLE_MASS, .3,
                                visual_cylinders=[(((px0 + shaft0) / 2 - L.BELT_X, 0, 0), L.SHAFT_R, px0 - shaft0, STEEL, 'X')])
    m, c = pulley_shapes('motor')
    bodies['motor_pulley'] = Body('motor_pulley', np.array([L.BELT_X, ym, za]), np.eye(3), L.PULLEY_MASS['motor'], .6,
                                  meshes=m, cylinders=c, offset=CONTACT_OFFSET)
    lv = L.LEVER
    hz0, hz1 = lv['hub_z']
    lev = Body('lever', np.array([lv['x'], ym + lv['dy'], (hz0 + hz1) / 2]), np.eye(3), lv['mass'], .8)
    lev.cylinders.append(((0, 0, 0), lv['hub_r'], hz1 - hz0, BLACK, 'Z'))
    # handle along -y (outward) when loose
    lev.boxes.append(((0, -(lv['arm'] / 2 + .004), (hz1 - hz0) / 2 - lv['arm_t'] / 2), (lv['arm_w'], lv['arm'] - .008, lv['arm_t']), BLACK))
    lev.cylinders.append(((0, -lv['arm'], (hz1 - hz0) / 2 - lv['arm_t'] / 2), .006, lv['arm_t'] + .004, BLACK, 'Z'))  # handle end knob
    bodies['lever'] = lev
    # --- belt links
    cen, rots = L.belt_start()
    lb, lm = link_shapes()
    I = np.array([L.BELT_WIDTH ** 2 + L.BELT_T ** 2, L.LINK_LEN ** 2 + L.BELT_T ** 2,
                  L.LINK_LEN ** 2 + L.BELT_WIDTH ** 2]) * LINK_MASS / 12 * LINK_INERTIA_SCALE
    links = []
    for i in range(L.N_LINKS):
        nm = f'belt_{i:02d}'
        bodies[nm] = Body(nm, cen[i], rots[i], LINK_MASS, BELT_FRICTION, boxes=list(lb), meshes=list(lm),
                          damping=(.2, .2), inertia=tuple(float(v) for v in I), offset=CONTACT_OFFSET)
        links.append(nm)
    return dict(bodies=bodies, links=links, start_c=c0)


class Bench(Box):
    """The bench, as one task-file object (subclasses Box only to pass yam_arena's checks)."""


_SPEC: dict = {}


def _spawn_bench(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    spec = bench_spec()
    _SPEC.clear()
    _SPEC.update(spec)
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    ox, oy, oz, ow = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(ow, ox, oy, oz))
    world = root.ComputeLocalToWorldTransform(0)
    _SPEC['root_world'] = np.array(world.ExtractTranslation(), float)
    looks, mats = {}, {}

    def look(color):
        if color not in looks:
            path = f'{prim_path}/Looks/c{len(looks)}'
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=.45)
            vis.func(path, vis)
            looks[color] = path
        return looks[color]

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
        q = _quat(rot)
        x.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(q[3]), *map(float, q[:3])))
        return x

    def collide(prim, body, color):
        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(float(body.offset if body.offset is not None else .0005))
        cp.CreateRestOffsetAttr(0.)
        bind_physics_material(str(prim.GetPath()), material(body.friction), stage=stage)
        bind_visual_material(str(prim.GetPath()), look(color), stage=stage)

    def rigid(path, b):
        prim = stage.GetPrimAtPath(path)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        ma = UsdPhysics.MassAPI.Apply(prim)
        ma.CreateMassAttr(float(b.mass))
        if b.inertia is not None:
            ma.CreateDiagonalInertiaAttr(Gf.Vec3f(*b.inertia))
            ma.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(.2)
        rb.CreateSolverPositionIterationCountAttr(POS_ITERS)
        rb.CreateSolverVelocityIterationCountAttr(4)
        rb.CreateLinearDampingAttr(float(b.damping[0]))
        rb.CreateAngularDampingAttr(float(b.damping[1]))
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.)

    def mesh_prim(path, v, faces, col, b, collision=True):
        mesh = UsdGeom.Mesh.Define(stage, path)
        mesh.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in v])
        mesh.CreateFaceVertexCountsAttr([len(f) for f in faces])
        mesh.CreateFaceVertexIndicesAttr([j for f in faces for j in f])
        mesh.CreateSubdivisionSchemeAttr('none')
        if collision:
            collide(mesh.GetPrim(), b, col)
            UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
            PhysxSchema.PhysxConvexHullCollisionAPI.Apply(mesh.GetPrim()).CreateMinThicknessAttr(.00001)
        else:
            bind_visual_material(path, look(col), stage=stage)

    paths = {}
    for b in spec['bodies'].values():
        path = f'{prim_path}/{b.name}'
        paths[b.name] = path
        xform(path, b.pos, b.rot)
        if not b.static:
            rigid(path, b)
        for i, (c, s, col) in enumerate(b.boxes):
            cube = UsdGeom.Cube.Define(stage, f'{path}/box_{i}')
            cube.CreateSizeAttr(1.)
            cube.AddTranslateOp().Set(Gf.Vec3d(*map(float, c)))
            cube.AddScaleOp().Set(Gf.Vec3f(*map(float, s)))
            collide(cube.GetPrim(), b, col)
        for i, (c, r, h, col, axis) in enumerate(b.cylinders):
            g = UsdGeom.Cylinder.Define(stage, f'{path}/cyl_{i}')
            g.CreateAxisAttr(axis)
            g.CreateRadiusAttr(float(r))
            g.CreateHeightAttr(float(h))
            g.AddTranslateOp().Set(Gf.Vec3d(*map(float, c)))
            collide(g.GetPrim(), b, col)
        for i, (v, faces, col) in enumerate(b.meshes):
            mesh_prim(f'{path}/mesh_{i}', v, faces, col, b)
        for i, (c, s, col) in enumerate(b.visuals):
            cube = UsdGeom.Cube.Define(stage, f'{path}/vis_{i}')
            cube.CreateSizeAttr(1.)
            cube.AddTranslateOp().Set(Gf.Vec3d(*map(float, c)))
            cube.AddScaleOp().Set(Gf.Vec3f(*map(float, s)))
            bind_visual_material(str(cube.GetPrim().GetPath()), look(col), stage=stage)
        for i, (c, r, h, col, axis) in enumerate(b.visual_cylinders):
            g = UsdGeom.Cylinder.Define(stage, f'{path}/vcyl_{i}')
            g.CreateAxisAttr(axis)
            g.CreateRadiusAttr(float(r))
            g.CreateHeightAttr(float(h))
            g.AddTranslateOp().Set(Gf.Vec3d(*map(float, c)))
            bind_visual_material(str(g.GetPrim().GetPath()), look(col), stage=stage)
    _joints(stage, prim_path, spec, paths)
    return root.GetPrim()


def _frame(path, spec):
    """(pos, rot) of a body path (None = world) in the bench root frame."""
    if path is None:
        return -_SPEC.get('root_world', np.zeros(3)), np.eye(3)
    b = spec['bodies'][path.rsplit('/', 1)[1]]
    return b.pos, b.rot


def _joint(stage, jtype, jpath, path0, path1, point, jrot, spec):
    """Joint of type ``jtype`` (UsdPhysics class) at world ``point`` with world frame orientation ``jrot``."""
    from pxr import Gf, Sdf
    j = jtype.Define(stage, jpath)
    if path0 is not None:
        j.CreateBody0Rel().SetTargets([Sdf.Path(path0)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(path1)])
    p0, r0 = _frame(path0, spec)
    p1, r1 = _frame(path1, spec)
    q0, q1 = _quat(r0.T @ jrot), _quat(r1.T @ jrot)
    j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, r0.T @ (np.asarray(point) - p0))))
    j.CreateLocalRot0Attr(Gf.Quatf(float(q0[3]), *map(float, q0[:3])))
    j.CreateLocalPos1Attr(Gf.Vec3f(*map(float, r1.T @ (np.asarray(point) - p1))))
    j.CreateLocalRot1Attr(Gf.Quatf(float(q1[3]), *map(float, q1[:3])))
    return j


def _joints(stage, prim_path, spec, paths):
    from pxr import Gf, Sdf, UsdPhysics
    B = spec['bodies']
    deg = math.pi / 180
    # joint pulley on the (fixed) joint shaft: revolute about x, bearing + joint friction
    j = _joint(stage, UsdPhysics.RevoluteJoint, f'{paths["joint_pulley"]}/shaft', None, paths['joint_pulley'],
               B['joint_pulley'].pos, np.eye(3), spec)
    j.CreateAxisAttr('X')
    # motor plate: prismatic along y in its 10 mm slot, dry friction (finger-loose T-nut)
    blk = B['motor_block']
    j = _joint(stage, UsdPhysics.PrismaticJoint, f'{paths["motor_block"]}/slide', None, paths['motor_block'],
               blk.pos, np.eye(3), spec)
    j.CreateAxisAttr('Y')
    y0 = blk.pos[1]
    j.CreateLowerLimitAttr(float(L.motor_y(L.C_MAX) - y0))
    j.CreateUpperLimitAttr(float(L.motor_y(L.C_MIN) - y0))
    _SPEC['slide_joint'] = str(j.GetPrim().GetPath())
    # motor shaft (axle body) fixed on the plate (prismatic joint with zero travel)
    j = _joint(stage, UsdPhysics.PrismaticJoint, f'{paths["motor_axle"]}/spring', paths['motor_block'],
               paths['motor_axle'], B['motor_axle'].pos, np.eye(3), spec)
    j.CreateAxisAttr('Y')
    j.CreateLowerLimitAttr(0.)
    j.CreateUpperLimitAttr(0.)
    # motor pulley on the motor shaft: revolute about x, stepper detent as a friction torque
    j = _joint(stage, UsdPhysics.RevoluteJoint, f'{paths["motor_pulley"]}/shaft', paths['motor_axle'],
               paths['motor_pulley'], B['motor_pulley'].pos, np.eye(3), spec)
    j.CreateAxisAttr('X')
    # clamp lever: revolute about z on the plate's foot (its thread axis), friction torque, stops
    j = _joint(stage, UsdPhysics.RevoluteJoint, f'{paths["lever"]}/thread', paths['motor_block'], paths['lever'],
               B['lever'].pos, np.eye(3), spec)
    j.CreateAxisAttr('Z')
    j.CreateLowerLimitAttr(float(L.LEVER_STOP_DEG[0]))
    j.CreateUpperLimitAttr(float(L.LEVER_STOP_DEG[1]))
    # belt hinges: D6 at the pitch line, link i's +x end to link i+1's -x end, frame = link i's axes at rest
    # (straight); translations locked, rotX twist / rotY flat bend / rotZ edgewise bend limited (hard limits only:
    # drives on maximal-coordinate joints are not honoured in this PhysX build, so the hinges have no bending spring)
    links = spec['links']
    for i, nm in enumerate(links):
        nxt = links[(i + 1) % len(links)]
        a, b = B[nm], B[nxt]
        jp = UsdPhysics.Joint.Define(stage, f'{paths[nm]}/hinge')
        jp.CreateBody0Rel().SetTargets([Sdf.Path(paths[nm])])
        jp.CreateBody1Rel().SetTargets([Sdf.Path(paths[nxt])])
        jp.CreateLocalPos0Attr(Gf.Vec3f(float(L.LINK_LEN / 2), 0., 0.))
        jp.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        jp.CreateLocalPos1Attr(Gf.Vec3f(float(-L.LINK_LEN / 2), 0., 0.))
        jp.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        jp.CreateExcludeFromArticulationAttr(True)
        prim = jp.GetPrim()
        for dof in ('transX', 'transY', 'transZ'):
            lim = UsdPhysics.LimitAPI.Apply(prim, dof)
            lim.CreateLowAttr(1.)
            lim.CreateHighAttr(-1.)
        for dof, lo in (('rotX', L.TWIST_LIMIT_DEG), ('rotY', L.BEND_LIMIT_DEG), ('rotZ', L.EDGE_LIMIT_DEG)):
            lim = UsdPhysics.LimitAPI.Apply(prim, dof)
            lim.CreateLowAttr(-float(lo))
            lim.CreateHighAttr(float(lo))
        del a, b


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
STATE: dict = dict(score=0., level=0., vetoes=[], steps=0, detail={})


def lever_angle_deg(block_rot, lever_rot):
    """Lever rotation about the block's z (deg, + = counter-clockwise seen from above; clockwise clamps)."""
    r = np.asarray(block_rot).T @ np.asarray(lever_rot)
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


class Runtime:
    """Reads the bench through PhysX tensor views; lever clamp rule; scorer."""

    def __init__(self, environment):
        import carb
        import omni.physics.tensors as tensors
        import torch
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdUtils
        # contact forces for the arm-contact view (CPU PhysX reports them only with contact processing on)
        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        self.base = environment.unwrapped
        self.env = environment
        self.torch = torch
        stage = get_current_stage()
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        self.simview = tensors.create_simulation_view('torch', stage_id=stage_id)
        self.simview.set_subspace_roots('/')
        root = '/World/envs/env_0/bench'
        self.root = root
        self.names = [n for n, b in _SPEC['bodies'].items() if not b.static]
        self.links = list(_SPEC['links'])
        self.robot_links = []
        for arm in ('left_robot', 'right_robot'):
            view = self.base.scene[arm].root_view
            paths = view.link_paths
            self.robot_links += list(paths[0] if paths and isinstance(paths[0], (list, tuple)) else paths)
        sensors = [f'{root}/{n}' for n in self.names]
        self.contact_view = self.simview.create_rigid_contact_view(
            sensors, filter_patterns=[self.robot_links] * len(sensors), max_contact_data_count=8192)
        self.view = self.simview.create_rigid_body_view(sensors)
        self.index = {n: i for i, n in enumerate(self.names)}
        self.link_idx = np.array([self.index[n] for n in self.links])
        self.origin = self.base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        self.stage = stage
        self.device = self.view.get_transforms().device
        self.i_block = self.index['motor_block']
        self.fr_idx = torch.tensor([self.index[n] for n in ('lever', 'joint_pulley', 'motor_pulley', 'motor_block')],
                                   dtype=torch.int32, device=self.device)
        self.rot_cap = torch.tensor([L.LEVER_FRICTION_NM, L.JOINT_FRICTION_NM, L.MOTOR_DETENT_NM], device=self.device)
        self.rot_k = torch.tensor(ROT_K, device=self.device)
        self.rot_c = torch.tensor(ROT_C, device=self.device)
        self.y_taut = float(L.motor_y(L.C_TENSION_ZERO) + self.origin[1])
        self.restore_start()
        self.reset()

    def reset(self):
        self.scorer = S.BeltScorer()
        self.clamped, self.engaged = False, False
        self.y_clamp, self.i_term = 0., self.torch.zeros((), device=self.device)
        self.rot_anchor, self.y_anchor = None, None
        self.t = 0.
        self.sub = 0
        self.hook_s = 0.
        self.touch_max = {}
        self.read_all()
        self.m = None
        self.measure_now(0.)
        STATE.update(score=0., level=0., vetoes=[], steps=0, detail={})

    def set_clamp(self, on):
        """Clamp rule (lever turned >= 75 deg clockwise): the plate is held where it is (stiff PI hold, up to 300 N)."""
        if on == self.clamped:
            return
        if on:
            self.y_clamp = float(self.pos[self.index['motor_block'], 1] + self.origin[1])
            self.i_term = self.torch.zeros((), device=self.device)
        self.clamped = on

    def restore_authored(self):
        """Environment reset only: every bench body and link back at its authored pose, zero velocity."""
        torch = self.torch
        tf = self.view.get_transforms().clone()
        for n, i in self.index.items():
            b = _SPEC['bodies'][n]
            tf[i, :3] = torch.tensor(np.asarray(b.pos, float) + self.origin, dtype=tf.dtype)
            tf[i, 3:7] = torch.tensor(_quat(b.rot), dtype=tf.dtype)
        idx = torch.arange(len(self.names), dtype=torch.int32, device=tf.device)
        self.view.set_transforms(tf, idx)
        self.view.set_velocities(torch.zeros_like(self.view.get_velocities()), idx)
        self.clamped, self.engaged = False, False
        self.rot_anchor, self.y_anchor = None, None

    def restore_start(self):
        """Environment reset only (after yam_arena's reset and settle, before the trial's first control step). The
        process's first reset records the settled start state (pose of every bench body and belt link) and leaves it
        as it is; every later reset writes it back with zero velocity. Without this the light hanging belt settles a
        few mm differently after each reset and the lever drifts (no friction forces act during the settle)."""
        torch = self.torch
        if 'tf' not in _START:
            _START['tf'] = self.view.get_transforms().clone()
            return
        idx = torch.arange(len(self.names), dtype=torch.int32, device=self.device)
        self.view.set_transforms(_START['tf'].clone(), idx)
        self.view.set_velocities(torch.zeros_like(self.view.get_velocities()), idx)

    def read_all(self):
        tf = self.view.get_transforms().detach().cpu().numpy().astype(float)
        self.pos = tf[:, :3] - self.origin
        self.quat = tf[:, 3:7]
        return self.pos, self.quat

    def pose(self, name):
        i = self.index[name]
        return self.pos[i].copy(), _rot(self.quat[i])

    def belt(self):
        """Link centres [70,3] and rotations [70,3,3] (world, env-relative)."""
        c = self.pos[self.link_idx]
        r = np.array([_rot(q) for q in self.quat[self.link_idx]])
        return c, r

    def block_tilt(self):
        """Motor plate's rotation from its authored orientation (deg about x, y, z; diagnostic)."""
        r = self.pose('motor_block')[1]
        return [round(math.degrees(math.atan2(r[2, 1], r[2, 2])), 3),
                round(math.degrees(-math.asin(max(-1., min(1., r[2, 0])))), 3),
                round(math.degrees(math.atan2(r[1, 0], r[0, 0])), 3)]

    def lever_deg(self):
        return lever_angle_deg(self.pose('motor_block')[1], self.pose('lever')[1])

    def bench_forces(self, dt):
        """Every physics step, on the GPU (no host sync): the forces that stand in for friction, the belt and the clamp
        (drives on maximal-coordinate joints are not honoured in this PhysX build, so none of this can be a joint
        drive), applied for the next step. Dry friction is a stick-slip spring anchor: a stiff spring to an anchor
        that is dragged along whenever the spring force would exceed the friction (so the force never exceeds it),
        plus light damping, all capped at the friction:
        * plate along its slot: 4 N (finger-loose T-nut); once clamped, a stiff PI hold at the clamped position
          (up to 300 N, the M5 clamp's thread force);
        * plate: the belt's pull while the belt is round both pulleys between their flanges, PLATE_K x (travel past
          taut) toward the joint pulley (= 2 T, capped at 60 N);
        * lever about its thread axis: 0.15 N m; joint pulley: ``JOINT_FRICTION_NM`` (shaft bearings); motor pulley:
          ``MOTOR_DETENT_NM`` (stepper detent).
        """
        torch = self.torch
        tf = self.view.get_transforms()
        vel = self.view.get_velocities()
        ib = self.i_block
        q = tf[self.fr_idx, 3:7]                     # lever, joint pulley, motor pulley, plate
        ang = torch.stack([
            torch.atan2(2 * (q[0, 3] * q[0, 2] + q[0, 0] * q[0, 1]), 1 - 2 * (q[0, 1] ** 2 + q[0, 2] ** 2))
            - torch.atan2(2 * (q[3, 3] * q[3, 2] + q[3, 0] * q[3, 1]), 1 - 2 * (q[3, 1] ** 2 + q[3, 2] ** 2)),
            torch.atan2(2 * (q[1, 3] * q[1, 0] + q[1, 1] * q[1, 2]), 1 - 2 * (q[1, 0] ** 2 + q[1, 1] ** 2)),
            torch.atan2(2 * (q[2, 3] * q[2, 0] + q[2, 1] * q[2, 2]), 1 - 2 * (q[2, 0] ** 2 + q[2, 1] ** 2))])
        w = torch.stack([vel[self.fr_idx[0], 5] - vel[ib, 5], vel[self.fr_idx[1], 3], vel[self.fr_idx[2], 3]])
        if self.rot_anchor is None:
            self.rot_anchor = ang.clone()
        d = torch.remainder(ang - self.rot_anchor + math.pi, 2 * math.pi) - math.pi
        lim = self.rot_cap / self.rot_k
        d = torch.clamp(d, -lim, lim)
        self.rot_anchor = ang - d                    # drag the anchors
        tq = torch.clamp(-self.rot_k * d - self.rot_c * w, -self.rot_cap, self.rot_cap)
        yb, vy = tf[ib, 1], vel[ib, 1]
        if self.clamped:
            err = self.y_clamp - yb
            self.i_term = torch.clamp(self.i_term + CLAMP_KI * err * dt, -L.SLIDE_CLAMPED_N, L.SLIDE_CLAMPED_N)
            fy = torch.clamp(CLAMP_KP * err - CLAMP_C * vy + self.i_term, -L.SLIDE_CLAMPED_N, L.SLIDE_CLAMPED_N)
            self.y_anchor = yb.clone()
        else:
            if self.y_anchor is None:
                self.y_anchor = yb.clone()
            dy = torch.clamp(yb - self.y_anchor, -L.SLIDE_FRICTION_N / SLIDE_K, L.SLIDE_FRICTION_N / SLIDE_K)
            self.y_anchor = yb - dy
            fy = torch.clamp(-SLIDE_K * dy - SLIDE_C * vy, -L.SLIDE_FRICTION_N, L.SLIDE_FRICTION_N)
        if self.engaged:
            fy = fy + torch.clamp(L.PLATE_K * (self.y_taut - yb), 0., 2 * L.BELT_T_CAP)
        f = torch.zeros((4, 3), device=self.device)
        t = torch.zeros((4, 3), device=self.device)
        f[3, 1] = fy
        t[0, 2] = tq[0]
        t[3, 2] = -tq[0]                             # the lever's friction reacts on the plate
        t[1, 0] = tq[1]
        t[2, 0] = tq[2]
        full_f = torch.zeros((len(self.names), 3), device=self.device)
        full_t = torch.zeros((len(self.names), 3), device=self.device)
        full_f[self.fr_idx.long()] = f
        full_t[self.fr_idx.long()] = t
        self.view.apply_forces_and_torques_at_position(full_f, full_t, None, self.fr_idx, True)

    def physics(self, dt):
        """Every physics step; the work (one GPU->CPU read of every bench body, the lever clamp rule, arm contacts and
        the scorer) runs every SCORE_EVERY steps (60 Hz at the default 8 and 480 Hz physics): each read synchronises with the GPU."""
        self.t += dt
        self.sub += 1
        self.bench_forces(dt)
        if self.sub % SCORE_EVERY:
            return
        t_hook = time.perf_counter()
        self.read_all()
        lev = self.lever_deg()
        self.set_clamp(lev <= L.LEVER_LOCK_DEG)
        touched = self.arm_contacts(dt)

        for n, f in touched.items():
            self.touch_max[n] = max(self.touch_max.get(n, 0.), f)
        self.measure_now(dt * SCORE_EVERY, touched)
        self.hook_s += time.perf_counter() - t_hook

    def measure_now(self, dt, touched=None):
        c, r = self.belt()
        tips = finger_tips(self.env) if dt > 0 else ()
        self.m = S.measure(c, r, self.pose('motor_pulley')[0][1], self.lever_deg(), None, tips, touched or {})
        self.engaged = all(self.m[k]['around'] and self.m[k]['between'] for k in ('joint', 'motor'))
        if dt > 0:
            self.scorer.step(dt, self.m)

    def arm_contacts(self, dt):
        f = self.contact_view.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
        mag = np.linalg.norm(f, axis=-1).sum(axis=-1)
        return {self.names[i]: round(float(mag[i]), 3) for i in np.nonzero(mag > ARM_CONTACT_N)[0]}

    def control(self, environment, dt=.05):
        STATE['steps'] += 1
        m = self.m or {}
        STATE.update(score=self.scorer.score, level=self.scorer.level, vetoes=list(self.scorer.vetoes),
                     detail=dict(t=round(self.t, 4), instant=self.scorer.instant, hold_s=round(self.scorer.hold_s, 3),
                                 centre_distance_mm=m.get('centre_distance_mm'), band=m.get('band'),
                                 tension_n=m.get('tension_n'), belt_load=bool(self.engaged), block_tilt_deg=self.block_tilt(),
                                 joint=m.get('joint'),
                                 lever_deg=m.get('lever_deg'), locked=m.get('locked'), clamped=self.clamped,
                                 motor=m.get('motor'), spans=m.get('spans'),
                                 clearance_m=m.get('clearance_m'), touched=m.get('touched'),
                                 arm_touch_max=dict(self.touch_max), hook_wall_s=round(self.hook_s, 2)))
        self.touch_max = {}


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
        r = _rot(pose[3:7])
        tip = pose[:3] + r @ (np.array(TCP_OFFSET) + [0, 0, PAD_TIP_PAST_TCP]) - origin
        pts += [tip + r[:, 0] * s * (half + .0032) for s in (1., -1.)]
    return pts


_RT: dict = {'rt': None, 'sim': None}
_START: dict = {}   # settled start state recorded at this process's first reset (Runtime.restore_start)


def runtime():
    return _RT['rt']


def bench_state():
    """Current poses {body: (pos, rot)} of every dynamic bench body (read-only, for the scripted demo)."""
    rt = _RT['rt']
    if rt is None:
        return None
    return {n: rt.pose(n) for n in rt.names}


def belt_state():
    """Belt link centres and rotations (read-only, for the scripted demo)."""
    rt = _RT['rt']
    return rt.belt() if rt else None


def _patch_scene(scene) -> None:
    if getattr(scene, '_bench_patched', False):
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
            p = rt.pose('motor_block')[0] if rt else np.array([L.PLATE_FRONT, L.motor_y(L.C_START), L.AXIS_Z])
            z = np.zeros(3)
            out['bench'] = ObjectState(pos=np.asarray(p), quat_xyzw=np.array([0., 0., 0., 1.]), lin_vel=z, ang_vel=z,
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
                return cfg

            environment.env_cfg_callback = configure
        return environment

    scene.build_task_environment = build
    scene.task_object = task_object
    scene.object_states = object_states
    scene._bench_patched = True


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, '_bench_patched', False):
        return
    base_task_step, base_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        if _RT['rt'] is not None:
            _RT['rt'].control(self._env, 1. / float(self.info.control_hz))
        return base_task_step(self, *a, **k)

    def reset(self, *a, **k):
        previous, _RT['rt'] = _RT['rt'], None
        if previous is not None:
            previous.restore_authored()
        obs = base_reset(self, *a, **k)
        if any(isinstance(o, Bench) for o in (self.task.objects if self.task is not None else ())):
            _wrap(self._env)
            _RT['rt'] = Runtime(self._env)
        return obs

    cls._task_step, cls.reset = _task_step, reset
    cls._bench_patched = True


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
