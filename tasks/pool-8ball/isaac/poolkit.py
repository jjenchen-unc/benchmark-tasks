"""poolkit: the 7 ft bar-box pool table, the two balls, the house cue and its rest for yam_arena (Isaac Lab-Arena /
PhysX), plus the runtime hooks.

Authored as one custom-spawned assembly (yam_arena task-file primitives are single boxes/spheres):
* table parts as kinematic rigid bodies (so PhysX contact views can report forces on them): the slate bed with its
  six pocket holes (convex cells), the six K-66-style cushion segments with WPA jaw facings, the wooden top rails,
  the six pocket liners (catch cups under the holes), the apron / cabinet (cosmetic);
* the robot pedestals and the camera post (replacing the station table, which would sit inside the pool table), and
  the two-hook cue rest;
* the called-pocket marker coin (kinematic; moved to the called pocket's rail on reset);
* the cue ball and the 8-ball as free spheres (2 1/4 in, 170 g), the cue as a free body (tapered shaft + butt + tip).
Geometry: ``../src/layout.py``; scorer: ``../src/scoring.py`` (pure functions).

Runtime hooks (installed on import; nothing in yam_arena is edited): after every physics step the runtime applies the
cloth's rolling resistance to each ball resting on the bed (a constant torque against its rolling, 1.4 mu_r m g R,
which decelerates a rolling ball at mu_r g; PhysX has no rolling resistance) and reads the contact forces (cue on cue
ball, cue ball on 8-ball, cue or robot on any ball); after every control step the scorer reads the ball poses.
Nothing here writes a pose or a velocity during a trial; ``Runtime.restore_authored`` places the balls, the cue and
the coin only on an environment reset. The scripted demo READS poses through ``bench_state()``.
"""

from __future__ import annotations

import math
import os
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SRC = HERE.parent / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

import layout as L  # noqa: E402
from scoring import Monitor, Scorer  # noqa: E402
from yam_arena.task_file import Box  # noqa: E402

PHYSICS_DT = float(os.environ.get("POOL_PHYSICS_DT", str(1 / 480)))
CONTACT_OFFSET = 0.002
SETUP_BASE = int(os.environ.get("POOL_SETUP", "0"))
FORCE_NAMED = "POOL_SETUP" in os.environ
DEBUG = bool(os.environ.get("POOL_DEBUG"))
BOUNCE_THRESHOLD = 0.02  # m/s: PhysX treats slower impacts as inelastic (Isaac Lab default 0.5 m/s)

CLOTH, RAILWOOD, CUSHION_COL, LINER, CABINET = (
    (0.05, 0.38, 0.22),
    (0.36, 0.20, 0.10),
    (0.04, 0.33, 0.19),
    (0.06, 0.06, 0.06),
    (0.15, 0.09, 0.05),
)
WHITE, BLACK, MAPLE, EBONY, TIP_BLUE, BRASS, GREY = (
    (0.95, 0.95, 0.92),
    (0.04, 0.04, 0.045),
    (0.85, 0.70, 0.48),
    (0.10, 0.07, 0.05),
    (0.15, 0.35, 0.75),
    (0.80, 0.62, 0.20),
    (0.45, 0.46, 0.48),
)
MOVING = ("cueball", "eightball", "cue", "coin")
KINEMATIC = ("bed", "cushions", "rails", "liners", "fixtures", "coin")


class PoolTable(Box):
    """The whole bench, as one task-file object (subclasses Box only to pass yam_arena's checks)."""


def _rot(q_xyzw):
    x, y, z, w = q_xyzw
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def _quat(r):
    from scipy.spatial.transform import Rotation

    q = Rotation.from_matrix(np.asarray(r, float)).as_quat()  # xyzw
    return q / np.linalg.norm(q)


def authored(setup):
    """Reset pose of the moving bodies for a setup: {name: (pos, rot)} in the env frame."""
    sp = L.setup_spec(setup)
    cp, cr = L.cue_rest_pose(L.rest_side(sp["pocket"]))
    return {
        "cueball": (np.array([sp["cue"][0], sp["cue"][1], L.BALL_Z + 0.0002]), np.eye(3)),
        "eightball": (np.array([sp["obj"][0], sp["obj"][1], L.BALL_Z + 0.0002]), L.rz(90.0)),
        "cue": (cp, cr),
        "coin": (L.coin_pos(sp["pocket"]), np.eye(3)),
    }


_SPEC: dict = {}


def _cyl_pts(r, z0, z1, n=24, center=(0.0, 0.0)):
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    ring = np.c_[center[0] + r * np.cos(a), center[1] + r * np.sin(a)]
    return np.vstack([np.c_[ring, np.full(n, z0)], np.c_[ring, np.full(n, z1)]])


def _frustum_x(s0, s1, r0, r1, n=20):
    """Points of a tapered cylinder along the cue's x axis from x=-s1 (radius r1) to x=-s0 (radius r0)."""
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    pts = []
    for s, r in ((s0, r0), (s1, r1)):
        pts += [[-s, r * math.cos(t), r * math.sin(t)] for t in a]
    return np.array(pts)


def _spawn_bench(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics
    from scipy.spatial import ConvexHull
    from yam_arena.station import linear_rgb

    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    ox, oy, oz, ow = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(ow, ox, oy, oz))
    looks, mats = {}, {}

    def look(color, rough=0.55):
        key = (color, rough)
        if key not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=rough)
            vis.func(path, vis)
            looks[key] = path
        return looks[key]

    def material(friction, restitution=0.0):
        key = (friction, restitution)
        if key not in mats:
            path = f"{prim_path}/Looks/phys{len(mats)}"
            pm = sim_utils.RigidBodyMaterialCfg(
                static_friction=friction,
                dynamic_friction=friction,
                restitution=restitution,
                friction_combine_mode="average",
                restitution_combine_mode="average",
            )
            pm.func(path, pm)
            mats[key] = path
        return mats[key]

    def xform(path, pos, rot):
        x = UsdGeom.Xform.Define(stage, path)
        x.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float, pos)))
        q = _quat(rot)
        x.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(q[3]), *map(float, q[:3])))
        return x

    def rigid(path, mass=None, kinematic=False, inertia=None, com=None, ball=False):
        prim = stage.GetPrimAtPath(path)
        rb = UsdPhysics.RigidBodyAPI.Apply(prim)
        if kinematic:
            rb.CreateKinematicEnabledAttr(True)
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(float(mass if mass is not None else 1.0))
        if inertia is not None:
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, inertia)))
        if com is not None:
            m.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
        px = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        px.CreateMaxDepenetrationVelocityAttr(0.5)
        px.CreateSolverPositionIterationCountAttr(16)
        px.CreateSolverVelocityIterationCountAttr(4)
        px.CreateLinearDampingAttr(0.0)
        px.CreateAngularDampingAttr(0.0 if ball else 0.05)
        px.CreateMaxAngularVelocityAttr(5000.0)
        px.CreateSleepThresholdAttr(0.0)  # never sleeps: a sleeping body reports no contacts
        px.CreateStabilizationThresholdAttr(0.0)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)

    count = {"n": 0}

    def collide(prim, friction, restitution):
        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(CONTACT_OFFSET)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), material(friction, restitution), stage=stage)
        count["n"] += 1

    def hull(path, v, color, friction=0.4, restitution=0.0, collision=True, rough=0.55):
        v = np.asarray(v, float)
        h = ConvexHull(v)
        m = UsdGeom.Mesh.Define(stage, path)
        m.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in v])
        m.CreateFaceVertexCountsAttr([3] * len(h.simplices))
        faces = []
        c = v.mean(axis=0)
        for s in h.simplices:
            a, b, d = v[s[0]], v[s[1]], v[s[2]]
            n = np.cross(b - a, d - a)
            faces.append(s if n @ (a - c) > 0 else s[::-1])
        m.CreateFaceVertexIndicesAttr([int(j) for f in faces for j in f])
        m.CreateSubdivisionSchemeAttr("none")
        m.CreateExtentAttr([Gf.Vec3f(*map(float, v.min(axis=0))), Gf.Vec3f(*map(float, v.max(axis=0)))])
        prim = m.GetPrim()
        if collision:
            collide(prim, friction, restitution)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
            hc = PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim)
            hc.CreateHullVertexLimitAttr(64)
            hc.CreateMinThicknessAttr(0.0005)
        bind_visual_material(str(prim.GetPath()), look(color, rough), stage=stage)
        return m

    def box(path, center, size, color, friction=0.4, restitution=0.0, collision=True, rough=0.55, yaw_deg=0.0):
        hx, hy, hz = (s / 2 for s in size)
        corners = np.array([[sx * hx, sy * hy, sz * hz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
        corners = corners @ L.rz(yaw_deg).T
        return hull(path, corners + np.asarray(center, float), color, friction, restitution, collision, rough)

    def sphere(path, r, color, friction, restitution):
        s = UsdGeom.Sphere.Define(stage, path)
        s.CreateRadiusAttr(float(r))
        s.CreateExtentAttr([Gf.Vec3f(-r, -r, -r), Gf.Vec3f(r, r, r)])
        collide(s.GetPrim(), friction, restitution)
        bind_visual_material(str(s.GetPrim().GetPath()), look(color, 0.15), stage=stage)
        return s

    def disc(path, r, z, color, n=24):
        """Visual-only thin disc (ball number spot), local frame."""
        return hull(path, _cyl_pts(r, z - 0.0004, z + 0.0004, n), color, collision=False, rough=0.2)

    zero = (0.0, 0.0, 0.0)
    # ---------------------------------------------------------------- slate bed with pocket holes (kinematic)
    xform(f"{prim_path}/bed", zero, np.eye(3))
    rigid(f"{prim_path}/bed", 50.0, kinematic=True)
    for i, (x0, y0, x1, y1) in enumerate(L.bed_cells()):
        box(
            f"{prim_path}/bed/c{i:04d}",
            ((x0 + x1) / 2, (y0 + y1) / 2, L.BED_Z - L.SLATE_T / 2),
            (x1 - x0, y1 - y0, L.SLATE_T),
            CLOTH,
            L.CLOTH_FRICTION,
            L.CLOTH_RESTITUTION,
            rough=0.95,
        )
    # ---------------------------------------------------------------- cushions
    xform(f"{prim_path}/cushions", zero, np.eye(3))
    rigid(f"{prim_path}/cushions", 10.0, kinematic=True)
    for i, seg in enumerate(L.cushion_segments()):
        hull(
            f"{prim_path}/cushions/k{i}",
            L.cushion_prism(seg),
            CUSHION_COL,
            L.CUSHION_FRICTION,
            L.CUSHION_RESTITUTION,
            rough=0.95,
        )
    # ---------------------------------------------------------------- rails, diamonds, apron
    xform(f"{prim_path}/rails", zero, np.eye(3))
    rigid(f"{prim_path}/rails", 20.0, kinematic=True)
    for i, (c, s) in enumerate(L.rail_boxes()):
        box(f"{prim_path}/rails/r{i}", c, s, RAILWOOD, L.RAIL_FRICTION, rough=0.35)
    # sights (diamonds) on the rail tops: cosmetic
    k = 0
    for xi in np.linspace(L.X0, L.XF, 9)[1:-1]:
        if abs(xi - L.XS) < 0.01:
            continue
        for sy in (-1, 1):
            c = (xi, sy * (L.YW + L.CUSHION_D + L.RAIL_W / 2), L.BED_Z + L.RAIL_TOP + 0.0005)
            hull(f"{prim_path}/rails/sight{k}", _cyl_pts(0.006, c[2] - 0.0005, c[2], 12, c[:2]), WHITE, collision=False)
            k += 1
    for yi in np.linspace(-L.YW, L.YW, 5)[1:-1]:
        for x, sx in ((L.X0, -1), (L.XF, 1)):
            c = (x + sx * (L.CUSHION_D + L.RAIL_W / 2), yi, L.BED_Z + L.RAIL_TOP + 0.0005)
            hull(f"{prim_path}/rails/sight{k}", _cyl_pts(0.006, c[2] - 0.0005, c[2], 12, c[:2]), WHITE, collision=False)
            k += 1
    # apron and cabinet (cosmetic, no collision): the table body below the rails, down to the floor
    out = L.CUSHION_D + L.RAIL_W
    xa, xb, ya, yb = L.X0 - out, L.XF + out, -L.YW - out, L.YW + out
    za, zb = L.BED_Z - 0.10, L.BED_Z - L.POCKET_DEPTH - 0.06
    box(f"{prim_path}/rails/apron", ((xa + xb) / 2, 0.0, (za + zb) / 2), (xb - xa, yb - ya, za - zb), CABINET, collision=False)
    for i, (px, py) in enumerate(((xa + 0.15, ya + 0.15), (xa + 0.15, yb - 0.15), (xb - 0.15, ya + 0.15), (xb - 0.15, yb - 0.15))):
        box(f"{prim_path}/rails/leg{i}", (px, py, (zb + L.FLOOR_Z) / 2), (0.16, 0.16, zb - L.FLOOR_Z), CABINET, collision=False)
    # ---------------------------------------------------------------- pocket liners (catch cups below the holes)
    xform(f"{prim_path}/liners", zero, np.eye(3))
    rigid(f"{prim_path}/liners", 5.0, kinematic=True)
    for name in L.POCKETS:
        c, r = L.pocket_hole(name)
        zf = L.BED_Z - L.POCKET_DEPTH
        h = r + 0.012
        box(f"{prim_path}/liners/{name}_floor", (c[0], c[1], zf - 0.005), (2 * h + 0.02, 2 * h + 0.02, 0.01), LINER, 0.5)
        for j, (dx, dy, sx, sy) in enumerate(((h + 0.005, 0, 0.01, 2 * h + 0.02), (-h - 0.005, 0, 0.01, 2 * h + 0.02),
                                              (0, h + 0.005, 2 * h + 0.02, 0.01), (0, -h - 0.005, 2 * h + 0.02, 0.01))):
            box(
                f"{prim_path}/liners/{name}_w{j}",
                (c[0] + dx, c[1] + dy, (zf + L.BED_Z - L.SLATE_T) / 2),
                (sx, sy, L.BED_Z - L.SLATE_T - zf),
                LINER,
                0.5,
            )
    # ---------------------------------------------------------------- fixtures: robot pedestals, camera post, cue rest
    xform(f"{prim_path}/fixtures", zero, np.eye(3))
    rigid(f"{prim_path}/fixtures", 20.0, kinematic=True)
    for side, sy in (("left", 1), ("right", -1)):
        y = sy * 0.305
        box(f"{prim_path}/fixtures/{side}_plate", (-0.03, y, L.BENCH_TOP - 0.015), (0.18, 0.12, 0.03), GREY)
        box(
            f"{prim_path}/fixtures/{side}_post",
            (-0.09, y, (L.BENCH_TOP - 0.03 + L.FLOOR_Z) / 2),
            (0.06, 0.12, L.BENCH_TOP - 0.03 - L.FLOOR_Z),
            GREY,
        )
    mx, my = L.MAST_XY
    hull(f"{prim_path}/fixtures/cam_post", _cyl_pts(0.020, L.FLOOR_Z, 0.762, 24, (mx, my)), GREY)
    for side in ("left", "right"):
        rp, rr = L.cue_rest_pose(side)
        yaw = math.degrees(math.atan2(rr[1, 0], rr[0, 0]))
        for i, sh in enumerate(L.CUE_REST_HOOK_S):
            hx, hy = L.rest_hook_xy(sh, side)
            r = L.cue_radius(sh)
            zb = L.CUE_REST_Z - r
            across = rr[:, 1]
            nm = f"{prim_path}/fixtures/{side}_hook{i}"
            box(f"{nm}_base", (hx, hy, zb - 0.005), (0.03, 2 * r + 0.024, 0.01), BLACK, 0.6, yaw_deg=yaw)
            for sgn in (-1, 1):
                c = np.array([hx, hy, zb + 0.008]) + sgn * (r + 0.007) * across
                box(f"{nm}_side{'ab'[sgn > 0]}", c, (0.03, 0.006, 0.026), BLACK, 0.6, yaw_deg=yaw)
            box(f"{nm}_post", (hx, hy, (zb - 0.01 + L.FLOOR_Z) / 2), (0.02, 0.02, zb - 0.01 - L.FLOOR_Z), GREY)
    # ---------------------------------------------------------------- marker coin (kinematic, moved on reset)
    a = authored(SETUP_BASE)
    xform(f"{prim_path}/coin", *a["coin"])
    rigid(f"{prim_path}/coin", 0.006, kinematic=True)
    hull(f"{prim_path}/coin/disc", _cyl_pts(0.012, 0.0, 0.0018, 24), BRASS, collision=False, rough=0.25)
    # ---------------------------------------------------------------- balls (free)
    inertia = (0.4 * L.BALL_MASS * L.BALL_R**2,) * 3
    for name, col in (("cueball", WHITE), ("eightball", BLACK)):
        xform(f"{prim_path}/{name}", *a[name])
        rigid(f"{prim_path}/{name}", L.BALL_MASS, inertia=inertia, ball=True)
        sphere(f"{prim_path}/{name}/s", L.BALL_R, col, L.BALL_FRICTION, L.BALL_RESTITUTION)
    # the 8-ball's white number spots (two, opposite) and the cue ball's red dot: visual only (they show its spin)
    for i, sz in enumerate((1, -1)):
        q = UsdGeom.Xform.Define(stage, f"{prim_path}/eightball/spot{i}")
        q.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
        disc(f"{prim_path}/eightball/spot{i}/d", 0.0105, sz * (L.BALL_R - 0.0003), WHITE)
        disc(f"{prim_path}/eightball/spot{i}/n", 0.0045, sz * (L.BALL_R + 0.0002), BLACK)
    disc(f"{prim_path}/cueball/dot", 0.004, L.BALL_R - 0.0002, (0.8, 0.1, 0.1))
    # ---------------------------------------------------------------- the cue (free body; frame: tip face at origin)
    xform(f"{prim_path}/cue", *a["cue"])
    rigid(f"{prim_path}/cue", L.CUE_MASS, inertia=L.CUE_INERTIA, com=L.CUE_COM)
    half = L.CUE_L / 2
    hull(f"{prim_path}/cue/tip", _frustum_x(0.0, L.TIP_LEN, L.CUE_R_TIP - 0.0003, L.CUE_R_TIP), TIP_BLUE,
         L.TIP_FRICTION, L.TIP_RESTITUTION)
    hull(f"{prim_path}/cue/shaft", _frustum_x(L.TIP_LEN, half, L.cue_radius(L.TIP_LEN), L.CUE_R_JOINT), MAPLE,
         L.SHAFT_FRICTION, 0.3, rough=0.3)
    hull(f"{prim_path}/cue/butt", _frustum_x(half, L.CUE_L, L.CUE_R_JOINT, L.CUE_R_BUTT), EBONY,
         L.BUTT_FRICTION, 0.3, rough=0.3)
    hull(f"{prim_path}/cue/ferrule", _frustum_x(L.TIP_LEN, L.TIP_LEN + 0.012, L.cue_radius(L.TIP_LEN) + 0.0002,
         L.cue_radius(L.TIP_LEN + 0.012) + 0.0002), WHITE, collision=False)
    hull(f"{prim_path}/cue/wrap", _frustum_x(L.CUE_L - 0.40, L.CUE_L - 0.12, L.cue_radius(L.CUE_L - 0.40) + 0.0003,
         L.cue_radius(L.CUE_L - 0.12) + 0.0003), BLACK, collision=False, rough=0.9)
    _SPEC["paths"] = {n: f"{prim_path}/{n}" for n in ("bed", "cushions", "rails", "liners", "fixtures") + MOVING}
    _SPEC["colliders"] = count["n"]
    return root.GetPrim()


def _arena_object(spec):
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class BenchSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_bench)

    return Object(
        name=spec.name,
        object_type=ObjectType.BASE,
        spawner_cfg=BenchSpawnerCfg(),
        initial_pose=Pose(position_xyz=(0.0, 0.0, 0.0), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)),
    )


# ---------------------------------------------------------------------------------------------- runtime state
STATE: dict = dict(score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, trial=-1, setup=None)


def finger_tips(environment):
    """Pad-tip centres of all four fingers (env frame)."""
    from yam_arena.constants import PAD_TIP_PAST_TCP, TCP_OFFSET

    base = environment.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
    pts = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        pose = robot.data.body_link_pose_w[0].detach().cpu().numpy().astype(float)[list(robot.body_names).index("gripper")]
        jp = robot.data.joint_pos[0].detach().cpu().numpy().astype(float)
        jn = list(robot.joint_names)
        half = max(abs(jp[jn.index(j)]) for j in ("joint7", "joint8"))
        r = _rot(pose[3:7])
        tip = pose[:3] + r @ (np.array(TCP_OFFSET) + [0, 0, PAD_TIP_PAST_TCP]) - origin
        pts += [tip + r[:, 0] * s * (half + 0.0032) for s in (1.0, -1.0)]
    return pts


class Runtime:
    """Applies the cloth's rolling resistance, reads the contact forces every physics step, scores every control
    step."""

    def __init__(self, environment):
        import carb
        import omni.physics.tensors as tensors
        import torch
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdUtils

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        self.base = environment.unwrapped
        self.env = environment
        self.torch = torch
        stage = get_current_stage()
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        self.simview = tensors.create_simulation_view("torch", stage_id=stage_id)
        self.simview.set_subspace_roots("/")
        root = "/World/envs/env_0/pool"
        P = lambda n: f"{root}/{n}"  # noqa: E731
        self.view = self.simview.create_rigid_body_view([P(n) for n in MOVING])
        robot_links = []
        for arm in ("left_robot", "right_robot"):
            lp = self.base.scene[arm].root_view.link_paths
            robot_links += list(lp[0] if lp and isinstance(lp[0], (list, tuple)) else lp)
        self.robot_links = robot_links
        self.n_robot = len(robot_links)
        # cue ball: [cue, eightball, robot links...]; 8-ball: [cue, cueball, robot links...]
        self.cv_cb = self.simview.create_rigid_contact_view(
            [P("cueball")], filter_patterns=[[P("cue"), P("eightball")] + robot_links], max_contact_data_count=2048
        )
        self.cv_8 = self.simview.create_rigid_contact_view(
            [P("eightball")], filter_patterns=[[P("cue"), P("cueball")] + robot_links], max_contact_data_count=2048
        )
        self.cv_cue = self.simview.create_rigid_contact_view(
            [P("cue")], filter_patterns=[robot_links], max_contact_data_count=4096
        )
        self.origin = self.base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        self.setup = SETUP_BASE
        self.trial_active = False
        self.dev = self.view.get_transforms().device
        self.ball_ids = torch.tensor([0, 1], dtype=torch.int32, device=self.dev)
        self._init_state()

    def _init_state(self):
        self.monitor = Monitor()
        self.scorer = Scorer()
        self.steps = 0
        self.t = 0.0
        self.hook_s = 0.0
        self.nonfinite = 0
        self.grip_peak = {}
        STATE.update(score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, setup=self.setup)

    def begin_trial(self, trial):
        self._init_state()
        self.trial_active = True
        sp = L.setup_spec(self.setup)
        STATE.update(trial=trial, setup=self.setup, start=dict(pocket=sp["pocket"], cue=sp["cue"].tolist(),
                                                              obj=sp["obj"].tolist(), name=sp["name"]))

    # ------------------------------------------------------------------ reading
    def read(self):
        tf = self.view.get_transforms().detach().cpu().numpy().astype(float)
        return {n: (tf[i, :3] - self.origin, _rot(tf[i, 3:7])) for i, n in enumerate(MOVING)}

    def velocities(self):
        v = self.view.get_velocities().detach().cpu().numpy().astype(float)
        return {n: (v[i, :3], v[i, 3:6]) for i, n in enumerate(MOVING)}

    # ------------------------------------------------------------------ cloth model
    def rolling_resistance(self, tf, vel):
        """Torque against each resting ball's rolling (constant magnitude 1.4 mu_r m g R, faded out below 5 mm/s
        surface speed) plus a spin-decay torque about the vertical; only while the ball rests on the bed."""
        torch = self.torch
        tq = np.zeros((2, 3))
        tau = 1.4 * L.ROLL_MU * L.BALL_MASS * L.G * L.BALL_R
        inertia = 0.4 * L.BALL_MASS * L.BALL_R**2
        for i in range(2):
            z = tf[i, 2] - self.origin[2]
            if abs(z - L.BALL_Z) > 0.003:
                continue
            w = vel[i, 3:6]
            wh = np.array([w[0], w[1], 0.0])
            n = float(np.linalg.norm(wh))
            if n > 1e-6:
                tq[i] -= tau * min(1.0, n * L.BALL_R / 0.005) * wh / n
            if abs(w[2]) > 1e-6:
                tq[i, 2] -= inertia * L.SPIN_DECEL * min(1.0, abs(w[2]) / 0.5) * math.copysign(1.0, w[2])
        full = torch.zeros((len(MOVING), 3), dtype=torch.float32, device=self.dev)
        full[:2] = torch.tensor(tq, dtype=torch.float32, device=self.dev)
        self.view.apply_forces_and_torques_at_position(None, full, None, self.ball_ids, True)

    def physics(self, dt):
        t0 = time.perf_counter()
        self.t += dt
        tf = self.view.get_transforms().detach().cpu().numpy().astype(float)
        vel = self.view.get_velocities().detach().cpu().numpy().astype(float)
        self.rolling_resistance(tf, vel)
        if self.trial_active:
            fcb = self.cv_cb.get_contact_force_matrix(dt=dt).detach().cpu().numpy()[0]
            f8 = self.cv_8.get_contact_force_matrix(dt=dt).detach().cpu().numpy()[0]
            fq = self.cv_cue.get_contact_force_matrix(dt=dt).detach().cpu().numpy()[0]
            nrm = lambda f: float(np.linalg.norm(f))  # noqa: E731
            cue_cb = nrm(fcb[0])
            cb_8 = max(nrm(fcb[1]), nrm(f8[1]))
            cue_8 = nrm(f8[0])
            robot_cb = float(np.linalg.norm(fcb[2:], axis=-1).max()) if fcb.shape[0] > 2 else 0.0
            robot_8 = float(np.linalg.norm(f8[2:], axis=-1).max()) if f8.shape[0] > 2 else 0.0
            robot_cue = float(np.linalg.norm(fq, axis=-1).max()) if fq.size else 0.0
            self.monitor.step(dt, cue_cb, cb_8, cue_8, max(robot_cb, robot_8), robot_cue, vel[0, :3])
            self.last_contacts = dict(
                cue_cb=round(cue_cb, 3), cb_8=round(cb_8, 3), cue_8=round(cue_8, 3),
                robot_ball=round(max(robot_cb, robot_8), 3), robot_cue=round(robot_cue, 3),
            )
            if DEBUG and (cue_cb > 0 or cb_8 > 0):
                print("CONTACT", round(self.t, 4), self.last_contacts, flush=True)
        self.hook_s += time.perf_counter() - t0

    # ------------------------------------------------------------------ reset
    def restore_authored(self, setup):
        """Environment reset only (never during a trial): balls at the setup's spots, cue in its rest, coin at the
        called pocket; all at rest."""
        torch = self.torch
        self.setup = setup
        self.trial_active = False
        a = authored(setup)
        tf = self.view.get_transforms().clone()
        for i, n in enumerate(MOVING):
            pos, rot = a[n]
            tf[i, :3] = torch.tensor(np.asarray(pos, float) + self.origin, dtype=tf.dtype)
            tf[i, 3:7] = torch.tensor(_quat(rot), dtype=tf.dtype)
        idx = torch.tensor([0, 1, 2, 3], dtype=torch.int32, device=tf.device)
        self.view.set_transforms(tf, idx)
        self.view.set_velocities(torch.zeros_like(self.view.get_velocities()), torch.tensor([0, 1, 2], dtype=torch.int32, device=tf.device))
        try:
            self.view.set_kinematic_targets(tf, torch.tensor([3], dtype=torch.int32, device=tf.device))
        except Exception:  # noqa: BLE001
            pass
        self._init_state()

    # ------------------------------------------------------------------ scoring
    def control(self, environment):
        if not self.trial_active:
            return
        self.steps += 1
        poses = self.read()
        vel = self.velocities()
        pc, p8 = poses["cueball"][0], poses["eightball"][0]
        if not (np.all(np.isfinite(pc)) and np.all(np.isfinite(p8))):
            self.nonfinite += 1
            STATE["nonfinite"] = self.nonfinite
            return
        sp = L.setup_spec(self.setup)
        tips = finger_tips(environment)
        tip_d = min(float(np.linalg.norm(t - p)) for t in tips for p in (pc, p8)) - L.BALL_R
        level = self.scorer.step(
            pc, p8, float(np.linalg.norm(vel["cueball"][0])), float(np.linalg.norm(vel["eightball"][0])),
            sp["pocket"], self.monitor, tip_d,
        )
        STATE.update(
            score=level,
            level=level,
            steps=self.steps,
            nonfinite=self.nonfinite,
            detail=dict(
                self.scorer.last,
                pocket=sp["pocket"],
                cue_pos_mm=np.round(poses["cue"][0] * 1000, 2).tolist(),
                cb_mm=np.round(pc * 1000, 2).tolist(),
                eight_mm=np.round(p8 * 1000, 2).tolist(),
                setup=self.setup,
                hook_wall_s=round(self.hook_s, 2),
                contacts=getattr(self, "last_contacts", None),
            ),
        )


_RT: dict = {"rt": None, "sim": None, "trials": 0}


def runtime():
    return _RT["rt"]


def bench_state():
    """Current poses {'cueball', 'eightball', 'cue', 'coin'} for the scripted demo (read-only)."""
    return _RT["rt"].read() if _RT["rt"] else None


def bench_velocities():
    return _RT["rt"].velocities() if _RT["rt"] else None


def _patch_scene(scene) -> None:
    if getattr(scene, "_pool_patched", False):
        return
    orig_task_object, orig_states, orig_room = scene.task_object, scene.object_states, scene.task_room

    def task_object(spec):
        if isinstance(spec, PoolTable):
            return _arena_object(spec)
        return orig_task_object(spec)

    def task_room(scene_name, top_cam):
        """The station's room without its work table (the pool table stands there; the arms sit on pedestals) and
        with the floor lowered under the pool table's legs; camera stand without its table foot (a post holds it)."""
        assets = orig_room(scene_name, top_cam)
        out = [a for a in assets if a.name not in ("table", "floor", "stand_foot")]
        out.append(scene._box("floor", (200.0, 200.0, 0.02), (0.0, 0.0, L.FLOOR_Z - 0.01), scene.STATION_FLOOR_COLOR))
        return out

    def object_states(environment, names):
        from yam_arena.task_file import ObjectState

        plain = [n for n in names if n != "pool"]
        out = orig_states(environment, plain) if plain else {}
        if "pool" in names:
            rt = _RT["rt"]
            p, r = rt.read()["eightball"] if rt else authored(SETUP_BASE)["eightball"]
            z = np.zeros(3)
            out["pool"] = ObjectState(pos=np.asarray(p), quat_xyzw=_quat(r), lin_vel=z, ang_vel=z, low=p - 0.03, high=p + 0.03)
        return {n: out[n] for n in names}

    orig_build = scene.build_task_environment

    def build(embodiment, task, control_hz=None):
        environment = orig_build(embodiment, task, control_hz)
        if any(isinstance(o, PoolTable) for o in task.objects):
            previous = environment.env_cfg_callback

            def configure(cfg):
                from yam_arena.constants import CONTROL_HZ

                cfg = previous(cfg)
                cfg.sim.dt = PHYSICS_DT
                cfg.decimation = round(1.0 / (PHYSICS_DT * (control_hz or CONTROL_HZ)))
                cfg.sim.render_interval = cfg.decimation
                phys = getattr(cfg.sim, "physics", None)
                if phys is not None and hasattr(phys, "bounce_threshold_velocity"):
                    phys.bounce_threshold_velocity = BOUNCE_THRESHOLD
                else:
                    print("poolkit: WARNING no physics.bounce_threshold_velocity on the sim cfg", flush=True)
                return cfg

            environment.env_cfg_callback = configure
        return environment

    scene.build_task_environment = build
    scene.task_object = task_object
    scene.object_states = object_states
    scene.task_room = task_room
    scene._pool_patched = True


def _has_pool(emb):
    return any(isinstance(o, PoolTable) for o in (emb.task.objects if emb.task is not None else ()))


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_pool_patched", False):
        return
    base_task_step, base_reset, base_ensure = cls._task_step, cls.reset, cls._ensure_sim

    def _ensure_sim(self, *a, **k):
        env_before = self._env
        out = base_ensure(self, *a, **k)
        if self._env is not None and self._env is not env_before and _has_pool(self):
            _wrap(self._env)
            _RT["rt"] = Runtime(self._env)
        return out

    def _task_step(self, *a, **k):
        if _RT["rt"] is not None:
            _RT["rt"].control(self._env)
        return base_task_step(self, *a, **k)

    def reset(self, *a, **k):
        rt = _RT["rt"]
        seed = k.get("seed")
        if seed is not None and not FORCE_NAMED:
            setup = ("seed", int(seed))
        else:
            setup = (SETUP_BASE + _RT["trials"]) % len(L.NAMED)
        if rt is not None:
            rt.restore_authored(setup)
        obs = base_reset(self, *a, **k)
        rt = _RT["rt"]
        if rt is not None:
            if rt.setup != setup:  # first reset of a fresh env
                rt.restore_authored(setup)
                obs = base_reset(self, *a, **k)
            rt.begin_trial(_RT["trials"])
            _RT["trials"] += 1
        return obs

    cls._ensure_sim, cls._task_step, cls.reset = _ensure_sim, _task_step, reset
    cls._pool_patched = True


def _wrap(environment) -> None:
    base = environment.unwrapped
    sim = base.sim
    if _RT["sim"] is sim:
        return
    orig = sim.step

    def step(*a, **k):
        out = orig(*a, **k)
        if _RT["rt"] is not None:
            _RT["rt"].physics(base.physics_dt)
        return out

    sim.step = step
    _RT["sim"] = sim


def install() -> None:
    import importlib.abc
    import importlib.util

    class _Finder(importlib.abc.MetaPathFinder):
        targets = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}

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
    if not any(
        type(f).__name__ == "_Finder" and getattr(f, "targets", None) and "yam_arena.scene" in f.targets
        for f in sys.meta_path
    ):
        sys.meta_path.insert(0, _Finder())


install()
