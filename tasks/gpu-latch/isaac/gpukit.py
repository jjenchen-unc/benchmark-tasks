"""gpukit: the side-open tower with its motherboard, the single-slot GPU seated in the PCIe x16 slot behind the slot's
retention latch, and the anti-static mat, for yam_arena (Isaac Lab-Arena / PhysX), plus the runtime hooks.

Authored as one custom-spawned assembly (yam_arena task-file primitives are single boxes/spheres):
* the benchtable (plate, legs, motherboard, slot housings, CPU cooler, bracket bar) as one kinematic rigid body (its
  frame is the slot frame S of ``../src/layout.py``); the anti-static mat as a second kinematic body on the table;
* the card as a free rigid body (gold-finger tab, retention lip, PCB, cooler shroud, back cover, bracket);
* the latch as a free rigid body on a revolute joint (hard limits 0-22 deg) at its pivot in the slot's end;
Geometry: ``../src/layout.py``; scorer: ``../src/scoring.py`` (pure functions).

Mechanism emulation (computed forces/torques applied every physics step from measured poses; drives on maximal joints
are inert in this PhysX build, so none is used):
* latch torsion spring: tau = -(TAU0 + K theta) - C theta_dot about the pivot (TAU0 0.035 N m, K 0.12 N m/rad), theta
  the measured opening; the closed stop is the joint's lower limit;
* contact rows: Coulomb friction on the card along the slot axis while its gold fingers are between the rows
  (limit 2 x 0.3 x 34 N = 20.4 N for the first 3 mm of the card's rise, then falling to 0 by 4.5 mm); the
  stick phase is a stiff spring to an anchor that slips once the limit is reached. The card sits in the slot's channel
  with 0.5 mm side clearance (rigid, frictionless walls). The friction force acts at the card's centre of mass.
Nothing here writes a card or latch pose or velocity during a trial; ``Runtime.restore_authored`` puts
everything back only on an environment reset. The scripted demo READS poses through ``rig_state()``.
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
import scoring as S  # noqa: E402
from yam_arena.task_file import Box  # noqa: E402

PHYSICS_DT = float(os.environ.get("GPU_PHYSICS_DT", str(1 / 240)))
CONTACT_OFFSET = float(os.environ.get("GPU_CONTACT_OFFSET", ".001"))
SETUP_BASE = int(os.environ.get("GPU_SETUP", "0"))  # GPU_START (offset starts) is read at each reset
FORCE_NAMED = "GPU_SETUP" in os.environ

PCB_GREEN, BOARD_COL, SLOT_BLACK, LATCH_GREY, GOLD, SHROUD_COL, SILVER, STEEL, ALU, MAT_COL, PLATE_COL, FAN, WHITE = (
    (0.10, 0.28, 0.16),
    (0.12, 0.13, 0.15),
    (0.06, 0.06, 0.07),
    (0.80, 0.80, 0.78),
    (0.85, 0.66, 0.22),
    (0.16, 0.17, 0.19),
    (0.72, 0.74, 0.77),
    (0.62, 0.64, 0.67),
    (0.70, 0.72, 0.75),
    (0.30, 0.42, 0.62),
    (0.10, 0.10, 0.11),
    (0.08, 0.08, 0.09),
    (0.93, 0.93, 0.92),
)
DYNAMIC = ("card", "latch")
COM_LOCAL = (np.array(L.CARD_COM) * L.MM, np.array([0.0, 0.0, 0.006]))  # body-frame centres of mass (forces act there)
KINEMATIC = ("bench", "mat")
BODIES = DYNAMIC + KINEMATIC


class Rig(Box):
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
    """Authored (reset) pose of every body for a setup: {name: (pos, rot)} in the env frame."""
    spec = L.setup_spec(setup)
    ps, rs = L.bench_frame(spec)
    out = {
        "bench": (ps, rs),
        "card": (ps.copy(), rs.copy()),
        "latch": (ps + rs @ np.array([L.PIVOT[0], 0.0, L.PIVOT[1]]) * L.MM, rs.copy()),
        "mat": (np.array([L.MAT_CENTER[0], L.MAT_CENTER[1], L.TOP]), np.eye(3)),
    }
    return out


_SPEC: dict = {}


def _spawn_rig(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
    from scipy.spatial import ConvexHull
    from yam_arena.station import linear_rgb

    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    ox, oy, oz, ow = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(ow, ox, oy, oz))
    looks, mats = {}, {}

    def look(color, opacity=1.0):
        key = (color, opacity)
        if key not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.4, opacity=opacity)
            vis.func(path, vis)
            looks[key] = path
        return looks[key]

    def material(friction, combine=None):
        key = (friction, combine)
        if key not in mats:
            path = f"{prim_path}/Looks/phys{len(mats)}"
            pm = sim_utils.RigidBodyMaterialCfg(static_friction=friction, dynamic_friction=friction)
            pm.func(path, pm)
            if combine:
                PhysxSchema.PhysxMaterialAPI.Apply(stage.GetPrimAtPath(path)).CreateFrictionCombineModeAttr().Set(combine)
            mats[key] = path
        return mats[key]

    def xform(path, pos, rot):
        x = UsdGeom.Xform.Define(stage, path)
        x.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float, pos)))
        q = _quat(rot)
        x.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(q[3]), *map(float, q[:3])))
        return x

    def rigid(path, mass=None, kinematic=False, com=None, inertia=None, iters=32):
        prim = stage.GetPrimAtPath(path)
        rb = UsdPhysics.RigidBodyAPI.Apply(prim)
        if kinematic:
            rb.CreateKinematicEnabledAttr(True)
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(float(mass if mass is not None else 1.0))
        if com is not None:
            m.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
        if inertia is not None:
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, inertia)))
        px = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        px.CreateMaxDepenetrationVelocityAttr(0.2)
        px.CreateSolverPositionIterationCountAttr(iters)
        px.CreateSolverVelocityIterationCountAttr(4)
        px.CreateLinearDampingAttr(0.0)
        px.CreateAngularDampingAttr(0.05)
        px.CreateSleepThresholdAttr(0.0)  # never sleeps: a sleeping body reports no contacts
        px.CreateStabilizationThresholdAttr(0.0)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)

    count = {"n": 0}

    def hull(path, v, color, friction=0.5, collide=True, combine=None, patch=None):
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
        if collide:
            UsdPhysics.CollisionAPI.Apply(prim)
            cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            cp.CreateContactOffsetAttr(CONTACT_OFFSET)
            cp.CreateRestOffsetAttr(0.0)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
            hc = PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim)
            hc.CreateHullVertexLimitAttr(64)
            hc.CreateMinThicknessAttr(0.0001)
            if patch is not None:
                cp.CreateTorsionalPatchRadiusAttr(float(patch))
                cp.CreateMinTorsionalPatchRadiusAttr(float(patch))
            bind_physics_material(str(prim.GetPath()), material(friction, combine), stage=stage)
            count["n"] += 1
        bind_visual_material(str(prim.GetPath()), look(color), stage=stage)
        return m

    def ub(path, u0, u1, n0, n1, z0, z1, color, friction=0.5, collide=True, combine=None, patch=None):
        return hull(path, L.ubox(u0, u1, n0, n1, z0, z1), color, friction, collide, combine, patch)

    def cyl_u(path, r, uc, nc, zc, h, color, axis="n", collide=False):
        """Cylinder (mm) with its axis along n (or z) centred at (uc, nc, zc)."""
        a = np.arange(20) * 2 * math.pi / 20
        if axis == "n":
            v = [[uc + r * math.cos(t), nc + s * h / 2, zc + r * math.sin(t)] for t in a for s in (-1, 1)]
        else:
            v = [[uc + r * math.cos(t), nc + r * math.sin(t), zc + s * h / 2] for t in a for s in (-1, 1)]
        return hull(path, np.array(v) * L.MM, color, collide=collide)

    A = authored(SETUP_BASE)
    # ---------------------------------------------------------------- benchtable (kinematic), frame S
    xform(f"{prim_path}/bench", *A["bench"])
    rigid(f"{prim_path}/bench", 5.0, kinematic=True)
    b = f"{prim_path}/bench"
    ub(f"{b}/plate", *L.PLATE, PLATE_COL)
    zt = -(L.BOARD_Z - L.TOP) / L.MM  # table top in S (mm)
    for i, (u, n) in enumerate(((-20, -75), (-20, 260), (252, -75), (252, 260))):
        ub(f"{b}/leg{i}", u - 8, u + 8, n - 8, n + 8, zt, L.PLATE[4], PLATE_COL)
    for i, (u, n) in enumerate(((6, -50), (6, 235), (230, -50), (230, 235), (120, 100))):
        cyl_u(f"{b}/standoff{i}", 3.0, u, n, (L.PLATE[5] - L.BOARD_T) / 2, -L.PLATE[5] - L.BOARD_T, (0.8, 0.7, 0.3),
              axis="z")
    ub(f"{b}/board", L.BOARD[0], L.BOARD[1], L.BOARD[2], L.BOARD[3], -L.BOARD_T, 0.0, BOARD_COL)
    for name, bounds in L.CASE_WALLS.items():
        ub(f"{b}/case_{name}", *bounds, PLATE_COL)
    # Separate bracket jambs; the card slides through the exposed slot opening.
    for k, (n0, n1) in enumerate(((-80, -16.5), (3.5, 60))):
        ub(f"{b}/jamb_{k}", -12, -1.5, n0, n1, 0, 119.2, ALU)
    for k, g in enumerate(L.GUIDES):  # frictionless locating guides (a bracket slides freely between dividers)
        ub(f"{b}/guide{k}", *g, ALU, friction=0.0, combine="min")
    # PCIe x16 slot of the card: floor, side walls (beside the contact rows), bracket-end wall, short latch-end wall
    s0, s1, hw, ch = L.SLOT_U[0], L.SLOT_U[1], L.SLOT_HALF_W, L.CHANNEL_HALF
    ub(f"{b}/slot_floor", s0, s1, -hw, hw, 0.0, L.SLOT_FLOOR, SLOT_BLACK)
    # the channel and end walls only locate the card (frictionless: the contact rows' grip is the friction model)
    ub(f"{b}/slot_wall_p", s0, s1, ch, hw, 0.0, L.SLOT_TOP, SLOT_BLACK, friction=0.0, combine="min")
    ub(f"{b}/slot_wall_m", s0, s1, -hw, -ch, 0.0, L.SLOT_TOP, SLOT_BLACK, friction=0.0, combine="min")
    ub(f"{b}/slot_end0", s0, L.FINGER[0] - 0.5, -ch, ch, L.SLOT_FLOOR, L.SLOT_TOP - 0.5, SLOT_BLACK, friction=0.0,
       combine="min")
    ub(f"{b}/slot_end1", L.FINGER[1] + 0.5, s1 + 1.0, -ch, ch, L.SLOT_FLOOR, 4.6, SLOT_BLACK, friction=0.0, combine="min")
    ub(f"{b}/latch_base", 138.0, 154.0, -hw, hw, 0.0, 0.5, SLOT_BLACK)
    ub(f"{b}/latch_cheek_p", 138.0, 144.5, hw, hw + 1.0, 0.0, 5.0, SLOT_BLACK)
    ub(f"{b}/latch_cheek_m", 138.0, 144.5, -hw - 1.0, -hw, 0.0, 5.0, SLOT_BLACK)
    # neighbouring parts (finished surroundings): an empty x16 slot two slots toward the CPU, an x1 slot on the
    # cooler side, the CPU's low-profile cooler and the 24-pin power connector
    ub(f"{b}/slot2", s0, s1, 40.64 - hw, 40.64 + hw, 0.0, L.SLOT_TOP, SLOT_BLACK)
    ub(f"{b}/slot2_latch", 139.0, 151.0, 40.64 - 4.0, 40.64 + 4.0, 0.0, 11.5, LATCH_GREY)
    ub(f"{b}/slot_x1", s0, s0 + 25.0, -20.32 - hw, -20.32 + hw, 0.0, L.SLOT_TOP, SLOT_BLACK)
    ub(f"{b}/cooler_base", 75.0, 170.0, 135.0, 230.0, 0.0, 6.0, SILVER)
    ub(f"{b}/cooler_fins", 70.0, 175.0, 130.0, 235.0, 6.0, 37.0, (0.55, 0.36, 0.25))
    ub(f"{b}/atx24", 215.0, 238.0, 150.0, 205.0, 0.0, 14.0, WHITE)
    ub(f"{b}/chipset", 150.0, 190.0, 60.0, 100.0, 0.0, 8.0, SILVER, collide=False)
    for k in range(3):  # gold contact lines inside the slot (visual)
        ub(f"{b}/slot_mark{k}", s0 + 2, s1 - 2, -hw - 0.01, -hw, 2 + 3 * k, 2.5 + 3 * k, (0.25, 0.25, 0.27),
           collide=False)
    # ---------------------------------------------------------------- anti-static mat (kinematic, on the table)
    xform(f"{prim_path}/mat", *A["mat"])
    rigid(f"{prim_path}/mat", 1.0, kinematic=True)
    mx, my = L.MAT_HALF
    hull(f"{prim_path}/mat/sheet", L.box_corners(-mx, mx, -my, my, 0.0, L.MAT_T), MAT_COL, friction=0.8)
    hull(f"{prim_path}/mat/snap", L.box_corners(mx - 0.03, mx - 0.015, my - 0.03, my - 0.015, L.MAT_T, L.MAT_T + 0.003),
         SILVER, collide=False)
    # ---------------------------------------------------------------- latch (dynamic, revolute about n at the pivot)
    xform(f"{prim_path}/latch", *A["latch"])
    rigid(f"{prim_path}/latch", L.LATCH_MASS, com=(0.0, 0.0, 0.006),
          inertia=(L.LATCH_INERTIA, L.LATCH_INERTIA, L.LATCH_INERTIA))
    lp = f"{prim_path}/latch"
    for k, v in L.latch_pieces().items():
        hull(f"{lp}/{k}", v, LATCH_GREY, friction=0.4)
    j = UsdPhysics.RevoluteJoint.Define(stage, f"{lp}/pivot")
    j.CreateAxisAttr("Y")
    j.CreateBody0Rel().SetTargets([Sdf.Path(b)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(lp)])
    j.CreateLocalPos0Attr(Gf.Vec3f(float(L.PIVOT[0] * L.MM), 0.0, float(L.PIVOT[1] * L.MM)))
    j.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    j.CreateLowerLimitAttr(float(L.LATCH_LIMIT_DEG[0]))
    j.CreateUpperLimitAttr(float(L.LATCH_LIMIT_DEG[1]))
    UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(lp)).CreateFilteredPairsRel().AddTarget(b)
    # ---------------------------------------------------------------- the card (dynamic), frame K
    xform(f"{prim_path}/card", *A["card"])
    rigid(f"{prim_path}/card", L.CARD_MASS, com=np.array(L.CARD_COM) * L.MM, inertia=L.CARD_INERTIA)
    cp = f"{prim_path}/card"
    pieces = L.card_pieces()
    hull(f"{cp}/finger", pieces["finger"], GOLD, friction=L.CONTACT_FRICTION, combine="min")
    hull(f"{cp}/lip", pieces["lip"], PCB_GREEN, friction=0.4)
    hull(f"{cp}/notch", pieces["notch"], PCB_GREEN, friction=0.4)
    hull(f"{cp}/pcb_a", pieces["pcb_a"], PCB_GREEN, friction=0.5)
    hull(f"{cp}/pcb_b", pieces["pcb_b"], PCB_GREEN, friction=0.5)
    hull(f"{cp}/shroud", pieces["shroud"], SHROUD_COL, friction=L.SHELL_FRICTION, combine="max")
    hull(f"{cp}/back", pieces["back"], (0.20, 0.21, 0.23), friction=L.SHELL_FRICTION, combine="max")
    hull(f"{cp}/bracket", pieces["bracket"], SILVER, friction=0.4)
    hull(f"{cp}/flange", pieces["flange"], SILVER, friction=0.4)
    # visual details: blower fan, silver trim stripes, four DisplayPort sockets on the bracket, the gold pads
    cyl_u(f"{cp}/fan", 30.0, 200.0, -15.35, 64.0, 0.4, FAN)
    cyl_u(f"{cp}/fan_hub", 11.0, 200.0, -15.5, 64.0, 0.4, SILVER)
    ub(f"{cp}/trim0", 8.0, 160.0, -15.5, -15.3, 100.0, 104.0, SILVER, collide=False)
    ub(f"{cp}/trim1", 8.0, 160.0, -15.5, -15.3, 24.0, 28.0, SILVER, collide=False)
    for k in range(4):
        ub(f"{cp}/dp{k}", -1.1, -0.9, -12.0, -2.0, 22.0 + 20.0 * k, 32.0 + 20.0 * k, (0.05, 0.05, 0.05), collide=False)
    for k in range(2):
        sg = 1 if k else -1
        ub(f"{cp}/pads{k}", 46.0, 133.0, sg * 0.79, sg * 0.80, 4.0, 10.5, GOLD, collide=False)
    _SPEC["paths"] = {n: f"{prim_path}/{n}" for n in BODIES}
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
    class RigSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_rig)

    return Object(
        name=spec.name,
        object_type=ObjectType.BASE,
        spawner_cfg=RigSpawnerCfg(),
        initial_pose=Pose(position_xyz=(0.0, 0.0, 0.0), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)),
    )


# ---------------------------------------------------------------------------------------------- runtime state
STATE: dict = dict(score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, trial=-1, setup=None)


def finger_tips(environment):
    """Pad-tip centres of all four robot fingers (env frame)."""
    from yam_arena.constants import PAD_TIP_PAST_TCP, TCP_OFFSET

    base = environment.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
    pts = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        pose = robot.data.body_link_pose_w[0].detach().cpu().numpy().astype(float)[
            list(robot.body_names).index("gripper")
        ]
        jp = robot.data.joint_pos[0].detach().cpu().numpy().astype(float)
        jn = list(robot.joint_names)
        half = max(abs(jp[jn.index(j)]) for j in ("joint7", "joint8"))
        r = _rot(pose[3:7])
        tip = pose[:3] + r @ (np.array(TCP_OFFSET) + [0, 0, PAD_TIP_PAST_TCP]) - origin
        pts += [tip + r[:, 0] * s * (half + 0.0032) for s in (1.0, -1.0)]
    return pts


class Runtime:
    """Computed latch spring torque and contact-row preload every physics step; scoring every control step."""

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
        self.stage = get_current_stage()
        stage_id = UsdUtils.StageCache.Get().GetId(self.stage).ToLongInt()
        self.simview = tensors.create_simulation_view("torch", stage_id=stage_id)
        self.simview.set_subspace_roots("/")
        root = "/World/envs/env_0/rig"
        self.root = root
        P = lambda n: f"{root}/{n}"  # noqa: E731
        self.view = self.simview.create_rigid_body_view([P(n) for n in BODIES])
        self.dyn = self.simview.create_rigid_body_view([P(n) for n in DYNAMIC])
        self.kin = self.simview.create_rigid_body_view([P(n) for n in KINEMATIC])
        robot_links = []
        for arm in ("left_robot", "right_robot"):
            lp = self.base.scene[arm].root_view.link_paths
            robot_links += list(lp[0] if lp and isinstance(lp[0], (list, tuple)) else lp)
        self.robot_links = robot_links
        self.cv_grip = self.simview.create_rigid_contact_view(
            [P("card")], filter_patterns=[robot_links], max_contact_data_count=4096
        )
        self.cv_support = self.simview.create_rigid_contact_view(
            [P("card")], filter_patterns=[[P("bench"), P("mat")]], max_contact_data_count=1024
        )
        self.cv_hook = self.simview.create_rigid_contact_view(
            [P("latch")], filter_patterns=[[P("card")]], max_contact_data_count=1024
        )
        self.cv_press = self.simview.create_rigid_contact_view(
            [P("latch")], filter_patterns=[robot_links], max_contact_data_count=1024
        )
        self.origin = self.base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        self.torch_f = torch.zeros((len(DYNAMIC), 3), dtype=torch.float32)
        self.force_idx = torch.arange(len(DYNAMIC), dtype=torch.int32)
        self.kin_idx = torch.arange(len(KINEMATIC), dtype=torch.int32)
        self.extra = {}  # development probes only: {'card'|'latch': force vector or callable}
        self.setup = SETUP_BASE
        self.spec = L.setup_spec(self.setup)
        self.trial_active = False
        self.rest_kin = None
        self.theta = 0.0
        self.anchor = None
        self.contact_f = 0.0
        self.prev_rise = None
        self.prev_in_slot = True
        self._init_state()

    def _init_state(self):
        self.scorer = S.Scorer()
        self.steps = 0
        self.t = 0.0
        self.hook_s = 0.0
        self.nonfinite = 0
        self.grip_peak = 0.0
        self.grip_link = None
        self.grip_sum = None
        self.grip_n = 0
        self.grip_sustained = 0.0
        self.hook_sum, self.hook_k = 0.0, 0
        self.hook_step_peak = 0.0
        self.press_sum, self.press_peak, self.press_sustained = 0.0, 0.0, 0.0
        self.fall_max = 0.0
        self.forced = False
        self.open_before_lift = False
        self.last_lip_rise = None
        self.flight_top = None
        self.max_descent = 0.0
        self.mat_load = 0.0
        self.robot_contact = False
        self.grip_now = {}
        self.grip_fric = [0, 0, 0]
        self.theta_peak = 0.0
        self.pull_sum, self.pull_sustained = 0.0, 0.0
        STATE.update(score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, setup=self.setup)

    def begin_trial(self, trial):
        self._init_state()
        self.trial_active = True
        STATE.update(trial=trial, setup=self.setup, start=self.spec)

    # ------------------------------------------------------------------ reading
    def read(self):
        tf = self.view.get_transforms().detach().cpu().numpy().astype(float)
        return {n: (tf[i, :3] - self.origin, _rot(tf[i, 3:7])) for i, n in enumerate(BODIES)}

    # ------------------------------------------------------------------ physics hook
    def physics(self, dt):
        t0 = time.perf_counter()
        self.t += dt
        tf = self.dyn.get_transforms().detach().cpu().numpy().astype(float)
        v = self.dyn.get_velocities().detach().cpu().numpy().astype(float)
        kt = self.kin.get_transforms().detach().cpu().numpy().astype(float)
        rs = _rot(kt[0, 3:7])
        rl = _rot(tf[1, 3:7])
        th = L.latch_angle(rl, rs)
        self.theta = th
        self.theta_peak = max(self.theta_peak, th)
        nax = rs[:, 1]
        w = float(v[1, 3:6] @ nax)
        f = np.zeros((len(DYNAMIC), 3))
        tq = np.zeros((len(DYNAMIC), 3))
        tq[1] = (-(L.LATCH_TAU0 + L.LATCH_K * max(th, 0.0)) - L.LATCH_C * w) * nax
        # contact rows: Coulomb friction along the slot axis with a stick spring to an anchor
        ps0, zs = kt[0, :3], rs[:, 2]
        rk = _rot(tf[0, 3:7])
        rise = L.finger_bottom_rise(tf[0, :3], rk, ps0, rs)
        fs = L.WITHDRAW_N * L.contact_fraction(max(rise, 0.0))
        zc = float((tf[0, :3] - ps0) @ zs)
        vz = float(v[0, :3] @ zs)
        eng = L.engaged(tf[0, :3], rk, ps0, rs)
        ins = L.in_slot(tf[0, :3], rk, ps0, rs)
        lip = np.array([L.LIP[1], 0, L.LIP[3]]) * L.MM
        lip_rise = float((tf[0, :3] + rk @ lip - ps0) @ zs - lip[2])
        clear = math.degrees(th) >= L.HOOK_CLEAR_DEG
        if lip_rise <= 0.0005:
            self.open_before_lift = clear
        elif not clear and lip_rise < L.HOOK_PASS_RISE:
            self.open_before_lift = False
        if (self.trial_active and self.last_lip_rise is not None and self.prev_in_slot
                and self.last_lip_rise < L.HOOK_PASS_RISE <= lip_rise and not self.open_before_lift):
            self.forced = True
        self.last_lip_rise = lip_rise
        self.prev_rise = rise
        self.prev_in_slot = ins
        if not eng:
            fs = 0.0
        if self.anchor is None or fs <= 0.0:
            self.anchor = zc
            fc = 0.0
        else:
            d = zc - self.anchor
            if L.STICK_K * abs(d) > fs:
                self.anchor = zc - math.copysign(fs / L.STICK_K, d)
                d = zc - self.anchor
            fc = float(np.clip(-L.STICK_K * d - L.STICK_C * vz, -fs, fs))
        self.contact_f = fc
        f[0] += fc * zs  # applied at the card's centre of mass (no couple)
        for name, fv in self.extra.items():  # development probes only
            k = DYNAMIC.index(name)
            if callable(fv):
                fv, at = fv(tf[k, :3] - self.origin, _rot(tf[k, 3:7]), v[k], self)
                com = tf[k, :3] - self.origin + _rot(tf[k, 3:7]) @ COM_LOCAL[k]
                tq[k] += np.cross(np.asarray(at, float) - com, fv)
            f[k] += np.asarray(fv, float)
        self.dyn.apply_forces_and_torques_at_position(
            self.torch.as_tensor(f, dtype=self.torch.float32), self.torch.as_tensor(tq, dtype=self.torch.float32),
            None, self.force_idx, True,
        )
        if self.trial_active:
            self.fall_max = max(self.fall_max, float(-v[0, 2]))
            gm = self.cv_grip.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
            support = self.cv_support.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
            self.mat_load = max(self.mat_load, abs(float(support[0, 1, 2])))
            self.robot_contact |= bool(np.linalg.norm(gm) > 0.05)
            z = float((tf[0, :3] + rk @ COM_LOCAL[0])[2])
            if ins or np.linalg.norm(gm) > 0.05 or np.linalg.norm(support) > 0.05:
                self.flight_top = None
            else:
                self.flight_top = z if self.flight_top is None else max(self.flight_top, z)
                self.max_descent = max(self.max_descent, self.flight_top - z)
            if gm.size:
                per = np.linalg.norm(gm[0], axis=-1)
                j = int(np.argmax(per))
                if per[j] > self.grip_peak:
                    self.grip_peak = float(per[j])
                    self.grip_link = self.robot_links[j].split("/")[-1]
                self.grip_sum = per if self.grip_sum is None else self.grip_sum + per
                fr, _, fcnt, fstart = (x.detach().cpu().numpy() for x in self.cv_grip.get_friction_data(dt=dt))
                self.grip_fric = np.round(fr[: int(fcnt.sum())].sum(axis=0), 2).tolist() if fcnt.sum() else [0, 0, 0]
                self.grip_now = {self.robot_links[i].split("/")[-2] + "/" + self.robot_links[i].split("/")[-1]:
                                 np.round(gm[0][i], 1).tolist() for i in np.nonzero(per > 1.0)[0]}
                self.grip_n += 1
            hm = self.cv_hook.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
            hn = float(np.linalg.norm(hm.reshape(-1, 3).sum(axis=0))) if hm.size else 0.0
            self.hook_sum += hn
            self.hook_k += 1
            self.hook_step_peak = max(self.hook_step_peak, hn)
            pm = self.cv_press.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
            pn = float(np.linalg.norm(pm.reshape(-1, 3).sum(axis=0))) if pm.size else 0.0
            self.press_sum += pn
            self.press_peak = max(self.press_peak, pn)
        self.hook_s += time.perf_counter() - t0

    # ------------------------------------------------------------------ reset
    def restore_authored(self, setup, override=None):
        """Environment reset only (never during a trial): every body to the setup's authored pose."""
        torch = self.torch
        self.setup = setup
        self.spec = L.setup_spec(setup)
        self.trial_active = False
        self.anchor = None
        self.prev_rise = None
        self.prev_in_slot = True
        a = authored(setup)
        if callable(override):
            override = override(self.spec)
        if override:
            a.update(override)
        tf = self.view.get_transforms().clone()
        for i, n in enumerate(BODIES):
            pos, rot = a[n]
            tf[i, :3] = torch.tensor(np.asarray(pos, float) + self.origin, dtype=tf.dtype)
            tf[i, 3:7] = torch.tensor(_quat(rot), dtype=tf.dtype)
        idx = torch.tensor(list(range(len(BODIES))), dtype=torch.int32, device=tf.device)
        self.view.set_transforms(tf, idx)
        dyn = torch.tensor([BODIES.index(n) for n in DYNAMIC], dtype=torch.int32, device=tf.device)
        self.view.set_velocities(torch.zeros_like(self.view.get_velocities()), dyn)
        self.rest_kin = torch.stack([tf[BODIES.index(n)] for n in KINEMATIC]).clone()
        try:
            self.kin.set_kinematic_targets(self.rest_kin.clone(), self.kin_idx)
        except Exception as e:  # noqa: BLE001  (first reset before the views are live)
            print("GPU kinematic targets at reset:", e, flush=True)
        self._init_state()

    # ------------------------------------------------------------------ scoring
    def measure(self, environment, hook_n, fall_v):
        poses = self.read()
        pk, rk = poses["card"]
        ps, rs = poses["bench"]
        rise = L.finger_bottom_rise(pk, rk, ps, rs)
        rel = L.card_in_slot(pk, rk, ps, rs)
        vertices = L.card_vertices(pk, rk)
        flat_c = abs(float(rk[2, 1]))
        tips = finger_tips(environment)
        tip = min((L.point_to_card(t, pk, rk) for t in tips), default=0.0)
        velocity = self.dyn.get_velocities().detach().cpu().numpy().astype(float)[0]
        com = pk + rk @ COM_LOCAL[0]
        inside, gap, on_mat, max_speed, released = S.placement(
            vertices, com, velocity, np.array(L.MAT_CENTER), np.array(L.MAT_HALF), L.MAT_TOP,
            flat_c, self.mat_load, self.robot_contact, tip,
        )
        still = max_speed < S.STILL_V
        detail = dict(rise_mm=round(rise * 1e3, 3), rel_mm={k: round(x * 1e3, 2) if k in ("du", "dn", "dz") else
                                                              round(x, 2) for k, x in rel.items()},
                      latch_deg=round(math.degrees(self.theta), 2), mat_inside=inside, low_mm=round(gap * 1e3, 2), flat_cos=round(flat_c, 4), tip_mm=round(tip * 1e3, 2),
                      card_mm=np.round(pk * 1e3, 2).tolist(), card_v_mm_s=round(float(np.linalg.norm(velocity[:3])) * 1e3, 2),
                      vertex_v_mm_s=round(max_speed * 1e3, 2), mat_load_n=round(self.mat_load, 3),
                      robot_contact=self.robot_contact)
        m = S.Measure(out=not L.in_slot(pk, rk, ps, rs), on_mat=on_mat, released=released, still=still,
                      hook_n=hook_n, fall_v=fall_v, forced=self.forced, detail=detail,
                      unsupported_descent_m=self.max_descent)
        return m, poses

    def control(self, environment):
        if not self.trial_active:
            return
        self.steps += 1
        hook_n = self.hook_sum / self.hook_k if self.hook_k else 0.0
        fall_v = self.fall_max
        press = self.press_sum / self.hook_k if self.hook_k else 0.0
        self.press_sustained = max(self.press_sustained, press)
        self.hook_sum, self.hook_k, self.fall_max, self.press_sum = 0.0, 0, 0.0, 0.0
        m, poses = self.measure(environment, hook_n, fall_v)
        self.mat_load = 0.0
        self.robot_contact = False
        pk, rk = poses["card"]
        if not (np.all(np.isfinite(pk)) and np.all(np.isfinite(rk))):
            self.nonfinite += 1
            STATE["nonfinite"] = self.nonfinite
            return
        level = self.scorer.step(m)
        if self.grip_n:
            self.grip_sustained = max(self.grip_sustained, float((self.grip_sum / self.grip_n).max()))
        self.grip_sum, self.grip_n = None, 0
        STATE.update(
            score=level,
            level=level,
            steps=self.steps,
            nonfinite=self.nonfinite,
            detail=dict(
                self.scorer.last,
                grip_peak_n=round(self.grip_peak, 2),
                grip_link=self.grip_link,
                grip_sustained_n=round(self.grip_sustained, 2),
                hook_step_peak_n=round(self.hook_step_peak, 2),
                press_now_n=round(press, 2),
                press_peak_n=round(self.press_peak, 2),
                press_sustained_n=round(self.press_sustained, 2),
                theta_peak_deg=round(math.degrees(self.theta_peak), 2),
                contact_f_n=round(self.contact_f, 2),
                grip_links=self.grip_now,
                grip_fric=self.grip_fric,
                setup=self.setup,
            ),
        )


_RT: dict = {"rt": None, "sim": None, "trials": 0, "override": None}


def runtime():
    return _RT["rt"]


def rig_state():
    """Current poses of every body for the scripted demo (read-only), plus the start spec and the latch angle."""
    rt = _RT["rt"]
    if rt is None:
        return None
    out = rt.read()
    out["spec"] = rt.spec
    out["latch_deg"] = math.degrees(rt.theta)
    return out


def _patch_scene(scene) -> None:
    if getattr(scene, "_gpu_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Rig):
            return _arena_object(spec)
        return orig_task_object(spec)

    def object_states(environment, names):
        from yam_arena.task_file import ObjectState

        plain = [n for n in names if n != "rig"]
        out = orig_states(environment, plain) if plain else {}
        if "rig" in names:
            rt = _RT["rt"]
            p, r = rt.read()["card"] if rt else authored(SETUP_BASE)["card"]
            z = np.zeros(3)
            out["rig"] = ObjectState(pos=np.asarray(p), quat_xyzw=_quat(r), lin_vel=z, ang_vel=z, low=p - 0.1,
                                     high=p + 0.1)
        return {n: out[n] for n in names}

    orig_build = scene.build_task_environment

    def build(embodiment, task, control_hz=None):
        environment = orig_build(embodiment, task, control_hz)
        if any(isinstance(o, Rig) for o in task.objects):
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
    scene._gpu_patched = True


def _has_rig(emb):
    return any(isinstance(o, Rig) for o in (emb.task.objects if emb.task is not None else ()))


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_gpu_patched", False):
        return
    base_task_step, base_reset, base_ensure = cls._task_step, cls.reset, cls._ensure_sim

    def _ensure_sim(self, *a, **k):
        env_before = self._env
        out = base_ensure(self, *a, **k)
        if self._env is not None and self._env is not env_before and _has_rig(self):
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
        starts = [x for x in os.environ.get("GPU_START", "").split(";") if x.strip()]
        if starts:  # verification aid: per-trial starts given as "along_mm,across_mm,dyaw_deg" offsets from the official
            setup = ("offset", *(float(v) for v in starts[_RT["trials"] % len(starts)].split(",")))
        elif seed is not None and not FORCE_NAMED:
            setup = ("seed", int(seed))
        else:
            setup = (SETUP_BASE + _RT["trials"]) % len(L.SETUPS)
        if rt is not None:
            rt.restore_authored(setup, _RT["override"])
        obs = base_reset(self, *a, **k)
        rt = _RT["rt"]
        if rt is not None:
            if rt.setup != setup:  # first reset of a fresh env
                rt.restore_authored(setup, _RT["override"])
                obs = base_reset(self, *a, **k)
            rt.restore_authored(setup, _RT["override"])
            rt.begin_trial(_RT["trials"])
            _RT["trials"] += 1
        return obs

    cls._ensure_sim, cls._task_step, cls.reset = _ensure_sim, _task_step, reset
    cls._gpu_patched = True


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
        targets = {
            "yam_arena.scene": _patch_scene,
            "yam_arena.inspect_embodiment": _patch_embodiment,
        }

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
        type(f).__name__ == "_Finder"
        and getattr(f, "targets", None)
        and "yam_arena.scene" in f.targets
        for f in sys.meta_path
    ):
        sys.meta_path.insert(0, _Finder())


install()
