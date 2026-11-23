"""flaskkit: the benchtop orbital shaker with its four 250 mL spring-finger flask clamps (three loaded with capped
culture flasks, one free), the empty 250 mL Erlenmeyer flask on the bench, for yam_arena (Isaac Lab-Arena / PhysX),
plus the runtime hooks.

Authored as one custom-spawned assembly (yam_arena task-file primitives are single boxes/spheres):
* the shaker housing, its platform, the free clamp's base plate, and each loaded clamp together with its flask as
  kinematic rigid bodies (PhysX contact views can report forces on them; the platform and everything on it are moved
  on the shaker's orbit only during the shake test);
* the free clamp's five stainless fingers as free rigid bodies, each on a revolute joint (hard limits) at its foot on
  the base plate;
* the empty flask as a free rigid body (three convex pieces: heel and cone, neck, lip).
Geometry: ``../src/layout.py``; seat rule, shake test and scorer: ``../src/scoring.py`` (pure functions).

Mechanism emulation (computed torques, applied every physics step from the measured finger poses; drives on maximal
joints are inert in this PhysX build, so none is used): each finger is a leaf spring about its foot,
tau = -K theta - C theta_dot (K 0.55 N m/rad, C 0.004 N m s/rad), theta = the measured outward swing from the formed
(free) shape. Nothing here writes a flask or finger pose or velocity during a trial; ``Runtime.restore_authored`` puts
everything back only on an environment reset. During the shake test the platform, the free clamp's base and the
loaded clamps follow the orbit as kinematic targets; the fingers and the flask follow through their joints and
contacts. The scripted demo READS poses through ``bench_state()``.
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

PHYSICS_DT = float(os.environ.get("FLASK_PHYSICS_DT", str(1 / 240)))
CONTACT_OFFSET = float(os.environ.get("FLASK_CONTACT_OFFSET", ".001"))
SETUP_BASE = int(os.environ.get("FLASK_SETUP", "0"))
FORCE_NAMED = "FLASK_SETUP" in os.environ
DEBUG = bool(os.environ.get("FLASK_DEBUG"))

STEEL, STEEL_DARK, HOUSING, PLATFORM_COL, MAT, GLASS, FOIL, MEDIUM, PANEL, LED, WHITE = (
    (0.78, 0.80, 0.82),
    (0.55, 0.57, 0.60),
    (0.88, 0.89, 0.90),
    (0.62, 0.64, 0.66),
    (0.20, 0.21, 0.23),
    (0.80, 0.90, 0.95),
    (0.85, 0.86, 0.88),
    (0.93, 0.70, 0.25),
    (0.12, 0.14, 0.18),
    (0.20, 0.85, 0.35),
    (0.95, 0.95, 0.93),
)
FINGERS = tuple(f"finger{i}" for i in range(L.N_FINGERS))
LOADED = ("loaded0", "loaded1", "loaded2")
KINEMATIC = ("shaker", "platform", "clamp") + LOADED
MOVING = ("platform", "clamp") + LOADED  # follow the orbit in the shake test
DYNAMIC = ("flask",) + FINGERS
BODIES = DYNAMIC + KINEMATIC
LOADED_YAW = (12.0, 48.0, 30.0)


class Bench(Box):
    """The bench, as one task-file object (subclasses Box only to pass yam_arena's checks)."""


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


def finger_rest(i):
    """Finger i's hinge position and rest rotation in the free clamp's body frame."""
    return L.finger_frame(i, 0.0)


def authored(setup):
    """Authored (reset) pose of every body for a setup: {name: (pos, rot)} in the env frame."""
    spec = L.setup_spec(setup)
    pxy = L.platform_xy(spec)
    rc = L.rz(spec["clamp_yaw"])
    pc = L.socket_center(spec, spec["socket"])
    out = {
        "shaker": (np.array([pxy[0], pxy[1], L.TOP]), np.eye(3)),
        "platform": (np.array([pxy[0], pxy[1], L.PLATFORM_Z]), np.eye(3)),
        "clamp": (pc, rc),
        "flask": L.flask_start(spec),
    }
    for i, f in enumerate(FINGERS):
        hp, hr = finger_rest(i)
        out[f] = (pc + rc @ hp, rc @ hr)
    for k, s in enumerate(L.loaded_sockets(spec)):
        out[LOADED[k]] = (L.socket_center(spec, s), L.rz(LOADED_YAW[k]))
    return out


_SPEC: dict = {}


def _spawn_bench(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
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
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.35, opacity=opacity)
            vis.func(path, vis)
            looks[key] = path
        return looks[key]

    def material(friction):
        if friction not in mats:
            path = f"{prim_path}/Looks/phys{len(mats)}"
            pm = sim_utils.RigidBodyMaterialCfg(static_friction=friction, dynamic_friction=friction)
            pm.func(path, pm)
            if friction == L.FINGER_FRICTION:  # stainless fingers on glass: the fingers' own value, not the average
                PhysxSchema.PhysxMaterialAPI.Apply(stage.GetPrimAtPath(path)).CreateFrictionCombineModeAttr().Set("min")
            mats[friction] = path
        return mats[friction]

    def xform(path, pos, rot):
        x = UsdGeom.Xform.Define(stage, path)
        x.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float, pos)))
        q = _quat(rot)
        x.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(q[3]), *map(float, q[:3])))
        return x

    def rigid(path, mass=None, kinematic=False, com=None, inertia=None):
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
        px.CreateSolverPositionIterationCountAttr(32)
        px.CreateSolverVelocityIterationCountAttr(4)
        px.CreateLinearDampingAttr(0.0)
        px.CreateAngularDampingAttr(0.05)
        px.CreateSleepThresholdAttr(0.0)  # never sleeps: a sleeping body reports no contacts
        px.CreateStabilizationThresholdAttr(0.0)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)

    count = {"n": 0}

    def hull(path, v, color, friction=0.5, collide=True, opacity=1.0, patch=None):
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
            if patch is not None:  # torsional friction of an area contact (gripper pads on the neck)
                cp.CreateTorsionalPatchRadiusAttr(float(patch))
                cp.CreateMinTorsionalPatchRadiusAttr(float(patch))
            bind_physics_material(str(prim.GetPath()), material(friction), stage=stage)
            count["n"] += 1
        bind_visual_material(str(prim.GetPath()), look(color, opacity), stage=stage)
        return m

    def box(path, x0, x1, y0, y1, z0, z1, color, friction=0.5, collide=True):
        return hull(path, L.box_corners(x0, x1, y0, y1, z0, z1), color, friction, collide)

    def disc(path, r, z0, z1, color, friction=0.5, collide=True, n=24, opacity=1.0, center=(0.0, 0.0)):
        v = np.vstack([L.ring(r, z0, n), L.ring(r, z1, n)])
        v[:, 0] += center[0]
        v[:, 1] += center[1]
        return hull(path, v, color, friction, collide, opacity)

    def flask_parts(base, dz, rot=np.eye(3), collide=True, medium=False):
        """Flask pieces (glass), foil cap (visual) and, for a culture flask, its medium (visual), at height dz."""
        off = np.array([0.0, 0.0, dz])
        for k, v in L.flask_hulls().items():
            hull(f"{base}/glass_{k}", v @ rot.T + off, GLASS, friction=L.F_FRICTION, collide=collide,
                 opacity=0.55, patch=L.NECK_PATCH_R if k in ("neck", "lip") else None)
        # aluminium foil cap over the mouth, folded down the neck (visual)
        cap = np.vstack([L.ring(L.F_LIP_R + 0.0012, L.F_HEIGHT - 0.030), L.ring(L.F_LIP_R + 0.0012, L.F_HEIGHT + 0.002)])
        hull(f"{base}/foil", cap + off, FOIL, collide=False)
        if medium:  # about 50 mL of culture medium (visual)
            v = np.vstack([L.ring(L.F_BASE_R - 0.004, 0.003), L.ring(L.flask_radius(0.022) - 0.003, 0.022)])
            hull(f"{base}/medium", v + off, MEDIUM, collide=False)

    def clamp_plate(base):
        disc(f"{base}/plate", L.PLATE_R, 0.0, L.SEAT_Z, STEEL, friction=L.PLATE_FRICTION, n=20)
        for b in range(3):  # bolt heads (visual)
            a = math.radians(36 + 120 * b)
            disc(f"{base}/bolt{b}", 0.0045, L.SEAT_Z, L.SEAT_Z + 0.003, STEEL_DARK, collide=False, n=10,
                 center=(0.020 * math.cos(a), 0.020 * math.sin(a)))

    def finger_pieces(base, theta, frame_rot=np.eye(3), frame_pos=np.zeros(3), collide=True):
        for k, v in L.finger_points(theta).items():
            hull(f"{base}_{k}", v @ frame_rot.T + frame_pos, STEEL, friction=L.FINGER_FRICTION, collide=collide)

    A = authored(SETUP_BASE)
    # ---------------------------------------------------------------- shaker housing (kinematic), frame on the bench
    xform(f"{prim_path}/shaker", *A["shaker"])
    rigid(f"{prim_path}/shaker", 20.0, kinematic=True)
    sh = f"{prim_path}/shaker"
    bx, by = L.BODY_HALF
    hz = L.BODY_TOP - L.TOP
    box(f"{sh}/housing", -bx, bx, -by, by, 0.004, hz, HOUSING)
    for i, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        box(f"{sh}/foot{i}", sx * bx - 0.02 * sx - 0.012, sx * bx - 0.02 * sx + 0.012, sy * by - 0.02 * sy - 0.012,
            sy * by - 0.02 * sy + 0.012, 0.0, 0.004, MAT)
    # front control panel (faces the robot, -x): display, speed knob, run LED (visual)
    box(f"{sh}/panel", -bx - 0.002, -bx, -0.120, 0.120, 0.018, hz - 0.015, PANEL, collide=False)
    box(f"{sh}/display", -bx - 0.003, -bx - 0.002, -0.090, -0.020, 0.040, 0.065, (0.30, 0.55, 0.75), collide=False)
    box(f"{sh}/knob", -bx - 0.016, -bx - 0.002, 0.030, 0.060, 0.035, 0.065, STEEL_DARK, collide=False)
    box(f"{sh}/led", -bx - 0.004, -bx - 0.002, 0.085, 0.095, 0.050, 0.060, LED, collide=False)
    disc(f"{sh}/post", 0.040, hz, L.PLATFORM_Z - L.PLATFORM_T - L.TOP, STEEL_DARK, collide=False)
    # ---------------------------------------------------------------- platform (kinematic), frame on its top surface
    xform(f"{prim_path}/platform", *A["platform"])
    rigid(f"{prim_path}/platform", 3.0, kinematic=True)
    pf = f"{prim_path}/platform"
    h = L.PLATFORM_HALF
    box(f"{pf}/plate", -h, h, -h, h, -L.PLATFORM_T, 0.0, PLATFORM_COL, friction=0.5)
    for i, s in enumerate(((-1, 0), (1, 0), (0, -1), (0, 1))):  # raised rim (visual)
        x0, x1 = (-h, h) if s[0] == 0 else ((h - 0.006, h) if s[0] > 0 else (-h, -h + 0.006))
        y0, y1 = (-h, h) if s[1] == 0 else ((h - 0.006, h) if s[1] > 0 else (-h, -h + 0.006))
        box(f"{pf}/rim{i}", x0, x1, y0, y1, 0.0, 0.004, PLATFORM_COL, collide=False)
    # ---------------------------------------------------------------- free clamp base (kinematic), frame C
    xform(f"{prim_path}/clamp", *A["clamp"])
    rigid(f"{prim_path}/clamp", 0.15, kinematic=True)
    cl = f"{prim_path}/clamp"
    clamp_plate(cl)
    for i in range(L.N_FINGERS):  # finger feet (visual tabs riveted to the plate)
        hp, hr = finger_rest(i)
        v = L.box_corners(-0.004, 0.003, -L.FINGER_W / 2 - 0.001, L.FINGER_W / 2 + 0.001, -0.004, 0.003)
        hull(f"{cl}/foot{i}", v @ hr.T + hp, STEEL_DARK, collide=False)
    # ---------------------------------------------------------------- loaded clamps with their culture flasks
    for k, name in enumerate(LOADED):
        xform(f"{prim_path}/{name}", *A[name])
        rigid(f"{prim_path}/{name}", 0.4, kinematic=True)
        lb = f"{prim_path}/{name}"
        clamp_plate(lb)
        for i in range(L.N_FINGERS):
            hp, hr = finger_rest(i)
            finger_pieces(f"{lb}/finger{i}", L.LOADED_THETA, hr, hp)
        flask_parts(lb, L.SEAT_Z, medium=True)
    # ---------------------------------------------------------------- free clamp fingers (dynamic, revolute feet)
    for i, f in enumerate(FINGERS):
        xform(f"{prim_path}/{f}", *A[f])
        rigid(f"{prim_path}/{f}", L.FINGER_MASS, com=(-0.004, 0.0, 0.034), inertia=(1.2e-5, 1.2e-5, 1.5e-6))
        fp = f"{prim_path}/{f}"
        finger_pieces(f"{fp}/strip", 0.0)
        hp, hr = finger_rest(i)
        q = _quat(hr)
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{fp}/hinge")
        j.CreateAxisAttr("Y")
        j.CreateBody0Rel().SetTargets([Sdf.Path(cl)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(fp)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, hp)))
        j.CreateLocalRot0Attr(Gf.Quatf(float(q[3]), *map(float, q[:3])))
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        j.CreateLowerLimitAttr(float(L.FINGER_LIMITS_DEG[0]))
        j.CreateUpperLimitAttr(float(L.FINGER_LIMITS_DEG[1]))
        fr = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(fp)).CreateFilteredPairsRel()
        fr.AddTarget(cl)
        fr.AddTarget(pf)
    # ---------------------------------------------------------------- the empty flask (dynamic), frame F
    xform(f"{prim_path}/flask", *A["flask"])
    rigid(f"{prim_path}/flask", L.F_MASS, com=(0.0, 0.0, L.F_COM_Z), inertia=(2.6e-4, 2.6e-4, 2.0e-4))
    flask_parts(f"{prim_path}/flask", 0.0)
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
    """Computed finger spring torques and the shake orbit every physics step; scoring every control step."""

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
        root = "/World/envs/env_0/bench"
        self.root = root
        P = lambda n: f"{root}/{n}"  # noqa: E731
        self.view = self.simview.create_rigid_body_view([P(n) for n in BODIES])
        self.dyn = self.simview.create_rigid_body_view([P(n) for n in DYNAMIC])
        self.kin = self.simview.create_rigid_body_view([P(n) for n in MOVING])
        self.clampv = self.simview.create_rigid_body_view([P("clamp")])
        robot_links = []
        for arm in ("left_robot", "right_robot"):
            lp = self.base.scene[arm].root_view.link_paths
            robot_links += list(lp[0] if lp and isinstance(lp[0], (list, tuple)) else lp)
        self.robot_links = robot_links
        self.cv_grip = self.simview.create_rigid_contact_view(
            [P("flask")], filter_patterns=[robot_links], max_contact_data_count=4096
        )
        self.cv_fingers = self.simview.create_rigid_contact_view(
            [P(f) for f in FINGERS], filter_patterns=[[P("flask")] for _ in FINGERS], max_contact_data_count=4096
        )
        self.cv_loaded = self.simview.create_rigid_contact_view(
            [P(n) for n in LOADED], filter_patterns=[robot_links + [P("flask")] for _ in LOADED],
            max_contact_data_count=4096,
        )
        self.origin = self.base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        self.torques = torch.zeros((len(DYNAMIC), 3), dtype=torch.float32)
        self.zero_f = torch.zeros((len(DYNAMIC), 3), dtype=torch.float32)
        self.force_idx = torch.arange(len(DYNAMIC), dtype=torch.int32)
        self.kin_idx = torch.arange(len(MOVING), dtype=torch.int32)
        self.extra = {}  # development probes only: {'flask': force vector (N, world)}
        self.setup = SETUP_BASE
        self.spec = L.setup_spec(self.setup)
        self.trial_active = False
        self.rest_kin = None
        self._init_state()

    def _init_state(self):
        self.scorer = S.Scorer()
        self.steps = 0
        self.t = 0.0
        self.hook_s = 0.0
        self.nonfinite = 0
        self.grip_peak = 0.0  # largest contact force on the flask from one robot link (a finger pad), N
        self.grip_link = None
        self.grip_sum = None  # per-link force summed over the current control interval's physics steps
        self.grip_n = 0
        self.grip_sustained = 0.0  # largest per-link force averaged over one control interval (50 ms), N
        self.push_sum = 0.0  # downward net robot force on the flask summed over the control interval
        self.push_sustained = 0.0  # largest |net vertical robot force| on the flask averaged over one control interval
        self.fz_min, self.fz_max = 0.0, 0.0  # control-interval means of the net vertical robot force (sign as reported)
        self.press_peak = 0.0  # largest total robot-on-flask contact force, N
        self.loaded_peak = 0.0  # largest robot/flask contact force on a loaded clamp or culture flask, N
        self.finger_n = np.zeros(L.N_FINGERS)
        self.finger_n_now = np.zeros(L.N_FINGERS)
        self.theta = np.zeros(L.N_FINGERS)
        self.theta_peak = 0.0
        self.shake_t0 = None
        self.shake_phase = 0.0
        self.shake_offset = np.zeros(3)
        self.was_shaking = False
        STATE.update(score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, setup=self.setup)

    def begin_trial(self, trial):
        self._init_state()
        self.trial_active = True
        STATE.update(trial=trial, setup=self.setup, start=self.spec)

    # ------------------------------------------------------------------ reading
    def read(self):
        tf = self.view.get_transforms().detach().cpu().numpy().astype(float)
        return {n: (tf[i, :3] - self.origin, _rot(tf[i, 3:7])) for i, n in enumerate(BODIES)}

    def thetas(self, poses):
        _, rc = poses["clamp"]
        return np.array([L.finger_theta(poses[f][1], rc, finger_rest(i)[1]) for i, f in enumerate(FINGERS)])

    # ------------------------------------------------------------------ physics hook
    def physics(self, dt):
        t0 = time.perf_counter()
        self.t += dt
        tf = self.dyn.get_transforms().detach().cpu().numpy().astype(float)
        v = self.dyn.get_velocities().detach().cpu().numpy().astype(float)
        ct = self.clampv.get_transforms().detach().cpu().numpy().astype(float)[0]
        cv = self.clampv.get_velocities().detach().cpu().numpy().astype(float)[0]
        rc = _rot(ct[3:7])
        tq = np.zeros((len(DYNAMIC), 3))
        for i in range(L.N_FINGERS):
            rf = _rot(tf[1 + i, 3:7])
            rest = finger_rest(i)[1]
            th = L.finger_theta(rf, rc, rest)
            axis = rc @ rest[:, 1]
            w = float((v[1 + i, 3:6] - cv[3:6]) @ axis)
            tq[1 + i] = (-L.FINGER_K * th - L.FINGER_C * w) * axis
            self.theta[i] = th
        self.theta_peak = max(self.theta_peak, float(self.theta.max()))
        f = np.zeros((len(DYNAMIC), 3))
        for name, fv in self.extra.items():  # development probes only
            k = DYNAMIC.index(name)
            if callable(fv):  # (force, point) from the current flask state
                fv, at = fv(tf[k, :3] - self.origin, _rot(tf[k, 3:7]), v[k])
                tq[k] += np.cross(np.asarray(at, float) - (tf[k, :3] - self.origin), fv)
            f[k] += np.asarray(fv, float)
        self.torques.copy_(self.torch.as_tensor(tq, dtype=self.torch.float32))
        self.dyn.apply_forces_and_torques_at_position(
            self.torch.as_tensor(f, dtype=self.torch.float32), self.torques, None, self.force_idx, True
        )
        if self.trial_active:
            self._shake(dt)
            gm = self.cv_grip.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
            if gm.size:
                per = np.linalg.norm(gm[0], axis=-1)
                j = int(np.argmax(per))
                if per[j] > self.grip_peak:
                    self.grip_peak = float(per[j])
                    self.grip_link = self.robot_links[j].split("/")[-1]
                self.press_peak = max(self.press_peak, float(np.linalg.norm(gm[0].sum(axis=0))))
                self.grip_sum = per if self.grip_sum is None else self.grip_sum + per
                self.grip_n += 1
                self.push_sum += self._robot_fz(dt)
            fm = self.cv_fingers.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
            if fm.size:
                fn = np.linalg.norm(fm[:, 0, :], axis=-1)
                self.finger_n = np.maximum(self.finger_n, fn)
                self.finger_n_now = fn
            lm = self.cv_loaded.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
            if lm.size:
                self.loaded_peak = max(self.loaded_peak, float(np.linalg.norm(lm, axis=-1).max()))
        self.hook_s += time.perf_counter() - t0

    def _robot_fz(self, dt):
        """Vertical component (N) of the net robot contact force on the flask this physics step: normal forces plus
        friction (the force matrix holds normal forces only, and a grip pushes through friction)."""
        f, _, n, _, cnt, start = (x.detach().cpu().numpy() for x in self.cv_grip.get_contact_data(dt=dt))
        fr, _, fcnt, fstart = (x.detach().cpu().numpy() for x in self.cv_grip.get_friction_data(dt=dt))
        tot = 0.0
        for j in range(cnt.shape[1]):
            a, k = int(start[0, j]), int(cnt[0, j])
            if k:
                tot += float((f[a:a + k].reshape(-1) * n[a:a + k, 2]).sum())
            a, k = int(fstart[0, j]), int(fcnt[0, j])
            if k:
                tot += float(fr[a:a + k, 2].sum())
        return tot

    def _shake(self, dt):
        torch = self.torch
        if self.rest_kin is None:
            return
        if self.scorer.shaking:
            if self.shake_t0 is None:
                self.shake_t0 = self.t
                self.shake_phase = 0.0
            ts = self.t - self.shake_t0
            self.shake_phase += 2 * math.pi * L.SHAKE_RPM / 60.0 * dt
            a = S.shake_amplitude(ts) * L.ORBIT_R
            self.shake_offset = np.array([a * math.cos(self.shake_phase), a * math.sin(self.shake_phase), 0.0])
            tgt = self.rest_kin.clone()
            tgt[:, :3] += torch.tensor(self.shake_offset, dtype=tgt.dtype)
            self.kin.set_kinematic_targets(tgt, self.kin_idx)
            self.was_shaking = True
        elif self.was_shaking:
            self.kin.set_kinematic_targets(self.rest_kin.clone(), self.kin_idx)
            self.shake_offset = np.zeros(3)
            self.shake_t0 = None
            self.was_shaking = False

    # ------------------------------------------------------------------ reset
    def restore_authored(self, setup, override=None):
        """Environment reset only (never during a trial): every body to the setup's authored pose (``override``:
        development probes only, {body: (pos, rot)})."""
        torch = self.torch
        self.setup = setup
        self.spec = L.setup_spec(setup)
        self.trial_active = False
        a = authored(setup)
        if callable(override):  # development probes only
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
        self.rest_kin = torch.stack([tf[BODIES.index(n)] for n in MOVING]).clone()
        try:
            self.kin.set_kinematic_targets(self.rest_kin.clone(), self.kin_idx)
        except Exception as e:  # noqa: BLE001  (first reset before the views are live)
            print("FLASK kinematic targets at reset:", e, flush=True)
        self._init_state()

    # ------------------------------------------------------------------ scoring
    def measure(self, environment):
        poses = self.read()
        pf, rf = poses["flask"]
        pc, rc = poses["clamp"]
        th = self.thetas(poses)
        rel = L.flask_in_clamp(pf, rf, pc, rc)
        seated, sd = S.seated_detail(rel, th, self.finger_n)
        tips = finger_tips(environment)
        tip = min(L.point_to_flask(t, pf, rf) for t in tips) if tips else 1.0
        v = self.dyn.get_velocities().detach().cpu().numpy().astype(float)[0, :3]
        shake_v = 0.0
        if self.scorer.shaking:
            shake_v = 2 * math.pi * L.SHAKE_RPM / 60.0 * float(np.linalg.norm(self.shake_offset))
        still = float(np.linalg.norm(v)) < S.STILL_V + shake_v
        detail = dict(rel={k: round(x * 1e3, 3) if k != "tilt_deg" else round(x, 3) for k, x in rel.items()},
                      theta_deg=np.round(np.degrees(th), 2).tolist(),
                      finger_n=np.round(self.finger_n, 3).tolist(), seat=sd, tip_mm=round(tip * 1e3, 2),
                      flask_mm=np.round(pf * 1e3, 2).tolist(), flask_v_mm_s=round(float(np.linalg.norm(v)) * 1e3, 2))
        return S.Measure(seated=seated, released=tip >= S.CLEAR_M, still=still, detail=detail), poses

    def control(self, environment):
        if not self.trial_active:
            return
        self.steps += 1
        m, poses = self.measure(environment)
        pf, rf = poses["flask"]
        if not (np.all(np.isfinite(pf)) and np.all(np.isfinite(rf))):
            self.nonfinite += 1
            STATE["nonfinite"] = self.nonfinite
            return
        level = self.scorer.step(m)
        self.finger_n = np.zeros(L.N_FINGERS)  # max over the next control interval's physics steps
        if self.grip_n:
            self.grip_sustained = max(self.grip_sustained, float((self.grip_sum / self.grip_n).max()))
            fz = self.push_sum / self.grip_n
            self.fz_min, self.fz_max = min(self.fz_min, fz), max(self.fz_max, fz)
            self.push_sustained = max(abs(self.fz_min), abs(self.fz_max))
        self.grip_sum, self.grip_n, self.push_sum = None, 0, 0.0
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
                push_sustained_n=round(self.push_sustained, 2),
                robot_fz_range_n=[round(self.fz_min, 2), round(self.fz_max, 2)],
                press_peak_n=round(self.press_peak, 2),
                loaded_peak_n=round(self.loaded_peak, 2),
                theta_peak_deg=round(math.degrees(self.theta_peak), 2),
                shake_offset_mm=np.round(self.shake_offset * 1e3, 2).tolist(),
                setup=self.setup,
            ),
        )


_RT: dict = {"rt": None, "sim": None, "trials": 0, "override": None}


def runtime():
    return _RT["rt"]


def bench_state():
    """Current poses of every body for the scripted demo (read-only), plus the start spec and finger angles."""
    rt = _RT["rt"]
    if rt is None:
        return None
    out = rt.read()
    out["spec"] = rt.spec
    out["theta"] = rt.thetas(out)
    return out


def _patch_scene(scene) -> None:
    if getattr(scene, "_flask_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Bench):
            return _arena_object(spec)
        return orig_task_object(spec)

    def object_states(environment, names):
        from yam_arena.task_file import ObjectState

        plain = [n for n in names if n != "bench"]
        out = orig_states(environment, plain) if plain else {}
        if "bench" in names:
            rt = _RT["rt"]
            p, r = rt.read()["flask"] if rt else authored(SETUP_BASE)["flask"]
            z = np.zeros(3)
            out["bench"] = ObjectState(
                pos=np.asarray(p),
                quat_xyzw=_quat(r),
                lin_vel=z,
                ang_vel=z,
                low=p - 0.1,
                high=p + 0.1,
            )
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
    scene._flask_patched = True


def _has_bench(emb):
    return any(isinstance(o, Bench) for o in (emb.task.objects if emb.task is not None else ()))


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_flask_patched", False):
        return
    base_task_step, base_reset, base_ensure = cls._task_step, cls.reset, cls._ensure_sim

    def _ensure_sim(self, *a, **k):
        env_before = self._env
        out = base_ensure(self, *a, **k)
        if self._env is not None and self._env is not env_before and _has_bench(self):
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
            setup = (SETUP_BASE + _RT["trials"]) % len(L.SETUPS)
        if rt is not None:
            rt.restore_authored(setup, _RT["override"])
        obs = base_reset(self, *a, **k)
        rt = _RT["rt"]
        if rt is not None:
            if rt.setup != setup:  # first reset of a fresh env
                rt.restore_authored(setup, _RT["override"])
                obs = base_reset(self, *a, **k)
            # the reset's settling steps may leave a finger swinging or the flask rocking: put everything back once
            # more after the settle
            rt.restore_authored(setup, _RT["override"])
            rt.begin_trial(_RT["trials"])
            _RT["trials"] += 1
        return obs

    cls._ensure_sim, cls._task_step, cls.reset = _ensure_sim, _task_step, reset
    cls._flask_patched = True


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
