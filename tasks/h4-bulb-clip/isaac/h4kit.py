"""h4kit: the rear of an H4 headlamp in an engine-bay mock-up, the new H4 bulb in its holder, the wire spring clip and
its catch, for yam_arena (Isaac Lab-Arena / PhysX), plus the runtime hooks.

Authored as one custom-spawned assembly (yam_arena task-file primitives are single boxes/spheres):
* kinematic bodies (fixed during a trial, so PhysX reports the contact forces on them): the headlamp ("housing": the
  bulb boss with its bore, the locating rim with three notches, the lamp's back shell, plus the engine-bay mock-up:
  radiator support, battery box, inner fender apron) and the bulb holder (moved per trial at reset);
* the bulb: a free rigid body (the cap: reference flange with its three tabs, collar, cap body, terminal blades) and
  its glass envelope as a second rigid body held to the cap by a fixed joint, so robot contact with the glass is read
  by collider identity;
* the wire spring clip (bail): a rigid body on a revolute joint (hard limits) at its hinge on the boss; hinge drag and
  friction are torques computed every physics step (drives on non-articulation joints are inert in this build);
* the catch: a body on a prismatic joint (hard limits), pushed toward its rest stop by a spring force computed every
  physics step (preload + rate); the bail's bight cams it aside along its 45 deg ramp and it snaps back over the wire.
Geometry: ``../src/layout.py``; scorer: ``../src/scoring.py`` (pure functions).

Nothing here writes a pose or velocity during a trial; ``Runtime.restore_authored`` places the bodies only on an
environment reset. The scripted demo READS poses through ``bench_state()``.
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
from scoring import measure, Monitor, Scorer  # noqa: E402
from yam_arena.task_file import Box  # noqa: E402

PHYSICS_DT = float(os.environ.get("H4_PHYSICS_DT", str(1 / 480)))
CONTACT_OFFSET = float(os.environ.get("H4_CONTACT_OFFSET", ".0001"))  # below the 0.2-0.5 mm clearances
SETUP_BASE = int(os.environ.get("H4_SETUP", "0"))
FORCE_NAMED = "H4_SETUP" in os.environ
DEBUG = bool(os.environ.get("H4_DEBUG"))

STEEL, BRASS, GLASS, PLASTIC, BLACK, GREY, BODY, BATT, FOAM = (
    (0.70, 0.71, 0.73),
    (0.80, 0.68, 0.38),
    (0.86, 0.92, 0.96),
    (0.20, 0.20, 0.22),
    (0.07, 0.07, 0.08),
    (0.45, 0.46, 0.48),
    (0.30, 0.32, 0.36),
    (0.12, 0.12, 0.13),
    (0.85, 0.35, 0.10),
)
KINEMATIC = ("housing", "holder")
DYNAMIC = ("bulb", "glass", "clip", "catch")
BODIES = KINEMATIC + DYNAMIC
METAL_FRICTION = 0.4
GLASS_FRICTION = 0.3
WIRE_FRICTION = 0.3


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


def authored(setup):
    """Authored (reset) poses of the bodies for a setup: {name: (pos, rot)} in the env frame."""
    sp = L.setup_spec(setup)
    bp, br = L.bulb_holder_pose(setup)
    return {
        "housing": (L.H.copy(), np.eye(3)),
        "holder": (L.holder_pos(setup), np.eye(3)),
        "bulb": (bp, br),
        "glass": (bp, br),
        "clip": (L.H + L.hinge_point(), L.clip_rot(sp["clip"])),
        "catch": (L.H.copy(), np.eye(3)),
    }


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
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(*(translation or (0.0, 0.0, 0.0)))
    )
    ox, oy, oz, ow = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(ow, ox, oy, oz))
    looks, mats = {}, {}

    def look(color):
        if color not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.45)
            vis.func(path, vis)
            looks[color] = path
        return looks[color]

    def material(friction):
        if friction not in mats:
            path = f"{prim_path}/Looks/phys{len(mats)}"
            pm = sim_utils.RigidBodyMaterialCfg(static_friction=friction, dynamic_friction=friction)
            pm.func(path, pm)
            mats[friction] = path
        return mats[friction]

    def xform(path, pos, rot):
        x = UsdGeom.Xform.Define(stage, path)
        x.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float, pos)))
        q = _quat(rot)
        x.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Quatd(float(q[3]), *map(float, q[:3]))
        )
        return x

    def rigid(path, mass=None, kinematic=False, inertia=None, com=None, iters=32, max_w_dps=None, ccd=False):
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
        px.CreateMaxDepenetrationVelocityAttr(0.2)
        px.CreateSolverPositionIterationCountAttr(iters)
        px.CreateSolverVelocityIterationCountAttr(4)
        px.CreateLinearDampingAttr(0.0)
        px.CreateAngularDampingAttr(0.05)
        px.CreateSleepThresholdAttr(0.0)  # never sleeps: a sleeping body reports no contacts
        px.CreateStabilizationThresholdAttr(0.0)
        if max_w_dps is not None:
            px.CreateMaxAngularVelocityAttr(float(max_w_dps))
        if ccd:
            px.CreateEnableCCDAttr(True)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)

    count = {"n": 0}

    def hull(path, v, color, friction=METAL_FRICTION, approximation="convexHull"):
        v = np.asarray(v, float)
        h = ConvexHull(v)
        m = UsdGeom.Mesh.Define(stage, path)
        m.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in v])
        faces = []
        c = v.mean(axis=0)
        for s in h.simplices:
            a, b, d = v[s[0]], v[s[1]], v[s[2]]
            n = np.cross(b - a, d - a)
            faces.append(s if n @ (a - c) > 0 else s[::-1])
        m.CreateFaceVertexCountsAttr([3] * len(faces))
        m.CreateFaceVertexIndicesAttr([int(j) for f in faces for j in f])
        m.CreateSubdivisionSchemeAttr("none")
        m.CreateExtentAttr(
            [Gf.Vec3f(*map(float, v.min(axis=0))), Gf.Vec3f(*map(float, v.max(axis=0)))]
        )
        prim = m.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(CONTACT_OFFSET)
        cp.CreateRestOffsetAttr(0.0)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(approximation)
        if approximation == "convexHull":
            hc = PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim)
            hc.CreateHullVertexLimitAttr(255)
            hc.CreateMinThicknessAttr(0.00005)
        bind_physics_material(str(prim.GetPath()), material(friction), stage=stage)
        bind_visual_material(str(prim.GetPath()), look(color), stage=stage)
        count["n"] += 1
        return m

    def box(path, center, size, color, friction=METAL_FRICTION):
        c = np.asarray(center, float)
        s = np.asarray(size, float) / 2
        return hull(path, L.box_pts(c - s, c + s), color, friction)

    A = authored(SETUP_BASE)
    P = {n: f"{prim_path}/{n}" for n in BODIES}
    # ---------------------------------------------------------------- headlamp + engine bay (kinematic, frame H)
    xform(P["housing"], *A["housing"])
    rigid(P["housing"], 5.0, kinematic=True)
    for i, v in enumerate(L.plate_pieces()):
        hull(f"{P['housing']}/boss_{i:02d}", v, GREY)
    for i, v in enumerate(L.rim_pieces()):
        hull(f"{P['housing']}/rim_{i:02d}", v, GREY)
    # lamp back shell: a flat annulus around the boss, then the shell wall
    for j in range(24):
        hull(
            f"{P['housing']}/shell_{j:02d}",
            L.annular_sector(L.PLATE_R, L.LAMP_R, j * 15.0, (j + 1) * 15.0, 0.0, 0.003, n=3),
            BODY,
        )
        hull(
            f"{P['housing']}/wall_{j:02d}",
            L.annular_sector(L.LAMP_R - 0.003, L.LAMP_R, j * 15.0, (j + 1) * 15.0, 0.0, L.SHELL_X[1], n=3),
            BODY,
        )
    # hinge lugs (cosmetic size, real contact): small blocks above / below the hinge line, outside the bail's legs
    for s in (1.0, -1.0):
        box(
            f"{P['housing']}/lug_{'u' if s > 0 else 'd'}",
            (L.HINGE_X + 0.0005, L.HINGE_Y, s * (L.CLIP_HALF_W + 0.0035)),
            (0.0035, 0.004, 0.004),
            GREY,
        )
    # catch guide (behind the catch's travel, out of the bail's path)
    box(
        f"{P['housing']}/catch_guide",
        (L.LIP_X[1] - 0.003, L.CATCH_OUTER_Y - 0.0065, 0.0),
        (0.008, 0.003, 0.008),
        GREY,
    )
    for name, (c, s), col in (
        ("panel", L.PANEL, BODY),
        ("battery", L.BATTERY, BATT),
        ("apron", L.APRON, BODY),
    ):
        box(f"{P['housing']}/{name}", np.asarray(c) - L.H, s, col)
    # ---------------------------------------------------------------- bulb holder (kinematic, moved per setup)
    xform(P["holder"], *A["holder"])
    rigid(P["holder"], 1.0, kinematic=True)
    zt = L.HOLDER_TOP
    hull(f"{P['holder']}/post", L.cylinder_z(0.020, 0.001, zt - L.CUP_DEPTH, n=24), FOAM)
    hull(
        f"{P['holder']}/cup_floor",
        L.cylinder_z(L.CUP_R[1], zt - L.CUP_DEPTH - 0.003, zt - L.CUP_DEPTH, n=24),
        FOAM,
    )
    for j in range(16):
        a0, a1 = math.radians(j * 22.5), math.radians((j + 1) * 22.5)
        pts = []
        for r in L.CUP_R:
            for a in np.linspace(a0, a1, 3):
                for z in (zt - L.CUP_DEPTH, zt):
                    pts.append((r * math.cos(a), r * math.sin(a), z))
        hull(f"{P['holder']}/cup_{j:02d}", np.array(pts), FOAM, friction=0.6)
    # ---------------------------------------------------------------- bulb (cap + glass, fixed joint)
    pieces = L.bulb_pieces()
    xform(P["bulb"], *A["bulb"])
    m_glass = L.GLASS_MASS
    rigid(
        P["bulb"],
        L.BULB_MASS - m_glass,
        inertia=L.bulb_inertia() * 0.7,
        com=(L.CAP_COM_X, 0.0, 0.0),
        iters=48,
    )
    for i, v in enumerate(pieces["metal"]):
        hull(f"{P['bulb']}/metal_{i}", v, STEEL if i else BRASS)
    for i, v in enumerate(pieces["base"]):
        hull(f"{P['bulb']}/base_{i}", v, PLASTIC if i < 2 else BRASS)
    xform(P["glass"], *A["glass"])
    rigid(
        P["glass"],
        m_glass,
        inertia=(1.2e-7, 8e-7, 8e-7),
        com=(L.GLASS_COM_X, 0.0, 0.0),
        iters=48,
    )
    for i, v in enumerate(pieces["glass"]):
        hull(f"{P['glass']}/env_{i}", v, GLASS, friction=GLASS_FRICTION)
    fj = UsdPhysics.FixedJoint.Define(stage, f"{P['glass']}/to_cap")
    fj.CreateBody0Rel().SetTargets([Sdf.Path(P["bulb"])])
    fj.CreateBody1Rel().SetTargets([Sdf.Path(P["glass"])])
    for setp, setr in (
        (fj.CreateLocalPos0Attr, fj.CreateLocalRot0Attr),
        (fj.CreateLocalPos1Attr, fj.CreateLocalRot1Attr),
    ):
        setp(Gf.Vec3f(0.0, 0.0, 0.0))
        setr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(P["glass"])).CreateFilteredPairsRel().AddTarget(
        P["bulb"]
    )
    # ---------------------------------------------------------------- wire clip (bail) on its hinge
    xform(P["clip"], *A["clip"])
    # the 1.6 mm wire must not tunnel through the 1 mm flange / rim or the catch lip: angular speed capped at
    # CLIP_MAX_W_DPS (a hand-swung clip moves far slower) and swept (CCD) contact on
    rigid(
        P["clip"],
        L.CLIP_MASS,
        inertia=L.clip_inertia(),
        com=(0.0, -L.CLIP_LEN / 2, 0.0),
        iters=48,
        max_w_dps=L.CLIP_MAX_W_DPS,
        ccd=True,
    )
    for i, v in enumerate(L.clip_pieces()):
        hull(f"{P['clip']}/wire_{i}", v, STEEL, friction=WIRE_FRICTION)
    j = UsdPhysics.RevoluteJoint.Define(stage, f"{P['clip']}/hinge")
    j.CreateAxisAttr("Z")
    j.CreateBody0Rel().SetTargets([Sdf.Path(P["housing"])])
    j.CreateBody1Rel().SetTargets([Sdf.Path(P["clip"])])
    j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, L.hinge_point())))
    j.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    j.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    j.CreateLowerLimitAttr(-L.CLIP_LIMITS_DEG[1])  # joint angle = -phi
    j.CreateUpperLimitAttr(-L.CLIP_LIMITS_DEG[0])
    # ---------------------------------------------------------------- catch (sprung slide along y_H)
    xform(P["catch"], *A["catch"])
    rigid(P["catch"], L.CATCH_MASS, inertia=(4e-7, 4e-7, 4e-7), iters=48, ccd=True)
    for i, v in enumerate(L.catch_pieces()):
        hull(f"{P['catch']}/lip_{i}", v, PLASTIC, friction=0.3)
    pj = UsdPhysics.PrismaticJoint.Define(stage, f"{P['catch']}/slide")
    pj.CreateAxisAttr("Y")
    pj.CreateBody0Rel().SetTargets([Sdf.Path(P["housing"])])
    pj.CreateBody1Rel().SetTargets([Sdf.Path(P["catch"])])
    for setp, setr in (
        (pj.CreateLocalPos0Attr, pj.CreateLocalRot0Attr),
        (pj.CreateLocalPos1Attr, pj.CreateLocalRot1Attr),
    ):
        setp(Gf.Vec3f(0.0, 0.0, 0.0))
        setr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    pj.CreateLowerLimitAttr(-L.CATCH_TRAVEL)
    pj.CreateUpperLimitAttr(0.0)
    UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(P["catch"])).CreateFilteredPairsRel().AddTarget(
        P["housing"]
    )
    _SPEC["paths"] = P
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
        pose = (
            robot.data.body_link_pose_w[0]
            .detach()
            .cpu()
            .numpy()
            .astype(float)[list(robot.body_names).index("gripper")]
        )
        jp = robot.data.joint_pos[0].detach().cpu().numpy().astype(float)
        jn = list(robot.joint_names)
        half = max(abs(jp[jn.index(j)]) for j in ("joint7", "joint8"))
        r = _rot(pose[3:7])
        tip = pose[:3] + r @ (np.array(TCP_OFFSET) + [0, 0, PAD_TIP_PAST_TCP]) - origin
        pts += [tip + r[:, 0] * s * (half + 0.0032) for s in (1.0, -1.0)]
    return pts


class Runtime:
    """Applies the clip-hinge and catch-spring forces and reads the contact forces every physics step; scores every
    control step."""

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
        root = "/World/envs/env_0/bench"
        P = lambda n: f"{root}/{n}"  # noqa: E731
        self.view = self.simview.create_rigid_body_view([P(n) for n in DYNAMIC])
        self.kview = self.simview.create_rigid_body_view([P(n) for n in KINEMATIC])
        self.index = {n: i for i, n in enumerate(DYNAMIC)}
        robot_links = []
        for arm in ("left_robot", "right_robot"):
            lp = self.base.scene[arm].root_view.link_paths
            robot_links += list(lp[0] if lp and isinstance(lp[0], (list, tuple)) else lp)
        self.robot_links = robot_links
        self.cv_glass = self.simview.create_rigid_contact_view(
            [P("glass")], filter_patterns=[robot_links], max_contact_data_count=2048
        )
        self.cv_bulb = self.simview.create_rigid_contact_view(
            [P("bulb")],
            filter_patterns=[robot_links + [P("housing"), P("holder"), P("clip")]],
            max_contact_data_count=4096,
        )
        self.n_links = len(robot_links)
        self.origin = self.base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        dev = self.view.get_transforms().device
        n = len(DYNAMIC)
        self.f = torch.zeros((n, 3), dtype=torch.float32, device=dev)
        self.tq = torch.zeros((n, 3), dtype=torch.float32, device=dev)
        self.pp = torch.zeros((n, 3), dtype=torch.float32, device=dev)
        self.idx = torch.arange(n, dtype=torch.int32, device=dev)
        self.setup = SETUP_BASE
        self.trial_active = False
        self.extra = {}  # development probes only: name -> (force world, at point world)
        self._init_state()

    def _init_state(self):
        self.monitor = Monitor()
        self.scorer = Scorer()
        self.steps = 0
        self.t = 0.0
        self.hook_s = 0.0
        self.nonfinite = 0
        self.peak = dict(catch_n=0.0, catch_mm=0.0, clip_min_deg=999.0, grip_on_bulb_n=0.0)
        STATE.update(score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, setup=self.setup)

    def begin_trial(self, trial):
        self._init_state()
        self.trial_active = True
        STATE.update(trial=trial, setup=self.setup, start=L.setup_spec(self.setup))

    # ------------------------------------------------------------------ reading
    def read(self):
        tf = self.view.get_transforms().detach().cpu().numpy().astype(float)
        out = {n: (tf[i, :3] - self.origin, _rot(tf[i, 3:7])) for n, i in self.index.items()}
        ktf = self.kview.get_transforms().detach().cpu().numpy().astype(float)
        for i, n in enumerate(KINEMATIC):
            out[n] = (ktf[i, :3] - self.origin, _rot(ktf[i, 3:7]))
        return out

    def velocities(self):
        v = self.view.get_velocities().detach().cpu().numpy().astype(float)
        return {n: (v[i, :3], v[i, 3:6]) for n, i in self.index.items()}

    def clip_state(self, poses=None):
        poses = poses or self.read()
        phi = L.clip_angle(poses["clip"][1])
        catch = float((poses["catch"][0] - L.H)[1])
        return phi, catch

    # ------------------------------------------------------------------ per physics step
    def physics(self, dt):
        t0 = time.perf_counter()
        self.t += dt
        poses = self.read()
        vel = self.velocities()
        phi, cy = self.clip_state(poses)
        om = float(vel["clip"][1][2])  # rad/s about +z (= -dphi/dt)
        f = np.zeros((len(DYNAMIC), 3))
        tq = np.zeros((len(DYNAMIC), 3))
        pp = np.zeros((len(DYNAMIC), 3))
        for n, i in self.index.items():
            pp[i] = poses[n][0] + self.origin
        # hinge drag + smoothed Coulomb friction about the hinge axis (torque on the clip; the housing is kinematic)
        tq[self.index["clip"], 2] = -L.CLIP_DAMPING * om - L.CLIP_FRICTION_NM * math.tanh(om / 0.05)
        # catch spring toward its rest stop (+y), preload + rate x deflection
        fc = L.CATCH_PRELOAD_N + L.CATCH_RATE_N_M * max(-cy, 0.0)
        f[self.index["catch"], 1] = fc
        for n, (fw, at) in self.extra.items():  # development probes only
            i = self.index[n]
            f[i] += fw
            pp[i] = np.asarray(at) + self.origin
        self.f.copy_(self.torch.as_tensor(f, dtype=self.torch.float32))
        self.tq.copy_(self.torch.as_tensor(tq, dtype=self.torch.float32))
        self.pp.copy_(self.torch.as_tensor(pp, dtype=self.torch.float32))
        self.view.apply_forces_and_torques_at_position(
            force_data=self.f, torque_data=self.tq, position_data=self.pp, indices=self.idx, is_global=True
        )
        self.peak["catch_mm"] = min(self.peak["catch_mm"], cy * 1e3)
        self.peak["catch_n"] = max(self.peak["catch_n"], fc)
        self.peak["clip_min_deg"] = min(self.peak["clip_min_deg"], phi)
        if self.trial_active:
            fg = self.cv_glass.get_contact_force_matrix(dt=dt).detach().cpu().numpy()[0]
            fb = self.cv_bulb.get_contact_force_matrix(dt=dt).detach().cpu().numpy()[0]
            glass_n = float(np.linalg.norm(fg, axis=-1).max()) if fg.size else 0.0
            robot_bulb = float(np.linalg.norm(fb[: self.n_links], axis=-1).max()) if fb.size else 0.0
            self.peak["grip_on_bulb_n"] = max(self.peak["grip_on_bulb_n"], robot_bulb)
            vz = float(vel["bulb"][0][2])
            self.monitor.step(dt, glass_n, vz, max(robot_bulb, glass_n))
            k = self.n_links
            self.last_contacts = dict(
                glass_n=round(glass_n, 4),
                robot_on_bulb_n=round(robot_bulb, 3),
                housing_on_bulb_n=round(float(np.linalg.norm(fb[k])), 3),
                holder_on_bulb_n=round(float(np.linalg.norm(fb[k + 1])), 3),
                clip_on_bulb_n=round(float(np.linalg.norm(fb[k + 2])), 3),
                catch_n=round(fc, 3),
            )
            if DEBUG and int(round(self.t / dt)) % 48 == 0:
                print("CONTACTS", self.last_contacts, round(phi, 2), round(cy * 1e3, 3), flush=True)
        self.hook_s += time.perf_counter() - t0

    # ------------------------------------------------------------------ reset
    def restore_authored(self, setup):
        """Environment reset only (never during a trial): every body back to its authored pose for the setup, at
        rest."""
        torch = self.torch
        self.setup = setup
        self.trial_active = False
        a = authored(setup)
        ktf = self.kview.get_transforms().clone()
        for i, n in enumerate(KINEMATIC):
            pos, rot = a[n]
            ktf[i, :3] = torch.tensor(np.asarray(pos, float) + self.origin, dtype=ktf.dtype)
            ktf[i, 3:7] = torch.tensor(_quat(rot), dtype=ktf.dtype)
        kidx = torch.arange(len(KINEMATIC), dtype=torch.int32, device=ktf.device)
        self.kview.set_transforms(ktf, kidx)
        try:
            self.kview.set_kinematic_targets(ktf, kidx)
        except Exception as e:  # noqa: BLE001
            print("h4kit: set_kinematic_targets failed (transforms set):", e, flush=True)
        tf = self.view.get_transforms().clone()
        for n, i in self.index.items():
            pos, rot = a[n]
            tf[i, :3] = torch.tensor(np.asarray(pos, float) + self.origin, dtype=tf.dtype)
            tf[i, 3:7] = torch.tensor(_quat(rot), dtype=tf.dtype)
        self.view.set_transforms(tf, self.idx)
        self.view.set_velocities(torch.zeros_like(self.view.get_velocities()), self.idx)
        self._init_state()

    # ------------------------------------------------------------------ scoring
    def control(self, environment):
        if not self.trial_active:
            return
        self.steps += 1
        poses = self.read()
        vel = self.velocities()
        pb, rb = poses["bulb"]
        lin, ang = vel["bulb"]
        if not (np.all(np.isfinite(pb)) and np.all(np.isfinite(rb)) and np.all(np.isfinite(lin))):
            self.nonfinite += 1
            STATE["nonfinite"] = self.nonfinite
            return
        phi, cy = self.clip_state(poses)
        tips = finger_tips(environment)
        m = measure(pb, rb, phi, cy, tips)
        level = self.scorer.step(m, float(np.linalg.norm(lin)), math.degrees(float(np.linalg.norm(ang))), self.monitor)
        STATE.update(
            score=level,
            level=level,
            steps=self.steps,
            nonfinite=self.nonfinite,
            detail=dict(
                self.scorer.last,
                bulb_pos_mm=np.round(pb * 1000, 3).tolist(),
                setup=self.setup,
                hook_wall_s=round(self.hook_s, 2),
                contacts=getattr(self, "last_contacts", None),
                kit_peak={k: round(v, 4) for k, v in self.peak.items()},
            ),
        )


_RT: dict = {"rt": None, "sim": None, "trials": 0}


def runtime():
    return _RT["rt"]


def bench_state():
    """Current poses {'bulb', 'glass', 'clip', 'catch', 'housing', 'holder'} for the scripted demo (read-only)."""
    return _RT["rt"].read() if _RT["rt"] else None


def _patch_scene(scene) -> None:
    if getattr(scene, "_h4_patched", False):
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
            p, r = rt.read()["bulb"] if rt else authored(SETUP_BASE)["bulb"]
            z = np.zeros(3)
            out["bench"] = ObjectState(
                pos=np.asarray(p), quat_xyzw=_quat(r), lin_vel=z, ang_vel=z, low=p - 0.1, high=p + 0.1
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
                physx_cfg = getattr(cfg.sim, "physics", None) or getattr(cfg.sim, "physx", None)
                if physx_cfg is not None and hasattr(physx_cfg, "enable_ccd"):
                    physx_cfg.enable_ccd = True  # swept contact for the thin clip wire (flagged on clip and catch)
                else:
                    print("h4kit: could not enable CCD (no enable_ccd field)", flush=True)
                return cfg

            environment.env_cfg_callback = configure
        return environment

    scene.build_task_environment = build
    scene.task_object = task_object
    scene.object_states = object_states
    scene._h4_patched = True


def _has_bench(emb):
    return any(isinstance(o, Bench) for o in (emb.task.objects if emb.task is not None else ()))


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_h4_patched", False):
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
            setup = (SETUP_BASE + _RT["trials"]) % len(L.SETUP_ROLL)
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
    cls._h4_patched = True


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
