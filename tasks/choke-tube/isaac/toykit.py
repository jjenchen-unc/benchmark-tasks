"""toykit: the toddler at the table, the no-choke testing tube, the nine toys, the placemat and the toy storage box for
yam_arena (Isaac Lab-Arena / PhysX), plus the runtime hooks.

Authored as one custom-spawned assembly (yam_arena task-file primitives are single boxes/spheres):
* kinematic rigid bodies (they do not move; PhysX reports the contact forces on them): the toddler manikin, her high
  chair, the placemat and the storage box;
* free rigid bodies: the testing tube (bore wall of 36 convex prisms + the slanted floor as one convex solid) and the
  nine toys (cubes, bricks, the peg, the stick and the peg doll as convex hulls, balls as PhysX spheres). Each trial uses
  three toys; the other six rest on the floor behind the arms (not part of the trial, out of every camera's working view).
Geometry: ``../src/layout.py``; scorer: ``../src/scoring.py`` (pure functions).

Runtime hooks (installed on import; nothing in yam_arena is edited; same pattern as earlier task kits): after every
physics step the monitor reads the contact forces (robot on each toy, robot on the toddler); after every control
step the scorer reads the toy and tube poses and the finger pad tips. Nothing here writes a pose or a velocity during
a trial; ``Runtime.restore_authored`` places the tube and toys only on an environment reset. The scripted demo READS
poses through ``bench_state()``.
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

PHYSICS_DT = float(os.environ.get("CHOKE_PHYSICS_DT", str(1 / 480)))
CONTACT_OFFSET = float(os.environ.get("CHOKE_CONTACT_OFFSET", "0.0008"))
SETUP_BASE = int(os.environ.get("CHOKE_SETUP", "0"))
# Trials reset WITH a seed (every inspect-robots rollout passes one) draw the toys and their places from the seed
# (layout.setup_spec); CHOKE_SETUP set in the environment forces the named setups instead.
FORCE_NAMED = "CHOKE_SETUP" in os.environ
# development only (never set in evidence runs): place the listed toys, one per trial, just above the tube's rim
PROBE = [s for s in os.environ.get("CHOKE_PROBE", "").split(",") if s]
DEBUG = bool(os.environ.get("CHOKE_DEBUG"))

SKIN, SHIRT, PANTS, HAIR = (0.93, 0.76, 0.64), (0.95, 0.62, 0.70), (0.30, 0.42, 0.62), (0.40, 0.26, 0.13)
CHAIR_COL, MAT_COL = (0.95, 0.95, 0.93), (0.55, 0.78, 0.72)
BOX_COL, TUBE_COL = (0.92, 0.92, 0.90), (0.98, 0.86, 0.25)
KINEMATIC = ("toddler", "chair", "mat", "box")
FREE = ("tube",) + L.TOY_NAMES
BODIES = KINEMATIC + FREE


class Kitchen(Box):
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


def probe_pose(name):
    """Development probe: the toy centred 6 mm above the tube's rim, as it lies on the table."""
    tp, tr = L.authored(SETUP_BASE)["tube"]
    rim = L.tube_rim_center(tp, tr)
    lo = -min(L.toy_corners(name)[:, 2])
    return rim + np.array([0.0, 0.0, lo + 0.006]), np.eye(3)


def authored(setup, trial=0):
    a = L.authored(setup)
    if PROBE:
        n = PROBE[trial % len(PROBE)]
        a[n] = probe_pose(n)
    return a


_SPEC: dict = {}


def _spawn_bench(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics
    from scipy.spatial import ConvexHull
    from yam_arena.station import linear_rgb

    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(*(translation or (0.0, 0.0, 0.0)))
    )
    ox, oy, oz, ow = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Quatd(ow, ox, oy, oz)
    )
    looks, mats = {}, {}

    def look(color):
        if color not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(
                diffuse_color=linear_rgb(color), roughness=0.5
            )
            vis.func(path, vis)
            looks[color] = path
        return looks[color]

    def material(friction):
        if friction not in mats:
            path = f"{prim_path}/Looks/phys{len(mats)}"
            pm = sim_utils.RigidBodyMaterialCfg(
                static_friction=friction, dynamic_friction=friction
            )
            pm.func(path, pm)
            mats[friction] = path
        return mats[friction]

    def xform(path, pos, rot):
        x = UsdGeom.Xform.Define(stage, path)
        x.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(*map(float, pos))
        )
        q = _quat(rot)
        x.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Quatd(float(q[3]), *map(float, q[:3]))
        )
        return x

    def rigid(path, mass=None, kinematic=False, inertia=None, com=None, ang_damp=0.05):
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
        px.CreateSolverPositionIterationCountAttr(32)
        px.CreateSolverVelocityIterationCountAttr(4)
        px.CreateLinearDampingAttr(0.0)
        px.CreateAngularDampingAttr(float(ang_damp))
        px.CreateSleepThresholdAttr(0.0)  # never sleeps: a sleeping body reports no contacts
        px.CreateStabilizationThresholdAttr(0.0)
        if not kinematic:
            px.CreateEnableCCDAttr(True)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)

    count = {"n": 0}

    def collider(prim, color, friction):
        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(CONTACT_OFFSET)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), material(friction), stage=stage)
        bind_visual_material(str(prim.GetPath()), look(color), stage=stage)
        count["n"] += 1

    def hull(path, v, color, friction=0.4, collide=True):
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
        m.CreateExtentAttr(
            [Gf.Vec3f(*map(float, v.min(axis=0))), Gf.Vec3f(*map(float, v.max(axis=0)))]
        )
        prim = m.GetPrim()
        if collide:
            collider(prim, color, friction)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
            hc = PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim)
            hc.CreateHullVertexLimitAttr(255)
            hc.CreateMinThicknessAttr(0.0001)
        else:
            bind_visual_material(str(prim.GetPath()), look(color), stage=stage)
        return m

    def sphere(path, center, r, color, friction=0.4):
        s = UsdGeom.Sphere.Define(stage, path)
        s.CreateRadiusAttr(float(r))
        s.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(*map(float, center))
        )
        s.CreateExtentAttr([Gf.Vec3f(-r, -r, -r), Gf.Vec3f(r, r, r)])
        collider(s.GetPrim(), color, friction)
        return s

    def box(path, center, size, color, friction=0.4):
        return hull(path, L.box_pts(center, size), color, friction)

    def cyl(path, r, z0, z1, color, center=(0.0, 0.0), n=24, friction=0.4):
        return hull(path, L.cylinder(r, z0, z1, n=n, center=center), color, friction)

    # ---------------------------------------------------------------- placemat and storage box (kinematic)
    xform(f"{prim_path}/mat", (L.MAT[0], L.MAT[1], L.TOP), np.eye(3))
    rigid(f"{prim_path}/mat", 0.2, kinematic=True)
    box(f"{prim_path}/mat/sheet", (0, 0, L.MAT_SIZE[2] / 2), L.MAT_SIZE, MAT_COL, 0.8)
    xform(f"{prim_path}/box", (L.BOX[0], L.BOX[1], L.TOP), np.eye(3))
    rigid(f"{prim_path}/box", 1.0, kinematic=True)
    bx, by, bh = L.BOX_SIZE
    w = L.BOX_WALL
    box(f"{prim_path}/box/floor", (0, 0, w / 2), (bx, by, w), BOX_COL)
    for i, (c, s) in enumerate(
        (
            ((bx / 2 - w / 2, 0, bh / 2), (w, by, bh)),
            ((-bx / 2 + w / 2, 0, bh / 2), (w, by, bh)),
            ((0, by / 2 - w / 2, bh / 2), (bx, w, bh)),
            ((0, -by / 2 + w / 2, bh / 2), (bx, w, bh)),
        )
    ):
        box(f"{prim_path}/box/wall_{i}", c, s, BOX_COL)
    # ---------------------------------------------------------------- high chair (kinematic), toddler (kinematic)
    cx, cy = L.CHAIR
    fz = L.FLOOR_Z
    seat = L.CHAIR_SEAT_Z
    xform(f"{prim_path}/chair", (cx, cy, 0.0), np.eye(3))
    rigid(f"{prim_path}/chair", 5.0, kinematic=True)
    for i, (dx, dy) in enumerate(((-0.17, -0.16), (0.17, -0.16), (-0.17, 0.10), (0.17, 0.10))):
        cyl(f"{prim_path}/chair/leg_{i}", 0.014, fz, seat - 0.02, CHAIR_COL, (dx, dy))
    box(f"{prim_path}/chair/seat", (0, -0.02, seat - 0.01), (0.34, 0.26, 0.03), CHAIR_COL)
    box(f"{prim_path}/chair/back", (0, -0.15, seat + 0.22), (0.34, 0.03, 0.44), CHAIR_COL)
    box(f"{prim_path}/chair/footrest", (0, 0.13, seat - 0.30), (0.30, 0.10, 0.015), CHAIR_COL)
    for s in (-1, 1):
        box(f"{prim_path}/chair/arm_{s + 1}", (s * 0.16, -0.02, seat + 0.13), (0.025, 0.24, 0.025), CHAIR_COL)
    xform(f"{prim_path}/toddler", (cx, cy, 0.0), np.eye(3))
    rigid(f"{prim_path}/toddler", 10.0, kinematic=True)
    sz = seat + 0.005
    sy = L.SHOULDER[1] - cy  # torso centre relative to the chair
    sh = L.SHOULDER[2]
    box(f"{prim_path}/toddler/hips", (0, sy - 0.01, sz + 0.05), (0.20, 0.14, 0.10), PANTS)
    for s in (-1, 1):  # thighs forward on the seat, shins down to the footrest
        box(f"{prim_path}/toddler/thigh_{s + 1}", (s * 0.05, sy + 0.08, sz + 0.04), (0.075, 0.17, 0.075), PANTS)
        box(f"{prim_path}/toddler/shin_{s + 1}", (s * 0.05, sy + 0.15, sz - 0.11), (0.06, 0.06, 0.22), PANTS)
    box(f"{prim_path}/toddler/torso", (0, sy, (sz + 0.10 + sh + 0.02) / 2), (0.21, 0.13, sh + 0.02 - sz - 0.10), SHIRT)
    sphere(f"{prim_path}/toddler/head", (0, sy + 0.01, sh + 0.085), 0.072, SKIN)
    sphere(f"{prim_path}/toddler/hair", (0, sy - 0.01, sh + 0.105), 0.066, HAIR)
    elbow = L.TOP + 0.03
    for s in (-1, 1):  # upper arms down to the table, forearms forward on the table
        box(f"{prim_path}/toddler/upper_arm_{s + 1}", (s * 0.125, sy + 0.03, (sh + elbow) / 2), (0.05, 0.05, sh - elbow), SHIRT)
        box(f"{prim_path}/toddler/forearm_{s + 1}", (s * 0.11, sy + 0.12, elbow - 0.002), (0.045, 0.15, 0.045), SKIN)
        sphere(f"{prim_path}/toddler/hand_{s + 1}", (s * 0.10, sy + 0.215, elbow), 0.027, SKIN)
    # ---------------------------------------------------------------- testing tube (free)
    a0 = authored(SETUP_BASE)
    xform(f"{prim_path}/tube", *a0["tube"])
    rigid(f"{prim_path}/tube", L.TUBE_MASS, inertia=L.tube_inertia(), com=L.tube_com())
    for i, v in enumerate(L.tube_wall_pieces()):
        hull(f"{prim_path}/tube/wall_{i:02d}", v, TUBE_COL, L.TUBE_FRICTION)
    hull(f"{prim_path}/tube/floor", L.tube_floor_piece(), TUBE_COL, L.TUBE_FRICTION)
    # ---------------------------------------------------------------- toys (free)
    for name in L.TOY_NAMES:
        t = L.TOYS[name]
        xform(f"{prim_path}/{name}", *a0[name])
        rigid(
            f"{prim_path}/{name}",
            t["mass"],
            inertia=L.toy_inertia(name),
            ang_damp=L.BALL_ANG_DAMPING if t["kind"] == "ball" else 0.05,
        )
        if t["kind"] == "ball":
            sphere(f"{prim_path}/{name}/solid", (0, 0, 0), t["size"][0] / 2, t["color"], t["friction"])
        elif t["kind"] in ("cyl", "doll", "stick"):
            hull(f"{prim_path}/{name}/solid", L.toy_hull_points(name, n=32), t["color"], t["friction"])
        else:
            box(f"{prim_path}/{name}/solid", (0, 0, 0), t["size"], t["color"], t["friction"])
        if t["kind"] == "brick":
            k, pitch, sr, sh = t["studs"]
            h = t["size"][2] / 2
            for ix in range(k):
                for iy in range(k):
                    c = ((ix - (k - 1) / 2) * pitch, (iy - (k - 1) / 2) * pitch)
                    cyl(f"{prim_path}/{name}/stud_{ix}{iy}", sr, h - 0.0002, h + sh, t["color"], c, 16, t["friction"])
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
    """Reads the contact forces every physics step and scores every control step."""

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
        root = "/World/envs/env_0/kitchen"
        P = lambda n: f"{root}/{n}"  # noqa: E731
        self.view = self.simview.create_rigid_body_view([P(n) for n in FREE])
        robot_links = []
        for arm in ("left_robot", "right_robot"):
            lp = self.base.scene[arm].root_view.link_paths
            robot_links += list(lp[0] if lp and isinstance(lp[0], (list, tuple)) else lp)
        self.robot_links = robot_links
        self.cv_toys = self.simview.create_rigid_contact_view(
            [P(n) for n in L.TOY_NAMES],
            filter_patterns=[robot_links for _ in L.TOY_NAMES],
            max_contact_data_count=8192,
        )
        self.cv_kid = self.simview.create_rigid_contact_view(
            [P("toddler")], filter_patterns=[robot_links], max_contact_data_count=2048
        )
        self.cv_tube = self.simview.create_rigid_contact_view(
            [P("tube")], filter_patterns=[robot_links], max_contact_data_count=2048
        )
        self.origin = self.base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        self.setup = SETUP_BASE
        self.trial = 0
        self.trial_active = False
        self._init_state()

    def _init_state(self):
        self.monitor = Monitor()
        self.scorer = Scorer(L.active_toys(self.setup))
        self.steps = 0
        self.t = 0.0
        self.hook_s = 0.0
        self.nonfinite = 0
        self.touch_acc = {n: 0.0 for n in L.TOY_NAMES}
        self.tube_grip_peak = 0.0
        STATE.update(score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, setup=self.setup)

    def begin_trial(self, trial):
        self._init_state()
        self.trial_active = True
        STATE.update(trial=trial, setup=self.setup, start=L.setup_spec(self.setup))

    # ------------------------------------------------------------------ reading
    def read(self):
        tf = self.view.get_transforms().detach().cpu().numpy().astype(float)
        return {n: (tf[i, :3] - self.origin, _rot(tf[i, 3:7])) for i, n in enumerate(FREE)}

    def velocities(self):
        v = self.view.get_velocities().detach().cpu().numpy().astype(float)
        return {n: (v[i, :3], v[i, 3:6]) for i, n in enumerate(FREE)}

    def physics(self, dt):
        t0 = time.perf_counter()
        self.t += dt
        if self.trial_active:
            ft = self.cv_toys.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
            fk = self.cv_kid.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
            fu = self.cv_tube.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
            for i, n in enumerate(L.TOY_NAMES):
                f = float(np.linalg.norm(ft[i], axis=-1).max()) if ft.size else 0.0
                self.touch_acc[n] = max(self.touch_acc[n], f)
            kid_n = float(np.linalg.norm(fk[0], axis=-1).max()) if fk.size else 0.0
            tube_n = float(np.linalg.norm(fu[0], axis=-1).max()) if fu.size else 0.0
            self.tube_grip_peak = max(self.tube_grip_peak, tube_n)
            self.monitor.step(dt, kid_n)
        self.hook_s += time.perf_counter() - t0

    # ------------------------------------------------------------------ reset
    def restore_authored(self, setup):
        """Environment reset only (never during a trial): tube and toys to the setup's start poses, at rest."""
        torch = self.torch
        self.setup = setup
        self.trial_active = False
        a = authored(setup, self.trial)
        tf = self.view.get_transforms().clone()
        for i, n in enumerate(FREE):
            pos, rot = a[n]
            tf[i, :3] = torch.tensor(np.asarray(pos, float) + self.origin, dtype=tf.dtype)
            tf[i, 3:7] = torch.tensor(_quat(rot), dtype=tf.dtype)
        idx = torch.arange(len(FREE), dtype=torch.int32, device=tf.device)
        self.view.set_transforms(tf, idx)
        self.view.set_velocities(torch.zeros_like(self.view.get_velocities()), idx)
        self._init_state()

    # ------------------------------------------------------------------ scoring
    def control(self, environment):
        if not self.trial_active:
            return
        self.steps += 1
        poses = self.read()
        vels = self.velocities()
        if not all(np.all(np.isfinite(p)) for p, _ in poses.values()):
            self.nonfinite += 1
            STATE["nonfinite"] = self.nonfinite
            return
        tips = finger_tips(environment)
        touched = {n: self.touch_acc[n] for n in L.TOY_NAMES}
        self.touch_acc = {n: 0.0 for n in L.TOY_NAMES}
        level = self.scorer.step(poses, vels, touched, tips, self.monitor)
        STATE.update(
            score=level,
            level=level,
            steps=self.steps,
            nonfinite=self.nonfinite,
            detail=dict(
                self.scorer.last,
                setup=str(self.setup),
                tube_robot_peak_n=round(self.tube_grip_peak, 2),
                hook_wall_s=round(self.hook_s, 2),
            ),
        )


_RT: dict = {"rt": None, "sim": None, "trials": 0}


def runtime():
    return _RT["rt"]


def bench_state():
    """Current poses {'tube', <toy>...} for the scripted demo (read-only)."""
    return _RT["rt"].read() if _RT["rt"] else None


def active_toys():
    rt = _RT["rt"]
    return L.active_toys(rt.setup if rt else SETUP_BASE)


def _patch_scene(scene) -> None:
    if getattr(scene, "_choke_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Kitchen):
            return _arena_object(spec)
        return orig_task_object(spec)

    def object_states(environment, names):
        from yam_arena.task_file import ObjectState

        plain = [n for n in names if n != "kitchen"]
        out = orig_states(environment, plain) if plain else {}
        if "kitchen" in names:
            rt = _RT["rt"]
            p, r = rt.read()["tube"] if rt else authored(SETUP_BASE)["tube"]
            z = np.zeros(3)
            out["kitchen"] = ObjectState(
                pos=np.asarray(p), quat_xyzw=_quat(r), lin_vel=z, ang_vel=z, low=p - 0.2, high=p + 0.2
            )
        return {n: out[n] for n in names}

    orig_build = scene.build_task_environment

    def build(embodiment, task, control_hz=None):
        environment = orig_build(embodiment, task, control_hz)
        if any(isinstance(o, Kitchen) for o in task.objects):
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
    scene._choke_patched = True


def _has_kitchen(emb):
    return any(isinstance(o, Kitchen) for o in (emb.task.objects if emb.task is not None else ()))


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_choke_patched", False):
        return
    base_task_step, base_reset, base_ensure = cls._task_step, cls.reset, cls._ensure_sim

    def _ensure_sim(self, *a, **k):
        env_before = self._env
        out = base_ensure(self, *a, **k)
        if self._env is not None and self._env is not env_before and _has_kitchen(self):
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
            setup = (SETUP_BASE + _RT["trials"]) % len(L.NAMED_SETUPS)
        if rt is not None:
            rt.trial = _RT["trials"]
            rt.restore_authored(setup)
        obs = base_reset(self, *a, **k)
        rt = _RT["rt"]
        if rt is not None:
            if rt.setup != setup or rt.trial != _RT["trials"]:  # first reset of a fresh env
                rt.trial = _RT["trials"]
                rt.restore_authored(setup)
                obs = base_reset(self, *a, **k)
            rt.begin_trial(_RT["trials"])
            _RT["trials"] += 1
        return obs

    cls._ensure_sim, cls._task_step, cls.reset = _ensure_sim, _task_step, reset
    cls._choke_patched = True


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
        type(f).__name__ == "_Finder" and getattr(f, "targets", None) and "yam_arena.scene" in f.targets
        for f in sys.meta_path
    ):
        sys.meta_path.insert(0, _Finder())


install()
