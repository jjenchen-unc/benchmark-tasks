"""ringkit: the etch-chamber cathode, the new edge ring, its stand and the maintenance pendant for yam_arena
(Isaac Lab-Arena / PhysX), plus the runtime hooks.

Authored as one custom-spawned assembly (yam_arena task-file primitives are single boxes/spheres):
* chamber parts as kinematic rigid bodies (so PhysX contact views can report the forces on them): the chuck puck, the
  quartz base ring with its three locating dowels, the quartz cover ring, the lower chamber body (bowl and cathode);
* the lift-pin carrier (three sapphire pins) as a kinematic body: a position-controlled device that the pendant keys
  move between the maintenance position (tips 25 mm above the seat) and lowered (1 mm below) at 5 mm/s;
* the ring stand (post + three arms) and the pendant with its two keys (kinematic);
* the new edge ring as a free rigid body made of 131 convex sector pieces with its three radial slots cut out.
Geometry: ``../src/layout.py``; scorer: ``../src/scoring.py`` (pure functions).

Runtime hooks (installed on import; nothing in yam_arena is edited; same pattern as earlier task kits): after every
physics step the monitor reads the contact forces (chuck touched, free fall, pressing) and the pendant keys, and moves
the lift-pin carrier; after every control step the scorer reads the ring pose and the finger pad tips. Nothing here
writes the ring's pose or velocity during a trial; ``Runtime.restore_authored`` puts the ring back on the stand (and
the carrier up) only on an environment reset. The scripted demo READS poses through ``bench_state()``.
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

PHYSICS_DT = float(os.environ.get("EDGE_PHYSICS_DT", str(1 / 480)))
CONTACT_OFFSET = float(
    os.environ.get("EDGE_CONTACT_OFFSET", ".00005")
)  # below the 0.1 mm dowel clearance and the 0.45 mm bore gap
SETUP_BASE = int(
    os.environ.get("EDGE_SETUP", "0")
)  # named setup of the first unseeded trial; later trials cycle
# Trials reset WITH a seed (every inspect-robots rollout passes one) draw the ring's stand angle and the stand offset
# from the seed (layout.setup_spec); EDGE_SETUP set in the environment forces the named setups instead.
FORCE_NAMED = "EDGE_SETUP" in os.environ
LIFT_WRITE = os.environ.get(
    "EDGE_LIFT_WRITE", "kinematic"
)  # 'kinematic' targets or 'transform' (fallback)
DEBUG = bool(os.environ.get("EDGE_DEBUG"))

SILICON, QUARTZ, CERAMIC, ALU, WHITE, BLACK = (
    (0.22, 0.24, 0.28),
    (0.86, 0.88, 0.9),
    (0.80, 0.78, 0.74),
    (0.62, 0.64, 0.67),
    (0.96, 0.96, 0.94),
    (0.08, 0.08, 0.09),
)
KEY_UP_COL, KEY_DOWN_COL, PENDANT_COL, SAPPHIRE = (
    (0.15, 0.45, 0.85),
    (0.95, 0.55, 0.1),
    (0.18, 0.19, 0.2),
    (0.7, 0.8, 0.9),
)
KINEMATIC = (
    "puck",
    "base",
    "cover",
    "bowl",
    "lifter",
    "stand",
    "pendant",
    "key_up",
    "key_down",
)
BODIES = KINEMATIC + ("ring",)
SUPPORT = ("lifter", "base", "cover", "bowl", "stand", "puck")


class Chamber(Box):
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
    """Authored (reset) pose of the moving bodies for a setup: {name: (pos, rot)} in the env frame."""
    rp, rr = L.ring_stand_pose(setup)
    return {
        "ring": (rp, rr),
        "lifter": (L.C + np.array([0.0, 0.0, L.LIFT_UP]), np.eye(3)),
        "stand": (L.stand_pos(setup), np.eye(3)),
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
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Quatd(ow, ox, oy, oz)
    )
    looks, mats = {}, {}

    def look(color):
        if color not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(
                diffuse_color=linear_rgb(color), roughness=0.45
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

    def rigid(path, mass=None, kinematic=False, inertia=None, com=None):
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
        px.CreateAngularDampingAttr(0.05)
        px.CreateSleepThresholdAttr(
            0.0
        )  # never sleeps: a sleeping body reports no contacts
        px.CreateStabilizationThresholdAttr(0.0)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)

    count = {"n": 0}

    def hull(path, v, color, friction=L.HARD_FRICTION, approximation="convexHull"):
        v = np.asarray(v, float)
        h = ConvexHull(v)
        m = UsdGeom.Mesh.Define(stage, path)
        m.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in v])
        m.CreateFaceVertexCountsAttr([3] * len(h.simplices))
        # outward winding
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
        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(CONTACT_OFFSET)
        cp.CreateRestOffsetAttr(0.0)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(approximation)
        if approximation == "convexHull":
            hc = PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim)
            hc.CreateHullVertexLimitAttr(255)
            hc.CreateMinThicknessAttr(0.0001)
        bind_physics_material(str(prim.GetPath()), material(friction), stage=stage)
        bind_visual_material(str(prim.GetPath()), look(color), stage=stage)
        count["n"] += 1
        return m

    def box(path, center, size, color, friction=L.HARD_FRICTION, yaw_deg=0.0):
        hx, hy, hz = (s / 2 for s in size)
        corners = np.array(
            [
                [sx * hx, sy * hy, sz * hz]
                for sx in (-1, 1)
                for sy in (-1, 1)
                for sz in (-1, 1)
            ]
        )
        corners = corners @ L.rz(yaw_deg).T + np.asarray(center, float)
        return hull(path, corners, color, friction)

    def annulus(path, r0, r1, z0, z1, color, sector=6.0, friction=L.HARD_FRICTION):
        for i, cell in enumerate(L.annulus_cells(r0, r1, sector)):
            hull(f"{path}_{i:03d}", L.prism(cell, z0, z1), color, friction)

    zc = L.C
    # ---------------------------------------------------------------- chamber (kinematic, chamber frame at the seat)
    for name in ("puck", "base", "cover", "bowl"):
        xform(f"{prim_path}/{name}", zc, np.eye(3))
        rigid(f"{prim_path}/{name}", 5.0, kinematic=True)
    # exact triangle mesh (kinematic body): a cooked convex hull of the 298 mm disc came out larger than authored and
    # closed the 0.45 mm gap to the ring's bore
    hull(
        f"{prim_path}/puck/face",
        L.cylinder(L.PUCK_R, L.PUCK_Z0, L.PUCK_TOP, n=180),
        CERAMIC,
        approximation="none",
    )
    for i, v in enumerate(L.base_ring_pieces()):
        hull(f"{prim_path}/base/q_{i:03d}", v, QUARTZ)
    for i, c in enumerate(L.loc_pin_centers()):
        hull(
            f"{prim_path}/base/dowel_{i}",
            L.cylinder(
                L.LOC_PIN_R, -0.004, L.LOC_PIN_H, n=32, chamfer=L.LOC_CHAMFER, center=c
            ),
            WHITE,
        )
    annulus(
        f"{prim_path}/cover/q",
        L.COVER_R[0],
        L.COVER_R[1],
        L.COVER_Z0,
        L.COVER_TOP,
        QUARTZ,
        sector=4.0,
    )
    annulus(
        f"{prim_path}/bowl/wall",
        L.BOWL_R[0],
        L.BOWL_R[1],
        L.BOWL_FLOOR,
        L.BOWL_TOP,
        ALU,
        sector=6.0,
    )
    hull(
        f"{prim_path}/bowl/floor",
        L.cylinder(L.BOWL_R[0], L.BOWL_FLOOR, L.BOWL_FLOOR + 0.003, n=64),
        ALU,
    )
    # cathode body under the rings (between the floor and the base ring / cover ring bottoms)
    hull(
        f"{prim_path}/bowl/cathode",
        L.cylinder(L.CATHODE_R, L.BOWL_FLOOR + 0.003, L.COVER_Z0 - 0.0005, n=96),
        ALU,
    )
    # ---------------------------------------------------------------- lift-pin carrier (kinematic device)
    lp, lr = authored(SETUP_BASE)["lifter"]
    xform(f"{prim_path}/lifter", lp, lr)
    rigid(f"{prim_path}/lifter", 0.2, kinematic=True)
    for i, c in enumerate(L.lift_pin_centers()):
        hull(
            f"{prim_path}/lifter/pin_{i}",
            L.cylinder(L.LIFT_PIN_R, -0.040, 0.0, n=24, center=c),
            SAPPHIRE,
        )
    # ---------------------------------------------------------------- ring stand
    xform(f"{prim_path}/stand", *authored(SETUP_BASE)["stand"])
    rigid(f"{prim_path}/stand", 3.0, kinematic=True)
    hull(
        f"{prim_path}/stand/post",
        L.cylinder(L.STAND_POST_R, L.TOP + 0.001 - L.STAND[2], -L.STAND_ARM_T, n=32),
        BLACK,
    )
    for i, a in enumerate(L.STAND_ARM_ANGLES):
        r0, r1 = L.STAND_ARM_R
        cen = L.rz(a) @ np.array([(r0 + r1) / 2, 0.0, -L.STAND_ARM_T / 2])
        box(
            f"{prim_path}/stand/arm_{i}",
            cen,
            (r1 - r0, L.STAND_ARM_W, L.STAND_ARM_T),
            BLACK,
            yaw_deg=a,
        )
    # ---------------------------------------------------------------- pendant and keys
    pc = L.PENDANT + np.array([0.0, 0.0, L.PENDANT_SIZE[2] / 2])
    xform(f"{prim_path}/pendant", pc, np.eye(3))
    rigid(f"{prim_path}/pendant", 1.0, kinematic=True)
    box(f"{prim_path}/pendant/body", (0.0, 0.0, 0.0), L.PENDANT_SIZE, PENDANT_COL)
    for name, col in (("key_up", KEY_UP_COL), ("key_down", KEY_DOWN_COL)):
        kc = L.key_center(name.split("_")[1])
        xform(f"{prim_path}/{name}", kc, np.eye(3))
        rigid(f"{prim_path}/{name}", 0.01, kinematic=True)
        box(f"{prim_path}/{name}/cap", (0.0, 0.0, 0.0), L.KEY_SIZE, col)
    # ---------------------------------------------------------------- the new edge ring (free body)
    rp, rr = authored(SETUP_BASE)["ring"]
    xform(f"{prim_path}/ring", rp, rr)
    rigid(
        f"{prim_path}/ring",
        L.RING_MASS,
        inertia=L.ring_inertia(),
        com=(0.0, 0.0, L.RING_T / 2),
    )
    for i, v in enumerate(L.ring_pieces()):
        hull(f"{prim_path}/ring/si_{i:03d}", v, SILICON, friction=L.RING_FRICTION)
    # robot never collides with... (nothing filtered: every contact is real)
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
        initial_pose=Pose(
            position_xyz=(0.0, 0.0, 0.0), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)
        ),
    )


# ---------------------------------------------------------------------------------------------- runtime state
STATE: dict = dict(
    score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, trial=-1, setup=None
)


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
    """Moves the lift-pin carrier, reads the contact forces every physics step, scores every control step."""

    def __init__(self, environment):
        import carb
        import omni.physics.tensors as tensors
        import torch
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdUtils

        # PhysX contact reports: Isaac Lab turns contact processing off; the CPU pipeline then reports nothing
        carb.settings.get_settings().set_bool(
            "/physics/disableContactProcessing", False
        )
        self.base = environment.unwrapped
        self.env = environment
        self.torch = torch
        stage = get_current_stage()
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        self.simview = tensors.create_simulation_view("torch", stage_id=stage_id)
        self.simview.set_subspace_roots("/")
        root = "/World/envs/env_0/chamber"
        P = lambda n: f"{root}/{n}"  # noqa: E731
        self.view = self.simview.create_rigid_body_view(
            [P("ring"), P("lifter"), P("stand")]
        )
        robot_links = []
        for arm in ("left_robot", "right_robot"):
            lp = self.base.scene[arm].root_view.link_paths
            robot_links += list(
                lp[0] if lp and isinstance(lp[0], (list, tuple)) else lp
            )
        self.robot_links = robot_links
        self.cv_puck = self.simview.create_rigid_contact_view(
            [P("puck")],
            filter_patterns=[[P("ring")] + robot_links],
            max_contact_data_count=4096,
        )
        self.cv_ring = self.simview.create_rigid_contact_view(
            [P("ring")],
            filter_patterns=[[P(n) for n in SUPPORT] + robot_links],
            max_contact_data_count=8192,
        )
        self.cv_ring_net = self.simview.create_rigid_contact_view([P("ring")])
        self.cv_keys = self.simview.create_rigid_contact_view(
            [P("key_up"), P("key_down")],
            filter_patterns=[robot_links, robot_links],
            max_contact_data_count=1024,
        )
        self.n_support = len(SUPPORT)
        self.origin = (
            self.base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        )
        self.setup = SETUP_BASE
        self.trial_active = False
        self.lift_z = L.LIFT_UP
        self.lift_target = L.LIFT_UP
        self._init_state()

    def _init_state(self):
        self.monitor = Monitor()
        self.scorer = Scorer()
        self.steps = 0
        self.t = 0.0
        self.key_s = {"up": 0.0, "down": 0.0}
        self.key_peak = {"up": 0.0, "down": 0.0}
        self.key_events = []
        self.hook_s = 0.0
        self.nonfinite = 0
        self.lift_write_mode = LIFT_WRITE
        STATE.update(
            score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, setup=self.setup
        )

    def begin_trial(self, trial):
        self._init_state()
        self.trial_active = True
        STATE.update(trial=trial, setup=self.setup, start=L.setup_spec(self.setup))

    # ------------------------------------------------------------------ reading
    def read(self):
        tf = self.view.get_transforms().detach().cpu().numpy().astype(float)
        return {
            "ring": (tf[0, :3] - self.origin, _rot(tf[0, 3:7])),
            "lifter": (tf[1, :3] - self.origin, _rot(tf[1, 3:7])),
        }

    def ring_velocity(self):
        v = self.view.get_velocities().detach().cpu().numpy().astype(float)
        return v[0, :3], v[0, 3:6]

    # ------------------------------------------------------------------ device
    def write_lifter(self, z):
        torch = self.torch
        tf = self.view.get_transforms().clone()
        tf[1, :3] = torch.tensor(
            L.C + np.array([0.0, 0.0, z]) + self.origin, dtype=tf.dtype
        )
        tf[1, 3:7] = torch.tensor([0.0, 0.0, 0.0, 1.0], dtype=tf.dtype)
        idx = torch.tensor([1], dtype=torch.int32, device=tf.device)
        if self.lift_write_mode == "kinematic":
            try:
                self.view.set_kinematic_targets(tf, idx)
                return
            except Exception as e:  # noqa: BLE001
                print(
                    "ringkit: set_kinematic_targets failed, falling back to set_transforms:",
                    e,
                    flush=True,
                )
                self.lift_write_mode = "transform"
        self.view.set_transforms(tf, idx)

    def physics(self, dt):
        t0 = time.perf_counter()
        self.t += dt
        # lift-pin carrier: toward its target at LIFT_SPEED
        dz = float(
            np.clip(
                self.lift_target - self.lift_z, -L.LIFT_SPEED * dt, L.LIFT_SPEED * dt
            )
        )
        if dz != 0.0:
            self.lift_z += dz
            self.write_lifter(self.lift_z)
        if self.trial_active:
            fp = self.cv_puck.get_contact_force_matrix(dt=dt).detach().cpu().numpy()[0]
            fr = self.cv_ring.get_contact_force_matrix(dt=dt).detach().cpu().numpy()[0]
            net = (
                self.cv_ring_net.get_net_contact_forces(dt=dt).detach().cpu().numpy()[0]
            )
            fk = self.cv_keys.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
            chuck_n = float(np.linalg.norm(fp, axis=-1).max()) if fp.size else 0.0
            support = fr[: self.n_support].sum(axis=0)
            robot_on_ring = (
                float(np.linalg.norm(fr[self.n_support :], axis=-1).max())
                if fr.shape[0] > self.n_support
                else 0.0
            )
            vz = float(self.view.get_velocities().detach().cpu().numpy()[0, 2])
            ring_dz = float(
                self.view.get_transforms().detach().cpu().numpy()[0, 2]
                - self.origin[2]
                - L.C[2]
            )
            base_n = float(np.linalg.norm(fr[SUPPORT.index("base")]))
            self.monitor.step(
                dt,
                chuck_n,
                vz,
                float(np.linalg.norm(support)),
                robot_on_ring,
                base_n=base_n,
                ring_dz=ring_dz,
            )
            self.last_contacts = dict(
                chuck_n=round(chuck_n, 4),
                net_n=round(float(np.linalg.norm(net)), 4),
                support_n=round(float(np.linalg.norm(support)), 4),
                support_by=[
                    round(float(np.linalg.norm(f)), 3) for f in fr[: self.n_support]
                ],
                robot_on_ring_n=round(robot_on_ring, 4),
                vz=round(vz, 4),
                base_n=round(base_n, 4),
                ring_dz_mm=round(ring_dz * 1e3, 4),
            )
            if DEBUG and int(round(self.t / dt)) % 24 == 0:
                print("CONTACTS", self.last_contacts, flush=True)
            for k, name in enumerate(("up", "down")):
                f = float(np.linalg.norm(fk[k], axis=-1).max()) if fk.size else 0.0
                self.key_peak[name] = max(self.key_peak[name], f)
                self.key_s[name] = self.key_s[name] + dt if f >= L.KEY_PRESS_N else 0.0
                if self.key_s[name] >= L.KEY_HOLD_S - 1e-9:
                    want = L.LIFT_UP if name == "up" else L.LIFT_DOWN
                    if want != self.lift_target:
                        self.lift_target = want
                        self.key_events.append(
                            dict(key=name, t=round(self.t, 3), step=self.steps)
                        )
        self.hook_s += time.perf_counter() - t0

    # ------------------------------------------------------------------ reset
    def restore_authored(self, setup):
        """Environment reset only (never during a trial): the ring back on the stand at the setup's angle, at rest; the
        lift-pin carrier back up."""
        torch = self.torch
        self.setup = setup
        self.trial_active = False
        a = authored(setup)
        tf = self.view.get_transforms().clone()
        for i, n in enumerate(("ring", "lifter", "stand")):
            pos, rot = a[n]
            tf[i, :3] = torch.tensor(
                np.asarray(pos, float) + self.origin, dtype=tf.dtype
            )
            tf[i, 3:7] = torch.tensor(_quat(rot), dtype=tf.dtype)
        idx = torch.tensor([0, 1, 2], dtype=torch.int32, device=tf.device)
        self.view.set_transforms(tf, idx)
        self.view.set_velocities(
            torch.zeros_like(self.view.get_velocities()),
            torch.tensor([0], dtype=torch.int32, device=tf.device),
        )
        self.lift_z = self.lift_target = L.LIFT_UP
        self.write_lifter(self.lift_z)
        self._init_state()

    # ------------------------------------------------------------------ scoring
    def control(self, environment):
        if not self.trial_active:
            return
        self.steps += 1
        poses = self.read()
        pr, rr = poses["ring"]
        lin, ang = self.ring_velocity()
        if not (
            np.all(np.isfinite(pr))
            and np.all(np.isfinite(rr))
            and np.all(np.isfinite(lin))
        ):
            self.nonfinite += 1
            STATE["nonfinite"] = self.nonfinite
            return
        tips = finger_tips(environment)
        m = measure(pr, rr, self.lift_z, tips)
        level = self.scorer.step(
            m,
            float(np.linalg.norm(lin)),
            math.degrees(float(np.linalg.norm(ang))),
            self.monitor,
        )
        STATE.update(
            score=level,
            level=level,
            steps=self.steps,
            nonfinite=self.nonfinite,
            detail=dict(
                self.scorer.last,
                lift_target_mm=round(self.lift_target * 1e3, 2),
                keys=self.key_events,
                key_peak_n={k: round(v, 2) for k, v in self.key_peak.items()},
                ring_pos_mm=np.round(pr * 1000, 3).tolist(),
                setup=self.setup,
                hook_wall_s=round(self.hook_s, 2),
                lift_write=self.lift_write_mode,
                contacts=getattr(self, "last_contacts", None),
            ),
        )


_RT: dict = {"rt": None, "sim": None, "trials": 0}


def runtime():
    return _RT["rt"]


def bench_state():
    """Current poses {'ring', 'lifter'} for the scripted demo (read-only)."""
    return _RT["rt"].read() if _RT["rt"] else None


def _patch_scene(scene) -> None:
    if getattr(scene, "_edge_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Chamber):
            return _arena_object(spec)
        return orig_task_object(spec)

    def object_states(environment, names):
        from yam_arena.task_file import ObjectState

        plain = [n for n in names if n != "chamber"]
        out = orig_states(environment, plain) if plain else {}
        if "chamber" in names:
            rt = _RT["rt"]
            p, r = rt.read()["ring"] if rt else authored(SETUP_BASE)["ring"]
            z = np.zeros(3)
            out["chamber"] = ObjectState(
                pos=np.asarray(p),
                quat_xyzw=_quat(r),
                lin_vel=z,
                ang_vel=z,
                low=p - 0.2,
                high=p + 0.2,
            )
        return {n: out[n] for n in names}

    orig_build = scene.build_task_environment

    def build(embodiment, task, control_hz=None):
        environment = orig_build(embodiment, task, control_hz)
        if any(isinstance(o, Chamber) for o in task.objects):
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
    scene._edge_patched = True


def _has_chamber(emb):
    return any(
        isinstance(o, Chamber)
        for o in (emb.task.objects if emb.task is not None else ())
    )


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_edge_patched", False):
        return
    base_task_step, base_reset, base_ensure = cls._task_step, cls.reset, cls._ensure_sim

    def _ensure_sim(self, *a, **k):
        env_before = self._env
        out = base_ensure(self, *a, **k)
        if self._env is not None and self._env is not env_before and _has_chamber(self):
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
            setup = (SETUP_BASE + _RT["trials"]) % len(L.SETUP_CLOCKING)
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
    cls._edge_patched = True


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
