"""hingekit: the wall-cabinet carcass with two clip-on mounting plates, the door with its two clip-on hinges, the
2x4 blocks and the groove block, for yam_arena (Isaac Lab-Arena / PhysX), plus the runtime hooks.

Authored as one custom-spawned assembly (yam_arena task-file primitives are single boxes/spheres):
* carcass + the two mounting plates: one kinematic body (never moves during a trial; an environment reset places it);
* the two 2x4 offcuts under it and the groove block: kinematic;
* the door as one floating articulation of three links: the door slab (root, with the two hinge cups) and the two
  hinge arms, each on a revolute joint on the hinge (knuckle) axis, hard limits 0 .. 105 deg of door opening, with a
  capped damping drive (pivot friction, 0.3 N m). Arm links do not collide with the carcass/plates (filtered pairs);
  their contact with the plate is the computed plate model below. Arms collide with the robot.
Geometry: ``../src/layout.py``; scorer: ``../src/scoring.py`` (pure functions).

Plate model (computed forces at the arm links, every physics step; values in layout.py):
* FREE: no plate force.
* HOOKED (the arm's front lug sits on the plate's nose): entered only when the lug comes DOWN onto the nose (from at
  least 0.5 mm above its seat to at most 3 mm below it) while lined up with it (+-3 mm along the plate and against the
  panel side of the nose) and with the arm's rear clear of the rear catch (rear 1.5 .. 12 mm off the plate). The lug
  is then held on its seat by a stiff spring-damper (like the hook's sides and seat); it rolls off (FREE) if pulled
  4 mm off the seat (about 60 N), if the rear swings more than 14 mm off the plate, or if the arm tilts by 10 deg.
  The rear: a stop at the plate (no penetration) and the latch detent, which pushes the rear 22 N away from the plate
  over the last 2 mm of its travel. Pushing the rear in past that force to within 0.2 mm of seated clicks the latch.
* CLIPPED: the arm is held on its seated pose by three stiff point springs (hook, rear, top). It never releases (the
  real release lever is not part of this task).
Hinge springs: a self-closing torque (0.6 N m per hinge below 35 deg of opening) and soft-close damping below 20 deg,
applied as joint efforts on the knuckle joints.
Nothing here writes a pose, a joint state or a score during a trial; ``Runtime.restore_authored`` runs only on an
environment reset. The scripted demo READS poses through ``hinge_state()``.
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
from scoring import measure, Scorer  # noqa: E402
from yam_arena.task_file import Box  # noqa: E402

PHYSICS_DT = float(os.environ.get("CDH_PHYSICS_DT", str(1 / 480)))
CONTACT_OFFSET = 0.002
SETUP_BASE = int(os.environ.get("CDH_SETUP", "0"))
FORCE_NAMED = "CDH_SETUP" in os.environ
DEBUG = bool(os.environ.get("CDH_DEBUG"))
PROBE = os.environ.get("CDH_PROBE", "")  # development probes only (dev/); empty in every trial
DEG = math.pi / 180
FREE, HOOKED, CLIPPED = "free", "hooked", "clipped"

COLORS = dict(
    white=(0.93, 0.93, 0.91),
    back=(0.85, 0.85, 0.83),
    pine=(0.86, 0.72, 0.50),
    door=(0.50, 0.62, 0.70),
    edge=(0.40, 0.50, 0.57),
    steel=(0.72, 0.74, 0.76),
    plate=(0.60, 0.62, 0.65),
    cup=(0.30, 0.30, 0.32),
)
LINKS = ("door", "arm0", "arm1")
FR_WOOD = 0.6
FR_DOOR = 0.8  # lacquered MDF against the rubber finger pads
FR_STEEL = 0.3


class HingeBench(Box):
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
    """Authored (reset) state: door and arm poses, carcass / groove block placement."""
    spec = L.setup_spec(setup)
    dp, dr = L.door_pose_start(spec)
    arms = [L.arm_pose_from_door(dp, dr, spec, i) for i in range(2)]
    gp, gr = L.groove_block_pose(spec)
    xf, yl = L.cabinet_origin(spec)
    return dict(spec=spec, door=(dp, dr), arm0=arms[0], arm1=arms[1], groove=(gp, gr),
                carcass=np.array([xf - L.X_FRONT0, yl - L.Y_LEFT0, 0.0]))


_SPEC: dict = {}


def _spawn_bench(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    ox, oy, oz, ow = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(ow, ox, oy, oz))
    looks, mats = {}, {}

    def look(color):
        if color not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(COLORS[color]), roughness=0.5)
            vis.func(path, vis)
            looks[color] = path
        return looks[color]

    def material(friction):
        if friction not in mats:
            path = f"{prim_path}/Looks/phys{len(mats)}"
            pm = sim_utils.RigidBodyMaterialCfg(static_friction=friction, dynamic_friction=friction * 0.9)
            pm.func(path, pm)
            mats[friction] = path
        return mats[friction]

    def xform(path, pos, rot):
        x = UsdGeom.Xform.Define(stage, path)
        x.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float, pos)))
        q = _quat(rot)
        x.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(q[3]), *map(float, q[:3])))
        return x

    def body(path, mass, kinematic=False, inertia=None, com=None, link=False):
        prim = stage.GetPrimAtPath(path)
        rb = UsdPhysics.RigidBodyAPI.Apply(prim)
        if kinematic:
            rb.CreateKinematicEnabledAttr(True)
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(float(mass))
        if inertia is not None:
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, inertia)))
        if com is not None:
            m.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
        px = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        px.CreateMaxDepenetrationVelocityAttr(0.2)
        px.CreateLinearDampingAttr(0.0)
        px.CreateAngularDampingAttr(0.05)
        if not link:
            px.CreateSolverPositionIterationCountAttr(32)
            px.CreateSolverVelocityIterationCountAttr(4)
            px.CreateSleepThresholdAttr(0.0)
            px.CreateStabilizationThresholdAttr(0.0)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
        return prim

    def cbox(path, center, size, color, friction, rot=None, collide=True):
        """A box as a convex hull mesh (collision) with its look."""
        from scipy.spatial import ConvexHull

        v = L.box_pts(center, size, rot)
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
            bind_physics_material(str(prim.GetPath()), material(friction), stage=stage)
        bind_visual_material(str(prim.GetPath()), look(color), stage=stage)
        return m

    def vcyl(path, center, axis, radius, height, color):
        cyl = UsdGeom.Cylinder.Define(stage, path)
        cyl.CreateRadiusAttr(float(radius))
        cyl.CreateHeightAttr(float(height))
        cyl.CreateAxisAttr(axis)
        cyl.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
        bind_visual_material(str(cyl.GetPrim().GetPath()), look(color), stage=stage)

    a0 = authored(SETUP_BASE)
    spec0 = a0["spec"]
    # ------------------------------------------------------------------ carcass + plates (kinematic, station frame
    # at the nominal position; a reset moves the body by the setup's carcass offset)
    nominal = dict(spec0, cdx=0.0, cdy=0.0)
    p = f"{prim_path}/carcass"
    xform(p, a0["carcass"], np.eye(3))
    body(p, 12.0, kinematic=True)
    for nm, c, s, col in L.carcass_pieces(nominal):
        cbox(f"{p}/{nm}", c, s, col, FR_WOOD)
    for i in range(2):
        po = L.plate_origin(nominal, i)
        for nm, c, s in L.plate_pieces_a():  # visual only: the arm-plate contact is the computed plate model
            cbox(f"{p}/plate{i}_{nm}", po + np.asarray(c), s, "plate", FR_STEEL, collide=False)
    for nm, c, s, col in L.blocks_pieces(nominal):
        cbox(f"{p}/{nm}", c, s, col, FR_WOOD)
    # ------------------------------------------------------------------ groove block (kinematic)
    p = f"{prim_path}/groove"
    gp, gr = a0["groove"]
    xform(p, gp, gr)
    body(p, 1.0, kinematic=True)
    for nm, c, s in L.groove_pieces():
        cbox(f"{p}/{nm}", c, s, "pine", FR_WOOD)
    # ------------------------------------------------------------------ the door (floating articulation)
    art = UsdGeom.Xform.Define(stage, f"{prim_path}/door_asm")
    UsdPhysics.ArticulationRootAPI.Apply(art.GetPrim())
    pa = PhysxSchema.PhysxArticulationAPI.Apply(art.GetPrim())
    pa.CreateSolverPositionIterationCountAttr(32)
    pa.CreateSolverVelocityIterationCountAttr(4)
    pa.CreateEnabledSelfCollisionsAttr(False)
    pa.CreateSleepThresholdAttr(0.0)
    pa.CreateStabilizationThresholdAttr(0.0)
    paths = {n: f"{prim_path}/door_asm/{n}" for n in LINKS}
    dp, dr = a0["door"]
    xform(paths["door"], dp, dr)
    body(paths["door"], L.M_DOOR, com=(L.DOOR_W / 2, L.DOOR_T / 2, L.DOOR_H / 2), link=True,
         inertia=(L.M_DOOR * (L.DOOR_T ** 2 + L.DOOR_H ** 2) / 12, L.M_DOOR * (L.DOOR_W ** 2 + L.DOOR_H ** 2) / 12,
                  L.M_DOOR * (L.DOOR_W ** 2 + L.DOOR_T ** 2) / 12))
    for nm, c, s in L.door_pieces():
        cbox(f"{paths['door']}/{nm}", c, s, "door", FR_DOOR)
    for i, hc in enumerate(L.HINGE_C):
        # hinge cup (visual, in the door's inner face)
        vcyl(f"{paths['door']}/cup{i}", (L.CUP_A, L.DOOR_T + 0.0005, hc), "Y", L.CUP_R + 0.002, 0.002, "cup")
    for i in range(2):
        ap, ar = a0[f"arm{i}"]
        xform(paths[f"arm{i}"], ap, ar)
        body(paths[f"arm{i}"], L.M_ARM, inertia=L.ARM_INERTIA, com=(-0.02, -0.006, 0.0), link=True)
        for nm, c, s in L.arm_pieces_a():
            cbox(f"{paths[f'arm{i}']}/{nm}", c, s, "steel", FR_STEEL)
        # neck from the channel to the cup (visual): runs forward in A x to the cup, stepping toward the door
        k_a = L.knuckle_in_arm(spec0, i)
        cup_x, cup_y = k_a[0] - L.CUP_A, k_a[1] - L.DOOR_T  # cup centre on the door's inner face, in A
        x1 = L.ARM_X[0]
        cbox(f"{paths[f'arm{i}']}/neck", ((cup_x + x1) / 2, -0.006, 0.0), (abs(x1 - cup_x), 0.012, 0.015), "steel",
             FR_STEEL, collide=False)
        cbox(f"{paths[f'arm{i}']}/boss", (cup_x, (cup_y - 0.012) / 2, 0.0), (0.030, abs(cup_y + 0.012), 0.030),
             "steel", FR_STEEL, collide=False)
    q_i = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
    for i, hc in enumerate(L.HINGE_C):
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{prim_path}/door_asm/knuckle{i}")
        j.CreateAxisAttr("Z")
        j.CreateBody0Rel().SetTargets([Sdf.Path(paths["door"])])
        j.CreateBody1Rel().SetTargets([Sdf.Path(paths[f"arm{i}"])])
        j.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, float(hc)))
        j.CreateLocalRot0Attr(q_i)
        ka = L.knuckle_in_arm(spec0, i)
        j.CreateLocalPos1Attr(Gf.Vec3f(*map(float, ka)))
        _, ar_d = L.arm_in_door(spec0, i)
        q1 = _quat(ar_d.T)
        j.CreateLocalRot1Attr(Gf.Quatf(float(q1[3]), *map(float, q1[:3])))
        j.CreateLowerLimitAttr(float(-L.ALPHA_OPEN))
        j.CreateUpperLimitAttr(float(L.ALPHA_MAX - L.ALPHA_OPEN))
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(0.0)
        d.CreateDampingAttr(float(L.KNUCKLE_DAMP * DEG))
        d.CreateTargetPositionAttr(0.0)
        d.CreateTargetVelocityAttr(0.0)
        d.CreateMaxForceAttr(float(L.KNUCKLE_FRICTION_NM))
        PhysxSchema.PhysxJointAPI.Apply(j.GetPrim()).CreateArmatureAttr(0.0)
    # arms never collide with the carcass / plates (their plate contact is the computed model)
    for i in range(2):
        fp = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(paths[f"arm{i}"]))
        fp.CreateFilteredPairsRel().SetTargets([Sdf.Path(f"{prim_path}/carcass")])
    _SPEC["paths"] = paths
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


def finger_arms(environment):
    """Per arm: (two pad-tip points (env frame), squeeze force N, [joint7, joint8] finger forces N)."""
    from yam_arena.constants import GRIPPER, PAD_TIP_PAST_TCP, TCP_OFFSET

    base = environment.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        pose = robot.data.body_link_pose_w[0].detach().cpu().numpy().astype(float)[
            list(robot.body_names).index("gripper")]
        jp = robot.data.joint_pos[0].detach().cpu().numpy().astype(float)
        jt = robot.data.joint_pos_target[0].detach().cpu().numpy().astype(float)
        jn = list(robot.joint_names)
        ids = [jn.index(j) for j in ("joint7", "joint8")]
        half = max(abs(jp[i]) for i in ids)
        fingers = [min(GRIPPER["effort"], GRIPPER["stiffness"] * max(0.0, abs(jp[i]) - abs(jt[i]))) for i in ids]
        r = _rot(pose[3:7])
        tip = pose[:3] + r @ (np.array(TCP_OFFSET) + [0, 0, PAD_TIP_PAST_TCP]) - origin
        pts = [tip + r[:, 0] * s * (half + 0.0032) for s in (1.0, -1.0)]
        out.append((pts, float(max(fingers)), [float(f) for f in fingers]))
    return out


class Hinge:
    """One hinge arm's plate state machine (FREE -> HOOKED -> CLIPPED; HOOKED -> FREE when it rolls off)."""

    def __init__(self, i):
        self.i = i
        self.state = FREE
        self.prev_dz = None
        self.events = []
        self.peak_rear_push = 0.0
        self.peak_hook_n = 0.0

    def geometry(self, ap, ar, seat):
        """Lug offset from its seat (world), rear gap (m, + = off the plate toward the cabinet interior), tilt."""
        hook = ap + ar @ L.HOOK_A
        rear = ap + ar @ L.REAR_A
        d = hook - (seat + L.HOOK_A)
        g = float((seat + L.REAR_A)[1] - rear[1])
        tilt = math.degrees(math.acos(float(np.clip(ar[2, 2], -1.0, 1.0))))
        return hook, rear, d, g, tilt


class Runtime:
    """Runs the plate model and the hinge springs every physics step, scores every control step."""

    def __init__(self, environment):
        import omni.physics.tensors as tensors
        import torch
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdUtils

        self.base = environment.unwrapped
        self.env = environment
        self.torch = torch
        stage = get_current_stage()
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        self.simview = tensors.create_simulation_view("torch", stage_id=stage_id)
        self.simview.set_subspace_roots("/")
        root = "/World/envs/env_0/hinge_bench"
        self.root = root
        self.art = self.simview.create_articulation_view(f"{root}/door_asm")
        self.link_names = [n.rsplit("/", 1)[-1] for n in self.art.shared_metatype.link_names]
        self.dof_names = list(self.art.shared_metatype.dof_names)
        self.li = {n: self.link_names.index(n) for n in LINKS}
        self.di = [self.dof_names.index(f"knuckle{i}") for i in range(2)]
        self.kview = self.simview.create_rigid_body_view([f"{root}/carcass", f"{root}/groove"])
        self.origin = self.base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        dev = self.art.get_dof_positions().device
        self.dev = dev
        self.idx0 = torch.tensor([0], dtype=torch.int32, device=dev)
        self.ndof = len(self.dof_names)
        self.nl = len(self.link_names)
        print("hingekit: links", self.link_names, "dofs", self.dof_names, flush=True)
        self.setup = SETUP_BASE
        self.spec = L.setup_spec(self.setup)
        self.trial_active = False
        self.restored = False
        self.soft = [False, False]
        self._seats()
        self._init_state()

    def _seats(self):
        self.seats = [L.plate_origin(self.spec, i) for i in range(2)]

    def _init_state(self):
        self.scorer = Scorer()
        self.steps = 0
        self.t = 0.0
        self.nonfinite = 0
        self.hinges = [Hinge(0), Hinge(1)]
        self.grip_peak = [0.0, 0.0]
        self.hook_s = 0.0
        self.alpha_min = 999.0
        STATE.update(score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, setup=self.setup)

    def begin_trial(self, trial):
        self._init_state()
        self.trial_active = True
        STATE.update(trial=trial, setup=self.setup, start=self.spec)

    # ------------------------------------------------------------------ reading
    def dofs(self):
        q = self.art.get_dof_positions().detach().cpu().numpy().astype(float)[0]
        return [float(q[i]) for i in self.di]

    def dof_vels(self):
        v = self.art.get_dof_velocities().detach().cpu().numpy().astype(float)[0]
        return [float(v[i]) for i in self.di]

    def read(self):
        tf = self.art.get_link_transforms().detach().cpu().numpy().astype(float)[0]
        if not np.all(np.isfinite(tf)):
            self.nonfinite += 1
            tf = np.nan_to_num(tf)
        return {n: (tf[i, :3] - self.origin, _rot(tf[i, 3:7])) for n, i in self.li.items()}

    def link_vels(self):
        return self.art.get_link_velocities().detach().cpu().numpy().astype(float)[0]

    def speeds(self):
        v = self.link_vels()
        i = self.li["door"]
        return float(np.linalg.norm(v[i, :3])), math.degrees(float(np.linalg.norm(v[i, 3:6])))

    # ------------------------------------------------------------------ physics step
    def physics(self, dt):
        t0 = time.perf_counter()
        self.t += dt
        torch = self.torch
        poses = self.read()
        vel = self.link_vels()
        fd = torch.zeros((1, self.nl, 3), dtype=torch.float32, device=self.dev)
        td = torch.zeros((1, self.nl, 3), dtype=torch.float32, device=self.dev)
        pd = torch.zeros((1, self.nl, 3), dtype=torch.float32, device=self.dev)
        any_force = False
        if PROBE and self.trial_active:
            import probe_hinge

            for n, (f, tq, at) in probe_hinge.step(self, poses, vel, dt).items():
                li = self.li[n]
                fd[0, li] += torch.tensor(np.asarray(f, float), dtype=torch.float32)
                td[0, li] += torch.tensor(np.asarray(tq, float) + np.cross(np.asarray(at) - poses[n][0], f) * 0,
                                          dtype=torch.float32)
                pd[0, li] = torch.tensor(np.asarray(at, float) + self.origin, dtype=torch.float32)
                any_force = True
        for h in self.hinges:
            n = f"arm{h.i}"
            li = self.li[n]
            ap, ar = poses[n]
            v_lin, w = vel[li, :3], vel[li, 3:6]
            com = ap + ar @ np.array([-0.02, -0.006, 0.0])
            f_tot, t_tot = self.hinge_step(h, ap, ar, v_lin, w, com)
            if f_tot is not None:
                if any_force and float(fd[0, li].abs().sum()) > 0:  # probe force on this link too: move it to com
                    at = pd[0, li].cpu().numpy().astype(float) - self.origin
                    f0 = fd[0, li].cpu().numpy().astype(float)
                    t_tot = t_tot + np.cross(at - com, f0) + td[0, li].cpu().numpy().astype(float)
                    f_tot = f_tot + f0
                fd[0, li] = torch.tensor(f_tot, dtype=torch.float32)
                td[0, li] = torch.tensor(t_tot, dtype=torch.float32)
                pd[0, li] = torch.tensor(com + self.origin, dtype=torch.float32)
                any_force = True
        if any_force:
            self.art.apply_forces_and_torques_at_position(fd, td, pd, self.idx0, True)
        # hinge springs: self-closing and soft-close on the knuckle joints
        th = self.dofs()
        thv = self.dof_vels()
        eff = torch.zeros((1, self.ndof), dtype=torch.float32, device=self.dev)
        for k, (q, qd) in enumerate(zip(th, thv)):
            a = L.door_alpha(q)
            self.alpha_min = min(self.alpha_min, a)
            tau = 0.0
            if a < L.CLOSE_FROM_DEG:
                tau -= L.CLOSE_NM * min(1.0, (L.CLOSE_FROM_DEG - a) / 3.0)
            eff[0, self.di[k]] = float(tau)
        self.art.set_dof_actuation_forces(eff, self.idx0)
        # soft-close damping: the knuckle drive's (implicit) damping and cap rise below SOFT_FROM_DEG
        soft = [L.door_alpha(q) < L.SOFT_FROM_DEG for q in th]
        if soft != self.soft:
            self.soft = soft
            c = self.art.get_dof_dampings().clone()
            m = self.art.get_dof_max_forces().clone()
            for k, on in enumerate(soft):
                c[0, self.di[k]] = L.SOFT_DAMP if on else L.KNUCKLE_DAMP
                m[0, self.di[k]] = L.SOFT_CAP_NM if on else L.KNUCKLE_FRICTION_NM
            cpu = torch.tensor([0], dtype=torch.int32)
            self.art.set_dof_dampings(c.cpu(), cpu)
            self.art.set_dof_max_forces(m.cpu(), cpu)
        self.hook_s += time.perf_counter() - t0

    def hinge_step(self, h, ap, ar, v_lin, w, com):
        """Plate model for one arm: state transitions and the force/torque (about the arm's COM) to apply."""
        seat = self.seats[h.i]
        hook, rear, d, g, tilt = h.geometry(ap, ar, seat)

        def pv(p):  # velocity of a point on the arm
            return v_lin + np.cross(w, p - com)

        forces = []
        if h.state == FREE:
            dz = float(d[2])
            if (h.prev_dz is not None and h.prev_dz > L.HOOK_DROP[0] and -L.HOOK_DROP[1] <= dz <= L.HOOK_DROP[0]
                    and abs(d[0]) <= L.HOOK_TOL_X and abs(d[1]) <= L.HOOK_TOL_Y
                    and L.HOOK_REAR_GAP[0] <= g <= L.HOOK_REAR_GAP[1] and tilt <= TILT_HOOK_DEG):
                h.state = HOOKED
                h.events.append(dict(ev="hooked", t=round(self.t, 3), d_mm=np.round(d * 1e3, 2).tolist(),
                                     rear_gap_mm=round(g * 1e3, 2)))
            h.prev_dz = dz
        elif h.state == HOOKED:
            off = float(np.linalg.norm(d))
            if off > L.HOOK_BREAK or g > L.HOOK_REAR_MAX or tilt > L.TILT_MAX_DEG:
                h.state = FREE
                h.prev_dz = float(d[2])
                h.events.append(dict(ev="unhooked", t=round(self.t, 3), off_mm=round(off * 1e3, 2),
                                     rear_gap_mm=round(g * 1e3, 2), tilt_deg=round(tilt, 2)))
            elif g <= L.G_LATCHED:
                h.state = CLIPPED
                h.events.append(dict(ev="clipped", t=round(self.t, 3), rear_gap_mm=round(g * 1e3, 3),
                                     off_mm=round(off * 1e3, 2), peak_rear_push_n=round(h.peak_rear_push, 1)))
        if h.state == HOOKED:
            f_h = -L.K_PLATE * d - L.C_PLATE * pv(hook)
            forces.append((f_h, hook))
            h.peak_hook_n = max(h.peak_hook_n, float(np.linalg.norm(f_h)))
            ny = np.array([0.0, 1.0, 0.0])  # toward the panel
            if g < 0.0:
                fr = (L.K_PLATE * g - L.C_PLATE * max(0.0, float(pv(rear)[1]))) * ny  # stop (pushes -y)
                forces.append((fr, rear))
                h.peak_rear_push = max(h.peak_rear_push, float(-fr[1]))
            elif g < L.G_CLICK:
                f = L.F_CLICK * min(1.0, (L.G_CLICK - g) / 0.0003)
                forces.append((-f * ny, rear))
                h.peak_rear_push = max(h.peak_rear_push, f)
        elif h.state == CLIPPED:
            for pa in (L.HOOK_A, L.REAR_A, L.TOP_A):
                p = ap + ar @ pa
                f = -L.K_PLATE * (p - (seat + pa)) - L.C_PLATE * pv(p)
                forces.append((f, p))
        if not forces:
            return None, None
        f_tot = np.zeros(3)
        t_tot = np.zeros(3)
        for f, p in forces:
            f_tot += f
            t_tot += np.cross(p - com, f)
        return f_tot, t_tot

    # ------------------------------------------------------------------ reset
    def restore_authored(self, setup):
        """Environment reset only (never during a trial): carcass and groove block to the setup, the door standing in
        the groove, arms at 90 deg, at rest."""
        torch = self.torch
        self.setup = setup
        self.trial_active = False
        a = authored(setup)
        self.spec = a["spec"]
        if PROBE:
            import probe_hinge

            a = probe_hinge.start(a)
        self._seats()
        ktf = self.kview.get_transforms().clone()
        ktf[0, :3] = torch.tensor(a["carcass"] + self.origin, dtype=ktf.dtype)
        ktf[0, 3:7] = torch.tensor([0.0, 0.0, 0.0, 1.0], dtype=ktf.dtype)
        gp, gr = a["groove"]
        ktf[1, :3] = torch.tensor(gp + self.origin, dtype=ktf.dtype)
        ktf[1, 3:7] = torch.tensor(_quat(gr), dtype=ktf.dtype)
        self.kview.set_transforms(ktf, torch.arange(2, dtype=torch.int32, device=ktf.device))
        rootname = self.link_names[0]
        rp, rr = a[rootname]
        rt = self.art.get_root_transforms().clone()
        rt[0, :3] = torch.tensor(rp + self.origin, dtype=rt.dtype)
        rt[0, 3:7] = torch.tensor(_quat(rr), dtype=rt.dtype)
        self.art.set_root_transforms(rt, self.idx0)
        self.art.set_root_velocities(torch.zeros_like(self.art.get_root_velocities()), self.idx0)
        dp = torch.zeros_like(self.art.get_dof_positions())
        self.art.set_dof_positions(dp, self.idx0)
        self.art.set_dof_position_targets(dp, self.idx0)
        self.art.set_dof_velocities(torch.zeros_like(self.art.get_dof_velocities()), self.idx0)
        self.restored = True
        self._init_state()

    # ------------------------------------------------------------------ scoring
    def control(self, environment):
        if not self.trial_active:
            return
        self.steps += 1
        poses = self.read()
        arms = finger_arms(environment)
        tips = [p for a in arms for p in a[0]]
        for i, a in enumerate(arms):
            self.grip_peak[i] = max(self.grip_peak[i], a[1])
        alphas = [L.door_alpha(q) for q in self.dofs()]
        m = measure(poses, self.speeds(), alphas, [h.state for h in self.hinges], tips, self.spec)
        level = self.scorer.step(m)
        STATE.update(
            score=level,
            level=level,
            steps=self.steps,
            nonfinite=self.nonfinite,
            detail=dict(
                self.scorer.last,
                hinges=[h.state for h in self.hinges],
                events=[e for h in self.hinges for e in h.events][-12:],
                peak_rear_push_n=[round(h.peak_rear_push, 1) for h in self.hinges],
                peak_hook_n=[round(h.peak_hook_n, 1) for h in self.hinges],
                grip_n=[round(a[1], 2) for a in arms],
                grip_peak_n=[round(g, 2) for g in self.grip_peak],
                alpha_min_deg=round(self.alpha_min, 2),
                setup=self.setup,
                hook_wall_s=round(self.hook_s, 2),
            ),
        )


TILT_HOOK_DEG = 6.0
_RT: dict = {"rt": None, "sim": None, "trials": 0}


def runtime():
    return _RT["rt"]


def hinge_state():
    """Current link poses {'door', 'arm0', 'arm1'}, knuckle angles 'alpha' (deg), hinge states, seats and 'spec',
    for the scripted demo (read-only)."""
    rt = _RT["rt"]
    if rt is None:
        return None
    out = rt.read()
    out["alpha"] = [L.door_alpha(q) for q in rt.dofs()]
    out["hinges"] = [h.state for h in rt.hinges]
    out["geom"] = [h.geometry(*out[f"arm{h.i}"], rt.seats[h.i]) for h in rt.hinges]
    out["seats"] = list(rt.seats)
    out["spec"] = rt.spec
    return out


def _patch_scene(scene) -> None:
    if getattr(scene, "_cdh_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, HingeBench):
            return _arena_object(spec)
        return orig_task_object(spec)

    def object_states(environment, names):
        from yam_arena.task_file import ObjectState

        plain = [n for n in names if n != "hinge_bench"]
        out = orig_states(environment, plain) if plain else {}
        if "hinge_bench" in names:
            rt = _RT["rt"]
            p, r = rt.read()["door"] if rt else authored(SETUP_BASE)["door"]
            z = np.zeros(3)
            out["hinge_bench"] = ObjectState(pos=np.asarray(p), quat_xyzw=_quat(r), lin_vel=z, ang_vel=z,
                                             low=p - 0.3, high=p + 0.3)
        return {n: out[n] for n in names}

    orig_build = scene.build_task_environment

    def build(embodiment, task, control_hz=None):
        environment = orig_build(embodiment, task, control_hz)
        if any(isinstance(o, HingeBench) for o in task.objects):
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
    scene._cdh_patched = True


def _has_bench(emb):
    return any(isinstance(o, HingeBench) for o in (emb.task.objects if emb.task is not None else ()))


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_cdh_patched", False):
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
            rt.restore_authored(setup)
        obs = base_reset(self, *a, **k)
        rt = _RT["rt"]
        if rt is not None:
            if rt.setup != setup or not rt.restored:  # first reset of a fresh env
                rt.restore_authored(setup)
                obs = base_reset(self, *a, **k)
            rt.begin_trial(_RT["trials"])
            _RT["trials"] += 1
        return obs

    cls._ensure_sim, cls._task_step, cls.reset = _ensure_sim, _task_step, reset
    cls._cdh_patched = True


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
    if not any(type(f).__name__ == "_Finder" and getattr(f, "targets", None) and "yam_arena.scene" in f.targets
               for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
