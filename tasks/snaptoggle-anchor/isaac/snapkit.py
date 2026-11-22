"""snapkit: the drywall wall mock-up, the strap toggle anchor and its staging block for yam_arena (Isaac Lab-Arena /
PhysX), plus the runtime hooks.

Authored as one custom-spawned assembly (yam_arena task-file primitives are single boxes/spheres):
* wall (kinematic, so PhysX contact views report forces on it): 1/2 in gypsum board with the 1/2 in hole (48 convex
  sector pieces around the hole), 2x4 studs and plates behind it, pencil cross (visual only). Moved as a whole on an
  environment reset (setup: lateral position of the hole, stand-off of the board).
* staging block (kinematic) the anchor lies on at the start.
* anchor: a floating PhysX articulation, root = the strap stub (the straps from the channel pivot to the break line),
  with the steel channel on a revolute pivot (free, limits +-100 deg) and the cap on a prismatic joint along the
  straps. The cap's ratchet teeth: an elasto-plastic anchor on a stiff prismatic articulation drive whose force is
  capped at RATCHET_FWD_N when the cap moves toward the channel and at RATCHET_BACK_N when it is pushed back (the
  teeth hold). The rest of the straps with the pull tab ("tail") is a free rigid body joined to the stub at the break
  line by a PhysX fixed joint with a break torque (the straps snap at the cap face when bent) and a break force.
Geometry: ``../src/layout.py``; scorer: ``../src/scoring.py`` (pure functions).

Runtime hooks (installed on import; nothing in yam_arena is edited): after every physics step the runtime updates the
ratchet anchor and reads the contact forces (robot on the board, channel on the board); after every control step the
scorer reads the poses. Nothing here writes a body pose or velocity during a trial; ``Runtime.restore_authored`` puts
the anchor back on the block (and moves the wall / block to the setup) only on an environment reset.
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
from scoring import bend_moment, cap_at_break_line, measure, Monitor, Scorer  # noqa: E402
from yam_arena.task_file import Box  # noqa: E402

PHYSICS_DT = float(os.environ.get("SNAP_PHYSICS_DT", str(1 / 480)))
CONTACT_OFFSET = float(os.environ.get("SNAP_CONTACT_OFFSET", "0.0003"))
SETUP_BASE = int(os.environ.get("SNAP_SETUP", "0"))
FORCE_NAMED = "SNAP_SETUP" in os.environ
DEBUG = bool(os.environ.get("SNAP_DEBUG"))

GYPSUM, PENCIL, WOOD, STEEL, STRAP, CAPC, BLOCKC = (
    (0.93, 0.92, 0.88),
    (0.25, 0.25, 0.27),
    (0.80, 0.66, 0.45),
    (0.70, 0.72, 0.74),
    (0.15, 0.15, 0.16),
    (0.12, 0.12, 0.13),
    (0.62, 0.48, 0.30),
)
KINEMATIC = ("wall", "block")
LINKS = ("stub", "channel", "cap")
FREE = ("tail",)


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


def wall_pose(setup):
    xf, hc = L.wall(setup)
    return np.array([xf, hc[1], 0.0]), np.eye(3)


def anchor_poses(pivot, s_rot, phi_deg, q):
    """World poses of the anchor bodies {name: (pos, rot)} from the pivot pose, channel angle and cap position."""
    ch = s_rot @ L.channel_rot_in_s(phi_deg)
    return {
        "stub": (pivot, s_rot),
        "channel": (pivot.copy(), ch),
        "cap": (pivot + s_rot @ np.array([q, 0.0, 0.0]), s_rot),
        "tail": (pivot + s_rot @ np.array([L.X_BRK, 0.0, 0.0]), s_rot),
    }


def authored(setup):
    pivot, s_rot, phi, q = L.anchor_start(setup)
    out = anchor_poses(pivot, s_rot, phi, q)
    out["wall"] = wall_pose(setup)
    out["block"] = (L.block_pos(setup), np.eye(3))
    out["phi"] = phi
    out["q"] = q
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

    def look(color):
        if color not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            metal = color == STEEL
            vis = sim_utils.PreviewSurfaceCfg(
                diffuse_color=linear_rgb(color), roughness=0.35 if metal else 0.7, metallic=0.7 if metal else 0.0
            )
            vis.func(path, vis)
            looks[color] = path
        return looks[color]

    def material(friction, combine="average"):
        key = (friction, combine)
        if key not in mats:
            path = f"{prim_path}/Looks/phys{len(mats)}"
            pm = sim_utils.RigidBodyMaterialCfg(
                static_friction=friction, dynamic_friction=friction, friction_combine_mode=combine
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

    def rigid(path, mass, kinematic=False, inertia=None, com=None, iters=32):
        prim = stage.GetPrimAtPath(path)
        rb = UsdPhysics.RigidBodyAPI.Apply(prim)
        if kinematic:
            rb.CreateKinematicEnabledAttr(True)
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(float(mass))
        if inertia is not None:
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, inertia)))
            m.CreatePrincipalAxesAttr(Gf.Quatf(1, 0, 0, 0))
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
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
        return prim

    count = {"n": 0}

    def hull(path, v, color, friction, collision=True, torsion=0.0, combine="average"):
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
        m.CreateExtentAttr([Gf.Vec3f(*map(float, v.min(axis=0))), Gf.Vec3f(*map(float, v.max(axis=0)))])
        prim = m.GetPrim()
        if collision:
            UsdPhysics.CollisionAPI.Apply(prim)
            cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            cp.CreateContactOffsetAttr(CONTACT_OFFSET)
            cp.CreateRestOffsetAttr(0.0)
            if torsion:
                # contact-patch (torsional) friction: a gripper pad pinching a flat face resists twisting over its patch
                cp.CreateTorsionalPatchRadiusAttr(float(torsion))
                cp.CreateMinTorsionalPatchRadiusAttr(float(torsion))
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
            hc = PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim)
            hc.CreateHullVertexLimitAttr(64)
            hc.CreateMinThicknessAttr(0.0001)
            bind_physics_material(str(prim.GetPath()), material(friction, combine), stage=stage)
            count["n"] += 1
        bind_visual_material(str(prim.GetPath()), look(color), stage=stage)
        return m

    def boxv(path, center, size, color, friction=L.WOOD_FRICTION, rot=None, collision=True, torsion=0.0):
        return hull(path, L.box_vertices(center, size, rot), color, friction, collision, torsion)

    # ---------------------------------------------------------------- wall (kinematic; local frame: board face x = 0,
    # hole at y = 0, z = HOLE_Z0 absolute)
    wp, wr = wall_pose(SETUP_BASE)
    xform(f"{prim_path}/wall", wp, wr)
    rigid(f"{prim_path}/wall", 20.0, kinematic=True)
    for i, v in enumerate(L.board_pieces(0.0, 0.0, L.HOLE_Z0)):
        hull(f"{prim_path}/wall/board_{i:02d}", v, GYPSUM, L.GYPSUM_FRICTION)
    for i, v in enumerate(L.frame_pieces(0.0)):
        hull(f"{prim_path}/wall/frame_{i}", v, WOOD, L.WOOD_FRICTION)
    # pencil cross on the face (visual only, 0.2 mm proud)
    for i, (sy, sz) in enumerate(((0.050, 0.0007), (0.0007, 0.050))):
        for k, off in enumerate((-1, 1)):
            c = off * (L.HOLE_R + 0.002 + 0.0125)
            cy, cz = (c, 0.0) if i == 0 else (0.0, c)
            boxv(
                f"{prim_path}/wall/pencil_{i}{k}",
                (-0.0001, cy, L.HOLE_Z0 + cz),
                (0.0002, 0.025 if i == 0 else 0.0007, 0.0007 if i == 0 else 0.025),
                PENCIL,
                collision=False,
            )
    # ---------------------------------------------------------------- staging block (kinematic)
    xform(f"{prim_path}/block", L.block_pos(SETUP_BASE), np.eye(3))
    rigid(f"{prim_path}/block", 1.0, kinematic=True)
    for i, v in enumerate(L.block_pieces()):
        hull(f"{prim_path}/block/part_{i}", v, BLOCKC, L.BLOCK_FRICTION)
    # ---------------------------------------------------------------- anchor articulation
    a = authored(SETUP_BASE)
    paths = {n: f"{prim_path}/{n}" for n in LINKS + FREE + KINEMATIC}
    sp, sr = a["stub"]
    xform(paths["stub"], sp, sr)
    stub = rigid(paths["stub"], L.STUB_MASS, inertia=(2e-8, 2e-8, 2e-8), com=((L.STUB_X0 + L.X_BRK) / 2, 0, 0), iters=64)
    UsdPhysics.ArticulationRootAPI.Apply(stub)
    ar = PhysxSchema.PhysxArticulationAPI.Apply(stub)
    ar.CreateEnabledSelfCollisionsAttr(False)
    ar.CreateSolverPositionIterationCountAttr(64)
    ar.CreateSolverVelocityIterationCountAttr(8)
    ar.CreateSleepThresholdAttr(0.0)
    ar.CreateStabilizationThresholdAttr(0.0)
    for k, sgn in enumerate((1, -1)):
        ln = L.X_BRK - L.STUB_X0
        boxv(
            f"{paths['stub']}/strap_{k}",
            (L.STUB_X0 + ln / 2, sgn * L.STRAP_Y, 0.0),
            (ln, L.STRAP_T, L.STRAP_H),
            STRAP,
            L.STRAP_FRICTION,
        )
    # channel: U section (web at +z, flanges at +-y); one box collider (the U is visual)
    cp_, cr_ = a["channel"]
    xform(paths["channel"], cp_, cr_)
    rigid(paths["channel"], L.CH_MASS, inertia=L.channel_inertia(), com=(L.CH_COM_X, 0, 0), iters=64)
    hull(f"{paths['channel']}/body", L.channel_vertices(), STEEL, L.STEEL_FRICTION, torsion=L.PAD_PATCH_R)
    pivot_j = UsdPhysics.RevoluteJoint.Define(stage, f"{paths['channel']}/pivot")
    pivot_j.CreateBody0Rel().SetTargets([Sdf.Path(paths["stub"])])
    pivot_j.CreateBody1Rel().SetTargets([Sdf.Path(paths["channel"])])
    pivot_j.CreateAxisAttr("Y")
    pivot_j.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
    pivot_j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    pivot_j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    q1 = _quat(L.ry(90.0))
    pivot_j.CreateLocalRot1Attr(Gf.Quatf(float(q1[3]), *map(float, q1[:3])))
    pivot_j.CreateLowerLimitAttr(-L.PHI_LIMIT)
    pivot_j.CreateUpperLimitAttr(L.PHI_LIMIT)
    PhysxSchema.PhysxJointAPI.Apply(pivot_j.GetPrim()).CreateArmatureAttr(1e-6)
    pp = pivot_j.GetPrim()
    pp.AddAppliedSchema("PhysxJointAxisAPI:angular")
    from pxr import Sdf as _Sdf

    for attr, value in (("staticFrictionEffort", 2e-5), ("dynamicFrictionEffort", 2e-5), ("viscousFrictionCoefficient", 2e-6)):
        pp.CreateAttribute(f"physxJointAxis:angular:{attr}", _Sdf.ValueTypeNames.Float).Set(float(value))
    # cap: flange (outer face at the cap origin) + collar toward the channel; ratchet drive on the slide
    cpp, crr = a["cap"]
    xform(paths["cap"], cpp, crr)
    rigid(paths["cap"], L.CAP_MASS, inertia=(5e-8, 4e-8, 4e-8), iters=64)
    hull(
        f"{paths['cap']}/flange",
        L.cylinder_x(L.CAP_FLANGE_R, -L.CAP_FLANGE_T, 0.0, n=32),
        CAPC,
        L.CAP_FRICTION,
        combine="min",
    )
    hull(
        f"{paths['cap']}/collar",
        L.cylinder_x(L.CAP_COLLAR_R, -L.CAP_FLANGE_T - L.CAP_COLLAR_L, -L.CAP_FLANGE_T, n=32, nose=L.CAP_NOSE),
        CAPC,
        L.CAP_FRICTION,
        combine="min",
    )
    # the cap's play on the straps (slot clearance + strap flex): two short lateral slides (+-CAP_PLAY, centring
    # spring) between the stub and the cap's ratchet slide; their carriers are tiny massive links without colliders
    def slide_joint(name, b0, b1, axis, lo, hi):
        j = UsdPhysics.PrismaticJoint.Define(stage, f"{b1}/{name}")
        j.CreateBody0Rel().SetTargets([Sdf.Path(b0)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(b1)])
        j.CreateAxisAttr(axis)
        j.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLowerLimitAttr(float(lo))
        j.CreateUpperLimitAttr(float(hi))
        PhysxSchema.PhysxJointAPI.Apply(j.GetPrim()).CreateArmatureAttr(L.JOINT_ARMATURE)
        return j

    prev = paths["stub"]
    for name, axis in (("cap_y", "Y"), ("cap_z", "Z")):
        cpath = f"{prim_path}/{name}"
        xform(cpath, sp, sr)
        rigid(cpath, 0.0005, inertia=(1e-9, 1e-9, 1e-9), iters=64)
        j = slide_joint(name, prev, cpath, axis, -L.CAP_PLAY, L.CAP_PLAY)
        dj = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
        dj.CreateTypeAttr("force")
        dj.CreateStiffnessAttr(float(L.CAP_PLAY_K))
        dj.CreateDampingAttr(0.5)
        dj.CreateMaxForceAttr(20.0)
        dj.CreateTargetPositionAttr(0.0)
        prev = cpath
    slide = slide_joint("slide", prev, paths["cap"], "X", L.Q_MIN, L.Q_MAX)
    d = UsdPhysics.DriveAPI.Apply(slide.GetPrim(), "linear")
    d.CreateTypeAttr("force")
    d.CreateStiffnessAttr(float(L.RATCHET_K))
    d.CreateDampingAttr(2.0)
    d.CreateMaxForceAttr(float(L.RATCHET_BACK_N))
    d.CreateTargetPositionAttr(float(a["q"]))
    PhysxSchema.PhysxJointAPI.Apply(slide.GetPrim()).CreateArmatureAttr(L.JOINT_ARMATURE)
    # tail: free body (straps beyond the break line + pull tab), fixed to the stub by a breakable joint
    tp, tr = a["tail"]
    xform(paths["tail"], tp, tr)
    tail_len = L.STRAP_LEN - L.X_BRK
    rigid(paths["tail"], L.TAIL_MASS, inertia=L.box_inertia(L.TAIL_MASS, (tail_len, 0.008, 0.012)),
          com=(tail_len * 0.6, 0, 0), iters=64)
    sl = tail_len - L.TAB[0]
    for k, sgn in enumerate((1, -1)):
        boxv(f"{paths['tail']}/strap_{k}", (sl / 2, sgn * L.STRAP_Y, 0.0), (sl, L.STRAP_T, L.STRAP_H), STRAP,
             L.STRAP_FRICTION)
    boxv(f"{paths['tail']}/tab", (sl + L.TAB[0] / 2, 0.0, 0.0), L.TAB, STRAP, L.STRAP_FRICTION, torsion=L.PAD_PATCH_R)
    brk = UsdPhysics.FixedJoint.Define(stage, f"{paths['tail']}/break")
    brk.CreateBody0Rel().SetTargets([Sdf.Path(paths["stub"])])
    brk.CreateBody1Rel().SetTargets([Sdf.Path(paths["tail"])])
    brk.CreateLocalPos0Attr(Gf.Vec3f(float(L.X_BRK), 0, 0))
    brk.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    brk.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    brk.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    brk.CreateExcludeFromArticulationAttr(True)
    brk.CreateJointEnabledAttr(True)
    fp = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(paths["tail"]))
    fp.CreateFilteredPairsRel().SetTargets([Sdf.Path(paths[n]) for n in LINKS])
    _SPEC["paths"] = paths
    _SPEC["break_joint"] = str(brk.GetPrim().GetPath())
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


class Ratchet:
    """Elasto-plastic anchor of the cap's slide drive: the cap moves toward the channel (q decreasing) against
    RATCHET_FWD_N; pushed back it holds up to RATCHET_BACK_N (then slips, as stripped teeth would)."""

    def __init__(self, q0):
        self.anchor = q0

    def update(self, q):
        lf = L.RATCHET_FWD_N / L.RATCHET_K
        lb = L.RATCHET_BACK_N / L.RATCHET_K
        if q < self.anchor - lf:
            self.anchor = q + lf
        elif q > self.anchor + lb:
            self.anchor = q - lb
        return self.anchor


class Runtime:
    def __init__(self, environment):
        import carb
        import omni.physics.tensors as tensors
        import torch
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics, UsdUtils

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        self.base = environment.unwrapped
        self.env = environment
        self.torch = torch
        stage = get_current_stage()
        self.stage = stage
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        self.simview = tensors.create_simulation_view("torch", stage_id=stage_id)
        self.simview.set_subspace_roots("/")
        root = "/World/envs/env_0/bench"
        self.root = root
        P = lambda n: f"{root}/{n}"  # noqa: E731
        self.names = ("stub", "channel", "cap", "tail", "wall", "block")
        self.view = self.simview.create_rigid_body_view([P(n) for n in self.names])
        self.idx = {n: i for i, n in enumerate(self.names)}
        self.art = self.simview.create_articulation_view(P("stub"))
        self.dof_names = list(self.art.shared_metatype.dof_names)
        self.idof = {n: self.dof_names.index(n) for n in self.dof_names}
        robot_links = []
        self.arm_links = []  # per arm: indices (into robot_links) of its two finger links and of its other links
        for arm in ("left_robot", "right_robot"):
            lp = self.base.scene[arm].root_view.link_paths
            lp = list(lp[0] if lp and isinstance(lp[0], (list, tuple)) else lp)
            n0 = len(robot_links)
            fingers = [n0 + j for j, p in enumerate(lp) if p.rstrip("/").split("/")[-1] in ("tip_left", "tip_right")]
            self.arm_links.append(dict(fingers=fingers, other=[n0 + j for j in range(len(lp)) if n0 + j not in fingers]))
            robot_links += lp
        self.robot_links = robot_links
        print("snapkit: finger links per arm", [len(x["fingers"]) for x in self.arm_links], flush=True)
        self.cv_wall = self.simview.create_rigid_contact_view(
            [P("wall")], filter_patterns=[robot_links + [P("channel"), P("cap"), P("stub"), P("tail")]],
            max_contact_data_count=4096,
        )
        self.cv_tail = self.simview.create_rigid_contact_view(
            [P("tail")], filter_patterns=[robot_links], max_contact_data_count=2048
        )
        self.n_robot = len(robot_links)
        self.break_joint = UsdPhysics.Joint(stage.GetPrimAtPath(f"{root}/tail/break"))
        self.origin = self.base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        self.dev = self.art.get_dof_positions().device
        self.ai = torch.tensor([0], dtype=torch.int32, device=self.dev)
        self.ai_cpu = torch.tensor([0], dtype=torch.int32)
        self.setup = SETUP_BASE
        self.trial_active = False
        self.fresh = True  # PhysX starts the articulation's joints at 0: the first reset writes the start state
        self._init_state()

    def _init_state(self):
        self.monitor = Monitor()
        self.scorer = Scorer()
        self.ratchet = Ratchet(self.dofs()["q"])
        self.steps = 0
        self.t = 0.0
        self.hook_s = 0.0
        self.nonfinite = 0
        self.broken = False
        self.break_info = None
        self.contacts = {}
        self.bend_s = 0.0
        self.bend_peak = 0.0
        self.hist = []
        self.tips = None
        STATE.update(score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, setup=self.setup)

    def begin_trial(self, trial):
        self._init_state()
        self.trial_active = True
        STATE.update(trial=trial, setup=self.setup, start=L.setup_spec(self.setup))

    # ------------------------------------------------------------------ reading
    def read(self):
        tf = self.view.get_transforms().detach().cpu().numpy().astype(float)
        return {n: (tf[i, :3] - self.origin, _rot(tf[i, 3:7])) for n, i in self.idx.items()}

    def velocities(self):
        v = self.view.get_velocities().detach().cpu().numpy().astype(float)
        return {n: (v[i, :3], v[i, 3:6]) for n, i in self.idx.items()}

    def dofs(self):
        q = self.art.get_dof_positions().detach().cpu().numpy()[0].astype(float)
        return dict(phi=math.degrees(q[self.idof["pivot"]]), q=float(q[self.idof["slide"]]))

    def tail_offset(self, poses=None):
        """Distance (m) of the tail's joint frame from the stub's break point (0 while joined)."""
        poses = poses or self.read()
        sp, sr = poses["stub"]
        tp, _ = poses["tail"]
        return float(np.linalg.norm(tp - (sp + sr @ np.array([L.X_BRK, 0.0, 0.0]))))

    # ------------------------------------------------------------------ physics step
    def physics(self, dt):
        t0 = time.perf_counter()
        self.t += dt
        torch = self.torch
        q = float(self.art.get_dof_positions()[0, self.idof["slide"]])
        anchor = self.ratchet.update(q)
        tg = self.art.get_dof_position_targets().clone()
        tg[0, self.idof["slide"]] = anchor
        self.art.set_dof_position_targets(tg, self.ai)
        if self.trial_active and not self.broken:
            self.check_snap(dt, q)
        if self.trial_active:
            fm = self.cv_wall.get_contact_force_matrix(dt=dt).detach().cpu().numpy()[0]
            robot_n = float(np.linalg.norm(fm[: self.n_robot].sum(axis=0))) if self.n_robot else 0.0
            parts = [float(np.linalg.norm(f)) for f in fm[self.n_robot:]]
            self.monitor.step(dt, robot_n)
            self.contacts = dict(robot_on_wall_n=round(robot_n, 3), channel_n=round(parts[0], 3),
                                 cap_n=round(parts[1], 3), stub_n=round(parts[2], 3), tail_n=round(parts[3], 3))
        self.hook_s += time.perf_counter() - t0

    def bend_moment(self):
        """Bending moment (N m) the robot puts on the strap tail about the break line (``scoring.bend_moment``)."""
        if self.tips is None:
            return 0.0
        fm = self.cv_tail.get_contact_force_matrix(dt=PHYSICS_DT).detach().cpu().numpy()[0]
        if not np.any(fm):
            return 0.0
        tf = self.view.get_transforms().detach().cpu().numpy().astype(float)
        i = self.idx["stub"]
        sp, sr = tf[i, :3] - self.origin, _rot(tf[i, 3:7])
        a = sr[:, 0]
        arms = []
        for k_arm, idx in enumerate(self.arm_links):
            fingers = [fm[j] for j in idx["fingers"]]
            other = fm[idx["other"]].sum(axis=0) if idx["other"] else np.zeros(3)
            arms.append((fingers, other, self.tips[2 * k_arm : 2 * k_arm + 2]))
        return bend_moment(arms, sp + a * L.X_BRK, a)

    def check_snap(self, dt, q):
        """The straps snap at the cap face when bent there: with the cap seated (``scoring.cap_at_break_line``), a
        bending moment of BREAK_MOMENT_NM or more held for BREAK_HOLD_S disables the stub-tail joint."""
        # gate on where the ratchet teeth hold the cap (its anchor), not on the elastic give of the drive: bending the
        # tail pushes the seated cap back a millimetre or so on the stiff ratchet spring
        at_line = cap_at_break_line(self.ratchet.anchor)
        m = self.bend_moment() if at_line else 0.0
        self.bend_peak = max(self.bend_peak, m)
        self.bend_s = self.bend_s + dt if m >= L.BREAK_MOMENT_NM else 0.0
        if self.bend_s >= L.BREAK_HOLD_S - 1e-9:
            self.break_joint.GetJointEnabledAttr().Set(False)
            self.broken = True
            self.break_info = dict(t=round(self.t, 3), step=self.steps, q_mm=round(q * 1e3, 2), moment_nm=round(m, 3))

    # ------------------------------------------------------------------ reset
    def restore_authored(self, setup):
        """Environment reset only (never during a trial): wall and block to the setup, the anchor folded on the block,
        the strap break joint re-made."""
        torch = self.torch
        self.setup = setup
        self.trial_active = False
        a = authored(setup)
        tf = self.view.get_transforms().clone()
        for n in ("tail", "wall", "block"):
            pos, rot = a[n]
            i = self.idx[n]
            tf[i, :3] = torch.tensor(np.asarray(pos, float) + self.origin, dtype=tf.dtype)
            tf[i, 3:7] = torch.tensor(_quat(rot), dtype=tf.dtype)
        ids = [self.idx[n] for n in ("tail", "wall", "block")]
        self.view.set_transforms(tf, torch.tensor(ids, dtype=torch.int32, device=tf.device))
        self.view.set_velocities(torch.zeros_like(self.view.get_velocities()),
                                 torch.tensor([self.idx["tail"]], dtype=torch.int32, device=tf.device))
        rt = self.art.get_root_transforms().clone()
        sp, sr = a["stub"]
        rt[0, :3] = torch.tensor(sp + self.origin, dtype=rt.dtype)
        rt[0, 3:7] = torch.tensor(_quat(sr), dtype=rt.dtype)
        self.art.set_root_transforms(rt, self.ai)
        self.art.set_root_velocities(torch.zeros_like(self.art.get_root_velocities()), self.ai)
        q = torch.zeros_like(self.art.get_dof_positions())  # cap play slides centred
        q[0, self.idof["pivot"]] = math.radians(a["phi"])
        q[0, self.idof["slide"]] = a["q"]
        self.art.set_dof_positions(q, self.ai)
        self.art.set_dof_velocities(torch.zeros_like(q), self.ai)
        self.art.set_dof_position_targets(q, self.ai)
        self.break_joint.GetJointEnabledAttr().Set(True)
        self._init_state()

    # ------------------------------------------------------------------ scoring
    def control(self, environment):
        if not self.trial_active:
            return
        self.steps += 1
        poses = self.read()
        if not all(np.all(np.isfinite(p)) for p, _ in poses.values()):
            self.nonfinite += 1
            STATE["nonfinite"] = self.nonfinite
            return
        vel = self.velocities()
        d = self.dofs()
        off = self.tail_offset(poses)
        xf, hc = L.wall(self.setup)
        wp = poses["wall"][0]
        xf = float(wp[0])
        hc = np.array([xf, float(wp[1]), L.HOLE_Z0])
        tips = finger_tips(environment)
        self.tips = tips
        m = measure(xf, hc, poses["channel"], poses["cap"], d["q"], poses["tail"], self.broken, tips)
        # "still" as a person judges it: mean speed of the channel and the cap over the last 1 s (displacement / time;
        # the instantaneous velocity of these gram-scale parts jitters at a few mm/s while they sit still)
        self.hist.append((poses["channel"][0].copy(), poses["cap"][0].copy()))
        if len(self.hist) > 21:
            del self.hist[0]
        if len(self.hist) == 21:
            speed = max(float(np.linalg.norm(self.hist[-1][i] - self.hist[0][i])) for i in (0, 1)) / 1.0
        else:
            speed = max(float(np.linalg.norm(vel[n][0])) for n in ("channel", "cap"))
        level = self.scorer.step(m, speed, self.monitor)
        STATE.update(
            score=level,
            level=level,
            steps=self.steps,
            nonfinite=self.nonfinite,
            detail=dict(
                self.scorer.last,
                phi_deg=round(d["phi"], 2),
                q_mm=round(d["q"] * 1e3, 3),
                cap_dyz_mm=np.round((poses["cap"][0] - hc)[1:] * 1e3, 2).tolist(),
                pivot_dyz_mm=np.round((poses["stub"][0] - hc)[1:] * 1e3, 2).tolist(),
                ratchet_mm=round(self.ratchet.anchor * 1e3, 3),
                broken=self.broken,
                break_info=self.break_info,
                bend_peak_nm=round(self.bend_peak, 3),
                tail_off_mm=round(off * 1e3, 2),
                contacts=self.contacts,
                setup=self.setup,
                hook_wall_s=round(self.hook_s, 2),
            ),
        )


_RT: dict = {"rt": None, "sim": None, "trials": 0}


def runtime():
    return _RT["rt"]


def bench_state():
    """Current poses for the scripted demo (read-only) plus the anchor joint values."""
    rt = _RT["rt"]
    if rt is None:
        return None
    out = rt.read()
    out.update(rt.dofs())
    out["broken"] = rt.broken
    out["contacts"] = dict(rt.contacts)
    return out


def _patch_scene(scene) -> None:
    if getattr(scene, "_snap_patched", False):
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
            p, r = rt.read()["channel"] if rt else authored(SETUP_BASE)["channel"]
            z = np.zeros(3)
            out["bench"] = ObjectState(pos=np.asarray(p), quat_xyzw=_quat(r), lin_vel=z, ang_vel=z, low=p - 0.1,
                                       high=p + 0.1)
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
    scene._snap_patched = True


def _has_bench(emb):
    return any(isinstance(o, Bench) for o in (emb.task.objects if emb.task is not None else ()))


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_snap_patched", False):
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
            setup = (SETUP_BASE + _RT["trials"]) % len(L.NAMED)
        if rt is not None:
            rt.restore_authored(setup)
        obs = base_reset(self, *a, **k)
        rt = _RT["rt"]
        if rt is not None:
            if rt.setup != setup or rt.fresh:  # first reset of a fresh env
                rt.fresh = False
                rt.restore_authored(setup)
                obs = base_reset(self, *a, **k)
            rt.begin_trial(_RT["trials"])
            _RT["trials"] += 1
        return obs

    cls._ensure_sim, cls._task_step, cls.reset = _ensure_sim, _task_step, reset
    cls._snap_patched = True


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
