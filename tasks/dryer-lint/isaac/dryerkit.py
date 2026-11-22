"""dryerkit: the dryer cabinet top with its lint-screen slot, the lint screen, the lint mat, the Start button and the
lint bin for yam_arena (Isaac Lab-Arena / PhysX), plus the runtime hooks.

Authored as one custom-spawned assembly (yam_arena task-file primitives are single boxes/spheres):
* dryer parts as kinematic rigid bodies (so PhysX contact views can report forces on them): the cabinet (top panel
  with the tapered slot and its two finger recesses, the screen housing below the slot, the rear console, the cabinet
  shell), the Start button cap on the front panel, and the bin;
* the lint screen as a free rigid body (tapered frame in four convex pieces plus its top grip bar with ears);
* the lint mat as a free rigid body held on the screen's front face by a breakable PhysX fixed joint (breaks at
  MAT_BREAK_N of pull or shear; contacts between mat and screen stay on, so a press goes through contact).
Geometry: ``../src/layout.py``; scorer: ``../src/scoring.py`` (pure functions).

Runtime hooks (installed on import; nothing in yam_arena is edited; same pattern as earlier task kits): after every
physics step the monitor reads the Start button's contact force from the robot and the mat joint state; after every
control step the scorer reads the screen and mat poses. Nothing here writes a body pose or velocity during a trial;
``Runtime.restore_authored`` places the screen (and the mat, re-creating its joint) only on an environment reset.
The scripted demo READS poses through ``bench_state()``.
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

PHYSICS_DT = float(os.environ.get("DRYER_PHYSICS_DT", str(1 / 480)))
CONTACT_OFFSET = float(os.environ.get("DRYER_CONTACT_OFFSET", ".0005"))
SETUP_BASE = int(os.environ.get("DRYER_SETUP", "0"))
FORCE_NAMED = "DRYER_SETUP" in os.environ
DEBUG = bool(os.environ.get("DRYER_DEBUG"))

WHITE, STEEL, GREY, DARK, LINT, BIN_COL, BTN, NYLON = (
    (0.93, 0.93, 0.91),
    (0.72, 0.73, 0.75),
    (0.55, 0.56, 0.58),
    (0.12, 0.12, 0.13),
    (0.55, 0.55, 0.60),
    (0.20, 0.42, 0.62),
    (0.85, 0.25, 0.15),
    (0.88, 0.88, 0.86),
)
PARK = np.array([-0.6, 0.9, 0.05])  # where the mat waits on a no-lint trial (on the floor behind the robot)
KINEMATIC = ("cabinet", "button", "bin")
BODIES = KINEMATIC + ("screen", "mat")


class Dryer(Box):
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
    """Reset pose of every body for a setup: {name: (pos, rot)} in the env frame."""
    d = L.dryer_origin(setup)
    sp, sr = L.screen_seated_pose(d)
    mp, mr = L.mat_pose_in_screen()
    lint = L.setup_spec(setup)["lint"]
    return {
        "cabinet": (d, np.eye(3)),
        "button": (L.button_center(d), np.eye(3)),
        "bin": (L.bin_center(d), np.eye(3)),
        "screen": (sp, sr),
        "mat": ((sp + sr @ mp, sr @ mr) if lint else (PARK.copy(), np.eye(3))),
        "lint": lint,
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
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    ox, oy, oz, ow = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(ow, ox, oy, oz))
    looks, mats = {}, {}

    def look(color):
        if color not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.5)
            vis.func(path, vis)
            looks[color] = path
        return looks[color]

    def material(friction, combine="average"):
        key = (friction, combine)
        if key not in mats:
            path = f"{prim_path}/Looks/phys{len(mats)}"
            pm = sim_utils.RigidBodyMaterialCfg(static_friction=friction, dynamic_friction=friction,
                                                friction_combine_mode=combine)
            pm.func(path, pm)
            mats[key] = path
        return mats[key]

    def xform(path, pos, rot):
        x = UsdGeom.Xform.Define(stage, path)
        x.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float, pos)))
        q = _quat(rot)
        x.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(q[3]), *map(float, q[:3])))
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
        px.CreateSleepThresholdAttr(0.0)  # never sleeps: a sleeping body reports no contacts
        px.CreateStabilizationThresholdAttr(0.0)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)

    count = {"n": 0}

    def hull(path, v, color, friction=L.HARD_FRICTION, collide=True, combine="average"):
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
            hc.CreateMinThicknessAttr(0.0005)
            bind_physics_material(str(prim.GetPath()), material(friction, combine), stage=stage)
            count["n"] += 1
        bind_visual_material(str(prim.GetPath()), look(color), stage=stage)
        return m

    a = authored(SETUP_BASE)
    # ---------------------------------------------------------------- cabinet (kinematic, dryer frame)
    xform(f"{prim_path}/cabinet", *a["cabinet"])
    rigid(f"{prim_path}/cabinet", 50.0, kinematic=True)
    colors = dict(top=WHITE, housing=GREY, pocket=GREY, console=WHITE, body=WHITE)
    for name, v, kind in L.cabinet_pieces():
        hull(f"{prim_path}/cabinet/{name}", v, colors[kind])
    c, size = L.cabinet_body_box()
    # cabinet shell below the top panel: visual only on the inside region (the table and the housing are inside it);
    # its front face collides (the arms can touch it)
    hull(f"{prim_path}/cabinet/shell", L.box_v(c, size), WHITE, collide=False)
    hull(f"{prim_path}/cabinet/front_face", L.box_v((0.002, 0.0, -0.10), (0.004, L.CAB_W, 0.16)), WHITE)
    hull(f"{prim_path}/cabinet/console_strip", L.box_v((L.CAB_D - L.CONSOLE[0] - 0.001, 0.0, 0.12),
                                                       (0.002, 0.5, 0.04)), DARK, collide=False)
    # cosmetic console details: cycle knob and display
    kx = L.CAB_D - L.CONSOLE[0] - 0.012
    ang = np.linspace(0, 2 * np.pi, 24, endpoint=False)
    knob = np.r_[np.c_[np.full(24, kx), 0.20 + 0.03 * np.cos(ang), 0.09 + 0.03 * np.sin(ang)],
                 np.c_[np.full(24, kx + 0.011), 0.20 + 0.03 * np.cos(ang), 0.09 + 0.03 * np.sin(ang)]]
    hull(f"{prim_path}/cabinet/knob", knob, STEEL, collide=False)
    hull(f"{prim_path}/cabinet/display", L.box_v((L.CAB_D - L.CONSOLE[0] - 0.002, -0.17, 0.09), (0.003, 0.10, 0.035)),
         (0.10, 0.30, 0.35), collide=False)
    # ---------------------------------------------------------------- Start button (kinematic cap on the front panel)
    xform(f"{prim_path}/button", *a["button"])
    rigid(f"{prim_path}/button", 0.01, kinematic=True)
    hull(f"{prim_path}/button/cap", L.box_v((0.0, 0.0, L.BUTTON_PROUD / 2),
                                            (2 * L.BUTTON_R, 2 * L.BUTTON_R, L.BUTTON_PROUD)), BTN)
    # ---------------------------------------------------------------- bin
    xform(f"{prim_path}/bin", *a["bin"])
    rigid(f"{prim_path}/bin", 1.0, kinematic=True)
    for i, v in enumerate(L.bin_pieces()):
        hull(f"{prim_path}/bin/w{i}", v, BIN_COL)
    # ---------------------------------------------------------------- lint screen (free body)
    xform(f"{prim_path}/screen", *a["screen"])
    inertia, com = L.screen_mass_props()
    rigid(f"{prim_path}/screen", L.SCREEN_MASS, inertia=inertia, com=com)
    for name, v in L.screen_pieces().items():
        hull(f"{prim_path}/screen/{name}", v, NYLON if name == "grip" else STEEL, friction=L.SCREEN_FRICTION)
    # ---------------------------------------------------------------- lint mat (free body)
    xform(f"{prim_path}/mat", *a["mat"])
    rigid(f"{prim_path}/mat", L.MAT_MASS, inertia=L.mat_inertia(), com=(-L.MAT_T / 2, 0.0, 0.0))
    back, front = L.mat_pieces()
    hull(f"{prim_path}/mat/back", back, LINT, friction=L.MAT_BACK_FRICTION, combine="min")
    hull(f"{prim_path}/mat/front", front, LINT, friction=L.MAT_FRICTION, combine="max")
    # the screen housing and the seated screen reach below the robot table's top inside the cabinet (the dryer stands
    # at the end of the table): the screen and the mat do not collide with the table slab (everything else does)
    env_root = prim_path.rsplit("/", 1)[0]
    for n in ("screen", "mat"):
        fp = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(f"{prim_path}/{n}"))
        fp.CreateFilteredPairsRel().SetTargets([Sdf.Path(f"{env_root}/table"), Sdf.Path(f"{env_root}/table/geometry/mesh")])
    _SPEC["paths"] = {n: f"{prim_path}/{n}" for n in BODIES}
    _SPEC["root"] = prim_path
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
        self.stage = get_current_stage()
        stage_id = UsdUtils.StageCache.Get().GetId(self.stage).ToLongInt()
        self.simview = tensors.create_simulation_view("torch", stage_id=stage_id)
        self.simview.set_subspace_roots("/")
        self.root = _SPEC["root"]
        P = lambda n: f"{self.root}/{n}"  # noqa: E731
        self.P = P
        self.view = self.simview.create_rigid_body_view([P("screen"), P("mat"), P("cabinet"), P("button"), P("bin")])
        robot_links = []
        for arm in ("left_robot", "right_robot"):
            lp = self.base.scene[arm].root_view.link_paths
            robot_links += list(lp[0] if lp and isinstance(lp[0], (list, tuple)) else lp)
        self.robot_links = robot_links
        self.cv_button = self.simview.create_rigid_contact_view(
            [P("button")], filter_patterns=[robot_links], max_contact_data_count=1024
        )
        self.cv_mat = self.simview.create_rigid_contact_view(
            [P("mat")], filter_patterns=[[P("screen"), P("bin"), P("cabinet")] + robot_links],
            max_contact_data_count=2048,
        )
        self.origin = self.base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        self.setup = SETUP_BASE
        self.trial_active = False
        self.joint_seq = 0
        self.joint_path = None
        self.lint = authored(SETUP_BASE)["lint"]
        self._init_state()

    def _init_state(self):
        self.scorer = Scorer()
        self.steps = 0
        self.t = 0.0
        self.button_s = 0.0
        self.button_peak = 0.0
        self.button_events = []
        self.started = None
        self.robot_on_mat_peak = 0.0
        self.hook_s = 0.0
        self.nonfinite = 0
        STATE.update(score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, setup=self.setup)

    def begin_trial(self, trial):
        self._init_state()
        self.trial_active = True
        STATE.update(trial=trial, setup=self.setup, start=L.setup_spec(self.setup))

    # ------------------------------------------------------------------ mat joint
    def make_mat_joint(self, poses):
        """Breakable fixed joint screen -> mat at their current relative pose (created on reset only)."""
        from pxr import Gf, Sdf, UsdPhysics

        sp, sr = poses["screen"]
        mp, mr = poses["mat"]
        self.joint_seq += 1
        path = f"{self.root}/screen/lint_bond_{self.joint_seq}"
        j = UsdPhysics.FixedJoint.Define(self.stage, path)
        j.CreateBody0Rel().SetTargets([Sdf.Path(self.P("screen"))])
        j.CreateBody1Rel().SetTargets([Sdf.Path(self.P("mat"))])
        rel_p, rel_r = sr.T @ (mp - sp), sr.T @ mr
        q = _quat(rel_r)
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, rel_p)))
        j.CreateLocalRot0Attr(Gf.Quatf(float(q[3]), *map(float, q[:3])))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateCollisionEnabledAttr(True)
        j.CreateBreakForceAttr(float(L.MAT_BREAK_N))
        j.CreateBreakTorqueAttr(float(L.MAT_BREAK_N * L.MAT_H / 2))
        j.CreateExcludeFromArticulationAttr(True)
        self.joint_path = path

    def drop_mat_joint(self):
        if self.joint_path:
            prim = self.stage.GetPrimAtPath(self.joint_path)
            if prim and prim.IsValid():
                self.stage.RemovePrim(self.joint_path)
        self.joint_path = None

    # ------------------------------------------------------------------ reading
    def read(self):
        tf = self.view.get_transforms().detach().cpu().numpy().astype(float)
        names = ("screen", "mat", "cabinet", "button", "bin")
        return {n: (tf[i, :3] - self.origin, _rot(tf[i, 3:7])) for i, n in enumerate(names)}

    def velocities(self):
        v = self.view.get_velocities().detach().cpu().numpy().astype(float)
        return v[0, :3], v[0, 3:6], v[1, :3]

    def physics(self, dt):
        t0 = time.perf_counter()
        self.t += dt
        if self.trial_active:
            fb = self.cv_button.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
            f = float(np.linalg.norm(fb[0], axis=-1).max()) if fb.size else 0.0
            self.button_peak = max(self.button_peak, f)
            self.button_s = self.button_s + dt if f >= L.BUTTON_PRESS_N else 0.0
            if self.button_s >= L.BUTTON_HOLD_S - 1e-9 and self.started is None:
                self.started = dict(t=round(self.t, 3), step=self.steps)
                self.button_events.append(dict(self.started))
            fm = self.cv_mat.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
            if fm.size:
                self.robot_on_mat_peak = max(self.robot_on_mat_peak, float(np.linalg.norm(fm[0, 3:], axis=-1).max()))
            self.last_contacts = dict(button_n=round(f, 3))
        self.hook_s += time.perf_counter() - t0

    def joint_broken(self):
        """True when the breakable mat joint no longer holds (PhysX broke it) or does not exist."""
        if not self.lint:
            return True
        poses = self.read()
        sp, sr = poses["screen"]
        mp, mr = poses["mat"]
        want_p, want_r = L.mat_pose_in_screen()
        dev = np.linalg.norm(sr.T @ (mp - sp) - want_p)
        return bool(dev > 0.002)

    # ------------------------------------------------------------------ reset
    def restore_authored(self, setup):
        """Environment reset only (never during a trial): every body to the setup's pose, the mat joint re-made."""
        torch = self.torch
        self.setup = setup
        self.trial_active = False
        a = authored(setup)
        self.lint = a["lint"]
        self.drop_mat_joint()
        tf = self.view.get_transforms().clone()
        names = ("screen", "mat", "cabinet", "button", "bin")
        for i, n in enumerate(names):
            pos, rot = a[n]
            tf[i, :3] = torch.tensor(np.asarray(pos, float) + self.origin, dtype=tf.dtype)
            tf[i, 3:7] = torch.tensor(_quat(rot), dtype=tf.dtype)
        idx = torch.tensor(list(range(len(names))), dtype=torch.int32, device=tf.device)
        self.view.set_transforms(tf, idx)
        self.view.set_velocities(
            torch.zeros_like(self.view.get_velocities()),
            torch.tensor([0, 1], dtype=torch.int32, device=tf.device),
        )
        if self.lint:
            self._sync_usd(a)
            self.make_mat_joint(a)
        self._init_state()

    def _sync_usd(self, a):
        """Write the screen / mat reset poses into USD too (a joint is authored against the USD poses)."""
        from pxr import Gf, UsdGeom

        for n in ("screen", "mat"):
            prim = self.stage.GetPrimAtPath(self.P(n))
            xf = UsdGeom.Xformable(prim)
            ops = {op.GetOpName(): op for op in xf.GetOrderedXformOps()}
            pos, rot = a[n]
            q = _quat(rot)
            if "xformOp:translate" in ops:
                ops["xformOp:translate"].Set(Gf.Vec3d(*map(float, pos)))
            if "xformOp:orient" in ops:
                ops["xformOp:orient"].Set(Gf.Quatd(float(q[3]), *map(float, q[:3])))

    # ------------------------------------------------------------------ scoring
    def control(self, environment):
        if not self.trial_active:
            return
        self.steps += 1
        poses = self.read()
        sp, sr = poses["screen"]
        mp, mr = poses["mat"]
        if not (np.all(np.isfinite(sp)) and np.all(np.isfinite(mp))):
            self.nonfinite += 1
            STATE["nonfinite"] = self.nonfinite
            return
        tips = finger_tips(environment)
        m = measure(sp, sr, mp, mr, poses["cabinet"][0], poses["bin"][0], self.lint, self.joint_broken(), tips)
        level = self.scorer.step(m, self.started is not None)
        STATE.update(
            score=level,
            level=level,
            steps=self.steps,
            nonfinite=self.nonfinite,
            detail=dict(
                self.scorer.last,
                started=self.started,
                button_peak_n=round(self.button_peak, 2),
                robot_on_mat_peak_n=round(self.robot_on_mat_peak, 2),
                screen_pos_mm=np.round(sp * 1000, 2).tolist(),
                mat_pos_mm=np.round(mp * 1000, 2).tolist(),
                setup=self.setup,
                lint=self.lint,
                hook_wall_s=round(self.hook_s, 2),
                contacts=getattr(self, "last_contacts", None),
            ),
        )


_RT: dict = {"rt": None, "sim": None, "trials": 0}


def runtime():
    return _RT["rt"]


def bench_state():
    """Current poses {'screen', 'mat', 'cabinet', 'button', 'bin'} for the scripted demo (read-only)."""
    return _RT["rt"].read() if _RT["rt"] else None


def _patch_scene(scene) -> None:
    if getattr(scene, "_dryer_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Dryer):
            return _arena_object(spec)
        return orig_task_object(spec)

    def object_states(environment, names):
        from yam_arena.task_file import ObjectState

        plain = [n for n in names if n != "dryer"]
        out = orig_states(environment, plain) if plain else {}
        if "dryer" in names:
            rt = _RT["rt"]
            p, r = rt.read()["screen"] if rt else authored(SETUP_BASE)["screen"]
            z = np.zeros(3)
            out["dryer"] = ObjectState(pos=np.asarray(p), quat_xyzw=_quat(r), lin_vel=z, ang_vel=z, low=p - 0.2,
                                       high=p + 0.2)
        return {n: out[n] for n in names}

    orig_build = scene.build_task_environment

    def build(embodiment, task, control_hz=None):
        environment = orig_build(embodiment, task, control_hz)
        if any(isinstance(o, Dryer) for o in task.objects):
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
    scene._dryer_patched = True


def _has_dryer(emb):
    return any(isinstance(o, Dryer) for o in (emb.task.objects if emb.task is not None else ()))


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_dryer_patched", False):
        return
    base_task_step, base_reset, base_ensure = cls._task_step, cls.reset, cls._ensure_sim

    def _ensure_sim(self, *a, **k):
        env_before = self._env
        out = base_ensure(self, *a, **k)
        if self._env is not None and self._env is not env_before and _has_dryer(self):
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
            if rt.setup != setup or rt.joint_path is None and authored(setup)["lint"]:  # first reset of a fresh env
                rt.restore_authored(setup)
                obs = base_reset(self, *a, **k)
            rt.begin_trial(_RT["trials"])
            _RT["trials"] += 1
        return obs

    cls._ensure_sim, cls._task_step, cls.reset = _ensure_sim, _task_step, reset
    cls._dryer_patched = True


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
