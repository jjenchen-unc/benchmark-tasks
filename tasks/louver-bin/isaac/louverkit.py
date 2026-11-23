"""louverkit: the louvered panel on its back board, the bin that already hangs on it and the loose bin, for yam_arena
(Isaac Lab-Arena / PhysX), plus the runtime hooks.

Authored as one custom-spawned assembly (yam_arena task-file primitives are single boxes/spheres):
* the panel (steel sheet, six louver rows, side returns) and its plywood back board as one kinematic rigid body (so
  PhysX contact views can report forces on it);
* the two AkroBins as free rigid bodies (convex pieces: bottom, back, front, two side walls with rim beads, hanger
  shelf and hanger tab).
Geometry: ``../src/layout.py``; scorer: ``../src/scoring.py`` (pure functions).

No computed forces: the bins hang, tip and fall by contact and gravity alone. Nothing here writes a bin pose or
velocity during a trial; ``Runtime.restore_authored`` puts the panel, the hanging bin and the loose bin at the setup's
poses only on an environment reset. The scripted demo READS poses through ``bench_state()``.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SRC = HERE.parent / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

import layout as L  # noqa: E402
from scoring import measure, Scorer  # noqa: E402
from yam_arena.task_file import Box  # noqa: E402

PHYSICS_DT = float(os.environ.get("LB_PHYSICS_DT", str(1 / 240)))
CONTACT_OFFSET = float(os.environ.get("LB_CONTACT_OFFSET", ".0005"))
SETUP_BASE = int(os.environ.get("LB_SETUP", "0"))  # named setup of the first unseeded trial; later trials cycle
FORCE_NAMED = "LB_SETUP" in os.environ

STEEL_GREY, BOARD, BIN_BLUE, BIN_RED, LABEL_WHITE, DARK = (
    (0.62, 0.64, 0.66),
    (0.72, 0.60, 0.42),
    (0.10, 0.30, 0.70),
    (0.75, 0.14, 0.12),
    (0.95, 0.95, 0.93),
    (0.15, 0.15, 0.16),
)
KINEMATIC = ("panel",)
DYNAMIC = ("bin", "ref")
BODIES = DYNAMIC + KINEMATIC


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
    """Authored (reset) pose of every body for a setup: {name: (pos, rot)} in the env frame."""
    spec = L.setup_spec(setup)
    return {"bin": L.loose_pose(spec), "ref": L.ref_pose(spec), "panel": (L.panel_origin(spec), np.eye(3))}


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
        x.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(q[3]), *map(float, q[:3])))
        return x

    def rigid(path, mass=None, kinematic=False, com=None):
        prim = stage.GetPrimAtPath(path)
        rb = UsdPhysics.RigidBodyAPI.Apply(prim)
        if kinematic:
            rb.CreateKinematicEnabledAttr(True)
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(float(mass if mass is not None else 1.0))
        if com is not None:
            m.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
        px = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        px.CreateMaxDepenetrationVelocityAttr(0.3)
        px.CreateSolverPositionIterationCountAttr(24)
        px.CreateSolverVelocityIterationCountAttr(4)
        px.CreateLinearDampingAttr(0.0)
        px.CreateAngularDampingAttr(0.05)
        px.CreateSleepThresholdAttr(0.0)  # never sleeps: a sleeping body reports no contacts
        px.CreateStabilizationThresholdAttr(0.0)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)

    count = {"n": 0}

    def hull(path, v, color, friction=L.STEEL_FRICTION, collide=True):
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
            bind_physics_material(str(prim.GetPath()), material(friction), stage=stage)
            count["n"] += 1
        bind_visual_material(str(prim.GetPath()), look(color), stage=stage)
        return m

    def box(path, x0, x1, y0, y1, z0, z1, color, friction=L.STEEL_FRICTION, collide=True):
        return hull(path, L.box_corners(x0, x1, y0, y1, z0, z1), color, friction, collide)

    A = authored(SETUP_BASE)
    # ---------------------------------------------------------------- panel + back board (kinematic); frame = panel
    # origin (face x, centre y, bottom edge z)
    xform(f"{prim_path}/panel", *A["panel"])
    rigid(f"{prim_path}/panel", 8.0, kinematic=True)
    pn = f"{prim_path}/panel"
    hw, H = L.PANEL_W / 2, L.PANEL_H
    # steel sheet: visible face at x = 0; collision slab behind it
    box(f"{pn}/sheet", 0.0, L.PANEL_COLL_T, -hw, hw, 0.0, H, STEEL_GREY)
    for s, tag in ((1, "l"), (-1, "r")):
        y0, y1 = sorted((s * hw, s * (hw - 0.0012)))
        box(f"{pn}/return_{tag}", 0.0, L.PANEL_RETURN, y0, y1, 0.0, H, STEEL_GREY)
    ly = hw - L.LOUVER_MARGIN
    for k in range(1, L.N_LOUVERS + 1):
        zt = L.lip_top_z(k) - L.PANEL_Z0
        box(f"{pn}/lip_{k}", -L.LIP_OUT, -L.LIP_OUT + L.LIP_T, -ly, ly, zt - L.LIP_H, zt, STEEL_GREY)
        box(f"{pn}/floor_{k}", -L.LIP_OUT, 0.0, -ly, ly, zt - L.LIP_H - L.FLOOR_T, zt - L.LIP_H, STEEL_GREY)
        # dark shadow strip inside the channel (visual only: the open louver reads dark from the front)
        box(f"{pn}/shadow_{k}", -0.0009, -0.0002, -ly, ly, zt - L.LIP_H, zt - 0.002, DARK, collide=False)
    # plywood back board (3/4 in) behind the returns, standing on the bench
    bx0 = L.PANEL_RETURN
    zb = L.TOP - L.PANEL_Z0 + 0.0005
    box(f"{pn}/board", bx0, bx0 + 0.019, -0.500, 0.500, zb, H + 0.060, BOARD, friction=0.5)
    for s, tag in ((1, "l"), (-1, "r")):  # board feet (bench brackets)
        box(f"{pn}/foot_{tag}", bx0 + 0.019, bx0 + 0.120, s * 0.480 - 0.010, s * 0.480 + 0.010, zb, zb + 0.080,
            BOARD, friction=0.5)
    # ---------------------------------------------------------------- the two bins
    for name, color in (("bin", BIN_BLUE), ("ref", BIN_RED)):
        xform(f"{prim_path}/{name}", *A[name])
        rigid(f"{prim_path}/{name}", L.BIN_MASS, com=L.BIN_COM)
        bp = f"{prim_path}/{name}"
        for piece, v in L.bin_pieces():
            hull(f"{bp}/{piece}", v, color, friction=L.BIN_FRICTION)
        # label card in the front label holder (visual)
        box(f"{bp}/label", -L.BIN_L - 0.0006, -L.BIN_L, -0.040, 0.040, -L.BIN_H + 0.020, -L.BIN_H + 0.050,
            LABEL_WHITE, collide=False)
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
    """Contact-force logging every physics step; scoring every control step."""

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
        robot_links, finger_links = [], []
        for arm in ("left_robot", "right_robot"):
            lp = self.base.scene[arm].root_view.link_paths
            lp = list(lp[0] if lp and isinstance(lp[0], (list, tuple)) else lp)
            robot_links += lp
        self.robot_links = robot_links
        self.finger_links = [p for p in robot_links if "link_7" in p or "link_8" in p or "finger" in p.lower()]
        self.cv_grip = self.simview.create_rigid_contact_view(
            [P("bin"), P("ref")], filter_patterns=[robot_links, robot_links], max_contact_data_count=4096
        )
        self.cv_robot = self.simview.create_rigid_contact_view(
            [P("panel")], filter_patterns=[robot_links], max_contact_data_count=4096
        )
        self.origin = self.base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        self.setup = SETUP_BASE
        self.spec = L.setup_spec(self.setup)
        self.trial_active = False
        self._init_state()

    def _init_state(self):
        self.scorer = Scorer()
        self.steps = 0
        self.t = 0.0
        self.nonfinite = 0
        self.grip_peak = {"bin": 0.0, "ref": 0.0}
        self.grip_link_peak = 0.0
        self.robot_panel_peak = 0.0
        self.last_contacts = None
        STATE.update(score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, setup=self.setup)

    def begin_trial(self, trial):
        self._init_state()
        self.trial_active = True
        STATE.update(trial=trial, setup=self.setup, start=self.spec)

    # ------------------------------------------------------------------ reading
    def read(self):
        tf = self.view.get_transforms().detach().cpu().numpy().astype(float)
        return {n: (tf[i, :3] - self.origin, _rot(tf[i, 3:7])) for i, n in enumerate(BODIES)}

    def velocities(self):
        v = self.dyn.get_velocities().detach().cpu().numpy().astype(float)
        return {n: (v[i, :3], v[i, 3:6]) for i, n in enumerate(DYNAMIC)}

    # ------------------------------------------------------------------ physics hook (logging only)
    def physics(self, dt):
        self.t += dt
        if not self.trial_active:
            return
        gm = self.cv_grip.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
        now = {}
        for k, b in enumerate(DYNAMIC):
            if gm.size:
                per_link = np.linalg.norm(gm[k], axis=-1)
                g = float(per_link.max())
            else:
                g = 0.0
            now[b] = g
            self.grip_peak[b] = max(self.grip_peak[b], g)
        rb = self.cv_robot.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
        rp = float(np.linalg.norm(rb, axis=-1).max()) if rb.size else 0.0
        self.robot_panel_peak = max(self.robot_panel_peak, rp)
        self.last_contacts = dict(grip_n={k: round(v, 3) for k, v in now.items()}, robot_panel_n=round(rp, 3))

    # ------------------------------------------------------------------ reset
    def restore_authored(self, setup, override=None):
        """Environment reset only (never during a trial): panel, hanging bin and loose bin to the setup's poses
        (``override``: development probes only, {body: (pos, rot)})."""
        torch = self.torch
        self.setup = setup
        self.spec = L.setup_spec(setup)
        self.trial_active = False
        a = authored(setup)
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
        self._init_state()

    # ------------------------------------------------------------------ scoring
    def control(self, environment):
        if not self.trial_active:
            return
        self.steps += 1
        poses = self.read()
        pb, rb = poses["bin"]
        pr, rr = poses["ref"]
        vel = self.velocities()
        if not all(np.all(np.isfinite(x)) for x in (pb, rb, pr, rr)):
            self.nonfinite += 1
            STATE["nonfinite"] = self.nonfinite
            return
        tips = finger_tips(environment)
        m = measure(self.spec, pb, rb, pr, rr, tips)
        lin = max(float(np.linalg.norm(vel[n][0])) for n in DYNAMIC)
        level = self.scorer.step(m, lin)
        STATE.update(
            score=level,
            level=level,
            steps=self.steps,
            nonfinite=self.nonfinite,
            detail=dict(
                self.scorer.last,
                grip_peak_n={k: round(v, 2) for k, v in self.grip_peak.items()},
                robot_panel_peak_n=round(self.robot_panel_peak, 2),
                bin_mm=np.round(pb * 1000, 3).tolist(),
                ref_mm=np.round(pr * 1000, 3).tolist(),
                setup=self.setup,
                contacts=self.last_contacts,
            ),
        )


_RT: dict = {"rt": None, "sim": None, "trials": 0, "override": None}


def runtime():
    return _RT["rt"]


def bench_state():
    """Current poses of every body for the scripted demo (read-only), plus the start spec."""
    rt = _RT["rt"]
    if rt is None:
        return None
    out = rt.read()
    out["spec"] = rt.spec
    return out


def _patch_scene(scene) -> None:
    if getattr(scene, "_lb_patched", False):
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
            p, r = rt.read()["bin"] if rt else authored(SETUP_BASE)["bin"]
            z = np.zeros(3)
            out["bench"] = ObjectState(pos=np.asarray(p), quat_xyzw=_quat(r), lin_vel=z, ang_vel=z, low=p - 0.2,
                                       high=p + 0.2)
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
    scene._lb_patched = True


def _has_bench(emb):
    return any(isinstance(o, Bench) for o in (emb.task.objects if emb.task is not None else ()))


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_lb_patched", False):
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
            # the reset's settling steps may have moved the bins (a bin left hanging or fallen in the last trial): put
            # them back once more, after the settle
            rt.restore_authored(setup, _RT["override"])
            rt.begin_trial(_RT["trials"])
            _RT["trials"] += 1
        return obs

    cls._ensure_sim, cls._task_step, cls.reset = _ensure_sim, _task_step, reset
    cls._lb_patched = True


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
