"""standkit: a loose rigid part built from boxes and cylinders, optionally with one swinging arm, for yam_arena task files.

yam_arena's task-file objects are single primitives. A ``Part`` is one rigid body made of several box and cylinder
colliders. Give it a ``Swing`` and it becomes an articulation: the swing is a second rigid body on a revolute joint
about the part's vertical (z) axis through ``Swing.anchor``, with rotational damping and no spring, so it turns
under a push and stays where it is left.

Importing this module hooks ``yam_arena.scene`` and the embodiment's step and reset without modifying yam_arena:
``success()/score()`` receive a :class:`PartState` for each part (the body origin's pose, the swing's angle and the
swing body's pose). ``on_step``/``on_reset`` register callbacks run after every control step / reset, and ``env()``
returns the Isaac Lab environment (used by the task to see where the grippers are).

Pure Python at import time; everything that needs Isaac Sim is imported lazily.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena import task_file
from yam_arena.task_file import Box, ObjectState

__all__ = ["Part", "PartState", "Swing", "part", "on_reset", "on_step", "env", "install"]

Color = tuple[float, float, float]
BoxSpec = tuple  # centre, size, colour (part frame, m), and optionally a turn about the vertical (rad)
CylSpec = tuple  # centre, radius, height (axis = part z), colour


@dataclass(frozen=True)
class Swing:
    """A rigid body on a revolute joint about the vertical through ``anchor`` (part frame). Its ``boxes`` are given
    in a frame at the anchor, at joint angle 0. ``com`` is its centre of mass in that frame."""

    name: str
    anchor: tuple[float, float, float]
    boxes: tuple[BoxSpec, ...]
    mass: float
    com: tuple[float, float, float]
    damping: float = 0.3  # N m s / rad


@dataclass(frozen=True)
class Part(Box):
    """A rigid part. It subclasses Box only so yam_arena's task-file checks accept it; ``size`` is its bounding
    box and ``pos`` the body origin."""

    boxes: tuple[BoxSpec, ...] = ()
    cylinders: tuple[CylSpec, ...] = ()
    com: tuple[float, float, float] | None = None
    swing: Swing | None = None
    friction: float = 0.6


def part(name, pos, boxes, mass, yaw=0.0, cylinders=(), com=None, swing=None, color=(0.5, 0.5, 0.5),
         friction=0.6) -> Part:
    pts = [np.array(b[0]) + s * np.array(b[1]) / 2 for b in boxes for s in (-1, 1)]
    pts += [np.array(c) + s * np.array([r, r, h / 2]) for c, r, h, _ in cylinders for s in (-1, 1)]
    lo, hi = np.min(pts, axis=0), np.max(pts, axis=0)
    return Part(name=name, pos=tuple(float(v) for v in pos), size=tuple((hi - lo).tolist()), mass=float(mass), color=color, yaw=float(yaw),
                boxes=tuple(boxes), cylinders=tuple(cylinders), com=com, swing=swing, friction=friction)


def _rot(quat_xyzw) -> np.ndarray:
    x, y, z, w = quat_xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


@dataclass(frozen=True)
class PartState(ObjectState):
    """The body origin's pose, the swing's angle (rad, 0 at the start, positive anticlockwise seen from above) and the
    swing body's pose (``swing_pos``, ``swing_quat_xyzw``: the anchor frame)."""

    swing_angle: float = 0.0
    swing_pos: np.ndarray | None = None
    swing_quat_xyzw: np.ndarray | None = None

    def rotation(self) -> np.ndarray:
        return _rot(self.quat_xyzw)

    def to_world(self, local_point) -> np.ndarray:
        return self.pos + self.rotation() @ np.asarray(local_point, float)

    def swing_to_world(self, local_point) -> np.ndarray:
        return self.swing_pos + _rot(self.swing_quat_xyzw) @ np.asarray(local_point, float)


# -- spawning -----------------------------------------------------------------------------------------------------

_SPECS: dict[str, Part] = {}


def _spawn_part(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb

    spec: Part = _SPECS[cfg.part_name]
    stage = get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))

    phys = f"{prim_path}/Looks/physics"
    pm = sim_utils.RigidBodyMaterialCfg(static_friction=spec.friction, dynamic_friction=spec.friction)
    pm.func(phys, pm)
    looks: dict[Color, str] = {}

    def look(color: Color) -> str:
        if color not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.5)
            vis.func(path, vis)
            looks[color] = path
        return looks[color]

    def body(path: str, pos, mass: float, com):
        xform = UsdGeom.Xform.Define(stage, path)
        xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*pos))
        xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
        prim = xform.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        mapi = UsdPhysics.MassAPI.Apply(prim)
        mapi.CreateMassAttr(mass)
        if com is not None:
            mapi.CreateCenterOfMassAttr(Gf.Vec3f(*com))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(1.0)
        rb.CreateSolverPositionIterationCountAttr(32)
        return prim

    def collide(prim, color):
        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(0.002)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), phys, stage=stage)
        bind_visual_material(str(prim.GetPath()), look(color), stage=stage)

    def boxes(path: str, specs):
        for i, (centre, size, color, *turn) in enumerate(specs):
            cube = UsdGeom.Cube.Define(stage, f"{path}/box_{i}")
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(*centre))
            if turn and turn[0]:
                cube.AddOrientOp().Set(Gf.Quatf(math.cos(turn[0] / 2), 0.0, 0.0, math.sin(turn[0] / 2)))
            cube.AddScaleOp().Set(Gf.Vec3f(*size))
            collide(cube.GetPrim(), color)

    def cylinders(path: str, specs):
        for i, (centre, radius, height, color) in enumerate(specs):
            cyl = UsdGeom.Cylinder.Define(stage, f"{path}/cyl_{i}")
            cyl.CreateAxisAttr("Z")
            cyl.CreateRadiusAttr(radius)
            cyl.CreateHeightAttr(height)
            cyl.AddTranslateOp().Set(Gf.Vec3d(*centre))
            collide(cyl.GetPrim(), color)

    main = f"{prim_path}/body"
    body(main, (0.0, 0.0, 0.0), spec.mass, spec.com)
    boxes(main, spec.boxes)
    cylinders(main, spec.cylinders)
    s = spec.swing
    if s is not None:
        path = f"{prim_path}/{s.name}"
        body(path, s.anchor, s.mass, s.com)
        boxes(path, s.boxes)
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{path}/{s.name}_hinge")
        j.CreateAxisAttr("Z")
        j.CreateBody0Rel().SetTargets([Sdf.Path(main)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*s.anchor))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(s.damping * math.pi / 180)  # USD angular drives are per degree
        drive.CreateTargetVelocityAttr(0.0)
        UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(main))
        art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(main))
        art.CreateEnabledSelfCollisionsAttr(False)
        art.CreateSolverPositionIterationCountAttr(32)
        art.CreateSolverVelocityIterationCountAttr(4)
    return root.GetPrim()


def _arena_object(spec: Part):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class PartSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_part)
        part_name: str = ""

    class PartObject(Object):
        def _generate_articulation_cfg(self):
            s = spec.swing
            actuators = {s.name: ImplicitActuatorCfg(joint_names_expr=[f"{s.name}_hinge"], stiffness=0.0,
                                                     damping=s.damping)}
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            return self._add_initial_pose_to_cfg(cfg)

    kind = ObjectType.ARTICULATION if spec.swing else ObjectType.RIGID
    return PartObject(
        name=spec.name,
        object_type=kind,
        spawner_cfg=PartSpawnerCfg(part_name=spec.name),
        initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2),
                                                                       math.cos(spec.yaw / 2))),
    )


def _np(x) -> np.ndarray:
    return getattr(x, "torch", x).detach().cpu().numpy().astype(np.float64)


def _part_state(environment, spec: Part) -> PartState:
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _np(base.scene.env_origins[0])
    pose = _np(asset.data.root_link_pose_w)[0]
    vel = _np(asset.data.root_vel_w)[0] if hasattr(asset.data, "root_vel_w") else np.zeros(6)
    R = _rot(pose[3:7])
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * np.array(spec.size) / 2
    pts = pose[:3] - origin + corners @ R.T
    extra = {}
    if spec.swing is not None:
        names = list(asset.joint_names)
        q = _np(asset.data.joint_pos)[0]
        bodies = list(asset.body_names)
        bp = _np(asset.data.body_link_pose_w)[0][bodies.index(spec.swing.name)]
        extra = dict(swing_angle=float(q[names.index(f"{spec.swing.name}_hinge")]), swing_pos=bp[:3] - origin,
                     swing_quat_xyzw=bp[3:7])
    return PartState(pos=pose[:3] - origin, quat_xyzw=pose[3:7], lin_vel=vel[:3], ang_vel=vel[3:6],
                     low=pts.min(0), high=pts.max(0), **extra)


def gripper_points(environment) -> list[np.ndarray]:
    """World points (env frame) on both grippers: the palm, the centre between the finger pads and each pad tip."""
    base = environment.unwrapped
    origin = _np(base.scene.env_origins[0])
    out = []
    for key in ("left_robot", "right_robot"):
        robot = base.scene[key]
        names = list(robot.body_names)
        g = _np(robot.data.body_link_pose_w)[0][names.index("gripper")]
        R, p = _rot(g[3:7]), g[:3] - origin
        jn = list(robot.joint_names)
        half = abs(float(_np(robot.data.joint_pos)[0][jn.index("joint7")])) + 0.008
        for local in ((0, 0, 0.06), (0, 0, 0.134), (half, 0, 0.147), (-half, 0, 0.147), (half, 0, 0.12),
                      (-half, 0, 0.12)):
            out.append(p + R @ np.array(local, float))
    return out


# -- hooking yam_arena -------------------------------------------------------------------------------------------

_STEP: list = []
_RESET: list = []
_ENV: dict = {"env": None}


def on_step(fn):
    _STEP.append(fn)
    return fn


def on_reset(fn):
    _RESET.append(fn)
    return fn


def env():
    return _ENV["env"]


def _patch_scene(scene) -> None:
    if getattr(scene, "_standkit_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Part):
            _SPECS[spec.name] = spec
            return _arena_object(spec)
        return orig_task_object(spec)

    def object_states(environment, names):
        _ENV["env"] = environment
        plain = [n for n in names if n not in _SPECS]
        out = orig_states(environment, plain) if plain else {}
        for n in names:
            if n in _SPECS:
                out[n] = _part_state(environment, _SPECS[n])
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._standkit_patched = True


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_standkit_patched", False):
        return
    base_task_step, base_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        for fn in _STEP:
            fn(self._env)
        return base_task_step(self, *a, **k)

    def reset(self, *a, **k):
        obs = base_reset(self, *a, **k)
        _ENV["env"] = self._env
        for fn in _RESET:
            fn(self._env)
        return obs

    cls._task_step, cls.reset = _task_step, reset
    cls._standkit_patched = True


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


def install() -> None:
    for name, patch in _Finder.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
assert task_file
