"""Task-local USD parts and evaluator hooks for the D1R pilot; Isaac imports are lazy.

Pattern: the same import-hook approach as the accepted main-repo partkit/cablekit and the
bottle baseline. The installed yam_arena/Arena SDK is not modified. After reset no object
pose, joint position, attachment, flag or score is written. Per-physics-substep writes are the
passive lid spring's constant drive target (native springref, re-applied because a scene reset
clears targets) and the elastic-cord forces of strap.py on the hooks and the lid (a mechanism
model computed from the current simulated hook/lid state). The policy cannot command any fixture
joint and never observes cord, route or seat state.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import importlib.abc
import importlib.util
import json
import math
import os
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import geometry as geo  # noqa: E402
from scoring import Scorer, SNAPBACK_MPS  # noqa: E402
import strap  # noqa: E402
from guards import GuardState, sensor_channels
from visuals import CordVisuals
from yam_arena.task_file import Box, ObjectState  # noqa: E402


@dataclass(frozen=True)
class Tote(Box):
    """Fixed-base tote articulation: rigid walls, brackets, eye bars and a lid on a sprung 2 cm slide."""


@dataclass(frozen=True)
class StrapTray(Box):
    """Fixed tray plate with slotted hook holders and visual tag cards."""


@dataclass(frozen=True)
class Hook(Box):
    """Free 50 g wire hook: handle box + shank box (native collision shapes)."""


_GUARDS = GuardState()
_VISUALS = None
_TRACE = None
_SCORER = None
_CORDS: dict[int, "strap.Cord"] = {}
_FRAME = None
_PREV_HOOK_POS: dict[str, np.ndarray] = {}  # for the 20 Hz displacement speed
_LAST_SPEED: dict[str, float] = {}
DIAGNOSTIC_LID_LOAD_N = 0.0  # evaluator-only calibration load (probe), never set in a trial
_SPECS: dict[str, Box] = {}


def arr(value):
    return getattr(value, "torch", value).detach().cpu().numpy()


# -- USD -------------------------------------------------------------------------------------

def _common(stage, prim_path, translation, orientation):
    from pxr import Gf, UsdGeom
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, x, y, z))
    return root


def _kit(stage, prim_path, friction):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb
    phys = f"{prim_path}/Looks/physics"
    pm = sim_utils.RigidBodyMaterialCfg(static_friction=friction, dynamic_friction=friction)
    pm.func(phys, pm)
    looks = {}

    def look(color):
        key = tuple(round(c, 3) for c in color)
        if key not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            cfg = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(key), roughness=0.5)
            cfg.func(path, cfg)
            looks[key] = path
        return looks[key]

    def box(path, centre, half, color, collide=True):
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(Gf.Vec3d(*centre))
        cube.AddScaleOp().Set(Gf.Vec3f(*(2 * h for h in half)))
        _finish(cube.GetPrim(), color, collide)

    def capsule_y(path, centre, radius, half_len, color, collide=True):
        cap = UsdGeom.Capsule.Define(stage, path)
        cap.CreateAxisAttr("Y")
        cap.CreateRadiusAttr(radius)
        cap.CreateHeightAttr(2 * half_len)
        cap.AddTranslateOp().Set(Gf.Vec3d(*centre))
        _finish(cap.GetPrim(), color, collide)

    def _finish(prim, color, collide):
        bind_visual_material(str(prim.GetPath()), look(color), stage=stage)
        if collide:
            UsdPhysics.CollisionAPI.Apply(prim)
            cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            cp.CreateContactOffsetAttr(0.001)
            cp.CreateRestOffsetAttr(0.0)
            bind_physics_material(str(prim.GetPath()), phys, stage=stage)

    def body(path, pos, mass, com=(0, 0, 0)):
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*pos))
        xf.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(mass)
        m.CreateCenterOfMassAttr(Gf.Vec3f(*com))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(0.5)
        rb.CreateSolverPositionIterationCountAttr(32)
        rb.CreateSolverVelocityIterationCountAttr(4)
        return prim

    return box, capsule_y, body


TOTE_RGB, LID_RGB, STEEL_RGB, BRACKET_RGB = (0.78, 0.80, 0.84), (0.30, 0.42, 0.60), (0.55, 0.55, 0.58), (0.35, 0.35, 0.38)


def spawn_tote(prim_path, spec, translation=None, orientation=None):
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdPhysics
    stage = get_current_stage()
    t = geo.tote(spec.name)
    root = _common(stage, prim_path, translation, orientation)
    box, capsule_y, body = _kit(stage, prim_path, friction=0.5)
    shell = f"{prim_path}/shell"
    body(shell, (0, 0, 0), geo.WALL_MASS * 5)
    for nm, pos, half in t["walls"]:
        box(f"{shell}/{nm}", pos, half, TOTE_RGB)
    for nm, pos, half in t["brackets"]:
        box(f"{shell}/{nm}", pos, half, BRACKET_RGB)
    for nm, pos in t["eyes"]:
        # Finite steel eye bar: 3 mm radius, 26 mm centreline along the tote's y (native) axis.
        capsule_y(f"{shell}/{nm}", pos, geo.BAR_RADIUS, geo.BAR_HALF, STEEL_RGB)
    # Fixed base: weld the shell to the world.
    fixed = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/world_weld")
    fixed.CreateBody1Rel().SetTargets([Sdf.Path(shell)])
    lid = f"{prim_path}/lid"
    shut = (0.0, 0.0, t["lid_shut_z"])
    body(lid, shut, geo.LID_MASS)
    box(f"{lid}/plate", (0, 0, 0), t["lid_half"], LID_RGB)
    j = UsdPhysics.PrismaticJoint.Define(stage, f"{prim_path}/lid_lift")
    j.CreateAxisAttr("Z")
    j.CreateBody0Rel().SetTargets([Sdf.Path(shell)])
    j.CreateBody1Rel().SetTargets([Sdf.Path(lid)])
    j.CreateLocalPos0Attr(Gf.Vec3f(*shut))
    j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    j.CreateLowerLimitAttr(0.0)
    j.CreateUpperLimitAttr(geo.LID_TRAVEL)
    j.CreateCollisionEnabledAttr(False)  # native <exclude body1=bin body2=lid>
    drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(t["kc"])
    drive.CreateDampingAttr(geo.LID_DAMPING)
    drive.CreateTargetPositionAttr(t["qref"])
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    art = PhysxSchema.PhysxArticulationAPI.Apply(root.GetPrim())
    art.CreateEnabledSelfCollisionsAttr(False)
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(4)
    return root.GetPrim()


def spawn_tray(prim_path, spec, translation=None, orientation=None):
    from isaaclab.sim.utils.stage import get_current_stage
    stage = get_current_stage()
    tr = geo.tray()
    root = _common(stage, prim_path, translation, orientation)
    box, _, _ = _kit(stage, prim_path, friction=0.6)
    # Static colliders (no rigid body API): fixed in place like a yam_arena fixed Box.
    box(f"{prim_path}/plate", (0, 0, 0), tr["plate_half"], (0.25, 0.25, 0.27))
    for nm, pos, half in tr["holders"]:
        box(f"{prim_path}/{nm}", (pos[0], pos[1], pos[2] - geo.TRAY_T), half, (0.32, 0.32, 0.35))
    for i, (nm, pos, half) in enumerate(tr["cards"]):
        box(f"{prim_path}/{nm}", (pos[0], pos[1], pos[2]), half, (0.96, 0.96, 0.94), collide=False)
    return root.GetPrim()


def spawn_hook(prim_path, spec, translation=None, orientation=None):
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, UsdPhysics
    stage = get_current_stage()
    colour = geo.COLOURS[int(spec.name.split("_")[1]) - 1]
    root = _common(stage, prim_path, translation, orientation)
    box, _, _ = _kit(stage, prim_path, friction=1.0)  # native hook friction 1
    prim = root.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
    m = UsdPhysics.MassAPI.Apply(prim)
    m.CreateMassAttr(geo.HOOK_MASS)
    # Native handle 40 g at origin + shank 10 g at +42.5 mm along the hook axis.
    m.CreateCenterOfMassAttr(Gf.Vec3f(geo.SHANK_MASS * geo.SHANK_POS[0] / geo.HOOK_MASS, 0.0, 0.0))
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateMaxDepenetrationVelocityAttr(0.5)
    rb.CreateSolverPositionIterationCountAttr(32)
    rb.CreateSolverVelocityIterationCountAttr(4)
    box(f"{prim_path}/handle", (0, 0, 0), geo.HANDLE, colour)
    box(f"{prim_path}/shank", geo.SHANK_POS, geo.SHANK_HALF, (0.7, 0.7, 0.72))
    # 12 mm L-foot at the shank tip (native task.json); the eye bar seats between shank and foot.
    box(f"{prim_path}/foot", strap.FOOT_POS, strap.FOOT_HALF, (0.7, 0.7, 0.72))
    # The foot does not collide with the walls of its own tray holder (holder modelled as notched for
    # the foot; native foot was non-colliding). Every other contact (eye bars, brackets, lid, grippers,
    # table, other holders) is resolved by PhysX.
    from pxr import Sdf
    strap_i, end = spec.name.split("_")[1], spec.name.split("_")[2]
    tray = prim_path.rsplit("/", 1)[0] + "/strap_tray"
    fp = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(f"{prim_path}/foot"))
    fp.CreateFilteredPairsRel().SetTargets([Sdf.Path(f"{tray}/holder_{strap_i}{end}_{w}") for w in ("xm", "xp", "ym", "yp")])
    return prim


SPAWNERS = {Tote: spawn_tote, StrapTray: spawn_tray, Hook: spawn_hook}


def arena_object(spec):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg, RigidObjectCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    fn = SPAWNERS[type(spec)]

    def spawn(prim_path, cfg, translation=None, orientation=None, **kwargs):
        return fn(prim_path, _SPECS[cfg.part_name], translation, orientation)

    @configclass
    class SpawnCfg(SpawnerCfg):
        func: object = clone(spawn)
        part_name: str = ""

    t = geo.tote(spec.name) if isinstance(spec, Tote) else None

    class PartObject(Object):
        def _generate_articulation_cfg(self):
            # Passive lid spring: native stiffness/damping, target = native springref (outside the 0..2 cm
            # joint range, so the spring is preloaded). No policy controls this joint.
            actuators = {"lid": ImplicitActuatorCfg(joint_names_expr=["lid_lift"], stiffness=t["kc"],
                                                    damping=geo.LID_DAMPING)}
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators,
                                  init_state=ArticulationCfg.InitialStateCfg(joint_pos={"lid_lift": geo.LID_TRAVEL},
                                                                             joint_vel={".*": 0.0}))
            return self._add_initial_pose_to_cfg(cfg)

    kind = {Tote: ObjectType.ARTICULATION, StrapTray: ObjectType.BASE, Hook: ObjectType.RIGID}[type(spec)]
    return PartObject(name=spec.name, object_type=kind, spawner_cfg=SpawnCfg(part_name=spec.name),
                      initial_pose=Pose(position_xyz=tuple(spec.pos),
                                        rotation_xyzw=_hook_quat(spec) if isinstance(spec, Hook)
                                        else (0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))))


def _hook_quat(spec):
    return next(h["quat_xyzw"] for h in geo.hooks() if h["name"] == spec.name)


# -- passive spring / elastic cords / measurement ---------------------------------------------

def _hook_pose(base, name):
    pose = arr(base.scene[name].data.root_link_pose_w)[0]
    origin = arr(base.scene.env_origins)[0]
    return pose[:3] - origin, pose[3:7]


def lid_gap(base):
    asset = base.scene["tote_tall"]
    return float(arr(asset.data.joint_pos)[0][list(asset.joint_names).index("lid_lift")])


def tote_gaps(base):
    return {name:float(arr(base.scene[name].data.joint_pos)[0][
        list(base.scene[name].joint_names).index("lid_lift")])
        for name in ("tote_tall","tote_low")}


def apply_cords(base):
    """Every physics substep: update cord routes, tension and failures, write cord wrenches."""
    import torch
    dev = base.device
    gap = lid_gap(base)
    lid_total = np.zeros(3)
    dt = float(base.physics_dt)
    forces, contacts = sensor_channels(base, arr)
    _GUARDS.sample(forces, contacts, dt)
    for i, cord in _CORDS.items():
        pa, qa = _hook_pose(base, f"strap_{i + 1}_a")
        pb, qb = _hook_pose(base, f"strap_{i + 1}_b")
        aa, ab = strap.hook_attach_world(pa, qa), strap.hook_attach_world(pb, qb)
        cord.update_route(aa, ab, gap)
        failure = cord.route_guard.check(cord.path(aa, ab, gap), tote_gaps(base))
        cord.failure = cord.failure or failure
        T, fa, fb, fl = cord.forces(aa, ab, gap, dt)
        if _VISUALS is not None and _GUARDS.substeps % 12 == 0:
            origin = arr(base.scene.env_origins)[0]
            _VISUALS.update(i, [p + origin for p in cord.path(aa, ab, gap)])
        if T > geo.HOOK_RATING_N and cord.failure is None:
            loaded = [strap.seat_check(p, q, 0.0, _FRAME) for p, q in ((pa, qa), (pb, qb))]
            if any(x is not None for x in loaded):
                cord.failure = "hook_bent_open"
        lid_total += fl
        for name, (p, q, f) in ((f"strap_{i + 1}_a", (pa, qa, fa)), (f"strap_{i + 1}_b", (pb, qb, fb))):
            F, tau = strap.hook_wrench(p, q, f)
            comp = base.scene[name].permanent_wrench_composer
            comp.reset()
            if T > 0:
                comp.add_forces_and_torques(torch.tensor(F, dtype=torch.float32, device=dev).view(1, 1, 3),
                                            torch.tensor(tau, dtype=torch.float32, device=dev).view(1, 1, 3),
                                            body_ids=[0], is_global=True)
    lid_total[2] -= DIAGNOSTIC_LID_LOAD_N
    tote = base.scene["tote_tall"]
    comp = tote.permanent_wrench_composer
    comp.reset()
    if np.linalg.norm(lid_total) > 0:
        comp.add_forces_and_torques(torch.tensor(lid_total, dtype=torch.float32, device=dev).view(1, 1, 3),
                                    torch.zeros((1, 1, 3), device=dev), body_ids=tote.find_bodies("lid")[0],
                                    is_global=True)
    return lid_total


def _gripper_points(base):
    pts = []
    for side in ("left", "right"):
        robot = base.scene[side + "_robot"]
        names = list(robot.body_names)
        idx = [k for k, n in enumerate(names) if any(t in n.lower() for t in ("finger", "tip", "gripper", "tcp", "link_6"))]
        if not idx:
            idx = list(range(len(names)))
        pos = arr(robot.data.body_pos_w)[0] - arr(base.scene.env_origins)[0]
        pts += [pos[k] for k in idx]
    return np.array(pts)



def configure_passive(base):
    import torch
    for name, spec in _SPECS.items():
        if isinstance(spec, Tote):
            asset = base.scene[name]
            q = getattr(asset.data.joint_pos, "torch", asset.data.joint_pos)
            targets = torch.full_like(q, float(geo.tote(name)["qref"]))
            asset.set_joint_position_target(targets)


def measure(emb, step, advance=False):
    """advance=True only from the per-control-step task hook; other callers reuse the last speeds."""
    base = emb._base
    origin = arr(base.scene.env_origins)[0]
    row = dict(step=step, time_s=step / emb.info.control_hz, totes={}, hooks={}, numerical_warning=False)
    for name, spec in _SPECS.items():
        asset = base.scene[name]
        if isinstance(spec, Tote):
            names = list(asset.joint_names)
            q = float(arr(asset.data.joint_pos)[0][names.index("lid_lift")])
            v = float(arr(asset.data.joint_vel)[0][names.index("lid_lift")])
            t = geo.tote(name)
            row["totes"][name] = dict(lid_gap_m=q, lid_vel_mps=v,
                                      spring_force_n=t["kc"] * (t["qref"] - q))
            row["numerical_warning"] |= not (math.isfinite(q) and math.isfinite(v))
        elif isinstance(spec, Hook):
            pose = arr(asset.data.root_link_pose_w)[0]
            vel = arr(asset.data.root_vel_w)[0]
            p = pose[:3] - origin
            prev = _PREV_HOOK_POS.get(name)
            # Displacement speed over the 20 Hz control step. PhysX's reported root velocity of a hook resting
            # under cord load chatters (~5 cm/s at 240 Hz contact) while its position is steady to 0.1 mm, so
            # stillness/snap-back use the displacement speed; the raw velocity is kept for diagnostics.
            if advance:
                dspeed = 0.0 if prev is None else float(np.linalg.norm(p - prev) * emb.info.control_hz)
                _PREV_HOOK_POS[name] = p.copy()
                _LAST_SPEED[name] = dspeed
            else:
                dspeed = _LAST_SPEED.get(name, 0.0)
            row["hooks"][name] = dict(pos=p.tolist(), quat_xyzw=pose[3:7].tolist(), speed_mps=dspeed,
                                      physx_vel_mps=float(np.linalg.norm(vel[:3])))
            row["numerical_warning"] |= not bool(np.isfinite(np.r_[pose, vel]).all())
    if _FRAME is not None and _CORDS:
        hands = _gripper_points(base)
        gap = row["totes"]["tote_tall"]["lid_gap_m"]
        lid_local = np.array([_FRAME.local(h) for h in hands])
        lid_z = _FRAME.lid_shut_z + gap
        d = np.maximum(np.abs(lid_local - np.array([0, 0, lid_z])) - np.array([_FRAME.hx, _FRAME.hy, geo.LID_T]), 0)
        row["hands_off_lid"] = _GUARDS.hooks_off("lid")
        row["straps"] = {}
        for i, cord in _CORDS.items():
            s = dict(route=cord.route, tension_n=cord.tension, max_tension_n=cord.max_tension, length_m=cord.length,
                     relaxed_m=cord.L0, failure=cord.failure, route_guard_detail=cord.route_guard.detail,
                     route_failure_sample=cord.route_guard.failure_sample)
            off = True
            for e in ("a", "b"):
                h = row["hooks"][f"strap_{i + 1}_{e}"]
                near = float(np.linalg.norm(hands - np.array(h["pos"]), axis=1).min())
                off &= _GUARDS.hooks_off(f"strap_{i + 1}_{e}")
                s[f"seated_{e}"] = strap.seat_check(h["pos"], h["quat_xyzw"], h["speed_mps"], _FRAME)
                if _GUARDS.hooks_off(f"strap_{i + 1}_{e}") and cord.tension > 0 and h["speed_mps"] > SNAPBACK_MPS and cord.failure is None:
                    cord.failure = "hook_snapback"
                    s["failure"] = cord.failure
            s["hands_off"] = off
            row["straps"][f"strap_{i + 1}"] = s
    joints = {}
    for side in ("left", "right"):
        robot = base.scene[side + "_robot"]
        joints[side] = arr(robot.data.joint_pos)[0].astype(float).tolist()
        row["numerical_warning"] |= not bool(np.isfinite(joints[side]).all())
    if os.environ.get("D1R_DIAGNOSTIC_ROBOT_STATE") == "1":
        row["robot_joint_pos"] = joints
    row.update(_GUARDS.fields())
    if advance:
        _GUARDS.finish_control()
    return row


def current_score():
    return 0.0 if _SCORER is None else _SCORER.score


# -- yam_arena hooks ------------------------------------------------------------------------

def patch_scene(scene):
    if getattr(scene, "_d1r_patched", False):
        return
    old_object, old_states = scene.task_object, scene.object_states
    old_build = scene.build_task_environment
    old_room = scene.task_room

    def task_object(spec):
        if isinstance(spec, (Tote, StrapTray, Hook)):
            _SPECS[spec.name] = spec
            return arena_object(spec)
        return old_object(spec)

    def states(env, names):
        mine = [n for n in names if n in _SPECS]
        out = old_states(env, [n for n in names if n not in _SPECS]) if len(mine) < len(names) else {}
        base = env.unwrapped
        origin = arr(base.scene.env_origins)[0]
        for n in mine:
            spec = _SPECS[n]
            if isinstance(spec, StrapTray):
                p = np.asarray(spec.pos, float)
                out[n] = ObjectState(pos=p, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3),
                                     ang_vel=np.zeros(3), low=p - np.array(spec.size) / 2, high=p + np.array(spec.size) / 2)
                continue
            asset = base.scene[n]
            pose = arr(asset.data.root_link_pose_w)[0]
            vel = arr(asset.data.root_vel_w)[0] if isinstance(spec, Hook) else np.zeros(6)
            p = pose[:3] - origin
            half = np.array(spec.size) / 2
            out[n] = ObjectState(pos=p, quat_xyzw=pose[3:7], lin_vel=vel[:3], ang_vel=vel[3:],
                                 low=p - half, high=p + half)
        return out

    def build(embodiment, task, control_hz=None):
        is_raised = embodiment.rig.name.endswith("-d1r-riser20")
        if is_raised:
            # Translate the entire camera stand, rather than stretching its mast.
            # Physical supports occupy the same0.20m elevation as the native port.
            from dataclasses import replace as dc_replace
            from rigging import RISE, up
            from yam_arena import station
            def room(scene_name, top_cam):
                if scene_name != "station":
                    return old_room(scene_name, top_cam)
                unraised = dc_replace(top_cam,pos=(top_cam.pos[0],top_cam.pos[1],top_cam.pos[2]-RISE))
                parts=station.camera_stand(unraised)
                assets=list(scene._station_room())
                for name,(kind,size,pos,rot) in parts.items():
                    assets.append(scene._stand_part(name,kind,size,up(pos),rot))
                for side,pos in embodiment.rig.bases.items():
                    assets.append(scene._box("mount_riser_"+side,(.16,.16,RISE),
                        (pos[0],pos[1],geo.TABLE_TOP+RISE/2),(.55,.57,.59)))
                foot=parts["stand_foot"][2]
                assets.append(scene._box("mount_riser_camera",(.11,.11,RISE),
                    (foot[0],foot[1],geo.TABLE_TOP+RISE/2),(.55,.57,.59)))
                return assets
            scene.task_room=room
        try:
            env = old_build(embodiment, task, control_hz)
        finally:
            scene.task_room=old_room
        if any(isinstance(o, Tote) for o in task.objects):
            previous = env.env_cfg_callback
            def configure(cfg):
                cfg = previous(cfg)
                # Preserve the frozen YAM adapter's 22 N per-finger constraint.
                # Task-local config only; SDK/robot source remains unchanged.
                for side in ("left", "right"):
                    getattr(cfg.scene, side+"_robot").actuators["gripper"].joint_effort_limit = 22.
                from isaaclab.sensors import JointWrenchSensorCfg, ContactSensorCfg
                for side, prim in (("left", "LeftArm"), ("right", "RightArm")):
                    setattr(cfg.scene, side+"_d1r_wrench", JointWrenchSensorCfg(prim_path="{ENV_REGEX_NS}/"+prim))
                from yam_arena.robot import asset_link_paths
                # PhysX requires each filter expression to resolve one rigid body per environment.
                link_paths = asset_link_paths()
                robot_filters = ["{ENV_REGEX_NS}/"+arm+path for arm in ("LeftArm","RightArm")
                                 for path in link_paths.values()]
                trace_path = os.environ.get("D1R_TRACE")
                if trace_path:
                    Path(trace_path).with_suffix(".sensor-layout.json").write_text(json.dumps(
                        dict(robot_link_paths=link_paths, filters=robot_filters),indent=2))
                for name in ("lid", *tuple(f"strap_{i}_{e}" for i in range(1,5) for e in ("a","b"))):
                    path = "tote_tall/lid" if name=="lid" else name
                    setattr(cfg.scene, "d1r_contact_"+name, ContactSensorCfg(
                        prim_path="{ENV_REGEX_NS}/"+path, update_period=0., history_length=0,
                        track_contact_points=True, max_contact_data_count_per_prim=128,
                        filter_prim_paths_expr=robot_filters))
                return cfg
            env.env_cfg_callback = configure
        return env
    scene.build_task_environment = build
    scene.task_object, scene.object_states = task_object, states
    scene._d1r_patched = True


def patch_embodiment(module):
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_d1r_patched", False):
        return
    old_reset, old_task = cls.reset, cls._task_step
    old_ensure = cls._ensure_sim

    def active(self):
        return self.task is not None and any(isinstance(o, Tote) for o in self.task.objects)

    def ensure_sim(self, goal):
        if active(self):
            from rigging import raised_rig
            raised=raised_rig(self.rig)
            if raised is not self.rig:
                self.rig=raised
                env_id=module.environment_id(self.rig.name,self.task)
                self.info=replace(self.info,environment_id=env_id)
                module.task_file.register(env_id,self.task)
        return old_ensure(self,goal)

    def reset(self, *args, **kwargs):
        global _SCORER, _TRACE, _LAST_CONTROL_ROW
        _LAST_CONTROL_ROW = None
        if not active(self):
            return old_reset(self, *args, **kwargs)
        if not math.isclose(float(self.info.control_hz), 20.0):
            raise ValueError("The D1R pilot requires 20 Hz: use -E control_hz=20")
        self._ensure_sim(None)
        for side in ("left","right"):
            self._base.scene[side+"_robot"]._d1r_total_effort_enabled=True
            self._base.scene[side+"_robot"]._d1r_total_effort_checked=False
        for name, spec in _SPECS.items():
            if isinstance(spec, Tote):
                asset = self._base.scene[name]
                if not getattr(asset, "_d1r_callback", False):
                    previous = asset.write_data_to_sim

                    def write(*a, _prev=previous, _name=name, **k):
                        configure_passive(self._base)
                        if _name == "tote_tall" and _CORDS:
                            apply_cords(self._base)
                        return _prev(*a, **k)
                    asset.write_data_to_sim = write
                    asset._d1r_callback = True
        global _FRAME, _VISUALS
        _PREV_HOOK_POS.clear()
        _LAST_SPEED.clear()
        _CORDS.clear()  # no cord forces while the scene resets; clear wrenches left from a previous episode
        for name, spec in _SPECS.items():
            if isinstance(spec, (Hook, Tote)):
                self._base.scene[name].permanent_wrench_composer.reset()
        # Build visual-only free cord paths before the first camera observation.
        _VISUALS = CordVisuals()
        nominal = {h["name"]:h for h in geo.hooks()}
        origin = arr(self._base.scene.env_origins)[0]
        for i in range(4):
            a, b = nominal[f"strap_{i+1}_a"], nominal[f"strap_{i+1}_b"]
            _VISUALS.update(i, [strap.hook_attach_world(a["pos"],a["quat_xyzw"])+origin,
                                strap.hook_attach_world(b["pos"],b["quat_xyzw"])+origin])
        obs = old_reset(self, *args, **kwargs)
        configure_passive(self._base)
        _FRAME = strap.ToteFrame("tote_tall")
        for i in range(4):
            _CORDS[i] = strap.Cord(i, _FRAME)
        _GUARDS.reset()
        _VISUALS = CordVisuals()
        for i, cord in _CORDS.items():
            pa, qa = _hook_pose(self._base, f"strap_{i + 1}_a")
            pb, qb = _hook_pose(self._base, f"strap_{i + 1}_b")
            origin = arr(self._base.scene.env_origins)[0]
            _VISUALS.update(i, [p + origin for p in cord.path(strap.hook_attach_world(pa,qa), strap.hook_attach_world(pb,qb), lid_gap(self._base))])
        _SCORER = Scorer(control_hz=20.0)
        trace = os.environ.get("D1R_TRACE")
        if _TRACE is not None:
            _TRACE.close()
        _TRACE = open(trace, "w", buffering=1) if trace else None
        return obs

    def task_step(self):
        if not active(self):
            return old_task(self)
        global _LAST_CONTROL_ROW
        # Include the just-completed final physics substep before scoring.
        forces, contacts = sensor_channels(self._base, arr)
        _GUARDS.sample(forces, contacts, 0.0)
        gap=lid_gap(self._base)
        for i,cord in _CORDS.items():
            pa,qa=_hook_pose(self._base,f"strap_{i+1}_a")
            pb,qb=_hook_pose(self._base,f"strap_{i+1}_b")
            aa,ab=strap.hook_attach_world(pa,qa),strap.hook_attach_world(pb,qb)
            cord.update_route(aa,ab,gap)
            cord.failure=cord.failure or cord.route_guard.check(cord.path(aa,ab,gap),tote_gaps(self._base))
        row = measure(self, _SCORER.steps + 1, advance=True)
        _LAST_CONTROL_ROW = row
        _SCORER.update(row)
        if _TRACE is not None:
            _TRACE.write(json.dumps(row) + "\n")
        result = old_task(self)
        info = dict(result.info, d1r_score_gate=_SCORER.gate, d1r_failure=_SCORER.failure,
                    d1r_score=_SCORER.score)
        if _SCORER.failure:
            return replace(result, terminated=True, termination_reason=_SCORER.failure, info=info)
        return replace(result, info=info)

    cls.reset, cls._task_step, cls._ensure_sim = reset, task_step, ensure_sim
    cls._d1r_patched = True


from motor_limits import patch_actions as patch_motor_actions


class Finder(importlib.abc.MetaPathFinder):
    targets = {"yam_arena.scene": patch_scene, "yam_arena.inspect_embodiment": patch_embodiment,
               "yam_arena.actions": patch_motor_actions}

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
        execute, patch = spec.loader.exec_module, self.targets[fullname]

        def exec_module(module):
            execute(module)
            patch(module)
        spec.loader.exec_module = exec_module
        return spec


def install():
    for name, patch in Finder.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, Finder())

_LAST_CONTROL_ROW = None
