"""panelkit: the wall-mounted load centre (tub, breakers, two started lower cover screws), its dead-front cover, the
cover stand and the ratcheting nut driver for yam_arena (Isaac Lab-Arena / PhysX), plus the runtime hooks.

Authored as one custom-spawned assembly (yam_arena task-file primitives are single boxes/spheres):
* kinematic bodies (so PhysX contact views can report the forces on them): the wall stub and tub ("tub"), the six
  breakers' bodies and handle shrouds ("breakers"), the six toggle handles (ON or OFF per trial), the two lower cover
  screws, the cover stand and the driver holster;
* the dead-front cover (16 gauge steel, 0.91 kg) and the nut driver as free rigid bodies.
Geometry: ``../src/layout.py``; scorer: ``../src/scoring.py`` (pure functions).

Screw model (the only rule-based part): each started screw is a kinematic body on its fixed axis. While the driver's
socket is over its hex head (axes within 8 deg and 1 mm, hex at least 1.5 mm inside the socket), the screw turns with
the socket in the tightening direction (the driver ratchets: turning back does nothing) and advances 0.794 mm per turn
(10-32 thread). It stops when its head reaches the cover's front face (0.1 mm interference, so the head clamps the
cover against the tub through contact) or, without a cover under it, the tub face. Nothing here writes the cover's
or the driver's pose or velocity during a trial; ``Runtime.restore_authored`` resets them only on an environment reset.
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
from scoring import measure, Monitor, Scorer  # noqa: E402
from yam_arena.task_file import Box  # noqa: E402

PHYSICS_DT = float(os.environ.get("PANEL_PHYSICS_DT", str(1 / 240)))
CONTACT_OFFSET = float(os.environ.get("PANEL_CONTACT_OFFSET", ".0002"))
SETUP_BASE = int(os.environ.get("PANEL_SETUP", "0"))  # named setup of the first unseeded trial; later trials cycle
FORCE_NAMED = "PANEL_SETUP" in os.environ
DEBUG = bool(os.environ.get("PANEL_DEBUG"))
PUSH_AHEAD = 0.00015  # how far the head may run into a cover that stands off the tub (it pushes it back)
STALL_TURN = math.pi / 2  # tightening input absorbed with the head stalled on a held-off cover before it counts as down

STEEL_GREY, TUB_GREY, PLY, BLACK, DARK, WHITE, ZINC, RED = (
    (0.80, 0.81, 0.80), (0.55, 0.57, 0.58), (0.78, 0.66, 0.48), (0.06, 0.06, 0.07), (0.16, 0.16, 0.18),
    (0.95, 0.95, 0.93), (0.74, 0.76, 0.78), (0.75, 0.12, 0.10))
HANDLES = tuple(f"h{i}" for i in range(6))
KINEMATIC = ("tub", "breakers") + HANDLES + ("screw_l", "screw_r", "stand", "holster")
FREE = ("cover", "driver")
BODIES = FREE + KINEMATIC
VIEW = ("cover", "driver", "tub", "breakers", "screw_l", "screw_r", "stand") + HANDLES
COVER_FILTERS = ("tub", "breakers", "screw_l", "screw_r", "stand", "driver") + HANDLES
FRICTION = dict(steel=0.35, hem=0.8, rubber=0.8)  # hem: steel against the rubber finger pads (combine mode max)


class Panel(Box):
    """The bench, as one task-file object (subclasses Box only to pass yam_arena's checks)."""


def _rot(q_xyzw):
    x, y, z, w = q_xyzw
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def _quat(r):
    from scipy.spatial.transform import Rotation

    q = Rotation.from_matrix(np.asarray(r, float)).as_quat()  # xyzw
    return q / np.linalg.norm(q)


def handle_pose(i, on, setup):
    """Toggle handle i: body frame at the shroud's front-face centre, tilted 20 deg toward ON (outboard) or OFF."""
    c = L.space_center(i)
    col = L.SPACES[i][0]
    outboard = 1.0 if col == "L" else -1.0
    sgn = outboard if on else -outboard
    pos = L.to_station(np.array([L.FACE_FRONT, c[1], c[2]]), setup)
    return pos, L.rz(-20.0 * sgn)


def screw_pose(i, advance, angle, setup):
    """Screw body frame: on the axis at the tub face, translated by the advance. The body is not turned with the screw
    angle: a kinematic shank spinning under the slot top would drag the cover sideways by friction at a rate no real
    screw thread produces (the angle is kept in the state and logged)."""
    del angle
    return L.to_station(np.array([advance, L.SCREW_Y[i], L.SCREW_Z]), setup), np.eye(3)


def authored(setup):
    """Authored (reset) pose of every moving body for a setup: {name: (pos, rot)} in the env frame."""
    sp = L.setup_spec(setup)
    po = L.panel_origin(setup)
    out = {
        "cover": L.cover_stand_pose(setup),
        "driver": L.driver_holster_pose(),
        "tub": (po.copy(), np.eye(3)),
        "breakers": (po.copy(), np.eye(3)),
        "screw_l": screw_pose(0, 0.0, 0.0, setup),
        "screw_r": screw_pose(1, 0.0, 0.0, setup),
        "stand": (L.stand_pos(setup), L.ry(L.LEAN)),
        "holster": (L.DRIVER_HOLSTER.copy(), np.eye(3)),
    }
    for i in range(6):
        out[f"h{i}"] = handle_pose(i, sp["on"][i], setup)
    return out


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
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.45)
            vis.func(path, vis)
            looks[color] = path
        return looks[color]

    def material(friction, combine=None):
        key = (friction, combine)
        if key not in mats:
            path = f"{prim_path}/Looks/phys{len(mats)}"
            pm = sim_utils.RigidBodyMaterialCfg(static_friction=friction, dynamic_friction=friction)
            pm.func(path, pm)
            if combine:
                PhysxSchema.PhysxMaterialAPI.Apply(stage.GetPrimAtPath(path)).CreateFrictionCombineModeAttr(combine)
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

    def hull(path, v, color, friction=FRICTION["steel"], collide=True, combine=None):
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
            bind_physics_material(str(prim.GetPath()), material(friction, combine), stage=stage)
            count["n"] += 1
        bind_visual_material(str(prim.GetPath()), look(color), stage=stage)
        return m

    def box(path, lo, hi, color, friction=FRICTION["steel"], rot=None, collide=True, combine=None):
        lo, hi = np.asarray(lo, float), np.asarray(hi, float)
        corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
        if rot is not None:
            corners = corners @ np.asarray(rot).T
        return hull(path, corners, color, friction, collide, combine)

    s0 = SETUP_BASE
    A = authored(s0)
    # ---------------------------------------------------------------- tub + wall stub (kinematic, frame = panel P)
    t = f"{prim_path}/tub"
    xform(t, *A["tub"])
    rigid(t, 5.0, kinematic=True)
    hw, hh, d, w = L.TUB_W / 2, L.TUB_H / 2, L.TUB_D, L.TUB_WALL
    zb = L.TOP + 0.001 - L.P0[2]  # table top in the panel frame (nominal; the stub stands on the table)
    box(f"{t}/wall", (d, -L.WALL_W / 2, zb), (d + L.WALL_T, L.WALL_W / 2, zb + L.WALL_H), PLY)
    box(f"{t}/back", (d - w, -hw, -hh), (d, hw, hh), TUB_GREY)
    box(f"{t}/side_l", (0.0, hw - w, -hh), (d, hw, hh), TUB_GREY)
    box(f"{t}/side_r", (0.0, -hw, -hh), (d, -hw + w, hh), TUB_GREY)
    box(f"{t}/top", (0.0, -hw, hh - w), (d, hw, hh), TUB_GREY)
    box(f"{t}/bottom", (0.0, -hw, -hh), (d, hw, -hh + w), TUB_GREY)
    for name, z0 in (("flange_b", -hh), ("flange_t", hh - L.FLANGE_H)):
        # front flanges, split around the screw holes (bottom) so the shanks pass
        if name == "flange_b":
            ys = [-hw, L.SCREW_Y[1] - 0.003, L.SCREW_Y[1] + 0.003, L.SCREW_Y[0] - 0.003, L.SCREW_Y[0] + 0.003, hw]
            for k in range(0, 6, 2):
                box(f"{t}/{name}_{k}", (0.0, ys[k], z0), (L.FLANGE_T, ys[k + 1], z0 + L.FLANGE_H), TUB_GREY)
            for k, (ya, yb) in enumerate(((ys[1], ys[2]), (ys[3], ys[4]))):
                box(f"{t}/{name}_hb{k}", (0.0, ya, z0), (L.FLANGE_T, yb, L.SCREW_Z - 0.003), TUB_GREY)
                box(f"{t}/{name}_ht{k}", (0.0, ya, L.SCREW_Z + 0.003), (L.FLANGE_T, yb, z0 + L.FLANGE_H), TUB_GREY)
        else:
            box(f"{t}/{name}", (0.0, -hw, z0), (L.FLANGE_T, hw, z0 + L.FLANGE_H), TUB_GREY)
    # interior (visual only): mounting pan, main lugs, neutral bar
    box(f"{t}/pan", (0.030, -0.085, -0.120), (d - w, 0.085, 0.120), DARK, collide=False)
    for k, y in enumerate((0.035, -0.035)):
        box(f"{t}/lug{k}", (0.030, y - 0.012, 0.122), (0.060, y + 0.012, 0.134), ZINC, collide=False)
    box(f"{t}/neutral", (0.030, -0.070, -0.105), (0.060, 0.070, -0.095), ZINC, collide=False)
    # ---------------------------------------------------------------- breakers (kinematic, frame = panel P)
    b = f"{prim_path}/breakers"
    xform(b, *A["breakers"])
    rigid(b, 2.0, kinematic=True)
    for i in range(6):
        c = L.space_center(i)
        col = L.SPACES[i][0]
        sgn = 1.0 if col == "L" else -1.0
        y_in, y_out = (c[1] - L.FACE_W / 2, c[1] - L.FACE_W / 2 + sgn * L.BODY_W)
        ylo, yhi = min(c[1] - sgn * L.FACE_W / 2, c[1] - sgn * L.FACE_W / 2 + sgn * L.BODY_W), \
            max(c[1] - sgn * L.FACE_W / 2, c[1] - sgn * L.FACE_W / 2 + sgn * L.BODY_W)
        box(f"{b}/body{i}", (L.FACE_BACK, ylo, c[2] - L.SPACE_PITCH / 2 + 0.0004),
            (L.BODY_BACK, yhi, c[2] + L.SPACE_PITCH / 2 - 0.0004), BLACK)
        box(f"{b}/face{i}", (L.FACE_FRONT, c[1] - L.FACE_W / 2, c[2] - L.FACE_H / 2),
            (L.FACE_BACK, c[1] + L.FACE_W / 2, c[2] + L.FACE_H / 2), BLACK)
        # amp rating label (visual)
        box(f"{b}/label{i}", (L.FACE_FRONT - 0.0003, c[1] - sgn * 0.016 - 0.004, c[2] - 0.004),
            (L.FACE_FRONT, c[1] - sgn * 0.016 + 0.004, c[2] + 0.004), WHITE, collide=False)
        del y_in, y_out
    # ---------------------------------------------------------------- toggle handles (kinematic, one body each)
    hx, hz, hd = L.HANDLE[0], L.HANDLE[1], L.HANDLE[2]
    for i in range(6):
        h = f"{prim_path}/h{i}"
        xform(h, *A[f"h{i}"])
        rigid(h, 0.01, kinematic=True)
        box(f"{h}/lever", (-hd, -hx / 2, -hz / 2), (0.0005, hx / 2, hz / 2), BLACK)
    # ---------------------------------------------------------------- lower cover screws (kinematic)
    for k, name in enumerate(("screw_l", "screw_r")):
        s = f"{prim_path}/{name}"
        xform(s, *A[name])
        rigid(s, 0.005, kinematic=True)
        hu = L.head_under_x(0.0)
        cyl = L.cylinder(L.SHANK_R, 0.0, 1.0, n=16)
        cyl[:, [0, 1, 2]] = cyl[:, [2, 0, 1]]  # axis along x
        shank = cyl.copy()
        shank[:16, 0] = 0.012
        shank[16:, 0] = hu
        hull(f"{s}/shank", shank, ZINC)
        wash = L.cylinder(L.HEAD_R, 0.0, 1.0, n=24)[:, [2, 0, 1]]
        wash[:24, 0] = hu
        wash[24:, 0] = hu - L.HEAD_WASHER_T
        hull(f"{s}/washer", wash, ZINC)
        hexp = L.cylinder(L.HEAD_HEX_R, 0.0, 1.0, n=6)[:, [2, 0, 1]]
        hexp[:6, 0] = hu - L.HEAD_WASHER_T
        hexp[6:, 0] = hu - L.HEAD_WASHER_T - L.HEAD_HEX_T
        hull(f"{s}/hex", hexp, ZINC)
    # ---------------------------------------------------------------- cover stand (kinematic, frame S)
    st = f"{prim_path}/stand"
    xform(st, *A["stand"])
    rigid(st, 2.0, kinematic=True)
    bx, by, bz = L.STAND_BASE
    gw, gd, wt = L.GROOVE_W, L.GROOVE_D, L.GROOVE_WALL_T
    box(f"{st}/floor", (-bx / 2, -by / 2, -bz), (bx / 2, by / 2, 0.0), PLY, FRICTION["rubber"])
    box(f"{st}/front", (-gw / 2 - wt, -by / 2, 0.0), (-gw / 2, by / 2, gd), PLY, FRICTION["rubber"])
    box(f"{st}/rear", (gw / 2, -by / 2, 0.0), (gw / 2 + wt, by / 2, gd), PLY, FRICTION["rubber"])
    box(f"{st}/backrest", (gw / 2, -L.BACKREST_Y, gd), (gw / 2 + wt, L.BACKREST_Y, L.BACKREST_H), PLY,
        FRICTION["rubber"])
    # ---------------------------------------------------------------- driver holster (kinematic)
    ho = f"{prim_path}/holster"
    xform(ho, *A["holster"])
    rigid(ho, 1.0, kinematic=True)
    sx, sy, sz = L.HOLSTER_SIZE
    hh_ = L.HOLSTER_HOLE / 2
    box(f"{ho}/a", (-sx / 2, -sy / 2, 0.0), (-hh_, sy / 2, sz), DARK)
    box(f"{ho}/b", (hh_, -sy / 2, 0.0), (sx / 2, sy / 2, sz), DARK)
    box(f"{ho}/c", (-hh_, -sy / 2, 0.0), (hh_, -hh_, sz), DARK)
    box(f"{ho}/d", (-hh_, hh_, 0.0), (hh_, sy / 2, sz), DARK)
    # ---------------------------------------------------------------- dead-front cover (free body, frame V)
    cv = f"{prim_path}/cover"
    xform(cv, *A["cover"])
    m, com, inertia = L.cover_mass_props()
    rigid(cv, m, inertia=inertia, com=com)
    for i, (y0, y1, z0, z1) in enumerate(L.cover_boxes()):
        edge = abs((y0 + y1) / 2) > L.COVER_W / 2 - L.HEM_W  # the hemmed edge strips: where the hands pinch
        box(f"{cv}/p{i:02d}", (-L.COVER_T, y0, z0), (0.0, y1, z1), STEEL_GREY,
            FRICTION["hem"] if edge else FRICTION["steel"], combine="max" if edge else None)
    for i, (y0, y1, z0, z1) in enumerate(L.hem_boxes()):
        box(f"{cv}/hem{i}", (0.0, y0, z0), (L.COVER_T, y1, z1), STEEL_GREY, FRICTION["hem"], combine="max")
    # circuit directory label and screw marks (visual only)
    box(f"{cv}/label", (-L.COVER_T - 0.0003, -0.06, 0.09), (-L.COVER_T, 0.06, 0.15), WHITE, collide=False)
    box(f"{cv}/brand", (-L.COVER_T - 0.0003, -0.03, -0.12), (-L.COVER_T, 0.03, -0.10), RED, collide=False)
    # ---------------------------------------------------------------- nut driver (free body, frame D)
    dv = f"{prim_path}/driver"
    xform(dv, *A["driver"])
    zs0, zs1 = 0.0, L.DRV_SOCKET_D
    zh0 = L.DRV_HANDLE_Z0
    zh1 = zh0 + L.DRV_HANDLE_L
    # mass: handle 0.12, shaft+socket 0.04 (COM in the handle)
    rigid(dv, L.DRV_MASS, inertia=(1.6e-4, 1.6e-4, 2.0e-5), com=(0.0, 0.0, zh0 + 0.040))
    zl = zs0 + L.DRV_LEADIN[1]
    for k, v in enumerate(L.ring_pieces(L.DRV_SOCKET_R[0], L.DRV_SOCKET_R[1], zs0, zl, n=12, r0_at_z0=L.DRV_LEADIN[0])):
        hull(f"{dv}/lead{k:02d}", v, ZINC)  # chamfered mouth (lead-in)
    for k, v in enumerate(L.ring_pieces(L.DRV_SOCKET_R[0], L.DRV_SOCKET_R[1], zl, zs1, n=12)):
        hull(f"{dv}/sock{k:02d}", v, ZINC)
    hull(f"{dv}/cap", L.cylinder(L.DRV_SOCKET_R[1], zs1, zs1 + 0.002, n=16), ZINC)
    hull(f"{dv}/shaft", L.cylinder(L.DRV_SHAFT_R, zs1 + 0.002, zh0, n=16), ZINC)
    hull(f"{dv}/handle", L.cylinder(L.DRV_HANDLE_R, zh0, zh1, n=20), RED, FRICTION["rubber"])
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

    return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=BenchSpawnerCfg(),
                  initial_pose=Pose(position_xyz=(0.0, 0.0, 0.0), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


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
            list(robot.body_names).index("gripper")]
        jp = robot.data.joint_pos[0].detach().cpu().numpy().astype(float)
        jn = list(robot.joint_names)
        half = max(abs(jp[jn.index(j)]) for j in ("joint7", "joint8"))
        r = _rot(pose[3:7])
        tip = pose[:3] + r @ (np.array(TCP_OFFSET) + [0, 0, PAD_TIP_PAST_TCP]) - origin
        pts += [tip + r[:, 0] * s * (half + 0.0032) for s in (1.0, -1.0)]
    return pts


def _wrap_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Runtime:
    """Runs the screw model every physics step, reads the contact forces, scores every control step."""

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
        root = "/World/envs/env_0/panel"
        P = lambda n: f"{root}/{n}"  # noqa: E731
        self.view = self.simview.create_rigid_body_view([P(n) for n in VIEW])
        self.idx = {n: i for i, n in enumerate(VIEW)}
        robot_links = []
        for arm in ("left_robot", "right_robot"):
            lp = self.base.scene[arm].root_view.link_paths
            robot_links += list(lp[0] if lp and isinstance(lp[0], (list, tuple)) else lp)
        self.robot_links = robot_links
        self.cv_cover = self.simview.create_rigid_contact_view(
            [P("cover")], filter_patterns=[[P(n) for n in COVER_FILTERS] + robot_links], max_contact_data_count=8192)
        self.cv_screws = self.simview.create_rigid_contact_view(
            [P("screw_l"), P("screw_r")], filter_patterns=[[P("cover"), P("driver")], [P("cover"), P("driver")]],
            max_contact_data_count=2048)
        self.n_filters = len(COVER_FILTERS)
        self.origin = self.base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        self.setup = SETUP_BASE
        self.trial_active = False
        self._init_state()

    def _init_state(self):
        self.monitor = Monitor()
        self.scorer = Scorer()
        self.steps = 0
        self.t = 0.0
        self.adv = [0.0, 0.0]
        self.ang = [0.0, 0.0]
        self.turned = [0.0, 0.0]  # tightening rotation applied (rad)
        self.engaged = [False, False]
        self.engaged_s = [0.0, 0.0]
        self.seated = [False, False]
        self.latched = [None, None]  # advance at which the head came down on the cover
        self.stall = [0.0, 0.0]
        self.phi_prev = None
        self.clamp_n = [0.0, 0.0]
        self.hook_s = 0.0
        self.nonfinite = 0
        self.peaks = dict(breaker_n=0.0, handle_n=0.0, robot_on_cover_n=0.0, clamp_n=[0.0, 0.0])
        self.last_contacts = {}
        STATE.update(score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, setup=self.setup)

    def begin_trial(self, trial):
        self._init_state()
        self.trial_active = True
        STATE.update(trial=trial, setup=self.setup, start=L.setup_spec(self.setup))

    # ------------------------------------------------------------------ reading
    def read(self):
        tf = self.view.get_transforms().detach().cpu().numpy().astype(float)
        return {n: (tf[i, :3] - self.origin, _rot(tf[i, 3:7])) for n, i in self.idx.items()}

    def velocity(self, name):
        v = self.view.get_velocities().detach().cpu().numpy().astype(float)
        i = self.idx[name]
        return v[i, :3], v[i, 3:6]

    def panel_origin(self):
        return L.panel_origin(self.setup)

    def screw_axis(self, i):
        return L.to_station(np.array([0.0, L.SCREW_Y[i], L.SCREW_Z]), self.setup)

    def head_front(self, i):
        """Station point at the centre of screw i's hex front face."""
        hu = L.head_under_x(self.adv[i])
        return self.screw_axis(i) + np.array([hu - L.HEAD_WASHER_T - L.HEAD_HEX_T, 0.0, 0.0])

    # ------------------------------------------------------------------ screw model
    def _engagement(self, i, dpos, drot):
        """(engaged, detail): the driver's socket is over screw i's hex."""
        axis_d = -drot[:, 2]  # direction the socket opens toward
        ang = math.degrees(math.acos(float(np.clip(axis_d[0], -1.0, 1.0))))
        a = self.screw_axis(i)
        rel = dpos - a
        radial = math.hypot(rel[1], rel[2])
        mouth_x = rel[0]  # panel-frame x of the socket mouth
        hu = L.head_under_x(self.adv[i])
        hex_front = hu - L.HEAD_WASHER_T - L.HEAD_HEX_T
        inside = mouth_x - hex_front  # hex length inside the socket (m)
        ok = ang < 8.0 and radial < 0.0010 and inside > 0.0015
        return ok, dict(ang=ang, radial_mm=radial * 1e3, inside_mm=inside * 1e3)

    def _stop(self, i, cpos, crot):
        """Advance at which screw i's head stops: on the cover's front face where the cover stands now (if it stands
        off the tub), but never past the front face of a cover seated flat on the tub (so the head cannot push the
        cover into the tub), or on the tub face without a cover under it. Returns (stop, cover under the head)."""
        flat_stop = L.START_GAP + L.SNUG_INTERFERENCE
        a = self.screw_axis(i)
        n = crot[:, 0]
        p_rel = crot.T @ (a + np.array([-L.COVER_T / 2, 0.0, 0.0]) - cpos)
        under = (abs(p_rel[1] - L.SCREW_Y[i]) < L.SLOT_W / 2 + 0.001 and -L.COVER_H / 2 - 0.002 < p_rel[2]
                 < L.SLOT_TOP + 0.004 and abs(p_rel[0] + L.COVER_T / 2) < 0.010)
        if not under or abs(n[0]) < 0.5:
            return L.max_advance(), False
        # front plane: (p - cpos) . n = -COVER_T ; on the axis p = a + s x
        s_front = (-L.COVER_T - (a - cpos) @ n) / n[0]
        if s_front < L.head_under_x(self.adv[i]) - 0.0005:
            return L.max_advance(), False  # the plate is in front of the head (not between head and flange)
        # the head may push a cover that stands off the tub back toward it (by 0.3 mm per step of the solver at most:
        # a cover held off by something else stalls the screw instead of being driven through it)
        return float(min(flat_stop, s_front + PUSH_AHEAD + L.COVER_T + L.START_GAP)), True

    def write_screws(self):
        torch = self.torch
        tf = self.view.get_transforms().clone()
        for i, name in enumerate(("screw_l", "screw_r")):
            pos, rot = screw_pose(i, self.adv[i], self.ang[i], self.setup)
            k = self.idx[name]
            tf[k, :3] = torch.tensor(pos + self.origin, dtype=tf.dtype)
            tf[k, 3:7] = torch.tensor(_quat(rot), dtype=tf.dtype)
        idx = torch.tensor([self.idx["screw_l"], self.idx["screw_r"]], dtype=torch.int32, device=tf.device)
        self.view.set_kinematic_targets(tf, idx)

    def physics(self, dt):
        t0 = time.perf_counter()
        self.t += dt
        if not self.trial_active:
            self.hook_s += time.perf_counter() - t0
            return
        poses = self.read()
        dpos, drot = poses["driver"]
        cpos, crot = poses["cover"]
        phi = math.atan2(drot[2, 0], drot[1, 0])  # driver roll about +x (right-handed about the screw axes)
        dphi = 0.0 if self.phi_prev is None else _wrap_angle(phi - self.phi_prev)
        self.phi_prev = phi
        moved = False
        self.eng_detail = []
        for i in (0, 1):
            ok, det = self._engagement(i, dpos, drot)
            self.eng_detail.append(det)
            self.engaged[i] = ok
            self.engaged_s[i] = self.engaged_s[i] + dt if ok else 0.0
            stop, under = self._stop(i, cpos, crot)
            if self.latched[i] is not None:
                stop = self.latched[i]  # head down on the cover: it turns no further (no torque model)
            turn = L.RATCHET_SIGN * dphi
            if ok and turn > 0.0:
                if self.adv[i] < stop - 1e-7:
                    new = min(stop, self.adv[i] + turn / (2 * math.pi) * L.PITCH)
                    self.ang[i] += (new - self.adv[i]) / L.PITCH * 2 * math.pi * L.RATCHET_SIGN
                    self.turned[i] += turn
                    self.adv[i] = new
                    moved = True
                elif under and self.latched[i] is None:
                    self.stall[i] += turn  # head on a cover that does not move back: the screw stalls
            flat_stop = L.START_GAP + L.SNUG_INTERFERENCE
            if under and self.latched[i] is None and self.turned[i] > 0.0 and (
                    self.adv[i] >= flat_stop - 2e-5 or self.stall[i] >= STALL_TURN):
                self.latched[i] = self.adv[i]
            self.seated[i] = bool(under and self.latched[i] is not None)
        if moved:
            self.write_screws()
        fc = self.cv_cover.get_contact_force_matrix(dt=dt).detach().cpu().numpy()[0]
        fs = self.cv_screws.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
        nf = self.n_filters
        norms = np.linalg.norm(fc, axis=-1)
        breaker_n = float(norms[COVER_FILTERS.index("breakers")])
        handle_n = float(norms[nf - 6:nf].max())
        robot_on_cover = float(norms[nf:].max()) if norms.shape[0] > nf else 0.0
        support = float(norms[:nf].sum())
        self.clamp_n = [float(np.linalg.norm(fs[k, 0])) for k in (0, 1)]
        vz = float(self.view.get_velocities().detach().cpu().numpy()[self.idx["cover"], 2])
        self.monitor.step(dt, vz, robot_on_cover, support > 0.5)
        pk = self.peaks
        pk["breaker_n"] = max(pk["breaker_n"], breaker_n)
        pk["handle_n"] = max(pk["handle_n"], handle_n)
        pk["robot_on_cover_n"] = max(pk["robot_on_cover_n"], robot_on_cover)
        pk["clamp_n"] = [max(a, b) for a, b in zip(pk["clamp_n"], self.clamp_n)]
        self.last_contacts = dict(
            by={n: round(float(norms[k]), 3) for k, n in enumerate(COVER_FILTERS) if norms[k] > 0.0},
            robot_on_cover_n=round(robot_on_cover, 3), clamp_n=[round(v, 3) for v in self.clamp_n],
            driver_on_screw_n=[round(float(np.linalg.norm(fs[k, 1])), 3) for k in (0, 1)], vz=round(vz, 4))
        if DEBUG and int(round(self.t / dt)) % 24 == 0:
            print("CONTACTS", self.last_contacts, flush=True)
        self.hook_s += time.perf_counter() - t0

    # ------------------------------------------------------------------ reset
    def restore_authored(self, setup):
        """Environment reset only (never during a trial): every body back to the setup's start pose, at rest."""
        torch = self.torch
        self.setup = setup
        self.trial_active = False
        self.adv, self.ang = [0.0, 0.0], [0.0, 0.0]
        a = authored(setup)
        tf = self.view.get_transforms().clone()
        for n, i in self.idx.items():
            pos, rot = a[n]
            tf[i, :3] = torch.tensor(np.asarray(pos, float) + self.origin, dtype=tf.dtype)
            tf[i, 3:7] = torch.tensor(_quat(rot), dtype=tf.dtype)
        idx = torch.arange(len(self.idx), dtype=torch.int32, device=tf.device)
        self.view.set_transforms(tf, idx)
        kin = [i for n, i in self.idx.items() if n not in FREE]
        self.view.set_kinematic_targets(tf, torch.tensor(kin, dtype=torch.int32, device=tf.device))
        free = torch.tensor([self.idx[n] for n in FREE], dtype=torch.int32, device=tf.device)
        self.view.set_velocities(torch.zeros_like(self.view.get_velocities()), free)
        self._init_state()

    # ------------------------------------------------------------------ scoring
    def control(self, environment):
        if not self.trial_active:
            return
        self.steps += 1
        poses = self.read()
        pc, rc = poses["cover"]
        lin, ang = self.velocity("cover")
        if not (np.all(np.isfinite(pc)) and np.all(np.isfinite(rc)) and np.all(np.isfinite(lin))):
            self.nonfinite += 1
            STATE["nonfinite"] = self.nonfinite
            return
        tips = finger_tips(environment)
        dpos, _ = poses["driver"]
        m = measure(pc, rc, self.panel_origin(), self.seated, tips, dpos,
                    [self.head_front(i) for i in (0, 1)])
        level = self.scorer.step(m, float(np.linalg.norm(lin)), math.degrees(float(np.linalg.norm(ang))),
                                 self.monitor)
        STATE.update(
            score=level, level=level, steps=self.steps, nonfinite=self.nonfinite,
            detail=dict(
                self.scorer.last,
                screws=dict(adv_mm=[round(v * 1e3, 3) for v in self.adv],
                            turns=[round(v / (2 * math.pi), 3) for v in self.turned],
                            engaged=list(self.engaged), seated=list(self.seated),
                            eng=getattr(self, "eng_detail", None)),
                peaks=dict(self.peaks),
                contacts=self.last_contacts,
                cover_pos_mm=np.round(pc * 1000, 2).tolist(),
                setup=self.setup,
                hook_wall_s=round(self.hook_s, 2),
            ),
        )


_RT: dict = {"rt": None, "sim": None, "trials": 0}


def runtime():
    return _RT["rt"]


def bench_state():
    """Current poses {name: (pos, rot)} and screw state for the scripted demo (read-only)."""
    rt = _RT["rt"]
    if rt is None:
        return None
    out = rt.read()
    out["screws"] = dict(adv=list(rt.adv), engaged=list(rt.engaged), seated=list(rt.seated),
                         head_front=[rt.head_front(i) for i in (0, 1)], axis=[rt.screw_axis(i) for i in (0, 1)])
    out["panel_origin"] = rt.panel_origin()
    return out


def _patch_scene(scene) -> None:
    if getattr(scene, "_panel_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Panel):
            return _arena_object(spec)
        return orig_task_object(spec)

    def object_states(environment, names):
        from yam_arena.task_file import ObjectState

        plain = [n for n in names if n != "panel"]
        out = orig_states(environment, plain) if plain else {}
        if "panel" in names:
            rt = _RT["rt"]
            p, r = rt.read()["cover"] if rt else authored(SETUP_BASE)["cover"]
            z = np.zeros(3)
            out["panel"] = ObjectState(pos=np.asarray(p), quat_xyzw=_quat(r), lin_vel=z, ang_vel=z, low=p - 0.2,
                                       high=p + 0.2)
        return {n: out[n] for n in names}

    orig_build = scene.build_task_environment

    def build(embodiment, task, control_hz=None):
        environment = orig_build(embodiment, task, control_hz)
        if any(isinstance(o, Panel) for o in task.objects):
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
    scene._panel_patched = True


def _has_panel(emb):
    return any(isinstance(o, Panel) for o in (emb.task.objects if emb.task is not None else ()))


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_panel_patched", False):
        return
    base_task_step, base_reset, base_ensure = cls._task_step, cls.reset, cls._ensure_sim

    def _ensure_sim(self, *a, **k):
        env_before = self._env
        out = base_ensure(self, *a, **k)
        if self._env is not None and self._env is not env_before and _has_panel(self):
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
            setup = (SETUP_BASE + _RT["trials"]) % 3
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
    cls._panel_patched = True


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
