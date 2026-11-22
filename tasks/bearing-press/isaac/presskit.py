"""presskit: the bench arbor press, the actuator housing, the SKF 61806-2RS1 bearing, the two driver discs and the tool
tray for yam_arena (Isaac Lab-Arena / PhysX), plus the runtime hooks.

Authored as one custom-spawned assembly (yam_arena task-file primitives are single boxes/spheres):
* the press as a fixed-base articulation: frame (base casting with its table, column, head; root, fixed to the world),
  the ram on a prismatic joint and the handle (pinion) on a revolute joint. Rack and pinion: a stiff articulation
  position drive on the ram follows the handle angle (12.7 mm of ram per handle radian) and the runtime applies the
  equal and opposite generalised force to the handle joint every physics step (so the ram's load is felt at the
  handle at the pinion ratio). The pinion has joint friction (it holds the handle and ram where they are left);
* the housing as a floating articulation: housing body (root, free on the press table) and the fit-model support (a
  thin annulus inside the bore under the bearing's outer ring) on a prismatic joint. Interference-fit model: an
  elasto-plastic friction anchor (stiff position drive capped at the fit force F(d), d = travel into the bore; the
  anchor follows when the push exceeds the cap): the support yields only while the bearing is pushed down harder
  than the fit force, so the bearing moves only under the press. Past the shoulder the bearing rests on the real
  shoulder face. A jam (bearing cocked, or pressed off-centre) latches the cap at 5 kN (the ring stops);
* the bearing (one rigid body: outer ring, seals, inner ring) and the two driver discs as free rigid bodies; the tray
  is kinematic.
Geometry: ``../src/layout.py``; scorer: ``../src/scoring.py`` (pure functions).

Runtime hooks (installed on import; nothing in yam_arena is edited; same pattern as earlier task kits): every physics
step the rack-and-pinion reaction, the fit anchor and the contact-force monitor; every control step the scorer.
Nothing here writes a part's pose or velocity during a trial; ``Runtime.restore_authored`` puts every part back only on
an environment reset. The scripted demo READS poses through ``bench_state()``.
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

PHYSICS_DT = float(os.environ.get("BP_PHYSICS_DT", str(1 / 480)))
CONTACT_OFFSET = float(os.environ.get("BP_CONTACT_OFFSET", ".0001"))  # below the 0.12 mm bore clearance
SETUP_BASE = int(os.environ.get("BP_SETUP", "0"))
FORCE_NAMED = "BP_SETUP" in os.environ
DEBUG = bool(os.environ.get("BP_DEBUG"))
DEV_PRESET = os.environ.get("BP_DEV_PRESET", "")  # development probes only: 'stack' = parts pre-stacked under the ram

RAM_K = 2.0e6  # N/m rack-and-pinion stiffness (ram drive)
RAM_D = 2.0e3  # N s/m
RAM_FMAX = 2.0e4
HANDLE_ARMATURE = 0.02  # kg m^2 (pinion and hub inertia)
FIT_K = 1.0e7  # N/m fit anchor stiffness (elastic travel 0.03 mm at 300 N)
FIT_D = 300.0
JAM_N = 5000.0

PAINT, STEEL, DARKSTEEL, ALU, BLACK, SEAL, FOAM, KNOB = (
    (0.20, 0.33, 0.30),
    (0.70, 0.71, 0.73),
    (0.42, 0.43, 0.45),
    (0.80, 0.81, 0.83),
    (0.06, 0.06, 0.07),
    (0.14, 0.10, 0.08),
    (0.16, 0.17, 0.19),
    (0.08, 0.08, 0.08),
)
FREE = ("bearing", "disc_out", "disc_in", "disc_big")
FRICTION = dict(steel=0.15, alu=0.2, foam=0.6, iron=0.18, handle=0.5)


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


def _hull_faces(v):
    from scipy.spatial import ConvexHull

    h = ConvexHull(v)
    faces = []
    for sx, eq in zip(h.simplices, h.equations):
        a, bb, c = v[sx]
        if np.dot(np.cross(bb - a, c - a), eq[:3]) < 0:
            sx = sx[[0, 2, 1]]
        faces.append([int(i) for i in sx])
    return faces


def _box_v(lo_hi):
    (x0, x1), (y0, y1), (z0, z1) = lo_hi
    return np.array([[x, y, z] for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)], float)


def _cyl_along(p0, p1, r, n=16):
    """Vertices of a convex cylinder from p0 to p1 (any axis)."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    ax = p1 - p0
    ax /= np.linalg.norm(ax)
    u = np.cross(ax, [0.0, 0.0, 1.0] if abs(ax[2]) < 0.9 else [1.0, 0.0, 0.0])
    u /= np.linalg.norm(u)
    w = np.cross(ax, u)
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.outer(np.cos(a), u) + np.outer(np.sin(a), w)
    return np.r_[p0 + r * ring, p1 + r * ring]


def _sphere_v(c, r, n=10):
    pts = []
    for i in range(1, n):
        th = math.pi * i / n
        for j in range(2 * n):
            ph = math.pi * j / n
            pts.append([r * math.sin(th) * math.cos(ph), r * math.sin(th) * math.sin(ph), r * math.cos(th)])
    pts += [[0, 0, r], [0, 0, -r]]
    return np.asarray(pts) + np.asarray(c, float)


def authored(setup):
    """Start poses of the free parts {name: (pos, rot)} (env frame)."""
    s = L.part_starts(setup)
    if DEV_PRESET == "stack":
        hc = np.array([L.RAM_XY[0], L.RAM_XY[1], L.PLATE_Z + 0.0001])
        bz = hc[2] + L.HOUSING_H - L.START_SINK + 0.0002
        s["housing"] = (hc, np.eye(3))
        s["bearing"] = (np.array([hc[0], hc[1], bz]), np.eye(3))
        s["disc_out"] = (np.array([hc[0], hc[1], bz + L.BEARING_B + 0.0002]), np.eye(3))
    return s


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
    rw = np.array(root.ComputeLocalToWorldTransform(0).ExtractTranslation(), float)
    looks, mats = {}, {}
    count = {"n": 0}

    def look(color):
        if color not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.45, metallic=0.0)
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

    def xform(path, pos, rot=np.eye(3)):
        x = UsdGeom.Xform.Define(stage, path)
        x.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float, pos)))
        q = _quat(rot)
        x.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(q[3]), *map(float, q[:3])))
        return x

    def rigid(path, mass, kinematic=False, inertia=None, com=None, iters=32, depen=0.3):
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
        px.CreateMaxDepenetrationVelocityAttr(float(depen))
        px.CreateSolverPositionIterationCountAttr(int(iters))
        px.CreateSolverVelocityIterationCountAttr(4)
        px.CreateLinearDampingAttr(0.0)
        px.CreateAngularDampingAttr(0.05)
        px.CreateSleepThresholdAttr(0.0)  # never sleeps: a sleeping body reports no contacts
        px.CreateStabilizationThresholdAttr(0.0)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
        return prim

    def hull(path, v, color, friction, approximation="convexHull"):
        v = np.asarray(v, float)
        m = UsdGeom.Mesh.Define(stage, path)
        m.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in v])
        faces = _hull_faces(v)
        m.CreateFaceVertexCountsAttr([3] * len(faces))
        m.CreateFaceVertexIndicesAttr([j for f in faces for j in f])
        m.CreateSubdivisionSchemeAttr("none")
        m.CreateExtentAttr([Gf.Vec3f(*map(float, v.min(axis=0))), Gf.Vec3f(*map(float, v.max(axis=0)))])
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

    def hulls(path, vs, color, friction):
        for i, v in enumerate(vs):
            hull(f"{path}_{i:03d}", v, color, friction)

    def visual_cyl(path, center, axis, r, h, color):
        g = UsdGeom.Cylinder.Define(stage, path)
        g.CreateAxisAttr(axis)
        g.CreateRadiusAttr(float(r))
        g.CreateHeightAttr(float(h))
        g.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
        bind_visual_material(str(g.GetPrim().GetPath()), look(color), stage=stage)

    def q_of(r):
        q = _quat(r)
        return Gf.Quatf(float(q[3]), *map(float, q[:3]))

    s0 = authored(SETUP_BASE)
    paths = {}
    # ------------------------------------------------------------ press: fixed-base articulation
    frame = f"{prim_path}/press"
    fo = np.array([L.RAM_XY[0], L.RAM_XY[1], L.TOP])  # frame origin: ram axis on the table
    xform(frame, fo)
    rigid(frame, 30.0, inertia=(1.0, 1.0, 1.0))
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(frame))
    ar = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(frame))
    ar.CreateEnabledSelfCollisionsAttr(False)
    ar.CreateSolverPositionIterationCountAttr(64)
    ar.CreateSolverVelocityIterationCountAttr(4)
    fj = UsdPhysics.FixedJoint.Define(stage, f"{frame}/to_world")
    fj.CreateBody1Rel().SetTargets([Sdf.Path(frame)])
    fj.CreateLocalPos0Attr(Gf.Vec3f(*map(float, fo + rw)))
    fj.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    fj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    fj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    for name, box in (("base", L.BASE_BOX), ("column", L.COLUMN_BOX), ("head", L.HEAD_BOX)):
        v = _box_v(box) - fo
        hull(f"{frame}/{name}", v, PAINT, FRICTION["iron"])
    # pinion hub (right side of the head)
    hub_c = L.PIVOT - fo
    y_face = L.HEAD_BOX[1][0] - fo[1] + 0.002  # from the head's right face out to the handle boss
    hull(f"{frame}/hub", _cyl_along([hub_c[0], y_face, hub_c[2]], hub_c + [0, 0.012, 0], 0.028, 24), PAINT, FRICTION["iron"])
    paths["press"] = frame
    # ram: joint frame at the ram axis; q < 0 = down
    ram = f"{prim_path}/ram"
    ram_c0 = np.array([L.RAM_XY[0], L.RAM_XY[1], L.RAM_FACE0])
    xform(ram, ram_c0)
    rigid(ram, L.RAM_MASS, inertia=(0.012, 0.012, 0.0002), com=(0, 0, L.RAM_LEN / 2), iters=64)
    w = L.RAM_W / 2
    hull(f"{ram}/bar", _box_v(((-w, w), (-w, w), (0.0, L.RAM_LEN))), STEEL, FRICTION["steel"])
    paths["ram"] = ram
    rj = UsdPhysics.PrismaticJoint.Define(stage, f"{ram}/slide")
    rj.CreateBody0Rel().SetTargets([Sdf.Path(frame)])
    rj.CreateBody1Rel().SetTargets([Sdf.Path(ram)])
    rj.CreateAxisAttr("Z")
    rj.CreateLocalPos0Attr(Gf.Vec3f(*map(float, ram_c0 - fo)))
    rj.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    rj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    rj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    rj.CreateLowerLimitAttr(float(-L.RAM_TRAVEL[1]))
    rj.CreateUpperLimitAttr(float(-L.RAM_TRAVEL[0] + 0.002))
    d = UsdPhysics.DriveAPI.Apply(rj.GetPrim(), "linear")
    d.CreateTypeAttr("force")
    d.CreateStiffnessAttr(float(RAM_K))
    d.CreateDampingAttr(float(RAM_D))
    d.CreateMaxForceAttr(float(RAM_FMAX))
    d.CreateTargetPositionAttr(0.0)
    # handle: link frame at the pivot, joint about +y (q > 0 = handle up); theta = -q
    hdl = f"{prim_path}/handle"
    xform(hdl, L.PIVOT)
    hd0 = L.handle_dir(0.0)
    rigid(hdl, L.HANDLE_MASS, inertia=(0.015, 0.015, 0.015), com=tuple(hd0 * 0.12), iters=64)
    yb = 0.0  # bar in the hub's plane (the link frame is at the bar's y)
    hull(f"{hdl}/bar", _cyl_along(hd0 * 0.02, hd0 * L.HANDLE_LEN, L.HANDLE_R, 16), STEEL, FRICTION["handle"])
    hull(f"{hdl}/knob", _sphere_v(hd0 * (L.HANDLE_LEN + 0.010), L.KNOB_R), KNOB, FRICTION["handle"])
    hull(f"{hdl}/boss", _cyl_along([0, yb + 0.012, 0], [0, yb - 0.012, 0], 0.016, 16), STEEL, FRICTION["steel"])
    paths["handle"] = hdl
    hj = UsdPhysics.RevoluteJoint.Define(stage, f"{hdl}/pinion")
    hj.CreateBody0Rel().SetTargets([Sdf.Path(frame)])
    hj.CreateBody1Rel().SetTargets([Sdf.Path(hdl)])
    hj.CreateAxisAttr("Y")
    hj.CreateLocalPos0Attr(Gf.Vec3f(*map(float, L.PIVOT - fo)))
    hj.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    hj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    hj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    hj.CreateLowerLimitAttr(float(-math.degrees(L.HANDLE_THETA_LIM[1])))
    hj.CreateUpperLimitAttr(float(-math.degrees(L.HANDLE_THETA_LIM[0])))
    hp = hj.GetPrim()
    PhysxSchema.PhysxJointAPI.Apply(hp).CreateArmatureAttr(float(HANDLE_ARMATURE))
    hp.AddAppliedSchema("PhysxJointAxisAPI:angular")
    for attr, value in (("staticFrictionEffort", L.HANDLE_FRICTION_NM), ("dynamicFrictionEffort", L.HANDLE_FRICTION_NM),
                        ("viscousFrictionCoefficient", 0.02)):
        hp.CreateAttribute(f"physxJointAxis:angular:{attr}", Sdf.ValueTypeNames.Float).Set(float(value))
    # ------------------------------------------------------------ housing: floating articulation (housing + fit support)
    hs = f"{prim_path}/housing"
    hpos, hrot = s0["housing"]
    xform(hs, hpos, hrot)
    rigid(hs, L.HOUSING_MASS, inertia=(4e-5, 4e-5, 6e-5), com=(0, 0, L.HOUSING_H / 2), iters=64)
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(hs))
    ar2 = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(hs))
    ar2.CreateEnabledSelfCollisionsAttr(False)
    ar2.CreateSolverPositionIterationCountAttr(64)
    ar2.CreateSolverVelocityIterationCountAttr(4)
    ar2.CreateSleepThresholdAttr(0.0)
    hulls(f"{hs}/al", L.housing_pieces(), ALU, FRICTION["alu"])
    paths["housing"] = hs
    pl = f"{prim_path}/plunger"
    q0 = -L.START_SINK
    xform(pl, hpos + hrot @ np.array([0.0, 0.0, L.HOUSING_H + q0]), hrot)
    rigid(pl, 0.05, inertia=(5e-6, 5e-6, 1e-5), iters=64)
    hulls(f"{pl}/s", L.plunger_pieces(), ALU, FRICTION["steel"])
    paths["plunger"] = pl
    pj = UsdPhysics.PrismaticJoint.Define(stage, f"{pl}/fit")
    pj.CreateBody0Rel().SetTargets([Sdf.Path(hs)])
    pj.CreateBody1Rel().SetTargets([Sdf.Path(pl)])
    pj.CreateAxisAttr("Z")
    pj.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, float(L.HOUSING_H)))
    pj.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
    pj.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))  # q = plunger top relative to the housing face (q0 at the start)
    pj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    pj.CreateLowerLimitAttr(float(-L.SEAT_DEPTH - 0.0001))
    pj.CreateUpperLimitAttr(float(q0))
    d = UsdPhysics.DriveAPI.Apply(pj.GetPrim(), "linear")
    d.CreateTypeAttr("force")
    d.CreateStiffnessAttr(float(FIT_K))
    d.CreateDampingAttr(float(FIT_D))
    d.CreateMaxForceAttr(float(L.FIT_START_N))
    d.CreateTargetPositionAttr(float(q0))
    # ------------------------------------------------------------ free parts
    bpos, brot = s0["bearing"]
    br = f"{prim_path}/bearing"
    xform(br, bpos, brot)
    ro, ri = L.BRG_OD / 2, L.BRG_D / 2
    izz = 0.5 * L.BRG_MASS * (ro * ro + ri * ri)
    rigid(br, L.BRG_MASS, inertia=(izz / 2 + L.BRG_MASS * L.BEARING_B ** 2 / 12,) * 2 + (izz,),
          com=(0, 0, L.BEARING_B / 2), iters=64, depen=0.5)
    pieces = L.bearing_pieces()
    nb = L.NB
    hulls(f"{br}/outer", pieces[:nb], STEEL, FRICTION["steel"])
    hulls(f"{br}/seal", pieces[nb:nb + 24], SEAL, FRICTION["steel"])
    hulls(f"{br}/inner", pieces[nb + 24:], STEEL, FRICTION["steel"])
    paths["bearing"] = br
    for name, pcs, r in (("disc_out", L.disc_out_pieces(), L.DISC_OUT_R), ("disc_in", L.disc_in_pieces(), L.DISC_IN_R),
                         ("disc_big", L.disc_big_pieces(), L.DISC_BIG_R)):
        p, rr = s0[name]
        path = f"{prim_path}/{name}"
        xform(path, p, rr)
        m = L.disc_mass(r)
        iz = 0.5 * m * r * r
        rigid(path, m, inertia=(iz / 2 + m * L.DISC_T ** 2 / 12,) * 2 + (iz,), com=(0, 0, L.DISC_T / 2), iters=64)
        hulls(f"{path}/s", pcs, STEEL, FRICTION["steel"])
        paths[name] = path
    # ------------------------------------------------------------ tray (kinematic foam insert)
    tr = f"{prim_path}/tray"
    xform(tr, np.r_[L.TRAY_C, L.TOP])
    rigid(tr, 1.0, kinematic=True)
    sx, sy, sz = L.TRAY_SIZE
    hull(f"{tr}/foam", _box_v(((-sx / 2, sx / 2), (-sy / 2, sy / 2), (0.0, sz))), FOAM, FRICTION["foam"])
    paths["tray"] = tr
    # put-away shelf for the disc that is not in the tray this trial (under the table, behind the robot; out of reach
    # and out of the cameras' view)
    pk = f"{prim_path}/park"
    xform(pk, np.r_[-0.45, 0.0, L.PARK_Z])
    rigid(pk, 1.0, kinematic=True)
    hull(f"{pk}/shelf", _box_v(((-0.05, 0.05), (-0.16, 0.16), (-0.01, 0.0))), FOAM, FRICTION["foam"])
    _SPEC["paths"] = paths
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
        mid = pose[:3] + r @ np.array(TCP_OFFSET) - origin
        for p in (tip, mid):
            pts += [p + r[:, 0] * s * (half + 0.0032) for s in (1.0, -1.0)]
    return pts


class Friction:
    """Elasto-plastic friction anchor for an articulation dof driven by a stiff position drive capped at ``cap``."""

    def __init__(self, stiff, q0):
        self.stiff = stiff
        self.anchor = q0

    def update(self, q, cap):
        lim = cap / self.stiff
        if q - self.anchor > lim:
            self.anchor = q - lim
        elif self.anchor - q > lim:
            self.anchor = q + lim
        return self.anchor


class Runtime:
    """Rack and pinion, fit model and contact monitor every physics step; scorer every control step."""

    def __init__(self, environment):
        import carb
        import omni.physics.tensors as tensors
        import torch
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdUtils

        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        if os.environ.get("BP_STREAM_WAIT"):  # development runs only (unset in evidence runs): cap the renderer's
            # per-frame RTX streaming wait, which hangs for 30 s a frame on one of this host's GPUs
            from isaaclab_physx.renderers import isaac_rtx_renderer_utils as rtx_utils

            rtx_utils._STREAMING_WAIT_TIMEOUT_S = float(os.environ["BP_STREAM_WAIT"])
        self.base = environment.unwrapped
        self.env = environment
        self.torch = torch
        stage = get_current_stage()
        stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        self.simview = tensors.create_simulation_view("torch", stage_id=stage_id)
        self.simview.set_subspace_roots("/")
        root = "/World/envs/env_0/bench"
        P = lambda n: f"{root}/{n}"  # noqa: E731
        self.P = P
        self.view = self.simview.create_rigid_body_view([P(n) for n in FREE])
        self.links = self.simview.create_rigid_body_view([P(n) for n in ("ram", "handle", "housing", "plunger")])
        self.press = self.simview.create_articulation_view(P("press"))
        self.hart = self.simview.create_articulation_view(P("housing"))
        self.press_dofs = list(self.press.shared_metatype.dof_names)
        self.i_ram = self.press_dofs.index("slide")
        self.i_hdl = self.press_dofs.index("pinion")
        self.dev = self.press.get_dof_positions().device
        self.ai = torch.tensor([0], dtype=torch.int32, device=self.dev)
        self.ai_cpu = torch.tensor([0], dtype=torch.int32)
        robot_links = []
        for arm in ("left_robot", "right_robot"):
            lp = self.base.scene[arm].root_view.link_paths
            robot_links += list(lp[0] if lp and isinstance(lp[0], (list, tuple)) else lp)
        self.robot_links = robot_links
        self.brg_from = ("disc_out", "disc_in", "disc_big", "ram", "housing", "plunger")
        self.cv_brg = self.simview.create_rigid_contact_view(
            [P("bearing")], filter_patterns=[[P(n) for n in self.brg_from] + robot_links], max_contact_data_count=8192
        )
        self.ram_from = ("disc_out", "disc_in", "disc_big", "bearing", "housing")
        self.cv_ram = self.simview.create_rigid_contact_view(
            [P("ram")], filter_patterns=[[P(n) for n in self.ram_from]], max_contact_data_count=4096
        )
        self.cv_hdl = self.simview.create_rigid_contact_view(
            [P("handle")], filter_patterns=[robot_links], max_contact_data_count=2048
        )
        self.origin = self.base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        self.setup = SETUP_BASE
        self.trial_active = False
        self.dev_torque = 0.0
        self._init_state()

    def _init_state(self):
        self.monitor = Monitor()
        self.scorer = Scorer()
        self.fit = Friction(FIT_K, -L.START_SINK)
        self.jammed = False
        self.support_off = False
        self.steps = 0
        self.t = 0.0
        self.hook_s = 0.0
        self.nonfinite = 0
        self.ram_force = 0.0
        self.peak = dict(ram_n=0.0, handle_robot_n=0.0, fit_cap_n=0.0, ram_drive_n=0.0)
        self.last_mech = {}
        self._prev_bearing = None
        STATE.update(score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, setup=self.setup)

    def begin_trial(self, trial):
        self._init_state()
        self.trial_active = True
        STATE.update(trial=trial, setup=self.setup, start=L.setup_spec(self.setup))

    # ------------------------------------------------------------------ reading
    def read(self):
        tf = self.view.get_transforms().detach().cpu().numpy().astype(float)
        lt = self.links.get_transforms().detach().cpu().numpy().astype(float)
        out = {n: (tf[i, :3] - self.origin, _rot(tf[i, 3:7])) for i, n in enumerate(FREE)}
        for i, n in enumerate(("ram", "handle", "housing", "plunger")):
            out[n] = (lt[i, :3] - self.origin, _rot(lt[i, 3:7]))
        return out

    def dofs(self):
        q = self.press.get_dof_positions().detach().cpu().numpy().astype(float)[0]
        v = self.press.get_dof_velocities().detach().cpu().numpy().astype(float)[0]
        hq = self.hart.get_dof_positions().detach().cpu().numpy().astype(float)[0]
        return dict(ram_q=-q[self.i_ram], ram_v=-v[self.i_ram], theta=-q[self.i_hdl], plunger_q=float(hq[0]))

    def velocities(self):
        v = self.view.get_velocities().detach().cpu().numpy().astype(float)
        return {n: (v[i, :3], v[i, 3:6]) for i, n in enumerate(FREE)}

    # ------------------------------------------------------------------ physics step
    def physics(self, dt):
        t0 = time.perf_counter()
        torch = self.torch
        self.t += dt
        q = self.press.get_dof_positions().clone()
        v = self.press.get_dof_velocities().clone()
        theta = float(-q[0, self.i_hdl])
        target = -L.ram_of_theta(theta)  # ram joint q (up +)
        f_drive = RAM_K * (target - float(q[0, self.i_ram])) - RAM_D * float(v[0, self.i_ram])
        f_drive = float(np.clip(f_drive, -RAM_FMAX, RAM_FMAX))
        tg = self.press.get_dof_position_targets().clone()
        tg[0, self.i_ram] = target
        self.press.set_dof_position_targets(tg, self.ai)
        eff = torch.zeros_like(q)
        # reaction of the rack on the pinion (handle joint q = -theta); dev_torque: development probes only (0 otherwise)
        eff[0, self.i_hdl] = -L.PINION_R * f_drive - self.dev_torque
        self.press.set_dof_actuation_forces(eff, self.ai)
        self.peak["ram_drive_n"] = max(self.peak["ram_drive_n"], -f_drive)
        # fit model
        hq = float(self.hart.get_dof_positions()[0, 0])
        depth = -L.START_SINK - hq
        if depth >= L.ENGAGE_LEN - 0.00005:
            # the ring is on the shoulder: the fit support lets go (its elastic travel would otherwise spring the ring
            # back up when the press load comes off; the real shoulder carries it from here)
            self.support_off = True
        cap = 0.0 if self.support_off else (JAM_N if self.jammed else L.fit_force(depth))
        anchor = self.fit.update(hq, cap)
        ht = self.hart.get_dof_position_targets().clone()
        ht[0, 0] = anchor
        self.hart.set_dof_position_targets(ht, self.ai)
        mf = self.hart.get_dof_max_forces().clone()
        mf[0, 0] = cap
        self.hart.set_dof_max_forces(mf, self.ai_cpu)
        self.peak["fit_cap_n"] = max(self.peak["fit_cap_n"], cap if not self.jammed else 0.0)
        if self.trial_active:
            fb = self.cv_brg.get_contact_force_matrix(dt=dt).detach().cpu().numpy()[0]
            fr = self.cv_ram.get_contact_force_matrix(dt=dt).detach().cpu().numpy()[0]
            fh = self.cv_hdl.get_contact_force_matrix(dt=dt).detach().cpu().numpy()[0]
            by = {n: float(np.linalg.norm(fb[i])) for i, n in enumerate(self.brg_from)}
            robot_on_brg = float(np.linalg.norm(fb[len(self.brg_from):], axis=-1).sum()) if fb.shape[0] > len(self.brg_from) else 0.0
            ram_by = {n: float(np.linalg.norm(fr[i])) for i, n in enumerate(self.ram_from)}
            ram_load = sum(ram_by.values())
            hand_on_handle = float(np.linalg.norm(fh, axis=-1).sum()) if fh.size else 0.0
            finger_max = float(np.linalg.norm(fh, axis=-1).max()) if fh.size else 0.0  # largest single robot link
            self.peak["handle_link_max_n"] = max(self.peak.get("handle_link_max_n", 0.0), finger_max)
            self.peak["ram_n"] = max(self.peak["ram_n"], ram_load)
            self.peak["handle_robot_n"] = max(self.peak["handle_robot_n"], hand_on_handle)
            poses = self.read()
            m = dict(
                ram_load=ram_load,
                inner_load=by["disc_in"] + by["disc_big"] + by["ram"],
                bearing=poses["bearing"],
                housing=poses["housing"],
                disc_out=poses["disc_out"],
                depth=depth,
            )
            jam = self.monitor.step(dt, m)
            if jam and not self.jammed:
                self.jammed = True
            self.last_mech = dict(
                theta_deg=round(math.degrees(theta), 3),
                ram_mm=round(-float(q[0, self.i_ram]) * 1e3, 3),
                ram_drive_n=round(-f_drive, 2),
                handle_torque_nm=round(-L.PINION_R * f_drive, 3),
                fit_depth_mm=round(depth * 1e3, 3),
                fit_cap_n=round(cap, 1),
                bearing_by_n={k: round(v_, 2) for k, v_ in by.items()},
                robot_on_bearing_n=round(robot_on_brg, 2),
                ram_by_n={k: round(v_, 2) for k, v_ in ram_by.items()},
                hand_on_handle_n=round(hand_on_handle, 2),
                jammed=self.jammed,
            )
            if DEBUG and int(round(self.t / dt)) % 48 == 0:
                print("MECH", self.last_mech, flush=True)
        self.hook_s += time.perf_counter() - t0

    # ------------------------------------------------------------------ reset
    def restore_authored(self, setup):
        """Environment reset only (never during a trial): every part back to its start pose; ram, handle and fit
        support back to their start."""
        torch = self.torch
        self.setup = setup
        self.trial_active = False
        a = authored(setup)
        tf = self.view.get_transforms().clone()
        for i, n in enumerate(FREE):
            pos, rot = a[n]
            tf[i, :3] = torch.tensor(np.asarray(pos, float) + self.origin, dtype=tf.dtype)
            tf[i, 3:7] = torch.tensor(_quat(rot), dtype=tf.dtype)
        idx = torch.arange(len(FREE), dtype=torch.int32, device=tf.device)
        self.view.set_transforms(tf, idx)
        self.view.set_velocities(torch.zeros_like(self.view.get_velocities()), idx)
        hp, hr = a["housing"]
        rt = self.hart.get_root_transforms().clone()
        rt[0, :3] = torch.tensor(np.asarray(hp, float) + self.origin, dtype=rt.dtype)
        rt[0, 3:7] = torch.tensor(_quat(hr), dtype=rt.dtype)
        self.hart.set_root_transforms(rt, self.ai)
        self.hart.set_root_velocities(torch.zeros_like(self.hart.get_root_velocities()), self.ai)
        hq = self.hart.get_dof_positions().clone()
        hq[0, 0] = -L.START_SINK
        self.hart.set_dof_positions(hq, self.ai)
        self.hart.set_dof_velocities(torch.zeros_like(hq), self.ai)
        self.hart.set_dof_position_targets(hq.clone(), self.ai)
        pq = self.press.get_dof_positions().clone()
        pq[:] = 0.0
        self.press.set_dof_positions(pq, self.ai)
        self.press.set_dof_velocities(torch.zeros_like(pq), self.ai)
        self.press.set_dof_position_targets(pq.clone(), self.ai)
        self._init_state()

    # ------------------------------------------------------------------ scoring
    def control(self, environment):
        if not self.trial_active:
            return
        self.steps += 1
        poses = self.read()
        vel = self.velocities()
        if not all(np.all(np.isfinite(p)) and np.all(np.isfinite(r)) for p, r in poses.values()):
            self.nonfinite += 1
            STATE["nonfinite"] = self.nonfinite
            return
        tips = finger_tips(environment)
        d = self.dofs()
        m = measure(poses, d["theta"], tips)
        # stillness from the bearing's pose change over the control step (its axis direction, not its spin about it):
        # the instantaneous velocity of a part squeezed between the ram stack and the shoulder is solver noise
        bp, br = poses["bearing"]
        prev = getattr(self, "_prev_bearing", None)
        dt_c = float(getattr(self.base, "step_dt", 0.05))
        if prev is None:
            lin_s, ang_s = 0.0, 0.0
        else:
            lin_s = float(np.linalg.norm(bp - prev[0])) / dt_c
            ang_s = math.degrees(math.acos(float(np.clip(br[:, 2] @ prev[1][:, 2], -1.0, 1.0)))) / dt_c
        self._prev_bearing = (bp.copy(), br.copy())
        level = self.scorer.step(m, lin_s, ang_s, self.monitor)
        STATE.update(
            score=level,
            level=level,
            steps=self.steps,
            nonfinite=self.nonfinite,
            detail=dict(
                self.scorer.last,
                mech=self.last_mech,
                peak={k: round(v, 2) for k, v in self.peak.items()},
                setup=self.setup,
                hook_wall_s=round(self.hook_s, 2),
            ),
        )


_RT: dict = {"rt": None, "sim": None, "trials": 0}


def runtime():
    return _RT["rt"]


def bench_state():
    """Current poses of the parts and press links, plus the press dofs (read-only, for the scripted demo)."""
    rt = _RT["rt"]
    if rt is None:
        return None
    out = rt.read()
    out["dofs"] = rt.dofs()
    return out


def _patch_scene(scene) -> None:
    if getattr(scene, "_bp_patched", False):
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
            p, r = rt.read()["bearing"] if rt else authored(SETUP_BASE)["bearing"]
            z = np.zeros(3)
            out["bench"] = ObjectState(pos=np.asarray(p), quat_xyzw=_quat(r), lin_vel=z, ang_vel=z, low=p - 0.3, high=p + 0.3)
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
    scene._bp_patched = True


def _has_bench(emb):
    return any(isinstance(o, Bench) for o in (emb.task.objects if emb.task is not None else ()))


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_bp_patched", False):
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
            setup = (SETUP_BASE + _RT["trials"]) % len(L.HOUSING_NAMED)
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
    cls._bp_patched = True


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
