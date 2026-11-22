"""grillkit: the Weber Q 1200 tabletop gas grill for yam_arena (Isaac Lab-Arena / PhysX), plus the runtime hooks.

The grill is ONE custom-spawned PhysX articulation (yam_arena task-file primitives are single boxes/spheres):
* ``body`` (floating root, 11.4 kg, on four rubber feet on the table): frame, cookbox, control housing, side tables,
  LP cylinder; cooking grate and burner are drawn (no collider: the robot never reaches into the cookbox);
* ``lid`` on a revolute ``hinge`` at the rear rim (2.1 kg; limits 0 = closed on the rim, 95 deg = resting open on its
  rear stop). Hinge friction: an elasto-plastic anchor (stiff position drive capped at the friction torque);
* ``stem`` on a prismatic ``push`` joint along the knob axis (5 mm travel) with a preloaded return spring (a position
  drive whose target is 12 mm behind the out stop: 7.2 N at rest, 600 N/m);
* ``knob`` on a revolute ``valve`` joint on the stem (0..170 deg counter-clockwise from OFF, seen from the front). Its
  resistance is an anchor drive: within 8 deg of OFF and with the knob pushed in less than 3 mm the OFF stop holds
  (anchor at OFF, capped at 10 N m); otherwise the valve's running friction (0.2 N m);
* ``button`` (igniter) on a prismatic ``igniter`` joint (4 mm travel) with a preloaded return spring (7 N at rest,
  700 N/m);
* ``flame``: a kinematic, collider-free body with the burner's flame drawn on it. It is moved under the grate while the
  burner is lit and parked under the table otherwise (rendering only).
Self-collision inside the articulation is off (the lid closes onto the rim through its joint limit). Robot links collide
with every grill collider. Geometry: ``../src/layout.py``; scorer and burner logic: ``../src/scoring.py`` (pure).

Runtime hooks (installed on import; nothing in yam_arena is edited): after every physics step the runtime reads the
joint positions and moves the friction anchors / OFF stop; after every control step it runs the burner model and the
scorer (robot-on-grill from the PhysX contact forces between robot links and grill links). Nothing writes a grill pose,
joint position or velocity during a trial; ``Runtime.restore_authored`` places the grill and zeroes its joints only on
an environment reset. The scripted demo READS state through ``bench_state()``.
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
from scoring import Scorer  # noqa: E402
from yam_arena.task_file import Box  # noqa: E402

PHYSICS_DT = float(os.environ.get("GRILL_PHYSICS_DT", str(1 / 480)))
CONTACT_OFFSET = 0.001
SETUP_BASE = int(os.environ.get("GRILL_SETUP", "0"))
FORCE_NAMED = "GRILL_SETUP" in os.environ
DEBUG = bool(os.environ.get("GRILL_DEBUG"))

HINGE_STIFF = 30.0  # N m/rad: hinge friction anchor (elastic play 0.3 / 30 rad = 0.6 deg)
HINGE_DAMP = 0.3  # N m s/rad
VALVE_STIFF = 150.0  # N m/rad: valve anchor / OFF stop (elastic play at the stop 10 / 150 rad = 3.8 deg)
VALVE_DAMP = 0.02
PUSH_TARGET = -L.PUSH_PRELOAD / L.PUSH_SPRING  # m, behind the out stop
IGN_TARGET = -L.IGN_PRELOAD / L.IGN_SPRING
LINKS = ("body", "lid", "stem", "knob", "button")
DOFS = ("hinge", "push", "valve", "igniter")

BLACK, CAST, GREY, HANDLE_C, RED, WHITE, GREEN, STEEL, RUBBER = (
    (0.04, 0.04, 0.045),
    (0.13, 0.13, 0.14),
    (0.62, 0.63, 0.64),
    (0.30, 0.31, 0.33),
    (0.80, 0.07, 0.06),
    (0.95, 0.95, 0.93),
    (0.10, 0.45, 0.20),
    (0.70, 0.71, 0.73),
    (0.05, 0.05, 0.05),
)
FLAME_BLUE, FLAME_TIP = (0.25, 0.45, 1.0), (1.0, 0.55, 0.10)
FLAME_PARK = np.array([0.0, 0.0, -0.40])  # under the table (hidden)


class Grill(Box):
    """The grill, as one task-file object (subclasses Box only to pass yam_arena's checks)."""


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


_SPEC: dict = {}


def _spawn_grill(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
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

    def look(color, emissive=None):
        key = (color, emissive)
        if key not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            if emissive is not None:
                vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), emissive_color=tuple(6.0 * v for v in linear_rgb(emissive)),
                                                  roughness=0.6)
            else:
                vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.5)
            vis.func(path, vis)
            looks[key] = path
        return looks[key]

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

    def rigid(path, mass, com=None, inertia=None, kinematic=False):
        prim = stage.GetPrimAtPath(path)
        rb = UsdPhysics.RigidBodyAPI.Apply(prim)
        if kinematic:
            rb.CreateKinematicEnabledAttr(True)
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(float(mass))
        if com is not None:
            m.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
        if inertia is not None:
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, inertia)))
            m.CreatePrincipalAxesAttr(Gf.Quatf(1, 0, 0, 0))
        px = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        px.CreateMaxDepenetrationVelocityAttr(0.3)
        px.CreateSolverPositionIterationCountAttr(32)
        px.CreateSolverVelocityIterationCountAttr(4)
        px.CreateLinearDampingAttr(0.0)
        px.CreateAngularDampingAttr(0.05)
        px.CreateSleepThresholdAttr(0.0)
        px.CreateStabilizationThresholdAttr(0.0)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
        return prim

    count = {"n": 0, "k": 0}

    def mesh(path, v, color, collide=True, friction=L.FRICTION["metal"], emissive=None):
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
            bind_physics_material(str(prim.GetPath()), material(friction), stage=stage)
            count["n"] += 1
        bind_visual_material(str(prim.GetPath()), look(color, emissive), stage=stage)
        count["k"] += 1
        return m

    def part(link, name, v, color, **kw):
        return mesh(f"{prim_path}/{link}/{name}", v, color, **kw)

    def frame_q(r):
        q = _quat(r)
        return Gf.Quatf(float(q[3]), *map(float, q[:3]))

    spec = L.setup_spec(SETUP_BASE)
    gp, gr = L.grill_pose(spec)
    # every link frame coincides with the grill frame at zero joint positions
    for n in LINKS:
        xform(f"{prim_path}/{n}", gp, gr)

    # ------------------------------------------------------------------ body (floating articulation root)
    bp = rigid(f"{prim_path}/body", L.BODY_MASS, com=(0.0, -0.02, 0.10), inertia=(0.45, 0.25, 0.60))
    UsdPhysics.ArticulationRootAPI.Apply(bp)
    ar = PhysxSchema.PhysxArticulationAPI.Apply(bp)
    ar.CreateEnabledSelfCollisionsAttr(False)
    ar.CreateSolverPositionIterationCountAttr(32)
    ar.CreateSolverVelocityIterationCountAttr(4)
    ar.CreateSleepThresholdAttr(0.0)
    ar.CreateStabilizationThresholdAttr(0.0)
    for i, (x, y) in enumerate(L.FEET):
        part("body", f"foot_{i}", L.box((x, y, L.FOOT_SIZE[2] / 2), L.FOOT_SIZE), RUBBER, friction=L.FRICTION["feet"])
    (fx, fy, fz0, fz1) = L.FRAME
    part("body", "frame", L.box_xyz(fx, fy, (fz0, fz1)), CAST)
    for i, v in enumerate(L.cookbox_hulls()):
        part("body", f"cookbox_{i}", v, CAST)
    cx, cy, cz0, cz1 = L.CONTROL
    part("body", "control", L.box_xyz(cx, cy, (cz0, cz1)), CAST)
    for i, (tx, ty) in enumerate(L.TABLES):
        part("body", f"table_{i}", L.box_xyz(tx, ty, L.TABLE_Z), GREY)
        # table bracket down to the frame
        yb = (ty[0], ty[0] + 0.02) if ty[0] > 0 else (ty[1] - 0.02, ty[1])
        part("body", f"table_arm_{i}", L.box_xyz((-0.12, 0.12), yb, (L.FRAME[3] - 0.01, L.TABLE_Z[0])), GREY)
    part("body", "cylinder", L.cyl("y", L.CYL_CENTER, L.CYL_R, L.CYL_L, n=20), GREEN)
    part("body", "regulator", L.cyl("y", (L.CYL_CENTER[0], -0.322, L.CYL_CENTER[2]), 0.018, 0.045, n=12), STEEL)
    # cooking grate (two halves, bars) and burner loop: drawn only
    for i, yb in enumerate(np.linspace(-0.225, 0.225, 16)):
        part("body", f"grate_{i}", L.box((0.0, yb, L.GRATE_Z - 0.006), (0.330, 0.008, 0.012)), BLACK, collide=False)
    part("body", "grate_rim", L.box((0.0, 0.0, L.GRATE_Z - 0.014), (0.340, 0.470, 0.004)), BLACK, collide=False)
    ax, ay = L.BURNER_OVAL
    for i in range(16):
        a0, a1 = 2 * math.pi * i / 16, 2 * math.pi * (i + 1) / 16
        p0 = np.array([ax * math.cos(a0), ay * math.sin(a0), L.BURNER_Z])
        p1 = np.array([ax * math.cos(a1), ay * math.sin(a1), L.BURNER_Z])
        seg = np.concatenate([p0 + d for d in L.cyl("z", (0, 0, 0), 0.010, 0.016, n=8)] +
                             [p1 + d for d in L.cyl("z", (0, 0, 0), 0.010, 0.016, n=8)]).reshape(-1, 3)
        part("body", f"burner_{i}", seg, STEEL, collide=False)
    # panel marks around the knob: OFF at 9 o'clock (seen from the front), START/HI 90 deg counter-clockwise from it
    # (6 o'clock), LO at 160 deg; the knob's index mark is on the fin's left end (points at OFF at the start)
    kc = L.KNOB_C
    for name, deg, size in (("off", 0.0, (0.002, 0.010, 0.004)), ("hi", L.KNOB_HI_DEG, (0.002, 0.010, 0.010)),
                            ("lo", L.KNOB_LO_DEG, (0.002, 0.005, 0.005))):
        a = math.radians(deg)
        rr = L.KNOB_R + 0.010
        a += math.radians(L.FIN_OFF_DEG)
        c = kc + np.array([-0.001, rr * math.cos(a), -rr * math.sin(a)])
        part("body", f"mark_{name}", L.box(c, size), WHITE, collide=False)
    part("body", "ign_ring", L.cyl("x", L.IGN_C + np.array([-0.0015, 0, 0]), L.IGN_R + 0.004, 0.003, n=20), STEEL,
         collide=False)

    # ------------------------------------------------------------------ lid
    lid_com = L.LID_COM
    rigid(f"{prim_path}/lid", L.LID_MASS, com=lid_com, inertia=(0.045, 0.030, 0.070))
    part("lid", "dome", L.lid_hull(), BLACK)
    hb = (L.HANDLE_X, 0.0, L.HANDLE_Z)
    part("lid", "handle", L.cyl("y", hb, L.HANDLE_R, 2 * L.HANDLE_HALF, n=16), HANDLE_C, friction=L.FRICTION["handle"])
    for i, y in enumerate(L.HANDLE_POSTS):
        part("lid", f"bracket_{i}", L.box_xyz((L.HANDLE_X, L.LID_FOOT[0][0] + 0.02), (y - 0.012, y + 0.012),
                                               (L.HANDLE_Z - 0.010, L.HANDLE_Z + 0.010)), HANDLE_C)
    part("lid", "thermo", L.cyl("z", (-0.060, 0.0, L.LID_TOP[2] + 0.002), 0.022, 0.006, n=20), STEEL, collide=False)
    part("lid", "thermo_face", L.cyl("z", (-0.060, 0.0, L.LID_TOP[2] + 0.0055), 0.017, 0.001, n=20), WHITE,
         collide=False)

    # ------------------------------------------------------------------ knob (stem + knob) and igniter button
    rigid(f"{prim_path}/stem", 0.02, com=kc, inertia=(2e-6, 2e-6, 2e-6))
    part("stem", "shaft", L.cyl("x", kc + np.array([-L.KNOB_GAP / 2 + 0.001, 0, 0]), 0.004, L.KNOB_GAP, n=10), STEEL,
         collide=False)
    rigid(f"{prim_path}/knob", 0.05, com=kc + np.array([-L.KNOB_GAP - L.KNOB_T / 2, 0, 0]), inertia=(1.5e-5, 1e-5, 1e-5))
    skirt_c = kc + np.array([-L.KNOB_GAP - L.KNOB_T / 2, 0.0, 0.0])
    part("knob", "skirt", L.cyl("x", skirt_c, L.KNOB_R, L.KNOB_T, n=24), BLACK, friction=L.FRICTION["knob"])
    fin_c = kc + np.array([-L.KNOB_GAP - L.KNOB_T - L.FIN[0] / 2, 0.0, 0.0])
    rf = L.rx(-L.FIN_OFF_DEG)  # fin turned about the knob axis: counter-clockwise seen from the front is about -x
    fin_v = (L.box((0.0, 0.0, 0.0), (L.FIN[0], L.FIN[1], L.FIN[2])) @ rf.T) + fin_c
    part("knob", "fin", fin_v, BLACK, friction=L.FRICTION["knob"])
    idx_v = (L.box((-L.FIN[0] / 2 - 0.0004, L.FIN[1] / 2 - 0.006, 0.0), (0.001, 0.010, 0.004)) @ rf.T) + fin_c
    part("knob", "index", idx_v, WHITE, collide=False)
    rigid(f"{prim_path}/button", 0.01, com=L.IGN_C, inertia=(1e-6, 1e-6, 1e-6))
    part("button", "cap", L.cyl("x", L.IGN_C + np.array([-L.IGN_PROUD / 2, 0, 0]), L.IGN_R, L.IGN_PROUD, n=20), RED,
         friction=L.FRICTION["button"])

    # ------------------------------------------------------------------ joints
    body = Sdf.Path(f"{prim_path}/body")

    def joint(kind, name, child, pos, rot, lower, upper):
        cls = UsdPhysics.RevoluteJoint if kind == "rev" else UsdPhysics.PrismaticJoint
        j = cls.Define(stage, f"{prim_path}/{child}/{name}")
        parent = body if child != "knob" else Sdf.Path(f"{prim_path}/stem")
        j.CreateBody0Rel().SetTargets([parent])
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/{child}")])
        j.CreateAxisAttr("X")
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, pos)))
        j.CreateLocalRot0Attr(frame_q(rot))
        j.CreateLocalPos1Attr(Gf.Vec3f(*map(float, pos)))
        j.CreateLocalRot1Attr(frame_q(rot))
        j.CreateLowerLimitAttr(float(lower))
        j.CreateUpperLimitAttr(float(upper))
        return j

    def drive(j, kind, stiff, damp, max_force, target):
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular" if kind == "rev" else "linear")
        d.CreateTypeAttr("force")
        scale = math.pi / 180 if kind == "rev" else 1.0  # USD angular drives are per degree
        d.CreateStiffnessAttr(float(stiff * scale))
        d.CreateDampingAttr(float(damp * scale))
        d.CreateMaxForceAttr(float(max_force))
        d.CreateTargetPositionAttr(float(target))

    # hinge: joint x along grill +y; positive lifts the front
    jh = joint("rev", "hinge", "lid", L.HINGE, L.rz(90.0), 0.0, L.LID_OPEN_STOP_DEG)
    drive(jh, "rev", HINGE_STIFF, HINGE_DAMP, L.HINGE_FRICTION_NM, 0.0)
    # push: joint x along grill +x (into the panel)
    jp = joint("pri", "push", "stem", kc, np.eye(3), 0.0, L.PUSH_TRAVEL)
    drive(jp, "pri", L.PUSH_SPRING, 4.0, 200.0, PUSH_TARGET)
    # valve: joint x along grill -x (toward the viewer): positive = counter-clockwise seen from the front
    jv = joint("rev", "valve", "knob", kc, L.rz(180.0), 0.0, L.KNOB_MAX_DEG)
    drive(jv, "rev", VALVE_STIFF, VALVE_DAMP, L.GATE_NM, 0.0)
    ji = joint("pri", "igniter", "button", L.IGN_C, np.eye(3), 0.0, L.IGN_TRAVEL)
    drive(ji, "pri", L.IGN_SPRING, 3.0, 200.0, IGN_TARGET)

    # ------------------------------------------------------------------ flame (kinematic, drawn only)
    xform(f"{prim_path}/flame", FLAME_PARK, np.eye(3))
    rigid(f"{prim_path}/flame", 0.1, kinematic=True)
    # 32 tongues along the burner loop: blue base, orange tips reaching up between the grate bars (seen from above)
    for i in range(32):
        a = 2 * math.pi * i / 32
        p = np.array([ax * math.cos(a), ay * math.sin(a), L.BURNER_Z + 0.006])
        out = np.array([math.cos(a) * 0.008, math.sin(a) * 0.008, 0.0])
        w = 0.007
        tongue = np.array([p + [w, 0, 0], p - [w, 0, 0], p + [0, w, 0], p - [0, w, 0],
                           p + out + [0, 0, 0.045]])
        mesh(f"{prim_path}/flame/blue_{i}", tongue, FLAME_BLUE, collide=False, emissive=FLAME_BLUE)
        top = L.GRATE_Z + 0.010 - p[2]
        tip = np.array([p + out * 0.8 + [0, 0, 0.035] + d for d in ([0.005, 0, 0], [-0.005, 0, 0], [0, 0.005, 0],
                                                                       [0, -0.005, 0])] + [p + out * 1.5 + [0, 0, top]])
        mesh(f"{prim_path}/flame/tip_{i}", tip, FLAME_TIP, collide=False, emissive=FLAME_TIP)

    _SPEC["paths"] = {n: f"{prim_path}/{n}" for n in LINKS + ("flame",)}
    _SPEC["colliders"] = count["n"]
    _SPEC["meshes"] = count["k"]
    return root.GetPrim()


def _arena_object(spec):
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class GrillSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_grill)

    return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=GrillSpawnerCfg(),
                  initial_pose=Pose(position_xyz=(0.0, 0.0, 0.0), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


# ---------------------------------------------------------------------------------------------- runtime state
STATE: dict = dict(score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, trial=-1, setup=None)


class Friction:
    """Elasto-plastic friction anchor for an articulation dof driven by a stiff position drive capped at ``cap``."""

    def __init__(self, stiff, q0=0.0):
        self.stiff = stiff
        self.anchor = q0

    def update(self, q, cap):
        lim = cap / self.stiff
        if q - self.anchor > lim:
            self.anchor = q - lim
        elif self.anchor - q > lim:
            self.anchor = q + lim
        return self.anchor


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


class Runtime:
    """Moves the friction anchors and the OFF stop every physics step; burner model + scorer every control step."""

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
        root = "/World/envs/env_0/grill"
        self.root = root
        P = lambda n: f"{root}/{n}"  # noqa: E731
        self.art = self.simview.create_articulation_view(P("body"))
        self.dof_names = list(self.art.shared_metatype.dof_names)
        self.idof = {n: self.dof_names.index(n) for n in DOFS}
        self.links = self.simview.create_rigid_body_view([P(n) for n in LINKS])
        self.flame = self.simview.create_rigid_body_view([P("flame")])
        robot_links = []
        for arm in ("left_robot", "right_robot"):
            lp = self.base.scene[arm].root_view.link_paths
            robot_links += list(lp[0] if lp and isinstance(lp[0], (list, tuple)) else lp)
        self.robot_links = robot_links
        self.cv = self.simview.create_rigid_contact_view([P(n) for n in LINKS],
                                                         filter_patterns=[robot_links] * len(LINKS),
                                                         max_contact_data_count=4096)
        self.dev = self.art.get_dof_positions().device
        self.ai = torch.tensor([0], dtype=torch.int32, device=self.dev)
        self.ai_cpu = torch.tensor([0], dtype=torch.int32)
        self.origin = self.base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        self.setup = SETUP_BASE
        self.trial_active = False
        self.lit_drawn = None
        self._init_state()

    def _init_state(self):
        self.scorer = Scorer()
        self.hinge = Friction(HINGE_STIFF, 0.0)
        self.valve = Friction(VALVE_STIFF, 0.0)
        self.steps = 0
        self.t = 0.0
        self.hook_s = 0.0
        self.nonfinite = 0
        self.peaks = dict(push_n=0.0, push_mm=0.0, valve_nm=0.0, igniter_n=0.0, igniter_mm=0.0, lid_deg=0.0,
                          contact_n=0.0, gate_nm=0.0)
        self.gate_engaged = True
        self.contact_link_n = [0.0] * len(LINKS)
        self.robot_link_peak = {}
        self.knob_axial_step = 0.0
        self.body_rot = np.eye(3)
        self.contact_max_step = 0.0
        self.events = []
        STATE.update(score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, setup=self.setup)

    def begin_trial(self, trial):
        self._init_state()
        self.trial_active = True
        STATE.update(trial=trial, setup=self.setup, start=L.setup_spec(self.setup))

    # ------------------------------------------------------------------ reading
    def dofs(self):
        q = self.art.get_dof_positions().detach().cpu().numpy()[0].astype(float)
        return {n: float(q[i]) for n, i in self.idof.items()}

    def joint_state(self):
        """Lid / knob angles in degrees, push / igniter travel in mm."""
        d = self.dofs()
        return dict(lid_deg=math.degrees(d["hinge"]), knob_deg=math.degrees(d["valve"]), push_mm=d["push"] * 1e3,
                    igniter_mm=d["igniter"] * 1e3)

    def read(self):
        tf = self.links.get_transforms().detach().cpu().numpy().astype(float)
        return {n: (tf[i, :3] - self.origin, _rot(tf[i, 3:7])) for i, n in enumerate(LINKS)}

    # ------------------------------------------------------------------ physics step
    def physics(self, dt):
        t0 = time.perf_counter()
        self.t += dt
        torch = self.torch
        d = self.dofs()
        push, knob = d["push"], d["valve"]
        # OFF stop: engaged within GATE_DEG of OFF unless the knob is pushed in past the unlock depth
        gate = knob < math.radians(L.GATE_DEG) and push < L.PUSH_UNLOCK
        if gate:
            self.valve.anchor = 0.0
            vcap = L.GATE_NM
            vt = 0.0
        else:
            vcap = L.VALVE_FRICTION_NM
            vt = self.valve.update(knob, vcap)
        ht = self.hinge.update(d["hinge"], L.HINGE_FRICTION_NM)
        targets = self.art.get_dof_position_targets().clone()
        targets[0, self.idof["hinge"]] = ht
        targets[0, self.idof["valve"]] = vt
        targets[0, self.idof["push"]] = PUSH_TARGET
        targets[0, self.idof["igniter"]] = IGN_TARGET
        self.art.set_dof_position_targets(targets, self.ai)
        if gate != self.gate_engaged:
            mf = self.art.get_dof_max_forces().clone()
            mf[0, self.idof["valve"]] = vcap
            self.art.set_dof_max_forces(mf, self.ai_cpu)
            self.gate_engaged = gate
            self.events.append(dict(t=round(self.t, 3), gate="engaged" if gate else "released",
                                    push_mm=round(push * 1e3, 2), knob_deg=round(math.degrees(knob), 2)))
        if self.trial_active:
            pk = self.peaks
            pk["push_mm"] = max(pk["push_mm"], push * 1e3)
            pk["push_n"] = max(pk["push_n"], L.PUSH_PRELOAD + L.PUSH_SPRING * push)
            pk["igniter_mm"] = max(pk["igniter_mm"], d["igniter"] * 1e3)
            pk["igniter_n"] = max(pk["igniter_n"], L.IGN_PRELOAD + L.IGN_SPRING * d["igniter"])
            pk["lid_deg"] = max(pk["lid_deg"], math.degrees(d["hinge"]))
            tq = min(vcap, VALVE_STIFF * abs(knob - vt))
            if gate:
                pk["gate_nm"] = max(pk["gate_nm"], tq)
            else:
                pk["valve_nm"] = max(pk["valve_nm"], tq)
            fm = self.cv.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
            per = np.linalg.norm(fm, axis=-1).sum(axis=-1) if fm.size else np.zeros(len(LINKS))
            if fm.size:  # which robot link pushes hardest on each grill link (diagnostics)
                nl = np.linalg.norm(fm, axis=-1)
                for gi, gname in enumerate(LINKS):
                    j = int(np.argmax(nl[gi]))
                    if nl[gi, j] > self.robot_link_peak.get(gname, (0.0, ""))[0]:
                        self.robot_link_peak[gname] = (float(nl[gi, j]), "/".join(self.robot_links[j].split("/")[-2:]))
            if fm.size:  # axial push on the knob: net robot force on the knob link along its axis (into the panel)
                ki = LINKS.index("knob")
                axis = self.body_rot @ np.array([1.0, 0.0, 0.0])
                ax_n = abs(float(fm[ki].sum(axis=0) @ axis))
                self.knob_axial_step = max(self.knob_axial_step, ax_n)
                pk["knob_axial_n"] = max(pk.get("knob_axial_n", 0.0), ax_n)
            self.contact_link_n = [max(a, float(b)) for a, b in zip(self.contact_link_n, per)]
            self.contact_max_step = max(self.contact_max_step, float(per.max()) if per.size else 0.0)
            pk["contact_n"] = max(pk["contact_n"], float(per.max()) if per.size else 0.0)
        self.hook_s += time.perf_counter() - t0

    # ------------------------------------------------------------------ flame (rendering only)
    def draw_flame(self, lit):
        if lit == self.lit_drawn:
            return
        torch = self.torch
        spec = L.setup_spec(self.setup)
        gp, gr = L.grill_pose(spec)
        pos, rot = (gp, gr) if lit else (FLAME_PARK, np.eye(3))
        tf = self.flame.get_transforms().clone()
        tf[0, :3] = torch.tensor(np.asarray(pos, float) + self.origin, dtype=tf.dtype)
        tf[0, 3:7] = torch.tensor(_quat(rot), dtype=tf.dtype)
        idx = torch.tensor([0], dtype=torch.int32, device=tf.device)
        try:
            self.flame.set_kinematic_targets(tf, idx)
        except Exception:  # noqa: BLE001
            self.flame.set_transforms(tf, idx)
        self.lit_drawn = lit

    # ------------------------------------------------------------------ reset
    def restore_authored(self, setup):
        """Environment reset only (never during a trial): grill placed for the setup, at rest, lid closed, knob OFF,
        igniter out; flame parked."""
        torch = self.torch
        self.setup = setup
        self.trial_active = False
        spec = L.setup_spec(setup)
        gp, gr = L.grill_pose(spec)
        rt = self.art.get_root_transforms().clone()
        rt[0, :3] = torch.tensor(gp + self.origin + np.array([0.0, 0.0, 0.0005]), dtype=rt.dtype)
        rt[0, 3:7] = torch.tensor(_quat(gr), dtype=rt.dtype)
        self.art.set_root_transforms(rt, self.ai)
        self.art.set_root_velocities(torch.zeros_like(self.art.get_root_velocities()), self.ai)
        q = torch.zeros_like(self.art.get_dof_positions())
        self.art.set_dof_positions(q, self.ai)
        self.art.set_dof_velocities(torch.zeros_like(q), self.ai)
        tg = q.clone()
        tg[0, self.idof["push"]] = PUSH_TARGET
        tg[0, self.idof["igniter"]] = IGN_TARGET
        self.art.set_dof_position_targets(tg, self.ai)
        mf = self.art.get_dof_max_forces().clone()
        mf[0, self.idof["valve"]] = L.GATE_NM
        self.art.set_dof_max_forces(mf, self.ai_cpu)
        self.lit_drawn = None
        self.draw_flame(False)
        self._init_state()

    # ------------------------------------------------------------------ scoring
    def control(self, environment, dt=0.05):
        if not self.trial_active:
            return
        self.steps += 1
        js = self.joint_state()
        if not all(np.isfinite(v) for v in js.values()):
            self.nonfinite += 1
            STATE["nonfinite"] = self.nonfinite
            return
        contact = self.contact_max_step > 0.05
        poses = self.read()
        bp, br = poses["body"]
        self.body_rot = br
        spec = L.setup_spec(self.setup)
        gp, gr = L.grill_pose(spec)
        shift_mm = float(np.linalg.norm(bp[:2] - gp[:2])) * 1e3
        yaw_shift = math.degrees(math.atan2(br[1, 0], br[0, 0])) - spec["yaw"]
        level = self.scorer.step(dt, js["lid_deg"], js["knob_deg"], js["igniter_mm"], contact,
                                 knob_axial_n=self.knob_axial_step, shift_mm=shift_mm, yaw_shift_deg=yaw_shift)
        self.knob_axial_step = 0.0
        self.draw_flame(self.scorer.burner.lit)
        STATE.update(
            score=level,
            level=level,
            steps=self.steps,
            nonfinite=self.nonfinite,
            detail=dict(
                self.scorer.last,
                push_mm=round(js["push_mm"], 3),
                gate=self.gate_engaged,
                contact_n=round(self.contact_max_step, 3),
                contact_by_link={n: round(v, 2) for n, v in zip(LINKS, self.contact_link_n)},
                robot_link_peak={k: [round(v[0], 1), v[1]] for k, v in self.robot_link_peak.items()},
                peaks={k: round(v, 3) for k, v in self.peaks.items()},
                body_shift_mm=round(float(np.linalg.norm(bp[:2] - gp[:2])) * 1e3, 2),
                body_yaw_shift_deg=round(math.degrees(math.atan2(br[1, 0], br[0, 0])) - spec["yaw"], 3),
                setup=self.setup,
                events=self.events[-6:],
            ),
        )
        self.contact_max_step = 0.0


_RT: dict = {"rt": None, "sim": None, "trials": 0}


def runtime():
    return _RT["rt"]


def bench_state():
    """Current link poses {'body', 'lid', 'stem', 'knob', 'button'} plus joint state, for the scripted demo
    (read-only)."""
    rt = _RT["rt"]
    if rt is None:
        return None
    out = rt.read()
    out["joints"] = rt.joint_state()
    return out


def _patch_scene(scene) -> None:
    if getattr(scene, "_grill_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Grill):
            return _arena_object(spec)
        return orig_task_object(spec)

    def object_states(environment, names):
        from yam_arena.task_file import ObjectState

        plain = [n for n in names if n != "grill"]
        out = orig_states(environment, plain) if plain else {}
        if "grill" in names:
            rt = _RT["rt"]
            if rt:
                p, r = rt.read()["body"]
            else:
                p, r = L.grill_pose(L.setup_spec(SETUP_BASE))
            z = np.zeros(3)
            out["grill"] = ObjectState(pos=np.asarray(p), quat_xyzw=_quat(r), lin_vel=z, ang_vel=z, low=p - 0.3,
                                       high=p + 0.3)
        return {n: out[n] for n in names}

    orig_build = scene.build_task_environment

    def build(embodiment, task, control_hz=None):
        environment = orig_build(embodiment, task, control_hz)
        if any(isinstance(o, Grill) for o in task.objects):
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
    scene._grill_patched = True


def _has_grill(emb):
    return any(isinstance(o, Grill) for o in (emb.task.objects if emb.task is not None else ()))


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_grill_patched", False):
        return
    base_task_step, base_reset, base_ensure = cls._task_step, cls.reset, cls._ensure_sim

    def _ensure_sim(self, *a, **k):
        env_before = self._env
        out = base_ensure(self, *a, **k)
        if self._env is not None and self._env is not env_before and _has_grill(self):
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
            rt.restore_authored(setup)
        obs = base_reset(self, *a, **k)
        rt = _RT["rt"]
        if rt is not None:
            gp, _ = L.grill_pose(L.setup_spec(setup))
            moved = float(np.linalg.norm(rt.read()["body"][0][:2] - gp[:2])) > 0.001
            if rt.setup != setup or moved or _RT["trials"] == 0:  # first reset of a fresh env, or the env reset moved it
                rt.restore_authored(setup)
                obs = base_reset(self, *a, **k)
            rt.begin_trial(_RT["trials"])
            _RT["trials"] += 1
        return obs

    cls._ensure_sim, cls._task_step, cls.reset = _ensure_sim, _task_step, reset
    cls._grill_patched = True


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
