"""regkit: the shop-air drop (non-relieving regulator, gauge, bleed valve) for yam_arena (Isaac Lab-Arena / PhysX),
plus the runtime hooks.

Bodies (custom-spawned; yam_arena task-file primitives are single boxes), one fixed-base articulation
(``../src/layout.py`` has the dimensions and constants):
* ``base``: the drop board, bracket, regulator body and bonnet, supply pipe, outlet tee, the gauge case and dial, the
  ball-valve body and muffler (fixed to the bench);
* ``carrier`` (prismatic along the knob axis on the base, the push-to-lock travel) and ``knob`` (revolute about the
  knob axis on the carrier): the 36 mm knurled adjusting knob. Detent: a stiff drive to the nearer end of the travel
  capped at F_DETENT (the knob snaps out or in only when pulled / pushed harder than that). Pushed in: the lock ring
  holds the knob's turn (stiff drive capped at T_LOCK). Pulled out: adjusting-screw friction (an elasto-plastic
  anchor capped at layout.knob_torque(setpoint));
* ``lever``: the ball valve's lever (revolute about the vertical stem, stops at 0 = closed and 90 deg = open), with
  the valve's operating friction (anchor capped at T_VALVE);
* ``needle``: the gauge pointer (revolute about the dial axis, no collider), driven to the angle of the gauge
  pressure.

Pressure (runtime, every physics step): spring setpoint = p0 + k x (clockwise knob turns since the start); line
pressure from layout.step_pressure (non-relieving regulator in, bleed out); gauge pressure follows the line pressure
with the snubber lag; the needle is driven there. Nothing is teleported during a trial: the knob and the lever move
only when the robot moves them. The scorer reads the gauge, the setpoint, the line pressure, the knob's travel, the
lever angle and the robot-on-part contact forces (PhysX contact force matrix between the robot's links and the parts:
collider identity). ``Runtime.restore_authored`` places the parts only on an environment reset. The scripted demo
READS state through ``bench_state()`` (the gauge reading, the knob's travel and the lever angle, as an operator sees
them; never the hidden setpoint or spring rate).
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

PHYSICS_DT = float(os.environ.get("RG_PHYSICS_DT", str(1 / 480)))
CONTACT_OFFSET = 0.001
SETUP_BASE = int(os.environ.get("RG_SETUP", "0"))
_SPAWN = {"setup": SETUP_BASE}  # start state the scene is authored in (the first reset's; see _patch_embodiment)
FORCE_NAMED = "RG_SETUP" in os.environ

K_SLIDE, D_SLIDE = 20000.0, 30.0  # knob travel detent (N/m, N s/m), capped at F_DETENT
K_KNOB, D_KNOB = 20.0, 0.02  # knob turn: lock / friction anchor (N m/rad, N m s/rad)
K_LEVER, D_LEVER = 20.0, 0.02  # valve lever friction anchor
K_NEEDLE, D_NEEDLE = 0.05, 0.002  # gauge pointer drive
LINKS = ("base", "carrier", "knob", "lever", "needle")
BODIES = ("base", "knob", "lever")  # contact-checked bodies
DOFS = ("slide", "turn", "valve", "pointer")

ALU, DARK, BLACK, STEEL, ZINC, WHITE, RED, BRASS, ORANGE, PLY, MUFF = (
    (0.78, 0.79, 0.80),
    (0.20, 0.20, 0.22),
    (0.03, 0.03, 0.03),
    (0.72, 0.73, 0.75),
    (0.55, 0.57, 0.60),
    (0.95, 0.95, 0.94),
    (0.80, 0.08, 0.06),
    (0.78, 0.62, 0.25),
    (0.98, 0.45, 0.05),
    (0.76, 0.62, 0.42),
    (0.55, 0.45, 0.30),
)


class AirDrop(Box):
    """The whole air drop, as one task-file object (subclasses Box only to pass yam_arena's checks)."""


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


def _wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


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

    def look(color, emissive=None):
        key = (color, emissive)
        if key not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            if emissive is not None:
                vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color),
                                                  emissive_color=tuple(8.0 * v for v in linear_rgb(emissive)),
                                                  roughness=0.5)
            else:
                metal = 0.6 if color in (ALU, STEEL, BRASS, ZINC) else 0.0
                vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.45, metallic=metal)
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

    def rigid(path, mass, com=None, inertia=None, depen=0.3):
        prim = stage.GetPrimAtPath(path)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(float(mass))
        if com is not None:
            m.CreateCenterOfMassAttr(Gf.Vec3f(*map(float, com)))
        if inertia is not None:
            m.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, inertia)))
            m.CreatePrincipalAxesAttr(Gf.Quatf(1, 0, 0, 0))
        px = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        px.CreateMaxDepenetrationVelocityAttr(depen)
        px.CreateSolverPositionIterationCountAttr(32)
        px.CreateSolverVelocityIterationCountAttr(4)
        px.CreateLinearDampingAttr(0.0)
        px.CreateAngularDampingAttr(0.0)
        px.CreateSleepThresholdAttr(0.0)
        px.CreateStabilizationThresholdAttr(0.0)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
        return prim

    count = {"n": 0, "k": 0}

    def mesh(path, v, color, collide=True, friction=0.5, emissive=None):
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

    spec = L.setup_spec(_SPAWN["setup"])
    rp, rr = L.reg_pose(spec)
    F = L.FRICTION
    H = L.H_BODY  # bench at R z = -H

    # ------------------------------------------------------------------ base (articulation root, fixed to the bench)
    B = f"{prim_path}/base"
    xform(B, rp, rr)
    bprim = rigid(B, 3.0, com=(0.05, -0.04, -0.05), inertia=(0.02, 0.02, 0.02))
    UsdPhysics.ArticulationRootAPI.Apply(bprim)
    ar = PhysxSchema.PhysxArticulationAPI.Apply(bprim)
    ar.CreateEnabledSelfCollisionsAttr(False)
    ar.CreateSolverPositionIterationCountAttr(32)
    ar.CreateSolverVelocityIterationCountAttr(8)
    ar.CreateSleepThresholdAttr(0.0)
    ar.CreateStabilizationThresholdAttr(0.0)
    fj = UsdPhysics.FixedJoint.Define(stage, f"{B}/fix_to_bench")
    fj.CreateBody1Rel().SetTargets([Sdf.Path(B)])
    fj.CreateLocalPos0Attr(Gf.Vec3f(*map(float, rp)))
    q = _quat(rr)
    fj.CreateLocalRot0Attr(Gf.Quatf(float(q[3]), *map(float, q[:3])))
    fj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    fj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    # drop board (18 mm plywood) on a foot, bracket, regulator body, bonnet and its collar
    mesh(f"{B}/board", L.box_xyz(L.BOARD_X, L.BOARD_Y, (-H, L.BOARD_ZTOP)), PLY, friction=F["board"])
    mesh(f"{B}/board_foot", L.box_xyz((L.BOARD_X[0] - 0.05, L.BOARD_X[1] + 0.05), L.BOARD_Y, (-H, -H + 0.018)),
         PLY, friction=F["board"])
    mesh(f"{B}/bracket", L.box_xyz((0.022, L.BOARD_X[0]), (-0.014, 0.014), (-0.028, -0.012)), STEEL,
         friction=F["body"])
    (bx0, bx1), (by0, by1), (bz0, bz1) = L.BODY
    mesh(f"{B}/body", L.box_xyz((bx0, bx1), (by0, by1), (bz0, bz1)), ZINC, friction=F["body"])
    mesh(f"{B}/bonnet", L.cyl_z((0, 0, sum(L.BONNET_Z) / 2), L.BONNET_R, L.BONNET_Z[1] - L.BONNET_Z[0], n=32), DARK,
         friction=F["body"])
    for i, v in enumerate(L.ring_segments(L.KNOB_R + 0.0006, L.BONNET_R + 0.0015, L.BONNET_Z[1], L.COLLAR_TOP, n=16)):
        mesh(f"{B}/collar_{i}", v, DARK, friction=F["body"])
    mesh(f"{B}/plug", L.cyl_z((0, 0, bz0 - 0.004), 0.012, 0.008, n=16), ZINC, collide=False)
    # supply in on the robot's left: nipple along +y, elbow, pipe back to the board
    mesh(f"{B}/inlet", L.cyl_axis((0, by1, 0), (0, 0.062, 0), L.PIPE_R), BRASS, friction=F["pipe"])
    mesh(f"{B}/inlet_elbow", L.box_xyz((-0.010, 0.010), (0.055, 0.075), (-0.010, 0.010)), STEEL, friction=F["pipe"])
    mesh(f"{B}/supply", L.cyl_axis((0.0, 0.065, 0), (L.BOARD_X[0], 0.065, 0), L.PIPE_R), STEEL, friction=F["pipe"])
    # outlet to the robot's right: nipple, tee (gauge branch up), nipple, ball valve, elbow down, muffler
    gx, gy, _ = L.GAUGE_C
    vx, vy, _ = L.VALVE_C
    mesh(f"{B}/outlet", L.cyl_axis((0, by0, 0), (0, gy + 0.010, 0), L.PIPE_R), BRASS, friction=F["pipe"])
    mesh(f"{B}/tee", L.box_xyz((gx - 0.010, gx + 0.010), (gy - 0.012, gy + 0.012), (-0.010, 0.012)), BRASS,
         friction=F["pipe"])
    mesh(f"{B}/gauge_nipple", L.cyl_z((gx, gy, 0.020), 0.006, 0.016, n=16), BRASS, friction=F["pipe"])
    mesh(f"{B}/gauge_case", L.cyl_z((gx, gy, sum(L.GAUGE_Z) / 2), L.GAUGE_R, L.GAUGE_Z[1] - L.GAUGE_Z[0], n=40),
         BLACK, friction=F["body"])
    zf = L.GAUGE_Z[1]
    mesh(f"{B}/dial", L.cyl_z((gx, gy, zf + 0.0004), L.GAUGE_R - 0.002, 0.0008, n=40), WHITE, collide=False)
    for p in range(0, 161, 2):  # graduations: 2 psi minor, 10 psi medium, 20 psi major (numerals not modelled)
        a = L.gauge_angle(p)
        r1 = L.GAUGE_R - 0.004
        ln = 0.0065 if p % 20 == 0 else (0.0042 if p % 10 == 0 else 0.0025)
        w = 0.0009 if p % 10 == 0 else 0.0005
        c, s = math.cos(a), math.sin(a)
        d = np.array([c, s, 0.0])
        n_ = np.array([-s, c, 0.0])
        pts = []
        for rr_ in (r1 - ln, r1):
            for ww in (-w / 2, w / 2):
                for zz in (zf + 0.0008, zf + 0.0012):
                    pts.append(np.array([gx, gy, 0.0]) + rr_ * d + ww * n_ + np.array([0, 0, zz]))
        mesh(f"{B}/tick_{p}", np.array(pts), BLACK, collide=False)
    mesh(f"{B}/valve_nipple", L.cyl_axis((0, gy - 0.010, 0), (0, vy + 0.014, 0), L.PIPE_R), BRASS, friction=F["pipe"])
    mesh(f"{B}/valve_body", L.box_xyz((vx - 0.012, vx + 0.012), (vy - 0.016, vy + 0.016), (-0.012, 0.012)), BRASS,
         friction=F["pipe"])
    mesh(f"{B}/valve_stem", L.cyl_z((vx, vy, 0.019), 0.005, 0.014, n=12), STEEL, friction=F["pipe"])
    mesh(f"{B}/valve_out", L.cyl_axis((0, vy - 0.016, 0), (0, vy - 0.030, 0), L.PIPE_R), BRASS, friction=F["pipe"])
    mesh(f"{B}/elbow_down", L.box_xyz((vx - 0.009, vx + 0.009), (vy - 0.040, vy - 0.026), (-0.010, 0.008)), BRASS,
         friction=F["pipe"])
    mesh(f"{B}/muffler", L.cyl_z((vx, vy - 0.033, -0.026), 0.0075, 0.030, n=16), MUFF, friction=F["pipe"])
    mesh(f"{B}/valve_clamp", L.box_xyz((vx + 0.012, L.BOARD_X[0]), (vy - 0.008, vy + 0.008), (-0.012, -0.002)),
         STEEL, friction=F["body"])
    # valve stops (visual lugs) at the closed and open lever positions
    mesh(f"{B}/stop_lug", L.box_xyz((vx - 0.004, vx + 0.004), (vy + 0.010, vy + 0.016), (0.012, 0.020)), STEEL,
         collide=False)

    # ------------------------------------------------------------------ knob carrier (push-to-lock travel)
    C = f"{prim_path}/carrier"
    xform(C, rp + rr @ np.array([0.0, 0.0, L.KNOB_Z0]), rr)
    rigid(C, 0.005, com=(0, 0, 0), inertia=(2e-7, 2e-7, 2e-7))

    # ------------------------------------------------------------------ knob (turns on the carrier)
    K = f"{prim_path}/knob"
    xform(K, rp + rr @ np.array([0.0, 0.0, L.KNOB_Z0]), rr)
    rigid(K, 0.035, com=(0, 0, L.KNOB_H / 2), inertia=(6e-6, 6e-6, 1.0e-5))
    mesh(f"{K}/grip", L.cyl_z((0, 0, L.KNOB_H / 2 + 0.0035), L.KNOB_R, L.KNOB_H - 0.007, n=32), BLACK,
         friction=F["knob"])
    mesh(f"{K}/skirt", L.cyl_z((0, 0, 0.00175), L.KNOB_R - 0.0004, 0.0035, n=32), ORANGE, collide=False)
    k0 = math.radians(spec.get("knob0", 0.0))
    for i in range(18):  # knurl ribs
        a = k0 + 2 * math.pi * i / 18
        c, s = math.cos(a), math.sin(a)
        pts = []
        for rr_ in (L.KNOB_R - 0.0005, L.KNOB_R + 0.0012):
            for ww in (-0.0012, 0.0012):
                for zz in (0.006, L.KNOB_H - 0.002):
                    pts.append([rr_ * c - ww * s, rr_ * s + ww * c, zz])
        mesh(f"{K}/rib_{i}", np.array(pts), DARK, collide=False)
    mesh(f"{K}/mark", L.box_xyz((L.KNOB_R - 0.006, L.KNOB_R - 0.001), (-0.0012, 0.0012), (L.KNOB_H, L.KNOB_H + 0.0004))
         @ L.rz(math.degrees(k0)).T, WHITE, collide=False)

    # ------------------------------------------------------------------ valve lever (on the stem)
    V = f"{prim_path}/lever"
    xform(V, rp + rr @ np.array([vx, vy, L.STEM_TOP]), rr)
    rigid(V, 0.030, com=(-0.040, 0, 0.005), inertia=(2e-6, 6.5e-5, 6.5e-5))
    mesh(f"{V}/hub", L.cyl_z((0, 0, 0.004), 0.008, 0.008, n=16), STEEL, friction=F["lever"])
    mesh(f"{V}/blade", L.box_xyz((-L.GRIP_SPAN[0] - 0.002, 0.0), (-0.005, 0.005), (0.001, 0.005)), STEEL,
         friction=F["lever"])
    mesh(f"{V}/sleeve", L.box_xyz((-L.GRIP_SPAN[1], -L.GRIP_SPAN[0]), (-L.GRIP_W / 2, L.GRIP_W / 2),
                                  (0.0, L.GRIP_T)), RED, friction=F["lever"])
    mesh(f"{V}/nut", L.cyl_z((0, 0, 0.010), 0.004, 0.004, n=6), STEEL, collide=False)

    # ------------------------------------------------------------------ gauge pointer
    N = f"{prim_path}/needle"
    xform(N, rp + rr @ np.array([gx, gy, zf + 0.002]), rr)
    rigid(N, 0.003, com=(0.006, 0, 0), inertia=(1e-7, 1e-6, 1e-6))
    # visual pointer, named "arm": the revolute joint below is <needle>/pointer, and a joint defined on a mesh's prim
    # path re-types that prim (the pointer would not render); 2.4 mm wide at the hub, 1.1 mm at the tip
    mesh(f"{N}/arm", np.array([[-0.006, -0.0012, 0.0], [-0.006, 0.0012, 0.0], [-0.006, -0.0012, 0.0008],
                               [-0.006, 0.0012, 0.0008], [L.NEEDLE_L, -0.00055, 0.0], [L.NEEDLE_L, 0.00055, 0.0],
                               [L.NEEDLE_L, -0.00055, 0.0008], [L.NEEDLE_L, 0.00055, 0.0008]]), BLACK, collide=False)
    mesh(f"{N}/hubcap", L.cyl_z((0, 0, 0.0006), 0.0022, 0.0012, n=12), BLACK, collide=False)

    # ------------------------------------------------------------------ joints
    def joint(kind, name, parent, child, axis, pos0, lo=None, hi=None):
        cls = UsdPhysics.RevoluteJoint if kind == "rev" else UsdPhysics.PrismaticJoint
        j = cls.Define(stage, f"{prim_path}/{child}/{name}")
        j.CreateBody0Rel().SetTargets([Sdf.Path(f"{prim_path}/{parent}")])
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/{child}")])
        j.CreateAxisAttr(axis)
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, pos0)))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        if lo is not None:
            j.CreateLowerLimitAttr(float(lo))
            j.CreateUpperLimitAttr(float(hi))
        PhysxSchema.PhysxJointAPI.Apply(j.GetPrim()).CreateJointFrictionAttr(0.0)
        return j

    def drive(j, kind, stiff, damp, max_force):
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular" if kind == "rev" else "linear")
        d.CreateTypeAttr("force")
        scale = math.pi / 180 if kind == "rev" else 1.0  # USD angular drives are per degree
        d.CreateStiffnessAttr(float(stiff * scale))
        d.CreateDampingAttr(float(damp * scale))
        d.CreateMaxForceAttr(float(max_force))
        d.CreateTargetPositionAttr(0.0)
        d.CreateTargetVelocityAttr(0.0)

    js = joint("pri", "slide", "base", "carrier", "Z", (0, 0, L.KNOB_Z0), 0.0, L.KNOB_OUT)
    drive(js, "pri", K_SLIDE, D_SLIDE, L.F_DETENT)
    jt = joint("rev", "turn", "carrier", "knob", "Z", (0, 0, 0))  # unlimited (multi-turn), unwrapped at runtime
    drive(jt, "rev", K_KNOB, D_KNOB, L.T_LOCK)
    jv = joint("rev", "valve", "base", "lever", "Z", (vx, vy, L.STEM_TOP), 0.0, 90.0)
    drive(jv, "rev", K_LEVER, D_LEVER, L.T_VALVE)
    jn = joint("rev", "pointer", "base", "needle", "Z", (gx, gy, zf + 0.002), -160.0, 160.0)
    drive(jn, "rev", K_NEEDLE, D_NEEDLE, 0.5)

    _SPEC["colliders"] = count["n"]
    _SPEC["meshes"] = count["k"]
    return root.GetPrim()


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


class Plant:
    """Knob lock state, unwrapped knob turn, spring setpoint, line and gauge pressure."""

    def __init__(self, spec):
        self.spec = spec
        self.locked = True
        self.out_end = False  # detent target: False = in, True = out
        self.rz_prev = None
        self.rz = 0.0  # unwrapped knob joint angle (rad, CCW from above +)
        self.rz0 = 0.0
        self.knob_anchor = Friction(K_KNOB)
        self.lever_anchor = Friction(K_LEVER)
        self.p = float(spec["p0"])
        self.pg = float(spec["p0"])
        self.pg_prev = self.pg
        self.events = []
        self.peaks = dict(knob_nm=0.0, lever_nm=0.0, pull_n=0.0)

    def unwrap(self, raw):
        if self.rz_prev is None:
            self.rz_prev = raw
            self.rz = raw
            self.rz0 = raw
            self.knob_anchor.anchor = raw
        self.rz += _wrap_pi(raw - self.rz_prev)
        self.rz_prev = raw
        return self.rz

    @property
    def turns_cw(self):
        return -(self.rz - self.rz0) / (2 * math.pi)

    @property
    def p_set(self):
        return min(L.P_MAX_SET, max(0.0, self.spec["p0"] + self.spec["k"] * self.turns_cw))


class Runtime:
    """Knob detent / lock / friction, valve friction and the pressure plant every physics step; scorer every control
    step."""

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
        root = "/World/envs/env_0/airdrop"
        self.root = root
        P = lambda n: f"{root}/{n}"  # noqa: E731
        self.art = self.simview.create_articulation_view(P("base"))
        self.dof_names = list(self.art.shared_metatype.dof_names)
        print("regkit: dofs", self.dof_names, flush=True)
        self.idof = {n: i for i, n in enumerate(self.dof_names)}
        self.view = self.simview.create_rigid_body_view([P(n) for n in LINKS])
        self.vidx = {n: i for i, n in enumerate(LINKS)}
        robot_links = []
        for arm in ("left_robot", "right_robot"):
            lp = self.base.scene[arm].root_view.link_paths
            robot_links += list(lp[0] if lp and isinstance(lp[0], (list, tuple)) else lp)
        self.robot_links = robot_links
        self.cv = self.simview.create_rigid_contact_view([P(n) for n in BODIES],
                                                         filter_patterns=[robot_links] * len(BODIES),
                                                         max_contact_data_count=4096)
        self.dev = self.art.get_dof_positions().device
        self.ai = torch.tensor([0], dtype=torch.int32, device=self.dev)
        self.ai_cpu = torch.tensor([0], dtype=torch.int32)
        self.origin = self.base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        self.setup = _SPAWN["setup"]
        self.trial_active = False
        self.mf_now = None
        self._init_state()

    def _init_state(self):
        self.scorer = Scorer()
        self.plant = Plant(L.setup_spec(self.setup))
        self.steps = 0
        self.t = 0.0
        self.hook_s = 0.0
        self.nonfinite = 0
        self.contact_step = {n: 0.0 for n in BODIES}
        self.contact_peak = {n: 0.0 for n in BODIES}
        self.robot_link_peak = {}
        self.min_p = self.plant.p
        self.max_valve = 0.0
        self.max_out = 0.0
        STATE.update(score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, setup=self.setup)

    def begin_trial(self, trial):
        self._init_state()
        self.trial_active = True
        STATE.update(trial=trial, setup=self.setup, start=L.setup_spec(self.setup))

    # ------------------------------------------------------------------ reading
    def dofs(self):
        q = self.art.get_dof_positions().detach().cpu().numpy()[0].astype(float)
        v = self.art.get_dof_velocities().detach().cpu().numpy()[0].astype(float)
        return {n: float(q[self.idof[n]]) for n in DOFS}, {n: float(v[self.idof[n]]) for n in DOFS}

    def read(self):
        tf = self.view.get_transforms().detach().cpu().numpy().astype(float)
        return {n: (tf[i, :3] - self.origin, _rot(tf[i, 3:7])) for n, i in self.vidx.items()}

    # ------------------------------------------------------------------ physics step
    def physics(self, dt):
        t0 = time.perf_counter()
        self.t += dt
        pl = self.plant
        q, _ = self.dofs()
        rz = pl.unwrap(q["turn"])
        s = q["slide"]
        # push-to-lock detent: snaps to the nearer end once past the middle of the travel
        if pl.out_end and s < L.LOCK_MID:
            pl.out_end = False
            pl.events.append(dict(t=round(self.t, 3), ev="knob pushed in", turns=round(pl.turns_cw, 4)))
        elif not pl.out_end and s > L.LOCK_MID:
            pl.out_end = True
            pl.events.append(dict(t=round(self.t, 3), ev="knob pulled out"))
        locked = not pl.out_end
        if locked and not pl.locked:
            pl.knob_anchor.anchor = rz  # lock ring engages where the knob is
        pl.locked = locked
        p_set = pl.p_set
        if locked:
            t_cap = L.T_LOCK
            tg_turn = pl.knob_anchor.update(rz, L.T_LOCK)
        else:
            t_cap = L.knob_torque(p_set)
            tg_turn = pl.knob_anchor.update(rz, t_cap)
        valve_deg = math.degrees(q["valve"])
        tg_valve = self.plant.lever_anchor.update(q["valve"], L.T_VALVE)
        # pressure plant and gauge
        pl.p = L.step_pressure(pl.p, p_set, L.bleed_frac(valve_deg), dt)
        pl.pg_prev = pl.pg
        pl.pg += (pl.p - pl.pg) * (1.0 - math.exp(-dt / L.TAU_GAUGE))
        targets = self.art.get_dof_position_targets().clone()
        targets[0, self.idof["slide"]] = L.KNOB_OUT if pl.out_end else 0.0
        targets[0, self.idof["turn"]] = float(tg_turn + (q["turn"] - rz))  # raw joint coordinate
        targets[0, self.idof["valve"]] = float(tg_valve)
        targets[0, self.idof["pointer"]] = float(L.gauge_angle(pl.pg))
        self.art.set_dof_position_targets(targets, self.ai)
        mf = np.array([L.F_DETENT, t_cap, L.T_VALVE, 0.5])
        if self.mf_now is None or np.max(np.abs(mf - self.mf_now)) > 1e-4:
            m = self.art.get_dof_max_forces().clone()
            for k, n in enumerate(DOFS):
                m[0, self.idof[n]] = float(mf[k])
            self.art.set_dof_max_forces(m, self.ai_cpu)
            self.mf_now = mf.copy()
        if self.trial_active:
            pk = pl.peaks
            pk["knob_nm"] = max(pk["knob_nm"], min(t_cap, K_KNOB * abs(rz - pl.knob_anchor.anchor)))
            pk["lever_nm"] = max(pk["lever_nm"], min(L.T_VALVE, K_LEVER * abs(q["valve"] - pl.lever_anchor.anchor)))
            tgt_s = L.KNOB_OUT if pl.out_end else 0.0
            pk["pull_n"] = max(pk["pull_n"], min(L.F_DETENT, K_SLIDE * abs(s - tgt_s)))
            self.min_p = min(self.min_p, pl.p)
            self.max_valve = max(self.max_valve, valve_deg)
            self.max_out = max(self.max_out, s)
            fm = self.cv.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
            if fm.size:
                nl = np.linalg.norm(fm, axis=-1)
                per = nl.sum(axis=-1)
                for i, n in enumerate(BODIES):
                    self.contact_step[n] = max(self.contact_step[n], float(per[i]))
                    self.contact_peak[n] = max(self.contact_peak[n], float(per[i]))
                    j = int(np.argmax(nl[i]))
                    if nl[i, j] > self.robot_link_peak.get(n, (0.0, ""))[0]:
                        self.robot_link_peak[n] = (float(nl[i, j]), "/".join(self.robot_links[j].split("/")[-2:]))
        self.hook_s += time.perf_counter() - t0

    # ------------------------------------------------------------------ reset
    def restore_authored(self, setup):
        """Environment reset only (never during a trial). The scene is authored in the first trial's start state (the
        drop's pose); the environment's own reset returns every joint to zero (knob in, lever closed, pointer at
        80 psi). This sets the plant for the start (line and gauge at p0, knob locked, valve closed) and the pointer
        to the start reading. A later trial in the same process keeps the drop where the scene was authored."""
        torch = self.torch
        self.setup = setup
        self.trial_active = False
        spec = L.setup_spec(setup)
        q = torch.zeros_like(self.art.get_dof_positions())
        q[0, self.idof["pointer"]] = float(L.gauge_angle(spec["p0"]))
        self.art.set_dof_positions(q, self.ai)
        self.art.set_dof_velocities(torch.zeros_like(q), self.ai)
        self.art.set_dof_position_targets(q.clone(), self.ai)
        self.mf_now = None
        self._init_state()

    # ------------------------------------------------------------------ scoring
    def measure(self):
        q, v = self.dofs()
        pl = self.plant
        gauge_psi = L.psi_of_angle(q["pointer"])
        gauge_rate = -math.degrees(v["pointer"]) * 160.0 / 270.0
        return dict(q=q, gauge_psi=gauge_psi, gauge_rate=gauge_rate, line_psi=pl.p, set_psi=pl.p_set,
                    knob_out_mm=q["slide"] * 1e3, valve_deg=math.degrees(q["valve"]), turns=pl.turns_cw,
                    locked=pl.locked)

    def control(self, environment, dt=0.05):
        if not self.trial_active:
            return
        self.steps += 1
        m = self.measure()
        if not all(math.isfinite(x) for x in m["q"].values()):
            self.nonfinite += 1
            STATE["nonfinite"] = self.nonfinite
            return
        contact = {n: self.contact_step[n] > 0.05 for n in BODIES}
        level = self.scorer.step(dt, m["gauge_psi"], m["gauge_rate"], m["line_psi"], m["set_psi"], m["knob_out_mm"],
                                 m["valve_deg"], contact)
        pl = self.plant
        STATE.update(
            score=level,
            level=level,
            steps=self.steps,
            nonfinite=self.nonfinite,
            detail=dict(
                self.scorer.last,
                turns=round(m["turns"], 4),
                locked=m["locked"],
                min_line_psi=round(self.min_p, 3),
                max_valve_deg=round(self.max_valve, 2),
                max_out_mm=round(self.max_out * 1e3, 3),
                peaks={k: round(v, 3) for k, v in pl.peaks.items()},
                events=pl.events[-6:],
                contact_n={n: round(v, 2) for n, v in self.contact_step.items()},
                contact_peak_n={n: round(v, 1) for n, v in self.contact_peak.items()},
                robot_link_peak={k: [round(v[0], 1), v[1]] for k, v in self.robot_link_peak.items()},
                setup=self.setup,
                hook_wall_s=round(self.hook_s, 2),
            ),
        )
        self.contact_step = {n: 0.0 for n in BODIES}


_RT: dict = {"rt": None, "sim": None, "trials": 0}


def runtime():
    return _RT["rt"]


def bench_state():
    """What an operator sees at the drop, for the scripted demo (read-only): the parts' poses, the gauge reading (psi,
    read from the needle) and its rate, the knob's travel (the orange lock band shows when it is out) and the lever's
    angle. The regulator's setpoint and spring rate are NOT given."""
    rt = _RT["rt"]
    if rt is None:
        return None
    m = rt.measure()
    out = dict(rt.read())
    out.update(gauge_psi=m["gauge_psi"], gauge_rate=m["gauge_rate"], knob_out_mm=m["knob_out_mm"],
               valve_deg=m["valve_deg"], knob_rz=rt.plant.rz)
    return out


def _arena_object(spec):
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class AirDropSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_bench)

    return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=AirDropSpawnerCfg(),
                  initial_pose=Pose(position_xyz=(0.0, 0.0, 0.0), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))




def _patch_scene(scene) -> None:
    if getattr(scene, "_rg_patched", False):
        return
    orig_task_object, orig_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, AirDrop):
            return _arena_object(spec)
        return orig_task_object(spec)

    def object_states(environment, names):
        from yam_arena.task_file import ObjectState

        plain = [n for n in names if n != "airdrop"]
        out = orig_states(environment, plain) if plain else {}
        if "airdrop" in names:
            rt = _RT["rt"]
            if rt:
                p, r = rt.read()["base"]
            else:
                p, r = L.reg_pose(L.setup_spec(_SPAWN["setup"]))
            z = np.zeros(3)
            out["airdrop"] = ObjectState(pos=np.asarray(p), quat_xyzw=_quat(r), lin_vel=z, ang_vel=z, low=p - 0.3,
                                       high=p + 0.3)
        return {n: out[n] for n in names}

    orig_build = scene.build_task_environment

    def build(embodiment, task, control_hz=None):
        environment = orig_build(embodiment, task, control_hz)
        if any(isinstance(o, AirDrop) for o in task.objects):
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
    scene._rg_patched = True


def _has_bench(emb):
    return any(isinstance(o, AirDrop) for o in (emb.task.objects if emb.task is not None else ()))


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_rg_patched", False):
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
            setup = (SETUP_BASE + _RT["trials"]) % len(L.NAMED_SETUPS)
        if rt is None:  # first reset: the scene is built (authored) in this start state
            _SPAWN["setup"] = setup
        if rt is not None:
            rt.restore_authored(setup)
        obs = base_reset(self, *a, **k)
        rt = _RT["rt"]
        if rt is not None:
            if rt.setup != setup:
                rt.restore_authored(setup)
                obs = base_reset(self, *a, **k)
            rt.begin_trial(_RT["trials"])
            _RT["trials"] += 1
        return obs

    cls._ensure_sim, cls._task_step, cls.reset = _ensure_sim, _task_step, reset
    cls._rg_patched = True


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
