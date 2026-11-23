"""balkit: the triple-beam balance bench for yam_arena (Isaac Lab-Arena / PhysX), plus the runtime hooks.

Bodies (custom-spawned; yam_arena task-file primitives are single boxes):
* one fixed-base articulation for the balance (``../src/layout.py`` has the dimensions and masses):
  - ``base``: the cast base with the pillar, the damper housing and the pointer's zero-mark plate (fixed to the bench);
  - ``beam``: the beam assembly (three beams, pointer, knife-edges, damper vane) on the fulcrum, a revolute joint about
    the knife-edge axis, with the magnetic damper (a velocity-proportional joint torque) and the pointer stops
    (joint limits). Its mass sits below the fulcrum (restoring torque, the balance's sensitivity) and is distributed
    so that the empty pan balances the riders at zero;
  - ``pan``: the 6 in stainless pan on its support, hanging on the pan-bearing knife-edge (revolute on the beam). The
    check link of the real balance (Roberval linkage) keeps the pan level: a PhysX mimic constraint ties the pan's
    joint angle to minus the beam's, so the pan only translates and its load acts at the bearing;
  - ``r500``, ``r100``, ``r10``: the riders, each a body on its beam (prismatic along the beam) whose weight acts on
    the beam where it sits. Notched riders: a detent at every notch (a stiff drive to the notch bottom capped at the
    force that lifts the rider out of the V, inside the V's half-width), elsewhere dry friction on the beam edge; the
    10 g poise: dry friction of its spring clip. Dry friction is an elasto-plastic anchor capped at the friction force.
* ``sample``: free rigid body, a sealed 250 mL jar standing on the pan; its mass (and centre of mass and inertia from
  its fill) is set for the start state at every reset.

Nothing is teleported during a trial: the riders move only when pushed (the robot's fingers), the beam turns only by
the torques on it. The scorer reads the beam angle and speed, the rider positions, the jar's pose and the robot-on-part
contact forces (PhysX contact force matrix between the robot's links and the balance parts: collider identity).
``Runtime.restore_authored`` places the parts only on an environment reset. The scripted demo READS state through
``bench_state()``.
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

PHYSICS_DT = float(os.environ.get("TB_PHYSICS_DT", str(1 / 480)))
CONTACT_OFFSET = 0.001
SETUP_BASE = int(os.environ.get("TB_SETUP", "0"))
_SPAWN = {"setup": SETUP_BASE}  # start state the scene is authored in (the first reset's; see _patch_embodiment)
FORCE_NAMED = "TB_SETUP" in os.environ

K_RIDER = dict(r500=6000.0, r100=3000.0, r10=800.0)  # N/m: detent / friction-anchor stiffness
D_RIDER = {k: 2.0 * math.sqrt(K_RIDER[k] * L.RIDER_MASS[k]) for k in L.RIDERS}
F_HOLD = {k: L.RIDER_MASS[k] * L.G * math.tan(math.radians(L.V_ALPHA_DEG) + math.atan(L.MU_RIDER))
          for k in ("r500", "r100")}  # push that walks a rider out of its V-notch
F_SLIDE = {"r500": L.MU_RIDER * L.RIDER_MASS["r500"] * L.G, "r100": L.MU_RIDER * L.RIDER_MASS["r100"] * L.G,
           "r10": L.POISE_FRICTION}
LINKS = ("base", "beam", "pan") + L.RIDERS
BODIES = ("beam", "pan") + L.RIDERS + ("sample", "base")  # contact-checked bodies

ALU, DARK, BLACK, STEEL, CAST, WHITE, RED, BRASS, CAPC, LABEL = (
    (0.78, 0.79, 0.80),
    (0.20, 0.20, 0.22),
    (0.03, 0.03, 0.03),
    (0.72, 0.73, 0.75),
    (0.10, 0.22, 0.42),
    (0.95, 0.95, 0.94),
    (0.80, 0.08, 0.06),
    (0.78, 0.62, 0.25),
    (0.15, 0.30, 0.65),
    (0.97, 0.93, 0.70),
)


class Bench(Box):
    """The whole bench, as one task-file object (subclasses Box only to pass yam_arena's checks)."""


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
                metal = 0.6 if color in (ALU, STEEL, BRASS) else 0.0
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
    bp, br = L.balance_pose(spec)
    F = L.FRICTION
    T = L.H_PIVOT  # fulcrum height above the bench (B frame z of the bench is -T)

    # ------------------------------------------------------------------ base (articulation root, fixed to the bench)
    B = f"{prim_path}/base"
    xform(B, bp, br)
    bprim = rigid(B, 2.2, com=(0.06, 0.0, -T + 0.02), inertia=(0.02, 0.06, 0.07))
    UsdPhysics.ArticulationRootAPI.Apply(bprim)
    ar = PhysxSchema.PhysxArticulationAPI.Apply(bprim)
    ar.CreateEnabledSelfCollisionsAttr(False)
    ar.CreateSolverPositionIterationCountAttr(32)
    ar.CreateSolverVelocityIterationCountAttr(8)
    ar.CreateSleepThresholdAttr(0.0)
    ar.CreateStabilizationThresholdAttr(0.0)
    fj = UsdPhysics.FixedJoint.Define(stage, f"{B}/fix_to_bench")
    fj.CreateBody1Rel().SetTargets([Sdf.Path(B)])
    fj.CreateLocalPos0Attr(Gf.Vec3f(*map(float, bp)))
    q = _quat(br)
    fj.CreateLocalRot0Attr(Gf.Quatf(float(q[3]), *map(float, q[:3])))
    fj.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    fj.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
    (x0, x1), (y0, y1) = L.BASE_X, L.BASE_Y
    mesh(f"{B}/slab", L.box_xyz((x0, x1), (y0, y1), (-T, -T + L.BASE_T)), CAST, friction=F["base"])
    mesh(f"{B}/pan_boss", L.box_xyz((-0.09, -0.02), (-0.03, 0.03), (-T + L.BASE_T, -T + L.BASE_T + 0.012)), CAST,
         friction=F["base"])
    mesh(f"{B}/pillar", L.box_xyz((-0.012, 0.012), (-0.045, 0.045), (-T + L.BASE_T, -0.016)), CAST, friction=F["base"])
    for sgn in (-1, 1):  # fulcrum bearing blocks either side of the beams
        mesh(f"{B}/bearing_{'lr'[sgn > 0]}", L.box_xyz((-0.008, 0.008), (sgn * 0.046, sgn * 0.054), (-0.016, 0.004)),
             STEEL, collide=False)
    # magnetic damper housing under the beam end, and the zero-mark plate (tower) at the pointer
    mesh(f"{B}/damper", L.box_xyz((0.215, 0.265), (-0.022, 0.022), (-T + L.BASE_T, -0.030)), CAST, friction=F["base"])
    for sgn in (-1, 1):
        mesh(f"{B}/magnet_{'lr'[sgn > 0]}", L.box_xyz((0.222, 0.258), (sgn * 0.006, sgn * 0.016), (-0.030, -0.018)),
             DARK, collide=False)
    tx = L.TOWER_X
    mesh(f"{B}/tower", L.box_xyz((tx, tx + 0.006), (-0.020, 0.020), (-T + L.BASE_T, 0.020)), CAST, friction=F["base"])
    mesh(f"{B}/scale_plate", L.box_xyz((tx - 0.0006, tx), (-0.016, 0.016), (-0.016, 0.016)), WHITE, collide=False)
    for k in range(-6, 7):  # scale lines every 2 mm, the zero line long and red
        z = 0.002 * k
        w = 0.016 if k == 0 else (0.010 if k % 2 == 0 else 0.006)
        mesh(f"{B}/scale_{k + 6}", L.box_xyz((tx - 0.0012, tx - 0.0005), (-w / 2, w / 2), (z - 0.0003, z + 0.0003)),
             RED if k == 0 else BLACK, collide=False)
    # levelling feet and zero-adjust knob (visual)
    for i, (fx, fy) in enumerate(((x0 + 0.02, y0 + 0.02), (x0 + 0.02, y1 - 0.02), (x1 - 0.02, y0 + 0.02),
                                  (x1 - 0.02, y1 - 0.02))):
        mesh(f"{B}/foot_{i}", L.cyl_z((fx, fy, -T - 0.002), 0.008, 0.004, n=12), BLACK, collide=False)
    mesh(f"{B}/zero_knob", L.cyl_z((-0.150, -0.060, -T + L.BASE_T + 0.006), 0.009, 0.012, n=16), DARK, collide=False)

    # ------------------------------------------------------------------ beam assembly (fulcrum revolute)
    BM = f"{prim_path}/beam"
    xform(BM, bp, br)
    rigid(BM, L.M_BEAM, com=(L.X_CB, 0.0, L.Z_CB), inertia=L.I_BEAM)
    bx0, bx1 = L.BEAM_X
    for k in L.RIDERS:
        y = L.BEAM_Y[k]
        mesh(f"{BM}/bar_{k}", L.box_xyz((bx0, L.S0 + L.SPAN + 0.012), (y - L.BEAM_W / 2, y + L.BEAM_W / 2),
                                        (L.BEAM_ZBOT, L.BEAM_ZTOP)), ALU, friction=F["beam"])
        # graduations: notch marks on the notched beams, 1 g lines (0.5 g short) on the poise beam
        if k in L.NOTCH:
            for n in range(L.n_notches(k) + 1):
                s = L.notch_s(k, n)
                mesh(f"{BM}/notch_{k}_{n}", L.box_xyz((s - 0.0012, s + 0.0012), (y - L.BEAM_W / 2 - 0.0004,
                                                                                 y + L.BEAM_W / 2 + 0.0004),
                                                      (L.BEAM_ZTOP - 0.0016, L.BEAM_ZTOP + 0.0002)), BLACK,
                     collide=False)
                mesh(f"{BM}/tick_{k}_{n}", L.box_xyz((s - 0.0005, s + 0.0005), (y - L.BEAM_W / 2 - 0.0006,
                                                                                y - L.BEAM_W / 2 - 0.0002),
                                                     (L.BEAM_ZTOP - 0.009, L.BEAM_ZTOP - 0.002)), BLACK, collide=False)
        else:
            for g in range(0, 21):
                s = L.s_of_grams(k, g * 0.5)
                tall = 0.007 if g % 2 == 0 else 0.004
                mesh(f"{BM}/tick_{k}_{g}", L.box_xyz((s - 0.0004, s + 0.0004), (y - L.BEAM_W / 2 - 0.0006,
                                                                                y - L.BEAM_W / 2 - 0.0002),
                                                     (L.BEAM_ZTOP - 0.002 - tall, L.BEAM_ZTOP - 0.002)), BLACK,
                     collide=False)
    # cross-members, knife-edge block, pointer, damper vane
    mesh(f"{BM}/cross_root", L.box_xyz((bx0, 0.012), (-0.034, 0.034), (-0.010, 0.002)), ALU, friction=F["beam"])
    mesh(f"{BM}/cross_end", L.box_xyz((L.S0 + L.SPAN + 0.008, L.S0 + L.SPAN + 0.016), (-0.034, 0.034),
                                      (-0.010, 0.002)), ALU, friction=F["beam"])
    mesh(f"{BM}/pointer", np.array([[L.S0 + L.SPAN + 0.012, -0.003, -0.004], [L.S0 + L.SPAN + 0.012, 0.003, -0.004],
                                    [L.S0 + L.SPAN + 0.012, -0.003, 0.004], [L.S0 + L.SPAN + 0.012, 0.003, 0.004],
                                    [L.L_POINTER, -0.0006, -0.0006], [L.L_POINTER, 0.0006, -0.0006],
                                    [L.L_POINTER, -0.0006, 0.0006], [L.L_POINTER, 0.0006, 0.0006]]), BLACK,
         collide=False)
    mesh(f"{BM}/vane", L.box_xyz((0.225, 0.255), (-0.0015, 0.0015), (-0.028, -0.010)), ALU, collide=False)
    mesh(f"{BM}/knife", L.box_xyz((-0.004, 0.004), (-0.046, 0.046), (-0.004, 0.002)), STEEL, collide=False)
    mesh(f"{BM}/pan_knife", L.box_xyz((-L.A_PAN - 0.004, -L.A_PAN + 0.004), (-0.030, 0.030), (-0.004, 0.002)), STEEL,
         collide=False)

    # ------------------------------------------------------------------ pan (on the pan bearing; kept level)
    PN = f"{prim_path}/pan"
    xform(PN, bp + br @ np.array([-L.A_PAN, 0.0, 0.0]), br)
    pc = L.PAN_X + L.A_PAN  # pan centre relative to the bearing
    rigid(PN, L.M_PAN, com=(pc, 0.0, L.PAN_Z - 0.006), inertia=(3.2e-4, 3.2e-4, 6.0e-4))
    mesh(f"{PN}/disc", L.cyl_z((pc, 0.0, L.PAN_Z - 0.0015), L.PAN_R, 0.003, n=40), STEEL, friction=F["pan"])
    for i, v in enumerate(L.ring_segments(L.PAN_R - 0.0015, L.PAN_R, L.PAN_Z - 0.003, L.PAN_Z + L.PAN_RIM, n=32)):
        mesh(f"{PN}/rim_{i}", v + np.array([pc, 0.0, 0.0]), STEEL, friction=F["pan"])
    mesh(f"{PN}/support", L.cyl_z((pc, 0.0, (L.PAN_Z - 0.003 - 0.004) / 2 - 0.002), 0.010, L.PAN_Z - 0.003, n=16),
         STEEL, collide=False)
    mesh(f"{PN}/spider", L.box_xyz((pc - 0.05, pc + 0.05), (-0.006, 0.006), (L.PAN_Z - 0.008, L.PAN_Z - 0.003)), STEEL,
         collide=False)
    mesh(f"{PN}/hanger", L.box_xyz((-0.005, 0.005), (-0.030, 0.030), (-0.006, 0.004)), STEEL, collide=False)

    # ------------------------------------------------------------------ riders (prismatic on their beams)
    for k in L.RIDERS:
        R = f"{prim_path}/{k}"
        y = L.BEAM_Y[k]
        xform(R, bp + br @ np.array([L.S0, y, 0.0]), br)
        m = L.RIDER_MASS[k]
        rigid(R, m, com=(0.0, 0.0, 0.0), inertia=(m * 2e-4, m * 2e-4, m * 2e-4))
        hx, hy, hz = L.RIDER_HANDLE[k]
        col = BRASS if k != "r10" else STEEL
        mesh(f"{R}/handle", L.box_xyz((-hx / 2, hx / 2), (-hy / 2, hy / 2), (L.HANDLE_Z0, L.HANDLE_Z0 + hz)), col,
             friction=F["rider"])
        # saddle over the beam (visual) and, for the notched riders, the hanging poise below it
        mesh(f"{R}/saddle_top", L.box_xyz((-hx / 2, hx / 2), (-0.004, 0.004), (L.BEAM_ZTOP, L.HANDLE_Z0)), col,
             collide=False)
        for sgn in (-1, 1):
            mesh(f"{R}/saddle_{'lr'[sgn > 0]}", L.box_xyz((-hx / 2, hx / 2), (sgn * 0.0024, sgn * 0.0036),
                                                          (L.BEAM_ZBOT - 0.002, L.BEAM_ZTOP)), col, collide=False)
        if k != "r10":
            hh = 0.016 if k == "r500" else 0.008
            mesh(f"{R}/poise", L.box_xyz((-hx / 2, hx / 2), (-0.0045, 0.0045), (L.BEAM_ZBOT - 0.002 - hh,
                                                                                L.BEAM_ZBOT - 0.002)), col,
                 collide=False)
        mesh(f"{R}/index", L.box_xyz((-0.0004, 0.0004), (-hy / 2 - 0.0004, hy / 2 + 0.0004),
                                     (L.HANDLE_Z0 + hz - 0.004, L.HANDLE_Z0 + hz + 0.0002)), BLACK, collide=False)

    # ------------------------------------------------------------------ sample jar (free body)
    S = f"{prim_path}/sample"
    sp, sr = L.sample_start(spec)
    xform(S, sp, sr)
    mj, zc, inert = L.jar_mass_props(spec["m"])
    rigid(S, mj, com=(0.0, 0.0, zc), inertia=inert)
    mesh(f"{S}/body", L.cyl_z((0, 0, -0.008), L.JAR_R, L.JAR_H - 0.016, n=28), WHITE, friction=F["sample"])
    mesh(f"{S}/cap", L.cyl_z((0, 0, L.JAR_H / 2 - 0.008), L.JAR_R - 0.001, 0.016, n=28), CAPC, friction=F["sample"])
    n = 10
    for i in range(n):  # paper label band on one side
        a0 = -0.9 + 1.8 * i / n
        a1 = -0.9 + 1.8 * (i + 1) / n
        pts = []
        for a in (a0, a1):
            c, s_ = math.cos(a), math.sin(a)
            for rr in (L.JAR_R + 0.0003, L.JAR_R + 0.0008):
                pts += [[rr * c, rr * s_, -0.030], [rr * c, rr * s_, 0.010]]
        mesh(f"{S}/label_{i}", np.array(pts), LABEL, collide=False)

    # ------------------------------------------------------------------ joints
    def joint(kind, name, parent, child, axis, pos0, lo, hi):
        cls = UsdPhysics.RevoluteJoint if kind == "rev" else UsdPhysics.PrismaticJoint
        j = cls.Define(stage, f"{prim_path}/{child}/{name}")
        j.CreateBody0Rel().SetTargets([Sdf.Path(f"{prim_path}/{parent}")])
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/{child}")])
        j.CreateAxisAttr(axis)
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, pos0)))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
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

    th = math.degrees(L.THETA_STOP)
    jb = joint("rev", "fulcrum", "base", "beam", "Y", (0, 0, 0), -th, th)
    drive(jb, "rev", 0.0, L.DAMP, 50.0)  # magnetic damper: torque = -DAMP x beam speed
    jp = joint("rev", "pan_bearing", "beam", "pan", "Y", (-L.A_PAN, 0, 0), -5.0, 5.0)
    mim = PhysxSchema.PhysxMimicJointAPI.Apply(jp.GetPrim(), "rotY")
    mim.CreateReferenceJointRel().SetTargets([jb.GetPath()])
    mim.CreateReferenceJointAxisAttr("rotY")
    mim.CreateGearingAttr(1.0)  # pan joint angle + beam angle = 0: the pan stays level (check link)
    mim.CreateOffsetAttr(0.0)
    for k in L.RIDERS:
        # joint position 0 = the rider on its zero line (the articulation's default state is all zeros)
        j = joint("pri", f"slide_{k}", "beam", k, "X", (L.S0, L.BEAM_Y[k], 0), -0.004, L.SPAN + 0.004)
        drive(j, "pri", K_RIDER[k], D_RIDER[k], 0.0)

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
    class BenchSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_bench)

    return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=BenchSpawnerCfg(),
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


class Runtime:
    """Rider detents and friction every physics step; scorer every control step."""

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
        root = "/World/envs/env_0/bench"
        self.root = root
        P = lambda n: f"{root}/{n}"  # noqa: E731
        self.art = self.simview.create_articulation_view(P("base"))
        self.dof_names = list(self.art.shared_metatype.dof_names)
        print("balkit: dofs", self.dof_names, flush=True)
        self.idof = {n: i for i, n in enumerate(self.dof_names)}
        self.view = self.simview.create_rigid_body_view([P(n) for n in LINKS] + [P("sample")])
        self.vidx = {n: i for i, n in enumerate(LINKS + ("sample",))}
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
        self.steps = 0
        self.t = 0.0
        self.hook_s = 0.0
        self.nonfinite = 0
        self.anchor = {k: Friction(K_RIDER[k], L.S0) for k in L.RIDERS}
        self.mode = {k: "notch" for k in L.RIDERS}
        self.contact_step = {n: 0.0 for n in BODIES}
        self.contact_peak = {n: 0.0 for n in BODIES}
        self.robot_link_peak = {}
        self.theta_hist = []
        STATE.update(score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, setup=self.setup)

    def begin_trial(self, trial):
        self._init_state()
        self.trial_active = True
        STATE.update(trial=trial, setup=self.setup, start=L.setup_spec(self.setup))

    # ------------------------------------------------------------------ reading
    def _rider_dofs(self):
        return {k: self.idof[f"slide_{k}"] for k in L.RIDERS}

    def q(self):
        q = self.art.get_dof_positions().detach().cpu().numpy()[0].astype(float)
        v = self.art.get_dof_velocities().detach().cpu().numpy()[0].astype(float)
        rd = self._rider_dofs()
        out = dict(theta=float(q[self.idof["fulcrum"]]), omega=float(v[self.idof["fulcrum"]]),
                   pan=float(q[self.idof["pan_bearing"]]))
        for k in L.RIDERS:
            out[k] = float(q[rd[k]]) + L.S0  # m from the fulcrum
            out["v_" + k] = float(v[rd[k]])
        return out

    def read(self):
        tf = self.view.get_transforms().detach().cpu().numpy().astype(float)
        return {n: (tf[i, :3] - self.origin, _rot(tf[i, 3:7])) for n, i in self.vidx.items()}

    # ------------------------------------------------------------------ physics step
    def physics(self, dt):
        t0 = time.perf_counter()
        self.t += dt
        qd = self.q()
        rd = self._rider_dofs()
        tg = {}
        mf = {}
        for k in L.RIDERS:
            s = qd[k]
            if k in L.NOTCH:
                n, off = L.notch_state(k, s)
                if abs(off) < L.NOTCH_CAPTURE:
                    tg[k], mf[k] = L.notch_s(k, n), F_HOLD[k]
                    self.anchor[k].anchor = s
                    self.mode[k] = "notch"
                    continue
            self.mode[k] = "slide"
            tg[k], mf[k] = self.anchor[k].update(s, F_SLIDE[k]), F_SLIDE[k]
        targets = self.art.get_dof_position_targets().clone()
        for k in L.RIDERS:
            targets[0, rd[k]] = float(tg[k]) - L.S0
        self.art.set_dof_position_targets(targets, self.ai)
        vec = np.array([mf[k] for k in L.RIDERS])
        if self.mf_now is None or np.max(np.abs(vec - self.mf_now)) > 1e-6:
            m = self.art.get_dof_max_forces().clone()
            for k in L.RIDERS:
                m[0, rd[k]] = float(mf[k])
            self.art.set_dof_max_forces(m, self.ai_cpu)
            self.mf_now = vec.copy()
        if self.trial_active:
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
        """Environment reset only (never during a trial). The scene is authored in the first trial's start state
        (balance pose, jar pose and mass); the environment's own reset returns every joint to zero (beam level, then
        it falls onto its stop; riders on their zero lines). For a later trial in the same process this also sets the
        jar's mass for the new start and stands it on the pan at the new start's offset (the balance itself stays
        where the scene was authored: its base is fixed to the bench)."""
        torch = self.torch
        self.setup = setup
        self.trial_active = False
        spec = L.setup_spec(setup)
        q = torch.zeros_like(self.art.get_dof_positions())
        rd = self._rider_dofs()
        q[0, self.idof["fulcrum"]] = -L.THETA_STOP
        q[0, self.idof["pan_bearing"]] = L.THETA_STOP
        for k in L.RIDERS:
            q[0, rd[k]] = 0.0  # zero line
        self.art.set_dof_positions(q, self.ai)
        self.art.set_dof_velocities(torch.zeros_like(q), self.ai)
        self.art.set_dof_position_targets(q.clone(), self.ai)
        self.mf_now = None
        k = self.vidx["sample"]
        ids_cpu = torch.tensor([k], dtype=torch.int32)
        mj, zc, inert = L.jar_mass_props(spec["m"])
        masses = self.view.get_masses().clone()
        masses[k] = float(mj)
        self.view.set_masses(masses, ids_cpu)
        coms = self.view.get_coms().clone()
        coms[k, :3] = torch.tensor([0.0, 0.0, zc], dtype=coms.dtype)
        coms[k, 3:7] = torch.tensor([0.0, 0.0, 0.0, 1.0], dtype=coms.dtype)
        self.view.set_coms(coms, ids_cpu)
        ins = self.view.get_inertias().clone()
        ins[k] = torch.tensor([inert[0], 0, 0, 0, inert[1], 0, 0, 0, inert[2]], dtype=ins.dtype)
        self.view.set_inertias(ins, ids_cpu)
        bp, br = self.read()["base"]
        sp, sr = L.sample_start(spec, (bp, br))
        tf = self.view.get_transforms().clone()
        tf[k, :3] = torch.tensor(np.asarray(sp, float) + self.origin, dtype=tf.dtype)
        tf[k, 3:7] = torch.tensor(_quat(sr), dtype=tf.dtype)
        ids = torch.tensor([k], dtype=torch.int32, device=self.dev)
        self.view.set_transforms(tf, ids)
        self.view.set_velocities(torch.zeros_like(self.view.get_velocities()), ids)
        self._init_state()

    # ------------------------------------------------------------------ scoring
    def measure(self):
        poses = self.read()
        qd = self.q()
        pp, pr = poses["pan"]
        sp, sr = poses["sample"]
        pan_top = pp + pr @ np.array([L.PAN_X + L.A_PAN, 0.0, L.PAN_Z])
        rel = pr.T @ (sp - pan_top)
        upright = float(sr[2, 2]) > math.cos(math.radians(12.0))
        on_pan = L.on_pan(rel) and upright
        s = {k: qd[k] for k in L.RIDERS}
        pointer_mm = -qd["theta"] * L.L_POINTER * 1e3  # + = pointer above the zero mark (pan side heavy)
        speed = abs(qd["omega"]) * L.L_POINTER * 1e3
        pan_tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(pr[2, 2])))))
        return dict(poses=poses, q=qd, s=s, pointer_mm=pointer_mm, speed_mm_s=speed, on_pan=on_pan,
                    reading=L.reading_g(s), jar_rel_mm=(rel * 1e3).round(2).tolist(), pan_tilt=pan_tilt)

    def control(self, environment, dt=0.05):
        if not self.trial_active:
            return
        self.steps += 1
        m = self.measure()
        if not all(np.all(np.isfinite(p)) for p, _ in m["poses"].values()):
            self.nonfinite += 1
            STATE["nonfinite"] = self.nonfinite
            return
        contact = {n: self.contact_step[n] > 0.05 for n in BODIES}
        spec = L.setup_spec(self.setup)
        level = self.scorer.step(dt, m["pointer_mm"], m["speed_mm_s"], m["s"], m["on_pan"], contact, spec["m"])
        STATE.update(
            score=level,
            level=level,
            steps=self.steps,
            nonfinite=self.nonfinite,
            detail=dict(
                self.scorer.last,
                theta_mrad=round(m["q"]["theta"] * 1e3, 4),
                pan_tilt_deg=round(m["pan_tilt"], 4),
                riders_mm={k: round(v * 1e3, 3) for k, v in m["s"].items()},
                modes=dict(self.mode),
                jar_rel_mm=m["jar_rel_mm"],
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
    """What an operator sees on the bench, for the scripted demo (read-only): the parts' poses, the pointer against
    its scale (mm, + above the zero mark) and its speed, and the riders' positions on their beams (m from the
    fulcrum, as read against the beam graduations)."""
    rt = _RT["rt"]
    if rt is None:
        return None
    m = rt.measure()
    out = dict(m["poses"])
    out.update(pointer_mm=m["pointer_mm"], speed_mm_s=m["speed_mm_s"], riders=dict(m["s"]), on_pan=m["on_pan"],
               modes=dict(rt.mode))
    return out


def _patch_scene(scene) -> None:
    if getattr(scene, "_tb_patched", False):
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
            if rt:
                p, r = rt.read()["base"]
            else:
                p, r = L.balance_pose(L.setup_spec(_SPAWN["setup"]))
            z = np.zeros(3)
            out["bench"] = ObjectState(pos=np.asarray(p), quat_xyzw=_quat(r), lin_vel=z, ang_vel=z, low=p - 0.3,
                                       high=p + 0.3)
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
    scene._tb_patched = True


def _has_bench(emb):
    return any(isinstance(o, Bench) for o in (emb.task.objects if emb.task is not None else ()))


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_tb_patched", False):
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
    cls._tb_patched = True


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
