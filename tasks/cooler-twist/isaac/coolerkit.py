"""coolerkit: the PC test bench with an AM4 processor and an AMD Wraith Prism cooler for yam_arena (Isaac Lab-Arena /
PhysX), plus the runtime hooks.

Bodies (custom-spawned; yam_arena task-file primitives are single boxes), all in ONE articulation:
* ``tbench``: floating root (open-air test bench: steel tray on rubber feet, micro-ATX board, AM4 socket, the two AM4
  retention brackets with their lugs, memory slots and VRM heatsinks), 4.5 kg standing on the table.
* ``cpu``: the processor, on a chain of six joints from the bench (``px``, ``py``, ``pz``, ``prx``, ``pry``, ``prz``;
  zero = seated in the socket).
* ``cooler``: the Wraith Prism, on a chain of six joints from the processor (``bx``, ``by``, ``bz``, ``brx``, ``bry``,
  ``brz``; zero = seated on the processor's lid). Massless-in-effect links (1 g) carry both chains.
* ``lever``: the clip's cam lever, revolute ``lev`` on the cooler (about the bench +y axis; 0 = locked).
Static: the anti-static mat. Articulation self-collision is on (the cooler rests on the lid, the processor on the
socket; the robot, the bench and the parts all collide).

Mechanism (runtime, every physics step, through articulation drive targets and force caps; nothing is teleported):
* clip: locked while the lever is below 55 deg (the cam holds it down with up to 0.6 N m, rising over the first
  20 deg): the cooler and the processor are held to the bench (hooks on the lugs, clip through the fin slot). Past
  55 deg the cam is over its high point: the clip goes slack and its spring nudges the lever onto its open stop
  (100 deg); pushed back below 30 deg it locks again.
* processor: seated, its pins in the closed ZIF socket: held laterally and in rotation; held down against a pull of
  up to F_ZIF = 25 N. Pulled 1 mm out it is yanked (pins out of the socket, bent): every processor joint goes free and
  the processor now hangs on whatever holds it (the paste on the cooler).
* paste film (cooler on the processor, clip slack): bonded: carries up to F_TENS = 110 N in tension (more than the
  ZIF hold, so a straight pull takes the processor with it), 4 N m of peel and a twist torque up to the seeded break
  torque (0.60-1.40 N m; lateral shear up to the same stress, break torque / 14.5 mm). Twist (or lateral) slip beyond
  that is plastic (elasto-plastic anchors); after 6 deg of accumulated slip the film is sheared: then it holds only a
  2 N tack, a 0.05 N m / 3 N running resistance, and parts once the faces are 1.5 mm apart (free).
* with the cam slack, the two guided clip ends must each be moved off their lug before the paste can shear.

Runtime hooks (installed on import; nothing in yam_arena is edited): after every physics step the mechanism; after
every control step the scorer (robot-on-part contact from PhysX contact forces between the robot's links and the
bench / processor / cooler / lever: collider identity). ``Runtime.restore_authored`` places the parts only on an
environment reset. The scripted demo READS state through ``bench_state()``.
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

PHYSICS_DT = float(os.environ.get("CT_PHYSICS_DT", str(1 / 480)))
CONTACT_OFFSET = 0.001
SETUP_BASE = int(os.environ.get("CT_SETUP", "0"))
FORCE_NAMED = "CT_SETUP" in os.environ

K_LIN, D_LIN = 50000.0, 40.0  # N / m, N s / m: all slides (processor and paste)
K_ANG, D_ANG = 200.0, 0.2  # N m / rad: tilt joints
K_TW, D_TW = 200.0, 0.05  # N m / rad: twist joints (elastic play of the friction anchors: cap / K_TW)
CPU_CHAIN = ("p_x", "p_y", "p_z", "p_rx", "p_ry")
COOLER_CHAIN = ("b_x", "b_y", "b_z", "b_rx", "b_ry")
CPU_DOFS = ("px", "py", "pz", "prx", "pry", "prz")
BOND_DOFS = ("bx", "by", "bz", "brx", "bry", "brz")
HOOK_NAMES = tuple(name for name, _ in L.HOOKS)
DOFS = CPU_DOFS + BOND_DOFS + ("lev",) + HOOK_NAMES + tuple(n + "_lift" for n in HOOK_NAMES)
BODIES = ("tbench", "cpu", "cooler", "lever") + HOOK_NAMES
CPU_HOLD = (500.0, 500.0, None, 20.0, 20.0, 20.0)  # seated: pins hold laterally / in rotation; pz cap = F_ZIF

PCB, SOCKC, BLACK, STEEL, GREEN, IHS, COPPER, ALU, DARK, GREY, MATC, RUBBER, WHITE, RED, BLUE = (
    (0.15, 0.13, 0.13),
    (0.90, 0.88, 0.80),
    (0.04, 0.04, 0.04),
    (0.62, 0.64, 0.66),
    (0.10, 0.45, 0.22),
    (0.78, 0.79, 0.80),
    (0.80, 0.45, 0.25),
    (0.72, 0.74, 0.76),
    (0.15, 0.15, 0.16),
    (0.45, 0.46, 0.48),
    (0.35, 0.37, 0.40),
    (0.06, 0.06, 0.06),
    (0.95, 0.95, 0.95),
    (0.85, 0.08, 0.06),
    (0.10, 0.25, 0.85),
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
        px.CreateAngularDampingAttr(0.05)
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

    spec = L.setup_spec(SETUP_BASE)
    bp, br = L.bench_pose(spec)
    o_cpu = L.ihs_station(spec)  # processor and cooler chain origin (lid top centre, seated)

    # ------------------------------------------------------------------ test bench (articulation root)
    B = f"{prim_path}/tbench"
    xform(B, bp, br)
    bprim = rigid(B, L.BENCH_MASS, com=L.BENCH_COM, inertia=(0.035, 0.035, 0.065))
    UsdPhysics.ArticulationRootAPI.Apply(bprim)
    ar = PhysxSchema.PhysxArticulationAPI.Apply(bprim)
    ar.CreateEnabledSelfCollisionsAttr(True)
    ar.CreateSolverPositionIterationCountAttr(32)
    ar.CreateSolverVelocityIterationCountAttr(4)
    ar.CreateSleepThresholdAttr(0.0)
    ar.CreateStabilizationThresholdAttr(0.0)
    fb = L.FRICTION["bench"]
    (tx0, tx1), (ty0, ty1) = L.TRAY
    for i, (fx, fy) in enumerate(((tx0 + 0.02, ty0 + 0.02), (tx0 + 0.02, ty1 - 0.02), (tx1 - 0.02, ty0 + 0.02),
                                  (tx1 - 0.02, ty1 - 0.02))):
        mesh(f"{B}/foot_{i}", L.cyl_z((fx, fy, L.TRAY_Z[0] / 2), 0.012, L.TRAY_Z[0], n=12), RUBBER, friction=fb)
    mesh(f"{B}/tray", L.box_xyz((tx0, tx1), (ty0, ty1), L.TRAY_Z), STEEL, friction=fb)
    (bx0, bx1), (by0, by1) = L.BOARD
    mesh(f"{B}/board", L.box_xyz((bx0, bx1), (by0, by1), L.BOARD_Z), PCB, friction=fb)
    for i, (sx, sy) in enumerate(((bx0 + 0.006, by0 + 0.006), (bx0 + 0.006, by1 - 0.006), (bx1 - 0.006, by0 + 0.006),
                                  (bx1 - 0.006, by1 - 0.006))):
        mesh(f"{B}/standoff_{i}", L.cyl_z((sx, sy, (L.TRAY_Z[1] + L.BOARD_Z[0]) / 2), 0.0028,
                                         L.BOARD_Z[0] - L.TRAY_Z[1], n=6), (0.8, 0.7, 0.3), collide=False)
    T = L.BOARD_TOP
    (sx0, sx1), (sy0, sy1), sz0, sz1 = L.SOCKET
    mesh(f"{B}/socket", L.box_xyz((sx0, sx1), (sy0, sy1), (sz0, sz1)), SOCKC, friction=fb)
    # ZIF lever, closed, along the socket's -y side
    mesh(f"{B}/zif_lever", L.box_xyz((sx0 - 0.004, sx1), (sy0 - 0.004, sy0 - 0.0015), (T + 0.001, T + 0.004)), STEEL,
         collide=False)
    mesh(f"{B}/zif_tip", L.box_xyz((sx0 - 0.006, sx0 - 0.002), (sy0 - 0.010, sy0 - 0.0015), (T + 0.001, T + 0.005)),
         STEEL, collide=False)
    dx, dy, dz = L.BRACKET
    lx, ly, lz = L.LUG
    for k, sgn in enumerate((-1.0, 1.0)):
        cx = sgn * L.BRACKET_X
        mesh(f"{B}/bracket_{k}", L.box((cx, 0.0, T + dz / 2), (dx, dy, dz)), BLACK, friction=fb)
        mesh(f"{B}/bracket_arm_{k}", L.box((cx - sgn * 0.012, 0.0, T + 0.002), (0.016, dy + 0.008, 0.004)), BLACK,
             collide=False)
        mesh(f"{B}/lug_{k}", L.box((cx + sgn * (dx / 2 + lx / 2), 0.0, T + 0.007), (lx, ly, lz)), BLACK, friction=fb)
    # Major obstacles around the socket, traced approximately from the ASRock
    # manual (1.3). Component heights are estimates for an unpopulated board.
    for i, y in enumerate((-0.058, -0.069)):
        mesh(f"{B}/dimm_{i}", L.box_xyz((-0.079, 0.083), (y - 0.003, y + 0.003), (T, T + 0.009)), BLACK, friction=fb)
    mesh(f"{B}/pcie_x1", L.box_xyz((-0.078, -0.068), (0.038, 0.063), (T, T + 0.011)), BLACK, friction=fb)
    mesh(f"{B}/pcie_x16", L.box_xyz((-0.099, -0.089), (-0.044, 0.064), (T, T + 0.011)), BLACK, friction=fb)
    mesh(f"{B}/atx_power", L.box_xyz((-0.029, 0.029), (-0.089, -0.077), (T, T + 0.013)), BLACK, friction=fb)
    mesh(f"{B}/chipset", L.box_xyz((-0.130, -0.108), (-0.040, -0.016), (T, T + 0.008)), DARK, friction=fb)
    mesh(f"{B}/battery", L.cyl_z((-0.037, 0.062, T + 0.003), 0.010, 0.006), STEEL, friction=fb)
    for i, (xlo, xhi) in enumerate(((-0.073, -0.056), (-0.053, -0.032), (-0.031, -0.017),
                                    (-0.010, -0.004), (0.002, 0.024), (0.037, 0.056), (0.070, 0.085))):
        mesh(f"{B}/rear_io_{i}", L.box_xyz((xlo, xhi), (0.089, 0.108), (T, T + 0.015)), STEEL, friction=fb)

    # ------------------------------------------------------------------ chains
    def joint(kind, name, parent, child, axis, pos0):
        cls = UsdPhysics.RevoluteJoint if kind == "rev" else UsdPhysics.PrismaticJoint
        j = cls.Define(stage, f"{prim_path}/{child}/{name}")
        j.CreateBody0Rel().SetTargets([parent])
        j.CreateBody1Rel().SetTargets([Sdf.Path(f"{prim_path}/{child}")])
        j.CreateAxisAttr(axis)
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, pos0)))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        return j

    def drive(j, kind, stiff, damp):
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular" if kind == "rev" else "linear")
        d.CreateTypeAttr("force")
        scale = math.pi / 180 if kind == "rev" else 1.0  # USD angular drives are per degree
        d.CreateStiffnessAttr(float(stiff * scale))
        d.CreateDampingAttr(float(damp * scale))
        d.CreateMaxForceAttr(0.0)
        d.CreateTargetPositionAttr(0.0)

    def chain(parent_path, links, child, names, pos0, lin_lim):
        kinds = ("pri", "pri", "pri", "rev", "rev", "rev")
        axes = ("X", "Y", "Z", "X", "Y", "Z")
        parents = [Sdf.Path(parent_path)] + [Sdf.Path(f"{prim_path}/{n}") for n in links]
        childs = list(links) + [child]
        for k in range(6):
            j = joint(kinds[k], names[k], parents[k], childs[k], axes[k], pos0 if k == 0 else (0, 0, 0))
            if kinds[k] == "pri":
                lo, hi = lin_lim[k]
                j.CreateLowerLimitAttr(lo)
                j.CreateUpperLimitAttr(hi)
                drive(j, "pri", K_LIN, D_LIN)
            elif k < 5:
                j.CreateLowerLimitAttr(-175.0)
                j.CreateUpperLimitAttr(175.0)
                drive(j, "rev", K_ANG, D_ANG)
            else:
                drive(j, "rev", K_TW, D_TW)

    for nme in CPU_CHAIN + ("cpu",) + COOLER_CHAIN + ("cooler",):
        xform(f"{prim_path}/{nme}", o_cpu, br)
    for nme in CPU_CHAIN + COOLER_CHAIN:
        rigid(f"{prim_path}/{nme}", 0.005, inertia=(1e-6, 1e-6, 1e-6))
    # processor (cpu frame: origin = lid top centre)
    Cp = f"{prim_path}/cpu"
    rigid(Cp, L.CPU_MASS, com=(0.0, 0.0, -0.003), inertia=(6e-6, 6e-6, 1.2e-5))
    sxs, sys_, szs = L.CPU_SUB
    ix, iy, iz = L.CPU_IHS
    mesh(f"{Cp}/substrate", L.box((0.0, 0.0, -iz - szs / 2 + 0.00005), (sxs, sys_, szs - 0.0001)), GREEN,
         friction=L.FRICTION["cpu"])
    mesh(f"{Cp}/ihs", L.box((0.0, 0.0, -iz / 2), (ix, iy, iz)), IHS, friction=L.FRICTION["cpu"])
    mesh(f"{Cp}/mark", L.box((-sxs / 2 + 0.003, -sys_ / 2 + 0.003, -iz - 0.0001), (0.003, 0.003, 0.0004)),
         (0.85, 0.7, 0.2), collide=False)
    chain(B, CPU_CHAIN, "cpu", CPU_DOFS, (0.0, 0.0, L.H_IHS),
          ((-0.6, 0.6), (-0.6, 0.6), (-0.01, 0.6)))
    # cooler (cooler frame: origin = base bottom centre)
    Co = f"{prim_path}/cooler"
    rigid(Co, L.COOLER_MASS, com=L.COOLER_COM, inertia=L.COOLER_INERTIA)
    fc = L.FRICTION["cooler"]
    bx_, by_, bz_ = L.BASE
    mesh(f"{Co}/base", L.box((0.0, 0.0, bz_ / 2), (bx_, by_, bz_)), COPPER, friction=fc)
    mesh(f"{Co}/fins", L.cyl_z((0.0, 0.0, (L.FIN_Z[0] + L.FIN_Z[1]) / 2), L.FIN_R, L.FIN_Z[1] - L.FIN_Z[0], n=32),
         ALU, friction=fc)
    nfin = 16
    for i in range(nfin):  # visual fin edges (slightly proud of the collider)
        z = L.FIN_Z[0] + 0.002 + i * (L.FIN_Z[1] - L.FIN_Z[0] - 0.004) / (nfin - 1)
        mesh(f"{Co}/fin_{i}", L.cyl_z((0.0, 0.0, z), L.FIN_R + 0.0008, 0.0007, n=32), (0.85, 0.86, 0.88),
             collide=False)
    for i in range(4):  # heat pipes (visual)
        a = math.radians(45 + 90 * i)
        mesh(f"{Co}/pipe_{i}", L.cyl_z((0.030 * math.cos(a), 0.030 * math.sin(a), 0.030), 0.0035, 0.040, n=10),
             COPPER, collide=False)
    mesh(f"{Co}/shroud", L.cyl_z((0.0, 0.0, (L.SHROUD_Z[0] + L.SHROUD_Z[1]) / 2), L.SHROUD_R,
                                 L.SHROUD_Z[1] - L.SHROUD_Z[0], n=40), BLACK, friction=fc)
    zt = L.SHROUD_Z[1]
    # The fan and RGB leads are disconnected at the start of this service task.
    # The inactive plastic light guide reflects the station lights.
    for i, v in enumerate(L.ring_segments(L.SHROUD_R - 0.006, L.SHROUD_R - 0.0005, zt, zt + 0.0008, n=24)):
        mesh(f"{Co}/light_guide_{i}", v, (0.52, 0.54, 0.54), collide=False)
    mesh(f"{Co}/fan_well", L.cyl_z((0.0, 0.0, zt + 0.0003), L.SHROUD_R - 0.007, 0.0008, n=40), DARK, collide=False)
    for i in range(7):  # fan blades (visual)
        a = 2 * math.pi * i / 7
        c, s = math.cos(a), math.sin(a)
        pts = []
        for r_, da in ((0.020, -0.15), (0.043, -0.05), (0.043, 0.25), (0.020, 0.30)):
            pts += [[r_ * math.cos(a + da), r_ * math.sin(a + da), zt + 0.0008],
                    [r_ * math.cos(a + da), r_ * math.sin(a + da), zt + 0.0016]]
        mesh(f"{Co}/blade_{i}", np.array(pts), (0.10, 0.10, 0.11), collide=False)
    mesh(f"{Co}/hub", L.cyl_z((0.0, 0.0, zt + 0.0015), 0.019, 0.0016, n=24), (0.12, 0.12, 0.13), collide=False)
    mesh(f"{Co}/logo", L.box((0.0, 0.0, zt + 0.0025), (0.016, 0.004, 0.0004)), WHITE, collide=False)
    hx = L.HOOK_X
    mesh(f"{Co}/clip_strap", L.box_xyz((-hx, hx), (-0.006, 0.006), (0.009, 0.011)), STEEL, collide=False)
    for name, sign in L.HOOKS:
        path = f"{prim_path}/{name}"
        origin = np.array([sign * hx, 0.0, 0.0])
        carrier = name + "_carrier"
        xform(f"{prim_path}/{carrier}", o_cpu + br @ origin, br)
        rigid(f"{prim_path}/{carrier}", 0.001, inertia=(1e-7, 1e-7, 1e-7))
        xform(path, o_cpu + br @ origin, br)
        rigid(path, 0.006, inertia=(6e-7, 6e-7, 5e-7))
        # The open loop surrounds the lug. Its crossbar sits below the lug at reset.
        for k, y in enumerate((-0.0085, 0.0085)):
            mesh(f"{path}/side_{k}", L.box((0, y, 0.004), (0.002, 0.0025, 0.023)), STEEL)
        mesh(f"{path}/toe", L.box((-sign * 0.002, 0, -0.0072), (0.006, 0.019, 0.0016)), STEEL)
        mesh(f"{path}/bridge", L.box((0, 0.006, 0.015), (0.003, 0.030, 0.004)), STEEL)
        ex, ey, ez = L.HOOK_EAR
        mesh(f"{path}/ear", L.box((sign * ex, ey, ez), L.HOOK_SIZE), STEEL, friction=0.9)
        slide = joint("pri", name, Sdf.Path(Co), carrier, "X", origin)
        # The toe must first clear the underside of the lug horizontally. The
        # unloaded steel loop then springs up and travels with the cooler.
        lift = joint("pri", name + "_lift", Sdf.Path(f"{prim_path}/{carrier}"), name, "Z", (0, 0, 0))
        lift.CreateLowerLimitAttr(0.0)
        lift.CreateUpperLimitAttr(0.011)
        drive(lift, "pri", 4500.0, 2.0)
        slide.CreateLowerLimitAttr(min(0.0, sign * L.HOOK_TRAVEL))
        slide.CreateUpperLimitAttr(max(0.0, sign * L.HOOK_TRAVEL))
        drive(slide, "pri", 3500.0, 2.0)
    mesh(f"{Co}/lever_boss", L.box((L.LEV_PIVOT[0], 0.0, L.LEV_PIVOT[2]), (0.008, 0.016, 0.010)), STEEL, collide=False)
    chain(Cp, COOLER_CHAIN, "cooler", BOND_DOFS, (0.0, 0.0, 0.0), ((-0.6, 0.6), (-0.6, 0.6), (-0.3, 0.6)))
    # cam lever (lever frame: origin = pivot, lever along -x when locked)
    Lv = f"{prim_path}/lever"
    xform(Lv, o_cpu + br @ np.array(L.LEV_PIVOT), br)
    rigid(Lv, L.LEV_MASS, com=(-L.LEV_LEN * 0.6, 0.0, 0.004), inertia=(2e-6, 1.2e-5, 1.2e-5))
    w, t = L.LEV_BAR
    mesh(f"{Lv}/bar", L.box((-L.LEV_LEN / 2, 0.0, 0.0), (L.LEV_LEN, w, t)), STEEL, friction=L.FRICTION["lever"])
    tl, tt, th = L.LEV_TAB
    mesh(f"{Lv}/tab", L.box((-L.LEV_LEN + tl / 2, 0.0, 0.0), (tl, tt, th)), BLACK,
         friction=L.FRICTION["lever"])
    jl = joint("rev", "lev", Sdf.Path(Co), "lever", "Y", L.LEV_PIVOT)
    jl.CreateLowerLimitAttr(-2.0)
    jl.CreateUpperLimitAttr(L.LEV_OPEN_DEG + 2.0)
    drive(jl, "rev", L.LEV_K, 0.01)

    # ------------------------------------------------------------------ static: anti-static mat
    S = f"{prim_path}/static"
    xform(S, (0.0, 0.0, 0.0), np.eye(3))
    (mx0, mx1), (my0, my1), mt = L.MAT
    mesh(f"{S}/mat", L.box_xyz((mx0, mx1), (my0, my1), (L.TOP, L.TOP + mt)), MATC, friction=L.FRICTION["mat"])
    mesh(f"{S}/mat_snap", L.cyl_z((mx0 + 0.012, my1 - 0.012, L.TOP + mt + 0.0006), 0.005, 0.0012, n=10), STEEL,
         collide=False)

    _SPEC["paths"] = {n: f"{prim_path}/{n}" for n in BODIES}
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


class Anchor:
    """Elasto-plastic friction anchor for an articulation dof driven by a stiff position drive capped at ``cap``."""

    def __init__(self, stiff, q0=0.0):
        self.stiff = stiff
        self.anchor = q0

    def update(self, q, cap):
        """Returns the anchor and how far it moved this step (the plastic slip)."""
        lim = cap / self.stiff
        old = self.anchor
        if q - self.anchor > lim:
            self.anchor = q - lim
        elif self.anchor - q > lim:
            self.anchor = q + lim
        return self.anchor, abs(self.anchor - old)


class Mech:
    """State of the clip, the processor and the paste film (see the module docstring)."""

    def __init__(self, tb):
        self.tb = float(tb)
        self.clip = "locked"
        self.cpu = "seated"
        self.bond = "bonded"
        self.released = set()
        self.slip = 0.0  # accumulated shear slip (rad equivalent)
        self.lock = np.zeros(6)  # bond targets while the clip is locked
        self.a_rz = Anchor(K_TW)
        self.a_x = Anchor(K_LIN)
        self.a_y = Anchor(K_LIN)
        self.events = []
        self.max_tw = 0.0
        self.max_pull = 0.0


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
    """Mechanism every physics step; scorer every control step."""

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
        self.art = self.simview.create_articulation_view(P("tbench"))
        self.dof_names = list(self.art.shared_metatype.dof_names)
        print("coolerkit: dofs", self.dof_names, flush=True)
        self.idof = {n: self.dof_names.index(n) for n in DOFS}
        self.view = self.simview.create_rigid_body_view([P(n) for n in BODIES])
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
        self.setup = SETUP_BASE
        self.trial_active = False
        self.mf_now = None
        self.probe_wrench = None  # development probes only (dev/probe.py); never set in a trial
        self._init_state()

    def _init_state(self):
        self.scorer = Scorer()
        self.mech = Mech(L.setup_spec(self.setup)["tb"])
        self.steps = 0
        self.t = 0.0
        self.hook_s = 0.0
        self.nonfinite = 0
        self.contact_step = {n: 0.0 for n in BODIES}
        self.contact_peak = {n: 0.0 for n in BODIES}
        self.robot_link_peak = {}
        STATE.update(score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, setup=self.setup)

    def begin_trial(self, trial):
        self._init_state()
        self.trial_active = True
        STATE.update(trial=trial, setup=self.setup, start=L.setup_spec(self.setup))

    # ------------------------------------------------------------------ reading
    def dofs(self):
        q = self.art.get_dof_positions().detach().cpu().numpy()[0].astype(float)
        return {n: float(q[i]) for n, i in self.idof.items()}

    def read(self):
        tf = self.view.get_transforms().detach().cpu().numpy().astype(float)
        return {n: (tf[i, :3] - self.origin, _rot(tf[i, 3:7])) for i, n in enumerate(BODIES)}

    # ------------------------------------------------------------------ physics step
    def _event(self, ev, **kw):
        self.mech.events.append(dict(t=round(self.t, 3), ev=ev, **kw))

    def physics(self, dt):
        t0 = time.perf_counter()
        self.t += dt
        m = self.mech
        d = self.dofs()
        lev = d["lev"]
        # clip
        if m.clip == "locked" and lev > math.radians(L.LEV_RELEASE_DEG):
            m.clip = "slack"
            self._event("clip slack", lev_deg=round(math.degrees(lev), 1))
        elif (m.clip == "slack" and lev < math.radians(L.LEV_RECLAMP_DEG) and m.bond != "free" and d["bz"] < 0.003
              and m.cpu == "seated" and not m.released):
            m.clip = "locked"
            m.lock = np.array([d[n] for n in BOND_DOFS])
            self._event("clip locked again")
        # processor
        if m.cpu == "seated" and d["pz"] > L.YANK_MM / 1000.0:
            m.cpu = "yanked"
            self._event("cpu yanked", pz_mm=round(d["pz"] * 1e3, 2))
        # paste film
        if m.bond == "sheared" and d["bz"] > L.PART_MM / 1000.0:
            m.bond = "free"
            self._event("parted", bz_mm=round(d["bz"] * 1e3, 2))
        mf = np.zeros(len(DOFS))
        tg = np.zeros(len(DOFS))
        idx = {n: k for k, n in enumerate(DOFS)}
        cam_locked = m.clip == "locked"
        for name, sign in L.HOOKS:
            if not cam_locked and sign * d[name] >= L.HOOK_RELEASE and name not in m.released:
                m.released.add(name)
                self._event("hook clear", hook=name, travel_mm=round(sign * d[name] * 1000, 2))
            lift_idx = idx[name + "_lift"]
            tg[lift_idx] = 0.011 if name in m.released else 0.0
            mf[lift_idx] = 8.0 if name in m.released else 50.0
            if name in m.released:
                tg[idx[name]] = sign * L.HOOK_TRAVEL
                mf[idx[name]] = 3.0
            else:
                mf[idx[name]] = 50.0 if cam_locked else 0.45
        locked = cam_locked or len(m.released) < len(L.HOOKS)
        # processor joints
        if m.cpu == "seated":
            for k, n in enumerate(CPU_DOFS):
                mf[idx[n]] = (L.CLIP_LOCK["f"] if locked else L.F_ZIF) if n == "pz" else CPU_HOLD[k]
        # paste joints
        brz = d["brz"]
        if m.bond != "free" or locked:
            if locked:
                tg[idx["bx"]:idx["brz"] + 1] = m.lock
                for n in ("bx", "by", "bz"):
                    mf[idx[n]] = L.CLIP_LOCK["f"]
                for n in ("brx", "bry", "brz"):
                    mf[idx[n]] = L.CLIP_LOCK["t"]
                m.a_rz.anchor, m.a_x.anchor, m.a_y.anchor = brz, d["bx"], d["by"]
            else:
                touching = d["bz"] < 0.0005
                if m.bond == "bonded":
                    t_cap, f_cap = m.tb, m.tb / L.PASTE_REFF
                    mf[idx["bz"]] = L.F_TENS
                    mf[idx["brx"]] = mf[idx["bry"]] = L.T_TILT_BOND
                else:  # sheared
                    t_cap, f_cap = (L.T_SLIDE, L.F_SLIDE) if touching else (0.0, 0.0)
                    mf[idx["bz"]] = L.F_TACK
                ar, sr = m.a_rz.update(brz, t_cap)
                ax, sx = m.a_x.update(d["bx"], f_cap)
                ay, sy = m.a_y.update(d["by"], f_cap)
                if m.bond == "bonded":
                    m.slip += sr + (sx + sy) / L.PASTE_REFF
                    if m.slip >= math.radians(L.SLIP_BREAK_DEG):
                        m.bond = "sheared"
                        self._event("paste sheared", brz_deg=round(math.degrees(brz), 2))
                tg[idx["bx"]], tg[idx["by"]], tg[idx["brz"]] = ax, ay, ar
                mf[idx["bx"]] = mf[idx["by"]] = f_cap
                mf[idx["brz"]] = t_cap
        # Cam angle and clip-end release are independent.
        if cam_locked:
            tg[idx["lev"]], mf[idx["lev"]] = 0.0, L.LEV_T_PEAK
        else:
            tg[idx["lev"]], mf[idx["lev"]] = math.radians(L.LEV_OPEN_DEG), L.LEV_OPEN_T
        targets = self.art.get_dof_position_targets().clone()
        for k, n in enumerate(DOFS):
            targets[0, self.idof[n]] = float(tg[k])
        self.art.set_dof_position_targets(targets, self.ai)
        if self.mf_now is None or np.max(np.abs(mf - self.mf_now)) > 1e-4 * (1.0 + np.max(np.abs(mf))):
            mx = self.art.get_dof_max_forces().clone()
            for k, n in enumerate(DOFS):
                mx[0, self.idof[n]] = float(mf[k])
            self.art.set_dof_max_forces(mx, self.ai_cpu)
            self.mf_now = mf.copy()
        if self.probe_wrench is not None:
            self.probe_wrench(self, dt)
        if self.trial_active:
            m.max_tw = max(m.max_tw, abs(math.degrees(brz)))
            m.max_pull = max(m.max_pull, d["pz"] * 1e3)
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
        """Environment reset only (never during a trial): the bench placed for the setup, the processor seated, the
        cooler seated on it with the clip locked, at rest."""
        torch = self.torch
        self.setup = setup
        self.trial_active = False
        spec = L.setup_spec(setup)
        bp, br = L.bench_pose(spec)
        rt = self.art.get_root_transforms().clone()
        rt[0, :3] = torch.tensor(bp + self.origin + np.array([0.0, 0.0, 0.0003]), dtype=rt.dtype)
        rt[0, 3:7] = torch.tensor(_quat(br), dtype=rt.dtype)
        self.art.set_root_transforms(rt, self.ai)
        self.art.set_root_velocities(torch.zeros_like(self.art.get_root_velocities()), self.ai)
        q = torch.zeros_like(self.art.get_dof_positions())
        self.art.set_dof_positions(q, self.ai)
        self.art.set_dof_velocities(torch.zeros_like(q), self.ai)
        self.art.set_dof_position_targets(q.clone(), self.ai)
        self.mf_now = None
        self._init_state()
        self.physics(0.0)  # locked-clip drive caps at once
        self.t = 0.0
        self.mech.events.clear()

    # ------------------------------------------------------------------ scoring
    def measure(self):
        poses = self.read()
        d = self.dofs()
        cp, cr = poses["cooler"]
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(cr[2, 2])))))
        base_z = cp[2] - (L.TOP + L.MAT[2])
        velocity = self.view.get_velocities()[BODIES.index("cooler")].detach().cpu().numpy()
        still = np.linalg.norm(velocity[:3]) + L.SHROUD_R * np.linalg.norm(velocity[3:]) < 0.01
        on_mat = still and L.on_mat(cp[:2]) and -0.003 < base_z < 0.004 and tilt < 15.0
        bp, br = poses["tbench"]
        return dict(poses=poses, dofs=d, cooler_tilt=tilt, cooler_base_z_mm=base_z * 1e3, on_mat=on_mat,
                    cpu_lift_mm=d["pz"] * 1e3, bond_z_mm=d["bz"] * 1e3, twist_deg=math.degrees(d["brz"]),
                    lever_deg=math.degrees(d["lev"]))

    def control(self, environment, dt=0.05):
        if not self.trial_active:
            return
        self.steps += 1
        m = self.measure()
        if not all(np.all(np.isfinite(p)) for p, _ in m["poses"].values()):
            self.nonfinite += 1
            STATE["nonfinite"] = self.nonfinite
            return
        cs = self.contact_step
        contact = dict(cooler=any(cs[n] > 0.05 for n in ("cooler", "lever") + HOOK_NAMES), cpu=cs["cpu"] > 0.05,
                       bench=cs["tbench"] > 0.05)
        mech = self.mech
        level = self.scorer.step(dt, m["cpu_lift_mm"], mech.bond == "free", m["on_mat"], contact)
        bp, br = m["poses"]["tbench"]
        spec = L.setup_spec(self.setup)
        STATE.update(
            score=level,
            level=level,
            steps=self.steps,
            nonfinite=self.nonfinite,
            detail=dict(
                self.scorer.last,
                clip=mech.clip,
                hooks_released=sorted(mech.released),
                cpu=mech.cpu,
                bond=mech.bond,
                slip_deg=round(math.degrees(mech.slip), 2),
                t_break=round(mech.tb, 3),
                lever_deg=round(m["lever_deg"], 2),
                twist_deg=round(m["twist_deg"], 2),
                max_twist_deg=round(mech.max_tw, 2),
                bond_z_mm=round(m["bond_z_mm"], 3),
                cooler_tilt_deg=round(m["cooler_tilt"], 2),
                cooler_pos=[round(v, 4) for v in m["poses"]["cooler"][0]],
                cooler_base_z_mm=round(m["cooler_base_z_mm"], 2),
                bench_shift_mm=round(float(np.linalg.norm(bp[:2] - np.array([spec["bx"], spec["by"]]))) * 1e3, 2),
                bench_yaw_deg=round(math.degrees(math.atan2(br[1, 0], br[0, 0])), 2),
                contact_n={n: round(v, 2) for n, v in self.contact_step.items()},
                contact_peak_n={n: round(v, 1) for n, v in self.contact_peak.items()},
                robot_link_peak={k: [round(v[0], 1), v[1]] for k, v in self.robot_link_peak.items()},
                events=mech.events[-6:],
                setup=self.setup,
                hook_wall_s=round(self.hook_s, 2),
            ),
        )
        self.contact_step = {n: 0.0 for n in BODIES}


_RT: dict = {"rt": None, "sim": None, "trials": 0}


def runtime():
    return _RT["rt"]


def bench_state():
    """Current poses {'tbench', 'cpu', 'cooler', 'lever'} plus the joint readings an operator can see (lever angle,
    how far the cooler has turned / lifted on the processor, whether the processor has lifted), for the scripted demo
    (read-only)."""
    rt = _RT["rt"]
    if rt is None:
        return None
    m = rt.measure()
    out = dict(m["poses"])
    out.update(lever_deg=m["lever_deg"], twist_deg=m["twist_deg"], bond_z_mm=m["bond_z_mm"],
               cpu_lift_mm=m["cpu_lift_mm"], cooler_tilt=m["cooler_tilt"], on_mat=m["on_mat"], dofs=m["dofs"])
    return out


def _patch_scene(scene) -> None:
    if getattr(scene, "_ct_patched", False):
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
                p, r = rt.read()["cooler"]
            else:
                p, r = L.bench_pose(L.setup_spec(SETUP_BASE))
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
    scene._ct_patched = True


def _has_bench(emb):
    return any(isinstance(o, Bench) for o in (emb.task.objects if emb.task is not None else ()))


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_ct_patched", False):
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
        if rt is not None:
            rt.restore_authored(setup)
        obs = base_reset(self, *a, **k)
        rt = _RT["rt"]
        if rt is not None:
            spec = L.setup_spec(setup)
            moved = float(np.linalg.norm(rt.read()["tbench"][0][:2] - np.array([spec["bx"], spec["by"]]))) > 0.001
            if rt.setup != setup or moved or _RT["trials"] == 0:
                rt.restore_authored(setup)
                obs = base_reset(self, *a, **k)
            rt.begin_trial(_RT["trials"])
            _RT["trials"] += 1
        return obs

    cls._ensure_sim, cls._task_step, cls.reset = _ensure_sim, _task_step, reset
    cls._ct_patched = True


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
