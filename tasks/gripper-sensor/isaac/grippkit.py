"""grippkit: the gripper test bench (pneumatic parallel gripper on a flange stand, its auto switch, the solenoid valve
with its manual override, the test part, the screwdriver and its rack) for yam_arena (Isaac Lab-Arena / PhysX), plus
the runtime hooks.

Authored as one custom-spawned assembly (yam_arena task-file primitives are single boxes/spheres):
* static parts as one kinematic body ``bench`` (flange stand, gripper body with its groove, valve, rack, part nest);
* the two jaws with their fingers as kinematic bodies: a device driven by the valve (the gripper closes at 0.1 m/s per
  jaw while the valve is switched and stops on the part; it opens when the valve drops out);
* the manual override button (kinematic): the valve switches while the robot pushes it with 5 N or more and drops out
  below 3 N (non-locking override);
* the auto switch and its set screw (kinematic): the switch slides along the groove only when pushed along it with more
  than its static friction (2.5 N; keeps sliding above 1.8 N); the screw turns only with the screwdriver's blade
  engaged in it (pushed in with 1.5 N or more, blade tip within 0.8 mm of the screw axis, tool within 12 deg of the
  axis) and turning; 90 deg clockwise locks the switch (it then holds 30 N);
* the test part (three kinematic blocks that together make one block of the seed's width);
* the screwdriver: a free rigid body (handle + blade).
Geometry: ``../src/layout.py``; scorer: ``../src/scoring.py`` (pure functions).

Runtime hooks (installed on import; nothing in yam_arena is edited; same pattern as earlier task kits): after every
physics step the monitor reads the contact forces on the override button, the switch and the screw, moves the jaws,
slides the switch and turns the screw; after every control step the scorer runs and the LED colour is set. Nothing here
writes the screwdriver's pose or velocity during a trial; ``Runtime.restore_authored`` resets the bench only on an
environment reset. The scripted demo READS state through ``bench_state()``.
"""

from __future__ import annotations

import functools
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
import scoring  # noqa: E402
from yam_arena.task_file import Box  # noqa: E402

PHYSICS_DT = float(os.environ.get("GS_PHYSICS_DT", str(1 / 480)))
CONTACT_OFFSET = float(os.environ.get("GS_CONTACT_OFFSET", ".0002"))
SETUP_BASE = int(os.environ.get("GS_SETUP", "0"))
FORCE_NAMED = "GS_SETUP" in os.environ
DEBUG = bool(os.environ.get("GS_DEBUG"))

ALU, ALU_DARK, STEEL, BLACK, BLUE_GREY, VALVE_COL = (
    (0.78, 0.79, 0.80),
    (0.45, 0.47, 0.50),
    (0.55, 0.56, 0.58),
    (0.07, 0.07, 0.08),
    (0.30, 0.36, 0.42),
    (0.20, 0.22, 0.24),
)
OVR_COL, SW_COL, LED_OFF, LED_ON, HANDLE_COL, BLADE_COL, RACK_COL, TUBE_A, TUBE_B = (
    (0.10, 0.35, 0.80),
    (0.12, 0.12, 0.13),
    (0.25, 0.05, 0.05),
    (1.0, 0.08, 0.04),
    (0.85, 0.15, 0.10),
    (0.70, 0.71, 0.73),
    (0.20, 0.45, 0.25),
    (0.05, 0.25, 0.80),
    (0.10, 0.10, 0.10),
)
MOVING = (
    "switch",
    "screw",
    "jaw_p",
    "jaw_n",
    "part_a",
    "part_b",
    "part_c",
    "driver",
    "ovr",
)
KIN_IDX = (
    0,
    1,
    2,
    3,
    4,
    5,
    6,
)  # kinematic bodies moved by the runtime (index into MOVING)


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


def rz(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def part_blocks(width):
    """Centres (y) and the common width of the three blocks that make a part of the given width."""
    bw = (
        L.part_width(L.PART_MAX_TRAVEL) / 2
    )  # each edge block is half the narrowest part
    yc = L.BODY_C[1]
    return [yc - width / 2 + bw / 2, yc + width / 2 - bw / 2, yc], bw


@functools.lru_cache(maxsize=64)
def spec_of(setup):
    return L.setup_spec(setup)


def authored(setup, travel_now=0.0, u_sw=None, screw_deg=0.0):
    """Pose of every moving body: {name: (pos, rot)} in the env frame (also installs the trial's switch)."""
    sp = spec_of(setup)
    L.set_sense(sp)
    u = sp["u0"] if u_sw is None else u_sw
    zc = L.BODY_C[2]
    ys, _ = part_blocks(sp["width"])
    sxy = L.screw_xy(u)
    out = {
        "switch": (
            np.array([L.switch_center_x(u), L.GROOVE_Y, L.switch_z()]),
            np.eye(3),
        ),
        "screw": (np.array([sxy[0], sxy[1], L.BODY_TOP + L.SW_PROUD]), rz(-screw_deg)),
        "jaw_p": (np.array([L.FINGER_X0, L.jaw_y(travel_now, 1), zc]), np.eye(3)),
        "jaw_n": (np.array([L.FINGER_X0, L.jaw_y(travel_now, -1), zc]), np.eye(3)),
        "part_a": (np.array([L.PART_X, ys[0], zc]), np.eye(3)),
        "part_b": (np.array([L.PART_X, ys[1], zc]), np.eye(3)),
        "part_c": (np.array([L.PART_X, ys[2], zc]), np.eye(3)),
        "driver": (L.driver_home(), np.eye(3)),
        "ovr": (L.ovr_center() - np.array([0.0, 0.0, L.OVR_PROUD]), np.eye(3)),
    }
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
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(*(translation or (0.0, 0.0, 0.0)))
    )
    ox, oy, oz, ow = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Quatd(ow, ox, oy, oz)
    )
    looks, mats = {}, {}

    def look(color, emissive=None, name=None):
        key = (color, emissive, name)
        if key not in looks:
            path = f"{prim_path}/Looks/{name or 'c%d' % len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(
                diffuse_color=linear_rgb(color),
                roughness=0.45,
                emissive_color=linear_rgb(emissive) if emissive else (0.0, 0.0, 0.0),
            )
            vis.func(path, vis)
            looks[key] = path
        return looks[key]

    def material(friction):
        if friction not in mats:
            path = f"{prim_path}/Looks/phys{len(mats)}"
            pm = sim_utils.RigidBodyMaterialCfg(
                static_friction=friction, dynamic_friction=friction
            )
            pm.func(path, pm)
            mats[friction] = path
        return mats[friction]

    def xform(path, pos, rot):
        x = UsdGeom.Xform.Define(stage, path)
        x.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Vec3d(*map(float, pos))
        )
        q = _quat(rot)
        x.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Quatd(float(q[3]), *map(float, q[:3]))
        )
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
        px.CreateSleepThresholdAttr(
            0.0
        )  # never sleeps: a sleeping body reports no contacts
        px.CreateStabilizationThresholdAttr(0.0)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)

    count = {"n": 0}

    def hull(path, v, color, friction=L.HARD_FRICTION, collide=True, look_path=None):
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
        m.CreateExtentAttr(
            [Gf.Vec3f(*map(float, v.min(axis=0))), Gf.Vec3f(*map(float, v.max(axis=0)))]
        )
        prim = m.GetPrim()
        if collide:
            UsdPhysics.CollisionAPI.Apply(prim)
            cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            cp.CreateContactOffsetAttr(CONTACT_OFFSET)
            cp.CreateRestOffsetAttr(0.0)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(
                "convexHull"
            )
            hc = PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim)
            hc.CreateHullVertexLimitAttr(255)
            hc.CreateMinThicknessAttr(0.0001)
            bind_physics_material(str(prim.GetPath()), material(friction), stage=stage)
            count["n"] += 1
        bind_visual_material(str(prim.GetPath()), look_path or look(color), stage=stage)
        return m

    def box(
        path,
        center,
        size,
        color,
        friction=L.HARD_FRICTION,
        yaw_deg=0.0,
        collide=True,
        look_path=None,
    ):
        hx, hy, hz = (s / 2 for s in size)
        corners = np.array(
            [
                [sx * hx, sy * hy, sz * hz]
                for sx in (-1, 1)
                for sy in (-1, 1)
                for sz in (-1, 1)
            ]
        )
        corners = corners @ rz(yaw_deg).T + np.asarray(center, float)
        return hull(path, corners, color, friction, collide, look_path)

    def cylinder(r, z0, z1, n=24, center=(0.0, 0.0), axis="z"):
        a = np.linspace(0, 2 * np.pi, n, endpoint=False)
        ring = lambda z: np.c_[r * np.cos(a), r * np.sin(a), np.full(n, z)]  # noqa: E731
        v = np.r_[ring(z0), ring(z1)]
        if axis == "x":
            v = v[:, [2, 0, 1]]
        elif axis == "y":
            v = v[:, [1, 2, 0]]
        return (
            v + np.r_[center, 0.0][:3] if axis == "z" else v + np.asarray(center, float)
        )

    def tube(path, pts, r, color):
        """Visual-only tube along a polyline (cosmetic air lines)."""
        for i in range(len(pts) - 1):
            a, b = np.asarray(pts[i], float), np.asarray(pts[i + 1], float)
            d = b - a
            ln = float(np.linalg.norm(d))
            z = d / ln
            x = (
                np.cross(z, [0.0, 0.0, 1.0])
                if abs(z[2]) < 0.9
                else np.cross(z, [1.0, 0.0, 0.0])
            )
            x /= np.linalg.norm(x)
            y = np.cross(z, x)
            ang = np.linspace(0, 2 * np.pi, 12, endpoint=False)
            ring = np.array([r * (math.cos(t) * x + math.sin(t) * y) for t in ang])
            hull(f"{path}_{i}", np.r_[ring + a, ring + b], color, collide=False)

    zc = L.BODY_C[2]
    # ---------------------------------------------------------------- static bench (one kinematic body, world frame)
    B = f"{prim_path}/bench"
    xform(B, (0.0, 0.0, 0.0), np.eye(3))
    rigid(B, 20.0, kinematic=True)
    sb = np.array([L.FLANGE_X + 0.035, L.BODY_C[1], L.TOP + L.STAND_BASE[2] / 2])
    box(f"{B}/stand_base", sb, L.STAND_BASE, ALU_DARK)
    pl_x = L.FLANGE_X - 0.010 - L.STAND_PLATE[0] / 2
    box(
        f"{B}/flange_plate",
        (pl_x, L.BODY_C[1], L.TOP + L.STAND_BASE[2] + L.STAND_PLATE[2] / 2),
        L.STAND_PLATE,
        ALU_DARK,
    )
    # ISO 9409-1-50-4-M6 style adapter plate between the flange plate and the gripper's mounting face
    hull(
        f"{B}/adapter",
        cylinder(
            0.0315,
            L.FLANGE_X - 0.010,
            L.FLANGE_X,
            n=32,
            center=(0.0, L.BODY_C[1], zc),
            axis="x",
        ),
        BLACK,
    )
    # gripper body (anodised aluminium) with the groove drawn on its top face, and the jaw guide in front of it
    box(f"{B}/body", L.BODY_C, (L.BODY_L, L.BODY_W, L.BODY_T), ALU)
    box(
        f"{B}/groove",
        (L.BODY_C[0], L.GROOVE_Y, L.BODY_TOP + 0.00005),
        (L.BODY_L - 0.002, L.GROOVE_W, 0.0001),
        BLACK,
        collide=False,
    )
    for k, gy in enumerate(
        (L.BODY_C[1] - 0.014, L.GROOVE_Y)
    ):  # the second groove line on the face (empty)
        if k == 0:
            box(
                f"{B}/groove2",
                (L.BODY_C[0], gy, L.BODY_TOP + 0.00005),
                (L.BODY_L - 0.002, L.GROOVE_W, 0.0001),
                BLACK,
                collide=False,
            )
    box(
        f"{B}/jaw_guide",
        (L.FLANGE_X + L.BODY_L + L.JAW_GUIDE_L / 2, L.BODY_C[1], zc - 0.006),
        (L.JAW_GUIDE_L, L.BODY_W, L.BODY_T - 0.012),
        ALU,
    )
    # ports on the -y side (cosmetic) and the part nest (post on the stand base)
    for k, px_ in enumerate((L.FLANGE_X + 0.030, L.FLANGE_X + 0.062)):
        hull(
            f"{B}/port_{k}",
            cylinder(
                0.004,
                L.BODY_C[1] - L.BODY_W / 2 - 0.006,
                L.BODY_C[1] - L.BODY_W / 2,
                n=16,
                center=(px_, 0.0, zc - 0.006),
                axis="y",
            )
            + np.array([0.0, 0.0, 0.0]),
            BLACK,
            collide=False,
        )
    nest_top = zc - L.PART_H / 2
    box(
        f"{B}/nest_post",
        (L.PART_X, L.BODY_C[1], (L.TOP + nest_top) / 2),
        (0.020, 0.024, nest_top - L.TOP),
        ALU_DARK,
    )
    # valve: bracket + sub-base, valve body, solenoid connector (cosmetic shapes on one kinematic body)
    v = L.VALVE
    box(
        f"{B}/valve_base",
        v + np.array([0.0, 0.0, L.VALVE_BASE[2] / 2]),
        L.VALVE_BASE,
        ALU_DARK,
    )
    vz = v[2] + L.VALVE_BASE[2]
    box(
        f"{B}/valve_body",
        (v[0], v[1], vz + L.VALVE_BODY[2] / 2),
        L.VALVE_BODY,
        VALVE_COL,
    )
    box(
        f"{B}/valve_coil",
        (v[0] - 0.034, v[1], vz + 0.012),
        (0.016, 0.010, 0.024),
        BLACK,
    )
    for k, dx in enumerate(
        (0.010, 0.020)
    ):  # A / B ports on the sub-base's +x side... cosmetic fittings
        hull(
            f"{B}/vport_{k}",
            cylinder(
                0.003,
                v[1] - L.VALVE_BASE[1] / 2 - 0.006,
                v[1] - L.VALVE_BASE[1] / 2,
                n=12,
                center=(v[0] + dx, 0.0, v[2] + 0.020),
                axis="y",
            ),
            BLACK,
            collide=False,
        )
    # rack (square hole made of four blocks)
    r = L.RACK
    hw, hh = L.RACK_SIZE[0] / 2, L.RACK_HOLE / 2
    t = hw - hh
    for k, (cx, cy, sx, sy) in enumerate(
        (
            (-(hh + t / 2), 0.0, t, L.RACK_SIZE[1]),
            (hh + t / 2, 0.0, t, L.RACK_SIZE[1]),
            (0.0, -(hh + t / 2), L.RACK_HOLE, t),
            (0.0, hh + t / 2, L.RACK_HOLE, t),
        )
    ):
        box(
            f"{B}/rack_{k}",
            (r[0] + cx, r[1] + cy, r[2] + L.RACK_SIZE[2] / 2),
            (sx, sy, L.RACK_SIZE[2]),
            RACK_COL,
        )
    # cosmetic air lines from the valve's A/B ports to the gripper's ports
    for k, (dx, gx, col) in enumerate(
        ((0.010, L.FLANGE_X + 0.030, TUBE_A), (0.020, L.FLANGE_X + 0.062, TUBE_B))
    ):
        a0 = np.array([v[0] + dx, v[1] - L.VALVE_BASE[1] / 2 - 0.006, v[2] + 0.020])
        a3 = np.array([gx, L.BODY_C[1] - L.BODY_W / 2 - 0.006, zc - 0.006])
        mid = np.array([a0[0] - 0.05, a0[1] - 0.05, L.TOP + 0.004 + 0.003 * k])
        mid2 = np.array([a3[0] - 0.02, a3[1] - 0.05, L.TOP + 0.004 + 0.003 * k])
        tube(f"{B}/tube{k}", [a0, mid, mid2, a3], 0.002, col)
    # ---------------------------------------------------------------- jaws + fingers (kinematic device)
    A = authored(SETUP_BASE)
    for name in ("jaw_p", "jaw_n"):
        side = 1 if name == "jaw_p" else -1
        xform(f"{prim_path}/{name}", *A[name])
        rigid(f"{prim_path}/{name}", 0.1, kinematic=True)
        box(
            f"{prim_path}/{name}/finger",
            (L.FINGER_L / 2, 0.0, 0.0),
            (L.FINGER_L, L.FINGER_W, L.FINGER_H),
            ALU,
        )
        box(
            f"{prim_path}/{name}/slider",
            (-L.JAW_GUIDE_L / 2, side * 0.002, 0.0),
            (L.JAW_GUIDE_L - 0.001, L.FINGER_W + 0.004, L.FINGER_H),
            STEEL,
        )
    # ---------------------------------------------------------------- test part (three coincident steel blocks)
    _, bw = part_blocks(L.part_width(0.0))
    mid_w = L.part_width(L.PART_MIN_TRAVEL) - 2 * bw
    for name, w in (("part_a", bw), ("part_b", bw), ("part_c", mid_w)):
        xform(f"{prim_path}/{name}", *A[name])
        rigid(f"{prim_path}/{name}", 0.1, kinematic=True)
        box(
            f"{prim_path}/{name}/block",
            (0.0, 0.0, 0.0),
            (L.PART_L, w, L.PART_H),
            STEEL,
            look_path=look(STEEL, name="part"),
        )
    # ---------------------------------------------------------------- override button (kinematic)
    xform(f"{prim_path}/ovr", *A["ovr"])
    rigid(f"{prim_path}/ovr", 0.005, kinematic=True)
    hull(f"{prim_path}/ovr/cap", cylinder(L.OVR_R, -0.002, L.OVR_PROUD, n=16), OVR_COL)
    # ---------------------------------------------------------------- auto switch + set screw (kinematic)
    xform(f"{prim_path}/switch", *A["switch"])
    rigid(f"{prim_path}/switch", 0.01, kinematic=True)
    # switch housing, open over the screw's capture cone (the cone's bottom is 0.6 mm below the housing top)
    sx = L.SW_L / 2 - L.SCREW_FROM_FRONT
    xb = sx - L.CUP_R_OUT
    box(
        f"{prim_path}/switch/body",
        ((xb - L.SW_L / 2) / 2, 0.0, 0.0),
        (xb + L.SW_L / 2, L.SW_W, L.SW_H),
        SW_COL,
        friction=0.5,
    )
    h_front = L.SW_H - (L.CUP_DEPTH - L.CUP_PROUD) - 0.0001
    box(
        f"{prim_path}/switch/front",
        ((xb + L.SW_L / 2) / 2, 0.0, -L.SW_H / 2 + h_front / 2),
        (L.SW_L / 2 - xb, L.SW_W, h_front),
        SW_COL,
        friction=0.5,
    )
    led_dx = -L.SW_L / 2 + L.LED_FROM_BACK
    led_look = look(LED_OFF, emissive=(0.0, 0.0, 0.0), name="led")
    box(
        f"{prim_path}/switch/led",
        (led_dx, 0.0, L.SW_H / 2 + 0.00005),
        (0.0040, L.SW_W - 0.0004, 0.0001),
        LED_OFF,
        collide=False,
        look_path=led_look,
    )
    # grey cable leaving the back end (cosmetic, a short straight run along the groove)
    box(
        f"{prim_path}/switch/cable",
        (-L.SW_L / 2 - 0.012, 0.0, -0.0004),
        (0.024, 0.0026, 0.0026),
        (0.55, 0.56, 0.57),
        collide=False,
    )
    _SPEC["led_shader"] = f"{led_look}/Shader"
    xform(f"{prim_path}/screw", *A["screw"])
    rigid(f"{prim_path}/screw", 0.001, kinematic=True)
    # screw head with the blade's capture cone: a flat bottom (the screw's slotted end) 0.5 mm down inside a 45 deg
    # conical lead-in (stands in for the slot that holds a slotted blade; see README fidelity)
    zt, zb = L.CUP_PROUD, L.CUP_PROUD - L.CUP_DEPTH
    hull(
        f"{prim_path}/screw/head",
        cylinder(L.CUP_R_IN, -0.0015, zb, n=16),
        STEEL,
        friction=0.5,
    )
    nseg = 12
    for k in range(nseg):
        a0, a1 = 2 * math.pi * k / nseg, 2 * math.pi * (k + 1) / nseg
        pts = []
        for a in (a0, a1):
            ca, sa = math.cos(a), math.sin(a)
            pts += [
                (L.CUP_R_IN * ca, L.CUP_R_IN * sa, zb),
                (L.CUP_R_OUT * ca, L.CUP_R_OUT * sa, zb - 0.001),
                (L.CUP_R_TOP * ca, L.CUP_R_TOP * sa, zt),
                (L.CUP_R_OUT * ca, L.CUP_R_OUT * sa, zt),
            ]
        hull(f"{prim_path}/screw/cone_{k:02d}", pts, STEEL, friction=0.5)
    box(
        f"{prim_path}/screw/slot",
        (0.0, 0.0, zb + 0.00005),
        (2 * L.CUP_R_IN - 0.0002, 0.0005, 0.0001),
        BLACK,
        collide=False,
    )
    # ---------------------------------------------------------------- screwdriver (free body; frame at the blade tip)
    xform(f"{prim_path}/driver", *A["driver"])
    m_h, m_b = L.DRV_MASS * 0.85, L.DRV_MASS * 0.15
    zh = L.DRV_BLADE_L + L.DRV_HANDLE_L / 2
    zb = L.DRV_BLADE_L / 2
    com_z = (m_h * zh + m_b * zb) / L.DRV_MASS
    ixx = (
        m_h * (3 * L.DRV_HANDLE_R**2 + L.DRV_HANDLE_L**2) / 12
        + m_h * (zh - com_z) ** 2
        + m_b * L.DRV_BLADE_L**2 / 12
        + m_b * (zb - com_z) ** 2
    )
    izz = 0.5 * m_h * L.DRV_HANDLE_R**2
    rigid(
        f"{prim_path}/driver",
        L.DRV_MASS,
        inertia=(ixx, ixx, izz),
        com=(0.0, 0.0, com_z),
    )
    hull(
        f"{prim_path}/driver/handle",
        cylinder(L.DRV_HANDLE_R, L.DRV_BLADE_L, L.DRV_BLADE_L + L.DRV_HANDLE_L, n=24),
        HANDLE_COL,
        friction=L.DRV_FRICTION,
    )
    hull(
        f"{prim_path}/driver/blade",
        cylinder(L.DRV_BLADE_R, 0.0, L.DRV_BLADE_L, n=12),
        BLADE_COL,
    )
    _SPEC["paths"] = {n: f"{prim_path}/{n}" for n in MOVING + ("bench",)}
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
        initial_pose=Pose(
            position_xyz=(0.0, 0.0, 0.0), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)
        ),
    )


# ---------------------------------------------------------------------------------------------- runtime state
STATE: dict = dict(
    score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, trial=-1, setup=None
)


def tcp_points(environment):
    """Tool centre points (between the finger pads) of both arms, env frame."""
    from yam_arena.constants import TCP_OFFSET

    base = environment.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(float)
    pts = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        pose = (
            robot.data.body_link_pose_w[0]
            .detach()
            .cpu()
            .numpy()
            .astype(float)[list(robot.body_names).index("gripper")]
        )
        pts.append(pose[:3] + _rot(pose[3:7]) @ np.array(TCP_OFFSET) - origin)
    return pts


class Runtime:
    """Valve, jaws, switch slide and screw, from the contact forces every physics step; scorer every control step."""

    def __init__(self, environment):
        import carb
        import omni.physics.tensors as tensors
        import torch
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdUtils

        carb.settings.get_settings().set_bool(
            "/physics/disableContactProcessing", False
        )
        self.base = environment.unwrapped
        self.env = environment
        self.torch = torch
        self.stage = get_current_stage()
        stage_id = UsdUtils.StageCache.Get().GetId(self.stage).ToLongInt()
        self.simview = tensors.create_simulation_view("torch", stage_id=stage_id)
        self.simview.set_subspace_roots("/")
        root = "/World/envs/env_0/bench"
        P = lambda n: f"{root}/{n}"  # noqa: E731
        self.view = self.simview.create_rigid_body_view([P(n) for n in MOVING])
        robot_links = []
        for arm in ("left_robot", "right_robot"):
            lp = self.base.scene[arm].root_view.link_paths
            robot_links += list(
                lp[0] if lp and isinstance(lp[0], (list, tuple)) else lp
            )
        self.robot_links = robot_links
        filt = robot_links + [P("driver")]
        self.n_robot = len(robot_links)
        self.cv_sw = self.simview.create_rigid_contact_view(
            [P("switch"), P("screw")],
            filter_patterns=[filt, filt],
            max_contact_data_count=4096,
        )
        self.cv_ovr = self.simview.create_rigid_contact_view(
            [P("ovr")], filter_patterns=[robot_links], max_contact_data_count=1024
        )
        self.origin = (
            self.base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        )
        self.setup = SETUP_BASE
        self.trial_active = False
        self.led_shader = None
        self.led_state = None
        self._reset_device(SETUP_BASE)
        self._init_state()

    def _reset_device(self, setup):
        sp = spec_of(setup)
        L.set_sense(sp)
        self.spec = sp
        self.travel_part = sp["travel"]
        self.u_sw = sp["u0"]
        self.travel = 0.0
        self.valve_on = False
        self.screw_deg = 0.0
        self.sliding = False
        self.last_drv_yaw = None

    def _init_state(self):
        self.scorer = scoring.Scorer()
        self.steps = 0
        self.t = 0.0
        self.ovr_peak = 0.0
        self.ovr_n = 0.0
        self.sw_force = np.zeros(3)
        self.sw_friction = np.zeros(3)
        self.sw_force_peak = 0.0
        self.engaged = False
        self.low_push_s = 0.0
        self.engage_s = 0.0
        self.engage_push_peak = 0.0
        self.touching = False
        self.valve_events = []
        self.slide_mm = 0.0
        self.hook_s = 0.0
        self.nonfinite = 0
        self.led = False
        STATE.update(
            score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, setup=self.setup
        )

    def begin_trial(self, trial):
        self._init_state()
        self.trial_active = True
        STATE.update(trial=trial, setup=self.setup, start=L.setup_spec(self.setup))

    # ------------------------------------------------------------------ reading
    def read(self):
        tf = self.view.get_transforms().detach().cpu().numpy().astype(float)
        return {
            n: (tf[i, :3] - self.origin, _rot(tf[i, 3:7])) for i, n in enumerate(MOVING)
        }

    def driver_pose(self):
        tf = self.view.get_transforms().detach().cpu().numpy().astype(float)
        i = MOVING.index("driver")
        return tf[i, :3] - self.origin, _rot(tf[i, 3:7])

    # ------------------------------------------------------------------ device writes
    def write_kinematic(self):
        torch = self.torch
        a = authored(self.setup, self.travel, self.u_sw, self.screw_deg)
        tf = self.view.get_transforms().clone()
        for i in KIN_IDX:
            pos, rot = a[MOVING[i]]
            tf[i, :3] = torch.tensor(
                np.asarray(pos, float) + self.origin, dtype=tf.dtype
            )
            tf[i, 3:7] = torch.tensor(_quat(rot), dtype=tf.dtype)
        idx = torch.tensor(list(KIN_IDX), dtype=torch.int32, device=tf.device)
        self.view.set_kinematic_targets(tf, idx)

    def friction_matrix(self, dt):
        """Per (sensor, filter) summed friction force on the switch / screw (the force matrix holds normal forces
        only; dragging the switch by a pressed fingertip acts through friction)."""
        n_s, n_f = 2, self.n_robot + 1
        out = np.zeros((n_s, n_f, 3))
        try:
            ff, _, cnt, start = self.cv_sw.get_friction_data(dt=dt)
        except Exception as e:  # noqa: BLE001
            if not getattr(self, "_fric_warned", False):
                print("grippkit: friction data unavailable:", e, flush=True)
                self._fric_warned = True
            return out
        ff = ff.detach().cpu().numpy().reshape(-1, 3)
        cnt = cnt.detach().cpu().numpy().reshape(n_s, n_f)
        start = start.detach().cpu().numpy().reshape(n_s, n_f)
        for i, j in zip(*np.nonzero(cnt)):
            out[i, j] = ff[int(start[i, j]) : int(start[i, j]) + int(cnt[i, j])].sum(
                axis=0
            )
        return out

    # ------------------------------------------------------------------ physics step
    def physics(self, dt):
        t0 = time.perf_counter()
        self.t += dt
        if self.trial_active:
            fo = self.cv_ovr.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
            f_ovr = float(np.linalg.norm(fo[0], axis=-1).max()) if fo.size else 0.0
            self.ovr_n = f_ovr
            self.ovr_peak = max(self.ovr_peak, f_ovr)
            was = self.valve_on
            if f_ovr >= L.OVR_PRESS_N:
                self.valve_on = True
            elif f_ovr < L.OVR_RELEASE_N:
                self.valve_on = False
            if was != self.valve_on:
                self.valve_events.append(
                    dict(on=self.valve_on, t=round(self.t, 3), step=self.steps)
                )
            fs = (
                self.cv_sw.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
            )  # (2, n_filt, 3) normal forces
            fr = self.friction_matrix(dt)  # (2, n_filt, 3) friction forces
            total = (fs + fr).reshape(-1, 3).sum(axis=0) if fs.size else np.zeros(3)
            self.sw_friction = fr.reshape(-1, 3).sum(axis=0)
            self.sw_force = total
            self.touching = bool(
                fs.size and float(np.linalg.norm(fs, axis=-1).max()) > 0.05
            )
            self.sw_force_peak = max(self.sw_force_peak, float(abs(total[0])))
            # screw engagement: blade on the screw head (collider identity), tool axis and tip position
            drv_on_screw = fs[1, self.n_robot] if fs.size else np.zeros(3)
            push = float(abs(drv_on_screw[2]))
            dp, dr = self.driver_pose()
            sxy = L.screw_xy(self.u_sw)
            tilt = math.degrees(math.acos(float(np.clip(dr[2, 2], -1.0, 1.0))))
            lat = float(np.hypot(dp[0] - sxy[0], dp[1] - sxy[1]))
            yaw = math.degrees(math.atan2(dr[1, 0], dr[0, 0]))
            # in the slot it stays in the slot: once engaged, the tip may wander up to ENGAGE_HOLD_R and the push may
            # dip below ENGAGE_PUSH_N for up to ENGAGE_GRACE_S (the blade does not leave a slot in a moment)
            self.low_push_s = self.low_push_s + dt if push < L.ENGAGE_PUSH_N else 0.0
            if self.engaged:
                eng = (
                    self.low_push_s < L.ENGAGE_GRACE_S
                    and tilt <= L.ENGAGE_TILT_DEG
                    and lat <= L.ENGAGE_HOLD_R
                )
            else:
                eng = (
                    push >= L.ENGAGE_PUSH_N
                    and tilt <= L.ENGAGE_TILT_DEG
                    and lat <= L.ENGAGE_R
                )
            if eng and self.engaged and self.last_drv_yaw is not None:
                dyaw = (yaw - self.last_drv_yaw + 180.0) % 360.0 - 180.0
                self.screw_deg = float(np.clip(self.screw_deg - dyaw, 0.0, 360.0))
            self.engaged = eng
            self.last_drv_yaw = yaw
            if eng:
                self.engage_s += dt
                self.engage_push_peak = max(self.engage_push_peak, push)
            self.engage_info = dict(
                push_n=round(push, 3),
                tilt_deg=round(tilt, 2),
                lat_mm=round(lat * 1e3, 3),
            )
            # switch slide along the groove (x): Coulomb stick / slip on the axial force
            fa = float(total[0])
            locked = self.screw_deg >= L.LOCK_TURN_DEG
            f_static = L.LOCKED_HOLD_N if locked else L.SLIDE_STATIC_N
            f_dyn = L.LOCKED_HOLD_N if locked else L.SLIDE_DYNAMIC_N
            if not self.sliding and abs(fa) > f_static:
                self.sliding = True
            if self.sliding and abs(fa) <= f_dyn:
                self.sliding = False
            if self.sliding:
                v = math.copysign(
                    min(L.SLIDE_VMAX, (abs(fa) - f_dyn) * L.SLIDE_GAIN), fa
                )
                u_new = float(np.clip(self.u_sw + v * dt, L.U_MIN, L.U_MAX))
                self.slide_mm += abs(u_new - self.u_sw) * 1e3
                self.u_sw = u_new
            if DEBUG and int(round(self.t / dt)) % 48 == 0:
                print(
                    "PHYS",
                    dict(
                        f_ovr=round(f_ovr, 2),
                        f_sw=np.round(total, 3).tolist(),
                        f_fric=np.round(self.sw_friction, 3).tolist(),
                        sliding=self.sliding,
                        u=round(self.u_sw * 1e3, 3),
                        valve=self.valve_on,
                        eng=eng,
                        screw=round(self.screw_deg, 1),
                        **self.engage_info,
                    ),
                    flush=True,
                )
        # jaws: toward the part (valve on) or open, at JAW_SPEED
        target = self.travel_part if self.valve_on else 0.0
        self.travel += float(
            np.clip(target - self.travel, -L.JAW_SPEED * dt, L.JAW_SPEED * dt)
        )
        self.write_kinematic()
        self.hook_s += time.perf_counter() - t0

    # ------------------------------------------------------------------ reset
    def restore_authored(self, setup):
        """Environment reset only (never during a trial)."""
        torch = self.torch
        self.setup = setup
        self.trial_active = False
        self._reset_device(setup)
        a = authored(setup)
        tf = self.view.get_transforms().clone()
        for i, n in enumerate(MOVING):
            pos, rot = a[n]
            tf[i, :3] = torch.tensor(
                np.asarray(pos, float) + self.origin, dtype=tf.dtype
            )
            tf[i, 3:7] = torch.tensor(_quat(rot), dtype=tf.dtype)
        idx = torch.tensor(
            list(range(len(MOVING))), dtype=torch.int32, device=tf.device
        )
        self.view.set_transforms(tf, idx)
        di = MOVING.index("driver")
        self.view.set_velocities(
            torch.zeros_like(self.view.get_velocities()),
            torch.tensor([di], dtype=torch.int32, device=tf.device),
        )
        self.write_kinematic()
        self._init_state()
        self.set_led(False)

    # ------------------------------------------------------------------ LED
    def set_led(self, on):
        if on == self.led_state:
            return
        from pxr import Gf, UsdShade

        if self.led_shader is None:
            self.led_shader = False
            for path in (
                _SPEC.get("led_shader"),
                "/World/envs/env_0/bench/Looks/led/Shader",
            ):
                prim = self.stage.GetPrimAtPath(path) if path else None
                if prim and prim.IsValid():
                    self.led_shader = UsdShade.Shader(prim)
                    break
            print("grippkit: LED shader", self.led_shader, flush=True)
        if self.led_shader:
            from yam_arena.station import linear_rgb

            col = linear_rgb(LED_ON if on else LED_OFF)
            em = tuple(20.0 * c for c in linear_rgb(LED_ON)) if on else (0.0, 0.0, 0.0)
            self.led_shader.GetInput("diffuseColor").Set(Gf.Vec3f(*map(float, col)))
            inp = self.led_shader.GetInput("emissiveColor")
            if inp:
                inp.Set(Gf.Vec3f(*map(float, em)))
        self.led_state = on

    def led_now(self):
        return scoring.led(self.u_sw, self.travel)

    # ------------------------------------------------------------------ scoring
    def control(self, environment):
        self.led = self.led_now()
        self.set_led(self.led)
        if not self.trial_active:
            return
        self.steps += 1
        dp, dr = self.driver_pose()
        if not (np.all(np.isfinite(dp)) and np.all(np.isfinite(dr))):
            self.nonfinite += 1
            STATE["nonfinite"] = self.nonfinite
            return
        locked = self.screw_deg >= L.LOCK_TURN_DEG
        sw_c = np.array([L.switch_center_x(self.u_sw), L.GROOVE_Y, L.BODY_TOP])
        d_hands = min(float(np.linalg.norm(p - sw_c)) for p in tcp_points(environment))
        d_drv = float(np.linalg.norm(dp - sw_c))
        hands_clear = min(d_hands, d_drv) >= L.HANDS_CLEAR_M
        self.d_clear = min(d_hands, d_drv)
        level = self.scorer.step(
            self.u_sw,
            self.travel_part,
            locked,
            self.valve_on,
            self.touching,
            self.screw_deg,
            hands_clear,
        )
        STATE.update(
            score=level,
            level=level,
            steps=self.steps,
            nonfinite=self.nonfinite,
            detail=dict(
                self.scorer.last,
                led=self.led,
                jaw_travel_mm=round(self.travel * 1e3, 3),
                clear_dist_mm=round(getattr(self, "d_clear", 0.0) * 1e3, 1),
                ovr_n=round(self.ovr_n, 3),
                ovr_peak_n=round(self.ovr_peak, 3),
                sw_force_n=np.round(self.sw_force, 3).tolist(),
                sw_axial_peak_n=round(self.sw_force_peak, 3),
                slide_total_mm=round(self.slide_mm, 3),
                engaged=self.engaged,
                engage_s=round(self.engage_s, 3),
                engage_push_peak_n=round(self.engage_push_peak, 3),
                engage=getattr(self, "engage_info", None),
                driver_tip_mm=np.round(dp * 1e3, 2).tolist(),
                valve_events=self.valve_events[-20:],
                setup=self.setup,
                hook_wall_s=round(self.hook_s, 2),
            ),
        )


_RT: dict = {"rt": None, "sim": None, "trials": 0}


def runtime():
    return _RT["rt"]


def bench_state():
    """Observable state for the scripted demo (read-only): body poses, LED, valve, jaw travel, switch position."""
    rt = _RT["rt"]
    if rt is None:
        return None
    d = rt.read()
    d.update(
        led=rt.led_now(),
        valve_on=rt.valve_on,
        travel=rt.travel,
        u_switch=rt.u_sw,
        screw_deg=rt.screw_deg,
        engaged=rt.engaged,
        ovr_n=rt.ovr_n,
        sw_force=rt.sw_force.copy(),
    )
    return d


def _patch_scene(scene) -> None:
    if getattr(scene, "_gs_patched", False):
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
            p, r = rt.read()["switch"] if rt else authored(SETUP_BASE)["switch"]
            z = np.zeros(3)
            out["bench"] = ObjectState(
                pos=np.asarray(p),
                quat_xyzw=_quat(r),
                lin_vel=z,
                ang_vel=z,
                low=p - 0.2,
                high=p + 0.2,
            )
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
    scene._gs_patched = True


def _has_bench(emb):
    return any(
        isinstance(o, Bench) for o in (emb.task.objects if emb.task is not None else ())
    )


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_gs_patched", False):
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
    cls._gs_patched = True


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
        type(f).__name__ == "_Finder"
        and getattr(f, "targets", None)
        and "yam_arena.scene" in f.targets
        for f in sys.meta_path
    ):
        sys.meta_path.insert(0, _Finder())


install()
