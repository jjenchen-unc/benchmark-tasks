"""cablekit: the stud offcut with its nail-on outlet box (push-in clamp tab) and the 14/2 NM-B cable, for yam_arena
(Isaac Lab-Arena / PhysX), plus the runtime hooks.

Authored as one custom-spawned assembly (yam_arena task-file primitives are single boxes/spheres):
* box: the stud and the box walls are colliders of one kinematic body (moved only on a reset); the clamp tab is a
  link of a small floating articulation (its base held on the box by a fixed joint) on a revolute joint along the opening's upper edge (hard stops at 55 deg (rest) and 88 deg from vertical) with an
  implicit articulation spring drive (target 20 deg below the rest stop: a preloaded spring);
* cable articulation (floating): 30 flat links (9.7 x 4.8 mm section; the 200 mm at the cut end in 10 mm links, the
  rest in 25 mm links) on spherical joints. Each joint's three rotations have implicit spring drives computed from the
  sheath's bending stiffness over the link length (flatwise, edgewise, twist) whose targets follow the joint past a
  yield moment: beyond it the bend stays (annealed copper holds its shape); a hard kink is possible.
Geometry: ``../src/layout.py``; scorer: ``../src/scoring.py``.

Computed elements (from measured poses, every physics step): the plastic update of the cable joints' drive targets,
and the one-way bite of the tab edge: while the tab bears on the sheath, a stiff one-way stop (up to 120 N) keeps the
cable from being pulled back out past the furthest point it was pushed to. Nothing pulls the cable in, centres it,
or writes a pose during a trial; ``Runtime.restore_authored`` lays the cable straight on the bench and moves the box
to the setup's pose only on an environment reset. The scripted demo READS poses through ``bench_state()``.
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
from scoring import measure, Scorer  # noqa: E402
from yam_arena.task_file import Box  # noqa: E402

PHYSICS_DT = float(os.environ.get("NMB_PHYSICS_DT", str(1 / 240)))
CONTACT_OFFSET = float(os.environ.get("NMB_CONTACT_OFFSET", ".0006"))
SETUP_BASE = int(os.environ.get("NMB_SETUP", "0"))
FORCE_NAMED = "NMB_SETUP" in os.environ
MIN_LINK_MASS = float(os.environ.get("NMB_MIN_MASS", "0.005"))
TAIL_MIN_MASS = 0.0025  # tail links (never squeezed in the demo): near real mass
LINK_INERTIA_MIN = float(os.environ.get("NMB_INERTIA", "4e-7"))  # kg m^2 floor per link axis (numerical: see README)
JOINT_ARMATURE = float(os.environ.get("NMB_ARMATURE", "2e-6"))  # kg m^2 per cable joint dof (numerical)

BOX_PVC, STUD_WOOD, TAB_COL, SHEATH, NAIL, PRINT = (
    (0.13, 0.13, 0.14),
    (0.80, 0.66, 0.45),
    (0.19, 0.19, 0.20),
    (0.93, 0.93, 0.90),
    (0.60, 0.60, 0.62),
    (0.15, 0.15, 0.15),
)
N_LINKS = L.LEAD_LINKS + L.TAIL_LINKS


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


def box_frame(spec):
    """Box articulation base pose: origin at the entry wall's inner face, opening upper edge, opening centre x."""
    o = L.box_origin(spec)
    return np.array([o[0]+L.ENTRY_X, L.entry_wall_y(spec), L.ENTRY_Z1]), np.eye(3)


# The uncut edge of the molded tongue is at the rear of the top-wall entry.
# Reverse the hinge axis and nose together so a positive joint angle still
# tips into the box. The shell frame stays on the front lip of the aperture.
TAB_BASIS = np.diag([-1.,1.,-1.])
TAB_OFFSET = np.array([0.,0.,-L.ENTRY_H])

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
    root.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0.0, 0.0, 0.0))
    root.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))
    looks, mats = {}, {}

    def look(color):
        if color not in looks:
            path = f"{prim_path}/Looks/c{len(looks)}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.5)
            vis.func(path, vis)
            looks[color] = path
        return looks[color]

    def material(friction):
        """Physics material; the clamp tab's smooth plastic uses 'min' combining (wins over 'average'), every other
        part 'average'."""
        mode = "min" if friction == L.TAB_FRICTION else "average"
        if friction not in mats:
            path = f"{prim_path}/Looks/phys{len(mats)}"
            pm = sim_utils.RigidBodyMaterialCfg(static_friction=friction, dynamic_friction=friction,
                                                friction_combine_mode=mode)
            pm.func(path, pm)
            mats[friction] = path
        return mats[friction]

    def xform(path, pos, rot):
        pos, rot = L.world_point(pos), L.FIXTURE_ROT @ rot
        x = UsdGeom.Xform.Define(stage, path)
        x.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*map(float, pos)))
        q = _quat(rot)
        x.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(float(q[3]), *map(float, q[:3])))
        return x

    def rigid(path, mass, iters=32, inertia=None, kinematic=False):
        prim = stage.GetPrimAtPath(path)
        rb = UsdPhysics.RigidBodyAPI.Apply(prim)
        if kinematic:
            rb.CreateKinematicEnabledAttr(True)
        ma = UsdPhysics.MassAPI.Apply(prim)
        ma.CreateMassAttr(float(mass))
        if inertia is not None:
            ma.CreateDiagonalInertiaAttr(Gf.Vec3f(*map(float, inertia)))
            ma.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, 0.0))
        px = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        px.CreateMaxDepenetrationVelocityAttr(0.2)
        px.CreateSolverPositionIterationCountAttr(iters)
        px.CreateSolverVelocityIterationCountAttr(4)
        px.CreateLinearDampingAttr(0.0)
        px.CreateAngularDampingAttr(0.05)
        px.CreateSleepThresholdAttr(0.0)
        px.CreateStabilizationThresholdAttr(0.0)
        PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
        return prim

    def hull(path, v, color, friction, collide=True, torsion=0.0):
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
            if torsion:
                # contact-patch (torsional) friction: a pad pinching the sheath resists the cable pivoting in the grip
                cp.CreateTorsionalPatchRadiusAttr(float(torsion))
                cp.CreateMinTorsionalPatchRadiusAttr(float(torsion))
            bind_physics_material(str(prim.GetPath()), material(friction), stage=stage)
        bind_visual_material(str(prim.GetPath()), look(color), stage=stage)
        return m

    def box(path, x0, x1, y0, y1, z0, z1, color, friction=L.BOX_FRICTION, collide=True):
        return hull(path, L.box_corners(x0, x1, y0, y1, z0, z1), color, friction, collide)

    def joint_frame(j, p0, r0, p1, r1):
        q0, q1 = _quat(r0), _quat(r1)
        j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, p0)))
        j.CreateLocalRot0Attr(Gf.Quatf(float(q0[3]), *map(float, q0[:3])))
        j.CreateLocalPos1Attr(Gf.Vec3f(*map(float, p1)))
        j.CreateLocalRot1Attr(Gf.Quatf(float(q1[3]), *map(float, q1[:3])))

    def drive(prim, dof, stiff_nm_rad, damp, max_nm, target_deg=0.0):
        d = UsdPhysics.DriveAPI.Apply(prim, dof)
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(float(stiff_nm_rad * math.pi / 180))  # USD angular drives are per degree
        d.CreateDampingAttr(float(damp * math.pi / 180))
        d.CreateMaxForceAttr(float(max_nm))
        d.CreateTargetPositionAttr(float(target_deg))

    spec = L.setup_spec(SETUP_BASE)
    # ------------------------------------------------------------------ box + stud: one kinematic body (moved only on
    # an environment reset); the tab: a small floating articulation (base + hinged tab) held on the box by a fixed joint
    bp_, br_ = box_frame(spec)
    bx = f"{prim_path}/box"
    xform(bx, bp_, br_)
    rigid(bx, 1.0, 64, kinematic=True)
    # box-frame coordinates: x across (opening centre 0), y from the entry wall's inner face (+ into the box),
    # z from the opening's upper edge
    for name, bounds in L.shell_parts().items():
        box(f"{bx}/{name}", *bounds, BOX_PVC)
    # The manufacturer's end view shows two molded tongues at either end.
    # Model the other three at their unworked rest angle. Only the selected
    # top entry has an open slot and the spring joint used during a trial.
    for tag,dx,lower in (("upper_far",-2*L.ENTRY_X,False),("lower_near",0.,True),
                         ("lower_far",-2*L.ENTRY_X,True)):
        vy=L.rz(180.) if lower else np.eye(3)
        origin=TAB_OFFSET + np.array([dx,L.BOX_IN_Y if lower else 0.,0.])
        axis=vy@TAB_BASIS@L.rx(math.radians(L.TAB_REST_DEG))
        verts=L.tab_vertices()@axis.T+origin
        hull(f"{bx}/{tag}_tongue",verts,TAB_COL,L.TAB_FRICTION)
    zr = L.BOX_RIM_Z - L.ENTRY_Z1
    ly = L.BOX_IN_Y
    sx = L.BOX_IN_X/2 + L.SIDE_WALL - L.ENTRY_X
    box(f"{bx}/bracket", sx, sx + 0.002, 0.010, ly - 0.010, zr - 0.030, zr - 0.010, BOX_PVC, collide=False)
    for k, yy in enumerate((0.022, ly - 0.022)):
        box(f"{bx}/nail_{k}", sx - 0.003, sx + 0.003, yy - 0.003, yy + 0.003, zr - 0.023, zr - 0.017, NAIL,
            collide=False)
    # printed "TOP" style marks are omitted; stud offcut (2x4) on its edge along y
    # Full-height vertical framing member. A narrow rear batten carries the movable service-loop clip.
    sy0 = L.ENTRY_HEIGHT - 0.490
    stz0 = zr - L.BOX_PROUD - L.STUD_H
    box(f"{bx}/stud", sx, sx + L.STUD_T, sy0, L.ENTRY_HEIGHT, stz0, stz0 + L.STUD_H, STUD_WOOD, friction=0.5)
    box(f"{bx}/foot", -0.24, sx + L.STUD_T + 0.018, L.ENTRY_HEIGHT - 0.015, L.ENTRY_HEIGHT,
        stz0 - 0.03, zr + 0.015, STUD_WOOD, friction=0.5)
    box(f"{bx}/rear_batten", -0.39, sx+L.STUD_T, -0.25, -0.14, -0.061, -0.043, STUD_WOOD)
    # clamp tab link: frame on the hinge (x along the hinge), tab plate hanging along -z, leaning into the box (+y)
    tbb = f"{prim_path}/tabbase"
    xform(tbb,bp_+TAB_OFFSET,br_@TAB_BASIS)
    bprim = rigid(tbb, 0.05, 64, inertia=(2e-5, 2e-5, 2e-5))
    UsdPhysics.ArticulationRootAPI.Apply(bprim)
    ar = PhysxSchema.PhysxArticulationAPI.Apply(bprim)
    ar.CreateEnabledSelfCollisionsAttr(False)
    ar.CreateSolverPositionIterationCountAttr(64)
    ar.CreateSolverVelocityIterationCountAttr(4)
    ar.CreateSleepThresholdAttr(0.0)
    ar.CreateStabilizationThresholdAttr(0.0)
    fj = UsdPhysics.FixedJoint.Define(stage, f"{tbb}/on_box")
    fj.CreateBody0Rel().SetTargets([Sdf.Path(bx)])
    fj.CreateBody1Rel().SetTargets([Sdf.Path(tbb)])
    joint_frame(fj, TAB_OFFSET,TAB_BASIS,np.zeros(3),np.eye(3))
    fj.CreateExcludeFromArticulationAttr(True)
    tb = f"{prim_path}/tab"
    rest = math.radians(L.TAB_REST_DEG)
    tr = L.rx(rest)
    xform(tb,bp_+TAB_OFFSET,br_@TAB_BASIS@tr)
    rigid(tb, 0.020, 64, inertia=(2e-6, 2e-6, 2e-6))
    # plate tapering to a wedge edge at its tip (moulded tab): no square end face for the cable end to butt against
    tab_geom = hull(f"{tb}/plate", L.tab_vertices(), TAB_COL, L.TAB_FRICTION)
    PhysxSchema.PhysxCollisionAPI(tab_geom.GetPrim()).GetContactOffsetAttr().Set(0.002)  # thin part: detect contact early
    rj = UsdPhysics.RevoluteJoint.Define(stage, f"{tb}/hinge")
    rj.CreateBody0Rel().SetTargets([Sdf.Path(tbb)])
    rj.CreateBody1Rel().SetTargets([Sdf.Path(tb)])
    UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(tb)).CreateFilteredPairsRel().AddTarget(bx)
    rj.CreateAxisAttr("X")
    joint_frame(rj, np.zeros(3), np.eye(3), np.zeros(3), np.eye(3))
    rj.CreateLowerLimitAttr(float(L.TAB_REST_DEG))
    rj.CreateUpperLimitAttr(float(L.TAB_OPEN_DEG))
    drive(rj.GetPrim(), "angular", L.TAB_STIFF, 0.002, 5.0, L.TAB_REST_DEG - L.TAB_PRELOAD_DEG)

    # ------------------------------------------------------------------ cable articulation (floating, root = cut end)
    links = L.cable_rest(spec)
    ls = L.link_lengths()
    paths = []
    for i, ((p, r), ln) in enumerate(zip(links, ls)):
        lp = f"{prim_path}/cable_{i:02d}"
        xform(lp, p, r)
        m_ = max(L.CABLE_LIN_MASS * ln, MIN_LINK_MASS if i < L.LEAD_LINKS else TAIL_MIN_MASS)
        ine = np.maximum(m_ / 12 * np.array([L.CABLE_W ** 2 + L.CABLE_T ** 2, ln ** 2 + L.CABLE_T ** 2,
                                             ln ** 2 + L.CABLE_W ** 2]), LINK_INERTIA_MIN)
        lprim = rigid(lp, m_, 64, ine)
        hw, ht = L.CABLE_W / 2, L.CABLE_T / 2
        # flat sheath with chamfered edges (octagonal section)
        c = 0.0004  # small edge round (a flat edge for the finger pads)
        sec = [(hw, ht - c), (hw - c, ht), (-hw + c, ht), (-hw, ht - c), (-hw, -ht + c), (-hw + c, -ht), (hw - c, -ht),
               (hw, -ht + c)]
        pts = [(sx_, y_, z_) for sx_ in (-ln / 2, ln / 2) for (y_, z_) in sec]
        if i == 0:  # cut end: its top and bottom edges slightly dressed (1 mm bevel), as a cut sheath end is
            pts = [(sx_, y_, z_) for sx_ in (-ln / 2,) for (y_, z_) in sec]
            pts += [(ln / 2 - 0.001, y_, z_) for (y_, z_) in sec]
            pts += [(ln / 2, y_, max(min(z_, ht - 0.001), -ht + 0.001)) for (y_, z_) in sec]
        hull(f"{lp}/geom", pts, SHEATH, L.CABLE_FRICTION, torsion=L.CABLE_PATCH_R)
        if i == 0:
            UsdPhysics.ArticulationRootAPI.Apply(lprim)
            ca = PhysxSchema.PhysxArticulationAPI.Apply(lprim)
            ca.CreateEnabledSelfCollisionsAttr(False)
            ca.CreateSolverPositionIterationCountAttr(64)
            ca.CreateSolverVelocityIterationCountAttr(4)
            ca.CreateSleepThresholdAttr(0.0)
            ca.CreateStabilizationThresholdAttr(0.0)
            # cut end: copper conductor ends visible
            for k, yy in enumerate((-0.0028, 0.0, 0.0028)):
                box(f"{lp}/cu_{k}", ln / 2, ln / 2 + 0.0006, yy - 0.0008, yy + 0.0008, -0.0008, 0.0008,
                    (0.80, 0.50, 0.25), collide=False)
        else:
            j = UsdPhysics.Joint.Define(stage, f"{lp}/bend")
            j.CreateBody0Rel().SetTargets([Sdf.Path(paths[-1])])
            j.CreateBody1Rel().SetTargets([Sdf.Path(lp)])
            joint_frame(j, (-ls[i - 1] / 2, 0, 0), links[i-1][1].T @ r, (ln / 2, 0, 0), np.eye(3))
            jp = j.GetPrim()
            for dof in ("transX", "transY", "transZ"):
                lim = UsdPhysics.LimitAPI.Apply(jp, dof)
                lim.CreateLowAttr(1.0)
                lim.CreateHighAttr(-1.0)
            lj = 0.5 * (ls[i - 1] + ln)
            for dof, ei, my, lim_deg in (("rotX", L.GJ, L.M_YIELD_TWIST, 90.0), ("rotY", L.EI_FLAT, L.M_YIELD_FLAT, 100.0),
                                         ("rotZ", L.EI_EDGE, L.M_YIELD_EDGE, 60.0)):
                lim = UsdPhysics.LimitAPI.Apply(jp, dof)
                lim.CreateLowAttr(-lim_deg)
                lim.CreateHighAttr(lim_deg)
                k = ei / lj
                drive(jp, dof, k, 0.002 * k, 4.0 * my)
            PhysxSchema.PhysxJointAPI.Apply(jp).CreateArmatureAttr(JOINT_ARMATURE)
        paths.append(lp)
    # The cable drapes over a physical round crosspiece. Both ends can slide freely.
    anchor = f"{prim_path}/service_clip"
    xform(anchor, L.guide_center(spec), np.eye(3))
    rigid(anchor, 0.2, kinematic=True)
    radius = L.DRAPE_RADIUS - L.CABLE_W/2
    rim = [(radius*math.cos(a), radius*math.sin(a)) for a in np.linspace(0, 2*math.pi, 32, endpoint=False)]
    vertices = [(x, y, z) for x,y in rim for z in (-0.090, 0.014)]
    hull(f"{anchor}/rounded_end", vertices, STUD_WOOD, 0.55)
    box(f"{anchor}/upper_stock_support", -0.345, 0., -radius, radius, -0.090, 0.014, STUD_WOOD, 0.55)
    from pxr import UsdShade, Vt
    def render_group(name,parts,color):
        mesh=UsdGeom.Mesh.Define(stage,f"{prim_path}/{name}")
        points=[];counts=[];indices=[];sources=[]
        for body_index,path,pos,rot in parts:
            shape=UsdGeom.Mesh(stage.GetPrimAtPath(path))
            local=np.array(shape.GetPointsAttr().Get(),dtype=float)
            start=len(points)
            points.extend(np.asarray(pos)+local@np.asarray(rot).T)
            counts.extend(shape.GetFaceVertexCountsAttr().Get())
            indices.extend(start+int(j) for j in shape.GetFaceVertexIndicesAttr().Get())
            sources.append((body_index,local))
        xyz=np.asarray(points,dtype=np.float32)
        mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(xyz))
        mesh.CreateFaceVertexIndicesAttr(indices)
        mesh.CreateFaceVertexCountsAttr(counts)
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateExtentAttr(Vt.Vec3fArray.FromNumpy(np.array([xyz.min(0),xyz.max(0)],dtype=np.float32)))
        bind_visual_material(str(mesh.GetPath()),look(color),stage=stage)
        return sources

    def link_part(i,name):
        p,r=links[i]
        return i,f"{paths[i]}/{name}",L.world_point(p),L.FIXTURE_ROT@r

    _SPEC["draw_groups"]={
        "sheath_surface":render_group("sheath_surface",[link_part(i,"geom") for i in range(N_LINKS)],SHEATH),
        "copper_surface":render_group("copper_surface",[link_part(0,f"cu_{k}") for k in range(3)],(.80,.50,.25)),
    }
    _SPEC["paths"] = dict(box=bx, tab=tb, cable=paths)
    _SPEC["joint_len"] = [0.5 * (ls[i - 1] + ls[i]) for i in range(1, N_LINKS)]
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
    """Plastic cable joints and the tab's one-way bite every physics step; scoring every control step."""

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
        cpaths = [f"{root}/cable_{i:02d}" for i in range(N_LINKS)]
        self.cpaths = cpaths
        self.box_art = self.simview.create_articulation_view(f"{root}/tabbase")
        self.boxv = self.simview.create_rigid_body_view([f"{root}/box"])
        self.anchor = self.simview.create_rigid_body_view([f"{root}/service_clip"])
        self.cable = self.simview.create_articulation_view(f"{root}/cable_00")
        self.links = self.simview.create_rigid_body_view(cpaths)
        self.tabv = self.simview.create_rigid_body_view([f"{root}/tab"])
        self.cdofs = list(self.cable.shared_metatype.dof_names)
        self.clinks = list(self.cable.shared_metatype.link_names)
        self.bdofs = list(self.box_art.shared_metatype.dof_names)
        robot_links = []
        for arm in ("left_robot", "right_robot"):
            lp = self.base.scene[arm].root_view.link_paths
            robot_links += list(lp[0] if lp and isinstance(lp[0], (list, tuple)) else lp)
        self.robot_links = robot_links
        self.cv_grip = self.simview.create_rigid_contact_view(
            cpaths, filter_patterns=[robot_links] * N_LINKS, max_contact_data_count=8192
        )
        self.cv_robot_box = self.simview.create_rigid_contact_view(
            [f"{root}/box", f"{root}/tab"], filter_patterns=[robot_links, robot_links], max_contact_data_count=4096
        )
        self.cv_cbox = self.simview.create_rigid_contact_view(
            cpaths, filter_patterns=[[f"{root}/box"]] * N_LINKS, max_contact_data_count=8192
        )
        self.cv_tab = self.simview.create_rigid_contact_view(
            [f"{root}/tab"], filter_patterns=[cpaths], max_contact_data_count=4096
        )
        self.origin = self.base.scene.env_origins[0].detach().cpu().numpy().astype(float)
        self.dev = self.cable.get_dof_positions().device
        self.ai = torch.tensor([0], dtype=torch.int32, device=self.dev)
        self.ai_cpu = torch.tensor([0], dtype=torch.int32)
        # per-dof yield angle: M_yield / k
        jl = _SPEC.get("joint_len") or [0.5 * (a + b) for a, b in zip(L.link_lengths()[:-1], L.link_lengths()[1:])]
        self.yield_rad = np.zeros(len(self.cdofs))
        self._map_dofs(jl)
        self.extra = {}  # development probes only: {link index: force vector (N, world)}
        self.setup = SETUP_BASE
        self.spec = L.setup_spec(self.setup)
        self.trial_active = False
        self.force_api = None
        self._init_state()
        print("NMB dofs", self.cdofs[:6], len(self.cdofs), "links", self.clinks[:3], len(self.clinks), "box dofs",
              self.bdofs, flush=True)

    def sync_visible_cable(self,links):
        from pxr import UsdGeom,Vt
        if not hasattr(self,"draw_groups"):
            self.draw_groups={name:(UsdGeom.Mesh(self.stage.GetPrimAtPath(f"{self.root}/{name}")),parts)
                              for name,parts in _SPEC["draw_groups"].items()}
        positions=np.array([L.world_point(p) for p,r in links])
        rotations=[L.FIXTURE_ROT@r for p,r in links]
        if not np.isfinite(positions).all() or not np.isfinite(rotations).all():return
        for mesh,parts in self.draw_groups.values():
            coords=np.concatenate([positions[i]+local@rotations[i].T for i,local in parts]).astype(np.float32)
            mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(coords))
            extent=np.array([coords.min(0),coords.max(0)],dtype=np.float32)
            mesh.GetExtentAttr().Set(Vt.Vec3fArray.FromNumpy(extent))

    def _map_dofs(self, jl):
        """Yield angle per cable dof. Dof names come from the joint prims (".../cable_07/bend" -> rotX/Y/Z order per
        joint as PhysX reports it); spherical-joint dofs are reported as (twist, swing1, swing2) = (rotX, rotY, rotZ)."""
        paths = list(getattr(self.cable, "dof_paths", [[]])[0]) if hasattr(self.cable, "dof_paths") else []
        n = len(self.cdofs)
        per = 3
        self.dof_joint = [d // per for d in range(n)]
        self.dof_axis = [d % per for d in range(n)]
        for d in range(n):
            j = self.dof_joint[d]
            lj = jl[min(j, len(jl) - 1)]
            ei, my = ((L.GJ, L.M_YIELD_TWIST), (L.EI_FLAT, L.M_YIELD_FLAT), (L.EI_EDGE, L.M_YIELD_EDGE))[self.dof_axis[d]]
            self.yield_rad[d] = my / (ei / lj)
        self.dof_paths_seen = paths[:6]

    def _init_state(self):
        self.scorer = Scorer()
        self.lead_hist = []
        self.steps = 0
        self.t = 0.0
        self.hook_s = 0.0
        self.nonfinite = 0
        self.grip_peak = 0.0
        self.grip_now = 0.0
        self.robot_box_peak = 0.0
        self.robot_box_link = None
        self.robot_box_point = None
        self.s_max = None
        self.bite_force = 0.0
        self.bite_peak = 0.0
        self.tab_peak = L.TAB_REST_DEG
        self.tab_force = 0.0
        self.cbox_now = (0.0, None)
        self.tab_force_peak = 0.0
        self.plastic_peak_deg = 0.0
        self.plastic_joint = None
        STATE.update(score=0.0, level=0.0, steps=0, detail={}, nonfinite=0, setup=self.setup)

    def begin_trial(self, trial):
        self._init_state()
        self.trial_active = True
        STATE.update(trial=trial, setup=self.setup, start=self.spec)

    # ------------------------------------------------------------------ reading
    def read_links(self):
        tf = self.links.get_transforms().detach().cpu().numpy().astype(float)
        return [L.fixture_pose(tf[i, :3] - self.origin, _rot(tf[i, 3:7])) for i in range(N_LINKS)]

    def link_speed(self, links):
        """Motion of the 200 mm lead section (the cable in and at the box; the tail on the bench is not judged): the
        largest displacement of any of its links over the last second, divided by that second (a video check; the
        instantaneous link speeds carry solver jitter while the cable stands still)."""
        from scoring import HOLD_STEPS

        pts = np.array([p for p, _ in links[: L.LEAD_LINKS]])
        self.lead_hist.append(pts)
        if len(self.lead_hist) > HOLD_STEPS + 1:
            self.lead_hist.pop(0)
        if len(self.lead_hist) < HOLD_STEPS + 1:
            return 1.0
        h = np.array(self.lead_hist)
        disp = float(np.linalg.norm(h - h[-1], axis=2).max())
        return disp / (HOLD_STEPS / 20.0)

    def tab_deg(self):
        q = self.box_art.get_dof_positions().detach().cpu().numpy()[0].astype(float)
        return math.degrees(float(q[0]))

    def box_pose(self):
        rt = self.boxv.get_transforms().detach().cpu().numpy()[0].astype(float)
        return L.fixture_pose(rt[:3] - self.origin, _rot(rt[3:7]))

    # ------------------------------------------------------------------ physics hook
    def _apply(self, f):
        """Apply per-link world forces (N, at the link centres)."""
        torch = self.torch
        ft = torch.as_tensor(f, dtype=torch.float32, device=self.dev)
        if self.force_api in (None, "rb"):
            try:
                idx = torch.arange(N_LINKS, dtype=torch.int32, device=self.dev)
                self.links.apply_forces(ft, idx, True)
                self.force_api = "rb"
                return
            except Exception as e:  # noqa: BLE001  (fall back to the articulation API once)
                print("NMB rigid-body apply_forces failed:", repr(e), flush=True)
                self.force_api = "art"
        order = [self.clinks.index(p.rsplit("/", 1)[1]) for p in self.cpaths]
        fa = torch.zeros((1, len(self.clinks), 3), dtype=torch.float32, device=self.dev)
        for i, k in enumerate(order):
            fa[0, k] = ft[i]
        self.cable.apply_forces_and_torques_at_position(fa, None, None, self.ai, True)

    def physics(self, dt):
        t0 = time.perf_counter()
        self.t += dt
        torch = self.torch
        # plastic update of the cable joints' drive targets
        q = self.cable.get_dof_positions().detach().cpu().numpy()[0].astype(float)
        tg = self.cable.get_dof_position_targets().detach().cpu().numpy()[0].astype(float)
        e = q - tg
        over = np.abs(e) > self.yield_rad
        if np.any(over):
            tg = np.where(over, q - np.sign(e) * self.yield_rad, tg)
            self.cable.set_dof_position_targets(torch.as_tensor(tg[None], dtype=torch.float32, device=self.dev),
                                                self.ai)
        if self.trial_active:
            pk = np.degrees(np.abs(tg[1::3]))
            if pk.max() > self.plastic_peak_deg:
                self.plastic_peak_deg = float(pk.max())
                self.plastic_joint = int(np.argmax(pk))  # joint k sits between links k and k+1 (k * 10 mm from the cut end)
        f = np.zeros((N_LINKS, 3))
        # one-way bite of the tab edge
        self.bite_force = 0.0
        links = self.read_links()
        if self.trial_active:
            ins = L.inserted_length(links, self.spec)
            tab = self.tab_deg()
            self.tab_peak = max(self.tab_peak, tab)
            if ins is not None and ins >= L.BITE_MIN_IN and tab >= L.TAB_REST_DEG + 1.5:
                if self.s_max is None or ins > self.s_max:
                    self.s_max = ins
                back = self.s_max - ins
                if back > 0.0003:
                    # the link that sits in the wall: force along the cable toward the cut end (into the box)
                    pts = L.cable_points(links)
                    y_in = L.entry_wall_y(self.spec)
                    k = next((i for i in range(N_LINKS) if pts[i + 1][1] < y_in), N_LINKS - 1)
                    ax = L.FIXTURE_ROT @ links[k][1][:, 0]
                    v = self.links.get_velocities().detach().cpu().numpy()[k, :3].astype(float)
                    fb = L.BITE_K * (back - 0.0003) - L.BITE_C * min(float(v @ ax), 0.0)
                    fb = float(np.clip(fb, 0.0, L.HOLD_N))
                    f[k] += fb * ax
                    self.bite_force = fb
                    self.bite_peak = max(self.bite_peak, fb)
            elif ins is None or ins < L.BITE_MIN_IN:
                self.s_max = None
            gm = self.cv_grip.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
            if gm.size:
                g = float(np.linalg.norm(gm, axis=-1).max())
                self.grip_now = g
                self.grip_peak = max(self.grip_peak, g)
            tm = self.cv_tab.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
            if tm.size:
                self.tab_force = float(np.linalg.norm(tm.reshape(-1, 3).sum(axis=0)))
                self.tab_force_peak = max(self.tab_force_peak, self.tab_force)
            cb = self.cv_cbox.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
            if cb.size:
                nb = np.linalg.norm(cb, axis=-1).reshape(N_LINKS, -1).max(axis=1)
                self.cbox_now = (round(float(nb.max()), 2), int(np.argmax(nb)))
            rb = self.cv_robot_box.get_contact_force_matrix(dt=dt).detach().cpu().numpy()
            if rb.size:
                nr = np.linalg.norm(rb, axis=-1).max(axis=0)
                j = int(np.argmax(nr))
                if nr[j] > self.robot_box_peak:
                    if nr[j] > 2.0 and self.robot_box_point is None:
                        try:
                            cd = self.cv_robot_box.get_contact_data(dt=dt)
                            fz = cd[0].detach().cpu().numpy().ravel()
                            pts = cd[1].detach().cpu().numpy().reshape(-1, 3)
                            if fz.size:
                                self.robot_box_point = np.round((pts[int(np.argmax(fz))] - self.origin) * 1e3, 1).tolist()
                        except Exception as e:  # noqa: BLE001  (diagnostic only)
                            self.robot_box_point = repr(e)[:80]
                    self.robot_box_peak = float(nr[j])
                    self.robot_box_link = "/".join(self.robot_links[j].split("/")[-2:]) if nr[j] > 0.5 else None
        for k, fv in self.extra.items():
            f[k] += np.asarray(fv, float)
        if np.any(f):
            self._apply(f)
        self.hook_s += time.perf_counter() - t0

    # ------------------------------------------------------------------ reset
    def restore_authored(self, setup):
        """Environment reset only (never during a trial): box to the setup's pose, tab at rest, cable draped over the upper timber support."""
        torch = self.torch
        self.setup = setup
        self.spec = L.setup_spec(setup)
        self.trial_active = False
        bp, br = box_frame(self.spec)
        bp, br = L.world_point(bp), L.FIXTURE_ROT @ br
        kt = self.boxv.get_transforms().clone()
        kt[0, :3] = torch.tensor(bp + self.origin, dtype=kt.dtype)
        kt[0, 3:7] = torch.tensor(_quat(br), dtype=kt.dtype)
        self.boxv.set_transforms(kt, torch.tensor([0], dtype=torch.int32, device=kt.device))
        rt = self.box_art.get_root_transforms().clone()
        rt[0, :3] = torch.tensor(bp+br@TAB_OFFSET+self.origin,dtype=rt.dtype)
        rt[0, 3:7] = torch.tensor(_quat(br@TAB_BASIS),dtype=rt.dtype)
        self.box_art.set_root_transforms(rt, self.ai)
        self.box_art.set_root_velocities(torch.zeros_like(self.box_art.get_root_velocities()), self.ai)
        bq = self.box_art.get_dof_positions().clone()
        bq[:] = math.radians(L.TAB_REST_DEG)
        self.box_art.set_dof_positions(bq, self.ai)
        self.box_art.set_dof_velocities(torch.zeros_like(bq), self.ai)
        links = L.cable_rest(self.spec)
        p0, r0 = links[0]
        p0, r0 = L.world_point(p0), L.FIXTURE_ROT @ r0
        ep, er = L.guide_center(self.spec), np.eye(3)
        anchor = self.anchor.get_transforms().clone()
        anchor[0, :3] = torch.tensor(L.world_point(ep) + self.origin, dtype=anchor.dtype)
        anchor[0, 3:] = torch.tensor(_quat(L.FIXTURE_ROT @ er), dtype=anchor.dtype)
        self.anchor.set_transforms(anchor, self.ai)
        ct = self.cable.get_root_transforms().clone()
        ct[0, :3] = torch.tensor(p0 + self.origin, dtype=ct.dtype)
        ct[0, 3:7] = torch.tensor(_quat(r0), dtype=ct.dtype)
        self.cable.set_root_transforms(ct, self.ai)
        self.cable.set_root_velocities(torch.zeros_like(self.cable.get_root_velocities()), self.ai)
        cq = self.cable.get_dof_positions().clone()
        self.cable.set_dof_positions(torch.zeros_like(cq), self.ai)
        self.cable.set_dof_velocities(torch.zeros_like(cq), self.ai)
        self.cable.set_dof_position_targets(torch.zeros_like(cq), self.ai)
        self.sync_visible_cable(links)
        self._init_state()

    def dev_place_cable(self, links):
        """Development probes only (never in a task run): lay the cable straight at the given link poses."""
        torch = self.torch
        p0, r0 = links[0]
        p0, r0 = L.world_point(p0), L.FIXTURE_ROT @ r0
        ct = self.cable.get_root_transforms().clone()
        ct[0, :3] = torch.tensor(p0 + self.origin, dtype=ct.dtype)
        ct[0, 3:7] = torch.tensor(_quat(r0), dtype=ct.dtype)
        self.cable.set_root_transforms(ct, self.ai)
        self.cable.set_root_velocities(torch.zeros_like(self.cable.get_root_velocities()), self.ai)
        cq = self.cable.get_dof_positions().clone()
        self.cable.set_dof_positions(torch.zeros_like(cq), self.ai)
        self.cable.set_dof_velocities(torch.zeros_like(cq), self.ai)
        self.cable.set_dof_position_targets(torch.zeros_like(cq), self.ai)

    # ------------------------------------------------------------------ scoring
    def control(self, environment):
        if not self.trial_active:
            return
        self.steps += 1
        links = self.read_links()
        if not all(np.all(np.isfinite(p)) for p, _ in links):
            self.nonfinite += 1
            STATE["nonfinite"] = self.nonfinite
            return
        self.sync_visible_cable(links)
        tips = [L.fixture_point(t) for t in finger_tips(environment)]
        tab = self.tab_deg()
        m = measure(self.spec, links, tab, tips)
        level = self.scorer.step(m, self.link_speed(links))
        STATE.update(
            score=level,
            level=level,
            steps=self.steps,
            nonfinite=self.nonfinite,
            detail=dict(
                self.scorer.last,
                bite_n=round(self.bite_force, 2),
                bite_peak_n=round(self.bite_peak, 2),
                s_max_mm=None if self.s_max is None else round(self.s_max * 1e3, 2),
                tab_peak_deg=round(self.tab_peak, 2),
                tab_force_n=round(self.tab_force, 2),
                cable_box_n=self.cbox_now,
                tab_force_peak_n=round(self.tab_force_peak, 2),
                grip_now_n=round(self.grip_now, 2),
                grip_peak_n=round(self.grip_peak, 2),
                robot_box_peak_n=round(self.robot_box_peak, 2),
                robot_box_link=self.robot_box_link,
                robot_box_point_mm=self.robot_box_point,
                plastic_peak_deg=round(self.plastic_peak_deg, 2),
                plastic_joint=self.plastic_joint,
                lead_mm=[[round(float(v), 1) for v in (p * 1e3)] for p, r in links[:6]],
                lead_pitch_deg=[round(float(math.degrees(math.asin(max(-1.0, min(1.0, r[2, 0]))))), 1) for p, r in links[:6]],
                setup=self.setup,
            ),
        )


_RT: dict = {"rt": None, "sim": None, "trials": 0}


def runtime():
    return _RT["rt"]


def bench_state():
    """Current cable link poses, box pose and tab angle for the scripted demo (read-only), plus the start spec."""
    rt = _RT["rt"]
    if rt is None:
        return None
    return dict(links=rt.read_links(), box=rt.box_pose(), tab_deg=rt.tab_deg(), spec=rt.spec)


def _patch_scene(scene) -> None:
    if getattr(scene, "_nmb_patched", False):
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
            p, r = rt.read_links()[0] if rt else L.cable_rest(L.setup_spec(SETUP_BASE))[0]
            p, r = L.world_point(p), L.FIXTURE_ROT @ r
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
    scene._nmb_patched = True


def _has_bench(emb):
    return any(isinstance(o, Bench) for o in (emb.task.objects if emb.task is not None else ()))


def _patch_embodiment(ie) -> None:
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_nmb_patched", False):
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
            if rt.setup != setup:
                rt.restore_authored(setup)
                obs = base_reset(self, *a, **k)
            rt.restore_authored(setup)
            rt.begin_trial(_RT["trials"])
            _RT["trials"] += 1
        return obs

    cls._ensure_sim, cls._task_step, cls.reset = _ensure_sim, _task_step, reset
    cls._nmb_patched = True


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
