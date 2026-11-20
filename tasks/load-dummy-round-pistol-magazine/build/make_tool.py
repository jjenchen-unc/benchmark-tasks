"""make_tool.py: CadQuery model of the loading tool v3 from toolgeom.prims(); exports STEP/STL (print orientation and
in-use frame), the collider json for the simulation, overlap checks against the magazine STL, the magazine collider and
the fixture. Run: python make_tool.py MAG_STL (MAG_STL = the community-CAD magazine body STL in inches; not bundled,
see README.md). Writes the STEP/STL here and ../isaac/tool/tool_hulls.json."""
import json
import sys
from pathlib import Path

import cadquery as cq
import numpy as np
import trimesh

H = Path(__file__).resolve().parent
sys.path[:0] = [str(H), str(H.parent / "isaac")]
import geom  # noqa: E402
import toolgeom as g  # noqa: E402

RHO = 1.24e-3  # g/mm3, PLA, 100 % infill


def solid(v):
    n = len(v) // 2
    poly = [(float(p), float(u)) for p, _, u in v[:n]]
    y0, y1 = float(v[0, 1]), float(v[n, 1])
    w = cq.Workplane("XY").workplane(offset=-y1).polyline(poly).close().extrude(y1 - y0)
    return w.rotate((0, 0, 0), (1, 0, 0), 90)  # (p, u, -y) -> (p, y, u)


def vol(m):
    return round(float(m.volume), 3) if len(m.faces) else 0.0


P = g.prims()
part = None
for k, v in P.items():
    assert g.ccw(v[:len(v) // 2][:, [0, 2]]), k
    s = solid(v)
    bb = s.val().BoundingBox()
    assert np.allclose([bb.xmin, bb.ymin, bb.zmin, bb.xmax, bb.ymax, bb.zmax], np.r_[v.min(0), v.max(0)], atol=1e-6), k
    part = s if part is None else part.union(s)
part = part.clean()
assert len(part.solids().vals()) == 1
v_cq = part.val().Volume()
com = part.val().Center()
# print orientation: roof top (u = U_T) on the bed, tube axis vertical
pr = part.rotate((0, 0, 0), (1, 0, 0), 180).translate((0, 0, g.U_T))
cq.exporters.export(pr, str(H / "loading_tool3.step"))
cq.exporters.export(pr, str(H / "loading_tool3.stl"), tolerance=0.01, angularTolerance=0.1)
# in-use frame (magazine frame, tool pressed): rotate about y by the rake, shift
use = part.translate((-110.0 * g.S, 0, 0)).rotate((0, 0, 0), (0, 1, 0), np.degrees(g.RAKE))
cq.exporters.export(use, str(H / "loading_tool3_in_use_frame.stl"), tolerance=0.01, angularTolerance=0.1)
tm = trimesh.load(H / "loading_tool3_in_use_frame.stl")
allv = np.vstack([g.to_mag(v) for v in P.values()])
assert np.allclose(tm.bounds, [allv.min(0), allv.max(0)], atol=0.02), (tm.bounds, allv.min(0), allv.max(0))
hulls = [trimesh.convex.convex_hull(g.to_mag(v)) for v in P.values()]
un = trimesh.boolean.union(hulls, engine="manifold")
com_m = g.to_mag(np.array([com.x, com.y, com.z]))
print("volume cadquery %.1f mm3, hull union %.1f mm3, stl %.1f, watertight %s" % (v_cq, un.volume, tm.volume, tm.is_watertight))
print("mass %.1f g (PLA 1.24, 100 %%), com mag frame" % (v_cq * RHO), com_m.round(2), "bbox in use", tm.bounds.round(2).tolist())
ppr = trimesh.load(H / "loading_tool3.stl")
print("print bbox", ppr.bounds.round(2).tolist())

# magazine: raw STL (inches) -> magazine frame; collider hulls; fixture
mag = trimesh.load(sys.argv[1])
V = mag.vertices * 25.4
mag = trimesh.Trimesh(np.c_[V[:, 0] - 78.0, -(V[:, 2] - 22.3093), V[:, 1]], mag.faces)
col = trimesh.boolean.union([trimesh.convex.convex_hull(h * 1000) for h in geom.body_hulls() if h[:, 2].max() > 0.06], engine="manifold")
fx = [trimesh.creation.box(extents=np.array(s) * 1000, transform=trimesh.transformations.translation_matrix(np.array(c) * 1000)) for c, s, _ in geom.FIX_BOXES]
fx += [trimesh.convex.convex_hull(np.array(v) * 1000) for v, _ in geom.FIX_HULLS]
pc, pr_, pl_ = np.array(geom.FIX_PIN[0]) * 1000, geom.FIX_PIN[1] * 1000, geom.FIX_PIN[2] * 1000
fx.append(trimesh.creation.cylinder(radius=pr_, height=pl_, transform=trimesh.transformations.translation_matrix(pc) @ trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0])))
fix = trimesh.boolean.union(fx, engine="manifold")
Uv = np.array([g.S, 0, g.C])


def overlap(shift):
    t = tm.copy()
    t.apply_translation(shift)
    return tuple(vol(trimesh.boolean.intersection([t, m], engine="manifold")) for m in (col, mag, fix))


chk = {}
for r in (0.0, 0.05, 0.7, 2, 5, 10, 14, 20, 30, 45, 60, 72):
    chk["up %g" % r] = overlap(Uv * r)
    print("raised %5.2f mm along U: overlap with collider %.3f mm3, raw STL %.3f, fixture %.3f" % (r, *chk["up %g" % r]))
for dx in (0, 3, 6, 9, 10.5):  # nose push: tooth underside 0.05 mm above the front-wall top, tool moved rearward
    chk["push %g" % dx] = overlap(Uv * g.S_PUSH + [-dx, 0, 0.05])
    print("push pose, %4.1f mm rearward: collider %.3f, raw STL %.3f, fixture %.3f" % (dx, *chk["push %g" % dx]))
for dy in (-1.5, 1.5):
    print("push pose, 9 mm rearward, %+.1f mm sideways: collider %.3f, raw STL %.3f, fixture %.3f" % (dy, *overlap(Uv * g.S_PUSH + [-9, dy, 0.05])))
# clearance of the tongue in the lip gap: a tongue fattened by 1.2 mm per side (y) must still not touch
fat = g.prims()["tongue"].copy()
fat[:, 1] *= (g.TONGUE_HW + 1.2) / g.TONGUE_HW
fh = trimesh.convex.convex_hull(g.to_mag(fat))
for r in (0.0, 5, 10, 14):
    t = fh.copy()
    t.apply_translation(Uv * r)
    print("tongue +1.2 mm per side, raised %4.1f: collider %.3f, raw STL %.3f mm3" % (r, vol(trimesh.boolean.intersection([t, col], engine="manifold")), vol(trimesh.boolean.intersection([t, mag], engine="manifold"))))
tf = g.to_mag(g.prims()["tooth"]) + Uv * g.S_PUSH
print("tooth in the push pose (mag frame): x %.2f to %.2f at the bottom, z %.2f to %.2f, face x at z 110.4: %.2f" % (
    tf[0, 0], tf[1, 0], tf[:, 2].min(), tf[:, 2].max(), tf[0, 0] + (110.4 - tf[0, 2]) * np.tan(g.TOOTH_LEAN)))
meta = dict(mass_g=round(v_cq * RHO, 2), volume_mm3=round(v_cq, 1), com_mag_mm=com_m.round(3).tolist(), z_foot=g.Z_FOOT, depress=g.DEPRESS, z_tip=round(g.Z_TIP, 3),
            u_r=round(g.U_R, 3), u_t=round(g.U_T, 3), s_push=round(g.S_PUSH, 3), tooth_face_x_mm=round(float(g.to_mag(g.prims()["tooth"])[0, 0]), 3),
            tooth_lean_deg=round(float(np.degrees(g.TOOTH_LEAN)), 1), grip_tip_mm=g.grip_point().round(3).tolist(), bbox_mm=tm.bounds.round(3).tolist(),
            hulls={k: g.to_mag(v).round(4).tolist() for k, v in P.items()}, overlap_check=chk)
(H.parent / "isaac" / "tool" / "tool_hulls.json").write_text(json.dumps(meta))
