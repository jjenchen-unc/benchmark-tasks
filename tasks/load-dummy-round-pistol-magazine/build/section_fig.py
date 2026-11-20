"""section_fig.py: drawings of the loading tool v3 on the magazine. Left: side section at y = 0, tool seated and fully
pressed, with the finger seat. Right top: the nose-push pose (tool 60.3 mm up the tube axis, then moved rearward).
Right bottom: cross section through the tongue. Run after make_tool.py."""
import json
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import trimesh

H = Path(__file__).resolve().parent
sys.path.insert(0, str(H))
import toolgeom as g  # noqa: E402


def hull_union(f, zmin=60):
    d = np.load(H.parent / "isaac" / "collider" / f)
    out, k = [], 0
    for n in d["n"]:
        out.append(trimesh.convex.convex_hull(d["verts"][k:k + n].astype(float) * 1000))
        k += n
    return trimesh.boolean.union([h for h in out if h.bounds[1][2] > zmin], engine="manifold")


def sec(ax, mesh, origin, normal, i, j, shift=(0, 0), **kw):
    s = mesh.section(plane_origin=origin, plane_normal=normal)
    if s is None:
        return
    lab = kw.pop("label", None)
    for n, p in enumerate(s.discrete):
        ax.fill(p[:, i] + shift[0], p[:, j] + shift[1], label=lab if n == 0 else None, **kw)


def poly(ax, v, shift=(0, 0), **kw):
    m = g.to_mag(np.c_[v[:, 0], np.zeros(len(v)), v[:, 1]])
    ax.fill(m[:, 0] + shift[0], m[:, 2] + shift[1], **kw)


def round_(ax, xb, zc, **kw):
    rc, lc, rb, lb = 4.98, 19.15, 4.51, 5.5
    t = np.linspace(-math.pi / 2, math.pi / 2, 30)
    ax.plot(np.r_[xb, xb + lc, xb + lc, xb + lc + lb, xb + lc + lb + rb * np.cos(t)[::-1], xb + lc, xb + lc, xb, xb],
            np.r_[zc + rc, zc + rc, zc + rb, zc + rb, zc + rb * np.sin(t)[::-1], zc - rb, zc - rc, zc - rc, zc + rc], **kw)


M = json.loads((H.parent / "isaac" / "tool" / "tool_hulls.json").read_text())
tool = trimesh.load(H / "loading_tool3_in_use_frame.stl")
mag = hull_union("body_hulls.npz")
fol = hull_union("follower_hulls.npz", 90)
Uv = np.array([g.S, 0, g.C])
fol_dn = fol.copy()
fol_dn.apply_translation(-Uv * g.DEPRESS)
fig = plt.figure(figsize=(19, 10))
gs = fig.add_gridspec(2, 2, width_ratios=[1.9, 1], height_ratios=[1, 1])
a = fig.add_subplot(gs[:, 0])
sec(a, mag, [0, 4.6, 0], [0, 1, 0], 0, 2, fc="#dddddd", ec="#999999", lw=0.6, label="magazine at y 4.6 (lip)")
sec(a, mag, [0, 0, 0], [0, 1, 0], 0, 2, fc="#888888", ec="k", lw=0.6, label="magazine at y 0")
sec(a, fol_dn, [0, 2.4, 0], [0, 1, 0], 0, 2, fc="#9cc3ee", ec="#3070c0", lw=0.8, label="follower, 5.0 mm down the tube axis")
P = g.prims()
v = g.to_mag(P["side_l"])
a.fill(v[:5, 0], v[:5, 2], fc="#ffd9a8", ec="#c06000", lw=0.8, alpha=0.6, label="tool side wall (|y| 11.65 to 14.15)")
v = g.to_mag(P["front_wall"])
a.fill(v[:5, 0], v[:5, 2], fc="#ffc27a", ec="#c06000", lw=0.8, alpha=0.8, label="front wall (bottom 6.7 mm above the standing plane)")
sec(a, tool, [0, 0, 0], [0, 1, 0], 0, 2, fc="#f08020", ec="k", lw=0.8, label="tool at y 0 (fin 7 mm thick, tongue, tooth)")
for n, (k, pv) in enumerate(g.seat().items()):
    poly(a, pv, fc="#7a3b00", ec="k", lw=0.8, label="finger seat lugs on both fin faces (4.5 mm proud)" if n == 0 else None)
b = g.blade()
tf = lambda q: np.array(g.xz(q[0], q[1]))  # noqa: E731
L = 13.9
bl = np.array([tf(b["rc"]), tf(b["fc"]), tf(b["fc"] + L * b["ef"]), tf(b["fc"] + L * b["ef"] - 22 * b["a"] + 6 * b["n"]), tf(b["rc"] + L * b["er"] - 22 * b["a"] - 6 * b["n"]), tf(b["rc"] + L * b["er"])])
a.fill(bl[:, 0], bl[:, 1], fc="none", ec="#208020", lw=1.4, ls="-.", label="tool arm fingertip blades (pinch the fin across y, gripper 40 deg back)")
a.annotate("s1: ridge under the\nblades' rear edge\n(press, rearward pull)", tf(g.seat()["s1"][2]), (-99, 86), fontsize=8, arrowprops=dict(arrowstyle="->"))
a.annotate("s2: stop ahead of\nthe fingertip ends", tf(g.seat()["s2"][3]), (-36, 62), fontsize=8, arrowprops=dict(arrowstyle="->"))
a.annotate("w: stop above the blades'\nfront edge (lift)", tf(g.seat()["w"][3]), (-30, 104), fontsize=8, arrowprops=dict(arrowstyle="->"))
tm_ = g.to_mag(P["tooth"])
a.annotate("nose-pusher tooth\n4 mm wide (y), 5 mm thick,\nface 15 deg", tm_[1][[0, 2]], (14, 60), fontsize=8, arrowprops=dict(arrowstyle="->"))
xb = (g.P_T1 / g.C + (107.3 - 110) * g.T)
round_(a, xb + 1.3, 107.3, color="#a07000", lw=1.2, label="round after the entry (schematic)")
p0 = np.array(g.xz(g.P_T0, g.U_R))
a.annotate("", p0 + [14 * g.S, 14 * g.C + 9], p0 + [0, 9], arrowprops=dict(arrowstyle="<->", color="r"))
a.text(-9, 138, "slides along the\ntube axis (21 deg)", color="r", fontsize=9)
bb = tool.bounds
txt = [f"one part, {bb[1][0] - bb[0][0]:.1f} x {bb[1][1] - bb[0][1]:.1f} x {bb[1][2] - bb[0][2]:.1f} mm in use, {M['mass_g']:.1f} g PLA at 100 % ({M['volume_mm3'] / 1000:.1f} cm3)",
       f"ring inside {g.P_FI - g.P_RI:.2f} x {2 * g.Y_I:.2f} mm (tube {g.P_FRONT - g.P_REAR:.2f} x {2 * g.W_HALF:.1f}); clearance {g.CLR} front and sides, {g.CLR_REAR} rear; walls {g.WALL} mm",
       f"tongue {2 * g.TONGUE_HW:.1f} wide (y) x {g.TONGUE_T:.1f} thick, tip z {g.Z_TIP:.2f} (follower travel {g.DEPRESS:.1f} mm); rear face {g.P_T0 - g.P_REAR_IN:.2f} mm ahead of the rear wall inside",
       f"roof {g.ROOF_T:.0f} mm, 1.0 mm above the lip tips; stop = ring bottom on the fixture tower top (z {g.Z_FOOT})",
       f"fin {2 * g.FIN_HW:.0f} mm thick, {g.FIN_L:.0f} long; seat lugs {g.LUG_W} mm proud; fingertip ends at x {g.GRIP_XZ[0]:.0f}, z {g.GRIP_XZ[1]:.0f}",
       f"tooth: |y| <= {g.TOOTH_HW:.0f}, {g.TOOTH_T:.0f} thick at the bottom, {g.RELIEF} mm below the front wall's bottom edge, face {math.degrees(g.TOOTH_LEAN):.0f} deg",
       f"nose push: tool {g.S_PUSH:.1f} mm up the tube axis (tooth underside on the front-wall top, z {g.Z_FRONT_TOP}), then rearward",
       "print: upside down (roof on the bed, tube axis vertical), no supports; all faces looking down in the print are",
       "45 deg or steeper except 1 mm flats; front wall = 23 mm bridge"]
a.text(-104, 49, "\n".join(txt), fontsize=8, va="top", family="monospace")
a.set_xlim(-106, 62)
a.set_ylim(22, 150)
a.set_aspect("equal")
a.grid(alpha=0.3)
a.set_xlabel("magazine x, mm (front = +)")
a.set_ylabel("z, mm above the floor-plate bottom")
a.set_title("side section (magazine frame), tool seated and fully pressed")
a.legend(loc="upper right", fontsize=7)

c = fig.add_subplot(gs[0, 1])
sec(c, mag, [0, 4.6, 0], [0, 1, 0], 0, 2, fc="#dddddd", ec="#999999", lw=0.6)
sec(c, mag, [0, 0, 0], [0, 1, 0], 0, 2, fc="#888888", ec="k", lw=0.6)
sec(c, fol, [0, 2.4, 0], [0, 1, 0], 0, 2, shift=tuple(-Uv[[0, 2]] * 1.8), fc="#9cc3ee", ec="#3070c0", lw=0.8)
for dx, al, ls in ((0.0, 0.35, "--"), (-7.8, 1.0, "-")):
    sh = Uv[[0, 2]] * g.S_PUSH + [dx, 0.0]
    v = g.to_mag(P["front_wall"])
    c.fill(v[:5, 0] + sh[0], v[:5, 2] + sh[1], fc="#ffc27a", ec="#c06000", lw=0.8, alpha=0.6 * al, ls=ls)
    v = g.to_mag(P["tooth"])
    c.fill(v[:4, 0] + sh[0], v[:4, 2] + sh[1], fc="#f08020", ec="k", lw=0.9, alpha=al, ls=ls)
round_(c, -9.0, 110.5, color="#a07000", lw=1.0, ls="--")
round_(c, -16.9, 110.4, color="#a07000", lw=1.4)
c.annotate("", (13.2, 119.5), (21.0, 119.5), arrowprops=dict(arrowstyle="->", color="r"))
c.text(10.5, 121, "tool pulled rearward\n(7 to 10 mm)", color="r", fontsize=8)
c.text(-19, 97.5, "dashed: round after the entry and tooth at the start\nsolid: round at the rear wall; the tooth's underside\nrides on the front-wall top (z 107.38)", fontsize=8, va="top")
c.set_xlim(-20, 34)
c.set_ylim(92, 124)
c.set_aspect("equal")
c.grid(alpha=0.3)
c.set_xlabel("magazine x, mm")
c.set_title(f"nose push with the tooth: tool {g.S_PUSH:.1f} mm up the tube axis (section y = 0)")

d = fig.add_subplot(gs[1, 1])
x0 = -13.0
sec(d, mag, [x0, 0, 0], [1, 0, 0], 1, 2, fc="#888888", ec="k", lw=0.6)
sec(d, fol_dn, [x0 - 1.8, 0, 0], [1, 0, 0], 1, 2, fc="#9cc3ee", ec="#3070c0", lw=0.8)
sec(d, tool, [x0 - 1.2, 0, 0], [1, 0, 0], 1, 2, fc="#f08020", ec="k", lw=0.8)
d.annotate("", (-g.TONGUE_HW, 113.6), (-4.25, 113.6), arrowprops=dict(arrowstyle="<->", color="r"))
d.text(-9.5, 112.6, f"{4.25 - g.TONGUE_HW:.2f} per side\n(gap 8.5)", color="r", fontsize=8, ha="right")
d.set_xlim(-18, 18)
d.set_ylim(92, 126)
d.set_aspect("equal")
d.grid(alpha=0.3)
d.set_xlabel("y, mm")
d.set_title("cross section through the tongue (x -13 / -14.2), tool pressed")
fig.tight_layout()
fig.savefig(H / "tool3_section.png", dpi=120)
