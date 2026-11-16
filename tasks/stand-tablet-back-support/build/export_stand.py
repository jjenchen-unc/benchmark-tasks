"""Export the folding back support as printable STL files and a dimensioned assembly drawing.

    pip install trimesh manifold3d matplotlib numpy scipy
    python -B build/export_stand.py            # from the task folder; writes into build/

Panel size, hinge line and magnet end are read from mechanism.py. Millimetres; the tablet's frame as in
MECHANISM-SPEC.md (+Y portrait-up, +Z out of the screen, back face at z = -3.5), with y measured from the bottom edge.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np
import trimesh

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import mechanism as M  # noqa: E402

MM = 1000.0
IPAD = (179.5, 248.6, 7.0)                       # iPad (A16) width, height, depth
BOTTOM = -M.TABLET_SIZE[1] / 2 * MM              # the simulated tablet's bottom edge, y = -120
BACK = -M.TABLET_SIZE[2] / 2 * MM                # back face, z = -3.5
PW, PL, PT = M.PANEL_SIZE * MM                   # 134 x 150 x 1.3
SPACER_T = -M.ROOT[2] * MM + BACK                # back face to the simulated hinge axis: 2.5 mm
HINGE1_UP = M.ROOT[1] * MM - BOTTOM              # hinge line above the bottom edge: 150 mm
LAND_T = SPACER_T + 2 * PT                       # hinge land, flush with folded panel 1's outer face: 5.1 mm
STRIP_W = 140.0
HINGE_STRIP = (HINGE1_UP - 10.0, HINGE1_UP + 1.0, HINGE1_UP + 11.0)   # spacer y0, land y0, land y1
FOOT_STRIP = (1.5, 13.5)                         # raised 1.5 mm so panel 2's end passes under it
SHIM = 0.3
MAG_X, MAG_FROM_TIP = 45.0, 3.0                  # magnet centres on panel 2, from its free end
MAG_D, MAG_H = 4.9, 0.9                          # pocket for a 3/16 x 1/32 in (4.8 x 0.8 mm) disc


def _block(x, y0, y1, t) -> trimesh.Trimesh:
    return trimesh.creation.box(extents=(x, y1 - y0, t)).apply_translation((0, (y0 + y1) / 2, t / 2))


def hinge_strip() -> trimesh.Trimesh:
    y0, y1, y2 = HINGE_STRIP
    return trimesh.boolean.union([_block(STRIP_W, y0, y2, SPACER_T), _block(STRIP_W, y1, y2, LAND_T)], engine="manifold")


def foot_strip() -> trimesh.Trimesh:
    return _block(STRIP_W, *FOOT_STRIP, SPACER_T)


def panel() -> trimesh.Trimesh:
    return trimesh.creation.box(extents=(PW, PL, PT))


def panel2() -> trimesh.Trimesh:
    """Panel 2 with two magnet pockets in its +Z face (the face toward panel 1 when folded) at its free end, y = -75."""
    pockets = [trimesh.creation.cylinder(radius=MAG_D / 2, height=2 * MAG_H, sections=64)
               .apply_translation((x, -PL / 2 + MAG_FROM_TIP, PT / 2)) for x in (-MAG_X, MAG_X)]
    return trimesh.boolean.difference([panel(), *pockets], engine="manifold")


def deployed_magnets() -> np.ndarray:
    """Panel-2 magnet centres in the simulation's deployed reference (q1 = 60, q2 = -120), y from the bottom edge."""
    from scipy.spatial.transform import Rotation as R
    p, q = M.initial_poses(np.zeros(3), [0, 0, 0, 1], math.radians(60), math.radians(-120))["panel2"]
    local = np.array([0.0, -PL / 2 + MAG_FROM_TIP, PT / 2]) / MM
    return (p + R.from_quat(q).as_matrix() @ local) * MM - [0, BOTTOM, 0]


def deployed_clearance() -> tuple[float, float]:
    """In the deployed reference: gap from panel 2's inner face to the steel L's outer corner, and magnet-to-corner distance."""
    from scipy.spatial.transform import Rotation as R
    p, q = M.initial_poses(np.zeros(3), [0, 0, 0, 1], math.radians(60), math.radians(-120))["panel2"]
    r = R.from_quat(q).as_matrix()
    face = (p + r @ [0, 0, PT / 2 / MM]) * MM - [0, BOTTOM, 0]
    corner = np.array([0.0, FOOT_STRIP[0] - SHIM, BACK - SPACER_T - SHIM])
    return float((corner - face) @ r[:, 2]), float(np.linalg.norm(corner - deployed_magnets()))


def drawing(path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    from scipy.spatial.transform import Rotation as R

    y0, y1, y2 = HINGE_STRIP
    fig, axes = plt.subplots(1, 3, figsize=(17, 8), gridspec_kw={"width_ratios": [1, 1.1, 1.3]})
    a = axes[0]
    w, h, _ = IPAD
    a.add_patch(Rectangle((-w / 2, 0), w, h, fill=False, lw=1.5))
    a.add_patch(Rectangle((-PW / 2, HINGE1_UP - PL), PW, PL, color="0.85"))
    a.add_patch(Rectangle((-STRIP_W / 2, y0), STRIP_W, y2 - y0, color="tab:orange", alpha=0.35))
    a.add_patch(Rectangle((-STRIP_W / 2, y1), STRIP_W, y2 - y1, color="tab:orange", alpha=0.6))
    a.add_patch(Rectangle((-STRIP_W / 2, FOOT_STRIP[0]), STRIP_W, FOOT_STRIP[1] - FOOT_STRIP[0], color="tab:orange", alpha=0.35))
    a.add_patch(Rectangle((-60, FOOT_STRIP[0] - SHIM), 120, 5, color="tab:blue", alpha=0.7))
    a.plot([-PW / 2, PW / 2], [HINGE1_UP] * 2, "r-", lw=2)
    a.text(0, y2 + 3, f"hinge strip: {SPACER_T:.1f} mm y {y0:.0f}-{y1:.0f}, {LAND_T:.1f} mm y {y1:.0f}-{y2:.0f}; "
           f"hinge 1 at y {HINGE1_UP:.0f}", ha="center", fontsize=7.5, color="r")
    for x in (-MAG_X, MAG_X):
        a.plot(x, HINGE1_UP - MAG_FROM_TIP, "ko", ms=7)
    a.text(0, HINGE1_UP - 16, f"panel 2 magnets (folded), x = +-{MAG_X:.0f}, {MAG_FROM_TIP:.0f} mm from its free end",
           ha="center", fontsize=7.5)
    a.text(0, 18, f"foot strip {SPACER_T:.1f} mm, y {FOOT_STRIP[0]:.1f}-{FOOT_STRIP[1]:.1f}; blue = steel L, 120 mm", ha="center",
           fontsize=7.5, color="tab:blue")
    a.set_title(f"iPad back (mm, y above the bottom edge); grey = folded panels {PW:.0f} x {PL:.0f} x {PT:.1f}",
                fontsize=9)
    a.set_xlim(-100, 100); a.set_ylim(-10, 260); a.set_aspect("equal"); a.grid(alpha=0.3)

    for a in axes[1:]:
        a.add_patch(Rectangle((BACK, 0), IPAD[2], IPAD[1], fill=False, lw=1.5))
        a.add_patch(Rectangle((BACK - SPACER_T, y0), SPACER_T, y2 - y0, color="tab:orange", alpha=0.6))
        a.add_patch(Rectangle((BACK - LAND_T, y1), LAND_T, y2 - y1, color="tab:orange", alpha=0.6))
        a.add_patch(Rectangle((BACK - SPACER_T, FOOT_STRIP[0]), SPACER_T, FOOT_STRIP[1] - FOOT_STRIP[0], color="tab:orange", alpha=0.6))
        f0 = FOOT_STRIP[0] - SHIM
        a.plot([BACK - SPACER_T - SHIM, BACK - SPACER_T - SHIM, BACK], [f0 + 5, f0, f0], "-", color="tab:blue", lw=2.5)
        a.plot(BACK - LAND_T, HINGE1_UP + 0.5, "r^", ms=8)
        a.set_aspect("equal"); a.grid(alpha=0.3)
    z0 = BACK - SPACER_T
    for k, n in enumerate(("panel 2", "panel 1")):
        axes[1].add_patch(Rectangle((z0 - (k + 1) * PT, HINGE1_UP - PL), PT, PL, color="0.4"))
        axes[1].text(z0 - (k + 1) * PT - 6, 60 + 30 * k, n, fontsize=8, rotation=90)
    axes[1].plot([z0 - PT] * 2, [HINGE1_UP - MAG_FROM_TIP] * 2, "ko", ms=5)
    axes[1].set_xlim(-24, 8)
    axes[1].set_title("Folded (side view): panel 2 on the strips,\npanel 1 outside; red = hinge 1 (tape)", fontsize=9)
    poses = M.initial_poses(np.zeros(3), [0, 0, 0, 1], math.radians(60), math.radians(-120))
    for n in ("panel1", "panel2"):
        p, q = poses[n]
        r = R.from_quat(q).as_matrix()
        corners = np.array([[0, sy, sz] for sy in (-.5, .5) for sz in (-.5, .5)]) * M.PANEL_SIZE * MM
        pts = (p * MM + corners @ r.T)[[0, 1, 3, 2, 0]]
        axes[2].plot(pts[:, 2], pts[:, 1] - BOTTOM, "-", color="0.3")
        axes[2].text(p[2] * MM, p[1] * MM - BOTTOM + 8, n.replace("panel", "panel "), fontsize=8)
    m = deployed_magnets()
    axes[2].plot(m[2], m[1], "ko", ms=5)
    axes[2].set_title("Deployed: simulation reference q1 = 60, q2 = -120 (real hinge 1 axis is\n"
                      f"{LAND_T - SPACER_T:.1f} mm further out); black = panel 2 magnets, blue = steel L", fontsize=9)
    fig.tight_layout(); fig.savefig(path, dpi=140)


def main() -> None:
    out = {}
    for name, mesh in (("hinge_strip", hinge_strip()), ("foot_strip", foot_strip()), ("panel1", panel()),
                       ("panel2", panel2())):
        mesh.export(HERE / f"{name}.stl")
        assert trimesh.load(HERE / f"{name}.stl").is_watertight, name
        out[name] = dict(extents_mm=mesh.extents.round(2).tolist(), volume_cm3=round(mesh.volume / 1e3, 2))
    drawing(HERE / "assembly.png")
    out["deployed_magnet_centre_mm"] = deployed_magnets().round(2).tolist()
    out["deployed_gap_face_to_L_corner_mm, magnet_to_corner_mm"] = np.round(deployed_clearance(), 2).tolist()
    for k, v in out.items():
        print(k, v)


if __name__ == "__main__":
    main()
