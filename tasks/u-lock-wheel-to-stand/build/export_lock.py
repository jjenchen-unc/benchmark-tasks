"""Export the simulated U-lock's shackle and crossbar as STL meshes and a dimensioned drawing.

    pip install trimesh manifold3d matplotlib numpy
    python build/export_lock.py            # from the task folder; writes into build/

The shapes are read from task.py (SHACKLE and CROSSBAR), so the files match the simulation exactly. Isaac Sim is not
needed: the simulator imports are replaced by stand-ins that only record the shapes.
"""

from __future__ import annotations

import math
import pathlib
import sys
import types

import numpy as np
import trimesh

HERE = pathlib.Path(__file__).resolve().parent
TASK = HERE.parent


def _stub_simulator() -> None:
    tf = types.ModuleType("yam_arena.task_file")
    tf.TABLE_TOP = {"station": 0.0}
    ya = types.ModuleType("yam_arena")
    ya.task_file = tf
    lk = types.ModuleType("lockkit")
    lk.Hinge = lambda name, anchor, **kw: types.SimpleNamespace(name=name, anchor=anchor, **kw)
    lk.part = lambda name, pos, **kw: types.SimpleNamespace(name=name, pos=pos, **kw)
    noop = lambda *a, **k: None  # noqa: E731
    lk.env = lk.finger_points = lk.set_weld = lk.weld = noop
    lk.on_reset = lk.on_step = lambda f: f
    sys.modules.update({"yam_arena": ya, "yam_arena.task_file": tf, "lockkit": lk})


def _capsule(a, b, r):
    import manifold3d as mf
    ends = [mf.Manifold.sphere(r, 48).translate(tuple(map(float, p))) for p in (a, b)]
    return mf.Manifold.batch_hull(ends)


def _cylinder(c, r, h, axis) -> trimesh.Trimesh:
    cyl = trimesh.creation.cylinder(radius=r, height=h, sections=64)
    v = {"X": [1, 0, 0], "Y": [0, 1, 0], "Z": [0, 0, 1]}[axis]
    cyl.apply_transform(trimesh.geometry.align_vectors([0, 0, 1], v))
    return cyl.apply_translation(c)


def shackle_mesh(T) -> trimesh.Trimesh:
    import manifold3d as mf
    k = 1000.0
    m = mf.Manifold.batch_boolean([_capsule(np.multiply(a, k), np.multiply(b, k), r * k)
                                   for a, b, r, _ in T.SHACKLE.capsules], mf.OpType.Add).to_mesh()
    return trimesh.Trimesh(m.vert_properties[:, :3], m.tri_verts)


def crossbar_mesh(T) -> trimesh.Trimesh:
    parts = [trimesh.creation.box(extents=s).apply_translation(c) for c, s, _ in T.CROSSBAR.boxes]
    parts += [_cylinder(c, r, h, ax) for c, r, h, _, ax in T.CROSSBAR.cylinders]
    return trimesh.boolean.union(parts, engine="manifold")


SLOT = (0.0052, 0.0304, 0.1505)        # z, x, y extents of each steel-bar pocket: 5 x 30 x 150 mm flat bar + clearance


def crossbar_print_mesh(T) -> trimesh.Trimesh:
    """The same outer shape with two closed pockets in the top and bottom walls for steel flat bars (mass)."""
    pockets = [trimesh.creation.box(extents=(SLOT[1], SLOT[2], SLOT[0])).apply_translation((0.0, 0.0, z))
               for z in (0.015, -0.015)]
    return trimesh.boolean.difference([crossbar_mesh(T), *pockets], engine="manifold")


def drawing(T, path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    mm = 1000.0
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(14, 7.5), gridspec_kw={"width_ratios": [2.2, 1]})
    for a, b, r, _ in T.SHACKLE.capsules:
        a1.plot([a[0] * mm, b[0] * mm], [a[1] * mm, b[1] * mm], lw=2 * r * mm * 0.9, color="0.6", solid_capstyle="round")
    a1.plot([T.HOOK_BEND * mm, T.HOOK_END[0] * mm], [-T.S * mm, T.HOOK_END[1] * mm], "k-", lw=0.8)
    a1.plot([0], [0], "k+", ms=10)
    for x, y, t in [(0, 0, "bend centre (0, 0)"), (T.TIP + T.LEG_R, T.S, f"straight end x={(T.TIP + T.LEG_R) * mm:.0f}"),
                    (T.HOOK_BEND, -T.S, f"foot bend ({T.HOOK_BEND * mm:.0f}, {-T.S * mm:.2f})"),
                    (T.HOOK_END[0], T.HOOK_END[1], f"toe centre ({T.HOOK_END[0] * mm:.0f}, {T.HOOK_END[1] * mm:.2f})")]:
        a1.annotate(t, (x * mm, y * mm), xytext=(8, 8), textcoords="offset points", fontsize=8)
    a1.set_title(f"Shackle, top view (mm). Rod {2 * T.LEG_R * mm:.1f} mm; leg axes y = +-{T.S * mm:.2f};\n"
                 f"bend centreline radius {T.S * mm:.2f}; {2 * T.COAT_R * mm:.0f} mm sleeve over bend and first "
                 f"{T.COAT_END * mm:.0f} mm of each leg", fontsize=9)
    a1.set_aspect("equal"); a1.grid(alpha=0.3)
    # Crossbar section in its mid-plane z = 0: the boxes that cross it.
    for (cx, cy, cz), (sx, sy, sz), _ in T.CROSSBAR.boxes:
        if abs(cz) - sz / 2 < 1e-9:
            a2.add_patch(Rectangle(((cx - sx / 2) * mm, (cy - sy / 2) * mm), sx * mm, sy * mm, color="0.25"))
    (hc, hr, hh, _, _), _ = T.CROSSBAR.cylinders
    a2.add_patch(Rectangle(((hc[0] - hr) * mm, (hc[1] - hh / 2) * mm), 2 * hr * mm, hh * mm, color="0.45"))
    a2.set_xlim(-30, 30); a2.set_ylim(-150, 100); a2.set_aspect("equal"); a2.grid(alpha=0.3)
    a2.set_title(f"Crossbar, section at mid-height (mm). {2 * T.CB_HALF * mm:.0f} x {2 * T.CB_HALF * mm:.0f} mm "
                 f"bar,\nmouths {2 * T.HOLE * mm:.0f} x {2 * T.HOLE * mm:.0f} mm open toward -x; straight socket bottom "
                 f"x={T.BOTTOM * mm:.0f};\nbent-foot lip to x={T.LIP_BACK * mm:.0f}, chamber to x=12, y=-84;\n"
                 f"seated: bar origin at shackle x={T.SEAT_X * mm:.0f};\nlock head {2 * hr * mm:.0f} mm dia., "
                 f"{hh * mm:.0f} mm long.", fontsize=9)
    fig.tight_layout(); fig.savefig(path, dpi=150)


def main() -> None:
    _stub_simulator()
    sys.path.insert(0, str(TASK))
    import task as T

    out = {}
    for name, mesh in (("shackle", shackle_mesh(T)), ("crossbar", crossbar_mesh(T)),
                       ("crossbar_print", crossbar_print_mesh(T))):
        if name != "shackle":     # STL in millimetres; the shackle is built in millimetres already
            mesh.apply_scale(1000.0)
        assert mesh.is_watertight, name
        mesh.export(HERE / f"{name}.stl")
        assert trimesh.load(HERE / f"{name}.stl").is_watertight, name
        lo, hi = mesh.bounds
        out[name] = dict(bounds_mm=[lo.round(2).tolist(), hi.round(2).tolist()], volume_cm3=round(mesh.volume / 1e3, 1))
    drawing(T, HERE / "dimensions.png")
    for k, v in out.items():
        print(k, v)


if __name__ == "__main__":
    main()
