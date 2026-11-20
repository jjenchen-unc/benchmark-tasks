"""Geometry (numpy only): fixture colliders in the magazine frame, magazine lower-tube block, mouth height map.
Magazine frame: x forward along the floor plate, y across, z up, origin under the mouth at CAD x 78, z = CAD y.
Fixture frame (build/make_fixture.py, mm): X = CAD x + 5.25, Y = CAD z - 22.31 (= -mag y), Z = CAD y - 1.08 + 6.
"""
import json
import math
import os
from pathlib import Path

import numpy as np

D = Path(__file__).resolve().parent / "collider"
META = json.loads((D / "meta.json").read_text())
T21 = math.tan(math.radians(21.0))
C, TB = 0.3, 6.0
H_TOWER = float(os.environ.get("FIX_H", "50"))
DX, DZ = 5.25, TB - 1.08
PX0, PX1 = 19.75 + DX - C, 60.15 + DX + C
PW, PT = 14.27 + C, 6.2 + 0.4
BW, BL = 11.25 + C, 36.67 + 2 * C
Y0, Y1 = -PW - 10, 22.0
ZT = TB + H_TOWER
PIN_Y, PIN_Z, PIN_R = BW + C + 2.6, TB + 30.0, 2.5
MAG_Z0 = DZ / 1000  # magazine origin above the fixture underside
GREY = (0.55, 0.57, 0.6)


def rear(Z):
    return 21.73 + DX - C + (Z - DZ - 8.8) * T21


def f2m(p):  # fixture mm -> magazine frame m
    p = np.asarray(p, float)
    return np.c_[p[..., 0] - 78.0 - DX, -p[..., 1], p[..., 2] - DZ] / 1000 if p.ndim > 1 else np.array([p[0] - 78.0 - DX, -p[1], p[2] - DZ]) / 1000


def _box(x0, x1, y0, y1, z0, z1):
    lo, hi = f2m((x0, y1, z0)), f2m((x1, y0, z1))
    return (tuple((lo + hi) / 2), tuple(hi - lo), GREY)


def _prism(xz, y0, y1):
    return (tuple(map(tuple, np.vstack([f2m([(x, y, z) for x, z in xz]) for y in (y0, y1)]))), GREY)


ZG = TB + PT
FIX_BOXES = (_box(0, 120, -40, 40, 0, TB),  # base plate
             _box(15, 90, Y0, -PW, TB, ZT),  # closed side wall
             _box(15, 90, -PW, -BW, ZG, ZT),  # wall above the floor-plate groove
             _box(15, PX0, -PW, Y1, TB, ZG),  # rear foot
             _box(PX1, 90, -PW, Y1, TB, ZG))  # front foot
FIX_HULLS = (_prism([(15, ZG), (rear(ZG), ZG), (rear(ZT), ZT), (15, ZT)], -BW, Y1),  # rear block, raked face
             _prism([(rear(ZG) + BL, ZG), (90, ZG), (90, ZT), (rear(ZT) + BL, ZT)], -BW, Y1))  # front block
_pc = f2m(((15 + 90) / 2, PIN_Y, PIN_Z))
FIX_PIN = (tuple(_pc), PIN_R / 1000, 0.075, (0.8, 0.8, 0.82), "X")  # keeper pin (static, part of the fixture)

# magazine lower tube (CAD y 7.28 to 50): solid raked block with the tube's outer section; the mouth collider starts at y 50
_xr = lambda z: (21.73 - 78.0) + (z - 8.8) * T21  # noqa: E731
LOW_Z = (7.3, 50.0)
MAG_LOW = (tuple((x / 1000, y / 1000, z / 1000) for z in LOW_Z for x in (_xr(z), _xr(z) + 36.67) for y in (-11.25, 11.25)),
           (0.16, 0.16, 0.18))


def body_hulls():
    d = np.load(D / "body_hulls.npz")
    out, k = [], 0
    for n in d["n"]:
        out.append(d["verts"][k:k + n].astype(float))
        k += n
    return out


def height_map(xs, ys, zmin=0.100):
    """max z of the magazine body collider on a grid (m); nan where nothing above zmin"""
    from scipy.spatial import ConvexHull
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    Hm = np.full(X.shape, np.nan)
    for v in body_hulls():
        if v[:, 2].max() < zmin:
            continue
        eq = ConvexHull(v).equations  # a.p + b <= 0
        i0, i1 = np.searchsorted(xs, [v[:, 0].min(), v[:, 0].max()])
        j0, j1 = np.searchsorted(ys, [v[:, 1].min(), v[:, 1].max()])
        if i1 <= i0 or j1 <= j0:
            continue
        x, y = X[i0:i1, j0:j1], Y[i0:i1, j0:j1]
        hi, lo = np.full(x.shape, np.inf), np.full(x.shape, -np.inf)
        for a0, a1, a2, b in eq:
            r = -(a0 * x + a1 * y + b)
            if abs(a2) < 1e-9:
                hi = np.where(r < 0, -np.inf, hi)
            elif a2 > 0:
                hi = np.minimum(hi, r / a2)
            else:
                lo = np.maximum(lo, r / a2)
        ok = hi >= lo
        Hm[i0:i1, j0:j1] = np.where(ok, np.fmax(Hm[i0:i1, j0:j1], hi), Hm[i0:i1, j0:j1])
    return Hm


if __name__ == "__main__":
    xs, ys = np.arange(-18, 20.01, 0.25) / 1000, np.arange(-12, 12.01, 0.25) / 1000
    Hm = height_map(xs, ys) * 1000
    np.save(Path(__file__).resolve().parent / "heightmap.npy", Hm)
    print("mag top height (mm) by x (rows) and |y| band (cols: 0-2, 2-4.4, 4.4-6, 6-8.5, 8.5-11)")
    bands = [(0, 2), (2, 4.4), (4.4, 6), (6, 8.5), (8.5, 11)]
    for x in np.arange(-16, 19.1, 1.0):
        i = np.argmin(abs(xs * 1000 - x))
        row = []
        for a, b in bands:
            m = (abs(ys * 1000) >= a) & (abs(ys * 1000) <= b)
            row.append(np.nanmax(Hm[max(0, i - 2):i + 3][:, m]) if np.isfinite(Hm[max(0, i - 2):i + 3][:, m]).any() else float("nan"))
        print(f"x {x:+5.1f}  " + "  ".join(f"{v:7.2f}" for v in row))
    b = body_hulls()
    V = np.vstack(b) * 1000
    lo = V[V[:, 2] < 53]
    print("body collider: z min", V[:, 2].min().round(2), "x range at z<53", lo[:, 0].min().round(2), lo[:, 0].max().round(2),
          "y", lo[:, 1].min().round(2), lo[:, 1].max().round(2), "block at z50 x", round(_xr(50), 2), round(_xr(50) + 36.67, 2))
    print("fixture boxes (mag frame mm):")
    for c, s, _ in FIX_BOXES:
        print("  centre", np.round(np.array(c) * 1000, 2), "size", np.round(np.array(s) * 1000, 2))
    for v, _ in FIX_HULLS:
        print("  hull", np.round(np.array(v).min(0) * 1000, 2), np.round(np.array(v).max(0) * 1000, 2))
    print("  pin", np.round(np.array(FIX_PIN[0]) * 1000, 2), "lip top", META["z_lip_top"], "mouth above table", META["z_lip_top"] + DZ)
