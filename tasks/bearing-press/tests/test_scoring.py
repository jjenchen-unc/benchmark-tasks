"""Unit checks of the bearing-press scorer, fit model and geometry (numpy only): python tests/test_scoring.py"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
import layout as L  # noqa: E402
import scoring as S  # noqa: E402

results = []


def check(name, cond):
    results.append((name, bool(cond)))
    print(("PASS" if cond else "FAIL"), name)


def rx(deg):
    a = math.radians(deg)
    return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])


HC = np.array([L.RAM_XY[0], L.RAM_XY[1], L.PLATE_Z])
FAR = [np.array([0.0, 0.4, 1.2])] * 8


def poses(bearing_bottom_depth, tilt=0.0, hxy=(0.0, 0.0), disc_dxy=(0.0, 0.0)):
    h = HC + np.array([hxy[0], hxy[1], 0.0])
    bz = h[2] + L.HOUSING_H - bearing_bottom_depth
    b = np.array([h[0], h[1], bz])
    d = b + np.array([disc_dxy[0], disc_dxy[1], L.BEARING_B])
    eye = np.eye(3)
    return {"housing": (h, eye), "bearing": (b, rx(tilt)), "disc_out": (d, eye), "disc_in": (d + [0, 0.3, 0], eye)}


def run(p, tips=FAR, theta=0.0, steps=25, mon=None, v=0.0):
    sc = S.Scorer()
    mon = mon or S.Monitor()
    lvl = 0.0
    for _ in range(steps):
        lvl = sc.step(S.measure(p, theta, tips), v, 0.0, mon)
    return lvl, sc


# fit model
check("fit force starts at 20 N", abs(L.fit_force(0.0) - 20.0) < 1e-9)
check("fit force 300 N at the shoulder", abs(L.fit_force(L.ENGAGE_LEN) - 300.0) < 1e-9)
check("fit force monotonic", all(L.fit_force(a) <= L.fit_force(a + 1e-4) for a in np.linspace(0, 0.006, 50)))
check("handle force at full fit under 20 N at the grip", L.FIT_FULL_N * L.PINION_R / L.GRIP_R < 20.0)
check("ram travel to seat fits the handle range",
      (L.RAM_GAP0 + L.ENGAGE_LEN) / L.PINION_R < L.HANDLE_THETA_LIM[1] - 0.1)
check("ram travel to seat within ram joint range", L.RAM_GAP0 + L.ENGAGE_LEN < L.RAM_TRAVEL[1])
# geometry
check("bore clearance 0.1..0.2 mm", 0.0001 <= L.BORE_R - L.BRG_OD / 2 <= 0.0002)
check("shoulder supports the outer ring only", L.THROUGH_R > L.INNER_OUT_R and L.THROUGH_R < L.OUTER_IN_R)
check("outer driver rim on the outer ring only", L.DISC_OUT_RELIEF_R >= L.OUTER_IN_R - 0.0006 and L.DISC_OUT_R <= L.BRG_OD / 2)
check("inner disc does not reach the outer ring", L.DISC_IN_R < L.OUTER_IN_R)
check("ram face clears the driver at the start", L.RAM_FACE0 - (L.PLATE_Z + L.HOUSING_H - L.START_SINK + L.BEARING_B + L.DISC_T) > 0.003)
check("housing start never near the ram axis", L.HOUSING_START_DY[0] > 0.05)
for sd in (1415158438, 0, 7, 13, 99):
    s = L.setup_spec(("seed", sd))
    check(f"seed {sd} housing offset in range", L.HOUSING_START_DY[0] <= s["hy"] <= L.HOUSING_START_DY[1])
swaps = {L.setup_spec(("seed", sd))["swap"] for sd in range(20)}
wrongs = [L.setup_spec(("seed", sd))["wrong"] for sd in range(200)]
check("wrong disc is the 32 mm one in 35-65 % of seeds (no size rule works)", 0.35 < wrongs.count("disc_in") / 200 < 0.65)
check("46 mm disc bears on both rings (wider than the inner ring and the outer ring)", L.DISC_BIG_R > L.BRG_OD / 2)
check("disc slots swap across seeds", swaps == {True, False})
# start layout: no overlaps with the press base, parts on their supports
bx, by, _ = L.BASE_BOX
tx = (L.TRAY_C[0] - L.TRAY_SIZE[0] / 2, L.TRAY_C[0] + L.TRAY_SIZE[0] / 2)
ty = (L.TRAY_C[1] - L.TRAY_SIZE[1] / 2, L.TRAY_C[1] + L.TRAY_SIZE[1] / 2)
check("tray clear of the press base", tx[1] < bx[0] - 0.003 or ty[0] > by[1] or ty[1] < by[0])
for sd in [1415158438] + list(range(30)):
    st = L.part_starts(("seed", sd))
    hp = st["housing"][0]
    ok = bx[0] + 0.008 < hp[0] - L.HOUSING_R and hp[1] + L.HOUSING_R < by[1] - 0.005
    for n, r in (("bearing", L.BRG_OD / 2), ("disc_out", L.DISC_OUT_R), (L.setup_spec(("seed", sd))["wrong"],
                 L.DISC_BIG_R if L.setup_spec(("seed", sd))["wrong"] == "disc_big" else L.DISC_IN_R)):
        p = st[n][0]
        ok = ok and tx[0] < p[0] - r and p[0] + r < tx[1] and p[0] + r < bx[0] - 0.003
    if not ok:
        check(f"seed {sd} start layout", False)
check("start layouts on their supports (31 seeds)", True)
# scorer levels
lvl, _ = run(poses(L.START_SINK))
check("bearing just set in the mouth: 0", lvl == 0.0)
lvl, _ = run(poses(0.0025))
check("started 2.5 mm deep, square: 0.25", lvl == 0.25)
lvl, _ = run(poses(0.0025, tilt=0.5))
check("started but cocked 0.5 deg: 0", lvl == 0.0)
lvl, _ = run(poses(L.SEAT_DEPTH))
check("seated, hands clear: 1.0", lvl == 1.0)
lvl, _ = run(poses(L.SEAT_DEPTH - 0.0001))
check("0.1 mm proud of the shoulder: not seated (0.25)", lvl == 0.25)
lvl, _ = run(poses(L.SEAT_DEPTH), steps=10)
check("seated but not yet held 1 s: 0.5", lvl == 0.5)
grip = L.handle_point(0.5, L.GRIP_R)
lvl, _ = run(poses(L.SEAT_DEPTH), tips=[grip] * 8, theta=0.5)
check("seated, hand still on the handle: 0.5", lvl == 0.5)
near = HC + np.array([-L.HOUSING_R - 0.01, 0.0, 0.01])
lvl, _ = run(poses(L.SEAT_DEPTH), tips=[near] * 8)
check("seated, hand at the work: 0.5", lvl == 0.5)
lvl, _ = run(poses(L.SEAT_DEPTH), v=0.01)
check("seated but moving: 0.5", lvl == 0.5)
# monitor
p = poses(0.003, tilt=0.5)
mon = S.Monitor()
for _ in range(30):
    mon.step(1 / 480, dict(ram_load=50.0, inner_load=0.0, bearing=p["bearing"], housing=p["housing"],
                           disc_out=p["disc_out"], depth=0.002))
check("cocked under the ram: jammed", mon.jammed and "cocked" in mon.jam_reason)
p = poses(0.003, hxy=(0.003, 0.0))
mon = S.Monitor()
for _ in range(30):
    mon.step(1 / 480, dict(ram_load=50.0, inner_load=0.0, bearing=p["bearing"], housing=p["housing"],
                           disc_out=p["disc_out"], depth=0.002))
check("3 mm off the ram axis under load: jammed", mon.jammed and "off-centre" in mon.jam_reason)
p = poses(0.003, hxy=(0.003, 0.0))
mon = S.Monitor()
for _ in range(30):
    mon.step(1 / 480, dict(ram_load=5.0, inner_load=0.0, bearing=p["bearing"], housing=p["housing"],
                           disc_out=p["disc_out"], depth=0.002))
check("off-centre but the ram not pushing: no jam", not mon.jammed)
p = poses(0.003, disc_dxy=(0.002, 0.0))
mon = S.Monitor()
for _ in range(30):
    mon.step(1 / 480, dict(ram_load=50.0, inner_load=0.0, bearing=p["bearing"], housing=p["housing"],
                           disc_out=p["disc_out"], depth=0.002))
check("driver 2 mm off the bearing under load: jammed", mon.jammed and "driver" in mon.jam_reason)
p = poses(0.003)
mon = S.Monitor()
for _ in range(30):
    mon.step(1 / 480, dict(ram_load=50.0, inner_load=25.0, bearing=p["bearing"], housing=p["housing"],
                           disc_out=p["disc_out"], depth=0.002))
check("inner ring loaded 25 N: brinelled", mon.brinelled and mon.veto)
mon2 = S.Monitor()
for _ in range(10):
    mon2.step(1 / 480, dict(ram_load=50.0, inner_load=25.0, bearing=p["bearing"], housing=p["housing"],
                            disc_out=p["disc_out"], depth=0.002))
check("inner ring touched for 21 ms only: not brinelled", not mon2.brinelled)
lvl, _ = run(poses(L.SEAT_DEPTH), mon=mon)
check("seated after brinelling: 0", lvl == 0.0)

bad = [n for n, ok in results if not ok]
print(f"{len(results) - len(bad)}/{len(results)} passed")
sys.exit(1 if bad else 0)
