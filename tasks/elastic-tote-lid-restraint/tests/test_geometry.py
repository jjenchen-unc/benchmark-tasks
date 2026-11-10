"""Pure-Python checks of the pilot layout against the frozen native constants (no simulator)."""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "d1r_arena"))
import geometry as g  # noqa: E402

t = g.tote("tote_tall")
assert abs(t["top"] - 1.0803) < 1e-3, t["top"]  # native lid top 1.08 m
kc, qref = g.lid_spring(30.0)
assert kc == 750.0 and abs(qref - (30 + 0.5 * 9.81) / 750) < 1e-12
up = lambda q: kc * (qref - q) - g.LID_MASS * g.G  # noqa: E731
assert abs(up(0.0) - 30.0) < 1e-9 and abs(up(0.02) - 15.0) < 1e-9  # task.json: 30 N shut, 15 N at stop
# transform: native mid-base -> Arena (SHIFT_X, 0), heights relative to table preserved
p = g.to_arena((0.02, -0.03, 0.80))
assert abs(p[0] - g.SHIFT_X) < 1e-12 and abs(p[1]) < 1e-12 and abs(p[2] - g.TABLE_TOP) < 1e-12
eyes = [g.eye_world("tote_tall", s) for s in (0, 1)]
assert abs(eyes[0][1] + eyes[1][1]) < 1e-9 and abs(abs(eyes[0][1]) - 0.23) < 1e-9
assert abs(eyes[0][2] - (0.87 - 0.80 + g.TABLE_TOP)) < 1e-9
# everything on the station table (x -0.29..1.01 or -0.19..0.91, |y| <= 0.55 conservative)
for h in g.hooks():
    assert -0.19 < h["pos"][0] < 0.91 and abs(h["pos"][1]) < 0.55, h
for name in ("tote_tall", "tote_low"):
    for s in (0, 1):
        e = g.eye_world(name, s)
        assert -0.19 < e[0] < 0.91 and abs(e[1]) < 0.55, (name, e)
# eight hooks, four distinct rows, relaxed lengths and colours as in task.json
assert len(g.hooks()) == 8 and g.RELAXED_LENGTHS == (0.36, 0.52, 0.58, 0.46)
assert len(g.tray()["holders"]) == 32 and len(g.tray()["cards"]) == 4
print("geometry checks passed")
