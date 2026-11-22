"""Unit checks of the dryer-lint scorer and geometry (numpy only): python tests/test_scoring.py"""

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


D = L.dryer_origin(0)
SP, SR = L.screen_seated_pose(D)
MP_S, MR_S = L.mat_pose_in_screen()
MP, MR = SP + SR @ MP_S, SR @ MR_S
BIN = L.bin_center(D)
IN_BIN = BIN + np.array([0.0, 0.0, 0.004 + L.MAT_T + 0.0005])  # lying flat on the bin floor
MR_FLAT = L.ry(-90.0)
ON_TOP = D + np.array([0.05, 0.25, 0.002])


def run(sp, sr, mp, broken, start_at=None, steps=40, lint=True, sp_after=None, mp_after=None):
    sc = S.Scorer()
    lvl = 0.0
    mr = MR_FLAT if mp is IN_BIN else MR
    for k in range(steps):
        p = sp if (sp_after is None or start_at is None or k < start_at) else sp_after
        m_ = mp if (mp_after is None or start_at is None or k < start_at) else mp_after
        m = S.measure(p, sr, m_, mr, D, BIN, lint, broken)
        lvl = sc.step(m, start_at is not None and k >= start_at)
    return lvl, sc


# geometry
check("screen thin end +y", abs(L.thickness(L.SCREEN_L / 2) - L.T_THIN) < 1e-9)
check("screen thick end -y", abs(L.thickness(-L.SCREEN_L / 2) - L.T_THICK) < 1e-9)
check("slot narrower than the thick end at the thin end (one way only)",
      L.T_THIN + L.SLOT_BACK_GAP + L.SLOT_FRONT_GAP < L.T_THICK)
check("grip rests on both lips at the thick end",
      L.GRIP_T / 2 - (L.T_THIN + L.T_THICK) / 4 > 0 and
      -(L.T_THIN + L.T_THICK) / 4 - L.GRIP_T / 2 < -L.T_THICK - L.SLOT_FRONT_GAP)
check("mat fits the front gap", L.MAT_T < L.SLOT_FRONT_GAP)
check("housing floor below the seated screen", L.HOUSING_DEPTH > L.SCREEN_H - L.GRIP_H)
check("official seed has lint and a large enough x offset",
      L.setup_spec(("seed", 1415158438))["lint"] and abs(L.setup_spec(("seed", 1415158438))["dx"]) >= 0.006)
check("seeded x offsets never below 6 mm", min(abs(L.setup_spec(("seed", s))["dx"]) for s in range(500)) >= 0.006)

# measures
m = S.measure(SP, SR, MP, MR, D, BIN, True, False)
check("seated at reset", m.seated and max(abs(g) for g in m.end_gap_mm) < 1e-6)
check("bonded mat counts as on screen", m.mat_on_screen and not m.clean)
m = S.measure(SP + [0, 0, 0.004], SR, MP, MR, D, BIN, True, False)
check("4 mm proud is not seated", not m.seated)
m = S.measure(SP, SR, MP, MR, D, BIN, True, True)
check("broken joint but mat still lying on the face = on screen", m.mat_on_screen)
rev = np.diag([-1.0, -1.0, 1.0])
m = S.measure(SP, rev, MP, MR, D, BIN, True, True)
check("reversed screen is not seated", not m.seated)
m = S.measure(SP, SR, D + np.array([0.15, 0.0, -0.08]), MR, D, BIN, True, True)
check("mat in the housing = in dryer", m.mat_in_dryer and not m.clean)
m = S.measure(SP, SR, IN_BIN, MR_FLAT, D, BIN, True, True)
check("mat lying flat in the bin", m.mat_in_bin and m.clean)

# levels
lvl, _ = run(SP, SR, IN_BIN, True, start_at=5)
check("clean, seated, lint in bin, started -> 1.0", lvl == 1.0)
lvl, _ = run(SP, SR, ON_TOP, True, start_at=5)
check("lint on the dryer top (off the screen), started -> 0.5", lvl == 0.5)
lvl, _ = run(SP, SR, MP, False, start_at=5)
check("started with lint on the screen -> 0", lvl == 0.0)
lvl, _ = run(SP + [0, 0, 0.006], SR, IN_BIN, True, start_at=5)
check("started with the screen 6 mm proud -> 0", lvl == 0.0)
lvl, _ = run(SP + [0, 0, 0.30], SR, IN_BIN, True, start_at=5)
check("started with the screen out -> 0", lvl == 0.0)
sc = S.Scorer()
for k in range(60):
    p = SP + [0, 0, 0.30] if k < 10 else SP
    lvl = sc.step(S.measure(p, SR, IN_BIN, MR_FLAT, D, BIN, True, True), k >= 5)
check("start latched: seating it after starting does not help", lvl == 0.0)
lvl, _ = run(SP, SR, IN_BIN, True, start_at=None)
check("clean and seated, not started -> 0.25", lvl == 0.25)
lvl, _ = run(SP, SR, MP, False, start_at=None)
check("do nothing -> 0", lvl == 0.0)
lvl, _ = run(SP, SR, IN_BIN, True, start_at=30)
check("1.0 needs a 1 s hold (started late) -> 0.5 before the hold completes", lvl == 0.5)

n_fail = sum(1 for _, ok in results if not ok)
print(f"{len(results) - n_fail}/{len(results)} passed")
sys.exit(1 if n_fail else 0)
