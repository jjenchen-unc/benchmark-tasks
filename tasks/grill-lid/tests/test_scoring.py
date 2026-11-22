"""Unit checks of the grill-lid scorer, burner model and layout (numpy only): python tests/test_scoring.py"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
import layout as L  # noqa: E402
import scoring as S  # noqa: E402

DT = 0.05
results = []


def check(name, cond):
    results.append((name, bool(cond)))
    print(("PASS" if cond else "FAIL"), name)


def run(seq, sc=None):
    """seq: list of (seconds, lid_deg, knob_deg, igniter_mm, contact)."""
    sc = sc or S.Scorer()
    lvl = 0.0
    for secs, lid, knob, ign, contact in seq:
        for _ in range(int(round(secs / DT))):
            lvl = sc.step(DT, lid, knob, ign, contact)
    return lvl, sc


OPEN = 95.0
# reference sequence: lid opened, knob to HI, igniter pressed 1 s after gas, hands off
good = [(1.0, 0.0, 0.0, 0.0, False), (3.0, OPEN, 0.0, 0.0, True), (1.0, OPEN, 90.0, 0.0, True),
        (0.5, OPEN, 90.0, 3.0, True), (0.5, OPEN, 90.0, 0.0, True), (1.2, OPEN, 90.0, 0.0, False)]
lvl, sc = run(good)
check("open lid, gas, spark within 5 s, hands off 1 s -> 1.0", lvl == 1.0 and sc.burner.lit)
check("  lid angle recorded at gas-on", sc.last["lid_at_gas"] == OPEN)
lvl, sc = run(good[:-1] + [(1.2, OPEN, 90.0, 0.0, True)])
check("lit but a hand still on the grill -> 0.5", lvl == 0.5)
lvl, sc = run([(2.0, 0.0, 0.0, 0.0, False)])
check("nothing done -> 0", lvl == 0.0)
lvl, sc = run([(3.0, OPEN, 0.0, 0.0, False), (2.0, OPEN, 0.0, 0.0, False)])
check("lid opened only -> 0", lvl == 0.0)
lvl, sc = run([(1.0, 0.0, 90.0, 0.0, True), (0.5, 0.0, 90.0, 3.0, True), (2.0, 0.0, 90.0, 0.0, False)])
check("lit with the lid closed -> 0 (veto gas_lid_closed)", lvl == 0.0 and sc.veto == "gas_lid_closed")
check("  burner model still lights (the flash the veto stands for)", sc.burner.lit)
lvl, sc = run([(1.0, 50.0, 90.0, 0.0, True), (0.5, 50.0, 90.0, 3.0, True), (2.0, 95.0, 90.0, 0.0, False)])
check("gas on with the lid only half open (50 deg) -> 0", lvl == 0.0 and sc.veto == "gas_lid_closed")
lvl, sc = run([(0.1, 0.0, 0.0, 0.0, True), (0.1, 0.0, 0.0, 3.0, True), (3.0, OPEN, 0.0, 0.0, True),
               (1.0, OPEN, 90.0, 0.0, True), (0.5, OPEN, 90.0, 3.0, True), (1.5, OPEN, 90.0, 0.0, False)])
check("igniter clicked with the lid closed but gas off (harmless) -> still 1.0", lvl == 1.0)
lvl, sc = run([(3.0, OPEN, 0.0, 0.0, True), (5.2, OPEN, 90.0, 0.0, True), (0.5, OPEN, 90.0, 3.0, True),
               (2.0, OPEN, 90.0, 0.0, False)])
check("spark 5.2 s after gas-on -> 0 (veto unlit_over_5s)", lvl == 0.0 and sc.veto == "unlit_over_5s")
lvl, sc = run([(3.0, OPEN, 0.0, 0.0, True), (4.6, OPEN, 90.0, 0.0, True), (0.3, OPEN, 90.0, 3.0, True),
               (2.0, OPEN, 90.0, 0.0, False)])
check("spark 4.6 s after gas-on, lit at 4.85 s -> 1.0", lvl == 1.0)
lvl, sc = run([(3.0, OPEN, 0.0, 0.0, True), (3.0, OPEN, 90.0, 0.0, True), (2.0, OPEN, 0.0, 0.0, False)])
check("gas on, no spark, turned off within 5 s -> 0.5 (safe abort)", lvl == 0.5 and sc.aborted == 1)
lvl, sc = run([(3.0, OPEN, 0.0, 0.0, True), (3.0, OPEN, 90.0, 0.0, True), (1.0, OPEN, 0.0, 0.0, True),
               (1.0, OPEN, 90.0, 0.0, True), (0.5, OPEN, 90.0, 3.0, True), (2.0, OPEN, 90.0, 0.0, False)])
check("aborted, then gas on again at once and lit -> 0 (retry_without_wait)", lvl == 0.0
      and sc.veto == "retry_without_wait")
lvl, sc = run([(3.0, OPEN, 0.0, 0.0, True), (1.0, OPEN, 10.0, 0.0, True), (0.5, OPEN, 10.0, 3.0, True),
               (2.0, OPEN, 10.0, 0.0, False)])
check("knob only 10 deg off OFF (valve shut) + spark -> not lit, 0", lvl == 0.0 and not sc.burner.lit
      and sc.attempts == 0)
b = S.Burner()
for _ in range(4):
    b.step(DT, 90.0, 3.0)  # 0.2 s of spark but gas only 0.2 s old
check("burner: needs 0.5 s of gas before it lights", not b.lit)
for _ in range(6):
    b.step(DT, 90.0, 3.0)
check("burner: lights once gas >= 0.5 s and spark >= 0.2 s", b.lit)
b.step(DT, 0.0, 0.0)
check("burner: goes out at OFF", not b.lit)
lvl, sc = run(good + [(1.0, 0.0, 90.0, 0.0, False)])
check("lid closed after lighting (normal preheat) -> still 1.0", lvl == 1.0)
lvl, sc = run(good + [(1.0, OPEN, 0.0, 0.0, False)])
check("lit, then turned off at the end -> 0", lvl == 0.0)

sc = S.Scorer()
for _ in range(10):
    sc.step(DT, OPEN, 30.0, 0.0, True, knob_axial_n=150.0)
check("knob pushed with 150 N for 0.5 s -> veto forced_knob", sc.veto == "forced_knob")
sc = S.Scorer()
sc.step(DT, OPEN, 30.0, 0.0, True, knob_axial_n=150.0)
sc.step(DT, OPEN, 30.0, 0.0, True, knob_axial_n=10.0)
check("one 50 ms spike of 150 N -> no veto", sc.veto is None)
sc = S.Scorer()
sc.step(DT, 0.0, 0.0, 0.0, True, shift_mm=4.0)
check("grill shoved 4 mm -> veto grill_shoved", sc.veto == "grill_shoved")
sc = S.Scorer()
sc.step(DT, 0.0, 0.0, 0.0, True, shift_mm=1.0, yaw_shift_deg=0.5)
check("grill moved 1 mm / 0.5 deg -> no veto", sc.veto is None)

# layout
check("balance angle between 60 and 80 deg (lid rests open on its 95 deg stop)", 60 < L.balance_deg() < 80)
check("closed lid needs 7-12 N up at the handle", 7.0 < L.handle_force_closed() < 12.0)
check("push spring: unlock depth needs < 12 N", L.PUSH_PRELOAD + L.PUSH_SPRING * L.PUSH_UNLOCK < 12.0)
check("igniter switch point needs > 7 N (no trigger on a light touch)",
      L.IGN_PRELOAD + L.IGN_SPRING * S.SPARK_MM / 1000 > 7.0)
seeds = [L.setup_spec(("seed", s)) for s in range(200)]
check("seeded starts: 10 mm <= |dx| <= 25 mm from nominal", all(0.010 - 1e-9 <= abs(s["x"] - L.NOMINAL[0]) <= 0.025 + 1e-9 for s in seeds))
check("seeded starts: within the box", all(abs(s["y"]) <= 0.010 and abs(s["yaw"]) <= 3.0 for s in seeds))
d = L.setup_spec(("seed", 1415158438))
check("default trial seed start (x %.3f, y %.3f, yaw %.2f)" % (d["x"], d["y"], d["yaw"]), True)

fails = [n for n, ok in results if not ok]
print(f"{len(results) - len(fails)}/{len(results)} passed")
sys.exit(1 if fails else 0)
