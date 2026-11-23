"""Unit tests of the t8-tube scorer (pure Python): python -m pytest tests/ (or python tests/test_scoring.py)."""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "src"))

from scoring import CLEAR_S, Scorer  # noqa: E402

DT = 0.05
NO = dict(lamp=False, carton=False, fixture=False)
HELD = dict(lamp=True, carton=False, fixture=False)


def run(seq):
    s = Scorer()
    lv = 0.0
    for holder, in_carton, speed, contact, dropped, broken in seq:
        lv = s.step(DT, holder, in_carton, speed, contact, dropped=dropped, broken=broken)
    return lv, s


def steps(n, *row):
    return [row] * n


def test_noop_scores_zero():
    lv, _ = run(steps(400, "locked", False, 0.0, NO, False, False))
    assert lv == 0.0


def test_turned_but_not_down_scores_zero():
    lv, _ = run(steps(100, "slot", False, 0.0, HELD, False, False))
    assert lv == 0.0


def test_full_success():
    seq = steps(50, "locked", False, 0.0, HELD, False, False) + steps(50, "out", False, 0.1, HELD, False, False)
    seq += steps(20, "out", True, 0.05, HELD, False, False)
    seq += steps(int(CLEAR_S / DT) + 1, "out", True, 0.0, NO, False, False)
    lv, _ = run(seq)
    assert lv == 1.0


def test_success_needs_hands_off_and_rest():
    seq = steps(50, "out", True, 0.0, HELD, False, False)
    assert run(seq)[0] == 0.0
    seq = steps(int(CLEAR_S / DT) + 5, "out", True, 0.1, NO, False, False)
    assert run(seq)[0] == 0.0


def test_out_not_in_carton_is_zero():
    seq = steps(60, "out", False, 0.0, HELD, False, False)
    assert run(seq)[0] == 0.0
    seq = steps(int(CLEAR_S / DT) + 5, "out", False, 0.0, NO, False, False)
    assert run(seq)[0] == 0.0


def test_drop_vetoes_even_in_carton():
    seq = steps(20, "out", False, 0.0, HELD, False, False) + steps(1, "out", False, 1.2, NO, True, False)
    seq += steps(int(CLEAR_S / DT) + 5, "out", True, 0.0, NO, False, False)
    lv, s = run(seq)
    assert lv == 0.0 and s.veto == "lamp_dropped"


def test_broken_lampholder_vetoes():
    seq = steps(20, "locked", False, 0.0, HELD, False, False) + steps(1, "broken", False, 0.0, HELD, False, True)
    seq += steps(int(CLEAR_S / DT) + 5, "out", True, 0.0, NO, False, False)
    lv, s = run(seq)
    assert lv == 0.0 and s.veto == "lampholder_broken"


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
