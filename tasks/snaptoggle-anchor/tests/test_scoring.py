"""Unit checks of the snaptoggle-anchor scorer (no simulator): python tests/test_scoring.py"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
import layout as L  # noqa: E402
import scoring as S  # noqa: E402

XF = L.XF0
HOLE = np.array([XF, 0.01, L.HOLE_Z0])
FAR_TIPS = [np.array([0.2, 0.3, 1.0])] * 4


def anchor(phi=0.0, gap=0.0, q=L.Q_SEAT, proud=0.0, snapped=True, tail_far=True, dy=0.0, roll=0.0):
    """Poses of a set anchor; gap = bearing face off the back face (m); proud = extra cap offset (m)."""
    pivot = HOLE + np.array([L.BOARD_T + L.BEAR_X + gap, dy, 0.0])
    s = L.rx(roll) @ L.S_SET  # roll: channel turned about the hole axis (still in the board plane)
    ch = (pivot, s @ L.channel_rot_in_s(phi))
    # cap flange position: on the face when proud = 0 (q measured from the pivot)
    cap_pos = pivot + s @ np.array([q, 0.0, 0.0])
    if proud:
        cap_pos = cap_pos + np.array([-proud, 0.0, 0.0])
    cap = (cap_pos, s)
    tail = (np.array([0.2, -0.2, L.TOP]), s) if tail_far else (pivot + s @ np.array([L.X_BRK, 0, 0]), s)
    return ch, cap, tail


def meas(**kw):
    q = kw.pop("q", L.Q_SEAT)
    snapped = kw.pop("snapped", True)
    tips = kw.pop("tips", FAR_TIPS)
    ch, cap, tail = anchor(q=q, snapped=snapped, **kw)
    return S.measure(XF, HOLE, ch, cap, q, tail, snapped, tips)


def run(m, steps=25, veto=False):
    sc = S.Scorer()
    mon = S.Monitor()
    if veto:
        for _ in range(60):
            mon.step(1 / 480, 50.0)
    lv = 0.0
    for _ in range(steps):
        lv = sc.step(m, 0.0, mon)
    return lv


def check(name, got, want):
    ok = abs(got - want) < 1e-9
    print(f"{'ok  ' if ok else 'FAIL'} {name}: {got} (want {want})")
    return ok


def main():
    res = []
    m = meas()
    res.append(check("set, snapped, held 1 s", run(m), 1.0))
    res.append(check("set, snapped, not yet held 1 s", run(m, steps=5), 0.6))
    res.append(check("set, straps on", run(meas(snapped=False, tail_far=False)), 0.6))
    res.append(check("set, snapped but tail left in the hole", run(meas(tail_far=False)), 0.6))
    res.append(check("channel turned 35 deg about the hole axis (still across)", run(meas(roll=35.0)), 1.0))
    res.append(check("channel tilted 10 deg off the board (one end 5 mm off)", run(meas(phi=10.0)), 0.0))
    res.append(check("channel 2 mm off the back face, cap 2 mm proud (loose)", run(meas(gap=0.002, q=L.Q_SEAT,
                                                                                     proud=0.002)), 0.3))
    res.append(check("cap seated on a loose anchor (play 3 mm)", run(meas(gap=0.003, q=L.Q_SEAT + 0.003)), 0.0))
    res.append(check("cap seated, play 2 mm, channel at 2 mm", run(meas(gap=0.002, q=L.Q_SEAT + 0.002)), 0.3))
    res.append(check("channel bearing, cap 10 mm proud", run(meas(q=L.Q_SEAT + 0.010, proud=0.0)), 0.3))
    res.append(check("channel folded in line (phi 90)", run(meas(phi=90.0)), 0.0))
    res.append(check("channel deep in the cavity", run(meas(gap=0.04)), 0.0))
    res.append(check("hand at the cap", run(meas(tips=[HOLE + np.array([-0.005, 0, 0])] * 4)), 0.6))
    res.append(check("board crushed", run(m, veto=True), 0.0))
    res.append(check("hole missed (channel 12 mm to the side)", run(meas(dy=0.012)), 0.0))
    # strap snap rule (pure parts of the runtime rule)
    res.append(check("cap seated: at the break line", float(S.cap_at_break_line(L.Q_SEAT)), 1.0))
    res.append(check("cap 50 mm out on the straps: not at the break line", float(S.cap_at_break_line(L.Q_SEAT + 0.05)), 0.0))
    res.append(check("cap 1 mm short of seated: at the line", float(S.cap_at_break_line(L.Q_SEAT + 0.001)), 1.0))
    res.append(check("cap 2 mm short of seated (not seated): no snap", float(S.cap_at_break_line(L.Q_SEAT + 0.002)), 0.0))
    ax = np.array([-1.0, 0.0, 0.0])
    brk = np.zeros(3)
    tips_far = [np.array([-0.13, 0.005, 0.0]), np.array([-0.13, -0.005, 0.0])]
    tips_near = [np.array([-0.02, 0.01, 0.0]), np.array([-0.02, 0.006, 0.0])]
    grip = ([np.array([0.0, 4.0, 0.0]), np.array([0.3, -2.7, 0.0])], np.zeros(3), tips_far)  # strap hand pinching
    push = ([np.array([0.0, -8.0, 0.0]), np.zeros(3)], np.zeros(3), tips_near)  # closed fingers pushing sideways
    pure = ([np.array([0.0, 4.0, 0.0]), np.array([0.0, -4.0, 0.0])], np.zeros(3), tips_far)
    res.append(check("pure pinch (equal and opposite finger forces) is 0", S.bend_moment([pure], brk, ax), 0.0))
    res.append(check("pinch plus a 1.3 N sideways push at 130 mm counts", round(S.bend_moment([grip], brk, ax), 4), 0.169))
    res.append(check("sideways push 8 N at 20 mm", round(S.bend_moment([push], brk, ax), 4), 0.16))
    res.append(check("axial pull is not a bend", S.bend_moment([([np.array([-5.0, 0, 0]), np.zeros(3)], np.zeros(3), tips_far)], brk, ax), 0.0))
    print("ALL OK" if all(res) else "SOME FAILED")
    return 0 if all(res) else 1


if __name__ == "__main__":
    raise SystemExit(main())
