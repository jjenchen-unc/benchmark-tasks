"""CPU checks of the cord law, route state machine, seat geometry and scorer bands (no simulator)."""
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "d1r_arena"))
import geometry as geo  # noqa: E402
import strap  # noqa: E402
from scoring import Scorer  # noqa: E402

f = strap.ToteFrame()
DT = 1 / 240

# 1. Cord law: slack -> 0, 10 % stretch -> 6.75 N, opposite forces along the cord.
c = strap.Cord(3, f)
T, fa, fb, fl = c.forces(np.zeros(3), np.array([0.40, 0, 0]), 0.02, DT)
assert T == 0 and not fa.any() and not fl.any()
c = strap.Cord(3, f)
T, fa, fb, fl = c.forces(np.zeros(3), np.array([0.46 * 1.1, 0, 0]), 0.02, DT)
assert abs(T - 6.75) < 1e-6 and np.allclose(fa, [6.75, 0, 0]) and np.allclose(fb, -fa) and not fl.any()

# 2. Seated poses are recognised on the two different eyes; a displaced hook is not seated; moving hook not seated.
pa, qa = strap.seated_hook_pose(f, 0)
pb, qb = strap.seated_hook_pose(f, 1)
assert strap.seat_check(pa, qa, 0.0, f) == 0 and strap.seat_check(pb, qb, 0.0, f) == 1
assert strap.seat_check(pa + np.array([0, 0, 0.02]), qa, 0.0, f) is None
assert strap.seat_check(pa, qa, 0.2, f) is None

# 3. Route: lowering a taut cord from above onto the lid -> over_lid; pushing through a wall -> failure.
aa, ab = strap.hook_attach_world(pa, qa), strap.hook_attach_world(pb, qb)
c = strap.Cord(3, f)
lift = np.array([0, 0, 0.30])
c.update_route(aa + lift, ab + lift, 0.02)
assert c.route == "free" and c.failure is None
for s in np.linspace(1, 0, 200):
    c.update_route(aa + s * lift, ab + s * lift, 0.02)
assert c.route == "over_lid" and c.failure is None, (c.route, c.failure)
c2 = strap.Cord(3, f)
side = f.R @ np.array([0, 0.4, 0])  # cord swept sideways through the tote at eye height
c2.update_route(aa + side, ab + side, 0.02)
for s in np.linspace(1, 0, 200):
    c2.update_route(aa + s * side, ab + s * side, 0.02)
assert c2.failure == "cord_through_solid", c2.failure

# 4. Analytic statics at the shut lid: only the 46 cm strap closes the lid (> 30 N net spring) under 28 N.
down = {}
for i in range(4):
    c = strap.Cord(i, f)
    c.route = "over_lid"
    T, _, _, fl = c.forces(aa, ab, 0.0, DT)
    down[geo.RELAXED_LENGTHS[i]] = (T, -fl[2])
assert down[0.46][0] < geo.HOOK_RATING_N and down[0.46][1] > geo.F_UP_TALL
assert down[0.36][0] > geo.HOOK_RATING_N
assert down[0.52][1] < geo.F_UP_TALL and down[0.58][1] < geo.F_UP_TALL
print("statics (tension N, lid down N):", {k: tuple(round(x, 2) for x in v) for k, v in down.items()})


# 5. Scorer bands from synthetic histories.
def row(gap, seated=("a", "b"), hands=True, route="over_lid", failure=None, strap_name="strap_4"):
    straps = {f"strap_{i}": dict(seated_a=None, seated_b=None, hands_off=True, route="free", failure=None) for i in range(1, 5)}
    s = straps[strap_name]
    s.update(seated_a=0 if "a" in seated else None, seated_b=1 if "b" in seated else None, hands_off=hands,
             route=route, failure=failure)
    return dict(wrist_force_measured=True, wrist_forces_n={"left":2.,"right":2.}, robot_contacts_measured=True, robot_contacts={n:False for n in ("lid", *tuple(f"strap_{i}_{e}" for i in range(1,5) for e in ("a","b")))}, guard_failure=None, totes={"tote_tall": dict(lid_gap_m=gap)}, hooks={}, straps=straps, hands_off_lid=hands)


def score(rows):
    sc = Scorer()
    for r in rows:
        sc.update(r)
    return sc.score


assert score([row(0.02, seated=())] * 600) == 0.0
assert score([row(0.0)] * 400) == 1.0
assert score([row(0.0)] * 399) == 0.5  # 19.95 s: full hold not reached
assert score([row(0.005)] * 400) == 0.5
assert score([row(0.012)] * 400) == 0.25  # both seated but gap > 10 mm
assert score([row(0.012, seated=("a",), route="free")] * 100) == 0.25
assert score([row(0.012, seated=("a",), route="free")] * 99) == 0.0
assert score([row(0.0)] * 380 + [row(0.0, hands=False)]) == 0.0  # hand on lid/hook at the end
assert score([row(0.0)] * 200 + [row(0.0, hands=False)] + [row(0.0)] * 399) == 0.5  # hold restarted
assert score([row(0.0, failure="hook_bent_open")] + [row(0.0)] * 500) == 0.0  # permanent
assert score([row(0.0, failure="cord_through_solid")] + [row(0.0)] * 500) == 0.0
two = row(0.0)
two["straps"]["strap_2"].update(seated_a=0)
assert score([two] * 500) == 0.0  # two straps on the tote: not "exactly one"
assert score([row(0.0, route="free")] * 500) == 0.25  # both seated but cord not over the lid
print("strap/scoring tests passed")
