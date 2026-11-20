"""Scorer of the one-round magazine loading task. Pure function of measured signals, no simulator (tests:
test_task_scoring.py).

Frame: the magazine frame, metres. x forward along the floor plate (bullet end of a loaded round points to +x), y across,
z up, origin under the mouth. Geometry: the magazine collider (collider/meta.json; community CAD, lips clipped to an
8.5 mm gap). The constants are the ones the demo was developed with.

Loaded (all at once): whole case under the constant-gap lips, base <= 2 mm from the rear wall, centre <= 1 mm from the
lip line, yaw < 5 deg, bullet forward, tilt < 10 deg.

Latched 0 for the whole trial if at any time: the round entered or rested in the lip gap from above (part of the case
axis over the constant-gap lips more than 1 mm above the lip line while the case's lowest point is below the lip tops);
the magazine moved more than 2 mm or tilted more than 2 deg from its seat in the fixture; any object fell off the table.
Otherwise the level of the current state, counted only while every finger-pad point is >= 50 mm from the round centre:
  0.25 round retained under the lips, bullet forward, but not loaded, or loaded with the tool still on the magazine;
  1.0  round loaded and tool off the magazine.
The score is the lowest level over the last 2 s (0 before 2 s have passed). The trial ends on 1.0.
"""
import json
import math
from collections import deque
from pathlib import Path

META = json.loads((Path(__file__).resolve().parent / "collider" / "meta.json").read_text())
MM = 0.001

# dummy round, 9x19 outline
R_CASE, L_CASE, R_BUL, L_BUL = 0.00498, 0.01915, 0.00451, 0.0055
OAL = L_CASE + L_BUL + R_BUL

# magazine mouth
Z_TIP = META["z_lip_top"] * MM
_HG, _SL = META["lip_face"]["halfgap_at_112"], META["lip_face"]["slope"]
Z_RET0 = 2 * (_HG + _SL * 112.0 - R_CASE * 1000 * math.hypot(1, _SL)) * MM  # lip line: centred round on both lip faces
X_LIP0, X_LIP1, X_RAMP1 = META["x_lip0"] * MM, META["x_lip1"] * MM, META["x_ramp1"] * MM  # constant-gap lips, ramp end
X_FRONT = META["x_front_in_at_z107"] * MM
X_R110, DXDZ = META["x_rear_at_z110"] * MM, META["rear_dxdz"]
W_IN = 0.0185

# loaded rule
GAP_MAX = 0.002  # base to rear wall
UNDER_TOL = 0.0003  # centre above the lip line allowed for "under the lips"
DZ_MAX = 0.001  # |centre - lip line|
YAW_MAX_DEG = 5.0
TILT_MAX_DEG = 10.0
THROUGH_TOL = 0.001  # case axis above the lip line over the constant-gap lips = case in the lip gap from above
CLEAR_MIN = 0.050  # every finger-pad point to the round centre
TOOL_OFF = 0.072  # tool origin (ring bottom centre) from its seated pose (the height at which the demo puts it on)
CONTACT_N = 0.05  # contact force counted as touching (as the finished task's scorer)
# vetoes
MAG_SHIFT, MAG_TILT_DEG = 0.002, 2.0
FALL = 0.05  # below the table top
HOLD_S = 2.0


def x_rear(z):
    return X_R110 + (z - 0.110) * DXDZ


def z_ret(e):
    return Z_RET0 - 2 * abs(e)


def _sub(p, q, k=1.0):
    return tuple(a - k * b for a, b in zip(p, q))


def round_state(c, a):
    """c: round centre, a: unit axis toward the bullet end, both in the magazine frame."""
    b = _sub(c, a, OAL / 2)
    gap = b[0] - x_rear(b[2] + R_CASE)
    mouth = b[0] + a[0] * L_CASE
    tilt = math.degrees(math.asin(max(-1.0, min(1.0, a[2]))))
    yaw = math.degrees(math.atan2(a[1], a[0]))
    dz = c[2] - z_ret(c[1])
    inside = (abs(c[1]) < W_IN / 2 + 0.001 and x_rear(c[2] + R_CASE) - 0.002 < b[0] and c[0] < X_FRONT + 0.004
              and 0.04 < c[2] < Z_TIP)
    fwd = a[0] > math.cos(math.radians(20))
    reversed_ = inside and (a[0] < -math.cos(math.radians(45)) or a[2] < -0.7)
    retained = (inside and fwd and c[2] < Z_RET0 + UNDER_TOL and abs(tilt) < TILT_MAX_DEG and abs(c[1]) < W_IN / 2
                and b[0] < X_LIP1)  # held by the constant-gap lips, at any depth
    loaded = retained and mouth < X_RAMP1 and gap < GAP_MAX and abs(dz) < DZ_MAX and abs(yaw) < YAW_MAX_DEG
    return dict(base=b, gap=gap, mouth=mouth, tilt_deg=tilt, yaw_deg=yaw, dz=dz, inside=inside, reversed=reversed_,
                retained=retained, loaded=loaded)


def through_gap_now(c, a, n=40):
    """Round entering or resting in the lip gap from above: at some point of the case axis with x between the two ends
    of the constant-gap section, the axis is more than 1 mm above the lip line and the case's lowest point is below the
    lip tops. A round slid in from the front under the lips did not meet this in any demo run; a round laid on top of
    the lips does."""
    b = _sub(c, a, OAL / 2)
    for k in range(n + 1):
        p = _sub(b, a, -L_CASE * k / n)
        if X_LIP0 <= p[0] <= X_LIP1 and abs(p[1]) < W_IN / 2 and z_ret(p[1]) + THROUGH_TOL < p[2] < Z_TIP + R_CASE:
            return True
    return False


def tool_off(tool_rel, tool_contact_n):
    return tool_contact_n < CONTACT_N and math.sqrt(sum(v * v for v in tool_rel)) >= TOOL_OFF


def level(m):
    """Level of one measured state (no veto, no hold)."""
    r = round_state(m["round_c"], m["round_a"])
    if m["finger_round_m"] < CLEAR_MIN or r["reversed"]:
        return 0.0
    if r["loaded"] and tool_off(m["tool_rel"], m["tool_contact_n"]):
        return 1.0
    return 0.25 if r["retained"] else 0.0


def veto(m):
    if m["through_gap"]:
        return "round entered or rested in the lip gap from above"
    if m["mag_shift_m"] > MAG_SHIFT or m["mag_tilt_deg"] > MAG_TILT_DEG:
        return "magazine moved in the fixture more than 2 mm / 2 deg"
    if m["fell"]:
        return "an object fell off the table"
    return None


class LoadScorer:
    """step() once per control step. m holds the current state and, for the veto flags, the worst value over the
    physics substeps of that control step:
      round_c, round_a: round centre and unit axis (bullet end), magazine frame, m
      finger_round_m: smallest distance from any finger-pad point (TCP and pad tip, 6 mm outside each pad, both
        arms) to the round centre, m
      tool_rel: tool origin minus its seated origin, magazine frame, m
      tool_contact_n: largest contact force of the tool on the magazine, follower or round, N
      through_gap: through_gap_now() was true in any substep
      mag_shift_m, mag_tilt_deg: magazine displacement / tilt from its start pose (max over substeps)
      fell: any object below the table top by more than 50 mm"""

    def __init__(self):
        self.veto = None
        self.t = 0.0
        self.hist = deque()
        self.score = 0.0

    def step(self, dt, m):
        self.t += dt
        if self.veto is None:
            self.veto = veto(m)
        lv = 0.0 if self.veto else level(m)
        self.hist.append((self.t, lv))
        while len(self.hist) > 1 and self.hist[1][0] <= self.t - HOLD_S + 1e-9:
            self.hist.popleft()
        held = self.t - self.hist[0][0] >= HOLD_S - 1e-9
        self.score = min(v for _, v in self.hist) if held else 0.0
        return self.score
