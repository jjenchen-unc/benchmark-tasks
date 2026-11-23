"""Geometry, masses, mechanism constants and start states of the triple-beam task (pure NumPy; no simulator).

Scene (station frame: x forward from the line between the arm bases, y to the robot's left of their midpoint, z up;
table top z = TOP): an Ohaus 750-SO triple-beam balance (610 g capacity, 0.1 g readability) on the bench in front of
the robot, its stainless pan on the robot's left and its pointer on the right; a sealed sample jar of seeded mass stands
on the pan.

Balance frame B: origin on the fulcrum (centre knife-edge) axis, +x along the beams toward the pointer, +y along the
knife-edge axis (away from the robot at yaw 0), +z up. Station rotation of B = rz(yaw - 90 deg).

Mechanism (all emergent in PhysX, see ``isaac/balkit.py``):
* the beam assembly turns on the fulcrum (revolute about B y), magnetically damped (velocity-proportional torque)
  and limited by the pointer's stops;
* the pan hangs on the pan-bearing knife-edge A_PAN to the pan side of the fulcrum (revolute) and is kept level by
  the check link (Roberval: a mimic constraint pan angle = -beam angle), so the load acts at the bearing wherever it
  lies on the pan;
* each rider is a body on its beam (prismatic along B x) whose weight acts on the beam where it sits; the 500 g and
  100 g riders sit in V-notches (detent), the 10 g poise slides with clip friction;
* the restoring torque comes from the beam's centre of mass below the fulcrum (sensitivity), the zero from the beam's
  mass distribution (balance zeroed with the empty pan and all riders at zero).

Real dimensions and masses are estimates for the Ohaus 700/800-series triple-beam balances (no drawing was available
on this host; see README fidelity). Rider masses follow from the lever law: rider mass x graduated span = beam
capacity x pan-bearing arm.
"""

from __future__ import annotations

import math

import numpy as np

TOP = 0.7475  # station table top (yam_arena station TABLE_TOP)
G = 9.81


def rz(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# ------------------------------------------------------------------------------------------------ beam / lever
H_PIVOT = 0.090  # fulcrum axis above the bench
A_PAN = 0.045  # pan-bearing knife-edge to fulcrum (lever arm of the load)
S0 = 0.050  # zero notch / zero line of every beam, from the fulcrum along +x
SPAN = 0.200  # graduated length of every beam (0..full scale)
L_POINTER = 0.290  # pointer tip from the fulcrum
CAP = dict(r500=0.500, r100=0.100, r10=0.010)  # full scale of each beam (kg)
NOTCH = dict(r500=0.100, r100=0.010)  # notch pitch in kg on the notched beams
RIDERS = ("r500", "r100", "r10")
RIDER_MASS = {k: CAP[k] * A_PAN / SPAN for k in RIDERS}  # lever law: 112.5 g, 22.5 g, 2.25 g
BEAM_Y = dict(r10=-0.030, r100=0.0, r500=0.030)  # front (toward the robot): 10 g; middle: 100 g; rear: 500 g
BEAM_W = 0.004  # beam bar thickness (y)
BEAM_ZTOP = 0.006  # beam top edge (notches) above the fulcrum axis
BEAM_ZBOT = -0.012
BEAM_X = (-0.030, L_POINTER + 0.002)

K_SENS = 0.025  # N m / rad restoring stiffness (beam centre of mass below the fulcrum): 0.1 g -> 0.51 mm at the pointer
THETA_STOP = 0.0241  # rad: pointer stops at +-7 mm
DAMP = 0.024  # N m s / rad magnetic damping (about 0.8 of critical; settles in about 3 s)
M_BEAM = 0.300  # beam assembly (three beams, pointer, knife-edges, damper vane)
M_PAN = 0.200  # 6 in stainless pan plus pan support (hangs on the pan bearing)
# zero: beam centre of mass placed so the empty pan balances the riders at zero; and below the axis by K/(M g)
X_CB = (M_PAN * A_PAN - sum(RIDER_MASS[k] * S0 for k in RIDERS)) / M_BEAM
Z_CB = -K_SENS / (M_BEAM * G)
I_BEAM = (2.0e-4, 3.4e-3, 3.4e-3)  # about its centre of mass (B axes)

ZERO_BAND_MM = 0.9  # pointer within +-0.9 mm of the zero mark (about +-0.18 g)
REST_SPEED_MM_S = 0.6  # pointer speed below this ...
REST_S = 1.5  # ... and inside the band for this long = at rest on zero

# ------------------------------------------------------------------------------------------------ riders
V_ALPHA_DEG = 45.0  # V-notch flank angle
MU_RIDER = 0.25  # rider on the beam's notch edge (nickel-plated steel on aluminium, estimate)
NOTCH_CAPTURE = 0.0015  # m: inside this the rider drops into the notch (V half-width)
SEAT_TOL = 0.0003  # m: seated = within 0.3 mm of the notch bottom
POISE_FRICTION = 0.15  # N: the 10 g poise's spring clip (slides under a light push)
RIDER_HANDLE = dict(r500=(0.016, 0.006, 0.022), r100=(0.014, 0.006, 0.022), r10=(0.012, 0.005, 0.022))  # x, y, z (gripped across y)
HANDLE_Z0 = BEAM_ZTOP + 0.002  # handles start above the beam top


def notch_s(k, n):
    """Position (m from the fulcrum) of notch ``n`` on notched beam ``k``."""
    return S0 + n * NOTCH[k] / CAP[k] * SPAN


def n_notches(k):
    return int(round(CAP[k] / NOTCH[k]))


def s_of_grams(k, grams):
    return S0 + grams / 1000.0 / CAP[k] * SPAN


def grams_of_s(k, s):
    return (s - S0) / SPAN * CAP[k] * 1000.0


def notch_state(k, s):
    """(nearest notch index, offset m) on notched beam k."""
    n = int(round((s - S0) / (NOTCH[k] / CAP[k] * SPAN)))
    n = max(0, min(n_notches(k), n))
    return n, s - notch_s(k, n)


def reading_g(s):
    """Balance reading (g) from the rider positions {k: s}: notch values plus the poise's scale reading; None when a
    notched rider is not seated in a notch (the reading cannot be taken)."""
    tot = 0.0
    for k in ("r500", "r100"):
        n, off = notch_state(k, s[k])
        if abs(off) > SEAT_TOL:
            return None
        tot += n * NOTCH[k] * 1000.0
    return tot + grams_of_s("r10", s["r10"])


def imbalance_g(m_g, s):
    """Pan-side excess (g) of a sample of m_g grams against the riders at {k: s} (positive: pan heavy)."""
    return m_g - sum(grams_of_s(k, s[k]) for k in RIDERS)


# ------------------------------------------------------------------------------------------------ pan, base, tower
PAN_R = 0.076  # 6 in pan
PAN_X = -0.100  # pan centre (B x)
PAN_Z = 0.026  # pan top surface above the fulcrum axis
PAN_RIM = 0.008
BASE_X = (-0.185, L_POINTER + 0.030)
BASE_Y = (-0.075, 0.075)
BASE_T = 0.028
TOWER_X = L_POINTER + 0.010  # pointer scale plate (fixed to the base)

# ------------------------------------------------------------------------------------------------ sample (sealed jar)
# A white 250 mL HDPE wide-mouth jar with its screw cap on, holding a soil sample (opaque: the fill does not show).
M_RANGE_G = (120.0, 480.0)
FINE_RANGE_G = (1.0, 9.0)  # grams past the 10 g step: the 10 g poise is always needed, never at an end
FINE_MID_EXCL_G = 0.6  # ... and never within 0.6 g of mid-scale (5 g: a "poise at the middle" guess never fits)
JAR_R = 0.0350  # 70 mm across
JAR_H = 0.0980  # with the cap
JAR_EMPTY_KG = 0.040  # jar plus cap
SOIL_RHO = 1750.0  # kg / m3 dry soil (estimate)
JAR_WALL_Z = 0.003  # jar bottom thickness
SAMPLE_OFF = 0.022  # m: jar axis within this of the pan centre


def jar_mass_props(m_g):
    """(mass kg, centre of mass z from the jar's centre, diagonal inertia) of the jar holding m_g grams in all."""
    m = m_g / 1000.0
    soil = max(0.0, m - JAR_EMPTY_KG)
    h = soil / SOIL_RHO / (math.pi * (JAR_R - 0.002) ** 2)  # fill height
    z_soil = -JAR_H / 2 + JAR_WALL_Z + h / 2
    z = (soil * z_soil) / m  # empty jar's own centre at the jar centre
    ixx = JAR_EMPTY_KG * (JAR_R ** 2 / 2 + JAR_H ** 2 / 6) + soil * ((JAR_R - 0.002) ** 2 / 4 + h ** 2 / 12) + soil * (
        z_soil - z) ** 2
    izz = JAR_EMPTY_KG * JAR_R ** 2 + soil * (JAR_R - 0.002) ** 2 / 2
    return m, z, (ixx, ixx, izz)


# ------------------------------------------------------------------------------------------------ placement
NOMINAL = (0.225, 0.060)  # fulcrum axis point (middle beam) station x, y
POS_D = 0.008  # m: seeded offset of the balance in x and y
YAW = (-3.0, 3.0)  # deg
FRICTION = dict(base=0.6, beam=0.4, pan=0.4, rider=0.5, sample=0.6)

OFFICIAL_SEED = 1415158438  # inspect-robots trial seed of eval seed 0 (the default start)
GUARD_G = 3.0  # open-loop guard: a seeded start's mass is at least this far from the default start's mass
NAMED_SETUPS = (
    dict(m=263.4, dx=0.0, dy=0.0, yaw=0.0, ox=0.006, oy=-0.004, ryaw=20.0),
    dict(m=347.8, dx=0.004, dy=-0.005, yaw=1.5, ox=-0.008, oy=0.006, ryaw=75.0),
    dict(m=158.2, dx=-0.005, dy=0.004, yaw=-2.0, ox=0.002, oy=0.010, ryaw=140.0),
)


def _draw_mass(rng):
    for _ in range(1000):
        m = float(rng.uniform(*M_RANGE_G))
        f = m % 10.0
        if FINE_RANGE_G[0] <= f <= FINE_RANGE_G[1] and abs(f - 5.0) >= FINE_MID_EXCL_G:
            return round(m, 1)
    raise RuntimeError("mass draw failed")


def _seeded(n):
    rng = np.random.default_rng(int(n))
    m = _draw_mass(rng)
    dx, dy = (float(v) for v in rng.uniform(-POS_D, POS_D, 2))
    yaw = float(rng.uniform(*YAW))
    r = SAMPLE_OFF * math.sqrt(float(rng.uniform(0.0, 1.0)))
    a = float(rng.uniform(0.0, 2 * math.pi))
    ryaw = float(rng.uniform(0.0, 360.0))
    return dict(m=m, dx=dx, dy=dy, yaw=yaw, ox=r * math.cos(a), oy=r * math.sin(a), ryaw=ryaw, name=f"seed {int(n)}")


def setup_spec(setup):
    """Start state {'m' (g), 'dx', 'dy', 'yaw', 'ox', 'oy' (jar offset on the pan, B frame), 'ryaw' (label turn),
    'name'}. ``setup`` is a named-setup index or ('seed', n).

    Seeded starts: mass 120-480 g with 1-9 g past the 10 g step (not 4.4-5.6 g), balance moved up to 8 mm in x and y
    and turned up to 3 deg, jar anywhere within 22 mm of the pan centre. Open-loop guard: any seeded start whose
    mass is within GUARD_G of the default (official-seed) start's mass gets its mass redrawn (second seeded stream), so
    that the default start's rider settings leave every other start at least 3 g out of balance."""
    if isinstance(setup, (tuple, list)) and setup[0] == "seed":
        n = int(setup[1])
        sp = _seeded(n)
        if n == OFFICIAL_SEED:
            return sp
        ref = _seeded(OFFICIAL_SEED)
        if abs(sp["m"] - ref["m"]) < GUARD_G:
            rng = np.random.default_rng([n, 7])
            for _ in range(1000):
                m = _draw_mass(rng)
                if abs(m - ref["m"]) >= GUARD_G:
                    sp["m"] = m
                    break
        return sp
    k = int(setup) % len(NAMED_SETUPS)
    return dict(NAMED_SETUPS[k], name=f"setup {k}")


def balance_pose(spec):
    """Station position of the fulcrum point (middle beam) and the rotation of frame B."""
    p = np.array([NOMINAL[0] + spec["dx"], NOMINAL[1] + spec["dy"], TOP + H_PIVOT])
    return p, rz(spec["yaw"] - 90.0)


def to_station(spec, pb):
    p, r = balance_pose(spec)
    return p + r @ np.asarray(pb, float)


def sample_start(spec, pose=None):
    """Jar centre station position and rotation, standing on the pan (beam on its lower stop: pan side down);
    ``pose`` = (fulcrum point, B rotation) of the balance, default: the start's own."""
    p, r = balance_pose(spec) if pose is None else pose
    pan_drop = A_PAN * math.sin(THETA_STOP)
    pb = np.array([PAN_X + spec["ox"], spec["oy"], PAN_Z - pan_drop + JAR_H / 2 + 0.0005])
    return np.asarray(p, float) + np.asarray(r, float) @ pb, np.asarray(r, float) @ rz(spec["ryaw"])


def on_pan(rel):
    """Jar centre ``rel`` relative to the pan centre's top point, in the pan frame (level): standing on the pan."""
    return math.hypot(rel[0], rel[1]) < PAN_R - 0.01 and abs(rel[2] - JAR_H / 2) < 0.008


# ------------------------------------------------------------------------------------------------ shapes
def box(center, size):
    hx, hy, hz = (s / 2 for s in size)
    c = np.asarray(center, float)
    return np.array([[sx * hx, sy * hy, sz * hz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) + c


def box_xyz(xr, yr, zr):
    return box(((xr[0] + xr[1]) / 2, (yr[0] + yr[1]) / 2, (zr[0] + zr[1]) / 2),
               (xr[1] - xr[0], yr[1] - yr[0], zr[1] - zr[0]))


def cyl_z(center, r, h, n=24):
    c = np.asarray(center, float)
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    return np.array([[r * math.cos(t), r * math.sin(t), z] for z in (-h / 2, h / 2) for t in a]) + c


def ring_segments(r_in, r_out, z0, z1, n=32):
    out = []
    for i in range(n):
        a0 = 2 * math.pi * i / n
        a1 = 2 * math.pi * (i + 1) / n
        pts = []
        for a in (a0, a1):
            c, s = math.cos(a), math.sin(a)
            pts += [[r_out * c, r_out * s, z0], [r_out * c, r_out * s, z1], [r_in * c, r_in * s, z1],
                    [r_in * c, r_in * s, z0]]
        out.append(np.array(pts))
    return out
