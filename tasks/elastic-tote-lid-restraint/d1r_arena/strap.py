"""Elastic bungee cord load path and hook-on-eye seat geometry for the D1R Arena pilot. Pure numpy.

Mechanism (approximation, disclosed): each strap is a massless 8 mm elastic cord between the
attachment points of its two hooks (far end of each hook handle). Tension follows the native law
T = EA * max(0, L - L0) / L0 (EA = 67.5 N per unit strain, i.e. 0.68 N per percent stretch) plus a
small taut-only damping term. The cord path is either straight ("free") or, once it has been lowered
onto the tall tote from above, wraps over the visible lid top edges ("over_lid", via points 4 mm outward
and above the lid corners as in native D1R). Equal and opposite cord forces act on the hooks (at the
attachment point, with the matching torque) and, in the over_lid route, the resultant of the two
wrap deflections acts on the lid body. The policy never sees any of this state.

The route is a state machine evaluated at every PhysX substep (240 Hz):
  free -> over_lid only if the straight cord at the previous substep lay entirely above the lid top
           where it crossed the tote footprint (i.e. it was lowered over the lid from above);
  over_lid -> free when the straight cord no longer touches the tote (lifted off);
  any other straight-cord intersection with the tote box (walls, bottom, lid) is a permanent
  cord_through_solid failure (fail closed). Both totes, their brackets and moving lids are checked by RouteGuard using
  exact segment distances and conservative interpolation between physics samples.
"""
from __future__ import annotations

import math

import numpy as np

import geometry as geo
from routeguard import RouteGuard, segment_box_distance

CORD_RADIUS = 0.004
CORD_DAMPING = 0.5  # N s/m, taut-only, numerical
VIA_OUT = 0.004  # native D1R via sites 4 mm outward/above lid corners
ATTACH_LOCAL = np.array([-geo.HANDLE[0], 0.0, 0.0])  # cord end: far end of the handle, hook frame
COM_LOCAL = np.array([geo.SHANK_MASS * geo.SHANK_POS[0] / geo.HOOK_MASS, 0.0, 0.0])
# L-shaped wire foot at the shank tip, 12 mm long toward hook-local -z (native "12 mm foot").
FOOT_POS, FOOT_HALF = (0.0565, 0.0, -0.010), (0.0035, 0.0035, 0.006)
# Seat region for the eye-bar centreline in hook coordinates: between shank underside and foot,
# inside the foot (bar of radius 3 mm, 1 mm compliance like the native seat tolerance).
SEAT_X = (FOOT_POS[0] - FOOT_HALF[0] - geo.BAR_RADIUS - 0.012, FOOT_POS[0] - FOOT_HALF[0] - geo.BAR_RADIUS + 0.001)
# Shank-side tolerance 2 mm: job-43188 measured the loaded bar 1.7 mm into the shank box (PhysX contact
# penetration of a thin box on a 3 mm capsule at 13.6 N) while the hook was demonstrably holding.
SEAT_Z = (FOOT_POS[2] - FOOT_HALF[2] + geo.BAR_RADIUS - 0.001, -geo.SHANK_HALF[2] - geo.BAR_RADIUS + 0.002)
SEAT_MAX_SPEED = 0.05  # m/s: a credited seat must be at rest (supported)
SEATED_BAR_LOCAL = np.array([SEAT_X[1] - 0.0015, 0.0, SEAT_Z[1] - 0.0005])  # used only by the diagnostic


def quat_to_R(q_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in q_xyzw)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def segment_hits_box(p, q, half, margin=CORD_RADIUS):
    """Slab test of segment p->q against the axis-aligned box |x|<=half (tote frame), inflated by margin."""
    h = np.asarray(half, float) + margin
    d = q - p
    t0, t1 = 0.0, 1.0
    for i in range(3):
        lo, hi = -h[i], h[i]
        if abs(d[i]) < 1e-12:
            if p[i] < lo or p[i] > hi:
                return False
            continue
        a, b = (lo - p[i]) / d[i], (hi - p[i]) / d[i]
        a, b = min(a, b), max(a, b)
        t0, t1 = max(t0, a), min(t1, b)
        if t0 > t1:
            return False
    return True


class ToteFrame:
    """Tall-tote frame: origin under the tote centre on the table, axes = native axes (long axis x)."""

    def __init__(self, name="tote_tall"):
        t = geo.tote(name)
        self.R = geo.rot_z(t["yaw"])
        self.o = np.asarray(t["origin"], float)
        self.hx, self.hy = t["h"]
        self.lid_shut_z = t["lid_shut_z"]
        self.eyes = [np.asarray(e[1], float) for e in t["eyes"]]
        self.bar_axis_world = self.R @ np.array([0.0, 1.0, 0.0])

    def local(self, p):
        return self.R.T @ (np.asarray(p, float) - self.o)

    def world(self, p):
        return self.o + self.R @ np.asarray(p, float)

    def box(self, lid_gap):
        """Centre and half extents of tote+lid solid (local)."""
        top = self.lid_shut_z + lid_gap + geo.LID_T
        return np.array([0.0, 0.0, top / 2]), np.array([self.hx, self.hy, top / 2]), top


class Cord:
    def __init__(self, strap: int, frame: ToteFrame):
        self.strap = strap
        self.L0 = geo.RELAXED_LENGTHS[strap]
        self.frame = frame
        self.route = "free"
        self.prev_above = False
        self.prev_len = None
        self.failure = None
        self.route_guard = RouteGuard()
        self.max_tension = 0.0
        self.tension = 0.0
        self.length = 0.0

    def path(self, pa_w, pb_w, lid_gap):
        """Shortest visible path in the fixed4mm-offset top-corner family.

        Edge positions along tote-y minimize the polygonal path length. At a
        free/one-corner transition the new edge point lies on the old segment;
        no unrelated far-corner detour or lateral jump is introduced.
        """
        a_w,b_w=np.asarray(pa_w,float),np.asarray(pb_w,float)
        if self.route!="over_lid":
            return [a_w,b_w]
        f=self.frame
        centre,half,top=f.box(lid_gap)
        a,b=f.local(a_w),f.local(b_w)
        xa=np.copysign(f.hx+VIA_OUT,a[0]);xb=np.copysign(f.hx+VIA_OUT,b[0])
        z=top+VIA_OUT
        def candidate(xs):
            xz=[np.array([a[0],a[2]])]+[np.array([x,z]) for x in xs]+[np.array([b[0],b[2]])]
            distances=[float(np.linalg.norm(q-p)) for p,q in zip(xz[:-1],xz[1:])]
            total=sum(distances)
            corners=[]
            walked=0.
            for i,x in enumerate(xs):
                walked+=distances[i]
                y=a[1]+(b[1]-a[1])*walked/max(total,1e-12)
                corners.append(np.array([x,np.clip(y,-f.hy,f.hy),z]))
            return [a,*corners,b]
        valid=[]
        for xs in ((),(xa,),(xb,),(xa,xb)):
            points=candidate(xs)
            if all(segment_box_distance(p,q,centre,np.eye(3),half)>=CORD_RADIUS-1e-8
                   for p,q in zip(points[:-1],points[1:])):
                length=sum(float(np.linalg.norm(q-p)) for p,q in zip(points[:-1],points[1:]))
                valid.append((length,len(points),points))
        if not valid:
            # Keep a visible path for diagnostics; unchanged history/solid guard
            # will reject this geometry. Never clear or bypass a failure.
            points=candidate((xa,xb))
        else:
            points=min(valid,key=lambda v:(v[0],v[1]))[2]
        return [f.world(p) for p in points]

    def update_route(self, pa_w, pb_w, lid_gap):
        f = self.frame
        c, half, top = f.box(lid_gap)
        a, b = f.local(pa_w) - c, f.local(pb_w) - c
        hits = segment_hits_box(a, b, half)
        # Height of the straight cord where it crosses the tote footprint (|x| <= hx).
        above = False
        if not hits:
            d = b - a
            ts = [0.0, 1.0] + [(s * (f.hx + CORD_RADIUS) - a[0]) / d[0] for s in (-1, 1) if abs(d[0]) > 1e-9]
            zs = [a[2] + d[2] * t for t in ts if 0 <= t <= 1 and abs(a[0] + d[0] * t) <= f.hx + CORD_RADIUS + 1e-9]
            above = bool(zs) and min(zs) >= half[2] + CORD_RADIUS - 1e-9
        opposite = f.local(pa_w)[0] * f.local(pb_w)[0] < 0 and min(abs(f.local(pa_w)[0]), abs(f.local(pb_w)[0])) > f.hx
        # A cord may retain one top-edge contact while its other endpoint moves
        # above the lid into the footprint. Continuous segment/solid history is
        # still certified by RouteGuard; an initial below-side entry stays invalid.
        upper_endpoint = max(a[2], b[2]) >= half[2] + CORD_RADIUS - 1e-9
        top_access = opposite or upper_endpoint
        if self.route == "over_lid":
            if not hits:
                self.route = "free"
            elif not top_access:
                self.failure = self.failure or "cord_through_solid"
        elif hits:
            if self.prev_above and top_access:
                self.route = "over_lid"
            else:
                self.failure = self.failure or "cord_through_solid"
        self.prev_above = above or (self.route == "over_lid")

    def forces(self, pa_w, pb_w, lid_gap, dt):
        """Tension and world forces on hook a, hook b and the lid (lid force is zero unless over_lid)."""
        pts = self.path(np.asarray(pa_w, float), np.asarray(pb_w, float), lid_gap)
        segs = [pts[i + 1] - pts[i] for i in range(len(pts) - 1)]
        L = float(sum(np.linalg.norm(s) for s in segs))
        rate = 0.0 if self.prev_len is None else (L - self.prev_len) / dt
        self.prev_len = L
        strain = (L - self.L0) / self.L0
        T = 0.0 if strain <= 0 else max(0.0, geo.STRAP_EA * strain + CORD_DAMPING * rate)
        self.tension, self.length = T, L
        self.max_tension = max(self.max_tension, T)
        unit = [s / max(np.linalg.norm(s), 1e-9) for s in segs]
        fa = T * unit[0]
        fb = -T * unit[-1]
        lid = np.zeros(3)
        if len(pts) > 2:
            # Sum of actual corner reactions telescopes; valid for one/two edges.
            lid = T * (-unit[0] + unit[-1])
        return T, fa, fb, lid


def hook_attach_world(pos, quat_xyzw):
    return np.asarray(pos, float) + quat_to_R(quat_xyzw) @ ATTACH_LOCAL


def hook_wrench(pos, quat_xyzw, force_w):
    """Force at the attachment point -> (force, torque about the COM) in world frame."""
    R = quat_to_R(quat_xyzw)
    r = R @ (ATTACH_LOCAL - COM_LOCAL)
    return np.asarray(force_w, float), np.cross(r, force_w)


def seat_check(pos, quat_xyzw, speed, frame: ToteFrame):
    """Return eye index (0/1) whose bar lies in the hook's seat region, else None. Evaluator geometry only."""
    R = quat_to_R(quat_xyzw)
    hook_y = R[:, 1]
    if abs(float(hook_y @ frame.bar_axis_world)) < 0.7:
        return None
    if speed > SEAT_MAX_SPEED:
        return None
    for i, e in enumerate(frame.eyes):
        e_w = frame.world(e)
        # Closest point on the finite bar centreline to the seat-region centre.
        seat_c = np.asarray(pos, float) + R @ np.array([np.mean(SEAT_X), 0.0, np.mean(SEAT_Z)])
        s = float(np.clip((seat_c - e_w) @ frame.bar_axis_world, -geo.BAR_HALF, geo.BAR_HALF))
        b = R.T @ (e_w + s * frame.bar_axis_world - np.asarray(pos, float))
        if SEAT_X[0] <= b[0] <= SEAT_X[1] and SEAT_Z[0] <= b[2] <= SEAT_Z[1] and abs(b[1]) <= geo.SHANK_HALF[1] + geo.BAR_RADIUS + 0.001:
            return i
    return None


def seated_hook_pose(frame: ToteFrame, eye: int, lid_gap: float = geo.LID_TRAVEL):
    """DIAGNOSTIC ONLY: a hook pose with eye bar `eye` in the seat, hook axis toward the lid edge."""
    e = frame.eyes[eye]
    _, _, top = frame.box(lid_gap)
    s = math.copysign(1.0, e[0])
    via = np.array([s * (frame.hx + VIA_OUT), e[1], top + VIA_OUT])
    u = frame.R @ ((via - e) / np.linalg.norm(via - e))
    x = -u
    y = frame.bar_axis_world
    y = y - (y @ x) * x
    y /= np.linalg.norm(y)
    z = np.cross(x, y)
    # Foot (-z) must point away from the lid so the bar sits between shank and foot on the outside.
    if (z @ (frame.R @ np.array([s, 0, 0]))) > 0:
        y, z = -y, -z
    R = np.column_stack([x, y, z])
    pos = frame.world(e) - R @ SEATED_BAR_LOCAL
    return pos, geo.quat_xyzw(R)


def seat_detail(pos, quat_xyzw, frame: ToteFrame, eye: int):
    """Diagnostic: eye-bar centre in hook coordinates (m) and bar/hook-lateral alignment."""
    R = quat_to_R(quat_xyzw)
    b = R.T @ (frame.world(frame.eyes[eye]) - np.asarray(pos, float))
    return dict(bar_in_hook=[round(float(v), 4) for v in b], align=round(float(abs(R[:, 1] @ frame.bar_axis_world)), 3))
