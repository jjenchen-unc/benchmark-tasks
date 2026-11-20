"""loading tool v3 geometry (numpy only; shared by the CadQuery generator and the simulation).
Ring, tongue, roof and stop as tool1 v6. New: finger seat on a thicker fin (in place of the 45 deg ledges) and a
nose-pusher tooth at the bottom of the front wall.
Tool frame (p, y, u), mm: u along the magazine tube axis U (21 deg rake), p across it in the mid-plane (forward +),
y sideways. p = 0 is the line through magazine-frame (x 0, z 110); u = x sin21 + z cos21 of the magazine frame.
All numbers are for the tool SEATED AND FULLY PRESSED on the magazine (ring bottom on the fixture tower top).
Every primitive is a convex prism; the union is the part (CAD) and the list is the collider (simulation)."""
import math

import numpy as np

RAKE = math.radians(21.0)
T, S, C = math.tan(RAKE), math.sin(RAKE), math.cos(RAKE)
C0 = 110.0 * T * S

# magazine numbers (COMMUNITY-CAD collider): outer tube section, rear wall inside, lip tips, follower top under the tongue
P_REAR, P_FRONT, W_HALF = -17.17 * C, 19.25 * C, 11.25
P_REAR_IN = -16.0 * C
Z_LIP_TOP = 114.89
Z_FOL = 108.0  # follower top at |y| = 2.5, x -15.6 .. -13 (empty magazine)
Z_FRONT_TOP = 107.38  # top of the magazine front wall

CLR = 0.4  # ring clearance per side
WALL = 2.5
DEPRESS = 6.5  # tool3: follower travel along U with the tool fully down (was 5.0 in v2.1)
ROOF_T = 3.0
P0 = -6.8  # roof front edge
Z_FOOT = 51.1  # ring bottom / standing plane = top of the fixture tower (magazine z 51.08): the tool's stop when pressed
TONGUE_HW, TONGUE_T = 2.5, 2.5
P_T0 = P_REAR_IN + 2.25  # tongue rear face: clear of the rear wall for any position of the ring
P_T1 = P_T0 + TONGUE_T
FIN_HW, FIN_L = 3.5, 80.0  # v2.1: 7 mm fin (10 mm fin blew up the grasp contact in the simulation, tool2 W1 vs W3)
LUG_W = 4.5  # finger-seat lugs stand this far out of each fin face (fingertip 3.9 thick)
RELIEF = 6.7  # front wall bottom this far (vertical) above the standing plane, except the pusher tooth
TOOTH_HW, TOOTH_T, TOOTH_LEAN = 2.0, 5.0, math.radians(15.0)  # nose-pusher tooth: half width (y), thickness at the bottom, face lean
GRIP_XZ, GRIP_LEAN = (-48.0, 80.0), math.radians(40.0)  # centre of the fingertip ends (magazine frame, tool pressed), gripper leaning back
TIP_HW, TIP_TAPER = 7.7, (13.0 - 7.7) / 12.84  # fingertip blade: half width at the end, half width gained per mm toward the palm
CH = 1.5  # lead-in chamfer on the ring's rear and front bottom edges

CLR_REAR = 1.2  # rear face: the simulated tube has collider slivers standing about 0.65 mm proud of its rear face (MEASURED in tool1)
P_RI, P_FI, Y_I = P_REAR - CLR_REAR, P_FRONT + CLR, W_HALF + CLR
P_RO, P_FO, Y_O = P_RI - WALL, P_FI + WALL, Y_I + WALL


def u_of(p, z):  # u of the point at tool p and magazine height z
    return p * T + z / C - C0


def xz(p, u):  # tool (p, u) -> magazine (x, z)
    return (p - 110.0 * S) * C + u * S, -(p - 110.0 * S) * S + u * C


def p_of(x, z):
    return (x - (z - 110.0) * T) * C


U_R = u_of(P0, Z_LIP_TOP + 1.0)  # roof underside: 1.0 mm above the lip tips at its front edge
U_T = U_R + ROOF_T
Z_TIP = Z_FOL - DEPRESS * C
uf = lambda p: u_of(p, Z_FOOT)  # noqa: E731
top = lambda p: U_T - max(0.0, p - P0)  # noqa: E731  45 deg edge ahead of the roof (prints without support)
S_PUSH = (Z_FRONT_TOP - Z_FOOT) / C  # tool this far up the tube axis: the tooth's underside is level with the front-wall top


def _prism(poly, y0, y1):
    return np.array([(p, y, u) for y in (y0, y1) for p, u in poly], float)


def blade():
    """fingertip blade of the tool arm at the nominal grasp, tool frame (p, u): end centre c, axis a (toward the tip),
    n (across, forward-up), rear and front end corners, directions of the rear and front edges toward the palm"""
    c = np.array([p_of(*GRIP_XZ), GRIP_XZ[0] * S + GRIP_XZ[1] * C])
    th = math.pi / 2 - GRIP_LEAN - RAKE
    a, n = np.array([math.cos(th), -math.sin(th)]), np.array([math.sin(th), math.cos(th)])
    k = math.atan(TIP_TAPER)
    rot = lambda v, w: np.array([v[0] * math.cos(w) - v[1] * math.sin(w), v[0] * math.sin(w) + v[1] * math.cos(w)])  # noqa: E731
    return dict(c=c, a=a, n=n, rc=c - TIP_HW * n, fc=c + TIP_HW * n, er=rot(-a, k), ef=rot(-a, -k))


def seat():
    """finger seat on each fin face, (p, u) polygons (counter-clockwise). s1: ridge under the blade's rear edge (takes the
    press and the rearward pull), s2: stop ahead of the fingertip end face, w: stop above the blade's front edge (lift).
    Every face that looks toward +u (down in the print) is 45 deg or steeper."""
    b = blade()
    r0 = b["rc"] + 6.0 * b["er"] - [0.0, 2.0]
    s1 = [r0 + [-5.0, -4.5], r0 + [5.0, -4.5], r0 + [0.5, 0.0], r0 + [-0.5, 0.0]]
    f = b["c"] + 2.5 * b["a"]
    s0, st = f - 8.0 * b["n"], f + 8.0 * b["n"]
    s2 = [s0, s0 + [3.25, -3.25], [st[0] + 3.0, s0[1] - 3.25], st + [3.0, -3.0], st]
    m = np.array([b["ef"][1], -b["ef"][0]])  # outward normal of the blade's front edge
    w0, w1 = b["fc"] + 2.0 * m + 1.0 * b["ef"], b["fc"] + 2.0 * m + 11.0 * b["ef"]
    w3 = w0 + [4.0, 5.0]
    t = (w3[0] - w1[0] + w3[1] - w1[1]) / 2
    w = [w1, w0, w0 + [4.0, 0.0], w3, w1 + [t, t]]
    return dict(s1=np.array(s1), s2=np.array(s2), w=np.array(w))


def tooth():
    """nose-pusher tooth at the bottom of the front wall, (p, u): its rear face leans TOOTH_LEAN forward at the top in the
    magazine frame, so the nose is pushed rearward and slightly up and the tooth is pushed down onto the front-wall top"""
    d = np.array([math.sin(TOOTH_LEAN - RAKE), math.cos(TOOTH_LEAN - RAKE)])  # up the face
    ft = np.array([P_FI, uf(P_FI) + RELIEF / C + 2.0])
    t = (ft[1] - uf(P_FI)) / (d[1] - T * d[0])
    a = ft - t * d
    b = np.array([a[0] + TOOTH_T, uf(a[0] + TOOTH_T)])
    return np.array([a, b, [P_FO, ft[1]], ft])


def prims():
    out = {}
    out["rear_wall"] = _prism([(P_RO, uf(P_RO)), (P_RI - CH, uf(P_RI - CH)), (P_RI, uf(P_RI) + CH * (1 + T)), (P_RI, U_T), (P_RO, U_T)], -Y_O, Y_O)
    side = [(P_RI, uf(P_RI)), (P_FI, uf(P_FI)), (P_FI, top(P_FI)), (P0, U_T), (P_RI, U_T)]
    out["side_l"] = _prism(side, Y_I, Y_O)
    out["side_r"] = _prism(side, -Y_O, -Y_I)
    r = RELIEF / C
    out["front_wall"] = _prism([(P_FI, uf(P_FI) + r + CH * (1 - T)), (P_FI + CH, uf(P_FI + CH) + r), (P_FO, uf(P_FO) + r), (P_FO, top(P_FO)), (P_FI, top(P_FI))], -Y_O, Y_O)
    out["tooth"] = _prism(tooth(), -TOOTH_HW, TOOTH_HW)
    out["roof"] = _prism([(P_RI, U_R), (P0, U_R), (P0, U_T), (P_RI, U_T)], -Y_I, Y_I)
    for k, v in seat().items():
        out[k + "_l"] = _prism(v, FIN_HW, FIN_HW + LUG_W)
        out[k + "_r"] = _prism(v, -FIN_HW - LUG_W, -FIN_HW)
    out["tongue"] = _prism([(P_T0, u_of(P_T0, Z_TIP) + 2.0), (P_T0 + 1.3, u_of(P_T0 + 1.3, Z_TIP)), (P_T1, u_of(P_T1, Z_TIP)), (P_T1, U_R), (P_T0, U_R)],
                           -TONGUE_HW, TONGUE_HW)
    xr = xz(P_RO - FIN_L, U_T)[0]  # the fin's rear edge is vertical in use
    out["fin"] = _prism([(P_RO - FIN_L + 28.0, U_T), (P_RO - FIN_L, U_T - 28.0), (p_of(xr, Z_FOOT), xr * S + Z_FOOT * C), (P_RO, uf(P_RO)), (P_RO, U_T)], -FIN_HW, FIN_HW)
    return out


def to_mag(v):  # tool-frame vertices (mm) -> magazine frame (mm), tool pressed
    v = np.asarray(v, float)
    x, z = xz(v[..., 0], v[..., 2])
    return np.stack([x, v[..., 1], z], -1)


def grip_point():
    return np.array([GRIP_XZ[0], 0.0, GRIP_XZ[1]])  # fingertip-end centre, tool pressed; the TCP is 12.84 mm above it


def ccw(v):
    e = np.roll(v, -1, 0) - v
    f = np.roll(e, -1, 0)
    return bool(np.all(e[:, 0] * f[:, 1] - e[:, 1] * f[:, 0] > 1e-9))


if __name__ == "__main__":
    for k, v in prims().items():
        m = to_mag(v)
        print(k, "tool", v.min(0).round(2), v.max(0).round(2), "mag", m.min(0).round(2), m.max(0).round(2))
    print({k: np.round(v, 3).tolist() for k, v in blade().items()})
    for k, v in {**seat(), "tooth": tooth()}.items():
        e = np.roll(v, -1, 0) - v
        nu = -e[:, 0] / np.linalg.norm(e, axis=1)  # u component of the outward normal
        print(k, np.round(v, 2).tolist(), "ccw convex", ccw(v), "max n_u %.3f" % nu.max())
    print("U_R", round(U_R, 2), "U_T", round(U_T, 2), "Z_TIP", round(Z_TIP, 2), "P_T0/1", round(P_T0, 2), round(P_T1, 2), "P_RO", round(P_RO, 2), "P_FI", round(P_FI, 2),
          "S_PUSH", round(S_PUSH, 2))
