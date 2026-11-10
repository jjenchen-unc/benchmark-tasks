"""D1R-tall fixture geometry in the yam_arena station frame. Pure Python, no Isaac imports.

Every number below is copied from the frozen native fixture
``input-d1-yam/fixture/scene/D1_straps/benches/D1_straps.py`` (sha256 d91d3fa0...)
and the YAM ``compact_yam_rear`` layout in ``input-d1-yam/adapter/d1r.py``.
Only the placement transform is new (see ``to_arena``); sizes, masses, the lid
spring law and the strap/hook ratings are unchanged.
"""
from __future__ import annotations

import math

import numpy as np

# -- native constants (D1_straps.py) -----------------------------------------------------
NATIVE_TABLE_Z = 0.80
G = 9.81
L0 = 0.45
HOOK_LEN = 0.075
HANDLE = (0.025, 0.020, 0.012)  # hook handle half sizes
SHANK_POS, SHANK_HALF = (0.0425, 0.0, 0.0), (0.0175, 0.0035, 0.004)
HOOK_MASS = 0.05
HANDLE_MASS, SHANK_MASS = HOOK_MASS - 0.01, 0.01
THROAT = (0.06, 0.0, 0.0)
STANDOFF = 0.08
LID_T = 0.006  # lid half thickness
LID_TRAVEL = 0.02
LID_MASS = 0.5
LID_DAMPING = 8.0  # N s/m
WALL_T = 0.004  # wall half thickness
WALL_MASS = 0.3
BAR_HALF = 0.013  # eye bar centreline half length
BAR_RADIUS = 0.003
TRAY_C_NATIVE = (0.025, -0.26 + 0.30)  # compact_yam(+_rear) tray translation +0.30 y
TRAY_H, TRAY_T = (0.21, 0.125), 0.003
STRAP_Y_NATIVE = tuple(y + 0.30 for y in (-0.155, -0.225, -0.295, -0.365))
HOLDER_TOP_ABOVE_TABLE = 2 * TRAY_T + 0.037
SLOT_W, SLOT_T = 0.014, 0.006
RD_CARD_SCALE, RD_CARD_W, RD_CARD_T, RD_CARD_DY = 0.25, 0.018, 0.0008, 0.022
RELAXED_LENGTHS = (0.36, 0.52, 0.58, 0.46)  # seed-42 order, strap_1..strap_4 (task.json setup)
COLOURS = ((0.85, 0.18, 0.14), (0.15, 0.40, 0.85), (0.95, 0.75, 0.10), (0.20, 0.62, 0.28))
COLOUR_NAMES = ("red", "blue", "yellow", "green")
STRAP_EA = 67.5  # N per unit strain
HOOK_RATING_N = 28.0
F_UP_TALL = 30.0  # contents push, shut
F_UP_LOW = 30.0
STRETCH = {"tote_low": 0.08, "tote_tall": 0.15}


def _bin(name, c, h, eye_z):
    leg = (L0 + STRETCH[name] - 2 * h[0]) / 2 + HOOK_LEN
    dz = math.sqrt(leg ** 2 - STANDOFF ** 2)
    return dict(c=c, h=h, eye_z=eye_z, top=eye_z + dz)


NATIVE_BINS = {
    # compact_yam relocated the unused low tote to (0.55, 0.50) native; tall tote unchanged.
    "tote_tall": _bin("tote_tall", (0.02, 0.32), (0.15, 0.07), 0.87),
    "tote_low": _bin("tote_low", (0.55, 0.50), (0.13, 0.07), 0.85),
}


# The unused low tote's transformed compact_yam position (0.68, -0.53) would hang off the station table
# (|y| <= 0.55), so it is placed long axis along Arena x at (0.66, -0.42). It is a distractor only.
LOW_TOTE_ARENA_OVERRIDE = {"tote_low": ((0.66, -0.42, 0.7475), 0.0)}  # z = station TABLE_TOP


def lid_spring(f_up: float) -> tuple[float, float]:
    """Native preloaded joint spring: (stiffness N/m, reference position m)."""
    kc = f_up / (2 * LID_TRAVEL)
    return kc, (f_up + LID_MASS * G) / kc


# -- placement -----------------------------------------------------------------------------
TABLE_TOP = 0.7475  # yam_arena.task_file.TABLE_TOP["station"]
NATIVE_BASE_MID = (0.02, -0.03)  # compact_yam_rear YAM base midpoint (native)
SHIFT_X = 0.15  # extra reach clearance between folded arms and the tray (new layout choice)
YAW = -math.pi / 2  # native +Y (YAM facing) -> Arena +x


def to_arena(p) -> np.ndarray:
    """Native fixture point -> Arena station frame (rotation -90 deg about z, table-height preserving)."""
    x, y, z = (float(v) for v in p)
    rx, ry = x - NATIVE_BASE_MID[0], y - NATIVE_BASE_MID[1]
    return np.array([ry + SHIFT_X, -rx, z - NATIVE_TABLE_Z + TABLE_TOP])


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1.0, 0], [-s, 0, c]])


def quat_xyzw(R: np.ndarray) -> tuple[float, float, float, float]:
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    if w > 1e-6:
        x, y, z = (R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w)
    else:  # 180-degree fallback
        x = math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
        y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 1])
        z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[0, 2])
    return (float(x), float(y), float(z), float(w))


def tote(name: str) -> dict:
    """Tote parts in the tote's own frame (origin on the table under its centre, axes = native axes)."""
    b = NATIVE_BINS[name]
    hx, hy = b["h"]
    rim = b["top"] - LID_T
    H = rim - NATIVE_TABLE_Z
    wt = WALL_T
    walls = [
        ("bottom", (0, 0, wt), (hx, hy, wt)),
        ("wall_xm", (-hx + wt, 0, H / 2), (wt, hy, H / 2)),
        ("wall_xp", (hx - wt, 0, H / 2), (wt, hy, H / 2)),
        ("wall_ym", (0, -hy + wt, H / 2), (hx, wt, H / 2)),
        ("wall_yp", (0, hy - wt, H / 2), (hx, wt, H / 2)),
    ]
    ez = b["eye_z"] - NATIVE_TABLE_Z
    brackets, eyes = [], []
    for side, s in enumerate((-1, 1)):
        ex = s * (hx + STANDOFF)
        brackets.append((f"bracket_{side}", (s * (hx + STANDOFF / 2 - 0.004), 0, ez - 0.014), (STANDOFF / 2 - 0.002, 0.012, 0.004)))
        eyes.append((f"eye_{side}", (ex, 0.0, ez)))  # D1R draws the bar AT the eye centre, along native y
    kc, qref = lid_spring(F_UP_TALL if name == "tote_tall" else F_UP_LOW)
    lid_shut_z = b["top"] - LID_T - NATIVE_TABLE_Z
    if name in LOW_TOTE_ARENA_OVERRIDE:
        origin, yaw = np.array(LOW_TOTE_ARENA_OVERRIDE[name][0]), LOW_TOTE_ARENA_OVERRIDE[name][1]
    else:
        origin, yaw = to_arena((b["c"][0], b["c"][1], NATIVE_TABLE_Z)), YAW
    return dict(name=name, walls=walls, brackets=brackets, eyes=eyes, lid_half=(hx, hy, LID_T),
                lid_shut_z=lid_shut_z, kc=kc, qref=qref, top=b["top"], c=b["c"], h=b["h"],
                origin=origin, yaw=yaw)


def hooks() -> list[dict]:
    """The eight hooks standing tip-down in their tray holders (native build(), D1R-tall, no pre-hooked strap)."""
    out = []
    hook_x = (TRAY_C_NATIVE[0] - 0.12, TRAY_C_NATIVE[0] + 0.12)
    R_native = rot_y(math.pi / 2)  # native quat (.7071, 0, .7071, 0): throat down
    R = rot_z(YAW) @ R_native
    for i in range(4):
        for e, hx_ in (("a", hook_x[0]), ("b", hook_x[1])):
            p = (hx_, STRAP_Y_NATIVE[i], NATIVE_TABLE_Z + HOLDER_TOP_ABOVE_TABLE + HANDLE[0])
            out.append(dict(name=f"strap_{i + 1}_{e}", strap=i, end=e, pos=tuple(to_arena(p)), R=R,
                            quat_xyzw=quat_xyzw(R), colour=COLOURS[i], native_pos=p))
    return out


def tray() -> dict:
    """Tray plate, slotted holders and tag cards in the tray frame (native axes, origin at plate centre)."""
    hook_x = (TRAY_C_NATIVE[0] - 0.12, TRAY_C_NATIVE[0] + 0.12)
    hh = (HOLDER_TOP_ABOVE_TABLE - 2 * TRAY_T) / 2
    holders, cards = [], []
    for i in range(4):
        y = STRAP_Y_NATIVE[i] - TRAY_C_NATIVE[1]
        for e, hx_ in (("a", hook_x[0]), ("b", hook_x[1])):
            x = hx_ - TRAY_C_NATIVE[0]
            for nm, dx, dy in (("xm", -1, 0), ("xp", 1, 0), ("ym", 0, -1), ("yp", 0, 1)):
                pos = (x + dx * (SLOT_W / 2 + SLOT_T / 2), y + dy * (SLOT_W / 2 + SLOT_T / 2), TRAY_T + hh)
                size = (SLOT_T / 2 if dx else SLOT_W / 2 + SLOT_T, SLOT_T / 2 if dy else SLOT_W / 2 + SLOT_T, hh)
                holders.append((f"holder_{i + 1}{e}_{nm}", pos, size))
        cards.append((f"tag_card_{i + 1}", (0.0, y + RD_CARD_DY, TRAY_T + RD_CARD_T),
                      (RD_CARD_SCALE * RELAXED_LENGTHS[i] / 2, RD_CARD_W / 2, RD_CARD_T)))
    return dict(plate_half=(TRAY_H[0], TRAY_H[1], TRAY_T), holders=holders, cards=cards,
                origin=to_arena((TRAY_C_NATIVE[0], TRAY_C_NATIVE[1], NATIVE_TABLE_Z + TRAY_T)))


def eye_world(name: str, side: int) -> np.ndarray:
    t = tote(name)
    return t["origin"] + rot_z(t["yaw"]) @ np.array(t["eyes"][side][1])
