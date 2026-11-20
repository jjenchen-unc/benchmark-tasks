"""tool1 planned path (numpy only; shared by ikcheck.py and run_tool.py). Magazine frame as plan.py.
P (tool arm): vertical gripper, pinches the tool's rear fin across mag y. N (round arm): vertical gripper, pinches the
bullet across mag y; the round is picked with the gripper leaning TOOL_TILT forward, so with the gripper vertical the
round is TOOL_TILT nose-up; it is then pushed along its own axis under the lips (follower held down by the tool)."""
import json
import math
import os
from pathlib import Path

import numpy as np

import plan as pl

E = os.environ.get
MM = 0.001
M = json.loads((Path(__file__).resolve().parents[1] / "tool" / "tool_hulls.json").read_text())
Z_FOOT = M["z_foot"] * MM
RAKE = math.radians(21.0)
U = np.array([math.sin(RAKE), 0.0, math.cos(RAKE)])
TOOL_O = np.array([0.0, 0.0, Z_FOOT])  # tool origin in the magazine frame, tool seated and pressed
G_TIP = np.array([float(v) for v in E("TOOL_GRIP", "-0.048,0,0.080").split(",")])  # centre of P's fingertip ends, magazine frame, tool pressed
P_LEAN = math.radians(float(E("TOOL_P_LEAN", "40")))  # P gripper leaning back (top toward mag -x, away from N) about mag y
R_OVER = float(E("TOOL_R_OVER", "0.062"))  # tool this far up the tube axis before it is put on
TILT = math.radians(float(E("TOOL_TILT", "11")))
CLEAR = float(E("TOOL_CLEAR", "0.00075"))  # round underside above the front-wall top on the entry line
X_IN0, X_IN1 = float(E("TOOL_X_IN0", "0.014")), float(E("TOOL_X_IN1", "-0.009"))
X_TONGUE = lambda z: (-10.19 / math.cos(RAKE) + (z * 1000 - 110) * math.tan(RAKE)) * MM  # noqa: E731  tongue front face
P_GRIP_OPEN = 0.030 / 0.0939
PARK = np.array([0.0, 0.0, 0.03])
S_PUSH = M["s_push"] * MM  # tool this far up the tube axis for the nose push (tooth underside level with the front-wall top)
PULL_LEAD = float(E("TOOL_PULL_LEAD", "0.06"))  # the pull command may run this far behind the tool


def zb_line(xb, tilt=None):
    t = TILT if tilt is None else tilt
    return pl.Z_FRONT + pl.R_CASE / math.cos(t) + CLEAR - (pl.X_FRONT - xb) * math.tan(t)


def g_rot(psi):
    return pl.n_rot(psi, -P_LEAN)


def p_tcp_tool(psi):  # P TCP relative to the tool origin (tool frame), nominal grasp
    return G_TIP - TOOL_O - g_rot(psi) @ np.array([0.0, 0.0, pl.TIP_Z])


def p_poses(mp, mr, tp, tr, psi):
    """P path in world: hover, grasp on the table, lift, over the magazine, seated, park"""
    g = p_tcp_tool(psi)
    rt, rm = tr @ g_rot(psi), mr @ g_rot(psi)
    over = mp + mr @ (TOOL_O + U * R_OVER + g)
    zc = over[2] + 0.004
    grasp = tp + tr @ g
    return dict(hover=(grasp - rt[:, 2] * 0.07, rt), grasp=(grasp, rt), lift=(np.array([grasp[0], grasp[1], zc]), rt),
                above=(np.array([over[0], over[1], zc]), rm), over=(over, rm), seat=(mp + mr @ (TOOL_O + g), rm),
                low=(mp + mr @ (TOOL_O - U * 0.004 + g), rm), push0=(mp + mr @ (TOOL_O + U * S_PUSH + g), rm),
                push1=(mp + mr @ (TOOL_O + U * S_PUSH + [-PULL_LEAD, 0, 0] + g), rm), park=(mp + mr @ (TOOL_O + U * R_OVER + g) + [0, 0.10, 0.0] + PARK, rm))


def n_in(xb, psi, tilt=None, lean=0.0, y=0.0):
    t = TILT if tilt is None else tilt
    return pl.n_tcp((xb, zb_line(xb, t)), t, y, psi=psi, beta=lean), pl.n_rot(psi, lean)


def n_poses(mp, mr, rp, ryaw, top, psi):
    """N path in world: pick (leaning TILT forward), lift, wait, entry line start and end, nose push start / tongue / end, away"""
    W = lambda p: mp + mr @ np.asarray(p)  # noqa: E731
    rf = pl.rot_z(ryaw)
    o = np.array([rp[0], rp[1], top]) - rf @ np.array([pl.OAL / 2, 0, 0])
    pick = o + rf @ pl.n_tcp((0.0, pl.R_CASE + 0.0003), 0.0, psi=psi, beta=TILT)
    rpk = rf @ pl.n_rot(psi, TILT)
    rm = mr @ pl.n_rot(psi)
    a, _ = n_in(X_IN0, psi)
    b, _ = n_in(X_IN1, psi)
    t0 = pl.nose_push_tcp(X_IN1 + pl.OAL + 0.0015, 0.1115, psi=psi)
    t1 = pl.nose_push_tcp(-0.0135 + pl.OAL, 0.1085, psi=psi)
    t2 = pl.nose_push_tcp(pl.X_END + pl.OAL - 0.0035, 0.1100, psi=psi)
    return dict(hover=(pick + [0, 0, 0.06], rpk), pick=(pick, rpk), lift=(W(a + [0.05, 0, 0.04]), rm), start=(W(a), rm), end=(W(b), rm),
                back=(W(b + [0.008, 0, 0.004]), rm), nose0=(W(t0), rm), nose1=(W(t1), rm), nose2=(W(t2), rm), away=(W(t2 + [0.055, 0, 0.025]), rm))
