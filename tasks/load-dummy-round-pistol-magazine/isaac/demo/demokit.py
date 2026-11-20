"""Demo-side measurements on top of the task scene (../magkit.py): the per-control-step rows the scripted demo plans
and servos from (finger contact forces on the tool and the round, tool pose in the magazine frame, follower travel).
Read-only: it adds physics/step hooks that read the simulator and never write to it. The task's scorer is magkit's.
Exposes the names the motion code (copied from the tool3 development folder) uses as `mk.*`.
STATE["hold"] = True keeps accumulating over the next control step, so one demo row can span several env steps."""
import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import numpy as np  # noqa: E402
from partkit import finger_points, on_physics, on_pre_reset, on_reset, on_step  # noqa: E402

import magkit  # noqa: E402
from magkit import (F1, F17, F_TOP, GRIP_N, K_SPRING, L_BUL, L_CASE, MAG_XY, MAG_YAW, MU_FIX, MU_MAG,  # noqa: E402,F401
                    MU_ROUND, MU_TOOL, OAL, PAD_FRICTION, PHYSICS_DT, PITCH, R_BUL, R_CASE, ROUND_G, SPRING_TARGET, TOOL_MASS,
                    TOP, U, FOL_ANCHOR, Z_FOOT)
import geom  # noqa: E402
import task_scoring as ts  # noqa: E402

SPRING = "a"
PRELOAD = 0
ROUNDS = [magkit.ROUND]
NAMES = ["mag", "round0"]
Z_TIP = H = Z_L = ts.Z_TIP
Z_RET0, X_LIP0, X_LIP1, X_RAMP1, X_FRONT, W_IN = ts.Z_RET0, ts.X_LIP0, ts.X_LIP1, ts.X_RAMP1, ts.X_FRONT, ts.W_IN
x_rear, z_ret = ts.x_rear, ts.z_ret
X_LF, X_REAR = X_RAMP1, x_rear(0.110)
LIP_GAP, LIP_LEN, FRONT_DROP, SIDE_DROP = 0.0085, X_RAMP1 - x_rear(Z_TIP), Z_TIP - magkit.META["z_front_top"] * 0.001, 0.0

STATE: dict = {"view": None, "hold": False}


def _fresh_sub():
    return dict(n=0, spring_max=0.0, grip_l_max=0.0, grip_r_max=0.0, vmax=0.0)


def _fresh():
    STATE.update(sim_t=0.0, steps=[], sub=_fresh_sub(), hold=False)


_fresh()


def follower_x(s):
    p, r = s["mag"][:2]
    return float(U @ (r.T @ (s["follower"][0] - p) - FOL_ANCHOR))


def spring_force(x):
    return K_SPRING * (SPRING_TARGET - x)


@on_pre_reset
def _pre():
    _fresh()


@on_reset
def _after(environment):
    STATE["view"] = magkit.STATE["view"]


@on_physics
def _substep(environment, dt):
    view = STATE["view"]
    if view is None or magkit.STATE["start"] is None:
        return
    STATE["sim_t"] += dt
    s = view.state()
    f = view.grip_forces(dt)
    sub = STATE["sub"]
    sub["n"] += 1
    sub["spring_max"] = max(sub["spring_max"], spring_force(follower_x(s)))
    gi = {g: k for k, g in enumerate(view.gnames)}
    mi, r0, fo, fx, ti = (view.sensors.index(n) for n in ("mag", "round0", "follower", "fixture", "tool"))
    mr = s["mag"][1]
    for side in "lr":
        g = max(float(np.linalg.norm(f[:, gi[f"{side}_{t}"]].sum(0))) for t in ("tip_left", "tip_right"))
        sub[f"grip_{side}_max"] = max(sub[f"grip_{side}_max"], g)
        ks = [gi[k] for k in gi if k.startswith(side + "_")]
        fr = mr.T @ f[r0, ks].sum(0)
        fm = float(np.linalg.norm(f[mi, ks].sum(0) + f[fo, ks].sum(0)))
        tip = np.array([np.linalg.norm(f[:, gi[f"{side}_{t}"]], axis=-1).sum() for t in ("tip_left", "tip_right")])
        a = sub.setdefault("arm_" + side, dict(fr=np.zeros(3), frmax=0.0, fm=0.0, fmmax=0.0, ffx=0.0, tip=np.zeros(2), tipmax=np.zeros(2), ft=np.zeros(3)))
        a["ft"] += mr.T @ f[ti, ks].sum(0)
        a["fr"] += fr
        a["frmax"] = max(a["frmax"], float(np.linalg.norm(fr)))
        a["fm"] += fm
        a["fmmax"] = max(a["fmmax"], fm)
        a["ffx"] = max(a["ffx"], float(np.linalg.norm(f[fx, ks].sum(0))))
        a["tip"] += tip
        a["tipmax"] = np.maximum(a["tipmax"], tip)
    sub["vmax"] = max(sub["vmax"], float(np.linalg.norm(s["round0"][2])))
    ft = mr.T @ view.tool_forces(dt).T  # columns: magazine body, follower, round
    t = sub.setdefault("tool", dict(f=np.zeros((3, 3)), fmax=0.0))
    t["f"] += ft
    t["fmax"] = max(t["fmax"], float(np.linalg.norm(ft[:, 0] + ft[:, 1])))


@on_step
def _control(environment):
    view = STATE["view"]
    if view is None or magkit.STATE["start"] is None or STATE["hold"]:
        return
    s = view.state()
    sub = STATE["sub"]
    STATE["sub"] = _fresh_sub()
    mp, mr = s["mag"][:2]
    x = follower_x(s)
    n = max(1, sub["n"])
    row = dict(t=round(STATE["sim_t"], 3), follower_mm=round(x * 1000, 2), spring_N=round(spring_force(x), 2),
               spring_max_N=round(sub["spring_max"], 2), grip_l_max_N=round(sub["grip_l_max"], 2), grip_r_max_N=round(sub["grip_r_max"], 2),
               round_vmax=round(sub["vmax"], 4), mag_tilt_deg=round(math.degrees(math.acos(min(1.0, mr[2, 2]))), 3),
               mag_xy_mm=np.round((mp[:2] - MAG_XY) * 1000, 3).tolist(), official_score=magkit.STATE["score"],
               official_steps=magkit.STATE["steps"])
    yaw0 = np.array([[math.cos(MAG_YAW), math.sin(MAG_YAW)], [-math.sin(MAG_YAW), math.cos(MAG_YAW)]])
    row["mag_in_fix_mm"] = np.round(np.r_[yaw0 @ (mp[:2] - MAG_XY), mp[2] - TOP - geom.MAG_Z0] * 1000, 3).tolist()
    for side in "lr":
        a = sub.get("arm_" + side)
        if a:
            row.update({f"f{side}_rnd_N": np.round(a["fr"] / n, 2).tolist(), f"f{side}_rnd_max_N": round(a["frmax"], 2),
                        f"f{side}_mag_N": round(a["fm"] / n, 2), f"f{side}_mag_max_N": round(a["fmmax"], 2),
                        f"f{side}_fix_max_N": round(a["ffx"], 2), f"f{side}_tip_N": np.round(a["tip"] / n, 2).tolist(),
                        f"f{side}_tip_max_N": np.round(a["tipmax"], 2).tolist(), f"f{side}_tool_N": np.round(a["ft"] / n, 2).tolist()})
    t = sub.get("tool")
    if t:
        row.update(tool_mag_N=np.round(t["f"][:, 0] / n, 2).tolist(), tool_fol_N=np.round(t["f"][:, 1] / n, 2).tolist(),
                   tool_rnd_N=np.round(t["f"][:, 2] / n, 2).tolist(), tool_max_N=round(t["fmax"], 2))
    tp, tr = s["tool"][:2]
    rel, rr = mr.T @ (tp - mp), mr.T @ tr
    d = rel - [0.0, 0.0, Z_FOOT]
    row.update(tool_mm=np.round(d * 1000, 2).tolist(), tool_u_mm=round(float(U @ d) * 1000, 2),
               tool_ypr_deg=np.round(np.degrees([math.atan2(rr[1, 0], rr[0, 0]), -math.asin(float(np.clip(rr[2, 0], -1, 1))),
                                                 math.atan2(rr[2, 1], rr[2, 2])]), 2).tolist())
    STATE["steps"].append(row)


def round_in_mag(s, name):
    mp, mr = s["mag"][:2]
    p, r = s[name][:2]
    return mr.T @ (p - mp), mr.T @ r[:, 0]


def round_status(s, name):
    c, a = round_in_mag(s, name)
    st = ts.round_state(tuple(c), tuple(a))
    return dict(inside=bool(st["inside"]), loaded=bool(st["loaded"]), retained=bool(st["retained"]), z_mm=round(c[2] * 1000, 2),
                base_gap_mm=round(st["gap"] * 1000, 2), axis=np.round(a, 3).tolist(), y_mm=round(c[1] * 1000, 2))


def score(s):
    return dict(score=magkit.STATE["score"], loaded=round_status(s, "round0")["loaded"])


def hands_clear(environment):
    s = STATE["view"].state()
    pts = np.array(finger_points(environment))
    return round(float(np.linalg.norm(pts - s["round0"][0], axis=1).min()), 4)
