"""Scene, measurements and score hook: an empty double-stack 9 mm pistol magazine standing in a bench fixture, one
inert 9x19 dummy round lying on the table, and a 3D-printed loading tool standing on the table. No firearm, no live
ammunition.

Physics: magazine body and feed lips as convex-hull colliders from a community CAD model (lips clipped to an 8.5 mm
gap); follower on a prismatic joint with a linear spring drive (preloaded against its top stop); the round and the tool
are free rigid bodies; the fixture is static. Nothing writes object poses during a trial.
Development-only rigs removed; the demo settings are fixed. Scoring: task_scoring.py.
"""
from __future__ import annotations

import json
import math
import os
import pathlib
import sys

os.environ.setdefault("MAG_DEPEN", "10")  # partkit: max depenetration velocity 10 m/s
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import numpy as np  # noqa: E402
from partkit import (Hinge, env, finger_points, on_physics, on_pre_reset, on_reset, on_step, part,  # noqa: E402
                     set_grip_force, set_pad_friction, set_physics_dt, set_targets)
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import geom  # noqa: E402
import task_scoring as ts  # noqa: E402

PHYSICS_DT = 1.0 / 480
set_physics_dt(PHYSICS_DT)
GRIP_N = 40.0
set_grip_force(GRIP_N)
PAD_FRICTION = (0.8, 0.7)  # ASSUMED rubber pad
set_pad_friction(PAD_FRICTION)

F1, F17 = 10.0, 26.0  # ASSUMED spring force at round 1 / round 17, N (setting a)
ROUND_G = 4.5  # aluminium snap cap
MU_ROUND, MU_MAG, MU_FIX, MU_TOOL = 0.4, 0.35, 0.35, 0.35  # ASSUMED

R_CASE, L_CASE, R_BUL, L_BUL, OAL = ts.R_CASE, ts.L_CASE, ts.R_BUL, ts.L_BUL, ts.OAL
CADDIR = pathlib.Path(__file__).resolve().parent / "collider"
META = ts.META
MM = 0.001
RAKE = math.radians(META["rake_deg"])
U = np.array([math.sin(RAKE), 0.0, math.cos(RAKE)])  # tube axis (up the tube), magazine frame
H = ts.Z_TIP
MAG_MASS = 0.075  # ASSUMED
PITCH = math.sqrt((2 * R_CASE) ** 2 - (0.018 - 2 * R_CASE) ** 2)
K_SPRING = (F17 - F1) / (16 * PITCH)  # N/m along the tube axis (170 N/m)
F_TOP = F1 - K_SPRING * 2 * R_CASE  # magazine empty, follower on its stop (8.3 N)
SPRING_TARGET = F_TOP / K_SPRING
FOL_LOW = -0.10

# fixed layout (station frame: x forward from the line between the arm bases, y left, z up; m, deg)
TOP = TABLE_TOP["station"]
MAG_XY, MAG_YAW = np.array([0.20, 0.00]), math.radians(-60.0)
ROUND_XY, ROUND_YAW = np.array([0.32, -0.26]), math.radians(-90.0)
TOOL_XY, TOOL_YAW = np.array([0.26, 0.18]), MAG_YAW
# test-only start-pose overrides (the demo's start-pose checks, README Known limits); unset in the official run
if os.environ.get("MAG_START_ROUND"):  # "x,y,yaw_deg"
    _v = [float(v) for v in os.environ["MAG_START_ROUND"].split(",")]
    ROUND_XY, ROUND_YAW = np.array(_v[:2]), math.radians(_v[2])
if os.environ.get("MAG_START_TOOL"):  # "x,y"
    TOOL_XY = np.array([float(v) for v in os.environ["MAG_START_TOOL"].split(",")])
MAG_Z = TOP + geom.MAG_Z0 + 0.0003

BLACK, DARK, BRASS, COPPER, GREY = (0.06, 0.06, 0.07), (0.16, 0.16, 0.18), (0.78, 0.62, 0.30), (0.72, 0.42, 0.25), (0.4, 0.4, 0.42)
ORANGE = (0.95, 0.5, 0.1)


def _hulls(name, color):
    d = np.load(CADDIR / name)
    out, k = [], 0
    for n in d["n"]:
        out.append((tuple(map(tuple, d["verts"][k:k + n].astype(float))), color))
        k += n
    return tuple(out)


PLATE = (((40.0 - META["XC"]) * MM, 0.0, 4.18 * MM), (40.4 * MM, 28.5 * MM, 6.2 * MM), BLACK)  # floor plate
FOL_ANCHOR = np.array([(META["fol_x0"] + META["fol_x1"]) / 2 * MM, 0.0, META["fol_top_low"] * MM])
FOLLOWER = Hinge("follower", anchor=tuple(FOL_ANCHOR), axis=tuple(U), linear=True, lower=FOL_LOW, upper=0.0, start=0.0,
                 rate=K_SPRING, damping=1.0, mass=0.005, target=SPRING_TARGET, hulls=_hulls("follower_hulls.npz", GREY))
MAG = part("mag", (MAG_XY[0], MAG_XY[1], MAG_Z), boxes=(PLATE,), hulls=_hulls("body_hulls.npz", DARK) + (geom.MAG_LOW,),
           mass=MAG_MASS, color=DARK, yaw=MAG_YAW, friction=MU_MAG, hinges=(FOLLOWER,), contact_offset=0.0005)
FIXTURE = part("fixture", (MAG_XY[0], MAG_XY[1], TOP + geom.MAG_Z0), yaw=MAG_YAW, friction=MU_FIX, boxes=geom.FIX_BOXES,
               hulls=geom.FIX_HULLS, cylinders=(geom.FIX_PIN,), mass=None, color=geom.GREY)


def _round_hulls(n=24):
    x0 = -OAL / 2
    t = np.linspace(0, 2 * math.pi, n, endpoint=False)
    ring = lambda x, r: [(x, r * math.cos(a), r * math.sin(a)) for a in t]  # noqa: E731
    case = ring(x0, R_CASE) + ring(x0 + L_CASE, R_CASE)
    xb = x0 + L_CASE + L_BUL
    nose = [(xb + R_BUL * math.sin(p), R_BUL * math.cos(p) * math.cos(a), R_BUL * math.cos(p) * math.sin(a))
            for p in np.linspace(0.3, 1.45, 4) for a in t[::2]] + [(xb + R_BUL, 0.0, 0.0)]
    bullet = ring(x0 + L_CASE, R_BUL) + ring(xb, R_BUL) + nose
    return ((tuple(case), BRASS), (tuple(map(tuple, bullet)), COPPER))


ROUND = part("round0", (ROUND_XY[0], ROUND_XY[1], TOP + R_CASE + 0.0003), yaw=ROUND_YAW, hulls=_round_hulls(),
             mass=ROUND_G / 1000, color=BRASS, friction=MU_ROUND)

TMETA = json.loads((pathlib.Path(__file__).resolve().parent / "tool" / "tool_hulls.json").read_text())
Z_FOOT = TMETA["z_foot"] * MM
TOOL_O = np.array([0.0, 0.0, Z_FOOT])  # tool origin in the magazine frame, seated and pressed
TOOL_MASS = TMETA["mass_g"] / 1000
TOOL = part("tool", (TOOL_XY[0], TOOL_XY[1], TOP + 0.0005), yaw=TOOL_YAW,
            hulls=tuple((tuple(map(tuple, np.array(v) * MM - [0, 0, Z_FOOT])), ORANGE) for v in TMETA["hulls"].values()),
            mass=TOOL_MASS, color=ORANGE, friction=MU_TOOL)

OBJECTS = [MAG, FIXTURE, TOOL, ROUND]
FREE = ("mag", "tool", "round0")


def quat_to_matrix(q):  # xyzw
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _np(x):
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x, dtype=np.float64)


GRIP_LINKS = ("tip_left", "tip_right", "gripper")


class View:
    def __init__(self, environment):
        import omni.physics.tensors as tensors
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdUtils
        base = environment.unwrapped
        stage = get_current_stage()
        found = {}
        for prim in stage.Traverse():
            parts = str(prim.GetPath()).split("/")
            if "env_0" not in parts or len(parts) < 2:
                continue
            if parts[-1] == "body" and parts[-2] in ("mag", "round0", "fixture", "tool"):
                found.setdefault(parts[-2], str(prim.GetPath()))
            if parts[-1] == "follower" and parts[-2] == "mag":
                found.setdefault("follower", str(prim.GetPath()))
        self.grip = {}
        for arm in ("left_robot", "right_robot"):
            links = base.scene[arm].root_view.link_paths
            links = links[0] if links and isinstance(links[0], (list, tuple)) else links
            for p in links:
                if p.rsplit("/", 1)[1] in GRIP_LINKS:
                    self.grip[f"{arm[0]}_{p.rsplit('/', 1)[1]}"] = p
        self.names = ["mag", "round0", "follower", "tool"]
        sid = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
        self.sim = tensors.create_simulation_view("torch", stage_id=sid)
        self.sim.set_subspace_roots("/")
        self.view = self.sim.create_rigid_body_view([found[n] for n in self.names])
        order = list(self.view.prim_paths)
        self.idx = {n: order.index(found[n]) for n in self.names}
        self.origin = _np(base.scene.env_origins[0])
        self.gnames = sorted(self.grip)
        self.sensors = ["mag", "round0", "follower", "fixture", "tool"]
        self.contacts = self.sim.create_rigid_contact_view([found[n] for n in self.sensors],
                                                           filter_patterns=[[self.grip[g] for g in self.gnames]] * len(self.sensors),
                                                           max_contact_data_count=2048)
        self.tool_contacts = self.sim.create_rigid_contact_view([found["tool"]], filter_patterns=[[found["mag"], found["follower"], found["round0"]]],
                                                                max_contact_data_count=1024)

    def state(self):
        tf = _np(self.view.get_transforms())
        v = _np(self.view.get_velocities())
        return {n: (tf[i, :3] - self.origin, quat_to_matrix(tf[i, 3:7]), v[i, :3], v[i, 3:6]) for n, i in self.idx.items()}

    def grip_forces(self, dt):
        return _np(self.contacts.get_contact_force_matrix(dt=dt))  # (sensors, gripper links, 3)

    def tool_forces(self, dt):
        return _np(self.tool_contacts.get_contact_force_matrix(dt=dt))[0]  # (mag body, follower, round, 3)


STATE: dict = {"view": None}


def _fresh_sub():
    return dict(through_gap=False, tg_axis_n=0, tg_centre_n=0, mag_shift=0.0, mag_tilt=0.0, fell=False, tool_n=0.0, tip_max={"l": np.zeros(2), "r": np.zeros(2)})


def _fresh():
    return dict(view=STATE.get("view"), sim_t=0.0, start=None, sub=_fresh_sub(), scorer=ts.LoadScorer(), score=0.0, steps=0,
                first_full=None, veto_event=None, tg={"axis": [], "centre": []}, finger_peak={"l": np.zeros(2), "r": np.zeros(2)}, trace=[])


STATE.update(_fresh())


def restore_authored():
    """Environment reset only: every free part back to its authored root pose (and the follower to its stop) with zero
    velocity, before yam_arena's own reset and settle (as the finished task's livekit.restore_authored)."""
    environment = env()
    if environment is None:
        return
    import torch
    base = environment.unwrapped
    origin = base.scene.env_origins[:1]
    for spec in OBJECTS:
        if spec.mass is None and not spec.hinges:
            continue
        a = base.scene[spec.name]
        root = torch.as_tensor(getattr(a.data.default_root_state, "torch", a.data.default_root_state)).clone()
        root[:, :3] += origin
        a.write_root_pose_to_sim(root[:, :7])
        a.write_root_velocity_to_sim(torch.zeros_like(root[:, 7:]))
        if spec.hinges:
            jp = torch.as_tensor(getattr(a.data.default_joint_pos, "torch", a.data.default_joint_pos)).clone()
            a.write_joint_state_to_sim(jp, torch.zeros_like(jp))


def mag_frame(s, name):
    mp, mr = s["mag"][:2]
    p, r = s[name][:2]
    return mr.T @ (p - mp), mr.T @ r


def mag_moved(s, start):
    (p0, r0), (p, r) = start, s["mag"][:2]
    shift = float(np.linalg.norm(p - p0))
    tilt = math.degrees(math.acos(float(np.clip((np.trace(r0.T @ r) - 1) / 2, -1, 1))))
    return shift, tilt


@on_pre_reset
def _pre():
    restore_authored()
    STATE.update(_fresh())


@on_reset
def _after(environment):
    set_targets(environment, "mag", {"follower": SPRING_TARGET})
    STATE["view"] = STATE["view"] or View(environment)
    s = STATE["view"].state()
    STATE["start"] = s["mag"][:2]
    STATE["sim_t"] = 0.0
    base = environment.unwrapped
    STATE["sim_cfg"] = dict(physics_dt=float(base.physics_dt), decimation=int(getattr(base.cfg, "decimation", 1)))
    print("RESET_STATE", json.dumps({n: [*np.round(s[n][0] * 1000, 4).tolist(), *np.round(s[n][1].ravel(), 6).tolist()] for n in s}), flush=True)


@on_physics
def _substep(environment, dt):
    view = STATE["view"]
    if view is None or STATE["start"] is None:
        return
    STATE["sim_t"] += dt
    set_targets(environment, "mag", {"follower": SPRING_TARGET})
    s = view.state()
    sub = STATE["sub"]
    c, r = mag_frame(s, "round0")
    tg = ts.through_gap_now(tuple(c), tuple(r[:, 0]))
    sub["through_gap"] |= tg
    centre = c[0] - r[0, 0] * OAL / 2 < ts.X_LIP1 and c[2] > ts.Z_RET0 + ts.THROUGH_TOL  # diagnostic only: older centre form of the lip-gap test
    sub["tg_axis_n"] += tg
    sub["tg_centre_n"] += bool(centre)
    shift, tilt = mag_moved(s, STATE["start"])
    sub["mag_shift"], sub["mag_tilt"] = max(sub["mag_shift"], shift), max(sub["mag_tilt"], tilt)
    sub["fell"] |= any(s[n][0][2] < TOP - ts.FALL for n in FREE)
    sub["tool_n"] = max(sub["tool_n"], float(np.linalg.norm(view.tool_forces(dt), axis=-1).max()))
    f = view.grip_forces(dt)
    gi = {g: k for k, g in enumerate(view.gnames)}
    for side in "lr":  # each fingertip's total contact force (reported against the 40 N / 30 N flags, not scored)
        tip = np.array([np.linalg.norm(f[:, gi[f"{side}_{t}"]], axis=-1).sum() for t in ("tip_left", "tip_right")])
        sub["tip_max"][side] = np.maximum(sub["tip_max"][side], tip)


def measurements(environment, s, sub):
    c, r = mag_frame(s, "round0")
    t, _ = mag_frame(s, "tool")
    pts = np.array(finger_points(environment))
    return dict(round_c=tuple(c), round_a=tuple(r[:, 0]), finger_round_m=float(np.linalg.norm(pts - s["round0"][0], axis=1).min()),
                tool_rel=tuple(t - TOOL_O), tool_contact_n=sub["tool_n"], through_gap=sub["through_gap"],
                mag_shift_m=sub["mag_shift"], mag_tilt_deg=sub["mag_tilt"], fell=sub["fell"])


@on_step
def _control(environment):
    view = STATE["view"]
    if view is None or STATE["start"] is None:
        return
    base = environment.unwrapped
    dt = float(base.physics_dt) * int(getattr(base.cfg, "decimation", 1))
    sub = STATE["sub"]
    STATE["sub"] = _fresh_sub()
    s = view.state()
    m = measurements(environment, s, sub)
    sc = STATE["scorer"]
    before = sc.veto
    sc.step(dt, m)
    STATE["steps"] += 1
    STATE["score"] = sc.score
    for side in "lr":
        STATE["finger_peak"][side] = np.maximum(STATE["finger_peak"][side], sub["tip_max"][side])
    if sc.veto is not None and before is None:
        STATE["veto_event"] = dict(step=STATE["steps"], t=round(STATE["sim_t"], 4), veto=sc.veto)
    for k in ("axis", "centre"):
        if sub[f"tg_{k}_n"]:
            STATE["tg"][k].append((STATE["steps"], sub[f"tg_{k}_n"]))
    if sc.score >= 1.0 and STATE["first_full"] is None:
        STATE["first_full"] = STATE["steps"]
    rs = ts.round_state(m["round_c"], m["round_a"])
    STATE["trace"].append(dict(step=STATE["steps"], t=round(STATE["sim_t"], 4), score=sc.score, level=ts.level(m), veto=sc.veto,
                               round_c_mm=np.round(np.array(m["round_c"]) * 1000, 2).tolist(), gap_mm=round(rs["gap"] * 1000, 2),
                               dz_mm=round(rs["dz"] * 1000, 2), yaw_deg=round(rs["yaw_deg"], 2), tilt_deg=round(rs["tilt_deg"], 2),
                               tool_rel_mm=np.round(np.array(m["tool_rel"]) * 1000, 1).tolist(), tool_n=round(m["tool_contact_n"], 2),
                               finger_round_mm=round(m["finger_round_m"] * 1000, 1), mag_shift_mm=round(sub["mag_shift"] * 1000, 3),
                               mag_tilt_deg=round(sub["mag_tilt"], 3), tg=[sub["tg_axis_n"], sub["tg_centre_n"]], tip_max_n={k: np.round(v, 1).tolist() for k, v in sub["tip_max"].items()}))


def summary():
    return dict(score=STATE["score"], veto=STATE["scorer"].veto, steps=STATE["steps"], first_full_step=STATE["first_full"],
                veto_event=STATE["veto_event"],
                through_gap_substeps={k: dict(steps=len(v), substeps=sum(n for _, n in v), first=v[:5]) for k, v in STATE["tg"].items()},
                finger_peak_n={k: np.round(v, 1).tolist() for k, v in STATE["finger_peak"].items()},
                physics_dt=PHYSICS_DT, sim_cfg=STATE.get("sim_cfg"), grip_n=GRIP_N, spring_k=round(K_SPRING, 1), f_top=round(F_TOP, 2), tool_g=round(TOOL_MASS * 1000, 1),
                start=dict(round_xy_yaw=[*ROUND_XY.tolist(), round(math.degrees(ROUND_YAW), 2)], tool_xy=TOOL_XY.tolist()))
