"""CPU-only reach study: python reach.py  (env: SCAN_X, SCAN_Y, SCAN_YAW, SCAN_PHI, SCAN_LEAN, SCAN_PROC, SCAN_OUT)
For each fixture placement (magazine origin x, y on the table, yaw, lean back about mag y) and role assignment, IK along
the whole planned path of both arms; reports the minimum joint margin per joint and the hull clearances arm-arm and
arm-magazine at the slide samples."""
import itertools
import json
import math
import os
import sys
from multiprocessing import Pool
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(1, str(Path(__file__).resolve().parents[1]))
import geom  # noqa: E402
import plan as pl  # noqa: E402
from yam_arena import analysis, ik, station  # noqa: E402
from yam_arena.constants import ARM_JOINTS, FINGER_OPEN, READY  # noqa: E402

E = os.environ.get
TOP = station.TABLE_TOP
SEEDS = [np.array(READY, float), np.array([0.0, 1.2, 1.4, -0.3, 0.0, 1.2]), np.array([0.0, 1.2, 1.4, -0.3, 0.0, -1.2]),
         np.array([0.5, 0.8, 0.9, -0.9, 0.6, 0.0]), np.array([-0.5, 0.8, 0.9, -0.9, -0.6, 0.0])]
ROUND_REL = [float(v) for v in E("SCAN_ROUND", "0.03,-0.12").split(",")]  # round start: mag frame x, y on the table


def path_ik(side, poses):
    """best (min margin, per-joint margins, qs) over seeds, following the path from each seed; None if unreachable"""
    best = None
    for seed in SEEDS:
        q, ms, qs, ok = seed, np.full(6, 1e9), [], True
        for p, r in poses:
            q, pe, re = ik.solve(p, ik.matrix_to_quat_xyzw(r), q_init=q, base_pos=station.BASES[side], iters=400)
            if pe > 0.0015 or re > math.radians(2):
                ok = False
                break
            ms = np.minimum(ms, np.degrees(np.minimum(q - ik.LOWER, ik.UPPER - q)))
            qs.append(q)
        if ok and (best is None or ms.min() > best[0]):
            best = (float(ms.min()), ms, qs)
    return best


def mag_pose(x, y, yaw, lean):
    r0 = pl.rot_z(math.radians(yaw))
    ry = pl.rot_y(-math.radians(lean))  # top rearward about the rear base edge of the fixture
    piv = np.array([-0.083, 0.0, -geom.MAG_Z0])
    return np.array([x, y, TOP + geom.MAG_Z0]) + r0 @ (piv - ry @ piv), r0 @ ry


def poses(mp, mr, yaw, psi, sgn, phi, beta=0.0):
    W = lambda p: mp + mr @ np.asarray(p)  # noqa: E731
    up = np.array([0, 0, 1.0])
    rn = mr @ pl.n_rot(psi, beta)
    K = dict(psi=psi, beta=beta)
    rp = mr @ pl.p_rot(sgn, phi)
    n, p = [], []
    lay = pl.n_tcp((pl.X_LAY, 0.1124), 0.0, **K)
    n += [(W(lay + [0, 0, 0.03]), rn), (W(lay), rn)]
    for xb in pl.slide_samples():
        t = pl.tilt_at(xb)
        zb = pl.base_z_at(xb, t)
        n.append((W(pl.n_tcp((xb, zb), t, **K)), rn))
        p.append((W(pl.p_tcp(pl.p_contact((xb, zb), t), sgn, phi)), rp))
    ns = len(n)
    back = n[-1][0] + mr @ np.array([0.008, 0, 0.004])
    t0 = pl.nose_push_tcp(pl.X_PINCH_END + pl.OAL + 0.001, pl.Z_RET0, **K)
    t1 = pl.nose_push_tcp(pl.X_END + pl.OAL, pl.Z_RET0, **K)
    n += [(back, rn), (W(t0), rn), (W(t1), rn), (W(t1 + [0.05, 0, 0.02]), rn)]
    p0 = pl.p_tcp(pl.p_contact((pl.X_LAY, 0.1124), 0.0), sgn, phi)
    p = [(W(p0) - rp[:, 2] * 0.05, rp), (W(p0), rp)] + p + [(p[-1][0] - rp[:, 2] * 0.08, rp)]
    return n, p, ns


def hulls(side, q, grip):
    qd = {j: float(v) for j, v in zip(ARM_JOINTS, q)}
    qd["joint7"] = qd["joint8"] = grip * FINGER_OPEN
    return analysis.world_hulls(analysis.fk_link_poses_world(qd, station.BASES[side]), ["link4", "link5", "gripper", "tip_left", "tip_right"])


_MAG = None


def mag_hulls(mp, mr):
    global _MAG
    if _MAG is None:
        from scipy.spatial import ConvexHull
        _MAG = []
        for v in geom.body_hulls():
            if v[:, 2].max() < 0.095:
                continue
            ch = ConvexHull(v)
            _MAG.append((v, ch.equations))
    out = []
    for v, eq in _MAG:
        n_w = eq[:, :3] @ mr.T
        out.append(("mag", analysis.Hull(vertices=v @ mr.T + mp, planes=np.c_[n_w, -eq[:, 3] + n_w @ mp])))
    return out


def one(cfg):
    x, y, yaw, lean, phi, role, beta = cfg
    beta = math.radians(beta)
    mp, mr = mag_pose(x, y, yaw, lean)
    sn, sp = ("right", "left") if role == "Nright" else ("left", "right")
    res = dict(x=x, y=y, yaw=yaw, lean=lean, phi=phi, role=role, beta=round(math.degrees(beta)))
    bn = bp = None
    for psi in (0.0, math.pi):
        n, _, ns = poses(mp, mr, yaw, psi, 1, math.radians(phi), beta)
        b = path_ik(sn, n)
        if b and (bn is None or b[0] > bn[0]):
            bn = (*b, psi, ns)
    for sgn in (1, -1):
        _, p, _ = poses(mp, mr, yaw, 0.0, sgn, math.radians(phi))
        b = path_ik(sp, p)
        if b and (bp is None or b[0] > bp[0]):
            bp = (*b, sgn)
    res["N"] = None if bn is None else dict(min=round(bn[0], 1), joints=np.round(bn[1], 1).tolist(), psi=round(math.degrees(bn[3])))
    res["P"] = None if bp is None else dict(min=round(bp[0], 1), joints=np.round(bp[1], 1).tolist(), sgn=bp[3])
    res["worst"] = None if (bn is None or bp is None) else round(min(bn[0], bp[0]), 1)
    if res["worst"] is not None and E("SCAN_CLEAR", "1") == "1" and res["worst"] > float(E("SCAN_CLEAR_MIN", "3")):
        mh = mag_hulls(mp, mr)
        ns = bn[4]
        dnp, dnm, dpm = [], [], []
        for k in range(len(pl.slide_samples())):
            hn = hulls(sn, bn[2][2 + k], 0.096)
            hp = hulls(sp, bp[2][2 + k], 0.0)
            dnp.append(analysis.hull_set_distance(hn, hp)[0])
            dnm.append(analysis.hull_set_distance(hn, mh)[0])
            dpm.append(analysis.hull_set_distance(hp, mh)[0])
        res["clear_mm"] = dict(N_P=np.round(np.array(dnp) * 1000, 2).tolist(), N_mag=np.round(np.array(dnm) * 1000, 2).tolist(),
                               P_mag=np.round(np.array(dpm) * 1000, 2).tolist())
    return res


if __name__ == "__main__":
    f = lambda k, d: [float(v) for v in E(k, d).split(",")]  # noqa: E731
    cfgs = list(itertools.product(f("SCAN_X", "0.30,0.36,0.42"), f("SCAN_Y", "-0.08,0,0.08"), f("SCAN_YAW", "-180,-150,-120,-90,-60,-30,0,30,60,90,120,150"),
                                  f("SCAN_LEAN", "0"), f("SCAN_PHI", "60"), E("SCAN_ROLE", "Nright").split(","), f("SCAN_BETA", "0")))
    out = open(E("SCAN_OUT", "logs/reach.jsonl"), "w")
    with Pool(int(E("SCAN_PROC", "16"))) as pool:
        for r in pool.imap_unordered(one, cfgs):
            out.write(json.dumps(r) + "\n")
            out.flush()
            print(r["x"], r["y"], r["yaw"], r["lean"], r["phi"], r["role"], r["beta"], "worst", r["worst"], "N", r["N"] and r["N"]["joints"], "P", r["P"] and r["P"]["joints"], r.get("clear_mm"), flush=True)
