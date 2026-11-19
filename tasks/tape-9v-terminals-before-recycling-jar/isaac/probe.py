"""Dev probe (not part of the task): scene load, settling, and the adhesive model under applied forces.

    python probe.py task.py OUT [--device cpu|cuda:0] [--stage settle|adhesive]
Uses probe-only fixtures (teleport + applied forces) that the task and the demo never use.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('task')
    ap.add_argument('out')
    ap.add_argument('--device', default='cuda:0')
    ap.add_argument('--stage', default='settle,adhesive')
    ap.add_argument('--cams', action='store_true')
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    from inspect_robots import Scene
    from inspect_robots.types import Action
    from inspect_robots_yam.packing import pack
    from yam_arena.constants import READY
    import recyclekit as K
    import layout as L
    emb = yam_arena_embodiment(task=str(Path(a.task).resolve()), object_jitter=0, control_hz=20, cam_width=320,
                               cam_height=176, device=a.device)
    obs = emb.reset(Scene(id='probe', instruction='probe'))
    print(f'PROBE reset done {time.time() - t0:.0f} s', flush=True)
    rt = K.runtime()
    act = Action(data=pack(np.r_[READY, 1.0], np.r_[READY, 1.0]))
    rep = dict(device=a.device, physics_dt=K.PHYSICS_DT)
    frames = []

    def step(n, tag):
        nonlocal obs
        for _ in range(n):
            r = emb.step(act)
            obs = r.observation
            if a.cams:
                frames.append(np.asarray(obs.images['top_cam']))
        p = rt.read()
        print(f'PROBE {tag} t={rt.t:.3f} bonds={rt.bond_list()} '
              f'score={K.STATE["score"]} hook={rt.hook_s:.1f}s', flush=True)
        return p

    p0 = rt.read()
    if 'settle' in a.stage:
        p1 = step(40, 'settle')
        drift = {n: round(float(np.linalg.norm(p1[n][0] - p0[n][0])) * 1000, 3) for n in p0}
        rep['settle_drift_mm'] = drift
        rep['settle_tape_z_mm'] = [round((p1[n][0][2] - L.TOP) * 1000, 3) for n in rt.tape]
        rep['bonds_after_settle'] = rt.bond_list()
        rep['contact_debug_settle'] = rt.contact_debug(K.PHYSICS_DT)
        print('PROBE contact settle', json.dumps(rep['contact_debug_settle'])[:800], flush=True)
        print('PROBE drift', json.dumps(drift), flush=True)
    if 'adhesive' in a.stage:
        # battery terminals down, centred over the tape, 1 mm above it
        r = np.column_stack([[1., 0, 0], [0, -1., 0], [0, 0, -1.]])
        c = L.TAPE_CENTER.copy()
        c[2] = L.TOP + L.TAPE_THICKNESS + L.SNAP_TOP_Z + 0.001
        rt.probe_place('nine_v', c, r)
        phase = {'f': np.array([0, 0, -6.0])}
        rt.fixture = lambda rt_, poses: rt_.probe_force({'nine_v': phase['f']})
        step(20, 'press 6 N')
        rep['contact_debug_press'] = rt.contact_debug(K.PHYSICS_DT)
        print('PROBE contact', json.dumps(rep['contact_debug_press'])[:1500], flush=True)
        rep['bonds_after_press'] = rt.bond_list()
        phase['f'] = np.array([0, 0, +0.50])  # lift (weight 0.44 N + tape)
        p = step(20, 'lift')
        rep['lift_battery_z_mm'] = round((p['nine_v'][0][2] - L.TOP) * 1000, 2)
        rep['lift_tape_z_mm'] = [round((p[n][0][2] - L.TOP) * 1000, 2) for n in rt.tape]
        rep['taped_after_lift'] = rt.tape_verdict(p)
        phase['f'] = np.array([0, 0, 0.44])
        # pull the -x end segment of the strip sideways/down hard: bonds must break at ~4 N
        rt.fixture = lambda rt_, poses: rt_.probe_force({'nine_v': np.array([0, 0, 0.44 + 0.01]),
                                                          rt_.tape[9]: np.array([0, 0, -8.0])})
        step(10, 'peel 8 N on segment 9')
        rep['bonds_after_peel'] = rt.bond_list()
        rt.fixture = None
        step(10, 'free')
        rep['bond_events'] = rt.bond_events
    if 'joint' in a.stage:
        import layout as LL
        LL.BOND_DRIFT_M = 1.0  # probe: keep bonds, measure raw drift
        step(10, 'pre')
        if 'moved' in a.stage:  # battery moved away from its USD (reset) pose first, like in the demo
            c0 = L.TAPE_CENTER.copy()
            c0[2] = L.MAT_TOP + L.TAPE_THICKNESS + L.NINE_V_SIZE[1] / 2 + 0.0005
            rt.probe_place('nine_v', c0 + np.array([0.0, 0.0, 0.0]), np.column_stack([[0., 1, 0], [0, 0, -1.], [-1., 0, 0]]))
            step(10, 'moved')
        p = rt.read()
        rt.make_bond(10, rt.stick_names.index('nine_v'), p, break_n=1e6)
        rt.make_bond(5, rt.stick_names.index('nine_v'), p, break_n=LL.BOND_BREAK_N)
        rel0 = {s: (p['nine_v'][1].T @ (p[rt.tape[s]][0] - p['nine_v'][0])) for s in (5, 10)}
        step(5, 'bonded')
        out = []
        rt.fixture = lambda rt_, poses: rt_.probe_force({'nine_v': np.array([0, 0, 0.9])})
        for k in range(6):
            q = step(5, f'lift {k}')
            row = dict(t=round(rt.t, 3), nine_v_z_mm=round((q['nine_v'][0][2] - L.TOP) * 1000, 2))
            for s_ in (5, 10):
                rel = q['nine_v'][1].T @ (q[rt.tape[s_]][0] - q['nine_v'][0])
                row[f'drift_{s_}_mm'] = round(float(np.linalg.norm(rel - rel0[s_])) * 1000, 3)
                row[f'tape_{s_}_z_mm'] = round((q[rt.tape[s_]][0][2] - L.TOP) * 1000, 2)
            out.append(row)
            print('PROBE joint', json.dumps(row), flush=True)
        rt.fixture = None
        rep['joint_test'] = out
        from pxr import UsdPhysics
        rep['joint_prims'] = {b['path']: bool(rt.stage.GetPrimAtPath(b['path']).IsValid()) for b in rt.bonds.values()}
    rep['nonfinite'] = rt.nonfinite
    rep['wall_s'] = round(time.time() - t0, 1)
    (Path(a.out) / 'probe.json').write_text(json.dumps(rep, indent=1, default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o)))
    if frames:
        import imageio
        imageio.mimwrite(out / 'probe_top.mp4', frames, fps=20)
        imageio.imwrite(out / 'probe_first.png', frames[0])
        imageio.imwrite(out / 'probe_last.png', frames[-1])
    print('PROBE done', flush=True)
    import os
    os._exit(0)


if __name__ == '__main__':
    main()
