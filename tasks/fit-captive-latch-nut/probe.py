"""Trial-fit table for the README: for each wrench, pick it from the rack with the demo's motions, push it onto the
nut, then turn it slowly (0.25 degrees per step, up to 20 degrees) and record what can be seen: whether the head
goes onto the nut, how far the wrench turns before the nut's paint mark moves, and whether the nut turns bright
(rounded). This script, not the demo, also reads the simulator's rounding state to confirm the colour.

    python probe.py task.py OUT [wrench names]
"""
import json, math, sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0, str(Path(__file__).resolve().parent))
import demo

LABEL = {'w12': '12 mm', 'w1_2': '1/2 in', 'w13': '13 mm', 'w14': '14 mm', 'wadj': 'adjustable (13.55 mm)'}


def yaw(R): return math.degrees(math.atan2(R[1, 0], R[0, 0]))


def main(task, out, names):
    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    import kit
    out = Path(out); out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=str(Path(task).resolve()), device='cpu', object_jitter=0, control_hz=20, cam_width=320, cam_height=176)
    rows = []
    for name in names:
        emb.reset(Scene(id='probe-' + name, instruction='probe'), seed=42)
        t = demo.Controller(emb, lambda *a: None)
        row = dict(wrench=LABEL[name])
        try:
            t.pick(name, lift=.06)
            N, psi = t.fit(name)
            seated = True
        except (RuntimeError, demo.Done) as e:
            seated = False; row['fit_error'] = str(e)
        p, R = t.tool(name)
        row['head_from_nut_axis_mm'] = round(1000 * float(np.hypot(*(p - (t.R @ demo.NUT + t.T))[:2])), 2)
        row['head_height_above_seat_mm'] = round(1000 * float(p[2] - (t.R @ demo.NUT + t.T)[2]), 1)
        row['goes_on'] = seated
        if seated:
            t.continuation_grasp = None; t.continuation_active = False
            y0 = yaw(R); n0 = t.nut_turn(); first = None
            for k in range(80):
                Rk = t.R @ Rotation.from_euler('z', psi + .25 * (k + 1), degrees=True).as_matrix()
                try: t.carry(name, N, Rk, 1, 'probe')
                except (RuntimeError, demo.Done): break
                turned = yaw(t.tool(name)[1]) - y0; nut = t.nut_turn() - n0; rounded = kit.ENGINE.mechanism.rounded
                if first is None and (nut > .5 or rounded):
                    first = dict(wrench_turn_deg=round(turned, 1), nut_turn_deg=round(nut, 1), rounded=rounded)
            row['when_nut_moves_or_rounds'] = first
            row['after_20_deg'] = dict(wrench_turn_deg=round(yaw(t.tool(name)[1]) - y0, 1), nut_turn_deg=round(t.nut_turn() - n0, 1), rounded=kit.ENGINE.mechanism.rounded)
        row['rounded_at_end'] = kit.ENGINE.mechanism.rounded
        rows.append(row); print('ROW', json.dumps(row), flush=True)
    (out / 'probe.json').write_text(json.dumps(rows, indent=1))


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], sys.argv[3:] or list(LABEL))
