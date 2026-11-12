"""Grip-length table for the README: runs the demo with several knob settings and reports the measured
quantities (handle angle, knob turns, gap) next to the latch's internal pawl clamp force and peak cam torque.
This script, not the demo, reads those internal values.

    python validate.py task.py out 0 0.5 1 1.5 1.8 2 2.25 2.5 3
"""
import json, math, os, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
import demo


def main(task, out, turns):
    emb = None; rows = []
    for t in turns:
        os.environ['DEMO_TURNS'] = str(t)
        import latch
        old = {'run': [0, 0, 0], 'i': 0}

        def on_step(step, res):
            # The previous rule: handle >= 175 deg, pawl clamp > 20 N, no robot contact, cam not stripped, 20 s.
            d = latch.EPISODE['diag']; s = latch.EPISODE['samples']
            for rec in s[old['i']:]:
                h, k, g, touch = rec
                ok = h >= math.radians(175) and d.get('clamp', 0) > 20 and not touch and not d.get('stripped')
                old['run'] = [n + 1 if ok and c else 0 for n, c in zip(old['run'], (g < .001, g < .002, True))]
            old['i'] = len(s)
        cycles = math.ceil(2 * math.pi * t / demo.WRIST_SPAN - 1e-9)
        res, emb = demo.run(task, Path(out) / f'turns-{t}', 225 * cycles + 365 + 440, record=False, emb=emb, on_step=on_step)
        h, k, g, _ = latch.EPISODE['samples'][-1]; d = latch.EPISODE['diag']
        old_score = 0. if d.get('stripped') else next((v for n, v in zip(old['run'], (1., .5, .25)) if n >= 10000), 0.)
        row = dict(turns_commanded=t, knob_turns=round(k / (2 * math.pi), 3), handle_deg=round(math.degrees(h), 1), gap_mm=round(g * 1000, 3),
                   clamp_N=round(d.get('clamp', 0.), 1), peak_cam_torque_Nm=round(d.get('peak_torque', 0.), 3), stripped=bool(d.get('stripped')),
                   old_score=old_score, new_score=res['task_score'], steps=res['steps'])
        rows.append(row); print('ROW', json.dumps(row, default=float), flush=True)
    (Path(out) / 'table.json').write_text(json.dumps(rows, indent=1, default=float))


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2], [float(x) for x in sys.argv[3:]])
