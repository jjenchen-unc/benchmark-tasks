"""Evaluator-only prefix-hold partial credit for the B4_bars stacking goals.

Never imported by the robot policy and never returned in an observation.
The frozen source predicate (scene/B4_bars/benches/B4_bars.py: complete/success)
is reused unchanged for the full stack; this module only adds lower stages.

Stage k (k = 1..len(order)) holds at a physics step when, for the first k bars
of the required bottom-to-top order:
  * bar j touches the item directly below it (support for j = 0, else bar j-1);
  * bar j touches nothing except that item and bar j+1 *if bar j+1 is also in
    the stage* (table, robot, the support's side faces under an upper bar, or
    any bar outside the stage touching it all break the stage, so an
    unscored bar leaning on the stack can never brace it);
  * bar j is level (long axis within 3.4 deg of horizontal, a flat face
    down), headed along world X within 10 deg, and still (|6-dof qvel| < .025);
  * GLOBAL: no robot geom touches any bar (hands off).
Each stage has its own continuous-hold timer, reset to zero whenever it fails.
Final score = max k / len(order) over stages whose timer is >= HOLD_S (20 s)
at the end of the episode, otherwise 0. The full stage is exactly the source
predicate plus the (already implied) hands-off rule, so score 1.0 <=> source
success. Single-bar goals have one stage, i.e. they stay binary.
External measurement: stack order by colour, contact by feeler gauge/photo,
level by inclinometer (3.4 deg), heading by protractor against the table edge,
stillness and the 20 s timer by video.
"""
from __future__ import annotations

import numpy as np


class PrefixHoldScorer:
    def __init__(self, bench, model, target):
        self.bench = bench
        self.enabled = target is not None
        sp = bench.spec(target)
        self.order = [bench.BARS.index(c) for c in sp['order']]
        self.k = bench.ids(model)
        self.hold = np.zeros(len(self.order))
        self.max_hold = np.zeros(len(self.order))
        self.dt = float(model.opt.timestep)
        self.robot_contact_steps = 0

    def _bar_ok(self, model, data, i):
        b = self.k['bars'][i]
        R = data.xmat[b].reshape(3, 3)
        lv = self.bench.LEVEL
        if abs(R[2, 0]) > np.sin(lv) or max(abs(R[2, 1]), abs(R[2, 2])) < np.cos(lv):
            return False
        if abs(R[1, 0]) > np.sin(self.bench.YAW_TOL):
            return False
        q = self.k['bar_dof'][i]
        return bool(np.linalg.norm(data.qvel[q:q + 6]) < self.bench.STILL)

    def stages(self, model, data):
        G = np.asarray(self.k['group']); n = data.ncon
        a, b = G[data.contact.geom1[:n]], G[data.contact.geom2[:n]]
        robot_on_task = bool(np.any(((a == -1) & (b >= 0) & (b <= 2)) | ((b == -1) & (a >= 0) & (a <= 2))))
        ok = np.zeros(len(self.order), bool)
        if robot_on_task:
            return ok, True
        for s in range(len(self.order)):
            good = True
            for j in range(s + 1):
                i = self.order[j]
                touch = set(b[a == i].tolist()) | set(a[b == i].tolist())
                below = 10 if j == 0 else self.order[j - 1]
                allowed = {below} | ({self.order[j + 1]} if j < s else set())
                if below not in touch or touch - allowed or not self._bar_ok(model, data, i):
                    good = False
                    break
            ok[s] = good                    # stages are independent: stage s already re-checks every lower bar,
                                            # and a lower stage legitimately fails once an in-order bar sits on it
        return ok, False

    def update(self, model, data):
        if not self.enabled:
            return
        ok, robot = self.stages(model, data)
        self.robot_contact_steps += int(robot)
        self.hold = np.where(ok, self.hold + self.dt, 0.)
        self.max_hold = np.maximum(self.max_hold, self.hold)

    def result(self):
        need = float(self.bench.HOLD_S)
        held = [bool(h >= need - 1e-9) for h in self.hold]
        k = max([s + 1 for s, h in enumerate(held) if h], default=0)
        return dict(partial_score=round(k / len(self.order), 6) if self.enabled else 0.,
                    stages_held=k, stage_count=len(self.order),
                    stage_hold_s=[round(float(h), 4) for h in self.hold],
                    stage_max_hold_s=[round(float(h), 4) for h in self.max_hold],
                    hold_required_s=need, robot_contact_physics_steps=self.robot_contact_steps,
                    rule='prefix-hold k/N; any robot-bar contact resets every stage; evaluator only')
