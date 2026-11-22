"""Scorer for the pool-8ball task (engine-independent, numpy only).

Per physics step the ``Monitor`` reads contact forces by collider identity (never contact positions): cue on cue
ball, cue ball on 8-ball, cue on 8-ball, robot links on either ball. It counts cue-on-cue-ball contact episodes
(strokes), notes whether the cue ball reached the 8-ball, and latches fouls: a second stroke, a push (cue tip on
the cue ball for longer than 60 ms), the cue touching the 8-ball, or the robot touching a ball.

Per control step the ``Scorer`` reads the two ball positions and speeds. Levels (score at the end of the trial):
  1.0   the 8-ball has dropped into the called pocket and the cue ball is still on the table; both balls at rest
        (or pocketed) for 1 s. Ends the trial.
  0.5   the 8-ball rests on the table within one ball width of the called pocket's mouth (its edge within 57 mm of
        the segment between the jaw points).
  0.25  the cue ball struck the 8-ball (a legal hit), but neither of the above.
  0     no hit; the cue ball pocketed or off the table (scratch); the 8-ball in another pocket or off the table; any
        foul (second stroke, push, cue on the 8-ball, robot touching a ball).
"""

from __future__ import annotations

import layout as L
import numpy as np

CONTACT_N = 0.02
EPISODE_GAP_S = 0.040  # a new stroke = cue-ball contact after 40 ms without contact
PUSH_S = 0.060  # tip on the cue ball for longer than this: a push, not a stroke
ROBOT_TOUCH_N = 0.05
STILL_V = 0.002
HOLD_STEPS = 20  # 1 s at 20 Hz


class Monitor:
    def __init__(self):
        self.strokes = 0
        self.t = 0.0
        self.last_cue_contact = -1.0
        self.contact_run = 0.0
        self.hit8 = False
        self.hit8_t = None
        self.stroke_t = None
        self.fouls = []
        self.peak = dict(cue_cb=0.0, cb_8=0.0, robot_ball=0.0, robot_cue=0.0)
        self.cb_speed_after = 0.0  # cue-ball speed 20 ms after the (first) stroke

    def _foul(self, why):
        if why not in self.fouls:
            self.fouls.append(why)

    def step(self, dt, cue_cb, cb_8, cue_8, robot_ball, robot_cue, cb_vel=None):
        self.t += dt
        for k, v in (("cue_cb", cue_cb), ("cb_8", cb_8), ("robot_ball", robot_ball), ("robot_cue", robot_cue)):
            self.peak[k] = max(self.peak[k], float(v))
        if cue_cb > CONTACT_N:
            if self.last_cue_contact < 0 or self.t - self.last_cue_contact > EPISODE_GAP_S:
                self.strokes += 1
                self.contact_run = 0.0
                if self.strokes == 1:
                    self.stroke_t = self.t
                else:
                    self._foul("second stroke")
            self.contact_run += dt
            if self.contact_run > PUSH_S:
                self._foul("push")
            self.last_cue_contact = self.t
        if (
            self.stroke_t is not None
            and cb_vel is not None
            and abs(self.t - self.stroke_t - 0.02) < dt / 2 + 1e-9
        ):
            self.cb_speed_after = float(np.linalg.norm(cb_vel))
        if cb_8 > CONTACT_N and not self.hit8:
            self.hit8 = True
            self.hit8_t = self.t
        if cue_8 > CONTACT_N:
            self._foul("cue touched the 8-ball")
        if robot_ball > ROBOT_TOUCH_N:
            self._foul("robot touched a ball")

    @property
    def foul(self):
        return bool(self.fouls)


def mouth_distance(pocket, pos):
    """Distance (m) from a ball centre to the segment between the pocket's jaw points (horizontal)."""
    a, b = L.pocket_jaws(pocket)
    p = np.asarray(pos[:2], float)
    t = float(np.clip((p - a) @ (b - a) / ((b - a) @ (b - a)), 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * (b - a))))


def classify(pc, p8, pocket, monitor):
    """(level before the at-rest hold, facts)."""
    e_in = L.which_pocket(p8)
    c_in = L.which_pocket(pc)
    c_on = L.on_table(pc)
    e_on = L.on_table(p8)
    near = e_on and mouth_distance(pocket, p8) <= 3 * L.BALL_R
    facts = dict(
        eight_pocket=e_in,
        cue_pocket=c_in,
        cue_on_table=bool(c_on),
        eight_on_table=bool(e_on),
        eight_mouth_mm=round(mouth_distance(pocket, p8) * 1e3, 1),
        hit8=monitor.hit8,
        strokes=monitor.strokes,
        fouls=list(monitor.fouls),
    )
    if monitor.foul:
        return 0.0, facts
    if c_in is not None or not c_on:
        return 0.0, facts  # scratch (or cue ball off the table)
    if e_in is not None:
        return (1.0 if e_in == pocket else 0.0), facts
    if not e_on:
        return 0.0, facts
    if near:
        return 0.5, facts
    if monitor.hit8:
        return 0.25, facts
    return 0.0, facts


class Scorer:
    def __init__(self):
        self.still = 0
        self.level = 0.0
        self.last = {}

    def step(self, pc, p8, vc, v8, pocket, monitor, tip_mm=None):
        raw, facts = classify(pc, p8, pocket, monitor)
        at_rest = (vc < STILL_V or facts["cue_pocket"] is not None) and (
            v8 < STILL_V or facts["eight_pocket"] is not None
        )
        self.still = self.still + 1 if (raw >= 1.0 and at_rest) else 0
        if raw >= 1.0:
            level = 1.0 if self.still >= HOLD_STEPS else 0.25  # until the balls have stopped: a legal hit
        else:
            level = raw
        self.level = level
        self.last = dict(
            level=level,
            raw=raw,
            still_steps=self.still,
            facts=facts,
            v_mm_s=[round(vc * 1e3, 2), round(v8 * 1e3, 2)],
            finger_ball_mm=None if tip_mm is None else round(tip_mm * 1e3, 1),
            peak={k: round(v, 3) for k, v in monitor.peak.items()},
            stroke_t=monitor.stroke_t,
            hit8_t=monitor.hit8_t,
            cb_speed_after=round(monitor.cb_speed_after, 4),
        )
        return level
