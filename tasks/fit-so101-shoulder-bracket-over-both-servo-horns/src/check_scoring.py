"""Synthetic checks of the scorer (no simulator): python check_scoring.py [out.json]. Exit code 1 on any failure."""
from __future__ import annotations

import json
import math
import sys

import numpy as np

import layout as L
from scoring import (HOLD_STEPS, Scorer, boxes_distance, measure, wrap_deg)

R_S, O_S = L.R_S, L.O_S
CHECKS = []


def check(name, cond, info=None):
    CHECKS.append(dict(name=name, ok=bool(cond), info=info))


def rx(deg):
    a = math.radians(deg)
    return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])


def pose(dx_mm=0., dy_mm=0., dz_mm=0., yaw=0., tilt=0.):
    """Bracket pose: home at yaw, then displaced in the bracket frame and tilted about its x axis."""
    p, r = L.bracket_home(yaw)
    r = r @ rx(tilt)
    return p + r @ (np.array([dx_mm, dy_mm, dz_mm]) / 1000.), r


def horn(deg):
    return L.horn_rot(deg)


def m_at(**kw):
    h = kw.pop('horn', 0.)
    p, r = pose(**kw)
    return measure(p, r, horn(h), O_S, R_S)


def main(out=None):
    # geometry of the measure
    m = m_at(yaw=25., horn=25.)
    check('home registered at matching yaw', m.home and m.registered and m.on, m.as_dict())
    m = m_at(yaw=25. + 90., horn=25.)
    check('any of the four 90-degree hole positions registers', m.registered, m.as_dict())
    m = m_at(yaw=0., horn=25.)
    check('home but 25 deg off is not registered', m.home and not m.registered and abs(m.register_deg + 25) < 1e-6,
          m.as_dict())
    m = m_at(yaw=3.9, horn=0.)
    check('3.9 deg registers', m.registered)
    m = m_at(yaw=4.1, horn=0.)
    check('4.1 deg does not register', m.home and not m.registered)
    m = m_at(dx_mm=5.)  # bracket backed off 5 mm toward its web side: horn axis at px = -5 mm
    check('backed off 5 mm: on, not home', m.on and not m.home and abs(m.px_mm + 5) < 1e-6, m.as_dict())
    m = m_at(dx_mm=9.0)
    check('backed off 9.0 mm (3.2 mm in the slot): on', m.on)
    m = m_at(dx_mm=9.4)
    check('backed off 9.4 mm (2.8 mm in the slot): not on', not m.on)
    m = m_at(dx_mm=.4)
    check('0.4 mm from home: home', m.home)
    m = m_at(dx_mm=.6)
    check('0.6 mm from home: not home', not m.home and m.on)
    # seated, leaning 0.8 deg back on the horns about the lower cheek (the modelled face play allows this): its top
    # cheek is 0.51 mm off the axis, the middle 0.25 mm: home
    th = math.radians(.8)
    ry = np.array([[math.cos(th), 0, math.sin(th)], [0, 1, 0], [-math.sin(th), 0, math.cos(th)]])
    p0, r0 = L.bracket_home(0.)
    low = np.array([0., 0., -.0364])
    r1 = r0 @ ry
    p1 = p0 + r0 @ low - r1 @ low
    m = measure(p1, r1, horn(0.), O_S, R_S)
    check('seated, leaning 0.8 deg about the lower cheek: home', m.home and abs(abs(m.px_mm) - .508) < .01 and
          m.radial_mid_mm < .3, m.as_dict())
    m = m_at(dz_mm=.8, dx_mm=3.)
    check('0.8 mm high, 3 mm out: on', m.on)
    m = m_at(dz_mm=1.5, dx_mm=3.)
    check('1.5 mm high: not on (cheeks not straddling)', not m.on)
    m = m_at(dz_mm=-36.4)  # the bottom cheek level with the top horn: wrong height by a whole cheek gap
    check('one cheek gap low: nothing', not m.on and not m.home)
    m = m_at(tilt=2.5, dx_mm=3.)
    check('2.5 deg tilt: on but not home', m.on and not m.home)
    m = m_at(tilt=180.)
    check('upside down: nothing', not m.on and not m.home)
    m = m_at(dy_mm=1.5, dx_mm=3.)
    check('1.5 mm off the slot line: not on', not m.on)
    p, r = L.bracket_start_rot(), None
    m = measure(L.BRACKET_START, L.bracket_start_rot(), horn(L.HORN_TOP_START_DEG), O_S, R_S)
    check('start pose scores nothing', not m.on and not m.home, m.as_dict())
    check('wrap', abs(wrap_deg(44., 90.) - 44.) < 1e-9 and abs(wrap_deg(46., 90.) + 44.) < 1e-9)

    # ladder and hold
    s = Scorer()
    home_reg = m_at(yaw=25., horn=25.)
    lv = [s.step(home_reg, .05, 0., 0.) for _ in range(HOLD_STEPS - 1)]
    check('registered, released, still: 0.75 until held 1 s', all(v == .75 for v in lv), lv[-3:])
    check('then 1.0', s.step(home_reg, .05, 0., 0.) == 1.0)
    check('hand back on it: 0.75', s.step(home_reg, .01, 0., 0.) == .75)
    s = Scorer()
    check('moving bracket never reaches 1.0', max(s.step(home_reg, .05, .01, 0.) for _ in range(40)) == .75)
    s = Scorer()
    check('held (gripper on it) never reaches 1.0', max(s.step(home_reg, .005, 0., 0.) for _ in range(40)) == .75)
    s = Scorer()
    check('home not registered: 0.5', s.step(m_at(yaw=0., horn=25.), .05, 0., 0.) == .5)
    check('on: 0.25', Scorer().step(m_at(dx_mm=5.), .05, 0., 0.) == .25)
    check('nothing: 0', Scorer().step(m_at(dx_mm=30.), .05, 0., 0.) == 0.)
    s = Scorer()
    for _ in range(HOLD_STEPS + 2):
        s.step(home_reg, .05, 0., 0.)
    check('pulled off after full score: final state counts (0)', s.step(m_at(dx_mm=30.), .05, 0., 0.) == 0.)

    # release distance helper
    p, r = L.bracket_home(0.)
    boxes = [(np.array([-.01, -.01, -.01]), np.array([.01, .01, .01]))]
    check('box distance outside', abs(boxes_distance(p + r @ np.array([.03, 0, 0]), boxes, p, r) - .02) < 1e-9)
    check('box distance inside', boxes_distance(p, boxes, p, r) == 0.)
    ok = all(c['ok'] for c in CHECKS)
    res = dict(passed=ok, n=len(CHECKS), failed=[c for c in CHECKS if not c['ok']], checks=CHECKS)
    if out:
        with open(out, 'w') as f:
            json.dump(res, f, indent=1, default=float)
    print(json.dumps(dict(passed=ok, n=len(CHECKS), failed=[c['name'] for c in CHECKS if not c['ok']])))
    return ok


if __name__ == '__main__':
    sys.exit(0 if main(sys.argv[1] if len(sys.argv) > 1 else None) else 1)
