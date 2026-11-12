"""Tiered scorer for the consolidated nose-down lens task (B6 nose_down_zoom).

Provenance: written by the fit_tool_service design worker (2026-11-08); hardened by repair round1 (2026-11-08) after the
independent review. Changes: complete typed schema validation; seated/locked/focal flags recomputed from numeric
measurements; explicit overload/drop coverage required; family check must be an explicit True; a continuous final hold per
tier recorded by rubric.py; and the former 0.25 "seated, not locked" tier REMOVED (see below). Source physics and source
success() are unchanged.

Externally checkable: lens flange on the mount face (axial gap <= 1.0 mm, axis offset <= 2.5 mm, axes within 4 deg), lens
index at the lock position (+-4 deg) with the lock pin engaged (cannot be turned back without the release button), the
focal-length scale reading (mm), a mount-overload / drop log, and a timer for the hands-off period.

Tiers (any veto -> 0):
  1.0  compatible lens seated + locked, focal 48-52 mm, continuously for the final 20 s with hands off camera and lenses
  0.5  compatible lens seated + locked continuously for the final 20 s with hands off (focal setting not held / not set)
  0.0  otherwise -- including a seated but UNLOCKED lens. Correction (wave-04, 2026-11-09): the round1 text said the capture
       weld retains an unlocked lens at the bayonet index because the lens weighs ~1.2 N. That was wrong. Lens masses are
       drawn from 0.30-0.70 kg (plus 0.05 kg ring and 0.15 kg front group, about 4.9-8.8 N), and the source lift_off rule
       (_rules) runs on every mount, including nose_down_zoom: an unlocked lens within 3 deg of the index whose low-passed
       outward load (robot contact force + weight along the mount axis) exceeds 2 N for 12 physics ticks is released.
       Measured (Slurm job 43122, seed 42, holding lens): case U (captured at the index, hand opened) -> lift-off, lens out
       of the mount into the open hand 46 mm below the seat; case UT (captured, turned 8.6 deg toward the lock, short of
       the click, hand opened) -> the detent's base torque turned the ring back to the index (0.09 deg) and the lens lifted
       off the same way. Both score 0 (no lens seated). The weld only retains an unlocked lens while the ring is held more
       than 3 deg past the index (lugs partly under the lip, as on a real bayonet). Such a state earns nothing either: the
       instruction asks for a locked lens, so it is a goal failure, not credit-worthy progress.
Vetoes: mount overload (> 1.5 N m against the stops for 50 ms) at any time, a lens dropped at any time (both sticky), a lens
of the wrong mount family on the mount, any missing / non-finite / inconsistent measurement or history field.
"""
from __future__ import annotations

import math

import numpy as np

FOCAL_TARGET, FOCAL_TOL = 50., 2.
HANDS_OFF_S = 20.
SEAT_GAP_MAX, SEAT_RAD_MAX, SEAT_TILT_DEG = .0010, .0025, 4.
LOCK_TOL_DEG = 4.
LENSES = ('lens_orange', 'lens_blue', 'lens_purple')
TIERS = (.5, 1.)


class SchemaError(ValueError):
    pass


def _num(x, name):
    if isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(float(x)):
        raise SchemaError(f'{name} missing or non-finite')
    return float(x)


def _bool(x, name):
    if not isinstance(x, bool):
        raise SchemaError(f'{name} must be an explicit boolean')
    return x


def _derived(r):
    if r['lens_on_mount'] is None:
        return dict(seated=False, locked=False)
    seated = bool(abs(r['axial_gap_m']) <= SEAT_GAP_MAX and r['radial_m'] <= SEAT_RAD_MAX and r['tilt_deg'] <= SEAT_TILT_DEG)
    locked = bool(seated and r['captured'] and r['pin_engaged'] and r['lock_angle_err_deg'] <= LOCK_TOL_DEG)
    return dict(seated=seated, locked=locked)


def validate_measurement(meas):
    if not isinstance(meas, dict):
        raise SchemaError('measurement missing')
    o = meas.get('lens_on_mount', 'MISSING')
    if o is None:
        for f in ('captured', 'pin_engaged', 'axial_gap_m', 'radial_m', 'tilt_deg', 'family_ok', 'lock_angle_err_deg', 'focal_mm'):
            if meas.get(f) is not None:
                raise SchemaError(f'no lens on mount but {f} reported')
        r = dict(lens_on_mount=None)
    else:
        if o not in LENSES + ('adapter_grey',):
            raise SchemaError('lens_on_mount invalid')
        r = dict(lens_on_mount=o, axial_gap_m=_num(meas.get('axial_gap_m'), 'axial_gap_m'), radial_m=_num(meas.get('radial_m'), 'radial_m'),
                 tilt_deg=_num(meas.get('tilt_deg'), 'tilt_deg'), family_ok=_bool(meas.get('family_ok'), 'family_ok'),
                 captured=_bool(meas.get('captured'), 'captured'), pin_engaged=_bool(meas.get('pin_engaged'), 'pin_engaged'),
                 lock_angle_err_deg=_num(meas.get('lock_angle_err_deg'), 'lock_angle_err_deg'))
        # Port-only schema repair: these measurements are magnitudes. Native
        # validator accepted negative magnitudes (including negative lock error).
        if any(r[f] < 0 for f in ('radial_m', 'tilt_deg', 'lock_angle_err_deg')):
            raise SchemaError('negative magnitude')
        if r['pin_engaged'] and not r['captured']:
            raise SchemaError('pin engaged on a lens that is not captured')
        if o in LENSES:
            r['focal_mm'] = _num(meas.get('focal_mm'), 'focal_mm')
            # plausibility only: the 18-55 mm scale, plus 1 mm for soft joint-limit overshoot (run1 N records end at 55.1 mm)
            if not 17. <= r['focal_mm'] <= 56.:
                raise SchemaError('focal_mm outside the 18-55 mm scale (+-1 mm)')
        elif meas.get('focal_mm') is not None:
            raise SchemaError('focal reading on a non-zoom part')
        else:
            r['focal_mm'] = None
    r.update(_derived(r))
    for f in ('seated', 'locked'):
        if f in meas and bool(meas[f]) != r[f]:
            raise SchemaError(f'{f} flag inconsistent with its measurements')
    return r


def validate_history(history):
    if not isinstance(history, dict):
        raise SchemaError('history missing')
    h = {f: _bool(history.get(f), f'history.{f}') for f in ('overload', 'dropped', 'invalid_measurement_seen')}
    for f in ('hands_off_end_s', 'hold_full_s', 'hold_locked_s'):
        v = _num(history.get(f), f'history.{f}')
        if v < 0:
            raise SchemaError(f'history.{f} negative')
        h[f] = v
    return h


def tier_now(r):
    """Configuration tier of a validated measurement, ignoring time: (score, tier, reasons)."""
    if r['lens_on_mount'] is not None and r['seated'] and r['family_ok'] is False:
        return 0., 'veto', ['wrong-family lens on mount']
    if not r['seated']:
        return 0., 'none', ['no lens seated on the mount']
    if not r['family_ok']:
        return 0., 'veto', ['wrong-family lens on mount']
    if not r['locked']:
        return 0., 'none', ['lens seated but not locked (no retention credit)']
    if r['focal_mm'] is None:
        return 0., 'veto', ['focal reading missing']
    if abs(r['focal_mm'] - FOCAL_TARGET) <= FOCAL_TOL:
        return 1., 'full', []
    return .5, 'locked', [f"focal {r['focal_mm']:.1f} mm outside 48-52 mm"]


def score(meas, history):
    """Fail-closed terminal score. history: overload, dropped, invalid_measurement_seen (explicit booleans),
    hands_off_end_s and the continuous final hold time of each tier (hold_full_s, hold_locked_s)."""
    try:
        r = validate_measurement(meas); h = validate_history(history)
    except SchemaError as e:
        return dict(score=0., tier='veto', reasons=['invalid measurement/history: ' + str(e)])
    reasons = [txt for f, txt in (('overload', 'mount overload'), ('dropped', 'lens dropped'),
                                  ('invalid_measurement_seen', 'missing/non-finite measurement during the episode')) if h[f]]
    if reasons:
        return dict(score=0., tier='veto', reasons=reasons)
    s, tier, why = tier_now(r)
    if s == 0.:
        return dict(score=0., tier=tier, reasons=why)
    if h['hands_off_end_s'] < HANDS_OFF_S:
        return dict(score=0., tier='none', reasons=['robot touched camera/lens within the final 20 s'])
    if s == 1. and h['hold_full_s'] >= HANDS_OFF_S:
        return dict(score=1., tier='full', reasons=[])
    # a locked lens whose focal setting drifted during the hold still earns the locked tier if locked the whole time
    if h['hold_locked_s'] >= HANDS_OFF_S:
        return dict(score=.5, tier='locked', reasons=why or ['focal not held at 48-52 mm for the full final 20 s'])
    return dict(score=0., tier='none', reasons=['locked configuration not held continuously for the final 20 s'])
