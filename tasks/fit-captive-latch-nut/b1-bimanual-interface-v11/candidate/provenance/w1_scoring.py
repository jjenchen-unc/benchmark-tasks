"""Tiered scorer for the B1 W1 latch-release goal (wave-07 repair, 2026-11-09).

Why: the independent review (SCO-02) found W1 awarded 0 for an undamaged, completed nut release whenever the arm was not
also swung to its stop. This module adds one documented intermediate tier. Source physics, source release/strip rules and
the source binary success() are unchanged; the full tier is the source W1 predicate plus one stricter, physically implied
condition (the nut must still be backed off >= 60 deg at the end).

Externally checkable measurements (simple tools):
  nut_backoff_deg   loosening rotation of the latch nut from its start position (paint mark vs a protractor ring), + = CCW
  arm_open_deg      swing-arm angle from its clamped position (protractor; the open stop is at 90 deg)
  latch_rounded     the nut's corners were rounded at any time (visual: bright rounded corners / a correct wrench slips)
  arm_free          the arm swings freely by hand after the trial (it is clamped until the nut is backed off 60 deg)
  contact_free      no hand, finger or wrench touches the arm or the nut (photo / observer), timed with a stopwatch

Tiers (any veto -> 0):
  1.0  arm free, nut backed off >= 60 deg, arm opened >= 85 deg, nut not rounded, nothing touching arm or nut,
       all continuously for the final 20 s
  0.5  arm free, nut backed off >= 60 deg, nut not rounded, nothing touching arm or nut, continuously for the final 20 s
       (any arm angle, including still closed)
  0.0  otherwise
Vetoes: the latch nut rounded at any time (sticky; later states cannot restore credit); any missing / non-finite /
inconsistent measurement or history field; a measurement failure at any time during the episode.
"""
from __future__ import annotations

import math

HANDS_OFF_S = 20.
RELEASE_DEG = 60.              # source B1_wrench.RELEASE_DEG
OPEN_DEG = 85.                 # source W1 task_done: arm angle <= -85 deg (stop at 90 deg)
TIERS = (.5, 1.)
TOL = 1e-6


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


def measure(B, m, d, st, k):
    """Evaluator-side measurement from the simulator (never exposed to the policy)."""
    n = st['nuts']['latch']; kn = k['nuts']['latch']
    actors = k['robot'] + k['tool_geoms']
    return dict(nut_backoff_deg=math.degrees(B.nut_angle(d, kn)),
                arm_open_deg=-math.degrees(float(d.qpos[k['arm']['q']])),
                latch_rounded=bool(n['rounded'] or st['flags'].get('nut_rounded')),
                arm_free=bool(st['arm_released']),
                contact_free=not B.contacts_between(d, actors, k['arm']['geoms'] + [kn['hex']]))


def validate_measurement(m):
    if not isinstance(m, dict):
        raise SchemaError('measurement must be a dict')
    out = dict(nut_backoff_deg=_num(m.get('nut_backoff_deg'), 'nut_backoff_deg'),
               arm_open_deg=_num(m.get('arm_open_deg'), 'arm_open_deg'),
               latch_rounded=_bool(m.get('latch_rounded'), 'latch_rounded'),
               arm_free=_bool(m.get('arm_free'), 'arm_free'),
               contact_free=_bool(m.get('contact_free'), 'contact_free'))
    # physical consistency: the clamped arm cannot open, and a free arm needs the nut to have been backed off
    if not out['arm_free'] and out['arm_open_deg'] > 5.:
        raise SchemaError('arm reported open while clamped')
    return out


def tier_now(m):
    """Instantaneous tier of a validated measurement: (score, tier, reasons)."""
    if m['latch_rounded']:
        return 0., 'veto', ['latch nut rounded']
    if not m['contact_free']:
        return 0., 'none', ['hand or wrench touching the arm or nut']
    released = m['arm_free'] and m['nut_backoff_deg'] >= RELEASE_DEG - TOL
    if not released:
        return 0., 'none', [f"nut backed off {m['nut_backoff_deg']:.1f} deg (< {RELEASE_DEG:.0f}) or arm still clamped"]
    if m['arm_open_deg'] >= OPEN_DEG - TOL:
        return 1., 'full', ['nut released undamaged, arm at its open stop']
    return .5, 'released', [f"nut released undamaged; arm opened only {m['arm_open_deg']:.1f} deg (< {OPEN_DEG:.0f})"]


HIST_KEYS = ('rounded_any', 'invalid_measurement_seen', 'hold_full_s', 'hold_release_s')


def validate_history(h):
    if not isinstance(h, dict) or set(h) - set(HIST_KEYS) or any(k not in h for k in HIST_KEYS):
        raise SchemaError('history must have exactly ' + ', '.join(HIST_KEYS))
    out = dict(rounded_any=_bool(h['rounded_any'], 'rounded_any'),
               invalid_measurement_seen=_bool(h['invalid_measurement_seen'], 'invalid_measurement_seen'),
               hold_full_s=_num(h['hold_full_s'], 'hold_full_s'), hold_release_s=_num(h['hold_release_s'], 'hold_release_s'))
    if out['hold_full_s'] < 0 or out['hold_release_s'] < 0:
        raise SchemaError('negative hold time')
    if out['hold_full_s'] > out['hold_release_s'] + 1e-9:
        raise SchemaError('full hold longer than release hold (full implies released)')
    return out


def score(meas, history):
    """Fail-closed terminal score. history: rounded_any, invalid_measurement_seen and the continuous final hold time of each
    tier condition (hold_release_s counts every step whose tier is 'released' or 'full')."""
    try:
        m = validate_measurement(meas); h = validate_history(history)
    except SchemaError as e:
        return dict(score=0., tier='veto', reasons=['invalid measurement/history: ' + str(e)])
    if h['invalid_measurement_seen']:
        return dict(score=0., tier='veto', reasons=['a measurement was missing or non-finite during the episode'])
    if h['rounded_any'] or m['latch_rounded']:
        return dict(score=0., tier='veto', reasons=['the latch nut was rounded during the episode'])
    s, tier, reasons = tier_now(m)
    if s == 1. and h['hold_full_s'] >= HANDS_OFF_S:
        return dict(score=1., tier='full', reasons=reasons)
    if s > 0. and h['hold_release_s'] >= HANDS_OFF_S:
        why = reasons if s == .5 else ['arm at its stop for only %.1f s; released state held %.1f s' % (h['hold_full_s'], h['hold_release_s'])]
        return dict(score=.5, tier='released', reasons=why)
    if s > 0.:
        return dict(score=0., tier='none', reasons=['released configuration not held hands-off for the final 20 s'])
    return dict(score=0., tier=tier, reasons=reasons)
