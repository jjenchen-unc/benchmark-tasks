"""battery-recycle scorer: pure functions over measured poses (no simulator imports).

What a person checks, and how it is computed here:
* 9 V taped (secure): seen from the snap end, no metal of either snap shows outside the tape, and the tape is stuck to
  the case beyond both snaps (both ends pressed down, so the strip cannot simply lift off).
  Computed: each of 9 sample points on each snap's top face (centre + 8 near the rim) lies under a tape segment
  (inside its 3 x 19 mm footprint, the segment's adhesive face within 2.5 mm of the point and facing it); and among
  the segments stuck (bonded) to the battery and lying on its case (not on a snap's top: centre at most 1 mm above
  the snap-end face, e.g. on a narrow side) there is at least one on each side, along the strip, of the segments
  stuck on the snap tops (of the covering segments if none is stuck there). Bonds come from the adhesive model (``recyclekit``): a segment sticks only where it was pressed on.
* In the jar: the battery's centre is inside the jar's inner wall (top view) and below its rim.
* Released: no robot link touches the battery (PhysX contact report). Still: it moved less than 5 mm (net) over the
  last second.

Score (SCORE_OVER = "end", veto latched):
* veto -> 0 for the whole trial: the 9 V is in the jar while not taped for 5 control steps in a row (0.25 s).
* otherwise: 9 V taped + in the jar + released 0.6; 9 V taped elsewhere 0.3; each AA in the jar + released 0.2.
* success (the episode ends after it holds 2 s, the task file's SUCCESS_HOLD_S): everything in the jar and released, 9 V
  taped, all three still.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np

import layout as L

VETO_STEPS = 5          # control steps (20 Hz) = 0.25 s
COVER_GAP_M = 0.0025    # adhesive face of the covering segment within 2.5 mm of the snap's top face
COVER_SEAM_M = 0.0003   # footprint margin along the strip (neighbouring segments meet at a joint)
ON_CASE_TOL_M = 0.001   # an anchoring segment's centre is at most 1 mm above the snap-end face (the snaps stand 3.2 mm)
STILL_WINDOW = 20      # control steps (1 s)
STILL_NET_M = 0.005    # still = moved less than 5 mm (net) over the last 1 s (ignores the solver's mm-scale contact jitter)
CREDIT = dict(nine_v_in_jar=0.6, nine_v_taped_out=0.3, aa_in_jar=0.2)
AA_NAMES = ('aa_1', 'aa_2')


def point_under_segment(p_world, seg_pos, seg_rot, gap=COVER_GAP_M):
    """True if world point ``p_world`` lies under the segment: inside its footprint, on its adhesive (+z) side, within
    ``gap`` of the adhesive face."""
    loc = seg_rot.T @ (np.asarray(p_world, float) - seg_pos)
    half_t = L.TAPE_THICKNESS / 2
    return bool(abs(loc[0]) <= L.TAPE_SEG_LEN / 2 + COVER_SEAM_M and abs(loc[1]) <= L.TAPE_WIDTH / 2
                and 0.0 <= loc[2] <= half_t + gap)


def snap_coverage(bat_pos, bat_rot, segs):
    """Per snap: (covered, indices of covering segments). ``segs`` = list of (pos, rot) in strip order."""
    out = {}
    for name in ('positive', 'negative'):
        pts = (bat_rot @ L.snap_points(name).T).T + bat_pos
        idx = set()
        ok = True
        for p in pts:
            hit = [i for i, (sp, sr) in enumerate(segs) if point_under_segment(p, sp, sr)]
            if not hit:
                ok = False
            idx.update(hit)
        out[name] = (ok, sorted(idx))
    return out


def on_case_beyond_snaps(bat_pos, bat_rot, seg_pos):
    """True if a segment's centre lies on the case (the snap-end face or below it, e.g. a narrow side), not raised on
    a snap's top: stuck there, it holds the strip down beyond the snaps."""
    loc = bat_rot.T @ (np.asarray(seg_pos, float) - bat_pos)
    return bool(loc[2] <= L.NINE_V_SIZE[2] / 2 + L.TAPE_THICKNESS / 2 + ON_CASE_TOL_M)


def tape_state(bat_pos, bat_rot, segs, bonded_to_battery):
    """Secure-taping verdict and its parts. ``bonded_to_battery``: set of segment indices stuck to the 9 V."""
    cov = snap_coverage(bat_pos, bat_rot, segs)
    covered = all(c[0] for c in cov.values())
    cover_idx = sorted(set(cov['positive'][1]) | set(cov['negative'][1]))
    if covered and cover_idx:
        on_case = {i for i in bonded_to_battery if on_case_beyond_snaps(bat_pos, bat_rot, segs[i][0])}
        on_snaps = set(bonded_to_battery) - on_case
        # the strip's middle: the segments stuck on the snap tops (if none, the covering segments); a loose segment
        # that merely hangs over a snap does not move it
        mid = sorted(on_snaps) or cover_idx
        anchor_lo = any(i < mid[0] for i in on_case)
        anchor_hi = any(i > mid[-1] for i in on_case)
    else:
        anchor_lo = anchor_hi = False
    return dict(covered=covered, positive=cov['positive'][0], negative=cov['negative'][0], cover_idx=cover_idx,
                anchor_lo=anchor_lo, anchor_hi=anchor_hi, secure=bool(covered and anchor_lo and anchor_hi))


def in_jar(pos, center=L.JAR_CENTER, inner_r=L.JAR_INNER_R, rim_z=L.JAR_RIM_Z, floor_z=L.TOP):
    p = np.asarray(pos, float)
    return bool(np.hypot(p[0] - center[0], p[1] - center[1]) < inner_r and floor_z < p[2] < rim_z)


@dataclass
class Scorer:
    """Steps once per control step on measured state; the veto latches."""
    veto: str | None = None
    veto_step: int | None = None
    bare_run: int = 0
    step_n: int = 0
    score: float = 0.0
    success: bool = False
    last: dict = field(default_factory=dict)
    hist: dict = field(default_factory=dict)

    def step(self, nine_v, aa, taped, touched, speeds=None):
        """``nine_v``/``aa[name]`` = position (centre); ``taped`` = tape_state() of the 9 V; ``touched[name]`` = robot
        in contact. ``speeds`` is not used (stillness is judged from the positions over the last second)."""
        self.step_n += 1
        for n, p in (('nine_v', nine_v),) + tuple(aa.items()):
            self.hist.setdefault(n, deque(maxlen=STILL_WINDOW + 1)).append(np.asarray(p, float).copy())
        nv_in = in_jar(nine_v)
        if nv_in and not taped['secure']:
            self.bare_run += 1
        else:
            self.bare_run = 0
        if self.veto is None and self.bare_run >= VETO_STEPS:
            self.veto, self.veto_step = 'bare 9 V in the jar (terminals not taped)', self.step_n
        released = {n: (not touched.get(n, False)) for n in ('nine_v',) + AA_NAMES}
        still = {n: bool(len(self.hist.get(n, ())) == STILL_WINDOW + 1
                         and np.linalg.norm(self.hist[n][-1] - self.hist[n][0]) < STILL_NET_M)
                 for n in ('nine_v',) + AA_NAMES}
        aa_in = {n: in_jar(aa[n]) for n in AA_NAMES}
        credit = 0.0
        parts = {}
        if taped['secure'] and nv_in and released['nine_v']:
            parts['nine_v'] = CREDIT['nine_v_in_jar']
        elif taped['secure'] and not nv_in:
            parts['nine_v'] = CREDIT['nine_v_taped_out']
        else:
            parts['nine_v'] = 0.0
        for n in AA_NAMES:
            parts[n] = CREDIT['aa_in_jar'] if aa_in[n] and released[n] else 0.0
        credit = round(sum(parts.values()), 6)
        self.success = bool(self.veto is None and taped['secure'] and nv_in and all(aa_in.values())
                            and all(released.values()) and all(still.values()))
        self.score = 0.0 if self.veto else (1.0 if self.success else min(credit, 1.0))
        self.last = dict(nine_v_in_jar=nv_in, aa_in_jar=aa_in, released=released, still=still, taped=taped,
                         parts=parts, bare_run=self.bare_run)
        return self.score
