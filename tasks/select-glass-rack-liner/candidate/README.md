# Prevent a glass slipping on a wet liner

This standalone Arena/YAM development port preserves the repaired C1 task's
six hidden dry/wet materials, fixed20degree rack, water-entry wet rule, thin
liner grasp, hands-off hold and global glass-damage veto. Native successes are
provenance, not scores for this port. `task.draft.json` is intentionally a draft;
no model run, media upload, official readiness or publication is claimed.

`task.py` builds the station scene; `c1_kit.py` contains task-local scene/action
hooks. It imports the installed Arena/Inspect stack, never another local task.
Run with the project's existing pinned runtime; no dependency installation or
SDK/driver/environment changes are part of this package. C1 runtime physics is
1ms with50substeps per20Hz control. The run limit is2268controls/113.4seconds,
including the full hold. Physics and rendering must run in root's GPU allocation,
with CPU PhysX and the allocated GPU for station RTX cameras.

```
python -B check_static.py --out static-checks.json
python -B evaluator.py --suite foundation --out /absolute/artifacts/c1-foundation
python -B evaluator.py --suite friction --out /absolute/artifacts/c1-friction
python -B evaluator.py --suite wet --out /absolute/artifacts/c1-wet
python -B evaluator.py --suite guard --out /absolute/artifacts/c1-guard
python -B evaluator.py --suite reset --out /absolute/artifacts/c1-reset
python -B reference.py --target C1-T1 --seed 42 --out /absolute/artifacts/c1-reference
python -B reference.py --target C1-T2 --seed 42 --out /absolute/artifacts/c1-wet-reference
```

`evaluator.py` is explicitly evaluator-only. Its placed objects and120N known
load are diagnostics and can never count as a robot demonstration. It saves the
exact source, actions, measured contact actors/forces, station images and results.
`reference.py` reads actual object state and hidden friction for disclosed
feasibility planning. Its only state-changing actuation is genuine14D
`emb.step(Action(...))`: six absolute YAM joint targets and normalized jaw per
arm, at20Hz. It rejects failed lifts and bad tracking rather than moving objects
to a planned pose. `replay.py` independently replays its saved14D actions.
The reference has not run yet; root's first exact-source validation request is
in the assigned delivery directory. A subsequent Dream-1 model run is separate.

Scoring is the repaired source rubric, measured after each1ms physics update:
0.25 for a qualifying released liner on the rack;0.5 for a released glass on
exactly one qualifying liner;0.75 after10seconds and1.0 after20seconds hold,
following0.5seconds settling. The glass must stay within5mm of its first settled
position and within15degrees of the rack normal. Losing that placement makes it
void until renewed robot contact. It cannot touch bare rack or furniture.
T2 additionally requires that liner to be wet. T3 averages two distinct liners
and requires both glasses' holds at the same time; T4 samples the source transfer
parameters. A cup tipping past60degrees, slipping below the rack low edge minus
10mm, or dropping over30mm from its last robot-contact height onto furniture
vetoes all credit. Mobile contact chains transmit robot support; static rack/table
contacts do not transmit that support to every resting object.

Raw numeric scores are reported separately. Official exposed score remains0
until matching-source material/contact/guard/motor/fullgoal channels have actual
validation evidence. `VERIFIED_CHANNELS` is empty; successful static checks or
native videos cannot fill it. No caller flag bypasses that gate.

The arm drive cap is27/7Nm INCLUDING gravity, through G/K position bias inside
the existing implicit drive; arm feed-forward effort iszero. Fingers are capped
at40N each. Wrist soft-stop is30N for20ms per motion call (source default;
source's allowed5..60N setting is available via `C1_FORCE_GUARD_N`); an actual
wrist force above80N for100ms aborts independently. Nonfinger link20ms load
above50N stops motion. Stop events hold the measured arm targets; new controls
can retry but do not clear the hard abort history. Arm speed is capped at2.175rad/s, target slew at1.84875rad/s; jaw total gap
speed is at most0.15m/s with the source0.3s filter. Reference target slew is at most0.9rad/s. See
`DEVIATIONS.md` for the source tool-frame/workspace translation and contact
approximation that must be validated physically.

Reset restores scene roots through Arena, dry friction, wet flags, damage/void
and hold state, force-window histories and the control budget. All six liners,
two cups, pan walls/floor, free tray and rack legs are rebuilt without external
assets. Seed42 is nominal;43/44 and transfer target are retained, not claimed as
newly verified. Native source documents/licenses are snapshotted under
`provenance/`; `source-provenance.json` records their exact original bytes.

V2 uses two declared, physically supported inward-facing arm mounts; see the
superseding DEVIATIONS section for exact coordinates and support dimensions.
The scene now has21 objects, nine free. Runtime commands automatically execute
a verified source-frozen copy with the process-only RTX check override before
cold SDK imports. Root's first dispatch is foundation only, capped at30minutes.
Every foundation case writes checkpoints and stops immediately on failure.

Private parking-controller-v5 full reference candidate (unsubmitted):

`reference.py --public-case --target C1-T1 --seed 3867854786 --out <new-output>`

The physical layout/material seed remains42. The complete original dry-liner,
full glass-placement and hands-off sequence remains. Exactly after original
control169/sample8450, a source-bound right-arm parking DATA path is inserted;
the left holds its measured precontact six-joint pose with its original jaw
command. Both command transitions retain the original0.045rad/action cap.
The remaining21 original rack-hover targets receive temporal subdivision
(42controls nominal,21 extra) to reach the SAME targets after the hold.
The original2268 total control budget counts the inserted actions too. This is
a privileged feasibility controller using observed simulator state, not a model.

CPU geometry checks describe finite nominal sampled kinematics, including the
measured individual fingers and held-liner transform. They do not prove physical
grasp retention, force compliance, or full task success. Runtime refuses a start
outside the documented nominal-state tolerances, retains every original physical
guard and stops on observer failures. No verification channel is activated.
The launcher freezes exact files before the SDK boundary; see parent inventory,
fingerprint, geometry report and pre-SDK boundary receipt for this candidate.
