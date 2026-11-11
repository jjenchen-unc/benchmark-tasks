# Private v9 measured tracking controller candidate

The selected first-pull evaluator runs the changed controller until measured
pinL withdrawal reaches10.5mm or the450-action evaluator cap. Full reference
mode retains both legs at holes2 and3, physical reseating, release and native
1s hands-off goal inside the unchanged4572-control budget. No physical success
is established by this bundle. See outer REPORT.md for measured first cause,
conditional checks, source closure and exact unsubmitted request.

Changes: knob jaw .12; after first knob grasp enable measured joint error
integral on both public arm targets (gain .2/control, per-control increment
bounds .0025rad proximal/.005rad wrist, bias .20rad proximal/.45rad wrist).
Clamp public commands to original SDK joint limits and feed back the clamp
for antiwindup. Forty closed-grip controls before each unload; allow120
withdrawal tracking controls and80 physical reseating controls. Native joint
stops, spring, caps, gains, contacts, scorer and final1s predicate are exact.
Public Action(data=14D) remains the only scene-changing controller call.

Historical documentation follows and does not establish v9 evidence.

# v7 bounded measured first-pull evaluator (private)

Selected entry: `python -I -B run.py --mode first-pull --legacy-rtx-override --out NEW_TRIAL`.
This is an evaluator-only 250-action prefix of unchanged v6 `reference.py`, not a repaired full-goal controller. It adds no placements or loads and changes the scene only through the existing public14D actions. Full native objective, four-operation sequence, variants, 1s hold, guards, 4572-control budget, geometry, material, caps and scorer remain unchanged. The evaluator deliberately stops after first pull and cannot establish task success.

`first_pull.py` records raw8 and all9 robot link poses plus task body/joint state at every2ms substep through support/knob grasp/unload/pull. It also records actual contact-view sensor/filter paths, all-link net contact force, and sparse nonzero pair NORMAL forces with zero-pair counts. Net forces include all colliders and are not friction-resolved target forces. Positive intended ownership requires both left fingertip/knobbL and both right fingertip/plate pairs; zero data never enables scoring. Actual shape identities are checked against fixture.json. No native/scorer contact adapter is replaced.

The outer bundle contains CPU reduction, exact UNSUBMITTED GPU request and source/proof manifests. Preserve final first-pull raw telemetry, source hashes and both contact logs even on abort. Review finite-actual-geometry.json and raw independent tips before choosing any jaw/feedback repair. `reference` mode remains the known-failing historical controller for provenance; it is NOT the selected verification.

Below is the preserved historical v6 documentation; its candidate claims are conditional and do not transfer physical success to v7.

# Private v6 controller candidate

Repairs the recorded left tip/pedestal witness using side entry for the left knob. World gripper rotation has columns (0,0,-1), (0,1,0), (1,0,0). Stage at knob + (-0.060,-0.060,+0.120) m; first rotate base yaw using public joint targets for 2 s, then fold to the solved staging configuration over 2 s. Retract through the same staging/yaw/home route after release. The right knob retains its original downward controller. No fixture or robot mount transforms. PairDiagnostic is retained as read-only telemetry; its empty rows do not establish contact coverage. CPU conditional/finite evidence is separate from fresh physical validation.

A 250 mm raised retreat and 2 s joint home reset after RIGHT-knob release prevents the next low base-yaw turn crossing the stand. Handle approach/release opening is 0.6 (nominal 56.34 mm for the 30 mm handle); closure and force caps are unchanged.

# Loaded stand reindexing — Arena development candidate

This executable candidate preserves QP-T1: raise BOTH legs from hole 1 to hole 3,
50 mm each. Both pins must seat, leg-top heights agree within 3 mm, and every
robot/stand contact must cease for the final continuous 1 s. A final released
hold at hole 2 earns 0.5. All other outcomes earn zero. Any instrument jolt over
3 g after a 5 ms low-pass, downward leg velocity over 0.2 m/s, nonfinite state,
or wrist force over 80 N continuously for 0.1 s permanently vetoes credit.
Keep wrist force below 60 N; the 80 N abort does not redefine that instruction.
Sampling is every 2 ms; 25 substeps per genuine 14D action at 20 Hz. The budget is
4,572 actions / 228.6 s. Holds are terminal, not best-so-far.

**No Arena physics, physical full-goal reference, Dream-1 or media has been
validated.** `contract.VALIDATED` is empty: actual score stays zero even if the
separate evaluator diagnostic predicate reaches 1. Native historical scores are
not Arena scores. Dream-1 is required for these two parallel-gripper arms.
This is one unit and one representative candidate, not five submitted tasks.
`source-goals.json` retains every native goal and its exact instruction; the
other native variants are not yet implemented as selectable Arena episodes.

## Run and score

Use the prepared YAM/Arena SDK environment described in `dependencies.json`.
Runtime imports neither MuJoCo nor any native workspace or Panda assets. All
runtime assets and helpers are task-local. No installation is performed by these
commands. Run physics only in the root supervisor's allocated GPU job:

```bash
python run.py --mode foundation --out /absolute/new/output
python run.py --mode negative --out /absolute/new/negative-output
python run.py --mode reference --out /absolute/new/reference-output
```

`run.py` freezes this directory, records its hashes, and **executes the copied
`task.py` before cold SDK scene/embodiment imports**. Its optional
`--legacy-rtx-override` changes only the process's AppLauncher arguments. Outputs
contain raw substep telemetry, actions, result and traceback on failure.
Foundation and negative modes are evaluator-only; their results are never a
robot full-goal demo. Negative mode applies a declared evaluator wrist load.
Reference mode uses the separate state-aware `reference.Teacher`; only ordinary
14D `Action` joint targets change objects, via robot contact. It does not write
object poses, joint states, external forces, mechanism flags or scoring history.
It stops on unresolved IK, failed physical pin withdrawal, safety termination or
time limit. Its proposed whole-goal sequence is not an executed success claim.

Inspect integration after root validation:

```bash
inspect-robots run --embodiment yam_arena -E task=task.py \
  -E object_jitter=0 -E control_hz=20 -E device=cpu \
  --instruction 'Raise both legs by two holes; seat both pins, level within 3mm, release for 1s.' \
  --scorer task_score
```

An actual policy/model and SDK CLI arguments must be supplied/verified by root;
this command declares the embodiment/scorer route, not a completed model run.
`task.score()` returns gated terminal credit; `result.json` and the JSONL trace
also expose diagnostic-only full/partial predicates for evaluator auditing.

Offline checks, which do not run physics:

```bash
python score_trace.py /absolute/output/measurements.jsonl --episode 1
python verify_static.py --out static-checks.json
python verify_usd.py --out usd-checks.json  # requires prepared pxr
python build_usd.py                       # regenerate local USD from fixture.json
```

## Layout and physical inventory

Lengths are metres. The source table was at z=0.8. Every task shape, mass, local
pose and relative dimension is preserved under the single transform
`p_arena = Rz(-90deg) p_native + (0.55,0.15,-0.0525)`.
`fixture.json` specifies all 12 source bodies, 63 shapes, materials/friction,
masses, inertias and 10 source joints. `stand.usda` contains only generated
primitives. The JSON's body transforms are neutral joint frames; reset joint
values are computed in `kit.initial()`, including the loaded +/-0.25 mm pin seat.

| Part | Geometry, mass and material |
| --- | --- |
| Two fixed inner legs | 30 mm square aluminium tubes, 249 mm high; 90 x 90 x 12 mm feet; five 8.5 mm square collision openings at 25 mm pitch, round visual marks |
| Two sliding outer legs | 38 mm square aluminium, 220 mm high, 250 g each; travel **-50 to +130 mm** from hole zero |
| Two spring pins | 8 mm capsule shaft, 6 mm rod, 50 g total per pin including knob; steel shaft/rod and red rigid polymer knob, 22 mm diameter x 24 mm long |
| Spring/rest slot | 1100 N/m, 9 N seated preload, 12 mm pull travel; after withdrawal beyond 10.2 mm, crossing 0.15 rad retains at max(-10.6 mm,current pull); return below 0.15 rad frees it; rotation stop 1.6 rad |
| Gas assist on R | Fixed source seed-42 value 29.05 N at midstroke, 10% progression each way over 150 mm; passive spring K=38.7333 N/m, damping 60 Ns/m; L has no assist |
| Safety pawl on L | Passive conditional lower joint stop, 0.6 mm below the preceding hole; ratchets after 4 mm overtravel; a 5 mm lift within 1 s clears it for 1 s |
| Top plate | 160 x 380 x 20 mm aluminium, 1 kg, two 30 mm diameter capsule handles with 70 mm spines, steel standoffs; plate connects to legs through pivots and right slot |
| Bolted instrument | 2.2 kg; 104 x 160 x 96 mm main housing, base/lens/screen/handle as enumerated in JSON; centre of mass native y=0.0008 (visible housing y=-0.03) |
| Right connection slot | 10 g, +/-12 mm passive slide |
| Green block | Fixed wooden riser, 120 x 120 x 50 mm, native centre (0.30,-0.48,0.825), Arena centre (0.07,-0.15,0.7725) |
| Added robot supports | Two bolted fixed steel pedestal approximations, 220 x 240 x 400 mm; centres (0.32,+/-0.305,0.9475), bottom on z=0.7475 table, top z=1.1475 |

Robot bases move from (0,+/-0.305,0.75) to (0.32,+/-0.305,1.15), orientation
unchanged. Their original 2.5 mm nominal base/table gap is retained above the
pedestals. Actual SDK base collision hull fits the support footprint; the first
200 mm square support failed that check and is preserved as a rejected attempt.
Endpoint SDK IK passes all eight start/goal knob/handle targets with source top
grasps. This does **not** prove support hardware stability, path, robot self,
cross-arm, fixture, camera-stand or furniture clearance. No such clearance is
claimed. The table and camera rig remain SDK station geometry.

YAM has six joints per arm and genuine action layout
`[L j1..j6,L jaw,R j1..j6,R jaw]`. Arm ceilings are actual SDK 27/27/27/7/7/7 Nm
TOTAL including gravity, with unchanged 80/10 gains. `motor_limits.py` puts G/K
inside the existing implicit drive and sets separate arm effort to zero. Actual
`root_view.get_dof_max_forces()` must confirm the limits. This is lower capacity
than native Panda's declared 80 Nm joints1–4 / 12 Nm joints5–7; no native motor
capacity or successful score transfers. Source 40 N finger ceilings are retained
below the SDK's 50 N maximum. Loaded feasibility needs actual later proof.

## Reset and fidelity

Every Inspect episode seed/ID selects the same explicitly declared source-42
physical case. There is no exact-42-only rejection. Object jitter must be zero.
The reset callback restores all ten custom joints, their passive targets and
limits, clears histories, settles through the real SDK, then starts the evaluator
clock. Foundation checks reset through two unrelated harness seeds. Actual settled
poses must match the declared reset; failure is reported, not hidden by pose
correction after an episode starts. Nothing is consumed. Real reset: unload the
plate, set both legs to hole1, return knobs to zero, restore block to its mark.

This is a rigid primitive approximation, not calibrated product hardware. The
native inner-tube solid is visual-only, with separate hidden perforated-wall
collision strips; no solid tube is placed across its holes. Native square hole
collision geometry and capsule pins are preserved. Conditional pawl/rest stops
are disclosed environment mechanisms. They change passive limits, never task
positions or commanded robot actions. Source global plate constraints and two
pivot connections are implemented as a serial articulation plus excluded-loop
spherical joints; backend acceptance/stability must be checked. Three 1 milligram
numerical fixed/axis-frame masses are additional, not native task masses.
Instrument acceleration is measured at its COM using actual link velocities at
2 ms; this equivalent kinematic readout still needs calibration against a known
load/impulse. Estimated product values are design evidence only.

Concrete source documentation/control issues addressed here: the true slide
lower bound is -50 mm, not the old asset note's -35 mm; block material is fixed
as wood rather than "wooden or plastic"; the source driver's [80x5,12x2]
residual clip and non-halting abort flag do not establish the declared TOTAL
[80x4,12x3] contract. This port uses the lower actual YAM bounds and enforces a
latched abort and step-after-termination guard. The numeric objective and safety
thresholds have not been relaxed.

Private v3 diagnostic (2026-11-10): controller motion/IK targets and native physics
are unchanged from actual 43829. Read-only primary shape events include normal and
friction impulses; independent contact views cover every robot link against all
resolved scene collision actors, including static shapes. A missing view or
invalid matrix aborts this diagnostic. No contact-report API/threshold/filter is
created or altered; existing enabled report delivery remains to be validated.
All task link poses and independent robot raw8/poses are copied numerically.
The cold launcher constructs the actual TaskFile before the simulator boundary.
This bundle diagnoses a failure; it is not a repaired route or a full-goal proof.
