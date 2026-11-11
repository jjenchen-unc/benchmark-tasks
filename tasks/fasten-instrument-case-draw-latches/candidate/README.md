# Fasten an instrument case with spring draw latches

Development Arena/YAM port of representative S2 DL-T2, physical case seed42.
Physics, contact calibration, robot success, Dream-1 and media are **unverified**.
Measured scoring is implemented but all runtime credit channels remain disabled
until root actual validation. This folder is not ready for publication.

Close the open blue-grey case, attach both hooks to any keeper bar, draw each
hook tight with its lever flat, slide both secondary catches home, then leave
both hands off the case and its instrument for20seconds. The other case and
both instruments remain subject to the damage checks. The native variants are
preserved in the original snapshot; this is one representative port, not five
new tasks. See VARIANTS.md.

## Run and score

Use the existing pinned Isaac/Arena/YAM environment; do not install or modify an
SDK. Commands run from this folder unless specified. Physics/RTX commands require
an allocated GPU even with `device=cpu`; never execute them on a login node.

```bash
python -B checks.py
python -B validate_assets.py                 # pxr only, no physics
python -B build_usd.py                       # optional offline regeneration
python -B run.py --mode foundation --out /absolute/output/new-foundation
python -B run.py --mode evaluator --out /absolute/output/new-evaluator
python -B run.py --mode reference --out /absolute/output/new-reference
```

Output must not already exist. The runner copies this entire folder, hashes it,
makes copied files read-only, and **executes copied task.py before cold SDK
imports**. The runtime uses that copy. Add `--legacy-rtx-override` only when the
explicit process-only AppLauncher driver-version check override is needed; no
host setting is changed. The first foundation allocation request is supplied
separately to the root supervisor. No job was submitted by this author.

For the final Inspect route, after actual runtime qualification, the task loader
entry is `task.py` with `-E task=/absolute/path/task.py -E object_jitter=0
-E control_hz=20 -E device=cpu`. Use the existing registered `task_score` scorer
and end-state score. Actual Dream-1 is still required for two parallel-gripper
arms; native scripted runs and an Inspect LLM run do not replace it. A public
model run must leave `S2_EVALUATOR_DIAGNOSTICS` unset: only3RGB images,14robot
joint/jaw coordinates and task text are policy observations. The privileged
teacher reads simulator state, explicitly disclosed, but changes objects only
by genuine14D20Hz robot `Action` calls.

An episode permits7964actions,398.2seconds of simulated time. Wall time and
real-time factor have not been measured in Arena. Neutral foundation checks
are not a robot demonstration. `robot-actions.jsonl` records every teacher
command and phase. `reference-failure.json` preserves failed approaches.
The nominal source push route failed actual SDK IK. This private candidate
uses the existing lid lip, a 30-degree upward approach, pull to 60 degrees,
and simultaneous opening/lifting before withdrawing. The exact lid COM makes
75 degrees open under gravity; the crossover is72.419751degrees. Native
hinge laws and goal thresholds are unchanged. Bounded IK, adaptive waypoint
subdivision, finite collision-part checks and raw independent finger readback
are controller checks, not physical proof. Original failing route evidence is
preserved outside this source snapshot. No full physical reference execution
or calibration is claimed; see the enclosing REPORT.md/result.json.

## Physical setup and reset

World coordinates: X forward, Y left, Z up. Standard YAM station table top
Z=.7475m, standard robot bases `(0,+.305,.75)` and `(0,-.305,.75)`. No robot
placement or pedestal adaptation is retained. Native coordinates are transformed
rigidly by `(X,Y,Z)=(.28+y,-x,z-.0525)`. Every relative task-object geometry,
size and mass is preserved. No free task body is fixed to obtain success.
Reset joint1 is+1.2rad left and-1.2rad right, with other READY joints and
independently open fingers unchanged. Finite sampled path checks are conditional;
physical tracking, contact, continuous swept clearance and scoring calibration
remain unverified.

The fixed wooden plinth is200x600x60mm, centred at `(.35,-.04,.7775)`.
Two loose plastic cases have240x160x180mm bodies,40mm lids and6mm walls.
Their root frames are `(.34,.10,.8075)` (blue-grey A) and
`(.34,-.18,.8075)` (green-grey B), yaw=-90degrees. Full geometry and all
local poses, inertias, colours, collision masks, springs and materials are in
fixture.json; static USD contains45shapes per case. Each compiled case,
including lid and latch hardware, weighs1.82kg. This corrects the old approximate
2.1kg inventory description; it does not change compiled source masses.

Each case has two steel lever/hook draw latches on its front,65mm either side
of its centre. Lever pivots are10mm proud and24mm below the seam; hook
pivots have18mm radius and25degree over-centre angle. Three5mm keeper bars
on each lid have6mm pitch; hook preload is4mm. Spring rates derive from the
unchanged physical seed42 parameters in fixture.json, not robot strength.
A has sideways secondary catches,12x12x14mm tabs with7mm travel; B has
fixed tabs and no secondary locking slide. Hardware inertial masses and exact
individual primitive sizes are listed in fixture.json.

Each case contains one upright instrument: cylinder40mm diameter,160mm
high,.30kg, with its original42mm diameter x12mm cap collider. Cylinder
centres start at `(.35,.10,.894)` and `(.35,-.18,.894)`. All94case/instrument
primitive shapes are retained, with the plinth as the95th task collider.
Case floors/plinth use nominal mu=.7; other case parts use source mu=1.1,
lever tabs1., catches1.2, and instruments1. Exact per-shape friction is in
fixture.json. These are chosen simulator values, not product measurements.

A starts with lid1.85rad open, both lever hinges1.9rad, free links
-100degrees, slides=-4mm, catches=7mm, and all hook connections off.
B starts closed on the source-selected keeper bar with both hooks connected.
Only episode reset restores task root/joint poses, joint ranges and connection
states. The SDK settles0.5s before the counted episode; the passive springs
already act during settling. The foundation must check that resulting initial
physical layout matches the declared reference. No trial properties are
randomized. **Every harness seed/derived episode ID routes to this same physical
seed42 layout and parameters**; arbitrary seeds are accepted. Object jitter
must be0. A reset clears all timers, force histories, failures and control count.

## Numeric scoring and safety

Measured at500Hz, controls20Hz. At episode end, no robot contact on target
case A or its instrument is required for every nonzero score. Lid angle<=2deg
earns.25; each fastener adds.25 if its hook is actually connected to one of
three keeper bars, lever<=.05rad, catch<=1.5mm and hook extension above
free length>=1.5mm. Both fasteners with lid closed therefore earn.75.
That complete state, hands off, held continuously20s earns1.0 and terminates.
Any broken condition resets the20s timer. Scores use current state, never best
historical partial credit. An unattached hook at a flat lever/preload earns
nothing, the concrete historical counterexample retained by checks.py.

At any time, either instrument tilt>20deg or either case tilt>15deg vetoes
all credit and terminates. Raw measured wrist force>80N continuously100ms
terminates as failure. Per-action wrist force>60N for20ms stops that arm's
command until the next action;60N is within the source's permitted guard
range. Moving non-finger link contact load averaged20ms>50N also stops the
arm. The current nonfinger implementation uses a conservative normal-plus-
tangential upper bound per substep before windowing, rather than native exact
per-contact vector magnitudes: it can stop earlier and needs runtime analysis.
Step after termination raises before advancing physics. No force, damage,
hook-tension or final-hold limit is relaxed to suit YAM.

YAM limits are27/27/27/7/7/7Nm **total including gravity**, with source40N
finger ceilings retained below the SDK's50N default. The task-local adapter
puts gravity inside the existing drive cap through a G/K target offset, then
sets separate feed-forward arm effort to zero. Actual gains and maximum forces
are checked using root_view.get_dof_stiffnesses/get_dof_max_forces. Native Panda
has higher80/12Nm arm capacity; those limits and any source evidence do not
transfer. YAM force/reach feasibility remains to be proved.

## Fidelity, provenance and dependencies

Rigid task primitives are compiled offline from an executed immutable copy of
the exact native bench. Runtime imports no MuJoCo, native workspace or Panda
asset. Cases are floating PhysX articulations. Hook engagement is an honestly
conditional spherical connection after actual hook/keeper contact within2.5mm
of a bar for150ms, with a15mm rearm distance. Slack/open-angle release and
catch-dependent lever stops preserve native rules. All policies are separate
from these passive environmental mechanisms. See DEVIATIONS.md for backend
approximations and outstanding calibration.

source-provenance.json and SOURCE_ASSETS.md record actual sources and notices.
Generated task primitives have no third-party mesh/texture assets. The installed
YAM/station remains an SDK dependency and is not bundled. Source Panda meshes
are neither included nor imported by the Arena runtime. No invented universal
asset-license blocker is asserted. See dependency-versions.json for observed
container distribution versions, including metadata differences from advertised
stack names; no environment was edited.
