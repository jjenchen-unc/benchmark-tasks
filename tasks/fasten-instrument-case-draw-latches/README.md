# Fasten an instrument case with spring draw latches

Close the open blue-grey case, seat each separate spring hook on any of its
three keeper bars, draw both hooks tight, return both secondary catches home,
and leave the case and its instrument without robot contact for20seconds.
The green case remains closed. Both loose cases and both instruments must
remain upright. This is the fixed DL-T2 representative, physical seed42.

Equipment cases use a hook, tensioning lever and independent safety catch to
secure a lid. Closing the lid or flattening an unattached lever is insufficient.
This task measures those individual fastening states and sustained released
closure. It does not ask for transporting the closed case or certify vibration
resistance or hardware durability. See IMPORTANCE.md for substantive overlap.

## Physical setup

Coordinates are metres, X forward,Y left,Z up. The standard YAM table top is
Z=.7475; robot bases remain(0,+.305,.75) and(0,-.305,.75). Reset arm joint1 is
+1.2rad left and-1.2rad right; other READY coordinates and both open fingers
are the exact posture.py values. Arm total effort limits are27/7Nm and finger
force40N, with the SDK's physical position/velocity limits retained.

- Fixed wooden plinth:200x600x60mm, centre(.35,-.04,.7775).
- Two loose rigid plastic cases:240x160x180mm bodies,40mm hinged lids,6mm
  walls;1.82kg each including lid and steel hardware. Blue-grey A root is
  (.34,.10,.8075), green-grey B root(.34,-.18,.8075), both yaw-90degrees.
- Each case has two steel lever/hook draw latches at±65mm. Each lid provides
  three5mm keeper bars per latch at6mm pitch. Hook preload4mm; passive spring
  constants and all inertias are in candidate/fixture.json. A has12x12x14mm
  secondary tabs moving sideways7mm; B has welded tabs and no catch slide.
- Each case contains one upright steel instrument surrogate:40mm diameter,
  160mm height,.30kg, with42mm diameter x12mm cap collider. Centres initially
  (.35,.10,.894) and(.35,-.18,.894).

SCENE_INVENTORY.md lists all95 task colliders and their dimensions. The
unchanged candidate/fixture.json defines every body-local pose, inertia,
colour, friction, collision mask, joint and spring. Nominal materials are rigid
surrogates, not measured product properties. Case/plinth floor mu=.7; case
hardware/source parts retain their per-shape values (including1.1,1.0,1.2),
and instruments mu=1.0. Source task coordinates transform as
(X,Y,Z)=(.28+y,-x,z-.0525); no object scale or free-body fixing is introduced.

A starts lid1.85rad, levers1.9rad, free links-100degrees, slides-4mm,
catches7mm, hooks disconnected. B starts closed with hooks on the native
selected bars and lever limits[0,1.9rad]. The SDK settles0.5s with passive
springs active before counted time; exact measured reset readback is retained
in the reviewed cache. Every Inspect seed is passed unchanged to the harness;
physical layout/properties are fixed42 and jitter0, with no hidden observation
parameters. Reset restores all loose bodies, joints, connections and history.

## Numeric scoring

All nonzero credit requires no robot contact with target A or its instrument.
Lid<=2degrees earns.25. Each fastener adds.25 only when a hook is physically
attached to any keeper, spring extension above its free length>=1.5mm,
lever<=.05rad and catch<=1.5mm. Full1.0 requires both fasteners, closed lid
and no target contact continuously20s. A broken hold restarts atzero. Before
20s the maximum is.75; an open lid earns0. Any case tilt>15degrees or either
instrument tilt>20degrees at any time vetoes all credit. All remaining outcomes
receive their currently satisfied tier, including0.

Wrist force>80N continuously100ms aborts with0. Wrist>60N for20ms or
nonfinger contact20ms-average>50N stops that arm for the remainder of the
current action. Nonfinger normal-plus-tangent force summation is conservative.
The physical clock is500Hz,25substeps per20Hz genuine14D robot action.
Maximum7964actions=398.2simulatedseconds. Early native success or physical
failure terminates model evaluation; no extra reset or recovery episode.

## Verified evidence and model result

The original whole scripted robot demonstration completed4056 genuine actions,
101400physics samples,202.8simulatedseconds and29.984s continuously hands off.
An independent all-sample source/contact/site/dwell/History audit gives1.0,
with no native failure, measurement error or guard stop. Original supervisor elapsed time was
1513.55s (the runtime result itself measured1511.02s) in the evaluated
CPU-PhysX/RTX environment. The teacher reads privileged
simulator state for planning, but changes the task only through robot Actions.

The historical demonstration's raw public score stays0 because all nine
validation channels were deliberately gated at capture time. A subsequent
source-bound physical review approves those channels. The existing model_task
bridge changes only the validation flags and consequent credit/termination;
actual History, physics, threshold and control bytes are unchanged. Exact
positive/negative bridge tests retain the original journal and demonstrate
why validated History gives1.0. No raw score or movie is rewritten.

The final actual evaluation used **Dream-1 + public gripper saturation[0,1]**
and scored **0.0** after6979actions. Inspect recorded success with no framework
error,1737successful model requests, one continuous session and no transport
retries. The native instrument-tilt veto ended the episode:20.0828degrees,
above its20degree limit. It latched at174465physical samples/348.93s, inside
the final control; supervised wall time was2127.97s. This is a valid physical
failure, not an API or parser error. No subsequent trial or recovery is included.

All20940original NPY observations (6980 per camera), complete records, actions,
source and media were backed up outside Git. Independent source, byte-hash,
frame/header/dtype, continuity, numeric-score and three-camera full H264 decode
checks passed. Every actual control was rechecked with the unchanged native
History.score using its recorded last row and native latched hold/failure;
no artificial history updates or extra hold credit were introduced. The physical
nine-channel receipt and source fingerprint remain unchanged.

The decoder projected16218out-of-range gripper values across all returned
finite rows to the public endpoints. Every arm value, row order and row count
was preserved and hash-checked. This is the disclosed decoder adaptation below,
not an unchanged vanilla policy. Two earlier episodes remain INVALID and
unscored: a parser error at3436actions and a provider authorization refusal
at1488actions. Their original raw data and error records remain separate;
neither supplies the final model score.

The candidate README/task.draft/DEVIATIONS are preserved historical source
bytes and contain earlier unvalidated status text. This outer README and
final task.json describe the submitted evaluated version. No source MuJoCo
score, failed approach or framework error is presented as the actual model score.

## Reproduce

Use the pinned Isaac/Arena/YAM environment specified in
runtime/runtime-source-pins.json and candidate/dependency-versions.json.
These SDK/robot/station dependencies are not bundled. RTX rendering needs a
compatible GPU even though physics device isCPU. The installed-source verifier
accepts only the reviewed bytes (with documented relocated editable roots).
Use its existing Python executable; do not install into or patch the SDK.

The public entry is task.py, which loads the receipt-checked model_task bridge.
Bare candidate/task.py is the preserved, deliberately uncredited source.
Set S2_EVIDENCE_CACHE to an absolute cache path outside this task folder for
an external Inspect harness; the default is ~/.cache/robocurve-s2-v30.
Keep object_jitter=0 and control_hz=20. Model observations are exactly the
original three RGB cameras plus robot joint/jaw state and the task instruction;
private physics/controller state is not added to model observations.

From this folder, using the environment's Python:

```bash
python run.py prepare --cache /absolute/cache/s2-v30
python run.py setup --cache /absolute/cache/s2-v30 --out /absolute/output/new-setup
python run.py replay --cache /absolute/cache/s2-v30 --out /absolute/output/new-replay
python run.py model --cache /absolute/cache/s2-v30 --out /absolute/output/new-model
python run.py verify-model --cache /absolute/cache/s2-v30 --out /absolute/output/new-model
```

Setup uses one actual reset and zero robot actions. Replay executes the exact
uncredited native teacher and records every action; read measured_score and
hold_s, not process exit0. Model executes the frozen actual Dream-1 beta
Inspect entry with7964steps,rows_per_request4 and320x176 three-camera images.
The external model decoder projects both gripper columns of every returned
finite row to the existing normalized[0,1] physical endpoints, then calls the
original SDK decoder. Arm numbers,row order,observations and all other validation
remain unchanged; no row is dropped and no inference is retried for a bad action.
Every preprojection grip range and clippedside/row/before/after value is logged.
This is disclosed as **Dream-1 + public gripper saturation[0,1]**, not an unchanged
vanilla policy. It addresses possible filter overshoot without claiming the
original failed reply was proved to be a filter artefact.

Only official same-session transient retries are allowed; expired/replaced
sessions, second resets or a second episode are rejected. The verifier checks
actual record/summary errors, score, source, instruction, sample clocks,
continuous session and every camera frame before encoding H264.

`prepare` downloads the immutable reviewed-input release asset, checks its
exact208633430bytes/SHA256 and every regular member, atomically extracts outside
this task, compares all46 readable source files with the frozen cache, verifies
the original review receipt, and cold-loads the public task before a simulator
is created. Source/cache tampering is rejected. For offline use add
`--archive /absolute/S2-v30-frozen-reviewed-inputs.tgz`; external task.py also
accepts S2_REVIEW_ARCHIVE. The frozen physical cache stays unchanged; the three
separately hashed reproduction files carry the reviewed decoder-only overlay.
For a direct Inspect task loader, the same adapter is installed once; optionally
set S2_GRIPPER_PROJECTION_LOG to an absolute log outside source/cache. `--preflight-only` on run.py phases performs no world
or model execution. Original large journals and videos stay outside Git.

## Fidelity limits

Task-only source primitives/masses/joints are preserved; both cases and both
instruments remain free. Native passive spring terms and SI joint friction
are retained. Hook capture requires measured contact,2.5mm proximity and150ms
dwell before enabling a preauthored spherical connection, with native release
and catch-limit rules. This is an explicit hybrid mechanism surrogate, not
simulation of detailed steel deformation. PhysX compliance/rolling friction
and contact solving differ from MuJoCo. Nonfinger guards may stop sooner.
The six-DOF YAM's total27/7Nm and40N fingers replace source Panda80/12Nm;
no torque/gain boost compensates for it. Force norm scale/API units have scoped
primary runtime evidence, not world-vector, all-load or hardware calibration.
Only this fixed representative and actual trial are claimed, not all native
variants, product durability or hardware validation. SOURCE_ASSETS.md records
original asset provenance; no manufacturer CAD or textures are bundled.

Six named original source files retain their reviewed final blank line; scoped
.gitattributes suppresses only that EOF-format warning, preserving their bytes.
