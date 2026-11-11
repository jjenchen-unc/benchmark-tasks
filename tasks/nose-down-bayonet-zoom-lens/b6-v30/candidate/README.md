B6 camera: Arena preparation

This is an executable development port for the original `nose_down_zoom` goal,
not a final submission. No Arena physics, robot full goal, Dream-1 or media result
has been established. Runtime credit is deliberately zero until actual B6
foundation channels pass. The source's native successes are design evidence only.

The robot must choose a compatible R-family zoom lens, physically mount and lock
it on the downward-facing right camera, set 50 mm and release all cameras/lenses
for a final continuous 20 s. Numeric tiers are 1.0 for locked and 48–52 mm for
that entire hold; 0.5 for locked throughout that hold with focal setting not
maintained; otherwise zero. Unlocked seating earns nothing. Any free/unheld
lens falling downward faster than 0.6 m/s, mount-stop/pin reaction above 1.5 N m
for 50 ms, wrist force above 80 N for 100 ms, or invalid measurements vetoes
credit. The episode ends on a failure, validated full score, or 3,034 actions
(151.7 simulation seconds at 20 Hz), including the final hold. A post-terminal
step raises before physics; reset starts a fresh episode.

The unchanged source package has four other goals (`A_mount_both`, `mount_lock`,
`swap_right`, `adapter_stack`). They remain read-only and are not additional
ported tasks. This directory implements one representative goal.

Reproduction commands, from this directory in an already installed Arena SDK
with the versions in `dependency-versions.json`:

```bash
python check_pure.py
python check_static.py
python check_constraints.py
python check_cold_sdk.py
python run.py --mode foundation --out /absolute/new/output/foundation
python run.py --mode reference --out /absolute/new/output/reference
python run.py --mode negative --case unlocked --out /absolute/new/output/unlocked
python run.py --mode negative --case creeping --out /absolute/new/output/creeping
python score_trace.py /absolute/new/output/reference/physics.jsonl
```

The first four commands are pure/static and do not launch a simulator. Runtime
commands require a GPU allocated by the root supervisor for RTX cameras, with
CPU PhysX (`device=cpu`). Do not run them on a login node. `run.py` copies the
complete package into its output, hashes it, executes that exact copied task
before cold SDK imports, and runs only copied helpers/assets. Output must not
exist. An optional explicit `--rtx-driver-check-override` adds only the process
AppLauncher flag `--/rtx/verifyDriverVersion/enabled=false`. No SDK, image, driver,
host or environment files are edited. No native simulation is imported at runtime.

A root-only serial request specifies 1 GPU, 4 CPUs, 32 GB and at most 30 minutes;
the author has not launched it. One episode's simulated runtime is bounded by
151.7 s. Actual wall runtime is unmeasured. First foundation checks neutral
clock/sensors, physical 27/7/40 motor ceilings, derived-seed reset and actual
120 N evaluator-only wrist loads. It does not prove capture/zoom/full goal and
does not activate credit. `negative_driver.py` is a separate evaluator driver
with explicitly labelled pose/joint arrangements; it cannot supply robot-demo
evidence. Preserved foundation snapshots and request are in the delivery record.

Physical inventory and setup

All geometry, local poses, masses, COMs, inertias, joint axes/limits and Coulomb
friction coefficients are in `fixture.json` and `setup-inventory.json`. Sizes
from native export are half-extents, or radius/half-height; units m/kg/rad.
Native task bodies, trays and both bracket pieces receive exactly the same
translation `(0.25, 0, -0.0525)` m. Objects are never resized. At reset:

| Object | Root/seat position (world m) | Geometry/material |
|---|---|---|
| Left camera, mount upward | (0.20, -0.28, 0.8775) | fixed 130×90×64 mm plastic-like housing, 80×6 mm metal-like face; source stand |
| Right camera, mount downward | (0.40, 0.28, 1.2475) | same housing/face, physically supported by original overhead bracket |
| Orange lens, family L | (0.37, -0.20, 0.7675) | free rigid 62 mm barrel, 58 mm ring, 52 mm sliding front |
| Blue lens, family R | (0.27, 0.06, 0.7675) | same dimensions, mass 0.47019328 kg |
| Purple lens, family R | (0.27, 0.20, 0.7675) | same dimensions, mass 0.44998257 kg |
| Left/right foam trays | (0.37,-0.27,0.7575), (0.27,0.13,0.7575) | 120×260×20 mm, fixed rigid approximation |
| Bracket arm | (0.40,0.35,1.3235) | 60×200×12 mm metal-like rigid support |
| Bracket post | (0.40,0.45,1.0385) | 40×40×582 mm metal-like rigid support |

Orange lens mass is 0.34473118 kg. Each total lens mass includes a 0.05 kg zoom
ring and 0.15 kg front group; no extra hidden ballast. Barrel is 50 mm long,
ring 20 mm, front 20 mm with 0–20 mm travel. Two source foam pads rest on the
unchanged station table (1.10×1.30×0.06 m, top 0.7475 m). Standard station camera
stand, light, floor and 3 RGB cameras come from the installed SDK; no Panda
assets are bundled. Exact small shapes and material coefficients are in inventory.

Both YAM bases are `(0,+/-0.305,0.90)` m on added 200×200×152.5 mm rigid pedestals
centred `(0,+/-0.305,0.82375)` m. These are a disclosed robot placement adaptation.
Their physical support and whole-robot/table/fixture clearance remain unverified;
there is no clearance claim. Genuine YAM joint/finger meshes and limits remain
from the installed SDK. Public actions are 14D absolute arm targets plus mean
jaw opening, 20 Hz. Shoulder caps are 27 N m, wrists 7 N m TOTAL including
gravity, using the task-local verified B5/D3 G/K drive representation. Fingers
retain source 40 N rather than SDK's 50 N default. Native Panda was 80 N m on
j1–4, 12 N m on j5–7; its capability and success do not transfer to YAM.

Reset and seed semantics

The physical case is the immutable native parameter draw for source seed 42.
Every Inspect seed, including harness-derived episode IDs, uses that same
geometry and physical properties; no exact-42 rejection or seed-dependent hidden
layout exists. `object_jitter=0` is required. Reset disables passive capture and
pin constraints, restores task root poses through Arena reset, restores all ten
joints and velocities before settling, and clears damage/timers/history only
at the new episode boundary. Reset after actual capture still needs runtime proof.
Real setup: restore all lenses upright in the listed slots at 18 mm, rings at
their index, release buttons out, all pins/captures disengaged, bracket and cameras
fixed to their supports, arms ready/open. No consumables are modeled.

Mechanics and fidelity

These are custom rigid training parts approximating camera mechanisms, not
manufacturer-validated equipment. Native bayonet lugs/neck/index are visual;
feet/face contact and measured alignment (>2 N pressing, <2.5 mm radial,
<1.5 mm axial, <4° tilt, <6° index error, mount ring <3°) engage a conditional
physical fixed joint at the current relative pose. Passive centring retains the
source 0.5 mm/s and 2°/s limits and 0.3 mm axial relief. Source mask exclusions
are preserved rather than making every visual part collision geometry.

The mount rotates on a real limited hinge with source friction/detent; a
conditional physical pin constraint engages after the source click+0.8°. Button
travel >2.5 mm releases the pin. An unlocked lens at the index detaches under
>2 N low-passed outward contact+gravity for 24 ms, unless pressed >1 N. Raw
incoming hinge torque is the proposed stop/pin channel; B6 known torque/sign
calibration is required. Current pull force sums contact normals and omits
friction contributions; this is disclosed and needs foundation measurement
before fidelity/credit can be asserted.

The lens has a real revolute zoom ring and prismatic front group. A balanced
passive helix potential with 20,000 N/m stiffness and 20 N s/m loss approximates
the source tight equality: extension = 0.02*angle/(pi/2). Front gravity can
back-drive zoom; source 18–55 mm range, 90° travel and blue/purple friction
0.06953881/0.01024142 N m are preserved. Static/kinetic joint-friction schema and
loaded zoom/creep need actual backend checks. Native torsional contact friction
is not automatically equivalent to PhysX friction. Fixed support bodies carry
1 milligram (1e-6 kg) numerical mass where native static bodies had zero mass; no free
lens is fixed or lightened. See `DEVIATIONS.md` for all unresolved differences.

Reference and evidence limits

`reference.py` implements the whole original sequence using only `emb.step(Action)`:
right pickup, physical bimanual handoff, left loaded flip/insertion/locking,
release, ring regrasp/50 mm setting, release and final hold. The teacher reads
simulator transforms/parameters; those are not added to policy observations.
It preserves a conservative default 30 N motion guard and the task's 80 N/100 ms
abort. It may fail IK or physical tracking and retains every attempted action.
Tested upright side handoffs and wrist-up insertion guesses failed nominal IK;
the full controller is **not nominal-path verified or physically proven**.
No guessed pose is described as a successful motion. Further feasible grasp/
path work is required without resizing parts, fixing lenses or raising limits.

`numeric_score.py` preserves the source tier rubric but rejects negative radial,
tilt and lock-error magnitudes (three executed source counterexamples are in
readiness artifacts). This does not change valid outcomes. Histories use measured
physics substeps, reset on any touch, fail closed on missing/nonfinite channels,
and keep overload/drop sticky. Measured raw rubric and externally creditable
runtime score are separate. No static assertion activates runtime channels.
Dream-1 is required for the final dual parallel-gripper package; no exemption
is claimed. Final model/video/media/board ownership checks remain outstanding.
