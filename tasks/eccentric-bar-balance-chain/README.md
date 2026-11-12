# Build an ordered balance chain with three eccentric bars

Stack the three sealed, unevenly weighted bars on the narrow steel post, orange
A at the bottom, blue B in the middle and purple C on top, then let go of all of
them for 20 seconds. A stack that looks centred does not balance: each bar has
a different centre of mass, and every bar added changes the load at the
contacts below it. The setup is one fixed physical case. No real-hardware
calibration is claimed.

## Reproduction

The task runs on the standard YAM/Arena stack pinned in
`task/runtime-expected.json`: Python 3.12.16, Isaac Sim 6.1.0.0, IsaacLab
distribution 17.0.2, Arena 0.3.0, Inspect Robots 1.0.0, Inspect YAM 0.38.0 and
Inspect Dream 0.1.0. The YAM checkout is
0059d02a1a91bc6834c1d0846022891df0c78730 and the Arena checkout is
c7edf3a9d5d8276565208f72422f8f8a0806d6cf. Distribution versions are not
repository release numbers. Before the simulator or model starts, the task
locates the installed packages and checks the pinned source files and
revisions. Physics runs on CPU; the RTX cameras need a compatible NVIDIA GPU.
The model command needs an authenticated Dream SDK session; credentials stay
outside the task folder.

From this task directory, with new output directories outside the task folder:

```bash
python -I -B package_checks.py
python -I -B task/probe.py --mode replay --out /absolute/new-replay
python -I -B task/probe.py --mode setup --out /absolute/new-setup
python -I -B run_model.py --preflight-only --out /absolute/new-model
python -I -B run_model.py --out /absolute/new-model
```

`package_checks.py` is an offline check (no simulator, no model): it verifies
file hashes, the pinned runtime and the saved actions.

The replay sends the saved 1,441 joint and finger commands (14 numbers per
step) at 20 Hz. It never sets object poses or applies forces to objects. It
writes the score, per-physics-step measurements, source hashes and three
synchronised camera videos. The setup mode captures the reset views after a
render-only warmup and checks that no actions or physics steps ran and that the
robot and object states are unchanged. On older NVIDIA drivers,
`B4_LEGACY_RTX_OVERRIDE=1` skips only Kit's driver-version check for that
process; it changes no driver or SDK files.

The model run uses the same scene, motor limits, scorer and 1,896-step budget,
with four action rows per request. The recorded Dream-1 trial used the
original instruction text kept in `task.draft.json` (same goal, written as a
longer specification); `task.json` gives the plain-language version. The helper
keeps the SDK's normal retry handling for transient transport or server errors
within one session and logs each retry; it rejects session expiry, episode
restarts, changed session IDs and fallback policies. Before a model score is
reported, the Inspect record and the frame counts of all three cameras are
checked. `tools/verify_model.py /absolute/completed-model` repeats those checks.

Seeds: the physical case is fixed (layout and masses never change). Inspect is
called with `--seed 42`; it may derive a different episode seed, which is
passed unchanged to the SDK and does not change the physical case.

## Physical setup

World frame: x forward from the line of arm bases, y left, z up; dimensions are
in metres unless stated. The fixed oak-look tabletop is at z 0.7475; the table
is 1.10 x 1.30 x 0.06 m, centred at (0.36, 0, 0.7175). There are exactly three
loose objects, each a 320 x 50 x 50 mm aluminium tube with a fixed sealed steel
weight and black plastic end caps. A solid box collider approximates the tube.
All long axes start along +x; the black cap marks each bar's -x end.

| Bar | Initial centre (x, y, z) | Total mass | Local COM x | Diagonal inertia (kg m²) |
| --- | --- | --- | --- | --- |
| Orange A | (0.25, -0.45, 0.7725) | 0.666439 kg | +0.052404 m | (0.0002621, 0.0024823, 0.0024823) |
| Blue B | (0.25, -0.34, 0.7725) | 0.777470 kg | -0.086185 m | (0.0003048, 0.0036414, 0.0036414) |
| Purple C | (0.25, -0.23, 0.7725) | 0.682144 kg | +0.075443 m | (0.0002681, 0.0032543, 0.0032543) |

The fixed, bolted steel post is 28 x 30 x 160 mm, centred at (0.25, 0.03,
0.8275), with its top at z 0.9075. Estimated friction is 0.6 for the bars and
0.5 for the post. Two YAM six-joint arms with parallel grippers have base
origins at (0, +/-0.305, 0.97), facing +x, on fixed steel risers 140 x 140 x
213 mm centred at (0, +/-0.305, 0.854). Their gravity-inclusive joint torque
limits are 27 N m on joints 1-3 and 7 N m on joints 4-6; each finger drive is
40 N. The model receives the overhead camera and two wrist cameras at 320 x 176;
the reset screenshot is 640 x 360. The station floor, table, overhead stand,
camera housing and light are the standard station furniture defined by the
pinned `yam_arena/station.py` (its hash is in `task/runtime-expected.json`);
they are not loose task parts.

`task/inventory.json`, `task/fixture.py` and `task/kit.py` give the exact
layout and collision and material settings. Reset restores these poses and
the open ready pose of the arms, with `object_jitter=0`. No parts are consumed.

## Score and safety

At the last physics sample, score the longest stack A, A+B or A+B+C whose
current valid state has lasted 20 continuous seconds: 1/3, 2/3 or 1.0.
Otherwise score 0. Each bar in the stack must touch the support directly below
it (A on the post, B on A, C on B) and nothing except that support and the bar
directly above it in the stack. Any robot contact with any bar, including a bar
outside the stack, restarts every count. Each long axis must be within 0.06 rad
of horizontal and within 10 degrees of either x direction, and one square side
face must be level within 0.06 rad. The norm of the six velocity components
(linear in m/s, angular in rad/s) must be strictly below 0.025. The score uses
the stack that holds at the end, not the best one reached earlier.

A wrist force above 30 N for 20 ms, or a conservative load above 50 N averaged
over 20 ms on any arm link other than the fingers, stops arm motion until it
clears. A wrist force above 80 N continuously for 100 ms, a missing or invalid
measurement, or robot-bar contact while the contacting arm's gripper point is
outside x [-0.05, 0.60], y [-0.55, 0.25], z [0.7725, 1.1275] makes the whole run
score 0. The budget is 1,896 steps at 20 Hz (94.8 simulated seconds), with
25 physics substeps of 2 ms per step. Partial stacks need the same 20 s hold as
the full stack. A toppled bar can be recovered within the remaining budget
unless a failure condition has tripped. No crushing or breakage is modelled.
`task/contract.py` is authoritative for exact boundary comparisons.

## Recorded evaluation

Dream-1 (`dreamlabs/dream-1-beta`): **0.0**, all 1,896 control steps used, in
one continuous session with a non-error Inspect record.

Scripted replay (demo): **1.0**, 1,441 steps (36,025 physics samples, 72.05
simulated seconds), with the final released A+B+C stack held for 25.198
seconds. Every measurement channel was available, no measurement error or
failure occurred, the force guard was clear at the end, and the motor limits in
effect matched the declared ones. The three videos each contain 1,442 frames at
20 fps, including the reset frame, with no inserted frames or extended hold.
The reset-only setup run used zero steps and zero physics samples.

The scripted demonstration may use privileged simulator state while planning.
All task motion comes from robot motor commands and contact, never from setting
object states. The physics, controller, action, measurement and scoring files
are byte-identical between the replay and setup runs and the model run, except
for one removed blank line at the end of `task/contacts.py` (same AST and
tokens). The 121,541,668-byte calibration result is shipped compressed as
`task/evidence/calibration-result.json.gz` with no change to its decompressed
bytes; its original hash and size are in `validation-source-provenance.json`.

Wall time depends on simulator start-up, rendering and (for Dream) service
latency. The simulated times above are exact.

## Fidelity and provenance

The rigid-box exterior keeps each bar's off-centre centre of mass and diagonal
inertia. Material and contact coefficients are estimates; there is no tube
flex, crushing or fracture model. The link-load guard uses sum(abs(normal))
plus sum(norm(friction)), a conservative bound rather than the exact resultant
force. The fixed-case measurement evidence is in `task/evidence` and is bound
by `task/actual-validation-receipt.json`; it does not claim hardware, FEM or
all-variant calibration. If the receipt is missing or does not match, the
runtime score is gated to 0.

The original design snapshot and source notices are kept in
`native-provenance/`, `THIRD_PARTY_NOTICES.md` and `LICENSE.i2rt`. Nested
READMEs and drafts describe earlier development states and are kept
byte-for-byte because the runtime manifests hash them; this top-level README
and `task.json` describe the evaluated task. `package-manifest.json` covers the
whole package.

Start-up note: three earlier model attempts failed during simulator start-up,
before any inference or control step, and have no score. The installed Kit
crash reporter mis-parses process arguments that contain newline characters,
and the original instruction text contains eight. The launcher therefore passes
the Inspect argument list through a JSON file, keeping every instruction byte
while keeping newlines out of the OS argument list. No SDK, physics, policy,
score, seed or step budget is changed.

Camera note: the wrist views are often blocked by the arms or table; the
overhead view shows the whole setup and robot motion.
