# Build an ordered balance chain with three eccentric bars

Place the three sealed, unevenly weighted bars in orange A / blue B / purple C
bottom-to-top order on the narrow steel pedestal, then withdraw both hands for
20 seconds. A centred-looking stack does not balance: each bar has a different
centre of mass, and adding a bar changes the loads at the interfaces below it.
The setup is one fixed physical case. No variable-mass inference benchmark or
real hardware calibration is claimed.

## Reproduction

Use the existing YAM/Arena stack described in `task/runtime-expected.json`:
Python3.12.16, Isaac Sim6.1.0.0, IsaacLab distribution17.0.2,
Arena0.3.0, Inspect Robots1.0.0, Inspect YAM0.38.0 and Inspect Dream0.1.0.
The YAM checkout is0059d02a1a91bc6834c1d0846022891df0c78730 and Arena
c7edf3a9d5d8276565208f72422f8f8a0806d6cf. Distribution versions are not
repository release numbers. Task-local dynamic root resolution checks the
pinned source files and revisions before the simulator or model starts.
Physics runs on CPU; RTX cameras need a compatible NVIDIA GPU. An authenticated
Dream SDK session is required for the model command. Credentials stay outside
the task folder. No historical developer workspace or native MuJoCo runtime is
needed.

From this task directory, use new external absolute output directories:

```bash
python -I -B package_checks.py
python -I -B task/probe.py --mode replay --out /absolute/new-replay
python -I -B task/probe.py --mode setup --out /absolute/new-setup
python -I -B run_model.py --preflight-only --out /absolute/new-model
python -I -B run_model.py --out /absolute/new-model
```

The replay sends the saved1441 genuine14D joint/finger controls at20Hz, without
setting object poses or applying evaluator forces. It writes the actual score,
per-physics-step measurements, source hashes and three synchronized observation
videos. Setup captures reset views after render-only warmup while checking zero
actions, zero physics samples and unchanged robot/object state. For the legacy
driver used by the recorded trial, `B4_LEGACY_RTX_OVERRIDE=1` disables only the
process-local Kit driver version check; it changes no driver or SDK files.

Dream uses the identical instruction from `task.draft.json`, the same scene,
caps and scorer, four action rows per request, and the complete1896-control
budget. The helper preserves official SDK handling of retryable transport/server
faults within one session and logs each retry. It rejects session expiration,
episode restarts, missing/changed session IDs and fallback policies. Inspect
records and all three cameras' frame counts are checked before a model score is
reported. `tools/verify_model.py /absolute/completed-model` repeats those checks.

## Physical setup

World frame: x forward from the line of arm bases, y left, z up; dimensions are
metres unless stated. The fixed oak-look station tabletop is at z0.7475; the
table is1.10 x1.30 x0.06, centred(0.36,0,0.7175). There are exactly three loose
objects, each a320 x50 x50mm aluminium tube with a fixed sealed steel slug and
black plastic end caps. The solid exterior collider approximates that shell.
All long axes begin along+x; the black cap marks the local-x end.

| Bar | Initial centre(x,y,z) | Total mass | Local COM x | Diagonal inertia(kg m²) |
| --- | --- | --- | --- | --- |
| Orange A | (0.25,-0.45,0.7725) | 0.666439kg | +0.052404m | (0.0002621,0.0024823,0.0024823) |
| Blue B | (0.25,-0.34,0.7725) | 0.777470kg | -0.086185m | (0.0003048,0.0036414,0.0036414) |
| Purple C | (0.25,-0.23,0.7725) | 0.682144kg | +0.075443m | (0.0002681,0.0032543,0.0032543) |

The fixed bolted steel pedestal is28 x30 x160mm, centred(0.25,0.03,0.8275),
with its top at0.9075. Estimated bar friction is0.6 and pedestal friction0.5.
Two YAM six-joint arms with parallel grippers have root origins(0,+/-0.305,0.97),
facing+x, on fixed steel risers140 x140 x213mm centred(0,+/-0.305,0.854).
Their total gravity-inclusive drive limits are27Nm on joints1–3 and7Nm on
joints4–6; each finger drive is40N. The model receives the calibrated overhead
and two wrist cameras at320 x176; the reset screenshot is640 x360. The stock
station floor, table, overhead stand/camera housing and light are fixed furniture
specified by the pinned `yam_arena/station.py`; its file hash is recorded in
`task/runtime-expected.json`. They are not extra loose task parts.

`task/inventory.json`, `task/fixture.py` and `task/kit.py` specify the exact
physical layout and collision/material configuration. Reset restores those
poses, fixed native seed42/setting S and open READY arms, with object_jitter=0.
Inspect may derive a different episode identity seed; it passes unchanged to
the SDK and does not change this physical case. No parts are consumed.

## Score and safety

At the last physics sample, score the longest prefix A, AB or ABC whose current
valid state has lasted20 continuous seconds:1/3,2/3 or1.0. Otherwise score0.
Every bar in the prefix must contact its required support (A on pedestal, B on
A, C on B) and no object except its required adjacent neighbours. Any robot
contact with ANY bar, including a bar outside the prefix, resets every hold.
Each long axis is within0.06rad of horizontal and10degrees of either X
direction. One transverse face of the square bar must be level within0.06rad.
The Euclidean norm of the six numerical velocity components, with linear
velocity in m/s and angular velocity in rad/s, must be strictly below0.025.
The score uses current holds, not the highest previously reached tier.

A wrist force above30N for20ms or a nonfinger conservative load averaged over
20ms above50N freezes arm motion until that guard clears. A wrist force above
80N continuously for100ms, unavailable/invalid measurements, or robot-bar
contact while the contacting arm's TCP is outside x[-0.05,0.60],
y[-0.55,0.25],z[0.7725,1.1275] permanently makes the trial score0.
The budget is1896 controls at20Hz:94.8 physical seconds,25 x2ms physics
substeps per control. Partial holds require the same20s as full success.
Toppling can be recovered within the remaining budget unless a failure gate
trips. No crushing, fracture or invented damage flag is scored. The contract
in `task/contract.py` is authoritative for exact boundary comparisons.

## Recorded evaluation

Dream-1 (`dreamlabs/dream-1-beta`): **0**, 1896 genuine control steps out of1896; one continuous session and a non-error Inspect record.

Fresh Brev scripted replay: **1.0**,1441 controls /36025 physics samples,
72.05 simulated seconds and25.198 seconds of the final released ABC hold.
Every measurement channel was available; measurement error/failure were absent,
the force guard was clear at the end and the actual motor tables matched the
declared caps. Three actual videos each contain1442 frames at20fps, including
reset, with no inserted frames or extended hold. Reset-only setup separately
used zero controls and zero physics samples. Full video decode and sampled
initial/middle/final views were checked; the uploaded overview shows the whole
task. These are fresh public scores, not historical native/Panda scores.

The action replay represents a scripted demonstration that can use privileged
simulator state during planning. Actual task motion is produced by robot motor
commands and contact, not object-state actuation. Python physics, controller,
action, measurement and scoring files are byte-identical between the passing
replay/setup and model candidate, except removing a blank line at the end of
`task/contacts.py`; its AST and non-whitespace tokens are unchanged. Publication packs the
121,541,668-byte calibration-result JSON as
`task/evidence/calibration-result.json.gz` without changing a decompressed byte.
Its original raw SHA/size, compressed SHA and original evaluation runtime map
are preserved in `validation-source-provenance.json`. Only the evidence filename
and source-bound receipt/manifests are rebound; no validated channel or physical
threshold changes. The external Dream wrapper adds within-session transport recovery, startup checkpoints and the scoped argument handling described below.

Wall time depends on cold simulator initialization, rendering and (for Dream)
service request latency. The simulated times above are exact; actual launch/end
receipts are kept with the evaluation evidence.

## Fidelity and provenance

The rigid-box exterior retains the native rounded eccentric COM and diagonal
inertia. Material/contact coefficients are estimates; there is no tube flex,
crushing or fracture model. The nonfinger guard uses sum(abs(normal)) plus
sum(norm(friction)), a conservative bound rather than exact native resultant
force equivalence. Actual fixed-case correspondence evidence is bundled under
`task/evidence` and bound by `task/actual-validation-receipt.json`; it does not
claim hardware, FEM or all-variant calibration. A missing/mismatched receipt
gates runtime credit to0.

The native design snapshot and source notices remain in `native-provenance/`,
`THIRD_PARTY_NOTICES.md` and `LICENSE.i2rt`. Nested historical READMEs/drafts
describe earlier development states and are retained byte-for-byte as evidence;
this top-level README and `task.json` describe the evaluated public task.
The source-bound runtime manifests intentionally preserve those historical
bytes. `package-manifest.json` checks the complete final submission package.

Startup provenance: three earlier model attempts failed at native startup, before inference or controls; they have no valid score. The installed Kit crashreporter reads `/proc/self/cmdline` with `std::getline` and passes the truncated buffer to `argz_count`; the eight newline characters in this task's instruction trigger a native overrun. A standalone CPU reproduction and an exact-entry native startup A/B confirmed the failure and resolution. The launcher now passes the exact Inspect argument array through an external JSON file, preserving every instruction byte while keeping newlines out of OS argv. The helper still restores Python argv after Kit construction. No SDK, importer, crashreporter, physics, policy, score, seed or action budget is changed. A conditional startup-only diagnostic exits before task environment creation; that gate is unset for the actual model trial.

Camera scope: sampled model wrist views are often occluded by the arms or table;
the overhead view shows the full setup and robot motion. The original camera
observations and geometry are retained for both model and replay.
