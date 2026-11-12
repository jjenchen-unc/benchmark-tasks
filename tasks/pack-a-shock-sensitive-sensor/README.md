# Pack a shock-sensitive sensor box

Pack blue-tagged A on suitable foam, close and tuck the carton, withdraw and let
the automatic physical tester drop it. The final numeric goal and every veto are
in task.json and runtime/d3_rubric.py.

Use the pinned Isaac Lab-Arena / YAM / Inspect Robots runtime specified in
dependency-pins.json. The task has no native MuJoCo runtime dependency.
Task physics and all four procedural USD assets are local under runtime/.
Robot and station assets are supplied by the pinned SDK. No sibling workspace is
imported. Keep the complete folder together when copying it elsewhere.

## Run the demo

Run commands from this folder with the pinned runtime Python:

```bash
python -B run_demo.py --out /absolute/path/to/new-demo
python -B capture_setup.py --out /absolute/path/to/new-setup
```

`python -B run_demo.py --out <new dir> --preflight-only` checks the runtime
file hashes and the action log without starting the simulator.

The demo is a predetermined 14-dimensional joint-position robot action policy at
20 Hz. All 363 actions execute through the real SDK embodiment and its original
physics, guards and scorer. No object pose, task joint, external force, latch or
score is written by the policy. The replay verifies its bundled source and action
hashes, records the actual runtime score and all three H.264 RGB streams, and
exits unsuccessfully unless the new run actually reaches 1.0. The policy was
planned using disclosed privileged simulation-state reads. It is not a model
policy. The recorded demo replay took about 27 minutes of wall time.

The original matching-source demo replay is described in demo-provenance.json.
A second replay on the final GPU runtime (NVIDIA L40S) independently also
achieved 1.0; evaluation-provenance.json binds that result and the actual
Dream-1 trial. Both used the same 21 runtime files (runtime/README.md was
reworded afterwards; no code, fixture or asset changed). Native Panda or
historical gated results are not transferred.

## Runtime and seeds

One fixed layout, seed 42, no object jitter. The action and time ceiling is 1,604
joint-position / normalized mean-jaw targets at 20 Hz, 80.2 simulated seconds.
The episode stops immediately on a guard or budget termination. Reset restores
both robots to ready, both indicators to white, all four lids open at 1.75 rad,
untucked latches, armed testers and empty histories. Nothing is consumed.

All physical object sizes, masses, materials, local primitives, starting poses,
station furniture and camera calibration are in setup-inventory.json,
runtime/fixture.json and the USD assets. In fixture.json, the old
initial_position_world_m fields are native provenance; task.py spawn positions
and setup-inventory.json are authoritative. Each empty assembly is 270 g:
200 g rigid carton base + 40 g lid + 30 g foam (15 g fixed in the base, 15 g in
the moving face).

The pad is a passive 6-DOF rigid top face, retaining global compression, tilt and
shear, 4 ms damping and nonlinear densification. Independent corner warp,
foam-side collision, cardboard tearing and flap flex are omitted. Physics uses
0.5 ms substeps, and 0.125 ms around the tester's hold, drop and post-landing
window as the task selects; controls remain exactly 20 Hz. Fingers are limited to
40 N; arms use total 27/7 N m drive caps including gravity. Force and shock
histories consume every physical substep. Force safety is the declared
continuous 80 N for 100 ms rule, not an instantaneous 80 N peak limit.
Parameter and material values are estimates, not hardware calibration claims.

The runtime subfolder is the frozen execution snapshot. Its README and draft
result declarations are historical development records; current submission
information is in this README and the top-level metadata. Numerical runtime
code, fixture and assets are the actual verified inputs.

## Dream-1 run

For the Dream-1 run, use the official Inspect CLI with yam_arena,
dreamlabs/dream-1-beta, task=runtime/task.py, device=cuda:0, object_jitter=0,
control_hz=20, camera 320x176, the exact top-level instruction, task_score and
max_steps 1604. The final batch records continuous server-session identity,
complete trial records and all three camera histories. Failed or partial model
scores are valid only when the task runtime itself completed without error.
No model score or uploaded media ID is invented by this package.

On an older GPU driver that fails the Kit driver-version check, add
--driver-check-override to the demo or setup command. This is an explicit
per-process Kit flag and changes no SDK, image or host driver. Leave it off on a
current driver.

## Actual final evaluation

Dream-1 ran 1,604 actions to the normal time limit and scored 0.0. All 393
requests used one continuous session; 1,605 actual RGB frames were stored per
camera. The final scripted replay scored 1.0 after 363 actions (18.15 s),
including the actual tuck, untouched physical 30 cm drop and final stable hold.
It stored 364 frames per camera. No native, Panda or historical score is
relabeled. The published model and demo videos are synchronized views from these
actual runs, with no overlays, added frames or holds. The model's 0 is a measured
outcome, not a runtime error. The one fixed-layout trial does not establish
robustness.

The automatic tester is an environment-owned finite-force apparatus; it acts
only after the robot packs, tucks and releases. The robot policy never applies
those external tester forces or writes task object, latch or score state. The
replay and reset commands above use only bundled inputs and the pinned SDK.

Reproduce the measured model run with existing Dream credentials:

```bash
python -B run_model.py --out /absolute/path/to/new-model-evaluation
```

This invokes the same official Inspect/Dream adapter and continuity wrapper used
for the recorded final trial, stores actual frames and records, and checks the
completed evaluator result. A runtime error or a changed session is rejected;
a valid low task score remains a valid model outcome. It never substitutes a
scripted policy. --preflight-only checks the runtime and instruction with zero
model calls. On an older GPU driver, set D3_DRIVER_CHECK_OVERRIDE=1 for this
model command; this affects only that process's existing Kit driver-version
check.
