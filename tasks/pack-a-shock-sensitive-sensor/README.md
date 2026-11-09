# Pack a shock-sensitive sensor box

Pack blue-tagged A on suitable foam, close and tuck the carton, withdraw and let
the automatic physical tester drop it. The final numeric goal and every veto are
in task.json and runtime/d3_rubric.py.

Use the existing pinned Isaac Lab-Arena / YAM / Inspect Robots runtime specified
in dependency-pins.json. The task has no native MuJoCo runtime dependency.
Task physics and all four procedural USD assets are local under runtime/.
Robot/station assets are supplied by the pinned SDK. No sibling workspace is
imported. Keep the complete folder together when copying it elsewhere.

Run commands from this folder with the pinned runtime Python:

```bash
python -B run_demo.py --out /absolute/path/to/new-demo
python -B capture_setup.py --out /absolute/path/to/new-setup
```

The demo is a predetermined genuine14D20Hz robot action policy. All 363 actions
must execute through the real SDK embodiment and its original physics, guards
and scorer. No object pose, task joint, external force, latch or score is written
by the policy. The replay verifies its bundled source/action hashes, records the
actual runtime score and all three H.264 RGB streams, and exits unsuccessfully
unless this new run actually reaches1.0. The policy was planned using disclosed
privileged simulation-state reads. It is not a model policy.

The original matching-source UNC verification is described in
demo-provenance.json. A new replay on the final Brev L40S runtime independently
also achieved1.0; evaluation-provenance.json binds that result and the actual
Dream-1 trial. Both use the same21runtime files. Native Panda or historical gated results are not transferred.
One fixed layout, source parameters42, no object jitter. The action/time ceiling
is1604 genuine joint-position/normalized mean-jaw targets,20Hz,80.2 simulated
seconds. Stop immediately on a guard or budget termination. Reset restores ready
robots, both white indicators, four open1.75rad lids, untucked latches, armed
testers and empty histories. Nothing is consumed.

All physical object sizes, masses, materials, local primitives, starting poses,
station furniture and camera calibration are in setup-inventory.json,
runtime/fixture.json and the USD assets. In fixture.json, the old
initial_position_world_m fields are native provenance; task.py spawn positions
and setup-inventory.json are authoritative. Each empty assembly is270g:
200g rigid carton base +40g lid +30g foam (15g fixed in base,15g moving face).

The pad is a passive6DOF rigid top face, retaining global compression/tilt/shear,
4ms damping and nonlinear densification. Independent corner warp, foam-side
collision, cardboard tearing and flap flex are omitted. Physics uses0.5ms
substeps and0.125ms around hold/drop/window as the task selects; controls remain
exactly20Hz. Fingers are40N; arms use total27/7Nm drive caps including gravity.
Force and shock histories consume every physical substep. Force safety is the
declared continuous80N/100ms rule, not an instantaneous80N peak limit.
Parameter/material values are estimates, not hardware calibration claims.

The runtime subfolder is the frozen execution snapshot. Its internal development
README and draft result declarations are historical input records; current
submission information is in this README and the top-level metadata. Numerical
runtime code, fixture and assets are the actual verified inputs.

For the genuine Dream-1 run, use the official Inspect CLI with yam_arena,
dreamlabs/dream-1-beta, task=runtime/task.py, device=cuda:0, object_jitter=0,
control_hz=20, camera320x176, the exact top-level instruction, task_score and
max_steps1604. The final batch records continuous server-session identity,
complete trial records and all three camera histories. Failed/partial model
scores are valid only when the task/runtime itself completed without error.
No model score or uploaded media ID is invented by this package.

On the validated older UNC driver only, add --driver-check-override to the demo
or setup command. This is an explicit process Kit flag and changes no SDK,
image or host driver. Leave it off on the final launchable stack.


## Actual final evaluation

Dream-1 ran1,604actions to the normaltime limit and scored0.0. All393requests
used one continuous session;1,605actualRGBframes were stored per camera.
The final Brev scripted replay scored1.0 after363genuine14Dactions/18.15s,
including the actual tuck, untouched physical30cm drop and final stable hold.
It stored364frames per camera. No native/Panda or historical score is relabeled.
The published model/demo videos are synchronized views from these actual runs,
with no overlays, added frames or holds. Model0 is a measured outcome, not a
runtimeerror. The one fixed-layout trial does not establish robustness.

The automatic tester is an environment-owned finite-force apparatus; it acts
only after the robot packs/tucks/releases. The robot policy never applies those
external tester forces or writes task object/latch/score state. Exact replay
and reset commands above use only bundled inputs and the pinned SDK.


Reproduce the measured model route with existing Dream credentials:

```bash
python -B run_model.py --out /absolute/path/to/new-model-evaluation
```

This invokes the same official Inspect/Dream adapter and continuity wrapper used
for the recorded final trial, stores actual frames/records, and checks the
completed evaluator result. A runtime error or a changed session is rejected;
a valid low task score remains a valid model outcome. It never substitutes a
scripted policy. --preflight-only checks source/instruction with zero model calls.
On the older UNC driver only, use D3_DRIVER_CHECK_OVERRIDE=1 for this model
command; this affects only that process's existing Kit driver-version check.
