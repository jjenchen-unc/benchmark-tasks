# timing-belt: fit, mesh and tension the GT2 timing belt of a robot arm joint's belt drive

The submission is the Isaac Lab-Arena (PhysX) `yam_arena` task in `isaac/`; `src/` holds the engine-independent
layout (`layout.py`) and scorer (`belt_scoring.py`, unit-tested in `tests/`). Dream-1 run (dreamlabs/dream-1-beta,
official `inspect-robots run`, 2,400 steps): task_score 0.

## The task
The rig is the 3:1 belt reduction of a desktop robot arm joint: a GT2 60-tooth pulley on the joint shaft (fixed
axis) and a GT2 20-tooth pulley on a NEMA 17 motor whose plate slides in a 12 mm slot on the frame rail and is clamped
by a quarter-turn lever on the slide's outer side. Initial state (same every trial, `task.json` "setup"): a 280 mm GT2 closed belt hangs over
the joint pulley in its teeth; the motor plate is at the inner (slack) end of its slot, lever loose.
The robot has to get the belt round the motor pulley as well (both pulleys inside the loop, the belt between each
pulley's flanges, teeth in the grooves), tension it to 6.2-12.4 N per span by pushing the motor plate outward (a
person checks 2-4 mm of mid-span deflection under a 1 N push), clamp the lever while holding the tension, and move both
grippers away. The belt cannot be stretched and the slot leaves only 16 mm of loop slack, so the belt cannot be
dropped over the motor pulley's flange: it has to be kept on the joint pulley and worked over the small pulley's
flange a little at a time.

## Run and score a trial
Host setup: Isaac Sim 6.1 + Isaac Lab-Arena + yam_arena + Inspect Robots in a container with one GPU.
`isaac/run_frozen.sh <run> "<cmd>"` copies `isaac/ src/ task.json` into `$TB_ISAAC_OUT/<run>/source` (read-only, with
per-file and combined SHA-256), mounts that snapshot at `/tb` and runs `<cmd>` in `/tb/isaac`; the log is
`<run>/run.log`. Set `RC_PODMAN` (container launcher), `ARENA_VENV_BIN` (Arena venv `bin/` inside the container),
`TB_ISAAC_OUT` / `TB_OUT_IN` (run directory on the host / inside the container) and optionally `RC_SLOT_CMD`
(a GPU slot allocator that sets `RC_GPU`) or `RC_GPU`.
```sh
cd <task folder>/isaac
I="$(python3 -c 'import json;print(json.load(open("../task.json"))["instruction"])')"
# official Inspect Robots path (default guardrails, task_score scorer, EvalLog for `inspect-robots video`): the scripted
# reference demo registered as a closed-loop policy `timing_belt_scripted` (official_run.py calls inspect_robots.cli.main)
bash run_frozen.sh <run> "python official_run.py --stage demo -- run --embodiment yam_arena \
  --policy timing_belt_scripted -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  -E device=cpu \
  --instruction \"$I\" --scorer task_score --max-steps <run_limit> --store-frames --no-prompt --log-dir $TB_OUT_IN/<run>"
# do-nothing baseline: inspect-robots run ... --policy yam_scripted (the task's demo() is empty: both arms hold READY;
# score 0; the generic `noop` policy is not compatible with yam_arena)
# scripted demo directly (acts only through the 14 joint/gripper commands): result.json, trace.jsonl, belt_trace.npy,
# demo_<camera>.mp4 (H.264), setup.png, final.png
bash run_frozen.sh <run> "python demo.py task.py $TB_OUT_IN/<run> --stage demo --device cpu --cam 320x176"
# partial / negative cases (one per --stage): neg-no-tension | neg-no-lock | neg-over-tension | neg-hands-on |
#   neg-drape (belt only laid over the small pulley, not worked round it) | idle
# several trials in ONE process with yam_arena's in-process reset between them (writes reset_check.json):
bash run_frozen.sh <run> "python demo.py task.py $TB_OUT_IN/<run> --stage disturb,idle,demo --device cpu --cam 320x176"
# scorer unit tests (any Python with numpy): python ../tests/test_scoring.py
```
Physics runs on the CPU (`--device cpu` for demo.py, `-E device=cpu` for the CLI; rendering stays on the GPU).
No environment variable changes the task, its physics, the reference demo or the scoring (all constants are in the
code; the variables above only tell `run_frozen.sh` where to run).

The score of a trial is the task's `score()` at the end of the run (`SCORE_OVER = "end"`): the scorer runs during
physics stepping (physics 480 Hz, scorer every 8th physics step, 60 Hz) on measured poses, the belt tension and contact reports, latches
its fall veto and times the 3 s hands-off hold; the episode ends as soon as the score reaches 1.0 (`success()`).
Runtime: the scripted demo reaches 1.0 after 1355 control steps (67.8 s simulated; the episode ends at success);
`run_limit` is 120 s (2400 steps). Wall clock on 3 CPU cores with PhysX on the CPU and three 320x176 cameras: about
15 min per trial (about 1 min of Kit start-up); a trial that runs to the full 2400 steps takes about 20 min. Seeds: none; the layout is fixed (`object_jitter=0`) and the demo is
deterministic on the CPU (two demo runs give identical traces).

Reset: the bench is one custom-spawned PhysX assembly (yam_arena task objects are single rigid bodies), so yam_arena's
own reset does not move it; `beltkit` puts every bench body and belt link back at its authored pose (zero velocity)
right before yam_arena's reset and settle; after the settle, the first reset of a process records the settled start
state (every body and link pose) and every later reset writes it back with zero velocity (the light hanging belt
would otherwise settle a few mm differently each time and the loose lever drift); it then unclamps the slide and
re-creates the scorer. Both writes happen inside the environment reset, never during a trial.

Reference demo (`demo.py`): it acts only through the 14 joint/gripper commands and follows a fixed plan from the start
state. It reads the bench and belt poses to aim and servo its moves (which link to pinch, keeping the pinched link in
the belt plane, following the lever's axis), but it has no recovery: if the belt slips out of the pinch or goes on
twisted, it carries on and the trial fails. From the start state of a fresh process it is deterministic and reaches
1.0. After an in-process reset the start poses are identical, but the velocities and the simulation's contact state are
not: in a process's first trial the hanging belt is still swinging slightly after the settle (its lowest point moves
within an 8 mm range during the first second), while a trial after a reset starts from the same poses at rest. In the
evidence runs the trial after a reset is already about 3 mm (largest belt-link difference) away from the first trial
at step 10, and the belt's contact with the pulleys and fingers amplifies such differences, so after a reset it
reaches 1.0 only on some starts. In development, post-reset starts of this demo and of a variant with a closed-loop pick
reached 1.0 in about half of the distinct cases; the failures were the belt slipping out of the first pinch, or a
stretch of belt going on twisted round the 60T (which the scorer rejects). The task's scoring does not depend on the
demo; a policy has to handle the belt it sees.

## Evidence (frozen source, combined SHA-256 `2b91411aa66e20bf70f44d29acd4e38e85bb37b45c5600743c9acb684f207aae`)
All runs below are on that snapshot of `isaac/ src/ task.json`, which is also the current state of those files. PhysX
on the CPU, cameras 320x176. Evidence files to upload (paths relative to the evidence bundle):
- `tiled/official_tiled_top_left_right.mp4` (H.264; overhead + left wrist + right wrist, same timing) of the official
  `inspect-robots run` (task_score 1.0; per-camera videos from `inspect-robots video` in the official run's `video/` directory),
  with `tiled/official_setup_tiled_top_left_right.png` and `tiled/official_final_tiled_top_left_right.png` (the end
  state: both wrist cameras on the drive, belt on both pulleys, lever turned forward);
- start state at higher resolution (the 320x176 setup frame is too small to show the belt):
  `tiled/setup_hires_composite_top_crop_right_wrist.png` (left: crop of a 1920x1056 overhead render of the start;
  right: the right wrist camera at step 62 of the demo, before anything touches the belt, 55 mm clearance; trace
  identical to the 320x176 demo run): the belt hangs on the 60T joint pulley, the 20T motor pulley is bare, the lever
  handle points outward. Also `tiled/setup_hires_top_1920x1056.png`, `tiled/setup_hires_top_bench_crop.png`,
  `tiled/setup_hires_right_wrist_step62.png`;
- `tiled/demo_*` (the same script through `demo.py`, 1.0), `setup_top.png`, `final_top.png`, `official_evallog.json`;
- `tiled/neg-*` and `*_result.json` for the partial cases; `tiled/reset-*`, `reset_check.json`,
  `reset-demo_reset_check.json`, `reset-*_result.json` for the in-process resets; `hashes.txt`.

| Run | Score | Stimulus (measured end state) |
|---|---|---|
| demo x2 (`demo.py`, identical traces) | 1.0, 1.0 | 1355 steps (67.8 s); C 101.59 mm, T 9.9 N, both wraps seated (60T: 19 hinges), lever -90.6 deg (clamped), hands clear 3 s |
| official CLI (`official_run.py`) | 1.0 | same script through `inspect_robots.cli.main`, default guardrails, 1355 steps, 0 errored trials |
| no-op (`--policy yam_scripted`; and the script's idle stage held to the run limit, 2400 steps) | 0.0, 0.0 | belt hangs on the joint pulley only |
| neg-no-tension | 0.25 | belt round both pulleys, plate never pushed (C 91.18, T 0) |
| neg-no-lock | 0.25 | tensioned, lever never turned: when the hand let go the plate slid back (T 1.6 N) |
| neg-hands-on | 0.75 | clamped in band (T 9.0 N) but the left hand stays on the motor (it lifts off once for 0.25 s and comes back, so the 3 s hold never completes; at the end it presses on the motor) |
| neg-drape | 0.0 | belt laid over the small pulley from above and let go, not worked round its flange: it stays draped across the top of the small pulley, partly outside its flanges, and never goes round it |
| reset check (`disturb,idle,demo` in one process; the disturbance pulls the belt off and drops it on the table) | 0.0, 0.0, then demo 1.0 | after each reset: score 0, every bench body and belt link at the first start's pose (0.0 mm), plate at the slack end, lever at 0.0 deg; the demo trial after the second reset reached 1.0 (1369 steps, T 10.6 N, lever -92.7 deg) |
| demo after a reset (`disturb,demo` in one process) | 1.0 | start 0.0 mm from the first start; 1363 steps, T 8.8 N, lever -83.2 deg |

Over-tension (above 12.4 N) is rejected by the scorer (unit test; in the demo trace the tension briefly reaches 12.8 N
while the lever is turned and the level drops to 0.25 until it is back under 12.4 N); a scripted over-tension stimulus
is not possible because one arm stalls at about 11.5 N in this pose. The two post-reset demo trials above reached 1.0;
as described under "Reference demo", that is not guaranteed for every start after a reset.

## Files
| File | Role |
|---|---|
| `isaac/task.py` | yam_arena task file (SCENE, OBJECTS, score/success/distance) |
| `isaac/beltkit.py` | bench spawn (frame, pulleys, motor plate, lever, 70-link belt) + runtime hooks (belt-tension model and its load on the plate, lever clamp rule, scorer) |
| `isaac/demo.py` | scripted demo and partial/negative variants (robot commands only, reads poses) |
| `isaac/official_run.py` | official `inspect-robots run` path: registers the scripted demo as closed-loop policy `timing_belt_scripted`, then calls the unmodified `inspect_robots.cli.main` |
| `isaac/run_frozen.sh` | evidence runner on a read-only source snapshot with SHA-256 manifest |
| `src/layout.py` | all dimensions, poses and belt shapes (start on the joint pulley, taut) |
| `src/belt_scoring.py` | scorer (pure functions) |
| `tests/test_scoring.py` | scorer unit tests (synthetic belt configurations) |

## Fidelity
Isaac Sim 6.1 / Isaac Lab-Arena yam_arena with PhysX on the CPU (rendering on the GPU), physics 480 Hz (scorer and belt-tension model every 8th physics step, 60 Hz), control 20 Hz, real YAM arms (URDF, implicit joint drives at their real effort limits, 50 N fingers). Real-size parts: GT2 belt of 280 mm pitch length modelled as 70 rigid 4 mm links (two real 2 mm-pitch trapezoid teeth each, 6 mm wide, 1.38 mm thick) joined at the pitch line by joints that only bend, within limits (twist 10, flat bending 70 and edgewise bending 2 degrees per joint; no bending spring); links weigh 0.1 g each (7 g belt, a real one weighs about 3 g); belt-on-pulley and belt-on-finger contact is plain PhysX contact (friction 0.8): nothing welds, snaps or attaches the belt. Pulleys: GT2 20T and 60T with 20 / 60 trapezoid teeth, real pitch, outside and flange diameters, turning on their shafts against 0.01 N m (joint pulley: shaft bearings) and 0.02 N m (motor pulley: the unpowered stepper's detent) of friction. Belt tension is a disclosed model, not the link chain (a chain of 0.1 g PhysX links stretches millimetres under a few newtons): the links give the belt's real shape, wrap and contacts, and the tension per span is computed from geometry as 10 N per mm of centre distance past 100.6 mm, and zero unless the belt is round both pulleys between their flanges. 100.6 mm is where the simulated belt is just taut with its teeth meshed on both pulleys (measured in the simulation: the link chain, with its joint compliance and its own weight, runs taut 1.4 mm later than the real belt's geometric 99.17 mm); teeth left riding on the tooth tips fail the seating check instead. This effective belt stiffness (EA about 0.7 kN) is far softer than a real glass-cord GT2-6 belt (about 25 kN), so that the belt's pull can be applied stably as an explicit force; the 6.2-12.4 N band therefore corresponds to 0.62-1.24 mm of plate travel past taut instead of a few hundredths of a millimetre. Because the link chain itself carries well under 1 N, its spans sag and vibrate by up to about 2.8 mm even when the modelled tension is in the band, so the simulated straightedge check allows a 3 mm gap (a person allows 1 mm). The matching reaction, 2 x the tension toward the joint pulley (capped at 60 N), is applied to the motor plate every physics step, so the robot has to push against the belt's load, and the scorer reads the same tension. Because drives on non-articulated PhysX joints are not honoured in this build, friction and the clamp are explicit forces too, applied every physics step (dry friction as a stick-slip spring anchor capped at the friction value; this also applies to the pulleys above): the motor plate slides along its 12 mm slot against 4 N of dry friction (finger-loose T-nut, an estimate); the clamp lever turns about its thread axis against 0.15 N m of friction, with stops; once it has been turned at least 75 degrees clockwise the plate is held where it is (stiff hold up to 300 N, the M5 clamp's thread force). These three (belt pull, friction, clamp) are the only rules standing in for materials; the clamp is triggered only by the robot physically turning the lever. Frame is static (bolted). Every scoring signal is measured in the simulation from poses (belt links, pulleys, plate, lever), the belt-tension model above and PhysX contact reports; the scorer never reads pixels.

## Third-party assets and licenses

The SDK is yam-isaac-lab-arena at revision 0059d02a1a91 (https://github.com/kairosiann/yam-isaac-lab-arena), the
program's official dependency, installed in the user's environment and loaded at run time. Nothing from it is
redistributed here. Licenses are given as each source states them.

| Asset | Used for | Source | License as stated by the source | Bundled |
|---|---|---|---|---|
| YAM arm and gripper (URDF, meshes, collision hulls, converted USD) | both arms | SDK `yam_arena/assets/yam/`, from i2rt (https://github.com/i2rt-robotics/i2rt, SOURCE_COMMIT 120c3c81400171174604e503943f8d1ebc891058) | MIT, Copyright (c) I2RT Robotics (SDK `LICENSE.i2rt`) | no |
| link2 / link3 meshes split into black and white parts | station arm colours | SDK `yam_arena/assets/yam/dream/`, from the dreamlabs 1.0.23 wheel | "MIT, see ../LICENSE.i2rt" (SDK `dream/SOURCE`) | no |
| Station floor and table | room | SDK `station.py` / `scene.py`: boxes with flat colours generated in code; no mesh or texture file | SDK README: "This repository has no license yet: most of it is ported from YAM-inspect-Isaac by Thomas Pierson, which has none." | no |
| Top-camera stand (foot, mast, arm, housing) | room | SDK `station.camera_stand`: boxes and capsules generated in code | as the row above | no |
| Dome light and lamp | lighting | SDK `scene.py`, Isaac Lab-Arena dome light and an Isaac Lab sphere light | as the row above | no |
| top_cam, left_cam, right_cam | policy RGB | SDK pinhole camera sensors at the calibrated poses | as the row above | no |
| Station fruit, bowl, plate (Arena object library) and water bottle | not used: this task's own objects replace them | - | - | no |
| Belt-drive bench (extrusion frame, joint plate, GT2 pulleys and belt, motor plate, clamp lever) | task objects | this folder (`isaac/beltkit.py`, `src/layout.py`): shapes generated in code | the authors' own geometry, MIT (this folder's `LICENSE`) | yes |

Isaac Lab-Arena (https://github.com/isaac-sim/IsaacLab-Arena, commit c7edf3a9d5d8) ships the Apache License 2.0.
Isaac Sim and Isaac Lab are runtime dependencies under their own licenses; none of them is redistributed here. No
third-party meshes or textures are used for the task objects. All task-specific code and geometry in this folder
(`isaac/`, `src/`, `tests/`) is the authors' own work under the MIT License.
