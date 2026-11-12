# Unlock, raise and pivot a monitor to portrait beside a box

Adjust an attached screen by releasing the appropriate spring index locks,
supporting its weight, avoiding an adjacent box, then relocking and letting go.
This models safe adjustment of a custom workstation monitor-stand trainer.
The three release buttons are identical and unmarked. The policy must discover
which axis each frees; their assignment is not supplied in the instruction.

Actual matching Isaac Lab-Arena/PhysX results at20Hz: Dream-1 scores0.0 after
1428 actions; the genuine YAM scripted controller scores1.0 after1204 actions,
including at least20s continuous hands-off. This is one fixed layout, not a claim
of model success or multi-seed robustness. The model trial uses one continuous
Dream server session,349 requests, no execution error. Required media linkage
is recorded in task.json after upload.

## Fixed scene and physical approximations

Use the complete setup in task.json and setup-inventory.json. The latter lists
all actual fixture primitives/body poses/mass properties and the live-validated
station furniture, including the table, camera stand and two physical arm
pedestals. Dimensions are metres unless marked otherwise. The desk top is
z0.7475; the YAM base origins are(0,+/-0.305,1.05), not the unraised station bases.
There is no layout jitter. CLI seed42 produces the recorded fixed setup; Inspect
also records its derived trial seed. No consumables are simulated.

Nominal materials: steel stand/locks/springs and arm mounts, plastic monitor
housing and buttons, a rigid grey box obstacle, laminate tabletop and metal
camera stand. Parameters are design estimates, not a measured commercial device.
Task objects are generated boxes/cylinders/capsules. The panel does not deform;
there are no screen cables, fatigue/fracture or yaw swivel. Pins interact with
lock teeth and retract through physical button linkages; geometric lock/stop
violations veto credit rather than modeling fracture.

The stand root and box are fixed in the simulator. A measured foot-reaction
wrench check enforces a resting280x220mm foot with coefficient0.5: loss of
compressive support, centre of pressure outside its physical footprint or an
unsupported coupled sliding/yaw reaction for20ms vetoes the trial. The footprint
rotates with the stand. This approximates a freestanding base; it is not a claim
that a world-fixed body is dynamically free.

## Runtime and dependencies

Use the official YAM/Arena environment with Python3.12, IsaacSim6.1.0.0,
Inspect Robots1.0.0, inspect-robots-yam0.38.0 and inspect-robots-dream0.1.0.
Pinned Arena release/0.3.1 commit:c7edf3a9d5d8276565208f72422f8f8a0806d6cf.
Pinned yam-isaac-lab-arena commit:0059d02a1a91bc6834c1d0846022891df0c78730.
The environment supplies numpy, scipy, Pillow and the SDK robot/station assets.
IsaacSim/Isaac Lab supply carb, omni and isaaclab_physx runtime modules.
Video encoding uses ffmpeg, or the existing imageio-ffmpeg executable.
No sibling project, native MuJoCo/Panda environment or downloaded task asset is
used by this task. See THIRD_PARTY_NOTICES.md and LICENSE.i2rt.

CPU PhysX at240Hz is required; use an allocated RTX-capable GPU for the actual
RGB cameras. This task's GPU-physics root-wrench reading failed a known-load
check; CPU physics passes the same loads. Final runs used an L40S/driver565.57.01.
Development-only driver-check overrides are opt-in and were not needed for the
final L40S trials. Never treat a black render or process exit0 alone as success.

The action/state is genuine14D joint_pos: left six revolute targets(rad), left
normalized gripper, then right six targets and right gripper. Gripper0 is closed,
1 is open. Existing joint/rate limits remain; each gripper is capped40N/finger.
Arm total PD plus gravity is limited27Nm on joints1-3 and7Nm on joints4-6.
The task-local adapter represents gravity as the internal drive target offset
G/K, with zero separate arm effort, so the actual PhysX drive cap bounds their
sum. The unsaturated controller is algebraically the same PD plus gravity.
Both arms' actual caps, known loads and80N/100ms abort were checked; a raw torque
telemetry estimate alone is not used to certify the bound.

Model observations contain only the actual top_cam,left_cam,right_cam RGB
streams and actual SDK joint/jaw state. They do not include fixture joints,
contact identities, scorer history or the reference controller. Scripted
planning explicitly reads simulator state and sends ordinary robot actions.
It never teleports a fixture, applies evaluator fixture loads, or writes score,
lock, damage or hold-timer flags. Ordinary episode reset and passive spring
configuration belong to the task runtime, not the policy.

## Third-party assets and licenses

The SDK is yam-isaac-lab-arena at 0059d02a1a91bc6834c1d0846022891df0c78730, the
program's official dependency, installed by the user's environment. Nothing from
it is redistributed here except its I2RT MIT notice (LICENSE.i2rt). Licenses are
given as each source states them.

| Asset | Used for | Source | License as stated by the source | Bundled |
|---|---|---|---|---|
| YAM arm and gripper (URDF, meshes, collision hulls, converted USD) | both arms | SDK `yam_arena/assets/yam/`, from i2rt (SOURCE_COMMIT 120c3c81400171174604e503943f8d1ebc891058) | MIT, Copyright (c) I2RT Robotics | notice only (LICENSE.i2rt) |
| link2/link3 meshes split into black and white parts | station arm colours | SDK `yam_arena/assets/yam/dream/`, from the dreamlabs 1.0.23 wheel | "MIT, see ../LICENSE.i2rt" (SDK `dream/SOURCE`) | no |
| Station floor and table | room | SDK `station.py`/`scene.py`: boxes with flat colours generated in code; no mesh or texture file loaded | SDK README: "This repository has no license yet" (Placeholder) | no |
| Top-camera stand (foot, mast, arm, housing) | room | SDK `station.camera_stand`: boxes and capsules generated in code | as the row above | no |
| Sky dome light (uniform colour, no HDR image) and lamp | lighting | SDK `scene.py`, using Isaac Lab-Arena's DomeLight and an Isaac Lab sphere light | as the row above | no |
| top_cam, left_cam, right_cam | policy RGB | SDK pinhole camera sensors at the calibrated poses; no camera mesh | as the row above | no |
| Station poses, sizes and colours | numbers in the SDK code | SDK README: "Dream Labs' calibrated YAM station, from Dream's MuJoCo replica (dreamlabs 1.0.23, `yam_mujoco`) and the calibration it follows (yamlab at `ec0455d`)" | license not stated by the SDK | no |
| Station fruit, bowl, plate (Arena object library) and water bottle | not used: this task's own OBJECTS replace them | - | - | no |
| Monitor, stand, buttons, box, robot pedestals | task fixture | this folder (`monitor.usda`, `task.py`, `kit.py`) | authors' own procedural geometry, submitted with this task | yes |

Isaac Lab-Arena release/0.3.1 (c7edf3a9d5d8276565208f72422f8f8a0806d6cf) ships
the Apache License 2.0 in its LICENSE.md. Isaac Sim and Isaac Lab are runtime
dependencies under their own licenses. None of them is redistributed here.

## Run the scripted demo

From this task folder inside the activated runtime, choose fresh output folders
outside the task source:

```bash
python -B run_demo.py --out /absolute/path/to/new-demo-output
```

It records issued14D commands, measurements, stages, actual RGB H264 streams,
source snapshots/hashes and result.json. Require full_goal_demonstrated=true,
runtime_result.score=1, no error.json, an action count<=1428, and successful
video finalization. The task.py registration demo() is a neutral hold; it is
not the full-score controller. The reference is state-aware and is not a model.

To record the same physical action replay with an extra evaluator overview
camera while retaining all three policy-camera views:

```bash
python -B record_overview.py --task-dir "$PWD" \
  --actions /absolute/path/to/new-demo-output/actions.json \
  --out /absolute/path/to/new-overview-output
```

This uses the SDK's built-in overview_cam. It changes no task object, robot
control, scoring rule or policy camera. A replay must independently reach1.0;
an action file alone is not success evidence. Video frames are actual renders,
played at20fps without added holds, labels or team identifiers.

## Run Dream-1 through Inspect Robots

Use the existing Dream authentication supported by the official environment;
never place credentials in this folder. Read the instruction directly from
task.json so the model and reference use the same wording.

```bash
instruction=$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')
python -B dream_continuity.py \
  --continuity-log /absolute/path/to/new-model-continuity.jsonl run \
  --embodiment yam_arena --policy dream \
  -P model=dreamlabs/dream-1-beta -P rows_per_request=4 \
  -E task="$PWD/task.py" -E device=cpu -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "$instruction" --scorer task_score \
  --max-steps 1428 --seed 42 --store-frames \
  --log-dir /absolute/path/to/new-model-output
```

The continuity helper rejects a missing/changed Dream server session; it does
not replace a model action. Inspect logs must report a completed, non-error
trial with actual scorer output. Task failure or score0 is allowed; an API or
internal execution error is not a measured zero-score trial. The recorded model
video includes the initial RGB frame plus1428 action frames at20fps(71.45s).
The action budget is1428/20=71.4 simulated seconds. The reference1204-action
run takes60.2 simulated seconds; observed wall times depend on hardware.

## Numeric scoring and reset

The exact formula and vetoes are in task.json,numeric_score.py and
telemetry_math.py. Height/pivot/tilt tolerances are strict(<6mm,<3deg,<3deg).
A qualifying hold requires height speed<20mm/s and angular speeds<0.02rad/s.
Only the final continuous hold counts; touching, leaving tolerance or moving
faster resets it. Missing or nonfinite channels fail closed. Reaching full score
earlier does not preserve it after invalid final state or a permanent veto.

For a physical reset, release each axis and restore lowest/landscape/untilted,
seat every pin, restore the box/stand to their marks and return both arms to
READY with open grippers. The simulator reset restores all nine fixture
positions/velocities and clears evaluator efforts before settling, then starts
a new measurement history. No past success, contact, damage or hold is inherited.

For simple real-world measurement, mark the130mm height on the column, measure
portrait/tilt with a protractor/inclinometer, and use two-angle video for the
last20s release, contact and motion. Mark the foot outline for slide/lift and
inspect pins/stops for forced motion. This source is an approximate simulator
trainer, not a claim that those physical trials have already been performed.
