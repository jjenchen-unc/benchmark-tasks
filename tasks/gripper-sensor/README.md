# Set a pneumatic gripper's position switch on the real part: hold the valve override, slide it to where its LED lights

The submission is the Isaac Lab-Arena (PhysX) `yam_arena` task in
`isaac/`; `src/` holds the engine-independent modules it imports (layout, scorer, fast IK) and `tests/` their unit
tests.

## The task
A double-acting pneumatic parallel gripper, the end effector a robot will carry, lies on its back on a flange test
stand, bolted by its mounting face. Its jaws carry two aluminium fingers; a steel test part sits in a nest between
them. One of the body's two auto-switch grooves faces up; a solid-state auto switch sits loose in it, somewhere along
the groove. The gripper is plumbed to a 5/2 single-solenoid valve beside it. The valve's manual override is the
non-locking push type: the valve switches, and the jaws close, only while it is held down. A 2.5 mm precision
screwdriver stands in a rack.

The robot must set the switch so it signals "part gripped": LED on when the jaws close on the part, off when they are
open, and off when they close with no part (a switch at the end-of-stroke position would also light for an empty
grip). It holds the override down with one hand, so the jaws stay closed on the part, and with the other slides the
switch along its groove by pressing on its top, watching the LED. The switching window is about 2 mm long. Then it
releases the override, and the jaws open and the LED goes out. Last, it tightens the switch's set screw with the
screwdriver.

Why it matters: every pneumatic gripper on a robot gets its jaw-position switches set on the real part when the robot
is built or re-tooled. The robot's program relies on them to know whether it has the part. The test part and the
switch's start position change every trial.

## Scoring (`src/scoring.py`, primary score 0..1, value at the end of the trial)
The scorer cycles the gripper as a person would (the switch position, part width and screw state determine it):
- 0.5: LED on with the jaws closed on the part, and off with the jaws open.
- 0.75: also off with the jaws closed empty (it detects the part, not the end of stroke).
- 1.0 (ends the trial): also on with tolerance blocks 1.2 mm narrower and 1.2 mm wider than the part (the switch sits
  within 0.4 mm of the middle of its switching window, which is where switch makers advise setting it), and locked:
  the set screw turned 90 deg clockwise with the screwdriver's blade in it and pushed in, so a 5 N push along the
  groove does not move the switch; the override released, nothing touching the switch, and both hands and the
  screwdriver 8 cm or more from it, for 1 s.
- 0 otherwise, including a switch locked in the wrong place.
Start variation and blind strategies: per trial the part width (window position along the piston travel), the
switch's start and where along its housing the switch senses (hidden; up to 10 mm either side of the middle) are drawn.
Computed with the scorer over 10,000 seeds, the best single fixed position for the switch housing scores 1.0 on 4.3 %
of seeds and 0 at the default seed; a fixed slide from the start scores 1.0 on 3.4 %; leaving the switch where it
starts or setting it at the end-of-stroke point scores 0.
Real trial (person check): cycle the valve with the part in, open, with the part removed and with the two tolerance
blocks, and watch the LED; then push the switch along the groove with a 5 N push-pull gauge.

## Dream-1 evaluation

The Inspect Robots run used model `dreamlabs/dream-1-beta` and returned a task score of 0.0 at the 2,400-step limit. The scripted reference run scored 1.0.
From the task's `isaac/` directory in the Isaac Sim 6.1 and Isaac Lab-Arena environment, run:

```bash
instruction=$(python -c 'import json; print(json.load(open("../task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
    -P model=dreamlabs/dream-1-beta -P rows_per_request=10 \
    -E task=task.py -E object_jitter=0 -E device=cpu -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
    --scorer task_score --seed 0 --max-steps 2400 --store-frames --log-dir ./dream-output
```

## Run and score a trial
Inside the Isaac Sim 6.1 + Isaac Lab-Arena container, working directory `isaac/` (`isaac/run_frozen.sh` does this on a
frozen copy of `isaac/`, `src/` and `task.json`, writes `source-sha256.txt` and logs to `<out>/<run>/run.log`; see its
header for the environment variables it needs):

```
# official Inspect Robots run of the scripted reference demo (registered closed-loop policy gripper_sensor_scripted)
python official_run.py -- run --embodiment yam_arena --policy gripper_sensor_scripted -E task=task.py \
    -E object_jitter=0 -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --instruction "$(python -c 'import json;print(json.load(open("../task.json"))["instruction"])')" \
    --scorer task_score --max-steps 2400 --store-frames --log-dir <out>/official
inspect-robots video <out>/official/<log>.json --out <out>/official/video     # H.264 video of every camera

# plain do-nothing run (task.py demo() gives the yam_scripted policy no waypoints: both arms hold READY)
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py ... --max-steps 2400

# direct demo / negative stages (trace.jsonl, result.json, demo_<camera>.mp4, setup.png, final.png)
python demo.py task.py <out>/<run> --stage demo                 # named setup 0; GS_SETUP=k for setup k
python demo.py task.py <out>/<run> --stage demo --seed 7        # a seeded start state, as in inspect-robots runs
python demo.py task.py <out>/<run> --stage neg-end-of-stroke    # switch set at the empty-jaw (end-of-stroke) point, locked: 0
python demo.py task.py <out>/<run> --stage neg-no-override      # override never worked, switch slid 6 mm blind, locked: 0
python demo.py task.py <out>/<run> --stage neg-no-lock          # set correctly, never locked: 0.75
python demo.py task.py <out>/<run> --stage neg-light-touch      # screwdriver turned resting on the screw, not pushed in: never locks
GS_REPLAY=<run>/trace.jsonl python demo.py task.py <out>/<run2> --stage replay --seed 7   # open-loop replay
python demo.py task.py <out>/<run> --stage idle,demo            # reset check (several trials in one process)
```
Unit checks (no simulator): `python -m pytest tests`.
Tiled evidence video (top + left wrist + right wrist, same timing):
`isaac/tile.sh <out_dir> <name> demo_top_cam.mp4 demo_left_cam.mp4 demo_right_cam.mp4`.

## Recorded media

- `model_run.mp4` shows the full Dream-1 trial (seed 0, 2,400 steps, final task score 0.0). The three views run at 20 Hz in order from left to right: overhead, left wrist, right wrist.
- `official_tiled_top_left_right.mp4`: the official Inspect Robots run of the scripted reference policy at the default
  seed (part 40.9 mm wide; the switch starts 7.8 mm above its switching point, which sits near the back of its
  housing), score 1.0 after 1,273 steps (H.264, 960 x 176). The right arm ends at its home pose holding the
  screwdriver, so the last frame shows the gripper.
- `setup_overhead_hires.png`: the start state at 1280 x 704 (named setup 0).

Start states: every trial reset with a seed (every `inspect-robots run` passes one) draws the per-jaw travel to the
part uniformly from 2.5 to 12.5 mm (part 35 to 55 mm wide) and the switch's start position from -12 to +24 mm along
the groove, at least 3.5 mm from the part's switching point; the default inspect-robots seed gives a 40.9 mm part, a
start 7.8 mm from the switching point and a sensing point 9.3 mm behind the housing's middle. Unseeded resets (direct `demo.py` runs) use five named setups in turn (`GS_SETUP` picks the
first; set, it also forces named setups on seeded resets).

## Fidelity (full statement)
- Engine: Isaac Sim 6.1 / Isaac Lab-Arena `yam_arena`, PhysX on the CPU, physics 480 Hz, control 20 Hz, cameras
  320 x 176. All parts are full size.
- Part data: the parts are modelled on an SMC MHZ2-40D gripper, a D-M9N auto switch and an SY3120-5LZ-M5 valve.
  The values used (30 mm total stroke, a switch about 22 x 3 x 4 mm with a red LED and an M2.5 slotted set screw, a
  non-locking push manual override) come from the manufacturer's catalog as recalled by the authors. They could not
  be checked against the catalog while the task was built, and body outlines are representative. Check them against
  the current SMC catalog before a real trial. The piston-magnet travel is assumed equal to each jaw's travel, and the
  switch's operating window is taken as 2 mm; both are estimates. A real switch also has about 0.5 to 1 mm of
  hysteresis, which is not modelled.
- Static parts (stand, gripper body, valve, rack, part nest) are one kinematic rigid body. The jaws and fingers, the
  test part, the override button, the switch and its set screw are kinematic bodies moved only by the device model
  below. The screwdriver is a free rigid body (30 g), and nothing writes its pose during a trial.
- Valve and jaws: the valve is switched while the robot pushes the override button with 5 N or more, and drops out
  below 3 N (non-locking override). The button does not travel; a real one travels about 1 mm. While the valve is
  switched, both jaws close at 0.1 m/s each and stop on the part; otherwise they open. This is a position-controlled
  device. The part is kinematic, so the jaws stop where its faces are (no grip-force model).
- Switch: it moves along the groove only when the contact forces on it and its screw along the groove exceed its
  static friction. Those forces come from the robot or the screwdriver, normal and friction (PhysX contact and
  friction reports). The static friction is 2.0 N, and the switch keeps sliding above 1.8 N. A tightened switch holds
  30 N. These values are estimates.
- Set screw: its end sits 0.8 mm down inside a 50 deg conical lead-in (a capture cone, 4.8 mm across at the top;
  stands in for the slot that holds a slotted blade, whose orientation is ignored). It turns only while the
  screwdriver is engaged: the blade tip is down in the cone (within 0.5 mm of the screw axis), pushed in with 1.5 N or
  more along the axis (contact with the screw's colliders), and the tool is within 12 deg of the axis. Once engaged it
  stays engaged while the tip stays on the screw (within 1.6 mm) and the push does not drop below 1.5 N for more than
  0.15 s. The screw then turns with the tool's own rotation about the axis; 90 deg clockwise locks the switch. There
  is no torque model (the real screw needs about 0.1 N m, small for the wrist).
- LED: an emissive patch on the switch, lit while the switch's sensing point is within 1 mm of the piston magnet,
  updated every control step.
- Hidden sensing point: per trial the switch's sensing point is placed up to 10 mm either side of the middle of its
  housing (a simulation device; one real switch model has one fixed sensing point). It stands for the fact that the
  switching spot cannot be read off the housing and forces the LED-guided search. A real rig gets the same effect by
  rotating among several switch makes / lengths whose sensing points differ.
- Scoring reads the switch position, part width and screw state (`src/scoring.py`). The scorer cycles the gripper
  analytically, the same three checks a person makes with the valve.
- Grip and push forces: the reference demo grips the screwdriver handle with about 12 N per finger (fingers commanded
  2.5 mm inside the handle against the 5000 N/m finger drive), presses the override with about 8 N (15 N peak) and the switch
  with 5 to 9 N, all far under the YAM's 40 N.
- The scripted demo only sends the 14 joint/gripper commands. It reads simulator state through
  `grippkit.bench_state()` (read-only): the LED, the jaws, the screwdriver pose and the switch position in the piston
  coordinate. That coordinate includes the hidden sensing-point offset, which a real operator cannot see; the demo
  uses it only to choose where its LED scan starts and ends and to place the screwdriver. The window itself is found
  from where the LED comes on and goes off. On an environment reset (between trials only) the kit puts the switch,
  screw, jaws, part and screwdriver back to the seed's start state.

## Real rig
- Gripper: SMC MHZ2-40D (double acting, 40 mm bore, 30 mm stroke) with two aluminium finger blanks 50 x 12 x 26 mm
  (60 mm clear gap open). It is bolted on its back to a 10 mm aluminium flange plate (ISO 9409-1-50 adapter) on a
  150 x 130 mm base plate, auto-switch groove facing up.
- Auto switch: SMC D-M9N (or D-M9P) in the upper groove, set screw backed off, at the start position from the seed.
  Grey cable led away along the groove. The switch's top stands about 1 mm proud of the body face.
- Valve: SMC SY3120-5LZ-M5 on a single sub-base on a bracket 13 cm to the robot's left of the gripper, with 4 mm
  tubing to the gripper ports, 0.5 MPa supply, coil unpowered (the manual override is used).
- Test part: a ground steel block 30 x W x 20 mm in a nest between the fingers. Per trial W is 35 to 55 mm (a set of
  blocks in 1 mm steps covers the range).
- Screwdriver: a 2.5 mm precision slotted screwdriver (40 mm blade, 15 x 85 mm handle), standing blade-down in a rack.
- Set-screw recess: a 4.8 mm countersink (0.8 mm deep) around the switch's set screw guides the blade; on a real switch
  the slot does this. A real rig can glue a small countersunk washer round the screw.
- Cost: under a few hundred USD (gripper, switch, valve, fittings, blocks, screwdriver).

## Third-party assets and licenses
| Asset | Source | License |
|---|---|---|
| YAM arm model (URDF, meshes) | i2rt, via the Isaac Lab-Arena YAM package | MIT |
| Station scene, cameras, `yam_arena` embodiment | Isaac Lab-Arena YAM package | as distributed with the package (no license file at the pinned revision) |
| Isaac Sim 6.1 / Isaac Lab-Arena | NVIDIA | NVIDIA Isaac Sim license / Apache-2.0 |
| Gripper, switch, valve, part, screwdriver, stand geometry | generated in code by this task (`src/layout.py`, `isaac/grippkit.py`) | MIT (this repository, `LICENSE`) |
| `src/fastik.py` (YAM URDF kinematics) | this task's authors | MIT |
