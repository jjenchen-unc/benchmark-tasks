# Set a pneumatic gripper's position switch on the real part: hold the valve override, slide it to where its LED lights

Scripted reference 1.0 in 1,242 control steps. Dream-1 (seed 0) scored 0.0 at the 2,400-step limit. Locking the switch at empty closure failed the gauge checks and scored 0.0.

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
switch along its groove by pressing on its top, watching the LED. The simulated switching window is 2 mm long. Then it
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
  within 0.4 mm of the middle of its switching window, the centre used by this scorer), and locked:
  the set screw turned 90 deg clockwise with the screwdriver's blade in it and pushed in, so a 5 N push along the
  groove does not move the switch; the override released, nothing touching the switch, and both hands and the
  screwdriver 8 cm or more from it, for 1 s.
- 0 otherwise, including a switch locked in the wrong place.
The reset chooses a block width from 36, 40, 44, 48 and 52 mm and varies the starting position of the switch in the groove. The sensing point stays fixed in the housing. A single fixed position can pass at most one of the five nominal widths.
Real trial (person check): cycle the valve with the part in, open, with the part removed and with the two tolerance
blocks, and watch the LED; then push the switch along the groove with a 5 N push-pull gauge.

## Dream-1 evaluation command

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

## Recorded media for this revision

The media package contains `model_run.mp4` (Dream-1), `official_tiled_top_left_right.mp4` (scripted reference) and `setup_overhead.png` (the original overhead frame before the first reference action). The videos show overhead, left wrist and right wrist at 20 Hz in a single 960 × 176 frame.

Each reset selects a 36, 40, 44, 48 or 52 mm nominal block and varies the switch starting location. All five setups use the same D-M9N with the cable to the rear. The non-cable face is the datum for the fixed 6 mm sensing point.

## Fidelity (full statement)
- Engine: Isaac Sim 6.1 / Isaac Lab-Arena `yam_arena`, PhysX on the CPU, physics 480 Hz, control 20 Hz, cameras
  320 x 176. All parts are full size.
- Part data: the parts are modelled on an SMC MHZ2-40D gripper, a D-M9N auto switch and an SY3120-5LZ-M5 valve.
  SMC specifies 30 mm total jaw stroke and shows the D-M9N body and sensing point in the dimensional drawing cited below. Body outlines and the manual-override force curve remain approximate. The model assumes 1:1 jaw-to-magnet travel, a symmetric 2 mm LED window and no hysteresis. Record real switching points on approach and return before comparing scores.
- Static parts (stand, gripper body, valve, rack, part nest) are one kinematic rigid body. The jaws and fingers, the
  test part, the override button, the switch and its set screw are kinematic bodies moved only by the device model
  below. The screwdriver is a free rigid body (30 g), and nothing writes its pose during a trial.
- Valve and jaws: the valve is switched while the robot pushes the override button with 5 N or more, and drops out
  below 3 N (non-locking override). The button does not travel; the real button has mechanical travel. While the valve is
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
  0.15 s. The screw then turns with the tool's own rotation about the axis; 90 deg clockwise locks the switch. The simulation accumulates yaw without a screw torque model.
- LED: an emissive patch on the switch, lit while the switch's sensing point is within 1 mm of the piston magnet,
  updated every control step.
- The inline D-M9N has one fixed sensing point. The simulated two-millimetre LED window is a nominal approximation; verify the turn-on and turn-off locations on the assembled rig.

- Scoring reads the switch position, part width and screw state (`src/scoring.py`). The scorer cycles the gripper
  analytically, the same three checks a person makes with the valve.
- Grip and push forces: the reference demo grips the screwdriver handle with about 12 N per finger (fingers commanded
  2.5 mm inside the handle against the 5000 N/m finger drive), presses the override with about 8 N (15 N peak) and the switch
  with 5 to 9 N, all far under the YAM's 40 N.
- The scripted demo only sends the 14 joint/gripper commands. It reads simulator state through
  `grippkit.bench_state()` (read-only): the LED, the jaws, the screwdriver pose and the switch position in the piston
  coordinate. That coordinate includes the fixed sensing-point offset, which a real operator cannot see; the demo
  uses it only to choose where its LED scan starts and ends and to place the screwdriver. The window itself is found
  from where the LED comes on and goes off. On an environment reset (between trials only) the kit puts the switch,
  screw, jaws, part and screwdriver back to the seed's start state.

## Real rig
- Gripper: SMC MHZ2-40D (double acting, 40 mm bore, 30 mm stroke) with two aluminium finger blanks 50 x 12 x 26 mm
  (60 mm clear gap open). It is bolted on its back to a 10 mm aluminium flange plate (ISO 9409-1-50 adapter) on a
  150 x 130 mm base plate, auto-switch groove facing up.
- Fit one inline SMC D-M9N NPN switch, cable end toward the back. Its drawing gives 22.8 mm body length, an M2.5 flat-point mounting screw and a sensing point 6 mm from the non-cable end. Use that end as the reference for placement. Connect brown to regulated +24 VDC, blue to 0 V, black to a sourcing PLC input with compatible common and current below the 40 mA output rating. Check the built-in red LED at both switch transitions. [SMC switch manual, section 4-3](https://www.smcworld.com/assets/manual/en-jp/files/MHx-OMY0019.pdf)
- Use the MHZ2-40D at 0.5 MPa through a regulator and the single-solenoid, spring-return body-ported SY3120-5LZ-M5 on a mounting bracket. Connect supply to P, outputs A/B to the two gripper ports so holding the manual override closes the fingers, and exhaust through EA/EB. The single valve has M5 working ports and a 24 VDC coil. Leave the coil unpowered for manual trials. [SMC valve specification](https://www.smcworld.com/webcatalog/s3s/en-jp/detail/?partNumber=SY3120-5LZ-M5). With the test block removed, verify that release opens the fingers and full closure gives the empty position. The catalogue lists 30 mm total stroke and gives separate switch hysteresis data. Record the actual LED on/off positions in both directions on the chosen unit. [SMC MHZ catalogue](https://ca01.smcworld.com/catalog/BEST-5-4-en/pdf/4-p0417-0485-mhz_2_en.pdf)
- Machine or surface-grind steel blocks 30 × W × 20 mm. For each nominal W of 36, 40, 44, 48 and 52 mm, prepare a nominal block and two verification blocks with W−1.2 and W+1.2 mm. Measure the central contact faces with a micrometer to ±0.05 mm and label the end faces. Score by actually cycling all three blocks for the selected nominal size.
- Screwdriver: use a watchmaker’s flat blade that fits the mounted screw. SMC specifies a 5–6 mm grip diameter for manual tightening. The simulated tool has a 2.5 mm cylindrical tip and a 15 mm handle for robot grasping.
- Use the stock slotted screw and its native recess for the hardware comparison. Record the blade and slot width and the mounting groove used.
- Include the gripper, switch, valve, fittings, fifteen blocks and screwdriver in the rig budget.

## Hardware calibration and screwdriver engagement

SMC's D-M9 drawing places the most sensitive point 6 mm from the non-cable nose on the 22.8 mm body. For the basic double-acting MHZ2-40D, the dimensioned drawing shows 60 mm open, 30 mm closed and a rear Ø42 H9 locating register 4 mm deep. Locate the adapter from that register and the rear mounting threads. SMC lists 0.9 mm of switch hysteresis for the MHZ2-40 and D-M9 family. This value describes the difference between switch-on and reset positions. The simulator uses a 2 mm active interval and identical transitions in both directions. On the real gripper use the round mounting groove for direct insertion, or fit the BMG2-012 bracket if using the square groove. Supply a separate valve mounting bracket. The SY3120-5LZ-M5 has a non-locking override, M5 × 0.8 working ports and a 24 VDC coil. The valve drawing specifies two Ø3.2 mm mounting holes and two M3 × 0.5 bracket threads 3.5 mm deep. The body outlines and override travel in the scene are simplified. Use the component drawings for machining the mounting plates. [D-M9 drawing, §4-3](https://www.smcworld.com/assets/manual/en-jp/files/MHx-OMY0019.pdf) · [SMC gripper drawings and switch data, pp. 454, 481–483](https://www.smcworld.com/catalog/BEST-5-4-en/pdf/4-p0417-0485-mhz_2_en.pdf) · [SMC valve configuration](https://www.smcworld.com/webcatalog/s3s/en-jp/detail/?partNumber=SY3120-5LZ-M5) · [SMC valve drawing, p. 1-4-12](https://content2.smcetech.com/pdf/SY.pdf)

The shop needs fifteen blocks. All widths are in mm. The last column is the simulator’s travel from fully open on one jaw, calculated from the 60 mm open gap.

| Nominal | Narrow | Wide | Per-jaw travel |
|---:|---:|---:|---:|
| 36 | 34.8 | 37.2 | 12 |
| 40 | 38.8 | 41.2 | 10 |
| 44 | 42.8 | 45.2 | 8 |
| 48 | 46.8 | 49.2 | 6 |
| 52 | 50.8 | 53.2 | 4 |

For each of the five nominal blocks, hold the jaws on the block. Use an indicator along the groove and record housing position at both LED transitions on a forward sweep and again on the return. Record the four positions and repeat the sweep three times. Set the switch inside the common lit interval, then cycle the nominal, narrow and wide blocks, empty closure and open jaws. Save the LED truth table with the measured block widths. A nominal 2 mm window alone cannot establish which physical blocks pass.

The simulation accepts any initial yaw when the tool is within 0.5 mm radially and 12° of vertical and applies at least 1.5 N to the screw. The reference engaged at 0.294 mm, 0.92° and 3.16 N. The cone and the ignored blade yaw materially ease **initial alignment**. Even an ideal zero-thickness 2.5 mm flat blade across the scene's 0.5 mm visual slot needs yaw within arcsin(0.5/2.5), about 11.5° of parallel modulo 180°. A centred perpendicular blade fits the simulated entry rule but spans five times the drawn slot width. For an available bench comparator, the Aspen DIN 551 M2.5 flat-point screw drawing lists a 0.4 mm nominal slot. For a centred zero-thickness blade with 2.5 mm of mating span, the corresponding bound is about 9.2°. SMC identifies the supplied screw as slotted M2.5 × 4 but gives no slot-width tolerance in the switch drawing. Measure the supplied screw before assigning it that bound. [Aspen comparator drawing](https://www.aspenfasteners.com/content/pdf/Metric_DIN_551_spec.pdf)

To measure the real effect, use the stock recess, mark the screw head and repeat centred insertions at 0°, 15°, 45° and 90° from the measured slot, using the same axial load and starting height. Record seating, slip and visible screw rotation separately. Score the real lock with the specified 5 N retention check. Physical LED transitions and hardware engagement measurements are still outstanding.

## Third-party assets and licenses
| Asset | Source | License |
|---|---|---|
| YAM arm model (URDF, meshes) | i2rt, via the Isaac Lab-Arena YAM package | MIT |
| Station scene, cameras, `yam_arena` embodiment | Isaac Lab-Arena YAM package | as distributed with the package (no license file at the pinned revision) |
| Isaac Sim 6.1 / Isaac Lab-Arena | NVIDIA | NVIDIA Isaac Sim license / Apache-2.0 |
| Gripper, switch, valve, part, screwdriver, stand geometry | generated in code by this task (`src/layout.py`, `isaac/grippkit.py`) | MIT (this repository, `LICENSE`) |
| `src/fastik.py` (YAM URDF kinematics) | this task's authors | MIT |
