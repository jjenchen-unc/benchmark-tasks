# Mount a zoom lens nose-down and keep it at 50 mm

Two YAM arms must pick a zoom lens that fits an overhead camera whose lens mount faces down, fit it from below, turn it until the bayonet pin locks, set the zoom to 50 mm and let go. The lens then hangs nose-down for the final 20 seconds. Two lenses fit the camera, but they have different zoom-ring friction: on the low-friction lens the front element slides under its own weight and the zoom creeps away from 50 mm, so the lens choice decides whether the setting holds.

## Results

| Run | Score | Steps |
|---|---:|---:|
| Dream-1 (dreamlabs/dream-1-beta) | 0.0 | 3,034 |
| Scripted demo | 1.0 | 2,285 |
| No-op (arms hold the ready pose) | 0.0 | 3,034 |

Scores are the live task score printed at the end of each run. The demo ends with success once the lens has been locked at 48–52 mm with no robot contact for 20 continuous seconds.

## Scene

Coordinates are in metres: x points away from the arm bases, y to the robots' left, z up. Every object starts at the same pose on every reset (no jitter).

- Table: 1.10 x 1.30 x 0.06 m, top at z = 0.7475, centred at (0.36, 0, 0.7175).
- Robots: two six-joint YAM arms with parallel grippers. Bases at (0, +0.305, 0.90) (left arm) and (0, -0.305, 0.90) (right arm), each on a rigid 200 x 200 x 152.5 mm pedestal. Joint effort limits including gravity: 27 N m shoulder joints, 7 N m wrist joints, 40 N per finger.
- Overhead camera (0.97 kg): 130 x 90 x 64 mm body with a 32 x 98 x 56 mm grip and an 80 mm diameter lens mount. It hangs upside down with the mount centre at (0.40, 0.28, 1.2475), facing straight down, about 50 cm above the table. It is fixed to a 40 x 40 x 582 mm post at (0.40, 0.45, 1.0385) and a 60 x 200 x 12 mm crossbar at (0.40, 0.35, 1.3235). Its bayonet locks after a 56.4 degree turn through a 0.32 N m click detent; a release button (13 mm diameter, 4 mm travel) sits beside the mount.
- Upright camera (1.26 kg): same body on a 70 x 70 x 60 mm stand, mount centre at (0.20, -0.28, 0.8775), facing up. Its bayonet has the other mount type (73.5 degree lock in the opposite direction).
- Three zoom lenses, each a 62 mm diameter x 50 mm barrel with a 58 mm diameter x 20 mm zoom ring and a 52 mm diameter x 20 mm front element that extends 20 mm as the zoom goes from 18 mm to 55 mm over a 90 degree ring turn. All start at 18 mm, standing mount-end down on the trays:
  - orange, 0.34 kg, at (0.37, -0.20): fits only the upright camera;
  - blue, 0.47 kg, at (0.27, 0.06): fits the overhead camera, zoom friction 0.070 N m;
  - purple, 0.45 kg, at (0.27, 0.20): fits the overhead camera, zoom friction 0.010 N m.
- Two foam trays, 120 x 260 x 20 mm, 0.62 kg each, at (0.37, -0.27) and (0.27, 0.13), lying on the table.

Hanging nose-down, the 0.15 kg front element pulls the zoom ring with about 0.019 N m. That exceeds the purple lens's friction, so its zoom creeps; the blue lens holds.

## Scoring

The scorer reads the simulator every physics step (500 Hz):

- 1.0: a compatible lens is seated on the overhead camera (gap at most 1 mm, centre offset at most 2.5 mm, tilt at most 4 degrees), captured and pin-locked within 4 degrees of the lock angle, and its focal length stays within 48–52 mm for the final continuous 20 seconds with no robot contact on any camera or lens.
- 0.5: the lens is seated and locked for that final 20 seconds, but the focal length leaves 48–52 mm.
- 0 otherwise. A seated but unlocked lens earns 0.
- Vetoes (score 0): a lens falling faster than 0.6 m/s while nobody holds it; more than 1.5 N m on a mount stop or pin for 50 ms; more than 80 N at a wrist for 100 ms.

The episode ends on success, on a veto or after 3,034 steps.

## Running

Requirements: Isaac Lab-Arena with the YAM embodiment and Inspect Robots (tested with Isaac Sim 6.1.0.0 and Isaac Lab 17.0.2), one NVIDIA GPU for rendering. Physics runs on CPU PhysX. Each command writes to a fresh output directory and records three 320 x 176 camera videos (`top_cam.mp4`, `left_cam.mp4`, `right_cam.mp4`), the per-step journal and `result.json` with the final score.

```sh
python run.py setup --out /tmp/zoom-setup              # reset only, writes setup.png
python run.py demo  --out /tmp/zoom-demo               # replays demo-actions.json
python run.py noop  --out /tmp/zoom-noop               # 3,034 steps of the ready pose
python run.py replay --actions dream1-actions.json --out /tmp/zoom-dream1
python run.py probe --lens lens_purple --out /tmp/zoom-probe-purple   # see "How the hidden property can be observed"
```

The last line of each run prints the final score, step count and termination reason. The demo takes about 10 minutes and a full 3,034-step run about 11 minutes on an L40S GPU.

The Dream-1 run uses the standard entry point:

```sh
inspect-robots run --embodiment yam_arena --policy dream -P model=dreamlabs/dream-1-beta -P rows_per_request=4 \
  -E task=sim/scene/task.py -E control_hz=20 -E device=cpu -E object_jitter=0 -E cam_width=320 -E cam_height=176 \
  --instruction "$(python -c 'import json;print(json.load(open("task.json"))["instruction"])')" \
  --scorer task_score --max-steps 3034 --seed 42 --no-prompt
```

For the recorded Dream-1 run, gripper values returned slightly outside [0, 1] were clipped to [0, 1]; arm values were passed unchanged. `dream1-actions.json` holds the 3,034 actions that were executed, and replaying them gives the same score.

## Demo

`demo-actions.json` is the recorded action log of the scripted controller in `sim/scene/reference.py` (`python run.py reference --out ...` runs it live). It is open loop from the published layout and uses the blue lens. At run time it reads only object and robot poses (what a camera sees), the robot's own joint positions and the step count; it never reads friction or other parameters, contact or wrist forces, or the task's internal capture and pin state. It picks the blue lens up with the right arm, hands it to the left arm, flips it nose-down, pushes it onto the overhead mount and turns it until the mount ring, measured from poses, has turned at least 57.15 degrees (the published 56.4 degree lock plus 0.75). It then turns the zoom ring to 50 mm using the ring angle measured from poses, lets go and moves both arms away for the final 20 seconds. It acts only through the 14-number joint-and-gripper action at 20 Hz, under the same limits and scorer as the model; the replay sends exactly those recorded actions.

## How the hidden property can be observed

Which lens holds 50 mm is not printed on the lenses; it shows up when a lens is tried. `python run.py probe --lens <lens> --out ...` (`sim/probe.py`) swaps that lens into the blue lens's start position (a pose write done by the probe script only, never by the demo), runs the same route as the demo, checks the lock from poses, sets 50 mm, lets go and watches the zoom ring for 5 seconds. Measured on the reference runtime:

| Lens | Mount ring turn at lock check | Lock (>= 57.15 degrees) | Zoom after setting | Zoom when the hand has let go (2 s) | Change over the next 5 s |
|---|---:|---|---:|---:|---:|
| blue | 57.20 degrees | yes | 49.95 mm | 48.98 mm | 0.00 mm |
| purple | 57.20 degrees | yes | 50.00 mm | 55.00 mm (end stop) | 0.00 mm |
| orange | not reached | not measured | - | - | - |

The difference is large and fast: the purple lens's zoom runs from 50 mm to its 55 mm end stop (front element fully out, 20 mm) within the 2 seconds in which the gripper opens, while the blue lens stays at 49 mm. Either is visible in the cameras as the position of the zoom ring and front element. The orange probe did not reach the mount: in this route the right hand lost its grasp on the orange lens during the first flip, so the fit of the orange lens was not measured by this script. By construction the overhead mount has no capture for the orange lens's mount type, so it cannot lock there.

## Reset

Every reset restores the same layout: object poses, the ten task joints (two bayonet rings, two release buttons, three zoom rings, three front elements), ready pose with open grippers, unlocked mounts and 18 mm zoom. The scene settles before the score history starts.

## Physical approximation

Rigid PhysX bodies with estimated masses and friction; no optics are simulated. The zoom ring and front element are coupled by a stiff spring along the zoom helix, so gravity on the front element turns the ring against its friction. Bayonet engagement is a fixed joint created when a compatible lens is pressed, aligned, onto the mount, and the pin locks when the ring reaches the lock angle; pulling with more than 2 N for 24 ms on an unlocked lens releases it. The overhead camera support and the robot pedestals are rigid fixtures.

## Neighbouring tasks

Camera assembly, optical-bench placement and bayonet locking already exist on the board (for example a bayonet cap lock and a multistage bayonet sleeve). This task combines a compatibility choice, inverted mounting, a contact bayonet lock and gravity-dependent zoom retention after release. LIBERO, SimplerEnv, RoboCasa, BEHAVIOR (camera on a tripod), DataCenterBench and RoboDojo have no lens-mount or zoom-retention task. Canon's lens instructions describe the same steps: align the index marks, turn to the lock, then set the zoom ring.

## Files

- `run.py`: entry point for the commands above.
- `sim/runtime.py`: runs one episode through the Inspect Robots YAM embodiment and records the cameras.
- `sim/scene/task.py`: Isaac Lab-Arena task file (objects and live score).
- `sim/scene/mechanics.py`, `numeric_score.py`: score and score history; `b6_physics.py`, `kit.py`, `contacts.py`, `force_vectors.py`, `unique_pin.py`, `reaction_frame.py`, `motor_limits.py`: bayonet, zoom, contact and motor-limit rules.
- `sim/scene/fixture.json`, `setup-inventory.json`, `assets/*.usda`: geometry, masses, joints and materials; `build_usd.py` rebuilds the USD files.
- `sim/scene/reference.py`, `endpoint_control.py`, `tracking.py`: the scripted controller that produced the demo.
- `sim/probe.py`: the lens probe described above.

All task geometry is generated from primitives in this folder. The YAM robots, the table and the video cameras come from the installed YAM Isaac Lab-Arena package under its own license.
