# Feed NM cable through an outlet box's push-in clamp: past its sprung one-way tab, until 150 mm is inside, then let go

The task is the Isaac Lab-Arena (PhysX) `yam_arena` task in `isaac/`; `src/` holds the engine-independent modules it
imports (layout and cable geometry, scorer, fast IK) and `tests/` their unit checks.

## The task

Mount a black single-gang nail-on PVC box on the side of an upright stud with its open face toward the robot and its active cable entry in the top wall. A free end of intact 14/2 NM-B cable approaches from above. Feed the end past the integral tab, draw in 130–170 mm of cable, and release both hands. The one-way clamp makes excess feed hard to undo. This models feeding the cable before stripping it or making a connection.

## Scoring (`src/scoring.py`, primary score 0..1, value at the end of the trial)
- 1.0 (ends the trial): the cable passes through the entry opening with the clamp tab lifted onto it and at least
  20 mm past it, 130-170 mm of cable inside the box (measured along the cable from the entry wall's inner face to the
  cut end), every fingertip 120 mm or more from the cable, and no point of the 200 mm of cable at the cut end moving
  more than 5 mm over 1 s.
- 0.5: through the clamp, released and still, but the length inside is outside 130-170 mm.
- 0 otherwise (not through the clamp, cable still held, or moving).
Real trial: the tab resting on the sheath (photo), a tape measure along the cable from the clamp to the cut end, the
video for the release.

## Dream-1 evaluation

Run from `isaac/` in the configured YAM environment. The revised evaluation uses seed 0 and a limit of 3,600 actions, each with 14 values.

```bash
inspect-robots run --embodiment yam_arena --policy dream \
    -P model=dreamlabs/dream-1-beta -P rows_per_request=10 \
    -E task=task.py -E object_jitter=0 -E device=cpu -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 \
    --instruction 'Push the end of the cable into the outlet box through its cable clamp and pull it in until about 150 mm of cable is inside the box, then let go.' --scorer task_score --seed 0 --max-steps 3600 --store-frames --log-dir ./dream-output
```

## Run and score a trial
Inside the Isaac Sim 6.1 + Isaac Lab-Arena container, working directory `isaac/` (`isaac/run_frozen.sh` does this on a
frozen copy of `isaac/`, `src/` and `task.json`, writes `source-sha256.txt` and logs to `<out>/<run>/run.log`; see its
header for the environment variables it needs):

```
# official Inspect Robots run of the scripted reference demo (registered closed-loop policy nm_cable_box_scripted)
python official_run.py -- run --embodiment yam_arena --policy nm_cable_box_scripted -E task=task.py -E object_jitter=0 \
    -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --instruction "$(python -c 'import json;print(json.load(open("../task.json"))["instruction"])')" \
    --scorer task_score --seed 0 --max-steps 3600 --store-frames --log-dir <out>/official
inspect-robots video <out>/official/<log>.json --out <out>/official/video     # H.264 video of every camera

# plain do-nothing run (task.py demo() gives the yam_scripted policy no waypoints: both arms hold READY)
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py ... --max-steps 3600

# Direct diagnostic runs
python demo.py task.py <out>/<run> --stage demo --seed 1415158438
python demo.py task.py <out>/<run> --stage neg-blind-aim --seed 1415158438
python demo.py task.py <out>/<run> --stage neg-push-far --seed 1415158438
```
Unit checks (no simulator): `python -m pytest tests/`.
Tiled evidence video (top + left wrist + right wrist, same timing):
`isaac/tile.sh <out_dir> <name> demo_top_cam.mp4 demo_left_cam.mp4 demo_right_cam.mp4`.

## Upright seed-0 evaluation

The scripted reference earned 1.0 at step 1,338 of 3,600. At termination the box retained 130.62 mm of cable, the tongue angle was 42.28°, and the nearest fingertip was 273.09 mm from the cable. Dream-1 earned 0.0 in 3,600 steps with 357 model requests. Both official runs use the same upright fixture, scoring rules and three camera views.

## Proposed real setup

Secure a 38 × 89 × 490 mm stud upright. Nail the Allied Moulded P-181RK to its side with the open face toward the arms, the rim 13 mm proud of the stud face, and the upper entry 145 mm above the work surface. Place the front lip of the active opening 380 mm forward of the line between the two YAM base centres and nominally midway between them. The modeled tongue root lies 14.5 mm farther back inside the box. Apply the lateral seed offset from `src/layout.py` to the complete fixture.

Drape a 450 mm length of de-energized 14/2 NM-B cable over the upper timber support. Lay the upstream stock across the rounded end and leave the cut end hanging above the top entry with the gap and lateral offset of the chosen setup. For the seed-0 setup, set the centre of the rounded support 333 mm above the work surface, with its 33 mm radius end under the drape. The cut end starts 85 mm above the upper entry and 39 mm closer to the arms than the front lip. Begin with both arms in READY and the grippers open. The physical fixture has not been built or tested.

## Dimensions and physical measurements

The manufacturer specifies a 58.4 × 95.3 × 68.6 mm box body, a 54.9 × 91.7 mm face opening and 83.4 mm between device mounting axes. The end view shows two molded tongues at each end. The simulation places the active tongue root toward the back and its free edge toward the box face. [Allied Moulded P-181RK engineering drawing](https://alliedmoulded.com/wp-content/uploads/P-181RK.pdf)

The model uses a 13 × 14.5 mm opening and a 12.5 mm wide, 15.5 mm long, 2.5 mm thick active tongue. These dimensions approximate features whose usable clearance is undimensioned in the drawing. The modeled rest angle is 40°, with 20° spring preload and 0.15 N·m/rad stiffness. The peak simulated cable-to-tab contact force during the successful reference was 9.75 N. No physical measurements of the entry clearance or opening force are available. Differences between a purchased unit and the simulation remain unknown.

## Simulation fidelity

Isaac Lab-Arena runs CPU PhysX at 240 Hz, with 20 Hz control. The 30-link cable has an elastic-plastic bending model and a link mass floor for solver stability. Total simulated cable mass is about 0.13 kg. The tab rotates on a one-way spring, with a modeled retention force capped at 120 N. Gravity acts in the station frame. The reference policy reads the simulated state and sends the standard 14 robot commands. It uses the installed YAM collision meshes with Pinocchio 3.9.0 (Python package `pin`), Coal 3.0.2 and trimesh 4.11.1 to plan a grasp. The three cameras render the jacket and cut-end copper from the same measured rigid-body poses that the scorer reads. These display meshes have no collision or rigid-body components.

## Third-party assets and licenses
| Asset | Source | License |
|---|---|---|
| YAM arm URDF and meshes | [I2RT Robotics model source](https://github.com/i2rt-robotics/i2rt/tree/120c3c81400171174604e503943f8d1ebc891058) | MIT. The installed runtime records the model commit in `yam_arena/assets/yam/SOURCE_COMMIT`. |
| Station, cameras and YAM embodiment | [YAM Arena runtime used on Brev](https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730#license) | Installed dependency. Its README reports that the integration code has no declared license. |
| Isaac Sim 6.1 and Isaac Lab-Arena | NVIDIA and [Isaac Lab-Arena](https://github.com/isaac-sim/IsaacLab-Arena/blob/c7edf3a9d5d8276565208f72422f8f8a0806d6cf/LICENSE.md) | NVIDIA Isaac Sim license and Apache-2.0 for the Arena package, respectively. |
| Box, stud, clamp tab and cable geometry | generated in code by this task (`src/layout.py`, `isaac/cablekit.py`) | MIT (this repository, `LICENSE`) |
| `src/fastik.py` (YAM URDF kinematics) | this task's authors | MIT |
