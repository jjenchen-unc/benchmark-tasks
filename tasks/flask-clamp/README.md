# Load a culture flask into an orbital shaker's spring clamp: push its heel past all five fingers until the bottom is down

The task is the Isaac Lab-Arena (PhysX) `yam_arena` task in `isaac/`; `src/` holds the engine-independent modules it
imports (layout and clamp geometry, seat rule, shake test and scorer, fast IK) and `tests/` their unit checks.

## The task
A low-profile benchtop orbital shaker stands on the bench in front of the arms. Its 305 x 305 mm platform carries four
stainless spring-finger clamps for 250 mL Erlenmeyer flasks (2 x 2, 125 mm apart). Three clamps already hold capped
culture flasks; one of the two near clamps is free. An empty 250 mL flask with a foil cap stands on the bench beside
the shaker, on the free clamp's side. The robot grips the flask by its neck, lifts it over the platform's edge, brings
it to the free clamp from the open side (not over the other cultures), centres it over the clamp and pushes it
straight down until its bottom is on the clamp's base plate and the fingers have closed round its body, then lets go.
The scorer then switches the shaker on for a short test.

Why it matters: every overnight bacterial or yeast culture is grown in a shaken flask, and on most shakers the flask
is held by exactly this kind of clamp. Loading flasks is a daily bench step in microbiology, molecular biology and
bioprocess labs.

What makes it hard:
- The flask's heel (85 mm) is wider than the gap between the five finger pads (60 mm free), so the flask has to go
  straight down past all five pads at once: the heel spreads the fingers on their flared tips, passes the pads and
  lands on the plate, and the fingers spring back onto the cone.
- A flask only set into the flared tips stands upright on them and looks loaded, but it sits about 5 cm above the plate
  and is not held: once the shaker runs it tips over within about a second (the visible cue: the gap under the flask,
  and it stands about 5 cm taller than the loaded flasks beside it).
- The flask is held only by its narrow glass neck: a parallel grip on the neck lets it pivot about the grip axis, and
  the arm is compliant, so pushing too fast or off the clamp's axis tips the flask onto the fingers of one side, where
  it jams half way in. The push has to be paced by how the flask goes down and kept on the clamp's axis.
- The free clamp sits between loaded culture flasks (40 mm between the flask bodies) and changes per trial.

## Scoring (`src/scoring.py`, primary score 0..1, value at the end of the trial)
- Seated: the flask's bottom on the clamp's base plate (within 1.5 mm), upright (within 2 deg), centred (within
  3 mm), and all five fingers in contact with the flask at 1.4-14 deg outward (pads resting on the conical body, not
  spread over the heel).
- Released: every robot fingertip at least 40 mm from the flask.
- Shake test: once the flask is seated, released and still for 0.5 s, the platform runs on its 19 mm orbit at 200 rpm
  for 2.5 s (0.5 s ramp up, 1.5 s, 0.5 s ramp down). A hand coming back within 40 mm stops the test.
- 1.0 (ends the trial): the shake test finished with the flask seated at every step of it.
- 0.5: seated but not released (a hand on or near it), or seated while the shake test runs.
- 0 otherwise: not seated (standing on the flared tips, tilted, beside the clamp, held in the air, on the bench) or
  thrown out of the clamp by the shake.
Real trial: look at the flask from the side (bottom on the plate, no gap; fingers closed on the glass), hands off,
then switch the shaker on for a few seconds and watch that the flask stays.

## Dream-1 evaluation

Run from `isaac/` in the configured YAM environment. The seed-0 trial recorded 1,800 executed actions, each with 14 values.

```bash
inspect-robots run --embodiment yam_arena --policy dream \
    -P model=dreamlabs/dream-1-beta -P rows_per_request=10 \
    -E task=task.py -E object_jitter=0 -E device=cpu -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 \
    --instruction 'Load the empty flask into the free clamp on the shaker platform so that the shaker can be started.' --scorer task_score --seed 0 --max-steps 1800 --store-frames --log-dir ./dream-output
```

## Run and score a trial
Inside the Isaac Sim 6.1 + Isaac Lab-Arena container, working directory `isaac/` (`isaac/run_frozen.sh` does this on a
frozen copy of `isaac/`, `src/` and `task.json`, writes `source-sha256.txt` and logs to `<out>/<run>/run.log`; see its
header for the environment variables it needs):

```
# official Inspect Robots run of the scripted reference demo (registered closed-loop policy flask_clamp_scripted)
python official_run.py -- run --embodiment yam_arena --policy flask_clamp_scripted -E task=task.py -E object_jitter=0 \
    -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --instruction "$(python -c 'import json;print(json.load(open("../task.json"))["instruction"])')" \
    --scorer task_score --max-steps 1800 --store-frames --log-dir <out>/official
inspect-robots video <out>/official/<log>.json --out <out>/official/video     # H.264 video of every camera

# plain do-nothing run (task.py demo() gives the yam_scripted policy no waypoints: both arms hold READY)
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py ... --max-steps 1800

# direct demo / negative stages (trace.jsonl, result.json, demo_<camera>.mp4, setup.png, final.png)
python demo.py task.py <out>/<run> --stage demo --seed 1415158438   # the default inspect-robots start
python demo.py task.py <out>/<run> --stage demo                      # named setup 0; FLASK_SETUP=k for setup k
python demo.py task.py <out>/<run> --stage neg-perched --seed 1415158438  # set on the flared tips and let go: 0
python demo.py task.py <out>/<run> --stage neg-held --seed 1415158438     # pushed home, never let go: 0.5
FLASK_REPLAY=<run>/trace.jsonl python demo.py task.py <out>/<run2> --stage replay --seed 7   # open-loop replay
FLASK_REPLAY_MIRROR=1 FLASK_REPLAY=<run>/trace.jsonl python demo.py task.py <out>/<run3> --stage replay --seed 5525  # its mirror
```
Unit checks (no simulator): `python tests/test_scoring.py`.
Tiled evidence video (top + left wrist + right wrist, same timing):
`isaac/tile.sh <out_dir> <name> demo_top_cam.mp4 demo_left_cam.mp4 demo_right_cam.mp4`.

## Submission media

- `model_run.mp4` shows the complete Dream-1 trial at seed 0, score 0.0 in 1,800 steps.
- `scripted_run.mp4` shows the official reference at seed 0, score 1.0 in 547 steps. Both videos show overhead, left wrist and right wrist views at 20 Hz.
- `setup_overhead_hires.png` shows the proposed setup from the overhead camera.

## Start states
Every trial reset with a seed (every `inspect-robots run` passes one) draws which near clamp is free (left or right,
50/50; the far clamps and the other near clamp hold culture flasks), moves the shaker by up to 8 mm in x and y, turns
the free clamp's finger ring by 0-72 deg, and stands the flask on the bench on the free clamp's side at x 30-46 cm,
|y| 24-30 cm. Every other seeded start keeps its flask at least 30 mm from the default start's flask position (same free clamp)
or from its left-right mirror image (other free clamp), so neither the default trajectory nor its mirror can be
replayed onto a near-identical start. Unseeded resets (direct `demo.py` runs) use the named setups 0-3 in turn (`src/layout.py`
SETUPS; `FLASK_SETUP` picks the first and, when set, also forces named setups on seeded resets).

## Fidelity (full statement)
- Engine: Isaac Sim 6.1 / Isaac Lab-Arena `yam_arena`, PhysX on the CPU, physics 240 Hz, control 20 Hz, cameras
  320 x 176. All parts are full size.
- Flask: 250 mL narrow-neck borosilicate Erlenmeyer flask with the ISO 1773 dimensions (85 mm base, 145 mm tall,
  34 mm neck), 140 g, empty, aluminium foil cap (visual). Collision shape: three convex pieces (rounded heel and cone,
  neck, lip); the glass is rendered translucent.
- Clamp: a stainless spring-finger 250 mL platform clamp of the common type: a 100 mm base plate (2 mm) and five
  12 mm stainless strips that lean in over the flask's cone and flare out at the top. Finger count, shape and
  stiffness are representative (no datasheet values). Each finger is a rigid strip (collision thickness 2 mm) on a
  revolute joint at its foot (hard limits -3..40 deg); joint drives on such (maximal) joints do not act in this PhysX
  build, so its leaf spring is a computed torque, -0.55 N m/rad x angle - 0.004 N m s/rad x rate, applied every
  physics step from the measured finger pose (estimate). Resting on the flask's cone, a finger presses with about
  1-2 N; pushing the flask in took a downward push of 24-28 N in the reference runs (the largest 50 ms mean of the net
  vertical robot force on the flask, friction included).
- Friction (static = dynamic): fingers on glass 0.2 (the fingers' value is used for that pair), glass on the gripper
  pads 0.7 and on the plate 0.4-0.5 (PhysX averaging). The neck's contacts carry a 9 mm torsional friction patch, which
  stands in for the area contact of the rubber gripper pads on the glass (a point contact would let the flask spin
  freely about the grip axis).
- Shaker: a generic low-profile benchtop orbital shaker (no model claimed): 340 x 340 mm housing, 305 x 305 mm
  platform 100 mm above the bench, 19 mm orbit. The shake test moves the platform, the free clamp's base and the loaded
  clamps as kinematic bodies on the orbit; the fingers and the flask follow through their joints and contacts.
- The three loaded clamps and their culture flasks are fixed (kinematic) bodies with their fingers at the seated angle;
  the culture medium is visual only.
- Contact offset 1 mm, rest offset 0. The flask and fingers never sleep.
- Nothing welds, snaps or teleports the flask; the scripted demo only sends the 14 joint / gripper commands and reads
  poses. The scorer only reads, except that it runs the platform orbit for the shake test. On an environment reset
  (between trials only) the kit puts every body back at the next start state.
- Grip: the reference demo commands its fingers 6 mm inside the neck against the 5000 N/m finger drive, about 30 N per
  finger (measured pad peaks 31.4-32.2 N in every evidence run, no spike above that). When the flask gives way past the pads, the demo eases its arm off and loosens
  its fingers 4 mm off the neck, so the flask is not driven into the plate by the wound-up arm.
- The free clamp is always one of the two near clamps: the arms cannot reach a flask in the far row at clamp height
  (this is a reach limit of the station, stated here rather than hidden).

## Real rig
- Shaker: any benchtop orbital shaker with a 12 x 12 in universal platform and a 19 mm orbit, platform about 10 cm
  above the bench.
- Four stainless spring-finger 250 mL flask clamps bolted to the platform, 125 mm apart; three hold capped 250 mL
  flasks with about 50 mL of water (stand-in for culture medium).
- One empty 250 mL narrow-neck Erlenmeyer flask with a foil cap, standing on the bench beside the shaker.

## Third-party assets and licenses
| Asset | Source | License |
|---|---|---|
| YAM arm URDF and meshes | [I2RT Robotics model source](https://github.com/i2rt-robotics/i2rt/tree/120c3c81400171174604e503943f8d1ebc891058) | MIT. The installed runtime records the model commit in `yam_arena/assets/yam/SOURCE_COMMIT`. |
| Station, cameras and YAM embodiment | [YAM Arena runtime used on Brev](https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730#license) | Installed dependency. Its README reports that the integration code has no declared license. |
| Isaac Sim 6.1 and Isaac Lab-Arena | NVIDIA and [Isaac Lab-Arena](https://github.com/isaac-sim/IsaacLab-Arena/blob/c7edf3a9d5d8276565208f72422f8f8a0806d6cf/LICENSE.md) | NVIDIA Isaac Sim license and Apache-2.0 for the Arena package, respectively. |
| Shaker, clamps, flasks geometry | generated in code by this task (`src/layout.py`, `isaac/flaskkit.py`) | MIT (this repository, `LICENSE`) |
| `src/fastik.py` (YAM URDF kinematics) | this task's authors | MIT |
