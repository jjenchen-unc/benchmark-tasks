# Press a thin-section bearing into a robot joint housing on an arbor press: centre it under the ram, press it home square

The task is the Isaac Lab-Arena (PhysX) `yam_arena` task in `isaac/`; `src/` holds the engine-independent modules it
imports (layout and part geometry, fit model, scorer, fast IK) and `tests/` their unit checks.

## The task
A quasi-direct-drive robot actuator needs its output bearing, an SKF 61806-2RS1 thin-section ball bearing (30 x 42 x
7 mm), pressed into the 42 mm bore of its aluminium output housing, down onto the bore's shoulder. The housing lies loose
on the table of a 1-ton bench arbor press, to the left of the ram (where it lies changes every trial). A foam tool tray
holds the bearing and two driver discs from a bushing-driver set: the 41 mm disc, whose face is relieved so it bears
on the bearing's outer ring only, and one plain disc, either 32 mm (it would bear on the inner ring) or 46 mm (it would
bear on both rings at once); which one, and which slot holds which, changes per trial. The robot sets the bearing square in the bore mouth, puts the right driver on it,
takes the housing by its sides and slides it across the press table until the bore is under the ram, holds it there
while the other hand lowers the ram with the handle onto the driver, lets go once the ram bears, and pulls the handle
until the ring bottoms on the shoulder: the ram stops and the handle stalls. Then it lets go of everything.

Why it matters: every joint of a legged or humanoid robot built on quasi-direct-drive actuators has its output bearing
pressed into a light-alloy housing like this one; the fit is a transition fit, so the ring must start square, be pushed
on the ring that is being fitted, and be pressed home onto the shoulder.

## Scoring (`src/scoring.py`, primary score 0..1, value at the end of the trial)
- 0 for the whole trial if at any time:
  - jammed: while the ram pushes on the work (ram contact force 20 N or more, 50 ms) the bearing is cocked more than
    0.3 deg in the bore, or the bore axis is more than 2.0 mm from the ram axis, or the driver is more than 1.5 mm off
    the bearing's axis (in a transition fit a cocked or off-centre ring jams; here the fit model then locks it);
  - brinelled: a plain disc (32 mm: inner ring only; 46 mm: both rings) or the ram itself pushes on the bearing with
    20 N or more for 50 ms (the inner ring's share of the load goes through the balls; bearing makers' mounting
    instructions say the force goes on the ring being fitted only).
- 0.25: the bearing is in the bore, square (0.3 deg or less) and at least 2 mm deep (bottom 2 mm below the housing
  face), not seated.
- 0.5: seated (outer ring on the shoulder: bottom within 0.05 mm of it, tilt 0.15 deg or less) but a hand is still on the
  handle or within 30 mm of the handle bar or the work.
- 1.0 (ends the trial): seated, both hands released and 30 mm or more from the handle bar and the work, bearing still
  for 1 s.
- Otherwise 0.
### Instrumented physical trial

Use a compression cell behind the ram face (range at least 1 kN), sampled at 1 kHz. Zero with the ram clear and verify at 20 N and a second point in the operating range with a calibrated reference. Mark the two plain discs and the relieved outer-ring driver so the scorer can identify the load path. The real trial uses loading through a prohibited face as the proxy for potential damage. For a trial in which another component also carries ram load, measure the load at the forbidden contact independently or reject the trace as unscorable.

Reference the housing to its machined bore. Engrave concentric witness arcs on the exposed tool faces before assembly and calibrate two synchronized orthogonal camera views with a scale at each working plane. Register the housing and tool to those fiducials so the bore and tool axes remain measurable under the ram. For a ring hidden by a driver, use an underside telecentric view through the open support and the housing's 37 mm through-bore to locate the concentric inner bore. Measure the *outer* race plane with three noncontact displacement probes at 120 degrees on a 40 mm pitch circle, through witness ports in the shoulder. Reference all three to a flat master on the shoulder. Use the three measured heights to fit the plane and calculate tilt from its slope. Keep the optical paths clear throughout the housing's travel. Qualify the setup with known 0.15 and 0.30 degree shims and calibrated lateral offsets of 1.5 and 2.0 mm. Log geometry at 200 Hz or faster on the force clock. Repeat an obscured or unsynchronized measurement.

Apply the simulated thresholds above to the synchronized data. Count consecutive time, clearing the timer when its condition ends. At release check insertion depth from the three outer-race heights relative to the housing face and shoulder, then check hand clearance and stillness on video. Do not infer transient force or sub-degree tilt from uncalibrated video.

### Selecting and resetting the interference pair

Use one numbered housing and bearing per trial. At the same stable room temperature, measure bearing OD with a 1 µm-reading micrometer and the 6061 housing with a bore comparator set against the same reference. Measure at two depths beyond the chamfer and at three azimuths, then record the extrema. Select a pair with 3–7 µm diametral interference throughout (target 5 µm). Include measurement uncertainty when deciding whether the pair falls inside this band. The K7 designation alone admits clearance pairs. Confirm on a sacrificial pair from the same machining lot that the bearing requires pressing and that an aligned outer-ring driver seats it. Record the force–depth trace. This is a proposed fixture acceptance test; the simulator's 300 N terminal force is an estimate.

After extraction inspect the bore for galling or raised material and repeat the six bore measurements. Replace a scored or out-of-band housing. Use a fresh gauged bearing after a prohibited-load or jam trial. Retain the pairing and reset records with the trial. [SKF mounting guidance](https://cdn.skfmediahub.skf.com/api/public/094fe398236d3d0a/pdf_preview_medium/03000EN_pdf_preview_medium.pdf)


## Dream-1 evaluation

Dream-1 scored 0.0 on seed 0 at the 3,000-step limit. Scripted demo: 1.0 in the official Inspect Robots run (default seed, 1,398 steps); 1.0 twice at the default seed (same trace); 1.0 at seed 7. Doing nothing: 0. Open-loop replay at seed 7: 0. Wrong disc: 0. Off-centre: 0.

From the task's `isaac/` directory in the Isaac Sim 6.1 and Isaac Lab-Arena environment, run:

```bash
instruction=$(python -c 'import json; print(json.load(open("../task.json"))["instruction"])')
inspect-robots run --embodiment yam_arena --policy dream \
    -P model=dreamlabs/dream-1-beta -P rows_per_request=10 \
    -E task=task.py -E object_jitter=0 -E device=cpu -E control_hz=20 \
    -E cam_width=320 -E cam_height=176 --instruction "$instruction" \
    --scorer task_score --seed 0 --max-steps 3000 --store-frames --log-dir ./dream-output
```

## Run and score a trial
Inside the Isaac Sim 6.1 + Isaac Lab-Arena container, working directory `isaac/` (`isaac/run_frozen.sh` does this on a
frozen copy of `isaac/`, `src/` and `task.json`, writes `source-sha256.txt` and logs to `<out>/<run>/run.log`; see its
header for the environment variables it needs):

```
# official Inspect Robots run of the scripted reference demo (registered closed-loop policy bearing_press_scripted)
python official_run.py -- run --embodiment yam_arena --policy bearing_press_scripted -E task=task.py -E object_jitter=0 \
    -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --instruction "$(python -c 'import json;print(json.load(open("../task.json"))["instruction"])')" \
    --scorer task_score --max-steps 3000 --store-frames --log-dir <out>/official
inspect-robots video <out>/official/<log>.json --out <out>/official/video     # H.264 video of every camera

# plain do-nothing run (task.py demo() gives the yam_scripted policy no waypoints: both arms hold READY)
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py ... --max-steps 3000

# direct demo / negative stages (trace.jsonl, result.json, demo_<camera>.mp4, setup.png, final.png)
python demo.py task.py <out>/<run> --stage demo --seed 1415158438  # the default inspect-robots trial seed
python demo.py task.py <out>/<run> --stage demo            # named setup 0; BP_SETUP=k for setup k
python demo.py task.py <out>/<run> --stage neg-inner-disc  # the plain disc instead of the driver: brinelled, 0
python demo.py task.py <out>/<run> --stage neg-off-centre  # housing left 4 mm short of the ram axis: jammed, 0
python demo.py task.py <out>/<run> --stage neg-not-home    # stops pressing 2.5 mm before the shoulder: 0.25
BP_REPLAY=<run>/trace.jsonl python demo.py task.py <out>/<run> --stage replay --seed 7   # open-loop replay
python negcheck.py negcheck.json <run> [<run> ...]         # stimulus + reaction check
```
BP_STREAM_WAIT=0.05 (environment) caps the renderer's per-frame RTX streaming wait (one GPU of the development host
stalled 30 s a frame); it changes no physics, scoring or policy input other than finishing frames sooner.
Unit checks (no simulator): `python tests/test_scoring.py`.
Tiled evidence video (top + left wrist + right wrist, same timing):
`isaac/tile.sh <out_dir> <name> demo_top_cam.mp4 demo_left_cam.mp4 demo_right_cam.mp4`.

## Start states
Every reset with a seed (every `inspect-robots run` passes one) draws the housing's place on the press table (centre
110-130 mm to the left of the ram axis, within 4 mm fore-aft), which plain disc is in the tray (32 or 46 mm) and which
tray slot holds the driver. Unseeded resets (direct `demo.py` runs) use five named setups in turn (`BP_SETUP` picks
the first). The default seed 1415158438 gives: housing 110.7 mm left, 1.6 mm back; the 46 mm plain disc; driver in
slot c.

## Recorded media

`model_run.mp4` shows the full seed 0 Dream-1 trial (3,000 steps, final task score 0.0).

- `official_tiled_top_left_right.mp4`: the official Inspect Robots run of the scripted reference policy at the default
  seed (H.264, 960 x 176).
- `setup_overhead_hires.png`: the start state at 1280 x 704 (default seed).

## Fidelity (full statement)
- Engine: Isaac Sim 6.1 / Isaac Lab-Arena `yam_arena`, PhysX on the CPU, physics 480 Hz, control 20 Hz, cameras
  320 x 176. All parts are full size.
- Bearing: SKF 61806-2RS1, 30 x 42 x 7 mm (dimensions confirmed from a retailer listing; the mass 0.026 kg and the
  0.3 mm ring chamfer are catalogue values recalled, not re-checked). One free rigid body: outer ring (48 convex
  sectors with the OD chamfers), recessed seals and inner ring; it does not turn internally (spinning is not simulated).
- Housing: representative actuator output housing (no product drawing claimed): 6061 aluminium, 62 mm OD, 22 mm tall,
  bore 7 mm deep to a shoulder, 37 mm through-bore, 0.8 x 45 deg lead-in. In the simulation the bore is 42.24 mm (0.12
  mm radial clearance, 48 convex staves) because rigid bodies cannot overlap; the transition fit's interference is
  replaced by the fit model below.
- Fit model (disclosed simulation rule standing in for the interference): a thin annulus inside the bore under the
  outer ring, a link of the housing on a prismatic joint, held by an elasto-plastic friction anchor (a stiff joint
  drive whose force is capped at F(d) = 20 N + 280 N x d / 6.2 mm, d = travel into the bore, the anchor following when
  the push exceeds the cap). The bearing therefore moves into the bore only while it is pushed down harder than F(d),
  i.e. only under the press, and the press must deliver up to about 300 N through the handle. 300 N is an estimate for
  about 5 um interference (a K7 housing in a light alloy, mid tolerance; friction 0.15); real fits vary from a slip to
  several hundred newtons. When the ring reaches the shoulder the support lets go for the rest of the trial and the
  bearing rests on the real shoulder face (otherwise the support's 0.03 mm elastic travel would spring it back up).
- Jam (disclosed rule): when the bearing is cocked more than 0.3 deg, or the press load is off-centre (bore axis more
  than 2.0 mm from the ram axis, driver more than 1.5 mm off the bearing) while the ram pushes, the fit cap rises to
  5 kN: the ring stops, as a cocked ring jams in a real transition fit. The 0.3 deg / 2.0 mm / 1.5 mm limits are
  estimates.
- Press: representative 1-ton bench arbor press (cast base with table, column, head; 25.4 mm square steel ram; no model
  number is claimed): a fixed-base articulation, ram on a prismatic joint, handle on a revolute joint. Rack and pinion:
  a stiff ram position drive follows the handle angle (12.7 mm of ram per handle radian; about 19:1 at the grip 245 mm
  from the pivot) and the equal and opposite generalised force is applied to the handle joint every physics step, so
  the ram's load is felt at the handle. Pinion friction 1.2 N m (holds the handle and ram where left).
- Driver discs: steel, 12 mm thick; 41 mm disc with its face relieved inside 38 mm (bears on the outer ring only)
  and a 29.2 mm pilot spigot 4 mm long that enters the bearing's 30 mm bore (0.4 mm radial clearance) and keeps the
  driver centred (it never bears axially);
  32 mm and 46 mm plain discs (one of the two is in the tray per trial; the other is put away on a shelf under the
  table, out of reach and out of view). Representative of a metric bushing-driver set (not a specific catalogue part).
- Friction coefficients (static = dynamic): steel on steel 0.15, aluminium on cast iron 0.2, handle bar 0.5, foam 0.6.
  Estimates.
- Grip forces of the reference demo: bearing, driver and housing pinched with the fingers commanded 2.5 mm inside the
  part (about 12.5 N a finger, 5000 N/m finger drive); the handle bar with 1.5 mm (7.5 N). Contact force between the
  handle and the robot, sampled once per control step, is 15-17 N for both pads together while pressing (median
  14.8 N, 99th percentile 17.5 N); single physics steps show contact spikes up to 43 N on one link (solver noise in
  the stiff rack-and-pinion contact chain, not a sustained force).
- Nothing welds, snaps or teleports a part; the scripted demo only sends the 14 joint / gripper commands and reads
  poses. The scorer only reads. On an environment reset (between trials only) the kit puts every part back.

## Real rig
- Press: any 1-ton bench arbor press (cast iron, 1 in square ram) bolted to the bench, handle on the right.
- Housing: a 6061 aluminium ring, 62 mm OD x 22 mm, bored 42 mm K7 x 7 mm deep to a shoulder with a 37 mm through-bore,
  0.8 x 45 deg lead-in (a lathe part; the drawing is these numbers).
- Bearing: SKF 61806-2RS1 (new, from stock).
- Drivers: 41 mm, 32 mm and 46 mm discs from a metric bushing-driver set; the 41 mm disc faced back 1.5 mm inside
  38 mm (or use a 38/41 mm ring of tube).
- Tool tray: foam-lined case, 160 x 100 x 45 mm.

## Third-party assets and licenses
| Asset | Source | License |
|---|---|---|
| YAM arm model (URDF, meshes) | i2rt, via the Isaac Lab-Arena YAM package | MIT |
| Station scene, cameras, `yam_arena` embodiment | Isaac Lab-Arena YAM package | as distributed with the package (no license file at the pinned revision) |
| Isaac Sim 6.1 / Isaac Lab-Arena | NVIDIA | NVIDIA Isaac Sim license / Apache-2.0 |
| Press, housing, bearing, discs, tray geometry | generated in code by this task (`src/layout.py`) | MIT (this repository, `LICENSE`) |
| `src/fastik.py` (YAM URDF kinematics) | this task's authors | MIT |
