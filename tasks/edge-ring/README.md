# Fit a new 300 mm etch-chamber edge ring, clocked by eye to its locating pins, without touching the chuck

The task is the Isaac Lab-Arena (PhysX) `yam_arena` task in
`isaac/`; `src/` holds the engine-independent modules it imports (layout and ring geometry, scorer, fast IK) and
`tests/` their unit checks.

## The task
The lid and upper chamber of a 300 mm plasma-etch chamber are off. On the cathode, the old silicon edge ring has
already been taken out; the three edge-ring lift pins stand raised in the tool's exchange position. The new ring lies
on a stand beside the chamber, turned to a start angle that changes every trial. The robot picks it up with both
hands, turns it so that its three radial slots line up with the chamber's three locating dowels (the white dowels can
be seen through the slots from above; only one angle fits, and the ring must come down with every dowel within
about 0.35 mm of its slot's centre; further off it can jam on a dowel's lead-in or land on the tips), sets it on the raised lift pins, takes its hands out, and presses RING DOWN on the maintenance pendant; the
pins then lower the ring onto the dowels. Lowering the pins first and setting the ring straight onto the dowels by hand
also counts. It must never touch the chuck face, drop the ring, press it onto a pin or leave it on the dowel tips.

Why it matters: the edge ring is a consumable of every capacitively coupled etch chamber (it shapes the plasma at the
wafer edge and erodes), replaced at chamber maintenance. On cathodes with edge-ring lift pins the pins raise the ring
to an exchange position for ring swaps and lower it onto its locating dowels; here the robot does the hand-off at that
position with the lid open.

## Scoring (`src/scoring.py`, primary score 0..1, value at the end of the trial)
- 0 for the whole trial if at any time anything touched the chuck (any contact force on the puck), the ring was
  dropped (moving down faster than 0.2 m/s, the speed of a 2 mm free fall, with no robot contact), or the robot pressed
  the ring onto the chamber parts with more than the ring's weight + 10 N for 50 ms, or the ring rested on the dowel
  tips (the base ring with its dowels carries half the ring's weight or more for 0.5 s while the ring's bottom is more
  than 0.8 mm above the seat; only the dowels stand above the seat). Vetoes latch: turning a ring into place after it
  sat on the tips does not recover the score.
- 0.5: seated (below) but a hand still in the bowl or within 30 mm of the ring.
- 1.0 (ends the trial): lift pins lowered, ring seated on the base ring within 0.3 mm and 0.15 deg (all three dowels
  through their slots), both hands out of the bowl and 30 mm or more from the ring, ring still for 1 s.
- Otherwise 0: ring on the stand, on the lift pins (however well placed: whether it fits is only shown when the pins
  lower it, and a person checks the same outcome), or lowered onto the dowel tips.
Real trial: video for the vetoes; seated = a 0.1 mm feeler does not enter under the ring at any slot (both 0.5 and 1.0),
plus the hands' position in the video.

## Dream-1 evaluation

The Inspect Robots run used model `dreamlabs/dream-1-beta` and returned a task score of 0.0 at the 3,000-step limit. The scripted reference run scored 1.0.
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
# official Inspect Robots run of the scripted reference demo (registered closed-loop policy edge_ring_scripted)
python official_run.py -- run --embodiment yam_arena --policy edge_ring_scripted -E task=task.py -E object_jitter=0 \
    -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --instruction "$(python -c 'import json;print(json.load(open("../task.json"))["instruction"])')" \
    --scorer task_score --max-steps 3000 --store-frames --log-dir <out>/official
inspect-robots video <out>/official/<log>.json --out <out>/official/video     # H.264 video of every camera

# plain do-nothing run (task.py demo() gives the yam_scripted policy no waypoints: both arms hold READY)
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py ... --max-steps 3000

# direct demo / negative stages (trace.jsonl, result.json, demo_<camera>.mp4, setup.png, final.png)
python demo.py task.py <out>/<run> --stage demo            # named setup 0 (-35 deg); EDGE_SETUP=k for setup k
python demo.py task.py <out>/<run> --stage demo --seed 7   # a seeded start state, as in inspect-robots runs
python demo.py task.py <out>/<run> --stage neg-unclocked   # never turned: set on the pins at its stand angle, lowered onto the dowel tips: 0
python demo.py task.py <out>/<run> --stage neg-offset      # clocked but 1.5 mm off: comes down on the dowel tips: 0
python demo.py task.py <out>/<run> --stage neg-drop        # let go 15 mm above the pins: 0
python demo.py task.py <out>/<run> --stage neg-press       # set on the raised pins, then pushed down onto them: 0
python demo.py task.py <out>/<run> --stage neg-not-lowered # placed on the raised pins, RING DOWN never pressed: 0
python demo.py task.py <out>/<run> --stage neg-hand-in     # seated, left hand left in the bowl: 0.5
python demo.py task.py <out>/<run> --stage neg-touch-chuck # fingertips touch the chuck face: 0
python demo.py task.py <out>/<run> --stage idle,demo       # reset check (several trials in one process)
python negcheck.py negcheck.json <run> [<run> ...]         # stimulus + reaction check
```
Unit checks (no simulator): `python tests/test_scoring.py`.
Tiled evidence video (top + left wrist + right wrist, same timing):
`isaac/tile.sh <out_dir> <name> demo_top_cam.mp4 demo_left_cam.mp4 demo_right_cam.mp4`.

## Recorded media

- `model_run.mp4` shows the full Dream-1 trial (seed 0, 3,000 steps, final task score 0.0). The three views run at 20 Hz in order from left to right: overhead, left wrist, right wrist.
- `official_tiled_top_left_right.mp4`: the official Inspect Robots run of the scripted reference policy at the default
  seed (ring 20.9 deg off on the stand), score 1.0 after 1,041 control steps (H.264, 960 x 176).
- `setup_overhead_hires.png`: the start state at 1280 x 704 (named setup 0, ring 35 deg off).

Start states: every trial reset with a seed (every `inspect-robots run` passes one) draws the ring's stand angle
uniformly from -35 to -10 or +10 to +22 deg (never within 10 deg of fitting, so the ring always has to be turned;
the bounds are the reach of the reference grips around the stand's arms) and moves the stand by up to 10 mm in x and y;
the default inspect-robots seed gives -20.9 deg. Unseeded resets (direct `demo.py` runs) use the named setups -35, -20,
-15, +15, +20 deg in turn (`EDGE_SETUP` picks the first; set, it also forces named
setups on seeded resets).

## Fidelity (full statement)
- Engine: Isaac Sim 6.1 / Isaac Lab-Arena `yam_arena`, PhysX on the CPU, physics 480 Hz, control 20 Hz, cameras
  320 x 176. All parts are full size.
- Geometry: representative 300 mm cathode ring-set dimensions (see "Real rig and replica"); OEM drawings are
  proprietary and no part number is claimed. The bowl wall is 40 mm above the seat (a real chamber is deeper; cut so
  the YAM arms reach the ring). The edge ring is flat in section (a real ring has a small step under the wafer edge,
  which the robot does not touch).
- Edge ring: one free rigid body (0.29 kg, silicon inertia) made of 131 convex prisms: 3 deg sectors with the three
  slots cut out. Chord sag of the sectors keeps 0.45 mm (not 0.5 mm) radial clearance at the bore. Contact offset
  0.05 mm, rest offset 0. The ring never sleeps (so its contacts are always reported).
- Chamber parts (chuck puck, quartz base ring with dowels, cover ring, bowl and cathode), the stand and the pendant are
  kinematic rigid bodies: they do not move, and PhysX reports the contact forces on them. The chuck puck collides as an
  exact triangle mesh (a cooked convex hull came out larger than the 298 mm disc and closed the 0.45 mm bore gap).
- Lift pins: one kinematic carrier with three pins, moved every physics step at 5 mm/s toward the position selected by
  the last key press (25 mm up or 1 mm below the seat). This is a position-controlled device, as on the tool; nothing
  else moves it.
- Keys: a key acts when the robot pushes on it with 5 N or more for 0.1 s (contact force); the key caps do not travel
  (a real key travels about 1 mm).
- Friction coefficients (static = dynamic): silicon on quartz / alumina / sapphire / Delrin 0.3; silicon on the
  gripper pads 0.5 (combined with the pads' own material by PhysX averaging). Estimates.
- Vetoes from contact forces read every physics step: any contact force on the chuck puck (0.05 N); a downward ring
  speed above 0.2 m/s with no robot contact (the speed after a 2 mm free fall); robot contact on the ring while the
  chamber parts push back with more than its weight + 10 N for 50 ms; the base ring with its dowels carrying half the
  ring's weight for 0.5 s while the ring's bottom is more than 0.8 mm above the seat (resting on the dowel tips).
  Contact rules use forces, not contact positions.
- Nothing welds, snaps or teleports the ring; the scripted demo only sends the 14 joint / gripper commands and reads
  poses. The scorer only reads. On an environment reset (between trials only) the kit puts the ring back on the stand
  and the lift pins up.
- The reference demo grips with about 12.5 N per finger (fingers commanded 2.5 mm inside the ring's faces against the
  5000 N/m finger drive), far under the YAM's 40 N.

## Real rig and replica
The ring set is drawn from representative dimensions of a 300 mm capacitively coupled etch cathode (OEM drawings are
proprietary; no part number is claimed): alumina chuck puck 298 mm; quartz base ring, bore 298.6 mm, outer 380 mm, top
2.5 mm below the chuck face, three alumina dowels 4.0 mm x 3.8 mm proud with a 0.8 mm x 45 deg tip chamfer on a 320 mm
circle at 0 / 115 / 245 deg; three sapphire lift pins 3 mm on a 340 mm circle at 60 / 180 / 300 deg; quartz cover ring
bore 362 mm, outer 420 mm; silicon edge ring outer 360 mm, bore 299 mm, 4.0 mm thick (0.29 kg) with three radial
through-slots 4.3 x 8 mm matching the dowels (0.15 mm play a side). A real etch chamber costs far more than a test rig, so real trials use a
replica built from these dimensions:
- chuck and base ring: a 298 mm aluminium disc (face painted or anodised light grey) on a 380 mm acrylic or aluminium
  plate 2.5 mm lower, three 4 mm hardened dowel pins pressed in (3.8 mm proud, tips chamfered 0.8 mm x 45 deg);
- lift pins: three 3 mm stainless pins through clearance holes, fixed to a plate under the base ring that a small
  motorised linear stage (26 mm travel, 5 mm/s, e.g. a lead-screw stage with a stepper and two limit switches) raises
  and lowers; two momentary push buttons (about 5 N) on a small box drive it to the upper / lower limit;
- bowl: a 460 mm aluminium or acrylic ring wall, 40 mm above the seat (cut down from a real chamber's depth so the
  arms reach);
- ring: a used silicon edge ring of this size if available, otherwise a 4 mm glass or ceramic ring of the same size
  and mass with the three slots cut;
- stand: a 40 mm post with three 15 mm arms carrying the ring 97.5 mm above the table, marks every 5 deg.

## Third-party assets and licenses
| Asset | Source | License |
|---|---|---|
| YAM arm model (URDF, meshes) | i2rt, via the Isaac Lab-Arena YAM package | MIT |
| Station scene, cameras, `yam_arena` embodiment | Isaac Lab-Arena YAM package | as distributed with the package (no license file at the pinned revision) |
| Isaac Sim 6.1 / Isaac Lab-Arena | NVIDIA | NVIDIA Isaac Sim license / Apache-2.0 |
| Chamber, rings, stand, pendant geometry | generated in code by this task (`src/layout.py`) | MIT (this repository, `LICENSE`) |
| `src/fastik.py` (YAM URDF kinematics) | this task's authors | MIT |
