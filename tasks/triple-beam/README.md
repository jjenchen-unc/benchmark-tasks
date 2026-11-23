# Weigh a sample jar on a triple-beam balance: riders seated in notches, poise slid until the free beam rests on zero

The task is the Isaac Lab-Arena (PhysX) `yam_arena` task in `isaac/`; `src/` holds the engine-independent modules it
imports (balance layout, masses and start states, scorer, fast IK) and `tests/` their unit checks.

## The task
A triple-beam balance modelled on the Ohaus 750-S0 (Triple Beam 700 series, 610 g capacity, 0.1 g readability,
magnetic damping, stainless pan) stands on the bench in front of the robot, its 6 in pan on the left and its pointer
and zero mark on the right. In the simulation its three beams carry a 500 g rider (notches every 100 g, rear beam), a
100 g rider (notches every 10 g, middle beam) and a 10 g poise that slides along a graduated front beam (0.1 g lines);
the real 750-S0 has the 500 g and 100 g beams the other way round (see "Known sim/real differences"). A sealed, opaque 250 mL sample jar stands on the pan; all riders
are at zero, so the pan side is down on its stop and the pointer is up against its top stop. The instruction is only
"Weigh the sample jar on the balance."

What makes it hard: the riders have to be moved while watching a beam that the hand itself disturbs.
1. Each notched rider must end *in* a notch: a rider let go on the crest between two notches gives no reading.
2. While a finger is on a rider, the beam cannot swing; the robot has to let go and lift its hand, then watch which
   way the free, damped beam moves (pointer up: the pan side is heavier; down: the riders are), and move again.
3. The 10 g poise is slid until the beam comes to rest with the pointer on the zero mark (0.1 g = 0.5 mm of pointer
   travel, 2 mm of poise travel); the jar's mass is different every trial and is not shown anywhere.
Cues the robot can see: the pointer against its scale (red zero line, lines every 2 mm), the riders' positions on the
beam graduations, the jar on the pan.

## Scoring (`src/scoring.py`, primary score 0..1, value at the end of the trial)
- Reading = the notch values of the 500 g and 100 g riders plus the poise's scale reading. There is no reading while a
  notched rider is not seated in a notch (more than 0.3 mm from a notch bottom).
- At rest on zero: the pointer within 0.9 mm of the zero mark (about 0.18 g) and moving slower than 0.6 mm/s, for
  1.5 s. Hands off: no robot contact with any part of the balance or the jar for 2 s.
- 1.0 (ends the trial): the jar on the pan, at rest on zero, hands off, both notched riders seated, and the reading
  within 0.2 g of the jar's true mass.
- 0.5: the jar on the pan, at rest on zero, hands off, but a notched rider is not seated in its notch (or the reading
  is more than 0.2 g off).
- 0 otherwise: the beam on a stop or still swinging, the pointer off the zero mark, a hand on the balance, the jar off
  the pan.
Person check (real rig): see "Real-trial scoring checklist" below (eyes or a phone video only).

## Dream-1 evaluation

On Brev, Dream-1 scored 0.0 in 2,400 of 2,400 control steps at evaluation seed 0. The Inspect Robots log contains 2,400 actions of 14 values each. Run the following from `isaac/` inside the configured YAM environment.

```bash
inspect-robots run --embodiment yam_arena --policy dream \
    -P model=dreamlabs/dream-1-beta -P rows_per_request=10 \
    -E task=task.py -E object_jitter=0 -E device=cpu \
    -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --instruction 'Weigh the sample jar on the balance.' --scorer task_score --seed 0 --max-steps 2400 --store-frames --log-dir ./dream-output
```

## Run and score a trial
Inside the Isaac Sim 6.1 + Isaac Lab-Arena container, working directory `isaac/` (`isaac/run_frozen.sh` does this on a
frozen copy of `isaac/`, `src/` and `task.json`, writes `source-sha256.txt` and logs to `<out>/<run>/run.log`; see its
header for the environment variables it needs):

```
# official Inspect Robots run of the scripted reference demo (registered closed-loop policy triple_beam_scripted)
python official_run.py -- run --embodiment yam_arena --policy triple_beam_scripted -E task=task.py -E object_jitter=0 \
    -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --instruction "Weigh the sample jar on the balance." --scorer task_score --max-steps 2400 --store-frames \
    --log-dir <out>/official
inspect-robots video <out>/official/<log>.json --out <out>/official/video     # H.264 video of every camera

# plain do-nothing run (task.py demo() gives the yam_scripted policy no waypoints: both arms hold READY)
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py ... --max-steps 2400

# direct demo / negative stages (trace.jsonl, result.json, demo_<camera>.mp4, setup.png, final.png);
# --seed defaults to the official eval-seed-0 trial seed (1415158438); "stage@seed" sets one trial's seed
python demo.py task.py <out>/<run> --stage demo
python demo.py task.py <out>/<run> --stage neg-coarse-only   # notched riders set, poise never moved: 0
python demo.py task.py <out>/<run> --stage neg-unseated      # 100 g rider left 7 mm short of its notch, balanced: 0.5
python demo.py task.py <out>/<run> --stage neg-hand-on       # balanced, but the hand stays on the poise: 0
python demo.py task.py <out>/<run> --stage replay@7          # TB_REPLAY=<trace.jsonl>: open-loop replay (TB_MIRROR=1: mirrored)
```
Unit checks (no simulator): `python -m pytest tests`.
Tiled evidence video (top + left wrist + right wrist, same timing):
`isaac/tile.sh <out_dir> <name> top.mp4 left.mp4 right.mp4`.

## Submission media

- `model_run.mp4` shows the full Dream-1 trial. The views are overhead, left wrist and right wrist at 20 Hz.
- `scripted_run.mp4` shows the independently rerun official scripted policy, which scored 1.0 in 1,281 steps.
- `setup_overhead_hires.png` — Overhead view of the reference setup.

Start states: every trial reset with a seed (every `inspect-robots run` passes one) draws the jar's mass between 120 g
and 480 g, always 1-9 g past a 10 g step (so the poise is always needed and never sits at an end of its beam) and
never 4.4-5.6 g past it (a "poise at mid-scale" guess never fits), moves
the balance up to 8 mm in x and y and turns it up to 3 deg, and stands the jar anywhere within 22 mm of the pan
centre. Any start whose mass would lie within 3 g of the default start's mass gets a new mass, so the default
start's rider settings leave every other start at least 3 g out of balance. Unseeded resets use three named setups.
The scene is built in the first trial's start state; a later trial in the same process gets its own jar mass and jar
position, but the balance stays where the first trial put it (its base is fixed to the bench). One trial per process
is the intended use (the official CLI runs one).

## Fidelity (full statement)
- Engine: Isaac Sim 6.1 / Isaac Lab-Arena `yam_arena`, PhysX on the CPU, physics 480 Hz, control 20 Hz, cameras
  320 x 176.
- Balance: Ohaus 700/800-series triple-beam layout at estimated real size (no drawing could be read while building the
  task): base 51 x 15 cm, fulcrum 90 mm above the bench, pan-bearing arm 45 mm, beams graduated over 200 mm (zero line
  50 mm from the fulcrum), pointer 290 mm from the fulcrum with stops at +-7 mm, beams 30 mm apart, 6 in (152 mm)
  stainless pan. Beam order front to back (10 g, 100 g, 500 g) is this simulation's layout; the real 750-S0 has
  the 500 g beam in the middle and the 100 g beam at the rear (see "Known sim/real differences (v2 fix)").
- Mechanism (all emergent in PhysX): the beam assembly (300 g) turns on the fulcrum (revolute joint); its centre of
  mass sits 8.5 mm below the fulcrum (restoring torque 0.025 N m/rad: 0.1 g moves the pointer 0.51 mm) and is placed so
  that the empty pan balances the riders at zero (the zero adjustment). The magnetic damper is a velocity-proportional
  torque (0.024 N m s/rad, about 0.8 of critical: the free beam settles in about 3 s). The pan (200 g) hangs on its
  bearing (revolute) and the check link of the real Roberval linkage is a PhysX mimic constraint (pan angle = -beam
  angle), so the pan stays level and the load counts at the bearing wherever the jar stands. Each rider is a body on
  its beam (prismatic joint) whose weight acts where it sits; masses from the lever law (rider mass x 200 mm = beam
  capacity x 45 mm): 112.5 g, 22.5 g, 2.25 g; their centres of mass are on the knife-edge line (the real riders hang
  round the beam). Checked in the engine: a balanced setting settles the pointer within 0.001 mm of the lever-law
  prediction; 0.5 g off settles at -2.52 mm (predicted -2.52 mm).
- Notches: not collided. A notched rider within 1.5 mm of a notch is pulled to its bottom by a stiff joint drive capped
  at the push that walks it out of a 45 deg V with friction 0.25 (1.66 x its weight: 1.8 N for the 500 g rider,
  0.37 N for the 100 g rider); elsewhere it slides with friction 0.25 x its weight. The poise slides with 0.15 N of
  spring-clip friction. These are forces between rider and beam only (they cannot turn the beam). The real riders
  are lifted slightly over the notch crests; here they are pushed over them.
- Jar: a 250 mL HDPE wide-mouth jar (70 x 98 mm, 40 g empty) holding dry sand or soil at an estimated 1.75 g/mL; its mass,
  centre of mass and inertia are set from the fill at every reset (the jar is opaque; no granular material is
  simulated).
- The base is fixed to the bench. No zero-adjust knob action is modelled (the balance starts zeroed).
- Forces in the reference demo (official seed; PhysX, max over physics steps): finger squeeze about 10 N per finger on
  the rider handles (robot-rider contact about 20 N summed over both pads); a notched rider needs 1.8 N (500 g) or
  0.37 N (100 g) along its beam to leave a notch.
- Robot-on-part contact (for "hands off") is read from PhysX contact forces between the robot's links and the beam,
  pan, riders, base and jar (collider identity, not contact positions).
- Nothing welds, snaps or teleports anything during a trial; the scripted demo only sends the 14 joint / gripper
  commands and reads state. On an environment reset (between trials only) the kit places the parts for the next start.

## Real rig (hardware, sample preparation)
- Balance: Ohaus 750-S0 triple-beam balance (Ohaus catalogue number 750-S0, Triple Beam 700 series: 610 g capacity,
  2610 g with the optional attachment weights, which are not used; 0.1 g readability; stainless 6 in pan; magnetic
  damping). Price about USD 150-250 from school and lab suppliers (estimate, not a quote). The similar Ohaus 760-00 /
  710-00 also fit. Level it on a firm bench and zero it with the empty pan and all riders at zero (zero-adjust knob).
- Sample: a white, opaque 250 mL HDPE wide-mouth jar with screw cap (about 70 mm across, about 40 g empty). For each
  start, put the empty jar on a 0.1 g kitchen or lab scale, fill it with dry play sand to the start's target mass (for
  the default start 373.8 g), cap it and seal the cap with a strip of tape, and write the mass under the jar's base
  where the robot cannot see it. About 2 minutes per jar; prepare one jar per start mass and reuse them. Check each
  jar on the 0.1 g scale before a session (sand does not change mass; the tape keeps the cap from being opened).
- Start: riders at zero (notched riders seated in their zero notches), the jar on the pan within 2 cm of its centre,
  the balance on its start marks (marks every 2 mm on the bench). Nothing can break or spill in a failed trial (the
  jar is sealed; riders and beam are captive on the balance).
- Camera for scoring: a phone on a small stand at the pointer end, at pointer height, looking at the zero mark (this
  is what the media-only close-up shows in the simulation).

## Real-trial scoring checklist (eyes or a phone video only)
1. Hands off: in the video, the robot's fingers are away from the balance and the jar for at least the last 2 s.
2. Jar: still standing on the pan.
3. Notched riders seated: each of the two notched riders sits down in a notch (not on the crest between two notches);
   a rider between notches means no valid reading (0.5 at most).
4. Pointer at rest on zero: in the close-up video, the pointer has stopped moving and rests on the zero line (on a
   real 750-S0, 0.1 g off balance moves the pointer visibly; the reading in step 5 is the 0.2 g check).
5. Reading: read the rider positions (notch values plus the poise scale) and compare with the mass written under the
   jar; within 0.2 g.
Score: 1.0 if 1-5 all hold; 0.5 if 1, 2 and 4 hold but 3 or 5 fails; 0 otherwise (pointer on a stop or still
swinging, a hand on the balance, jar off the pan).

## Known sim/real differences (v2 fix)
- Beam order differs on the two notched beams. OHAUS specifies a 10 g front beam graduated in 0.1 g, a 500 g middle beam with 100 g steps, and a 100 g rear beam with 10 g steps. The simulation places the 100 g beam in the middle and the 500 g beam at the rear. Use the graduations on each beam when repeating the trial on hardware. [OHAUS 700-series specifications](https://www.ohaus.ca/pdf/triple-beam-DS.pdf)
- Dimensions and masses are estimates (no Ohaus drawing used): beam lengths, the 45 mm pan-bearing arm and the rider
  masses derived from them (112.5 g, 22.5 g, 2.25 g) have not been measured on a real 750-S0; the real riders are
  lighter or heavier in proportion to the real arm lengths. A v2 fix should measure a real balance (rider masses on a
  kitchen scale, beam and arm lengths with a ruler) and use those numbers.
- Notches are a detent force model, not collided V-notches (see Fidelity); the real riders are lifted slightly over
  the crests, here they are pushed over them.
- Base: a plain slab; the cast base's shape is not modelled. The base is fixed to the bench in the simulation.
- Sample: dry sand in the real rig; the simulation uses a rigid jar with the same mass (estimated soil/sand density
  1.75 g/mL for its centre of mass).

## Third-party assets and licenses
| Asset | Source | License |
|---|---|---|
| YAM arm URDF and meshes | [I2RT Robotics model source](https://github.com/i2rt-robotics/i2rt/tree/120c3c81400171174604e503943f8d1ebc891058) | MIT. The installed runtime records the model commit in `yam_arena/assets/yam/SOURCE_COMMIT`. |
| Station, cameras and YAM embodiment | [YAM Arena runtime used on Brev](https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730#license) | Installed dependency. Its README reports that the integration code has no declared license. |
| Isaac Sim 6.1 and Isaac Lab-Arena | NVIDIA and [Isaac Lab-Arena](https://github.com/isaac-sim/IsaacLab-Arena/blob/c7edf3a9d5d8276565208f72422f8f8a0806d6cf/LICENSE.md) | NVIDIA Isaac Sim license and Apache-2.0 for the Arena package, respectively. |
| Balance, riders, pan, jar geometry | generated in code by this task (`src/layout.py`, `isaac/balkit.py`); simplified convex pieces at estimated real sizes, no manufacturer CAD | MIT (this repository, `LICENSE`) |
| `src/fastik.py` (YAM URDF kinematics) | this task's authors | MIT |
