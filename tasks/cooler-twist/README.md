# Take an AMD Wraith Prism off an AM4 CPU: twist the stuck cooler free before lifting, or the CPU comes out with it

The task is the Isaac Lab-Arena (PhysX) `yam_arena` task in `isaac/`; `src/` holds the engine-independent modules it
imports (bench layout and mechanism constants, scorer, fast IK) and `tests/` their unit checks.

## The task
An open-air PC test bench (steel tray on rubber feet) carries a micro-ATX AM4 motherboard. An AM4 processor sits
in the AM4 socket (ZIF lever closed) and an AMD Wraith Prism cooler sits on it, held down by its retention clip, whose
cam lever is closed. The cooler has been on for a long time and its thermal paste has dried. A grey anti-static mat
lies on the bench beside the test bench. The instruction is "Take the CPU cooler off the processor and put it on the
mat."

Dried paste glues the cooler's base to the processor's lid. In tension it holds harder than the closed ZIF socket holds
the processor's pins, so pulling the cooler straight up brings the processor out with it ("CPU yank"): the pins leave
the closed socket and can bend. Every PC builder's advice is to twist the cooler gently side to side first: the paste
film shears under a modest twist and then the cooler lifts away cleanly. The way:
1. open the clip's cam lever (swing it up over the cam's high point until it lies open; the clip goes slack);
2. Pull each slack steel clip end outward and upward until its loop clears its own bracket lug. Both ends must clear.
3. Hold the cooler by its fan shroud's rim with both hands (108 mm, too wide to grip across) and twist it a few
   degrees one way, then the other, until it is seen to turn on the processor (the paste lets go; how hard it is stuck varies from trial
   to trial);
4. Lift it straight up, and put it on the mat; hands off.
Cues the robot can see: the lever closed or open, the cooler turning (or not) relative to the socket and board, the
processor rising with the cooler (the green substrate edge lifts out of the white socket).

A builder documented a Ryzen 3700X workstation with a Wraith Prism and an RTX 3080, then used it to train a CycleGAN. A cooler replacement or processor swap on this machine requires releasing this type of AM4 clip. The report establishes the hardware and workload. Service frequency remains unknown. [Firsthand workstation build and training run](https://nuadev.com/building-my-ai-gaming-rig-sharingan/)

An MLCommons earthquake-benchmark study reports running its workload on a Ryzen 5950X desktop with an RTX 3090. It documents an AM4 research workflow. The sacrificial Athlon fixture reproduces the PGA socket and clip service operation. We have no evidence of how often the study's machine needed this repair or what cooler it used. [MLCommons earthquake benchmark study](https://www.frontiersin.org/journals/high-performance-computing/articles/10.3389/fhpcp.2023.1233877/full)

## Scoring (`src/scoring.py`, primary score 0..1, value at the end of the trial)
- 0 for the whole trial (veto latches) if the processor comes 1 mm or more out of its socket at any time (it came off
  with the cooler: its pins left the closed socket).
- 1.0 (ends the trial): the cooler is off the processor and rests upright on the mat, the processor never left its
  socket, and the assembly remains still on the mat with all parts released for three continuous seconds.
- 0.5: the cooler is off the processor and the processor never left its socket, and the full placement hold has not finished, including while still held.
- 0 otherwise (cooler still on the processor).
Person check (no instruments; full checklist under "Real-world setup"): the processor is still seated in the socket
(never visibly lifted, ZIF lever down), the cooler stands upright on the mat; the video shows the cooler turning on the
processor before it is lifted.

## Verified simulation

Official Inspect Robots, evaluation seed 0 (Brev). Scripted reference scored 1.0 in 886 steps. Dream-1 scored 0.0 in 1800 steps over 177 model calls. Opening the cam and lifting with both clip ends still engaged scored 0.0 in 451 steps. After both clips released, a straight lift without a twist scored 0.0 in 827 steps and triggered the CPU-lift veto. The recorded trials include every joint action and all three camera views at 20 Hz.

## Dream-1 evaluation

Run the following from `isaac/` inside the configured YAM environment.

```bash
inspect-robots run --embodiment yam_arena --policy dream \
    -P model=dreamlabs/dream-1-beta -P rows_per_request=10 \
    -E task=task.py -E object_jitter=0 -E device=cpu \
    -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --instruction 'Take the CPU cooler off the processor and put it on the mat.' --scorer task_score --seed 0 --max-steps 1800 --store-frames --log-dir ./dream-output
```

## Run and score a trial
Inside the Isaac Sim 6.1 + Isaac Lab-Arena container, working directory `isaac/` (`isaac/run_frozen.sh` does this on a
frozen copy of `isaac/`, `src/` and `task.json`, writes `source-sha256.txt` and logs to `<out>/<run>/run.log`; see its
header for the environment variables it needs):

```
# official Inspect Robots run of the scripted reference demo (registered closed-loop policy cooler_twist_scripted)
python official_run.py -- run --embodiment yam_arena --policy cooler_twist_scripted -E task=task.py -E object_jitter=0 \
    -E device=cpu -E control_hz=20 -E cam_width=320 -E cam_height=176 \
    --instruction "Take the CPU cooler off the processor and put it on the mat." --scorer task_score \
    --max-steps 1800 --store-frames --log-dir <out>/official
inspect-robots video <out>/official/<log>.json --out <out>/official/video     # H.264 video of every camera

# plain do-nothing run (task.py demo() gives the yam_scripted policy no waypoints: both arms hold READY)
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py ... --max-steps 1800

# direct demo / negative stages (trace.jsonl, result.json, demo_<camera>.mp4, setup.png, final.png)
python demo.py task.py <out>/<run> --stage demo --seed 1415158438
python demo.py task.py <out>/<run> --stage neg-straight-pull   # clip opened, cooler pulled straight up: processor out, 0
python demo.py task.py <out>/<run> --stage neg-short-twist     # token twist (1 deg), then lifted: processor out, 0
python demo.py task.py <out>/<run> --stage neg-no-hooks       # cam opened, clip ends still engaged, then lifted
python demo.py task.py <out>/<run> --stage neg-no-lever        # lever left closed, twist and pull tried: nothing moves, 0
python demo.py task.py <out>/<run> --stage replay@7            # CT_REPLAY=<trace.jsonl>: open-loop replay (stage@seed);
                                                               # CT_REPLAY_MIRROR=1 plays it mirrored
```
Unit checks (no simulator): `python -m pytest tests`.
Tiled evidence video (top + left wrist + right wrist, same timing):
`isaac/tile.sh <out_dir> <name> top.mp4 left.mp4 right.mp4`.

Start states: every trial reset with a seed (every `inspect-robots run` passes one) moves the test bench 15-35 mm from
its nominal spot in a random direction, turns it -90 +-12 deg (the clip lever faces the left arm) and draws how hard the
paste sticks (break torque 0.60-1.40 N m). When a start would put the clip lever's finger paddle where the default
start's has it (less than 32 mm to the side of it and less than 22 mm further in), the bench is placed again (position
and turn), so a trajectory recorded at the default start never finds the lever (and with the clip locked nothing can
come off). Unseeded resets (direct
`demo.py` runs) use three named setups in turn.

## Fidelity (full statement)
- Engine: Isaac Sim 6.1 / Isaac Lab-Arena `yam_arena`, PhysX on the CPU, physics 480 Hz, control 20 Hz, cameras
  320 x 176. The thin motherboard uses the CPU convex collider. PhysX reports that its aspect ratio prevents cooking a GPU-compatible convex mesh.
- Parts are convex pieces. The board uses the published ASRock 230 × 201 mm outline on 6.4 mm standoffs over a steel tray on rubber feet
  (the whole test bench is one free 4.5 kg body on the table); AM4 socket body 45 x 50 mm, PGA processor 40 x 40 mm
  with a 38 x 38 mm lid, 46 g; Wraith Prism main cooler body 0.55 kg: 38 x 38 mm copper base, fin stack 86 mm across
  (one convex collider; the fins are drawn as thin rings, not as the real radial fins), black fan shroud 108 mm
  across and 29 mm tall that overhangs the fins by 11 mm. The stationary fan and unlit light guide are visual geometry. The two AM4 retention brackets and their lugs are colliders; the
  strap is visual. Each end has a separate sliding loop, toe and colliding finger tab.
- Clip: the cam lever is a jointed part on the cooler (40 mm to its flat finger paddle). While it is below 55 deg the
  clip holds the cooler and processor to the bench (joint drives, up to 400 N / 20 N m) and the cam pushes the lever
  back closed with up to 0.6 N m (rising over the first 20 deg; about 15 N at the paddle; estimate from a ~300 N
  clip preload and ~2 mm cam throw). Past 55 deg the clip is slack and its spring nudges the lever onto its open stop
  (100 deg); pushed back below 30 deg it locks again. With the cam slack, each end can travel 15 mm on a horizontal guide. At 9 mm displacement its toe clears its own lug, then the slack end rises on an 11 mm guide. Both clip ends must be released before the paste bond can shear.
- Processor in the socket: held by joint drives (laterally and in rotation by its pins; upward by the closed ZIF
  contacts up to 25 N, an estimate). Once it has come 1 mm out it is free (pins out; their bending is not simulated,
  the scorer counts it).
- Dried paste (joint drives between cooler and processor, elasto-plastic anchors): 110 N in tension and 4 N m of peel
  (more than the socket's 25 N, so a straight pull takes the processor), and a twist torque up to a break torque drawn
  per trial from 0.60-1.40 N m (lateral shear up to the matching stress); slip beyond that is plastic, and after 6 deg
  of accumulated slip the film is sheared (then 2 N of tack and a light drag until the faces are 1.5 mm apart). Only
  twist or sideways slip shears the film here: rocking (tilting) the cooler on the processor does not. Values
  are estimates (shear stress 30-67 kPa over the 38 x 38 mm face; suction plus adhesion in tension); the real spread
  between pastes and ages is wide.
- Robot-on-part contact (for "hands off") is read from PhysX contact forces between the robot's links and the cooler,
  lever, both clip ends, processor and test bench. The hold resets when a robot touches one of those bodies or the cooler moves out of the resting pose.
- Nothing welds or teleports anything during a trial; the scripted demo only sends the 14 joint / gripper commands and
  reads state (part poses, lever angle, how far the cooler has turned or risen on the processor, whether the processor
  is rising; never the break torque or the paste state). On an environment reset (between trials only) the kit places
  the parts for the next start.

## Real-world setup (reproducible)
AMD’s procedure for a clip-mounted AM4 cooler includes disconnecting the fan and RGB leads, unlocking the cam lever, releasing both clips, then gently twisting the cooler and lifting it. In the proposed physical setup, disconnect both leads before each trial. The simulation includes separate contact-driven releases at both lugs. The simplified clip travel, finger access and paste resistance have not been measured. Prices below are approximate. [AMD cooler removal procedure](https://www.amd.com/en/resources/support-articles/faqs/CPU-7.html)

| Item | Product / part number | Approx. price | Notes |
|---|---|---|---|
| Cooler | AMD Wraith Prism RGB boxed SKU 199-999888, AM4 cam-lever clip | USD 45 listed by AMD, availability varies | the clip hooks over the two AM4 bracket lugs; the lever closes the clip |
| Motherboard | ASRock A320M-HDV R4.0 (micro-ATX, AM4, stock AM4 retention brackets) | USD 30-60 | any AM4 board with the stock plastic retention brackets and lugs works |
| Processor (sacrificial) | AMD Athlon 200GE, YD200GC6FBBOX (PGA, AM4) | USD 15-25 used; keep 2-3 spares | a yank bends its pins (pins are on the processor, not the board), so it is a consumable |
| Thermal paste | ARCTIC MX-4, 4 g syringe (ACTCP00002B) | about USD 6 | non-curing paste |
| Test bench | open-air bench, e.g. Streacom BC1 (V2), or a 30 x 30 cm plywood plate with the board on 6 mm standoffs and rubber feet | USD 15-100 | must not be screwed to the table (the sim bench is free, 4.5 kg) |
| Mat | any ESD mat, cut to 20 x 13.5 cm, grey | about USD 10 | the set-down target |

The board, processor and fan stay unpowered during robot trials. The optional thermal preparation requires a separately powered, cooled system before removal.

**Paste preparation.** Use the same recorded amount of MX-4 at the centre of the processor's lid, set the cooler on square and close the clip. The sticking that causes real
CPU yanks builds over many heat cycles; MX-4 does not harden on a fixed schedule, so no cure time is claimed. Two
preparations, neither validated on hardware by us:
(a) "aged": power the board with a PSU and run the processor at full load for at least 2 h, then switch off and let it
cool to room temperature for at least 1 h before the trial. Disconnect power, the fan and RGB leads before removal. Repeat the same cycle for every trial.
(b) "fresh": no heat cycle (a fresh, thick film still resists a fast straight pull far more than a twist, which is the
physics the task tests, but it holds much less).
How hard a given preparation sticks varies trial to trial (the sim draws the twist break torque from 0.60-1.40 N m to
cover this). It cannot be checked beforehand without breaking the bond; check it afterwards from the video: the
cooler should NOT turn under the first light touch, and should turn after a firm two-hand twist. Record the preparation and every trial, including assemblies that turn on first touch.

**Reset procedure**
1. After a success (cooler on the mat): wipe old paste off the lid and the cooler base with a lint-free cloth and
   isopropyl alcohol, re-apply, re-mount, close the clip, re-run the chosen preparation.
2. After a processor comes out with the cooler, separate it from the cooler and inspect the processor pins and socket. Set damaged parts aside. Fit an intact spare processor with the ZIF lever open, align the corner marks, close the lever and repeat the preparation.
3. Place the test bench on its start marks (tape marks every 5 mm on the table; the start offset and turn come from
   the trial seed's start state), clear the mat, arms to READY.

**Real-trial scoring checklist (eyes, video, a ruler; no force instrument).**
- Veto (0 for the trial): at any time the processor visibly lifts out of the socket (the green substrate edge rises
  above the white socket in the video), or after the trial the processor is not in the socket / is stuck to the
  cooler's base. The model uses 1 mm of rise as its numerical threshold.
- 1.0: the cooler stands upright on its base on the mat (its centre over the mat, not on its side), the processor is
  still in the socket with the ZIF lever down, and the video shows no robot contact with cooler, processor or bench
  for the full last three seconds while the cooler remains upright and still on the mat.
- 0.5: the cooler is off the processor and the processor never came out, and the three-second hold is incomplete. This covers a cooler still in the grippers, a set-down elsewhere, or a cooler still moving on the mat.
- 0 otherwise (cooler still on the processor at the end).
No real-trial check uses a force or torque; the sim's forces (clip, socket hold, paste) only decide what happens.

## Known simulation and hardware differences
1. AMD lists a 119 × 105 × 93 mm envelope and 545 g for the boxed Wraith Prism. The simulated cooler body is 93 mm tall and has a mass of 550 g. Its separate lever, clip ends and guide carriers add 34 g. Its 108 mm shroud span, 11 mm overhang and lever contact surfaces are geometry estimates. The fan and its light guide remain inactive during removal. Measure those local features on the physical assembly before judging gripper access. [AMD Wraith Prism specifications](https://shop-us-en.amd.com/amd-wraith-prism-thermal-solution/)
2. The fins are one solid convex cylinder (drawn as rings); the real heatsink has radial fins and a different base.
3. The sliding guides approximate flexible clip ends. The AMD procedure establishes the order of release. The clip trajectory, detent and force values are model assumptions.
4. The ASRock A320M-HDV R4.0 board measures 230 × 201 mm. The model matches that outline. Socket, DIMM, PCIe, power connector, battery, chipset and rear I/O positions are estimated by scaling the board plan on printed page 7 of the manual. Their heights and the exact socket position have yet to be measured. The model represents the major obstacles as simple colliders. [ASRock specifications](https://www.asrock.com/mb/AMD/A320M-HDV%20R4.0/) · [ASRock manual, motherboard layout](https://download.asrock.com/Manual/A320M-HDV%20R4.0.pdf#page=12)
5. Processor hold (25 N pull-out from the closed socket), paste tension (110 N) and twist break torque (0.60-1.40 N m)
   have yet to be measured on the named parts.
6. Pins are not simulated; the scorer counts a 1 mm rise as a yank. The yank check also counts while the clip is
   still locked. A hard sideways drag can stretch the simulated joint toward 1 mm and trigger a veto.
7. After the paste shears, the slack hooks no longer stop the cooler sliding sideways; rocking the cooler does not
   shear the paste in the sim (twist or sideways slip does).
8. The sim test bench is one free 4.5 kg body; a real bench's mass and feet differ.

## Third-party assets and licenses
| Asset | Source | License |
|---|---|---|
| YAM arm URDF and meshes | [I2RT Robotics model source](https://github.com/i2rt-robotics/i2rt/tree/120c3c81400171174604e503943f8d1ebc891058) | MIT. The installed runtime records the model commit in `yam_arena/assets/yam/SOURCE_COMMIT`. |
| Station, cameras and YAM embodiment | [YAM Arena runtime used on Brev](https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730#license) | Installed dependency. Its README reports that the integration code has no declared license. |
| Isaac Sim 6.1 and Isaac Lab-Arena | NVIDIA and [Isaac Lab-Arena](https://github.com/isaac-sim/IsaacLab-Arena/blob/c7edf3a9d5d8276565208f72422f8f8a0806d6cf/LICENSE.md) | NVIDIA Isaac Sim license and Apache-2.0 for the Arena package, respectively. |
| Test bench, motherboard, socket, processor, cooler, clip, mat geometry | generated in code by this task (`src/layout.py`, `isaac/coolerkit.py`); simplified convex pieces at estimated real sizes, no manufacturer CAD; product names are used only to describe the real objects | MIT (this repository, `LICENSE`) |
| `src/fastik.py` (YAM URDF kinematics) | this task's authors | MIT |
