# Feed NM cable through an outlet box's push-in clamp: past its sprung one-way tab, until 150 mm is inside, then let go

The task is the Isaac Lab-Arena (PhysX) `yam_arena` task in `isaac/`; `src/` holds the engine-independent modules it
imports (layout and cable geometry, scorer, fast IK) and `tests/` their unit checks.

## The task
A 2x4 stud offcut lies on the bench on its narrow edge, and a blue single-gang non-metallic nail-on outlet box is nailed
to its side, open face up (a wall section laid down on the bench). The box's end wall facing the robot's right has the
push-in cable entry at the box back: a 20 x 10 mm opening (its lower edge flush with the back) covered from inside by a sprung plastic clamp tab that
leans into the box. A 0.45 m length of white 14/2 NM-B cable lies flat on the bench to the right of the box,
diagonally, its cut end toward the box. The robot picks up the cable near its end, turns the end square to the entry
(flat, as the opening needs), pushes it through the clamp tab in short strokes, then feeds it on while the other hand
takes the end inside the box and guides it up out of the box face, until about 150 mm (6 in) of cable is inside, and
lets go.

Why it matters: when a residential rough-in uses a box with integral push-in cable clamps, the electrician feeds each cable through its entry and pulls it to length (the trial measures intact sheathed cable from the entry wall to the cut end).

What makes it hard:
- Aim: the flat cable (about 9.7 x 4.8 mm) must arrive flat and square at a 20 x 10 mm opening (about +-5 mm sideways,
  +-2.6 mm in height), and the box's place changes per trial.
- Force through a pinch that cannot hold an angle: the tab needs a push of roughly 10-15 N to lift (spring estimate;
  about 7-12 N contact force measured in the runs). A top-down pinch on the flat cable's narrow edges cannot hold its
  pitch, so the cable has to be kept riding on the opening's lower edge; if it rides up into the tab it bends and kinks
  (copper takes a set). It is fed in short strokes, regripping, with the grip close behind the end.
- One-way and exact: once the tab bears on the sheath it bites, so the cable cannot be pulled back out. The length
  inside (130-170 mm accepted) must be reached without overshooting: too much cannot be undone.
- Two hands: one hand feeds from outside in short strokes, the other reaches into the box, takes the end and guides it
  up and out of the box face, watching the length.

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

Run from `isaac/` in the configured YAM environment. The seed-0 trial recorded 3,600 executed actions, each with 14 values.

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
    --scorer task_score --max-steps 3600 --store-frames --log-dir <out>/official
inspect-robots video <out>/official/<log>.json --out <out>/official/video     # H.264 video of every camera

# plain do-nothing run (task.py demo() gives the yam_scripted policy no waypoints: both arms hold READY)
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=task.py ... --max-steps 3600

# direct demo / negative stages (trace.jsonl, result.json, demo_<camera>.mp4, setup.png, final.png)
python demo.py task.py <out>/<run> --stage demo             # named setup 0; NMB_SETUP=k for setup k
python demo.py task.py <out>/<run> --stage demo --seed 1415158438   # the default inspect-robots start
python demo.py task.py <out>/<run> --stage neg-blind-aim    # aims at the box's nominal place, not where it is: 0
python demo.py task.py <out>/<run> --stage neg-push-far     # one push from 150 mm behind the end to 50 mm, let go: 0.5 (short)
python demo.py task.py <out>/<run> --stage neg-overshoot    # NOT a negative: aims for 188 mm, but in the evidence
                                                            # the feed stalls at 156 mm, inside the window, and the
                                                            # run scores 1.0. No run shows a length above 170 mm.
NMB_REPLAY=<run>/trace.jsonl python demo.py task.py <out>/<run2> --stage replay --seed 732   # open-loop replay
```
Unit checks (no simulator): `python -m pytest tests/`.
Tiled evidence video (top + left wrist + right wrist, same timing):
`isaac/tile.sh <out_dir> <name> demo_top_cam.mp4 demo_left_cam.mp4 demo_right_cam.mp4`.

## Submission media

- `model_run.mp4` shows the complete Dream-1 trial at seed 0, score 0.0 in 3,600 steps.
- `scripted_run.mp4` shows the official reference at seed 0, score 1.0 in 1,376 steps. Both videos show overhead, left wrist and right wrist views at 20 Hz.
- `setup_overhead_hires.png` shows the proposed setup from the overhead camera.

## Start states
Every trial reset with a seed (every `inspect-robots run` passes one) moves the box (with its stud) by 8 to 20 mm in x
(never within 8 mm of its nominal place; every seed other than the default one also keeps the box at least 7 mm in x
from the default seed's box: a draw inside either band is redrawn, so box x is spread evenly over -20 to -8 mm and
15.2 to 20 mm, about 72 % / 28 % of seeds) and up to 30 mm in y, and lays the cable straight and flat with its cut end
75-120 mm in front of the entry wall, up to 20 mm to the side of the opening, at 10-30 deg to the entry axis. Unseeded
resets (direct `demo.py` runs) use the named setups 0-4 in turn (`src/layout.py` SETUPS; `NMB_SETUP` picks the first
and, when set, also forces named setups on seeded resets).

## Fidelity (full statement)
- Engine: Isaac Sim 6.1 / Isaac Lab-Arena `yam_arena`, PhysX on the CPU, physics 240 Hz, control 20 Hz, cameras
  320 x 176. All parts are full size.
- Box: a single-gang non-metallic nail-on box of the 18 cu in class, modelled from representative dimensions (55 x 92 mm
  inside, 58 mm deep, 2.5 mm walls;  the depth is at the shallow end of the class so that the YAM fingers reach the box floor), nailed to the side of a 2x4
  offcut (38 x 89 x 400 mm) lying on its narrow edge, face 13 mm proud of the stud. Box and stud are one kinematic body
  (contact forces are reported; it moves only between trials).
- Push-in clamp: a 20 x 10 mm opening in the box's top end wall at its back (lower edge flush with the back), covered inside by a 12 x 18 mm plastic tab
  (2.5 mm thick, rounded tip) hinged on the opening's upper edge. The tab is a link of a small articulation held on the
  box by a fixed joint: a revolute joint with a hard rest stop at 55 deg from vertical (its tip 1.7 mm into the cable's
  path) and an implicit spring drive (0.15 N m/rad, preloaded 20 deg against the stop): about 10 N to start lifting it and
  15 N at full lift (5.2 mm lever). The cable lifts it by contact. 15 N is the proposal's estimate for a push-in clamp;
  no catalog or measured value was available. Shape and rate are estimates.
- One-way bite: once the cut end is 12 mm or more past the wall and the tab bears on the sheath (1.5 deg or more off its
  stop), a computed one-way stop (40 kN/m, up to 120 N, applied to the cable link in the wall) keeps the cable from
  being pulled back out past the furthest point it was pushed to. In a real clamp the tab's edge bites the sheath;
  PhysX friction alone did not reliably self-lock the thin tab, so this is computed.
- Cable: 0.45 m of 14/2 NM-B (flat sheath 9.7 x 4.8 mm, small edge round; the cut end's top and bottom edges dressed by
  1 mm) as a floating 30-link articulation (the 200 mm at the cut end in 10 mm links, the rest in 25 mm links) on
  spherical joints. Each joint's three rotations have implicit drives with stiffness EI / l (flatwise EI 0.12 N m^2,
  edgewise 2.0 N m^2, torsion GJ 0.05 N m^2; estimates from the copper conductors) whose targets follow the joint past a
  yield moment (flatwise 0.18 N m, edgewise 0.8 N m, torsion 0.06 N m): beyond it the bend stays, as annealed copper
  holds its shape, and a cable forced up into the tab bends and kinks. The 20 links at the cut end (the ones the
  grippers squeeze) are floored at 5 g, the others at 2.5 g, inertias at 4e-7 kg m^2, with a small joint armature, for
  numerical stability: the simulated cable weighs about 0.13 kg (real: about 0.04 kg for 0.45 m). Contact-patch
  (torsional) friction radius 15 mm on the sheath.
- Friction (static = dynamic, averaged per contact unless noted): sheath 1.0 (about 0.75 against the finger pads), box
  0.35, stud 0.5, tab 0.12 ('min' combining: smooth moulded tab on PVC).
- Stillness is judged from displacement over 1 s of the 200 mm at the cut end (the instantaneous link speeds carry about
  20 mm/s of solver jitter while the cable stands still).
- Nothing welds, snaps or teleports the cable; the scripted demo only sends the 14 joint / gripper commands and reads
  poses. The scorer only reads. Between trials the kit lays the cable straight and flat on the bench and moves the box
  and stud to the next start state.
- Grip: the reference demo commands its fingers 6 mm inside the sheath's edges against the 5000 N/m finger drive, about
  30 N per finger. Measured finger-cable contact force (control steps): at the default seed (the official run) peak
  27.9 N, median about 24 N, robot-box contact 1.4 N. At named setup 0 the third feed stroke presses the cable end
  against the box while the left hand guides it: grip above 35 N for 47 control steps (peak 40.8 N; single-physics-
  step peak 42.6 N) and a left fingertip on the box at up to 88 N. In the neg-overshoot run (which stalls inside the
  window and scores 1.0, see the run commands above) up to 74 N grip and 131 N box contact.
- The scene is a practice bench (the wall laid down, box face up) so that the YAM can reach into the box from above;
  in a wall the box faces sideways and the cable comes down from above.

The U.S. Bureau of Labor Statistics describes installation of wiring and electrical systems as an electrician duty. This fixture isolates cable entry and retention during rough-in. [Occupational Outlook Handbook, Electricians](https://www.bls.gov/ooh/construction-and-extraction/electricians.htm)

## Real rig
- Use an Allied Moulded P-181RK, an 18 cu in single-gang nail-on box with integral Speed Klamps. Its published outside dimensions are 2-11/16 in deep, 2-5/16 in wide and 3-3/4 in high. Nail it to a 400 mm stud offcut with its face 13 mm proud. Orient the entry toward the cable and leave the cable de-energized. The simulated cavity and entry are representative. Measure the clear opening and tab force on the purchased unit when comparing physical trials. [Manufacturer catalog, FlexBox single-gang table](https://alliedmoulded.com/wp-content/uploads/Residential-Catalog-2023_WEB.pdf)
- Cable: 0.45 m of 14/2 NM-B (with ground) cut square, straightened, laid flat on the bench.

## Third-party assets and licenses
| Asset | Source | License |
|---|---|---|
| YAM arm URDF and meshes | [I2RT Robotics model source](https://github.com/i2rt-robotics/i2rt/tree/120c3c81400171174604e503943f8d1ebc891058) | MIT. The installed runtime records the model commit in `yam_arena/assets/yam/SOURCE_COMMIT`. |
| Station, cameras and YAM embodiment | [YAM Arena runtime used on Brev](https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730#license) | Installed dependency. Its README reports that the integration code has no declared license. |
| Isaac Sim 6.1 and Isaac Lab-Arena | NVIDIA and [Isaac Lab-Arena](https://github.com/isaac-sim/IsaacLab-Arena/blob/c7edf3a9d5d8276565208f72422f8f8a0806d6cf/LICENSE.md) | NVIDIA Isaac Sim license and Apache-2.0 for the Arena package, respectively. |
| Box, stud, clamp tab and cable geometry | generated in code by this task (`src/layout.py`, `isaac/cablekit.py`) | MIT (this repository, `LICENSE`) |
| `src/fastik.py` (YAM URDF kinematics) | this task's authors | MIT |
