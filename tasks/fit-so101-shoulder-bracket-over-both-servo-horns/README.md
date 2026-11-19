# Fit an SO-101 arm's shoulder bracket over both horns of its fixed servo and line up its screw holes

Task for the bimanual YAM station in Isaac Lab-Arena (`yam_arena`, PhysX). Results through the official
`inspect-robots run` path on the evaluation machine: scripted demo 1.0 (690 steps), Dream-1 0 (2400 steps, hit the
step limit), doing nothing 0 (`task.json` "scores").

## The task
The SO-101 is the open-source robot arm (TheRobotStudio / Hugging Face LeRobot) built in large numbers to collect
robot-learning data. Every joint is assembled the same way: a 3D-printed link with two arms (cheeks) straddles the
STS3215 servo, one cheek on each of the servo's two horns, then 4 M3 screws go in per side. This task is the first
joint (official assembly text, Joint 1: "Attach the shoulder part. Tighten the shoulder part with 4 M3x6mm screws on
top and 4 M3x6mm screws on the bottom"), from the point where the servo is already seated and screwed in the base and
both horns are on, up to the point where the screws go in. Driving the screws is the next, separate step and is not
part of this task. The servo is fixed, so the printed part moves onto it: unlike seating a servo in a housing, the
bracket must straddle both horn faces at once, with one of them out of sight, and then be turned to register its
holes.

Start (same every trial): the partly built arm (printed base + base motor holder + STS3215 servo, both horns fitted)
is held to the table by one C-clamp (about 5 cm jaw) on the base's 18 mm thick flange at its back edge, the side away
from the robot, clear of the bracket's path (in the sim the base is fixed and the clamp is not modelled); the printed
shoulder bracket stands on its lower arm on the table to the robot's right; the
top horn's hole pattern is 25 deg off the bracket's square-on position.

The robot must hold the bracket so that both cheeks stay level with the two horn faces (36.4 mm apart, line-to-line;
the lower cheek and lower horn are out of sight under the servo, 2 mm above the base floor), slide it on along the
one path its slots allow (the top horn's M3 screw head runs in the top cheek's 5.8 mm slot, the idler boss in the
bottom one) until the top horn meets the cheek's lip, with the bracket turned (either before it goes on, or on the
horns once seated) so that its four top holes are over the top horn's four holes (within 4.0 deg, 0.49 mm), then let go. The
scripted reference reads the horn's hole angle, slides the bracket on at that angle and turns it after seating only
if it is more than 3 deg off.

## Scoring (final state; `SCORE_OVER = "end"`)
| Level | Condition (code: `src/scoring.py`) | Person check |
|---|---|---|
| 0.25 | both cheeks straddle both horns: screw head at least 3 mm inside the top slot, within 1 mm of the slot line, bracket height within 1 mm, tilt <= 3 deg | side photo; screw head seen in the slot |
| 0.5 | home: horn axis within 0.5 mm of the cheeks' hole-pattern axis, midway between the cheeks (top horn on the lip), height within 0.5 mm, tilt <= 2 deg | horn rim on the lip, centre screw centred in the cheek's middle hole |
| 0.75 | home and registered: bracket within 4.0 deg of one of the four 90-deg hole positions on the top horn (0.49 mm at the 7.07 mm hole radius) | a 2.35 mm pin (e.g. a drill bit shank) passes through each top hole into the horn's hole (see below) |
| 1.0 | registered, every finger pad tip >= 20 mm from the bracket, bracket still (< 2 mm/s, < 2 deg/s) for 1 s | video frames |
| 0 | otherwise (elsewhere, jammed on a rim, one horn only, upside down, knocked over) | |
The episode ends as soon as 1.0 is reached. The bottom horn's holes are not scored: that horn turns freely on its
boss and is turned to its holes with the screw tip when the bottom screws go in (as in the real build).

Hole sizes behind the 0.75 check (official CAD, measured on the bracket mesh): each cheek's four holes are 3.2 mm
through bores (3.3 mm long) under 5.2-5.4 mm head counterbores; the horn's holes are M3 threads (core 2.46 mm). An M3
screw (3.0 mm) has only 0.1 mm play in the 3.2 mm bore, so "a screw starts by hand" would pass only at about 0.1-0.2 mm
off centre, stricter than the scored 0.49 mm. A straight pin of diameter d passes both holes while the offset is at most
(3.2 - d)/2 + (2.46 - d)/2; for d = 2.35 mm that is 0.48 mm, which matches the scored 4.0 deg (0.49 mm). The setup's
drill-bit test keeps the printed bore at 3.2 mm.

## Run and score a trial
Host: Isaac Sim 6.1 + Isaac Lab-Arena + yam_arena + Inspect Robots in a container with one GPU.
`isaac/run_frozen.sh <run> "<cmd>"` copies `isaac/ src/ assets/ task.json` into `$SO101_OUT/<run>/source`
(read-only, per-file and combined SHA-256) and runs `<cmd>` there; env vars: `ISAAC_LAUNCHER` (container launcher),
`GPU_SLOT_WRAPPER` (optional GPU slot wrapper), `ARENA_VENV_BIN` (Arena venv `bin/` inside the container), `SO101_OUT`.
```sh
I="$(python3 -c 'import json;print(json.load(open("task.json"))["instruction"])')"
# official Inspect Robots path (default guardrails, task_score), scripted reference policy registered by official_run.py
bash isaac/run_frozen.sh <run> "python official_run.py --stage demo -- run --embodiment yam_arena \
  --policy so101_joint_scripted -E task=task.py -E object_jitter=0 -E device=cpu -E control_hz=20 -E cam_width=320 \
  -E cam_height=176 --instruction \"$I\" --scorer task_score --max-steps 2400 --store-frames --no-prompt \
  --log-dir <out>/<run>"
# any model policy: same arguments with --policy <name>; --policy yam_scripted holds READY (do-nothing baseline)
# scripted demo directly (14 joint/gripper commands only): result.json, trace.jsonl, demo_<camera>.mp4, setup.png
bash isaac/run_frozen.sh <run> "python demo.py task.py <out>/<run> --stage demo --max-steps 2400"
# negative / partial cases (one per --stage): neg-partial | neg-unregistered | neg-held | neg-jammed | neg-beside | idle
# several trials in one process with yam_arena's in-process reset (writes reset_check.json):
bash isaac/run_frozen.sh <run> "python demo.py task.py <out>/<run> --stage neg-partial,neg-unregistered,neg-held"
```
Development-only environment switches (unset in every demo, official and negative run): `SO101_HORN_JOINTS=articulation` (the rejected
horn version, for probe_horn / probe_mouth comparisons), `SO101_CONTACT_OFFSET`, `SO101_HULL_MIN_THICKNESS`,
`SO101_PHYSICS_DT`, `SO101_HORN_DAMPING`, `SO101_DEBUG_DROP` / `SO101_DROP_PIECES` (leave colliders out),
`SO101_CONTACT_REPORT`, `SO101_DEBUG_NOVIS`.
Engine-independent checks: `cd src && python check_scoring.py` (synthetic scorer checks, numpy only).
Geometry asset: `src/make_assets.py` rebuilds `assets/so101_joint1.npz` from the official CAD (see Assets).

## Files
| File | Role |
|---|---|
| `isaac/task.py` | yam_arena task file (SCENE, OBJECTS, score/success/distance) |
| `isaac/jointkit.py` | bench spawn (static clamped base, jointed horns, free bracket) + scorer hook |
| `isaac/demo.py` | scripted demo and negative variants (robot commands only; reads poses) |
| `isaac/official_run.py` | official `inspect-robots run` path with the scripted policy registered |
| `isaac/run_frozen.sh` | evidence runner on a read-only source snapshot |
| `isaac/probe.py` | bench probe (development) |
| `isaac/probe_horn.py` | horn friction check: torque steps on the horns, holds / turns / stops |
| `isaac/probe_fit.py` | fit check: bracket released at set slide positions, where it comes to rest (no arm) |
| `isaac/probe_mouth.py` | free-bracket push check: constant force along the slots, how far it gets, its speed |
| `isaac/negcheck.py` | stimulus + expected-level check of finished demo / negative runs (JSON only) |
| `src/layout.py`, `src/scoring.py`, `src/check_scoring.py` | layout, scorer, scorer checks |
| `src/make_assets.py` | official CAD -> visual meshes, static collider, convex slab pieces of the bracket |
| `assets/so101_joint1.npz`, `assets/LICENSE-SO101-Apache-2.0.txt` | derived geometry + its licence |

## Fidelity (full statement)
`task.json` "fidelity" is a short version of this statement.

Isaac Sim 6.1 / Isaac Lab-Arena yam_arena (PhysX on the CPU, rendering on the GPU), physics 960 Hz, control 20 Hz, YAM arms from their URDF with implicit joint drives at their real effort limits and 50 N fingers. Geometry is the official SO-101 CAD (TheRobotStudio STL files, Apache-2.0): visual meshes decimated; the base, base motor holder and servo case are one static triangle-mesh collider (fixed in the sim; clamped to the table in a real trial, clamp not modelled); the bracket's collider is its official mesh cut into 14 horizontal slabs at its feature levels (cheeks, slots, lip, web, upright), each cross-section split into convex prisms, so the slots (5.8 mm top, 6.2 mm bottom), the 1 mm lip and the cheek ends are the CAD's. The bracket's screw holes (including their edge chamfers, which break into the lower slot's walls by about 0.1 mm near the cheek face and would otherwise give the collider sharp-edged notches) and the motor-2 pocket are filled in the collider. Horns: 20.0 mm discs on revolute joints about the servo axis, each with a stiff zero-velocity drive capped at its friction torque (dry friction: below the cap it creeps 0.006 deg/s at 0.54 N m; PhysX's articulation-joint friction was tried and rejected: in this engine build its contacts with the bracket chattered and locked the bracket in its slots); the top horn carries its M3 screw head (5.0 mm; modelled 1.3 mm proud instead of 1.5 so that, with the face clearance below taken up by the bracket's weight, it keeps 0.2 mm under the roof of the 1.5 mm groove it runs in) and resists turning with a friction torque of 0.6 N m (an estimate of the unpowered 1:345 gearbox's backdrive torque; measure it with a spring scale on the horn; checked in the engine: creeps 0.006 deg/s under 0.54 N m, turns under 0.9 N m, stops when the torque is removed), the bottom horn turns freely (0.002 N m); the servo case's 6.0 mm idler boss under it (static) is modelled 5.7 mm across (0.25 mm a side in its 6.2 mm slot) and 0.2 mm longer than in the CAD, so that with the face clearance below (the bracket hangs 0.2 mm lower on the top horn than in the CAD) it still engages the bottom cheek's slot walls over the CAD's 0.6 mm. Fit: the CAD is line-to-line (cheek inner faces 36.4 mm apart = across the horns; lip radius 9.89 mm on a 10.00 mm horn). PhysX needs a finite gap and the YAM wrist is soft, so the sim has 0.2 mm per side at the horn faces (a tilt of more than about 1 degree still pinches both horns) and 0.1 mm radial at the lip; PhysX contact offset 0.1 mm per shape, below those slot clearances (with 0.15 mm or more the engine kept contacts on both walls of a slot at once and the bracket locked in its slots; with 0.1 mm the pins can be pressed up to about 0.2 mm into their slot walls under load, solver compliance); real printed parts come out line-to-line to a light press and the PLA cheeks flex slightly, which a real arm feels as a light push (hence the setup's fit test: 5 N or less to slide home, else a new print). Each horn's outer rim has a 0.4 mm chamfer (estimate of the rounded rim of the kit horns), the only lead-in; the printed edges are sharp in the collider. Masses: bracket 35 g (estimate, weigh the print), horns 3 g; friction PLA on PLA/aluminium 0.35. No welds, snaps, proximity rules or teleports: the bracket goes on only by contact with the horns, the screw head and the boss in its slots. The scorer reads poses only (bracket, top horn, finger pad tips). The servo's cable and the controller board are not modelled (cable tucked in the base). Known engine artifact: solver contact noise can turn the top horn while the bracket is held pressed home (up to about 12 deg/s in the recorded runs; a real horn would not move), and a free bracket can turn a few degrees as it settles on the horns; registration is always scored against the horn's actual angle.

Checks and notes: `isaac/probe_horn.py` checks that the top horn holds, turns and stops as authored; with the horns as
articulation links instead, their contacts with the bracket chattered and locked it in its slots (`isaac/probe_mouth.py`).
In the recorded runs the horn and the bracket stayed put once the hand had let go. Registration is scored against the
horn's actual angle (where its holes really are), so a policy has to check the holes after letting go. The scripted
reference grips the bracket with about 13 N per finger and pushes with a few newtons.

## Measurements to take before submission
These values are estimates from the CAD and data sheets. Measure each on one real print and one STS3215 with simple
tools, then put the number in the named fields. If a code constant changes, the evidence runs must be redone (they are
tied to the source SHA-256).
| Value (now) | How to measure | Feeds |
|---|---|---|
| Top-horn backdrive torque (0.6 N m, estimate) | servo unpowered; tie a thread to a screw in an outer horn hole (7.07 mm radius) or to a 10 cm lever screwed to the horn; pull tangentially with a spring scale and read the force at which the horn starts to turn; torque = force x radius; mean of 3 pulls | task.json "fidelity", README Fidelity; `HORN_TOP_FRICTION_NM` in `src/layout.py` |
| Bracket mass (35 g, estimate) | kitchen scale (0.1 g) on the printed bracket | task.json "setup" and "fidelity", README Fidelity; `BRACKET_MASS` in `src/layout.py` |
| Slide-on push force (sim: 0.2 mm per side at the horn faces, 0.1 mm at the lip) | push-pull spring scale on the web, horn on its marker line, bracket square; force to slide from the slot mouths to home; mean of 3 | the 5 N limit of the fit test in task.json "setup"; sim clearances `HORN_TOP_Z`, `HORN_BOTTOM_Z` in `src/layout.py` |
| Bracket hole bore (3.2 mm) and horn thread core (2.46 mm) | drill bits or pin gauges in 0.05 mm steps: largest that passes each hole | the pin size of the 0.75 check in task.json "scoring" and README Scoring; scorer threshold `REGISTER_DEG` in `src/scoring.py` stays 4.0 |
| Horn mass (3 g) and rim chamfer (0.4 mm), PLA friction (0.35) | kitchen scale; calipers or a photo with a ruler; tilt-plane test (angle at which the bracket slides on a horn: friction = tan(angle)) | README Fidelity; `HORN_MASS`, `HORN_CHAMFER`, `BRACKET_FRICTION` in `src/layout.py` |
| Clamp position | clamp a real print as in the setup; check the bracket slides home and both arms reach it without touching the clamp | task.json "setup", README Start |

## Third-party assets and licenses
| Item | Source | License |
|---|---|---|
| SO-101 CAD: STL meshes (`base_so101_v2`, `base_motor_holder_so101_v1`, `sts3215_03a_v1`, `rotation_pitch_so101_v1`) and the MJCF transforms | TheRobotStudio SO-ARM100 / SO-101, as packaged in the Hugging Face repo `lerobot/so101-tabletop-mujoco` (`robotstudio_so101/`) | Apache-2.0 (`assets/LICENSE-SO101-Apache-2.0.txt`) |
| `assets/so101_joint1.npz` (decimated visual meshes, static collider, convex slab pieces) | derived from the SO-101 CAD by `src/make_assets.py` (SHA-256 of the sources in its `meta`) | Apache-2.0 (derived work) |
| Task code (`isaac/`, `src/`) | written for this task | MIT (`LICENSE`) |
| Task geometry generated in code (horn discs, top-horn screw head, idler boss) | written for this task (`isaac/jointkit.py`) | MIT (`LICENSE`) |
| YAM robot model (URDF, meshes) | i2rt, vendored by yam-isaac-lab-arena | MIT (i2rt) |
| Station scene (`yam_arena` station, IK, cameras, embodiment) | yam-isaac-lab-arena benchmark SDK, installed separately | see upstream (no licence file in the repository; used, not redistributed) |
| Isaac Lab-Arena, Isaac Lab, Inspect Robots | upstream, installed separately | see upstream |
| Isaac Sim 6.1 | NVIDIA, installed separately | see upstream |

## Runtime
Scripted demo: 690 control steps (34.5 s simulated, run limit 2400); about 6-8 min wall time per trial on one GPU
with PhysX on the CPU and three 320x176 cameras (official run 6.5 min; the built-in `yam_scripted` do-nothing policy
stops itself after 61 steps, 1.3 min; held at READY for the full 2400 steps through the official CLI, `--policy
yam_scripted -P hold_s=130 --max-steps 2400`, it scores 0 and takes 17.8 min, so a model trial that runs to the limit
takes about 18-22 min). Seeds: none (layout fixed, `object_jitter=0`); with PhysX on the CPU repeated
runs of the same code are identical (demo x2: same 690 steps, same final pose). The full score is not robust to
tiny changes: in a multi-trial process, the same script run after an in-process reset (bodies back within 0.1 um)
ended home but unregistered (0.5; the top horn turned 29 deg while the bracket was pressed home, see the artifact
above).

## Evidence media
Start state at 1280x704 (same frozen source, idle stage, 3 steps): `setup_top_cam_1280x704.png`,
`setup_left_cam_1280x704.png`, `setup_right_cam_1280x704.png` and the tiled `tiled_setup_hires.png` (top | left |
right).
Each demo / official run writes one H.264 video per camera (`demo_top_cam.mp4`, `demo_left_cam.mp4`,
`demo_right_cam.mp4`; the right arm's wrist camera is `right_cam`), `setup.png` and `final_<camera>.png`. Tiled views
(top | left | right, same timing): `tiled/tiled_top_left_right.mp4`, `tiled/tiled_setup.png`, `tiled/tiled_final.png`.
The final wrist-camera frame shows the bracket's top arm and its hole pattern from above at an angle: the trial ends
1 s after the hand lets go, while the hand is still swinging to its look-down pose.
