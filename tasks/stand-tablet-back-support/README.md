# Stand a tablet by unfolding its attached back support

Prepare a tablet for hands-free viewing by deploying the thin folding support
already attached to its back. The tablet begins screen-up and the support lies
folded against it. The tablet and passive panels form one connected assembly:
handling the panels changes the supported tablet's geometry and balance. The
central manipulation is exposing and unfolding those panels, then transferring
the tablet's weight from the grippers onto its bottom edge and rear support.
The final result is a released, stable tablet in portrait orientation.

## Instruction and results

> Raise the tablet, unfold its attached back stand, and leave it standing in portrait orientation facing you on the desk, with its screen about 60 degrees above the desk. Release both hands and leave it stable for 5 seconds.

The fixed limit is 3600 public control steps, 180 seconds at 20 Hz. Seed 0,
fixed initial layout, no object jitter. The reported score is the end score.

| Run | Score | Actions | Simulated time | Observed wall time |
| --- | --- | --- | --- | --- |
| Native scripted demonstration | 1.0 | 2405 | 120.25 s | 526.711 s |
| Dream-1 beta | 0.0 | 3600 | 180.00 s | 1042.396 s |

The demonstration starts from the actual flat reset without fixture initialization.
It uses public robot joint actions; measured object state guides its controller.
Dream uses the unchanged official Inspect adapter and standard top/left/right
camera images plus robot joint state, never task object poses or scoring values.
Its 892 requests stayed in one server session with zero retries or execution
errors, ending at the action limit. It did not leave the tablet standing.
This is one fixed-layout trial, not a success-rate or robustness estimate.
`VALIDATION.json` records source and media hashes and measured results.

## Reproduce

Use the official YAM / Isaac Lab-Arena / Inspect environment with the versions
below, an NVIDIA GPU for rendering and existing Dream credentials for the model
command. Run these commands from this task directory. Use new output paths.

```sh
# Load the environment configuration of your installed official YAM runtime.
source /path/to/IsaacLab-Arena/yam.env

python entry.py demo.py /absolute/path/to/demo-output --stage outside_full
python model.py /absolute/path/to/model-output --task ./task.py --max-steps 3600 --rows-per-request 4 --driver-check-override
```

`entry.py` and the model's `--driver-check-override` disable only the RTX driver
version check inside the process, as used with the tested L40S driver 565.57.01.
They do not modify the driver, physics, task or model inputs. `model.py --task
./task.py --check-imports` performs an import-only check without creating a model
session. Physics is CPU PhysX at 1 kHz; public robot commands run at 20 Hz.

Validated versions: Python 3.12.16, Isaac Sim 6.1.0.0, IsaacLab 17.0.2,
IsaacLab-Arena 0.3.0, yam-arena 0.1.0, Inspect Robots 1.0.0, Inspect Robots
Dream 0.1.0, dreamlabs-client 1.1.3, NumPy 2.3.1 and SciPy 1.17.0.

The demonstration writes `result.json`, `runtime.json`, `trace.jsonl`,
`setup.png`, `final.png` and `demo.mp4`. After saving that successful episode,
it resets the same simulator and records a separate `reset_check/` directory.
The model writes official Inspect logs/actions/frames, `registration.json`,
`measurements.jsonl`, `continuity.jsonl` and `last-measurement.json`.
`Runtime.measure` is only wrapped for separate logging; those measurements are
not policy observations. Session replacement aborts the model trial.

The submitted videos retain every frame, including the initial frame: the demo
is 2406 frames / 120.30 s and the model is 3601 frames / 180.05 s, both 20 fps.
The model video shows the actual three policy cameras, with its top view enlarged
above the two wrist views. The demo additionally has a review-only overview.

## Fixed setup and scoring

The scene uses two native YAM arms with parallel grippers, bases at
(0,+/-0.305,0.75) m, and the standard 1.10 x 1.30 m wooden tabletop with top
z=0.7475 m and bounds x=[-0.19,0.91], y=[-0.65,0.65] m. The tablet is a rigid
180 x 240 x 7 mm glass/aluminium analogue of mass 480 g. It starts screen-up,
center (-0.040,0.100,0.75415) m, yaw -90 degrees; local +Y is portrait-up and
local +Z is out of the screen. Each attached rigid polymer-laminate panel is
134 x 150 x 1.3 mm and 35 g. The support is already attached; installation is
not part of the task. There is no spacer, added grasp handle or initially open
flap. Material names are prototype assumptions, not measured product data.

The collidable camera support has a 160 x 160 x 16 mm foot centered at
(0.80,0,0.7555) m; mast/arm/drop radii are 20/16/9 mm and its housing is
50 x 42 x 22 mm. Endpoint positions come from the pinned station camera pose
and `mount.py`; the optical pose of the standard camera is unchanged. The
floor, robot mounts and standard sensors come from the pinned SDK. Exact panel
anchors, initial transforms, dimensions, friction and forces are in
`MECHANISM-SPEC.md`, `mechanism.py`, `mount.py` and `runtime.py`.

At trial end, full score requires screen tilt 50–70 degrees above the desk,
portrait roll <=15 degrees and screen facing world -X within 30 degrees. Tablet
bottom and rear support must both contact the desktop, all task bodies must lie
within its bounds, no robot or other object may support the assembly, and hands
must be at least 30 mm clear. These conditions must hold for the final five
seconds; translation must remain within 5 mm and rotation within 3 degrees of
the first pose of that interval. Score 0.5 uses tilt 25–<50 degrees under the
same other conditions. All other outcomes score 0. `scoring.py` is authoritative.
Bounds include a 2 mm simulator tolerance; no hidden fold-angle or magnet-gap
success gate is used. Hand distance uses conservative hand/finger bounds.

For physical evaluation, use an inclinometer and reference photos for angles,
a ruler for clearance/bounds, and a fixed camera with scale/orientation
references for five-second stability. Directly observe both tabletop supports
and withdrawal of the hands. Simulator contact telemetry implements those
observable support conditions; no force gauge is required to score the task.

The demo ends at tilt 60.26486 degrees, hand clearance 68.783 mm and both
supports on the desk. Independently recomputed final-five-second movement is
0.055387 mm / 0.019828 degrees. Stable hand-supported and initial flat negatives
score 0. A same-instance 40-action reset returns the flat folded state, score 0
and tilt 0.017838 degrees. Reset restores all object and robot states and clears
timers; there are no consumables. Diagnostic fixture stages and JSON snapshots
are retained for debugging, but are explicitly marked as fixtures and cannot
claim full fresh-start proof. `outside_full` rejects a capture-fixture argument.

## Fidelity and assets

This is an estimated serial two-panel back support inspired by folding tablet
stands, not a calibrated Qek W90 replica or a model of diagonal origami creases.
Rigid panels and passive hinges omit compliant skins, adhesive peeling and
damage. Two finite equal/opposite Gaussian endpoint attractions have 1.5 N peak
force each and 6 mm scale; there is no snap, weld or driven fold joint. Forces,
friction and inertia are estimates. All three task-body collision pairs remain
active. Original robot effort/velocity limits and finger actuators are retained;
shoulder/wrist stiffness is LEFT x8 / RIGHT x4 with square-root-scaled damping.

Read-only overlap telemetry records finite contact compliance. In the submitted
demo, peak depth 0.525202 mm is confined to the distal 0.596450 mm of a 150 mm
leaf, with 48 ms total above 0.25 mm and none above 1 mm; other task-body pairs
have zero overlap. Later transfer, release and final hold have zero overlap.
This was reviewed as localized terminal contact compliance, not broad sheet
crossing. The monitor never changes forces, poses or scores.

Task code and procedural object geometry are MIT-licensed under `LICENSE`.
No manufacturer CAD, image, logo or scanned asset is included. Reused helper
provenance is in `PROVENANCE.json`. External robot/runtime assets remain supplied
by the installed SDK and are not redistributed in this task directory:

| Asset/runtime | Revision | License |
| --- | --- | --- |
| i2rt YAM robot model / URDF meshes | 120c3c81400171174604e503943f8d1ebc891058 | MIT, installed `LICENSE.i2rt` |
| yam-isaac-lab-arena station/runtime | 0059d02a1a91bc6834c1d0846022891df0c78730 | Repository README states no repository license yet; robot model separately MIT |
| Isaac Lab-Arena | c7edf3a9d5d8276565208f72422f8f8a0806d6cf | Apache-2.0, installed `LICENSE.md` |

Isaac Sim is the externally installed NVIDIA runtime under its own distribution
terms. No SDK binaries, robot meshes or third-party textures are copied here.
