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

## Building the support for a real trial

The first review (TK-MZKG78) asked for a buildable folding support. `build/`
holds it, generated from `mechanism.py`; all positions are in millimetres,
measured on the tablet's back from its bottom edge (`assembly.png`):

| File | What it is |
| --- | --- |
| `panel1.stl`, `panel2.stl` | the two panels, 134 x 150 x 1.3 mm; `panel2.stl` has two 4.9 mm pockets, 0.9 mm deep, in one face, centred 45 mm either side of the middle and 3 mm from its free end |
| `hinge_strip.stl` | 140 mm wide: 2.5 mm thick from y 140 to 151 (rest for panel 2's end), 5.1 mm thick from y 151 to 161 (hinge land) |
| `foot_strip.stl` | 140 x 12 x 2.5 mm, y 1.5 to 13.5 |
| `assembly.png` | back view, folded and deployed side views |
| `export_stand.py` | writes the files above (`pip install trimesh manifold3d matplotlib numpy scipy`, then `python -B build/export_stand.py`) |

Tablet: an iPad (A16, 2025) Wi-Fi, 248.6 x 179.5 x 7.0 mm and 477 g. The
simulated tablet is 240 x 180 x 7 mm and 480 g. Because every support position
is measured from the bottom edge, the deployed support is the simulated one;
the iPad is 8.6 mm taller at the top. A dummy 179.5 x 248.6 x 7 mm, 477 g
plate can stand in for the iPad to protect it. With the strips, panels and
magnets the assembly weighs about 0.56 kg (0.55 kg in the simulation).

Materials: PETG for all four printed parts, 100 % infill, printed flat (each
panel about 33 g; 35 g in the simulation); 25 mm cloth bookbinding tape for the
hinges; two 3/16 x 1/32 in (4.8 x 0.8 mm) neodymium disc magnets; a
120 x 7.5 mm strip of 0.3 mm steel shim; thin removable double-sided tape.

Assembly, on the iPad's back, screen down, bottom edge toward you:

1. Steel L: bend the shim lengthways 2.5 mm from one edge to 90°. Glue it to
   the foot strip so that the 2.5 mm leg covers the strip's long bottom face and
   the 5 mm leg its outer face, centred.
2. Stick the foot strip to the back with its bottom face 1.5 mm above the
   bottom edge (lay a 1.5 mm spacer along the edge while placing it), and the
   hinge strip from y 140 to 161, thick part at the top, both centred left to
   right. The 1.5 mm gap lets panel 2's end pass under the foot strip, so the
   iPad keeps standing on its own bottom edge.
3. Press the magnets into `panel2`'s pockets, flush, and glue them. Call that
   face panel 2's inner face.
4. Hinge 2: lay panel 1 on panel 2's inner face, edges flush, magnet end of
   panel 2 away from you. Wrap tape round the two edges nearest you, so the
   panels open like a book.
5. Hinge 1: lay the pair on the strips, panel 2 underneath, hinge 2 at the
   bottom edge. Panel 2's magnet end now rests on the thin part of the hinge
   strip, 1 mm short of the thick part. Panel 1's top end lies flush with the
   thick part's outer face; tape across those two outer faces only, along
   y 150, keeping the tape off panel 2.
6. Check: stand the iPad on the desk and open the support. Panel 2 lies on the
   desk and its free end reaches just under the foot strip's steel-covered
   corner (about 1 mm clear in the simulated pose); the magnets lift the light
   panel end onto that corner while the iPad stays on its bottom edge. Pulling
   the panel end straight off should read about 100–200 g on a kitchen scale (1–2 N; the
   simulation's attraction peaks at 1.5 N, 6 mm from contact). Add a second
   layer of shim if it is weaker.

Differences from the simulation, all small: the real hinge-1 axis lies on the
5.1 mm land, 2.6 mm further from the back than the simulated axis, which
changes the standing angle by about 1°; folded, the panels lie about 2 mm
further out (on the 2.5 mm strips); the real magnet catch is strongest at
contact, while the simulated attraction is zero at contact and peaks 6 mm away;
tape hinges are stiffer than the simulated hinges; the catch lifts panel 2's
end about 1 mm off the desk. The iPad's own back magnets (for its folio
covers) may pull weakly on folded panel 2's magnets; the dummy plate avoids
this.

Scene: a desk with the robot, and the folded iPad screen-up with its centre at
the starting position and yaw in `setup` and "Fixed setup and scoring" (the
iPad's centre is 4.3 mm further up its long axis than the simulated tablet's
when the bottom edges coincide; align the centres). Mark its outline on the desk
with tape. Scoring as in "Fixed setup and scoring". Reset: fold panel 2 back
under panel 1, lay both flat on the strips, and put the iPad screen-up on its
outline.

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

Task code, procedural object geometry and `build/` are MIT-licensed under `LICENSE`. The build script uses
trimesh (MIT), manifold3d (Apache 2.0), NumPy and SciPy (BSD-3-Clause) and Matplotlib (PSF-based license);
none is bundled.
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
