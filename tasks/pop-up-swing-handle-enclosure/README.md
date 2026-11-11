# Pop-up swinghandle enclosure access

Box 1 is the enclosure on the robot's right (negative Y). Press its red cap, lift and turn its swing handle to release the multipoint lock, open its lid past 60 degrees, and leave every part of that enclosure untouched by both arms for 20 seconds.

Box 1 is the enclosure on the robot's right (negative Y); box 2 is in the middle and box 3 on its left. The setup image and `setup-inventory.json` define this numbering. Coordinates use X forward from the arm bases, Y to the robot's left and Z up.

The scripted demonstration scored **1.0** in 1,530 genuine controls: 76.50 simulated seconds, a final opening of 62.418 degrees and a continuous 20.048-second hands-off hold. Its 1,531 video frames include the initial reset frame. The genuine Dream-1 beta trial with the explicit robot-right target instruction scored **0.0** after all 4,198 controls, 1,042 requests and one continuous session, without an API or evaluation error. Its 4,199 frames per camera include reset. The target lid remained closed. Earlier decoding/service-error attempts are unscored; an earlier complete ambiguous-input trial is diagnostic only. No trial was resumed or stitched.

## Setup and fidelity

Three bench-bolted steel-like rigid enclosures have nominal outer dimensions 200 × 220 × 112 mm, 200 × 200 × 12 mm lids, 125 × 22 × 14 mm lever bars, 12 mm red caps and rubber-like gaskets. Shell centres are (0.25, -0.32/-0.08/0.16, 0.7975) m; tabletop Z is 0.7475 m. All native fixture geometry receives the same translation (+0.25, 0, -0.0525) m. Both stock YAM bases remain at (0, ±0.305, 0.75) m with the standard Arena station, illumination, top camera and wrist cameras.

`setup-inventory.json` includes every fixture body, primitive, local dimension, mass, inertia, material parameter, joint and actual settled pose. The frozen candidate README explains primitive size conventions, the 0.5 kg lid plates, 0.12 kg handles, 0.05 kg hubs and 0.01 kg pivots/caps. Fixed shells represent physical bench bolting. Robot meshes and the standard station are external pinned SDK assets.

This is a functional rigid-body approximation. Passive finite-compliance cams/catches replace source equality constraints; source SI joint-friction efforts, springs and gasket torque are retained. There are no explicit multipoint rods, flexible seals, fracture or product-specific hardware calibration. The arms use total 27/7 Nm ceilings including gravity; each finger actuator is capped at 50 N. The scripted teacher uses measured simulator state and wrist forces to plan genuine joint targets.

Industrial swinghandles nest in a housing and pop out before actuating a cam or multipoint linkage, as described by [Southco](https://southco.com/en_us_int/latches/swinghandle-multipoint-systems/actuators-and-integrated-systems/h3-swinghandle-multi-point-system). This scene is not a product-exact H3 model. Nearest board tasks and six primary benchmark catalogs are compared in the two `evidence/S1-*-comparison.json` / `S1-board-nearest-review.json` files. Generic opening, sequencing, color, layout and hold duration are not claimed new.

## Scoring, limits and reset

At every 1 ms physics sample, an untared wrist-force norm above 80 N continuously for 0.1 s permanently zeroes the episode. Otherwise the final tier is: 1.0 for lid >60 degrees with all robot links clear of every target-enclosure part continuously for 20 s; 0.5 for final opening >60 degrees without that hold; 0.25 for a released lock and final opening >10 degrees; otherwise 0. The exact source scorer covers the boundary cases.

The limit is 4,198 absolute 14-D YAM controls at 20 Hz, or 209.9 simulated seconds. There are 50 native physics samples per control. Reset restores roots, task joints and passive mechanism memory, performs the actual SDK settling sequence, then starts a fresh task clock. Use object_jitter=0. Physical case 42 is fixed; Inspect-derived integer reset seeds pass unchanged and do not change fixture parameters. The reported Dream run used CLI seed 42 and Inspect-derived reset seed 3867854786; its physical case remains 42. No multiseed robustness is claimed.

## Reproduce

Use an existing compatible Arena/Inspect runtime and Dream credentials. Runtime API versions and 74 installed source pins are checked before execution. From this folder, set PY to that runtime's Python and choose writable absolute paths outside the task folder:

```bash
"$PY" -I -B run.py prepare --cache /absolute/writable/s1-review-cache
"$PY" -I -B run.py setup --cache /absolute/writable/s1-review-cache --out /absolute/writable/s1-setup
"$PY" -I -B run.py demo --cache /absolute/writable/s1-review-cache --out /absolute/writable/s1-demo
"$PY" -I -B run.py model --cache /absolute/writable/s1-review-cache --out /absolute/writable/s1-model
```

The loader downloads the immutable 29,288,085-byte review archive, verifies its SHA256 and every member, then verifies all visible runtime sources and the six-channel physical review receipt. `--archive /absolute/path/S1-v7-frozen-physical-review.tgz` supplies the exact archive offline. The cache is external, and every subsequent use checks its contents. The process-only S1_LEGACY_RTX=1 override is available for the tested older graphics driver; no SDK files are changed.

The model uses Dream-1 beta through Inspect, four executed rows per request, one continuous session and no automatic request retry or session recovery. The task-local adapter saturates finite gripper fractions to [0,1] before the standard decoder, exactly matching the native YAM embodiment's own saturation. Arm-joint predictions are unchanged. Inspect’s standard clamp and per-step delta-limit guards remain enabled. The independent review exactly reproduced every executed action through the recorded guards. Three consumed gripper values exceeded the original decoder's slack and were saturated to the native [0,1] bounds; all raw predictions and projections are preserved. Full prediction shape/finiteness, raw predictions, projected grippers and actual commands are logged. A separate parent verifies Inspect's scored result, native history/accounting, continuity and complete raw frame sequence; child exit code zero alone is insufficient.

The demo took about 13 minutes on a shared GPU server with CPU PhysX and RTX cameras. The final explicit-target model trial took 26.8 minutes including initialization, inference and final checks; wall time varies with load and latency. Camera observations are 320 × 176; publication videos retain the complete synchronized trial in a three-view 1280 × 720 composite with a separate static result panel. The scripted demo is grainy; the setup image uses actual cleaner model-server reset observations.

The frozen core contains historical development annotations and optional source variants. They are retained byte for byte; this README and the source-bound final evidence describe the selected SH-T1 task. Use the outer entry commands above for the reported demo/model. SDK dependencies retain their upstream licenses; no third-party robot mesh is redistributed here.


The declared instruction explicitly locates the existing Box 1 on the robot's right
(negative Y). declared-input/task.draft.json records that spatial clarification;
the wrapper checks it against the frozen native instruction and target fingerprint.
It replaces only the CLI and registration instruction, and checks every prompt
handed to Dream. The unchanged native verifier reads this declared instruction
when checking the final record; all original score, clock, frame, status and
continuity checks remain. The physical target and verified demo are unchanged.
An earlier valid trial with the ambiguous name alone is diagnostic evidence.
