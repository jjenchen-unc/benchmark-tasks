# Mount a zoom lens nose-down and retain 50 mm

The actual Dream-1 beta trial scored **0.0** over 3,034 controls and 751 successful requests in one continuous provider session. The privileged-state scripted demonstration scored **1.0** in 2,402 controls. Both use the same physical source, instruction, action limits and scorer. The model uses the disclosed public gripper saturation adapter described below.

The robot must choose a compatible lens, fit it to the downward-facing camera, align and rotate the bayonet until the physical pin engages, set 50 mm, then release every camera and lens for the final continuous 20 seconds. An unlocked seat earns 0. Lens selection matters: the two compatible lenses have different passive zoom friction under gravity.

## Setup and hardware

This fixed source-seed 42 case contains two camera bodies, three free zoom lenses, two rigid foam-tray approximations and the overhead camera bracket. Task-local geometry is translated by (0.25, 0, -0.0525) m. Camera axes and all compound shapes, estimated masses, joint definitions, friction and material parameters are enumerated in b6-v30/candidate/setup-inventory.json and b6-v30/candidate/fixture.json; SI dimensions below are full dimensions unless explicitly called half-extents. No optical/electronic camera function is simulated.

| Assembly | Root position (m) | Estimated total mass (kg) | Material/model |
|---|---|---:|---|
| body_left | 0.2, -0.28, 0.8775 | 1.26358 | rigid camera housing/metal-like mount/polymer button |
| body_right | 0.4, 0.28, 1.2475 | 0.969575 | rigid camera housing/metal-like mount/polymer button |
| lens_orange | 0.37, -0.2, 0.7675 | 0.344731 | rigid polymer/rubber-like lens housing and metal-like flange |
| lens_blue | 0.27, 0.06, 0.7675 | 0.470193 | rigid polymer/rubber-like lens housing and metal-like flange |
| lens_purple | 0.27, 0.2, 0.7675 | 0.449983 | rigid polymer/rubber-like lens housing and metal-like flange |
| tray_left | 0.37, -0.27, 0.7575 | 0.624 | rigid foam approximation |
| tray_right | 0.27, 0.13, 0.7575 | 0.624 | rigid foam approximation |

The table is 1.10×1.30×0.06m, centered at(0.36,0,0.7175)m with top z=0.7475m. Genuine six-axis YAM arms have bases(0,+0.305,0.90)m and(0,-0.305,0.90)m. Each base sits on a0.20×0.20×0.1525m rigid pedestal, centered at z=0.82375m. The public action is14D: six joint targets and one normalized jaw command per arm, at20Hz. Each arm retains the native27Nm shoulder/7Nm wrist **total including gravity** effort bounds and40N per finger. A task-local internal gravity-target offset keeps PD+gravity under the same implicit drive caps without changing public targets, gains or assets.

The right mount is(0.40,0.28,1.2475)m and its optical axis points down, R=diag(1,-1,-1). Its fixed support has a40×40×582mm post centered at(0.40,0.45,1.0385)m and60×200×12mm crossbar centered at(0.40,0.35,1.3235)m. The post bottom meets the table at z=0.7475m, post/crossbar overlap is40×20×12mm and the camera rear housing meets the crossbar underside at z=1.3175m over60×75mm. These are rigid fixed-fixture approximations, with no bolt flexibility or structural certification. Final actual path/contact evidence is required in the publication receipt.

Blue and purple lenses fit the right mount; orange belongs to the other family. Zoom spans18–55mm over90degrees. For the blue lens, static and dynamic zoom friction are both 0.0695388 Nm. For purple they are both 0.0102414 Nm; for orange both are 0.06 Nm. Actual pinned PhysX readbacks confirm the per-lens static/dynamic pairs; the friction-property viscous slot is 0, with separate authored zoom drive damping 0.002Nm·s/rad. The 0.15 kg translating front element creates an ideal nose-down helix gravity torque 0.01873572 Nm. This source-level comparison explains the retention choice; it is not a measured real-camera breakaway test.

## Scoring and duration

1.0: a compatible lens is seated (axial gap <=1 mm, radial offset <=2.5 mm, tilt <=4 degrees), physically captured and pin-locked within 4 degrees, at 48–52 mm throughout the final continuous 20 seconds without robot contact on any camera or lens. 0.5: a compatible lens is seated and locked throughout that final hands-off period, but the focal setting is not maintained in 48–52 mm. Otherwise 0. Any unmounted/unheld lens falling downward faster than 0.6 m/s, stop/pin reaction above 1.5 N m for 50 ms, wrist force above 80 N for 100 ms, or invalid/missing telemetry vetoes all credit. Numeric credit uses the measured per-substep history and the source-bound task-specific review receipt.

3,034 genuine 14D controls at 20 Hz (151.7 simulation seconds), including the final 20-second hold. Failure, validated full success or timeout ends the episode; stepping after termination raises before physics.

At 20 Hz every public action advances 25 native 0.002 s physics substeps. The 3,034-action limit includes the final continuous 20 s hold. A physical failure or time limit terminates the episode; stepping after termination raises before further physics. The disclosed privileged-state scripted controller has a whole-path bound 3,033 controls including 401 final hold controls; its private 30 N planning force guard, 20 mm endpoint pose guard and 5 mm / .08 rad grasp checks do not replace the task rubric.

## Reset and physical approximation

Every Inspect seed restores the same documented source-seed-42 physical case, object_jitter=0. Restore original roots, ten joint positions/velocities and ready/open arms, disable capture/pin constraints, set zoom to18mm, settle before clearing episode history. No dependence on a particular harness-derived episode ID.

Estimated-parameter rigid PhysX camera-service mockup; no optical image formation or measured real-camera calibration. Hinges/slides, authored SI joint friction and an energy-paired finite-stiffness zoom helix model passive creep. Contact-triggered capture/pin constraints approximate bayonet engagement; full normal+tangential robot contacts drive pull-out. Robot supports and overhead camera fixture are rigid fixed fixtures. See README for dimensions and limits.

Zoom translation/rotation are paired by F=-20000(x-hθ)-20(v-hω), τ=-hF, h=0.020/(π/2)m/rad. Capture requires compatible physical alignment and seat pressure; the pin and passive centering use constraints. Capture pull-out uses complete WORLD normal+tangential robot contact forces across all three descendant parts of each lens, plus gravity, with the original2N/24ms release condition. The corrected producer preserves normal contact caches used for held/touch and seat pressure. The port approximates a native equality coupling with finite stiffness and physical constraint activation, rather than simulating a manufactured camera.

## Evidence and reproducibility

The original V29 genuine reference completed 2,402 controls and 60,050 native physics samples in 120.1 simulated seconds. Its measured rubric is 1.0: 25.898 s continuous untouched locked hold at 49.004773 mm. Final axial gap is 0.372291 mm, radial error approximately zero, tilt 0 degrees and lock error 0.814594 degrees. No drop, overload, measurement error or controller failure occurred. The historical raw journal retains its original unapproved public score 0; the source-bound ROOT receipt subsequently approves all nine channels and credits this same actual reference as demo 1. Original source/History/action/force caches and all three H.264 320×176 videos (2,403 decoded frames each) passed offline review; all 144 archived files were SHA-verified in backup storage. Physical manifest: `cc0330bc346c05cfdfdc90a6d14a7b9fd4d7f511f3418b031843ab808be289af`. The outer-only V30 validation derivative has manifest `f7db7f019989b30f88921430caeded1aed0df1f108061b72c0c21afd17a43994`; every physical candidate byte remains exact. It preserves original manifests and data and replaces a generic second identical-action world with strict one-world/source/component/full-force proof requirements. All nine channels and source/support review are approved. The restored trial completed without an API/evaluation error, with all 3,035 actual frames per camera preserved and full native telemetry. Two earlier provider-invalid trials (session replacement, then provider authorization failure) have no score and are excluded from these reported results. The current source-bound whole-path bound is 3,033 controls including 401 final hold controls, within the native 3,034 limit.

The readable physical core is `b6-v30/`; original development notes and historical unapproved journal scores are retained verbatim as provenance. `envelopes-v3/` preserves the evaluated external model helper and shared gripper conversion. No installed SDK file is modified. All finite returned public gripper values are saturated to the existing [0,1] interval before the original decoder; arm bytes, row count and row order are preserved. Official transient retries may continue within one provider session. Expired/replacement sessions and another reset are rejected.

Use the pinned IsaacLab-Arena runtime (Isaac Sim 6.1.0.0, IsaacLab 17.0.2) and its NumPy dependencies. The first preparation downloads the immutable 54,318,811-byte physical review archive, verifies its SHA256 and all 35 regular members, then rechecks the source-bound nine-channel receipt. Expanded evidence is 618,954,281 bytes. `--archive /absolute/path/B6-v30-frozen-physical-review.tgz` supports the exact offline asset. Cache and output directories must be outside this task folder. All media remain outside Git.

```sh
source /data/IsaacLab-Arena/yam.env
B6_PY=/data/IsaacLab-Arena/.venv/bin/python
$B6_PY -I -B run.py prepare --cache /tmp/b6-review-cache
$B6_PY -I -B run.py setup --cache /tmp/b6-review-cache --out /tmp/b6-setup
$B6_PY -I -B run.py demo --cache /tmp/b6-review-cache --out /tmp/b6-demo
$B6_PY -I -B run.py model --cache /tmp/b6-review-cache --out /tmp/b6-model
$B6_PY -I -B run.py verify-model --cache /tmp/b6-review-cache --out /tmp/b6-model
```

Each new setup/demo/model output must be absent. The demo runs the unchanged privileged-state reference controller and recomputes `approved-demo-score.json` from the native history and the approved receipt; it preserves the original raw journal. `verify-model` rejects provider/evaluation errors rather than labelling them score zero. Use existing authorized Dream credentials for the model command. The restored model took approximately 20.3 minutes on the validation L40S; the reference took approximately 8.5 minutes. Runtime depends on renderer and service latency. The 151.7-second simulation budget and single-session validity gate are unchanged. Of the returned public gripper values, 1,348 were projected to [0,1]; every arm value and returned row/order was preserved. This is a task-local disclosed policy conversion, not an SDK change.

## Capability and overlap

The construct combines compatible lens selection, inverted mounting, physical bayonet/pin locking and friction-dependent passive zoom retention after contact release. BEHAVIOR camera-on-tripod placement and Canon’s index/rotate-click/zoom operation are primary context; the mockup adds neither optical calibration nor image-quality scoring. The six-suite and nearest-board comparisons are scope-limited in importance-review.md. Optical-bench lens positioning, depth-camera setting tabs, wrist-camera clips, camera-clamp removal and camera-tripod/cable tasks are neighboring capabilities. Their existence is disclosed; no exhaustive absence or organizer acceptance claim is made.
