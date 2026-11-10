# Lower a fragile cabinet with a hydraulic release valve

Use the red jack to lower the already raised cabinet onto the steel rails. The
robot manipulates the release knob; it cannot earn credit by hand-lowering the
cabinet. The task evaluates metering a load-driven hydraulic release, a gentle
transfer of support, and a released stable finish. The selected case is the
original `J2_lower_cabinet`, setting S, fixed physical seed 42.

## Reproduce a trial

Use the existing Isaac Sim / IsaacLab Arena / YAM / Inspect Robots / Dream stack
pinned in `runtime/candidate/dependency-versions.json`, `runtime/runtime-source-pins.json`
and `runtime/provenance`. CPU PhysX integrates at 500 Hz; GPU RTX renders the
three 320×176 cameras. The model receives standard RGB and 14D joint-position
observations and emits 14D joint-position controls at 20 Hz. Runtime imports no
MuJoCo or external source workspace. The native authoring snapshots remain for
provenance and are not required as an installed simulator.

All paths below must be absolute, with fresh output directories. Use the SDK's
Python interpreter and normal environment; configure Dream credentials through
its usual credential mechanism. The task does not store or print credentials.

```bash
python -B run.py prepare --cache /absolute/review-cache --preflight-only
python -B run.py setup --cache /absolute/review-cache --out /absolute/setup
python -B run.py replay --cache /absolute/review-cache --out /absolute/demo
python -B run.py model --cache /absolute/review-cache --out /absolute/model
```

`prepare` checks the original native bundle and the archived, source-bound
physical review without launching a simulator or model. The source manifest fixes the public verification-cache release URL, byte length and SHA256. On a fresh cache, the wrapper downloads that public asset without credentials, checks every archive member against its declared size/hash, rejects links and escaping paths, then extracts atomically outside the submission directory. For offline use, add `--evidence-archive /absolute/review-evidence.tar.gz` to supply the same checked asset. No generated media are committed with the source, and no runtime source is edited. `setup` performs the ordinary reset and captures initial cameras without
scored controls. `replay` loads the same production credit adapter as the model
and executes the genuine state-aware reference. Its planning reads simulator
state, but every task-changing action uses the robot's ordinary control API;
there are no load pose/joint edits or evaluator-applied forces.

The model command invokes Inspect Robots with `dreamlabs/dream-1-beta`, four rows
per request, `task_score`, `control_hz=20`, `device=cpu`, `object_jitter=0`, seed 42,
3,210 maximum steps, and stored actual frames. It uses one continuous Dream
session and one reset. Official transient transport retries may occur in that
same session; expiration or a replacement session invalidates the trial. There
is no best-of-run selection or reset recovery. The exact evaluated outer model
helper, verifier and their SHA256 manifest are in `execution/workflow/`.

On the tested runtime, the process-only `HYDRAULIC_LEGACY_RTX=1` option disables
AppLauncher's RTX driver-version check; it makes no system or driver change.
The original source fingerprint is preserved, while the declared outer execution
helper differs from the original transport wrapper. These are separate source
bindings, not a claim that every executable byte is the original bundle.

## Setup, reset and inventory

`task.json` describes the actual evaluated mount and layout. The table top is
Z=0.7475 m. The left YAM base is (0,0.305,0.75) m. The right base is
(0.68,-0.15,0.75) m with yaw π, upright on a 120×120×2.5 mm steel plate whose
bottom rests on the table and top matches base Z=0.75 m. There are no robot
risers. Both arms start at [0,1,1,-0.6,0,0] radians with open 93.9 mm jaws.
Per-arm total joint effort limits, including gravity, are 27 Nm on joints 1–3
and 7 Nm on 4–6; both independent fingers are limited to 40 N.

Both fixed jacks, both full steel rails, six support posts, retaining lips, free
cabinet, four side glass panels, two shelves and four ornaments remain. The
cabinet is approximately 9.02987 kg; source `cabinet_weight=88.583` is newtons,
not kilograms. Its plinth starts 50 mm above the rail plane. The red ram begins
at 55 mm extension, the blue at zero. Fixture geometry is a single translation
(+0.12,-0.30,-0.0525) m of the native geometry, without scaling or rotation.
`runtime/candidate/fixture.json` enumerates exact runtime primitive dimensions,
poses, masses, inertias and friction. `physical-inventory.json` includes the
original visual/collision inventory. Historical notes claiming unchanged robot
bases are superseded by the actual mount above and the archived reset telemetry.
The entire native variant inventory remains in `variant-preservation.json`;
other goals/settings are not additional publicly demonstrated tasks.

Reset restores both passive jack mechanisms and the free cabinet, performs the
ordinary unscored settling, refreshes cameras, and clears history. Every Inspect
harness seed routes to the same explicit physical seed 42; no object jitter or
hidden seed-dependent layout is used. No consumables need replacement.

## Numeric score and limit

The episode permits at most 3,210 actual robot controls: 160.5 simulated seconds
at 20 Hz, with 25 two-millisecond physics samples per control. The current
terminal score counts. Success or a permanent failure ends the trial.

Permanent zero overrides progress for any sampled glass normal contact >15 N,
rail landing speed >25 mm/s, cabinet tilt >10°, cabinet underside more than
40 mm below the rail plane, unavailable/nonfinite measurements, or wrist-force
guard termination. The default gripper incoming-joint reaction norm guard is
30 N; the independent >80 N for 100 ms abort is also retained. These include
self weight and are not external-world force-vector claims.

With all safety conditions satisfied, award 0.25 for at least 10 mm of hands-off
jack-carried descent supported by corresponding oil flow, 0.5 for at least
30 mm, 0.75 for a gentle jack-mediated landing on the rails with no robot/load
contact, and 1.0 when that rail support remains hands-off continuously for
3 seconds, with cabinet linear speed ≤1 mm/s and angular speed ≤1°/s. Other
outcomes score zero. The hold is the original 3 seconds, not the 20-second hold
from other native hydraulic goals.

## Physical scope and source binding

The source-bound review checks the actual CPU contact API, clocks, drive limits,
pad/rail load transfer, pose history, wrist-force norms and supported mount.
It enables the production public score only for the reviewed task/source.
The earlier raw replay remains public zero because its channels were not yet
approved; its diagnostic native one is never relabeled. A new actual replay
through the production adapter independently achieved public 1.0 after review.

This is a rigid-body approximation. A load-dependent oil target and stiff ram
drive approximate hydraulics, with no fluid, leakage, compliance or hardware
calibration claim. The source Coulomb resistance is smoothed. Glass damage is a
normal-contact history veto rather than fracture; point-to-panel attribution
is conservative near frame boundaries. Positive pad and rail forces were
measured, but the successful trial had zero panel contact: no fresh positive
panel-load calibration is claimed. Wrist norm checks reuse the same pinned
backend's six ±20 N API cases and do not establish world-vector signs or real
robot calibration. Only the selected fixed case has this full-goal evidence.

## Actual measured results

The single actual Dream-1 trial scored **0.0 after all 3,210 controls / 160.5 simulated seconds**. Inspect reported success with no evaluation error; termination was the time limit. It used 795 requests in one uninterrupted session, with no transport retries or reset recovery. Independent verification recomputed all 80,250 active 2 ms physics samples, checked all 3,210 action/status pairs, and fully decoded each of the three 3,211-frame actual H.264 videos. The cabinet remained jack supported; there was no measured descent or safety/measurement failure. Total model process time was approximately 14.6 minutes on the tested runtime.

The publication instruction adds spaces around units for readability; the exact evaluated wording remains recorded in the original candidate metadata and consumed model helper. Its meaning, goals, thresholds and budget are unchanged.

The genuine scripted production replay scored **1.0 in 629 controls / 31.45 s**,
including 15,725 active physics samples. It lowered the cabinet 49.52 mm, landed
at 13.34 mm/s and completed a 3.014-second released rail hold. All three original
H.264 videos fully decode to 630 frames at 20 Hz; no labels, synthetic views or
extra frames are added. Its measured runtime was about 120 seconds on the tested
GPU runtime. Code, review evidence, source fingerprints, exact outer helpers and
reproduction commands are included; generated submission media are stored and
uploaded separately.

The release law is `v_mm_s=(161.49/30)*clip(phi_deg-109.651,0,90)*sqrt(pad_force_N/50)`. At nominal88.583 N it yields7.16497 mm/s per degree and644.848 mm/s at full90-degree opening. The successful demo used loaded openings0.24784–2.47004 degrees, rather than a hard automatic safe-speed cap. This arithmetic is the oil-target law, not a measured counterfactual full-open landing: that final-source failure trial was not run. The actual metering trajectory and this limitation are recorded in `evidence/metering-difference.json`.

The four original unpacked runtime source files listed in `.gitattributes` retain their
reviewed byte-for-byte endings. That attribute permits only their existing
blank line at end of file; other whitespace checks remain enabled.

Historical native provenance is distributed as `native-provenance-source.tar.gz`,
a lossless source archive containing no generated media. Before any runtime
import, the launcher restores it with the readable native source into the
external review cache and verifies every one of the207 executed native files
against its original SHA256. It never changes the source checkout. This is a
source packaging change; task physics, controllers, score, limits and the
evaluated model helper remain byte-for-byte unchanged after extraction.
