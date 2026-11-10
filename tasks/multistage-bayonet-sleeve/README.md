# Remove a multistage bayonet sleeve

Take the purple sleeve with three white bars off its fixed shaft and put it in
the green tray. Leave the shaft clear and the sleeve resting wholly inside the
tray without robot contact for 20 seconds.

This is an unpowered mechanical servicing fixture. Two opposed pins constrain
the sleeve through a spring-loaded internal track: downward compression,
rotation, an axial rise, and a reversal precede extraction. A straight pull or
one uninterrupted quarter-turn cannot pass its physical ledges. The mechanism
uses rigid colliders and a passive spring rather than a policy-controlled latch.
The custom fixture tests following a constrained release path and recovering
the loose part. It does not claim to reproduce a particular commercial product.

## Setup and reset

Use two six-joint YAM arms with parallel grippers at the standard station.
Coordinates are metres from the midpoint between the bases, x forward, y left,
z up. The tabletop is z=0.7475. Each base sits at (0, +/-0.305, 0.90) on a fixed
metal extension measuring 0.14 x 0.14 x 0.15, centred at z=0.825.

The metal base plate is 120 x 110 x 16 mm, bolted at (0.28, 0.02). Its steel
shaft is 31 mm in diameter and 85 mm high, with two opposed 6 mm pins at 70 mm.
The 250 g aluminium sleeve is 58 mm in outside diameter and 80 mm long, with a
purple band and three white bars on its cap. It starts seated over the shaft.
The spring-loaded steel plunger has a 25 g body and 32 mm stroke; reset preload
is 15 mm. The green plastic tray is fixed at (0.28, 0.40), with a 200 x 150 mm
interior, its floor top 12 mm above the table and walls 28 mm high.

`setup-inventory.json`, `fixture.json` and `fixture.xml` specify each body,
collider, mass, pose, material estimate and track dimension. `FIDELITY.md`
explains the source-to-station translation and contact approximations. The
uploaded screenshot is an actual reset observation; no actions, settling steps
or render warmups were added for it. Reset restores the sleeve and spring,
empty tray and READY/open robot poses. Nothing is consumed.

## Score and limits

The scorer measures the whole sleeve relative to the tray, physical floor
support, separation from the shaft and robots, and linear/angular speed.
The sleeve must be wholly within the tray interior, touch its floor, touch
neither shaft nor robot, move below 20 mm/s and rotate below 0.10 rad/s.
This current state earns 0.5. Holding it uninterrupted for 20 seconds earns 1.0.
All other outcomes earn 0. Missing/invalid measurements or an episode failure
veto credit. A wrist force greater than 80 N continuously for 100 ms aborts and
scores 0. `scoring.py` and the preserved native containment function define
the numeric geometry; a ruler, video and a timer can check the final condition.

The model budget is 6,000 genuine 14-dimensional robot actions at 20 Hz:
300 simulated seconds. Physics uses 25 substeps of 2 ms per action, CPU PhysX
and GPU camera rendering. Total arm torques, including gravity, are capped at
27 Nm for the proximal joints and 7 Nm for the distal joints; finger limits
are 40 N. No task-body pose writes or external release forces are used by
the demonstration. The scripted planner can inspect simulation state.

## Reproduce

Use the official YAM/Arena environment with its existing `yam.env`, Python
3.12.16, Inspect Robots and the Dream policy plugin. Exact consumed runtime
sources are pinned in `runtime-pins.json` and `runtime-force-pins.json` and are
resolved from installed modules, independent of the checkout location.
Dream credentials are configured outside this directory.

From this task directory, select new absolute output directories outside it:

```bash
python -I -B package_checks.py
python -I -B driver.py --mode setup --out /absolute/new-setup --driver-check-override
python -I -B driver.py --mode replay --out /absolute/new-demo --driver-check-override
python -I -B run_model.py --preflight-only --out /absolute/new-model
python -I -B run_model.py --out /absolute/new-model
python -I -B tools/encode_model.py /absolute/new-model
```

`--driver-check-override` only disables the legacy RTX driver version check
within that process. It changes no installed driver, SDK or physics setting.
The replay executes the 1,490 saved robot actions. Its actual Brev run scored
1.0 in 74.5 simulated seconds with 26.85 seconds of uninterrupted final hold;
37,250 substep samples reported all nine required channels verified, no force
abort and no measurement error. The actual three-camera demo has 1,491 frames
per camera at 20 fps, including its reset frame.

The submitted Dream-1 (`dreamlabs/dream-1-beta`) trial scored 0.0
after 6000 actions, with termination `time_limit`.
Its actual wall time was 1404.97 seconds, including startup and service
requests. The action budget, physical seed 42 and zero object jitter match the
demo. Inspect's derived reset seed is passed unchanged to the SDK; the physical
fixture remains fixed. Runtime depends on startup, rendering and network
latency; videos retain the actual 20 Hz action timeline.

The first model trial ended on a provider `DA-6000 GATEWAY_FAULT` connection
timeout after 764 actions and produced no valid score. The submitted trial is
the subsequent complete trial. Official SDK transport retries within its single
continuous session are logged; a session change, expiration or episode restart
invalidates the evaluation. No best-of-successful-trials selection is used.

`model-verification.json` and `continuity.jsonl` in the output verify the actual
Inspect trial, step budget, scorer output, single session and three-camera
frame counts. `tools/encode_model.py` encodes those recorded RGB frames as H.264
MP4 without adding labels, frames or hold time. Demo videos are produced by
the replay, and reset PNGs by setup mode. Generated media stays outside this
source directory.

## Fidelity and provenance

The custom two-pin track retains all 290 sleeve primitives, false seat,
blind ledge and final reversal. The fixed seed gives a 308.9 N/m spring and
1.5 Ns/m damping. Materials, friction and rigid contact are authored estimates;
fracture, plastic deformation, wear and hardware calibration are not modelled.
The numerical native plunger armature is not added to physical mass. This is
a scoped fixed-case transfer, not proof that two physics engines or arbitrary
variants have identical contacts. The measured-channel evidence in `evidence/`
and `calibration.json` is preserved. Historical evidence files retain their
original scores; this README and the submitted scores describe the fresh run.
Source notices and source hashes are in `THIRD_PARTY_NOTICES.md` and
`source-provenance.json`.
