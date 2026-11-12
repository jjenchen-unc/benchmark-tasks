# Take a spring-loaded bayonet coupling sleeve off its shaft: push down, turn, lift, turn back, pull off

Take the purple sleeve with three white bars off its fixed shaft and put it in
the green tray. Leave the shaft clear and the sleeve resting wholly inside the
tray without robot contact for 20 seconds.

The sleeve is a spring-loaded bayonet coupling, like the bayonet cap on a
spring-loaded sensor probe or the coupling ring of a circular connector. Two
opposed pins on the shaft run in a two-stage J-shaped track hidden inside the
sleeve. To remove it, the sleeve must be pressed down against its spring,
turned, lifted, turned back the other way and then pulled off. A straight pull
or one uninterrupted quarter-turn stops at a ledge. The mechanism is rigid
colliders and a passive spring; there is no scripted latch. The fixture is not
a copy of one specific commercial product.

## Setup and reset

Two six-joint YAM arms with parallel grippers at the standard station.
Coordinates are metres from the midpoint between the bases, x forward, y left,
z up. The tabletop is at z = 0.7475. Each base sits at (0, +/-0.305, 0.90) on a
fixed metal extension measuring 0.14 x 0.14 x 0.15, centred at z = 0.825.

The metal base plate is 120 x 110 x 16 mm, bolted at (0.28, 0.02). Its steel
shaft is 31 mm in diameter and 85 mm high, with two opposed 6 mm pins at 70 mm.
The 250 g aluminium sleeve is 58 mm in outside diameter and 80 mm long, with a
purple band and three white bars on its cap. It starts seated over the shaft.
The spring-loaded steel plunger has a 25 g body and 32 mm stroke; reset preload
is 15 mm. The green plastic tray is fixed at (0.28, 0.40), with a 200 x 150 mm
interior, its floor top 12 mm above the table and walls 28 mm high.

`setup-inventory.json`, `fixture.json` and `fixture.xml` specify each body,
collider, mass, pose, material estimate and track dimension. `FIDELITY.md`
explains how the original fixture was placed on the station and the contact
approximations. The uploaded screenshot is a reset observation; no actions,
settling steps or render warmups were added for it. Reset restores the sleeve
and spring, the empty tray and the ready robot pose with grippers open.
Nothing is consumed.

## Score and limits

The scorer measures the whole sleeve relative to the tray, floor support,
contact with the shaft and robots, and linear and angular speed. The sleeve
must be wholly within the tray interior, touch its floor, touch neither shaft
nor robot, move slower than 20 mm/s and rotate slower than 0.10 rad/s. This
state earns 0.5; holding it without interruption for 20 seconds earns 1.0.
Anything else earns 0. Missing or invalid measurements or an episode failure
set the score to 0. A wrist force above 80 N for 100 ms aborts the episode
with score 0. `scoring.py` and the original containment function define the
geometry; a ruler, the video and a timer can check the final condition.

The model budget is 6,000 robot actions (14 joint targets each) at 20 Hz:
300 simulated seconds. Physics uses 25 substeps of 2 ms per action, CPU PhysX
and GPU camera rendering. Total arm torques, including gravity, are capped at
27 N m for the proximal joints and 7 N m for the distal joints; finger force is
limited to 40 N. The demo uses no task-body pose writes or external release
forces. The scripted planner can read simulator state.

## Reproduce

Use the official YAM/Arena environment with its existing `yam.env`, Python
3.12.16, Inspect Robots and the Dream policy plugin. The runtime sources used
are pinned in `runtime-pins.json` and `runtime-force-pins.json` and are
resolved from the installed modules. Dream credentials are configured outside
this directory.

From this task directory, choose new absolute output directories outside it:

```bash
python -I -B package_checks.py
python -I -B driver.py --mode setup --out /absolute/new-setup --driver-check-override
python -I -B driver.py --mode replay --out /absolute/new-demo --driver-check-override
python -I -B run_model.py --preflight-only --out /absolute/new-model
python -I -B run_model.py --out /absolute/new-model
python -I -B tools/encode_model.py /absolute/new-model
```

`--driver-check-override` only disables the legacy RTX driver version check
within that process; it changes no installed driver, SDK or physics setting.

The replay executes the 1,490 saved robot actions. It scored 1.0 in 74.5
simulated seconds with 26.85 seconds of uninterrupted final hold; its 37,250
substep samples reported all nine required measurement channels verified, no
force abort and no measurement error. The demo video has 1,491 frames per
camera at 20 fps, including the reset frame.

The submitted Dream-1 (`dreamlabs/dream-1-beta`) run scored 0.0 after 6,000
actions and ended at the time limit. It took 1,404.97 seconds of wall time,
including startup and service requests. The action budget, physical seed 42 and
zero object jitter match the demo. Inspect's derived reset seed is passed
unchanged to the SDK; the fixture stays fixed. Wall time depends on startup,
rendering and network latency; the videos keep the 20 Hz action timeline.

An earlier model run ended on a provider connection timeout
(`DA-6000 GATEWAY_FAULT`) after 764 actions and produced no valid score; the
submitted run is the next complete one. SDK transport retries within its single
session are logged; a session change, expiry or episode restart invalidates the
run. Runs were not selected for score.

`model-verification.json` and `continuity.jsonl` in the output verify the
Inspect run, step budget, scorer output, single session and three-camera frame
counts. `tools/encode_model.py` encodes the recorded RGB frames as H.264 MP4
without adding labels, frames or hold time. Demo videos come from the replay
and reset images from setup mode. Generated media stays outside this directory.

## Fidelity and provenance

The two-pin track keeps all 290 sleeve primitives, the false seat, the blind
ledge and the final reversal. The spring is 308.9 N/m with 1.5 N s/m damping.
Materials, friction and rigid contact are estimates; fracture, plastic
deformation, wear and hardware calibration are not modelled. The plunger's
numerical slide armature is not added to its physical mass. This is a single
fixed case, not proof that two physics engines or other variants give identical
contacts. The measurement evidence in `evidence/` and `calibration.json` is
kept. Older evidence files keep their original scores; this README and the
submitted scores describe the latest runs. Source notices and hashes are in
`THIRD_PARTY_NOTICES.md` and `source-provenance.json`.
