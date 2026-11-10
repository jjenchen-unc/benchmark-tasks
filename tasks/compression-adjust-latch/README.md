# Adjustable enclosure compression latch

Adjust the unloaded grip-length knob, turn the wing handle clockwise to its locked position, then leave both hands off every enclosure part for 20 seconds.

The fixture is a metal equipment enclosure with an adjustable compression latch. The knob sets the pawl reach while unloaded; the wing handle rotates the cam to clamp the flange. Forcing the cam beyond the specified torque/time limit irreversibly strips it until reset. All equipment is fixed and unpowered.

## Measured results

| Trial | Score | Controls | Evidence |
| --- | --- | --- | --- |
| Dream-1 (`dreamlabs/dream-1-beta`) | 0.0 | 3848 | Completed Inspect trial, time limit, 954 requests, one continuous session; 3849 actual frames per camera |
| Saved-action YAM demonstration | 1.0 | 1959 | 48,975 physical 2ms samples, 97.95 simulated seconds, final qualifying hold 25.398s |

Both trials ran on the pinned Arena/YAM stack with CPU PhysX and an NVIDIA L40S for RTX cameras. The demo ended with gap 0.761059mm, clamp186.1036N, handle3.1415901rad and no damage/failure/robot contact. The full original3848-control limit is retained. Model score0 is an actual completed model evaluation, not an infrastructure error. A preceding missing-credential failure happened before any model session; it was excluded, and only the subsequent single completed model trial is reported.

The demonstration replays measured14D actions through the normal robot API. Its original planner read simulator state; replay does not set task state. The model used only the declared observations through Inspect. No native/Panda score is substituted for these YAM results. Actual source bindings, results, continuity log, setup state/clock checks and a compressed Inspect record are under `evidence/`.

## Reproduction

Use Python3.12.16 and the exact installed versions/source commits in `dependency-pins.json`; no historical development directory is required. IsaacSim6.1.0.0, IsaacLab17.0.2, Arena0.3.0, Inspect1.0/YAM0.38/Dream0.1, Torch2.11.0+cu128, NumPy2.3.1 and SciPy1.17.0 were verified. CPU physics still requires an RTX-capable GPU for the three cameras. FFmpeg/libx264 or imageio-ffmpeg, NumPy and Pillow provide capture. MuJoCo3.3.7 is optional for source recompilation and is not needed by replay. Existing SDK assets retain their licenses; `LICENSE.i2rt` and notices are included.

After activating that environment, run from this task directory:

```bash
python -B run_demo.py --preflight-only --out /absolute/unused
python -B run_model.py --preflight-only --out /absolute/unused
python -B capture_setup.py --preflight-only --out /absolute/unused
python -B run_demo.py --out /absolute/new/demo
python -B capture_setup.py --out /absolute/new/setup
python -B run_model.py --out /absolute/new/model
python -B tools/check_package.py
```

Every output path must be new and outside the task directory. Dream credentials use the installed client's environment/credential-file mechanism. `run_model.py` invokes genuine Inspect Dream-1,3848 maximum controls,20Hz, CPU,320x176 cameras, four rows/request, seed42 and no jitter. The fixture remains the disclosed physical source42 when Inspect derives an episode seed. Continuity rejects session changes, restarts and fallback; the verifier requires a completed error-free record and steps+1 synchronized frames. Wall runtime depends on hardware and model latency; the measured full model trial completed within the90-minute resource window.

`run_demo.py` freezes a task copy, verifies the29 runtime files and ten calibration bindings, resets once and sends the exact1959 saved absolute joint/jaw actions. It requires terminal score1.0 and records actual three-camera H264/yuv420p20Hz videos. Both arms have six joints plus one normalized jaw. The published overview demo has1960frames/98s; the extra frame is the initial observation. No frames, labels or terminal holds were fabricated.

`capture_setup.py` uses the same reset and40 render-only warmups. The submitted setup image was additionally checked with direct physics-counter/DT/observation-time reads: counter250 before/after, DT0.002, time0, zero episode actions/samples and every robot/body/joint state unchanged. Setup and demo use640x360 RGB. Optional demo/setup `--driver-check-override`, or model environment `S4_DRIVER_CHECK_OVERRIDE=1`, changes only that process's RTX driver-check argument.

## Setup and scoring

`setup-inventory.json` gives every primitive, body, material, principal inertia, passive joint, support and camera calibration. The13 fixture primitives are the300x400x127.5mm metal shell;300x400x2.5mm flange;288x388x6.8mm visual elastomer gasket; two radius6mm/height40mm hinge knuckles;300x400x4mm lid; radius22mm/height4mm escutcheon; radius12mm/height6mm hub;8x50x25mm wing;6x6x0.6mm painted wing mark; radius15mm/height18mm knob;2.4x12x0.6mm knob mark;24x20x6mm pawl tip. Gasket and painted marks are noncolliding; gasket compression uses the disclosed force law. All other fixture surfaces use the retained metal/contact approximations.

The root is(0.28,0,0.7475)m/yaw-105deg on the standard Arena table. Two YAM bases are(0,+/-0.305,0.9)m on fixed240x120mm supports whose tops meet the actual base-mesh undersides. The installed station, camera stand and three cameras are included in the inventory. Body masses are enclosure16.36890699kg(fixed), lid1.45kg, handle0.09kg, knob0.03kg, pawl0.03kg. Authored zero-joint poses are distinguished from actual gravity-settled reset poses. There is no object jitter.

Every credited tier requires20 continuous terminal seconds with handle>=175degrees, clamp>20N and no robot contact on any enclosure part. Gap<1mm earns1; gap<2mm earns0.5; otherwise a locked latch earns0.25. Any missing/nonfinite measurement, cam stripping or wrist>80N persisting100ms yields0. Cam torque>3Nm for100ms strips permanently. Knob locks above2N pawl clamp and releases below0.5N. Full reset restores mechanism and history; nothing is consumed. YAM total drive limits27/7Nm include gravity, with left50N/right40N fingers; native235N capability is not claimed.

## Fidelity and provenance

Geometry, physical masses/inertias, passive thresholds, scoring and genuine controller inputs are unchanged from the verified runtime. Balanced finite-compliance cam and screw restraints approximate native equality; contact friction, solver and effective numerical inertia differ. Lid armature20 and pawl250 are generalized numerical inertias, not extra physical masses. Details are in `runtime/DEVIATIONS.md`.

The29-file `runtime/` subtree, all ten calibration-bound inputs, native provenance and saved actions are byte-identical to the measured successful source. Historical draft/development files remain as provenance and may describe earlier pending work or placements; this README, final `task.json` and final evidence describe the submitted configuration. Those historical paths are not runtime dependencies. `runtime-manifest.json` binds runtime; `package-manifest.json` binds portable files. `tools/check_package.py` checks integrity and serialization, not hardware fidelity or task acceptance.
