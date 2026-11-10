# Ordered eccentric-bar balance chain — Arena preparation

Representative goal is native `B4_bars:stack_ABC`, setting S, physical parameter seed42. This is **one historical unit, not five submitted tasks**. Arena physics, actual force/contact channels, robot feasibility, Dream1 and media are unverified. All official credit remains gated to zero. `diagnostic_ungated_score` is evaluator telemetry, never a submitted score.

Stack A on the fixed point pedestal, B on A and C on B, all along X, then release all bars for20 continuous seconds at episode end. Each lower prefix can independently earn1/3 or2/3; a complete chain earns1. Level is exactly0.06rad, heading10degrees, and the Euclidean norm of concatenated linear(m/s) and angular(rad/s) velocity is strictly below0.025, as in source. Only immediate required neighbours may touch each scored bar. Any robot contact on ANY bar resets every tier. A valid higher tier does not require lower-tier timers to remain valid: the lower tier rightly fails when an additional bar touches it. Historical early-break scoring is not reintroduced. Timers are terminal, not highest-ever. No damage/drop veto is invented: native toppling costs time; wrist80N for100ms aborts and invalidates score.1896 controls,94.8s,20Hz.

## Physical setup and disclosed adaptation

`inventory.json` gives every task object, exact masses/COM/inertia, poses and materials. A single rigid translation `(0.25,-0.15,-0.0525)m` maps native task coordinates to the station; ALL relative task geometry and height above the table are preserved. Table top0.7475m. Bars are320x50x50mm free rigid boxes approximating aluminium square tubes with sealed internal steel slugs. Orange A, blue B, purple C; black8mm end caps identify-X. Tube-shell mass190g, total mass550–850g source range; this fixed case uses native NumPy seed42/S. COM and inertia use native XML rounding, not a centred box approximation. Friction estimated: bars0.6, support0.5; PhysX average combination differs from MuJoCo's contact combination and needs verification. Tube exterior is solid for contact; no interior access is part of this goal. Torsional/rolling friction, contact softness and solver details do not match MuJoCo by declaration.

The point support is28x30x160mm fixed steel, bolted to the table; it is fixed in native source too. Free bars are never welded or resized. Two **additional bolted steel mounting risers**,140x140x213mm at `(0,+/-0.305,0.854)m`, support elevated YAM arms. Their bottoms are at table height0.7475 and tops0.9605m; YAM base link frames are at0.970m (9.5mm above the foot, SDK station convention). Robot support/clearance and fasteners require root runtime/physical validation; no clearance claim is made. Risers are visible colliders. Other station furniture/cameras remain stock. Robot base relocation is a disclosed embodiment adaptation, not a claim of stock Dream station calibration. Dream1 feasibility on this exact mounting/camera configuration remains required and unresolved.

Genuine installed YAM retains6arm joints plus gripper per arm, absolute14D actions at20Hz. Total drive ceilings27Nm(first3),7Nm(last3) include gravity through the copied task-local G/K bias adapter; actual `get_dof_max_forces` is checked. Per-finger cap40N preserves the source value (SDK default50N is not increased). This capacity is substantially below native Panda80/12Nm, and no native success transfers. Source40N fingers are preserved; this unit has no235/200N native grip claim. Source wrist30N/20ms guard holds actual arm joints from the next physics substep (task-local motor safety target override, awaiting runtime timing verification);80N/100ms abort persists. Nonfinger50N/20ms monitoring conservatively sums normal and friction magnitudes per body (an upper bound on resultants), rather than exact source sums of per-contact resultant magnitudes; this can stop earlier and remains unvalidated. Runtime contact-buffer/friction selection, sensor basis and force-gravity interpretation require actual tests before enabling credit. Source tool reach box is preserved under translation in the reference plan; the public SDK interface remains joint commands. Actual TCP outside that box while its arm touches a bar aborts and invalidates credit. Initial contact-free startup travel outside the box is permitted for the elevated ready pose; this differs from the native Cartesian API that rejects individual requested goals outside its box. Actual workspace-contact timing remains unvalidated.

## Reset and runtime

Pass `object_jitter=0`, `control_hz=20`. Inspect's arbitrary harness seed/episode ID is accepted. Every reset uses the disclosed fixed42/S case, restores free bodies through the normal Arena reset, settles0.5s, clears history and rebuilds actual contact selections. Physical substeps2ms,25 per control. Scene.update is sampled after each physics step. Poses/velocities are read from actual rigid-body tensors; contacts come from PhysX solver records, including active positive-separation solver support. All external colliders are discovered from live USD; all actual robot links are selected via `root_view.link_paths`. After termination another step raises before physics. No policy pose/joint/force/latch/score/history writes to task objects exist. Initial scene spawns and episode reset are not policy actions.

## Commands

On an allocated compatible Arena runtime, from this directory (or using absolute script path):

```sh
python probe.py --mode foundation --out /absolute/new/foundation-run
python probe.py --mode reference --out /absolute/new/reference-run
python probe.py --mode evaluator --out /absolute/new/evaluator-placed-run
python probe.py --mode reference --variant one_bar --out /absolute/new/one-bar
python probe.py --mode reference --variant two_bar --out /absolute/new/two-bar
python probe.py --mode reference --variant hold_on --out /absolute/new/hold-on
python probe.py --mode reference --variant truncate --out /absolute/new/truncate
python checks.py
python ik_check.py
```

Each probe creates and verifies a source copy and **executes that copy before cold SDK imports**. `--snapshot-only` proves this without loading SDK/physics. Optional `B4_LEGACY_RTX_OVERRIDE=1` modifies only AppLauncher process arguments. No host/SDK edits. `B4_TRACE` is set to run-local physics JSONL. Foundation uses20 genuine controls, checks tensors/cameras/substep clock, then a second harness seed reset. Runtime wall duration is unmeasured; first request is bounded30minutes. `evaluator` directly preplaces free bars solely to test contacts/hold measurement and must never count as a robot demo. `reference` contains a whole robot-only plan and an explicit state-aware teacher using fixed-case COM; any IK failure stops without relaxing tolerances. It has no object setter/force or credit writes. It is not a Dream1 model. Each physical, score, model or publication claim requires its corresponding evidence; this controller repair adds no blanket hardware gate.

## Other source goals and overlap

`stack_CBA` usesC/B/A and40x100mm support, `A_stack_pedestal` usesABC on50.8x101.6x152.4mm riser, `edge_A`/`edge_B` balance one specified bar on24/40x100mm ridges; native S/T-new parameter variants remain unchanged in their readonly packages. They are not counted as extra ports and are not silently mapped to this task's narrower pedestal or fixed physical case. This new executable represents stack_ABC only.

Cached primary board2026-11-09 20:07UTC: pendingTK-TF83Z9 requires locating one sealed weight, selecting its card, single-bar release3s; pendingTK-GFB2Z6 requires one uniform bar and two end blocks, release1s. Neither description covers an ordered chain of three eccentric bars and20s terminal contact-only holds. PendingTK-722AMJ stacks size-ordered crates with2s hold; TK-HVKKYT stacks steel case/cartons with a weight-order rule. These share real balance/stacking abilities and create a material novelty risk. No official clearance or live ownership claim; primary comparison must be refreshed before publication. Single-bar goal overlap remains stronger; no cosmetic rename is used.

Assets are task-specific generated primitives; no Panda/robosuite assets or MuJoCo runtime imports are bundled. Source notices and provenance are in `source-provenance.json` and the native ASSETS notice copied as `SOURCE_ASSETS.md`. Source descriptions are historical evidence, not Arena validation or current robot specifications. Requirements: Python3.12, NumPy2.x, installed YAM/Arena stack (IsaacSim6.1, IsaacLab3.0, Arena0.3.1); actual dependency hashes accompany delivery. This is draft packaging only; real Dream1 score, media ID and screenshot pixels are absent.

Additional evaluator commands use `--mode evaluator --variant wrist_abort`, `wrong_order`, `one_bar`, `two_bar` or `truncate`, each with a fresh output directory. Wrist abort applies a disclosed evaluator-only120N external load and checks that posttermination stepping cannot advance physics. Friction-data collection follows the verified D3 `get_friction_data(dt)` API but these B4 body/filter selections remain unvalidated. Foundation records actual mount poses and collider/body selections for root review.

## Actual-run controller repair v2

Job43681 executed1340 real controls but reached zero raw goal. First A lift
level departure was sample2787/control112/time5.574s; the bar remained gripped
while rotating to about0.94rad. No bar/pedestal contact occurred. This candidate
grasps at each native rounded COM (privileged disclosed teacher knowledge).
It preserves original task/fixture/materials/physics/scorer/motor adapter.
Loaded lift/carry/preplace/place stages use actual bar COM/orientation and
independent finger states for at most32 feedback controls per stage, bounded
to60mm translation and0.18rad rotation command correction. It requires actual
support before release, then freezes arm targets while opening from the largest
actual independent finger opening. Final native hands-free hold stays420controls.
Maximum whole path1724controls remains below1896. No object pose, force,
score, hold timer or task state is written by the controller.

Installed SDK CPU full-controller tests start at measured initial raw8 and pass
only a conditional kinematic fixture. The biased fixture injects synthetic
measurement offsets (not physically consistent actual raw8 dynamics) solely
to exercise feedback. These tests do not prove frictional retention, safe
swept-mesh motion, loaded tracking, release rebound, contact forces, native
goal completion or score. No physical run of this candidate has been made.
