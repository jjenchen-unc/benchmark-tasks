# Raise a loaded two-leg instrument stand by two holes

Raise a loaded stand from hole 1 to hole 3 on both legs, re-engage both pull pins, level it within 3 mm, and leave it untouched for one second. The left leg has a safety pawl; the right has gas assist. The robot must support the payload while releasing and reindexing each pin.

## Actual evaluation

| Run | Native score | Controls | Evidence |
| --- | --- | --- | --- |
| Scripted demonstration | 1.0 | 2,694 | Every one of 67,350 native samples independently regraded; wrist peaks 53.10/58.53 N, zero samples at or above 60 N; hands off 1.048 s. |
| Dream-1 beta | 0.0 | 4,572 | Installed Inspect policy, 1,135 completed requests in one continuous session, zero errored trials; time-limit termination. |

The Dream policy is `dreamlabs/dream-1-beta`. Its SDK setting `rows_per_request=4` executes four rows from each returned 48-row prediction before requesting again. The original three policy images are 320x176 RGB, with joint-position observations. The submitted videos use an additional 640x360 recording camera; it is excluded from policy inputs. The normal SDK startup behavior remains part of the recorded control budget.

The state-aware demonstration reads simulator state for planning and uses ordinary 14D joint-position/gripper Actions to move the robot. The controller does not place objects, apply evaluator forces, or rewrite score/history state. Component load probes are separate diagnostics and contribute no demonstration score.

## Reproduce

Use the pinned YAM/Isaac Lab environment described in `source/dependencies.json`. The runtime checks 369 installed source hashes relative to package roots, the frozen task inventory and the source-bound eight-channel review before enabling scoring. It requires the same inspected implementation, including Isaac Sim 6.1. Configure Dream credentials through the installed SDK for the model run.

```bash
python run.py prepare
python run.py demo --out /tmp/instrument-stand-demo --seed 19358472
python run.py model --out /tmp/instrument-stand-model --seed 42
```

Runtime per trial is at most 228.6 simulated seconds (134.7 seconds for the successful demo). On the measured L40S host, wall times including initialization and recording were 983.9 seconds for the demo and 1,397.3 seconds for Dream; provider latency and rendering affect wall time.

Each output directory must be fresh and outside the task folder. `prepare` and `setup` run CPU preflight; an actual rendered setup image is produced by the demo/model entry. The model CLI derives its trial seed from seed 42; that derived seed passes unchanged to reset. The recorded model trial seed is 3867854786. Physical layout remains source case 42 with zero jitter.

The demo preserves the frozen reference and accepts only its actual cached success Result, with native/public score 1. Failures and unrelated exceptions propagate. The model uses one genuine SDK session; expiry, replacement and restart are invalid runs. A source-identical task-local adapter saturates only public gripper commands to [0,1], preserving arm columns and recording the conversion.

## Geometry, limits and score

`task.json` gives the complete public instruction, setup and numeric score. `source/fixture.json`, `source/joints.json`, `source/stand.usda` and `source/kit.py` define the executed scene. Arm bases are elevated on fixed supports. The source-native Panda limits were 80/12 Nm; this port uses YAM's lower 27/7 Nm total ceilings, including gravity, and 40 N finger ceilings. Actual successful execution establishes feasibility under these limits.

Full credit requires both slides at 75+/-1.5 mm, both pins within 3 mm of seated, height difference <=3 mm and no robot/stand contact for one continuous second. Half credit uses 50 mm and does not end the trial. The native 2 ms history enforces the jolt, downward-speed and 80 N/0.1 s aborts. The instruction's 60 N operating bound was independently checked over the entire full-score demonstration. Maximum filtered instrument acceleration was 0.75853 g and maximum downward leg speed 0.07214 m/s.

Rigid primitives, hidden square hole strips, conditional passive stops, a constant gas-assist approximation, a serial articulation and two excluded-loop spherical joints approximate the original mechanism. Acceleration is a kinematic COM estimate with a 5 ms filter. The additional numeric frame masses are 1e-6 kg each. These approximations and all reset values are explicit in the source.

The frozen `source/task.draft.json` and source README retain development provenance. The public `task.json`, this README and the attached actual runs describe the submitted result. Scoring approval changes only the original runtime credit gate after matching source/evidence checks; it preserves the physical equations and predicates.

## Mechanism comparison

The current board comparison includes TK-VPGY3G (one mis-indexed shelf support), TK-VGVX32 (a single extending handle), TK-CENFC8 and TK-ZYVUFX (rack rails), and TK-AKTDPZ (a hinged bracket). This task coordinates two load-bearing telescoping supports, unequal pawl/gas responses, hold-out pin locks and a delicate payload's jolt/drop constraints. `source/source-goals.json` preserves the source goals; the submitted representative is QP-T1/S, source case 42.

Public runtime files, compact source-bound reviews and the actual uploaded media are included or named by `task.json`. Full raw simulation journals are retained separately and are not required by the runtime.
