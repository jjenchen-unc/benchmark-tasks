# Fit a wrench, release a captive latch nut and open its arm

Choose a fitting tool for an unmarked captive fastener, release the clamp without rounding the nut, open the arm, and leave the mechanism untouched for 20 seconds. This is the fixed W1/S physical case, seed42. Only this goal is submitted.

Actual Brev results on the shipped v11 source are a genuine14D teacher score **1.0** (2069 controls,51,725 native2ms samples,20.048s released full hold) and a Dream-1 score **0.0** (3870 controls,96,750 native samples,960 requests in one continuous session). Both completed without evaluation/runtime error. Every measured sample and both-arm raw-joint/FK/force observation was independently checked. The model never contacted a tool and did not release the fixture. The earlier model attempt with a journal callback attribute error is invalid and contributes no score; the external callback was repaired before this valid trial.

## Reproduce

Use the pinned IsaacLab/YAM environment. Source its `yam.env`, provide Dream credentials outside the task directory, and use its Python interpreter. The wrapper downloads the immutable physical review release asset into a cache outside this source tree. It checks archive size/SHA256, every safe member path/type/size/SHA256, the submitted source map, and the actual ROOT approval receipt before SDK initialization. `--archive` accepts the exact offline asset instead. Raw observations and calibration media are in the release asset, outside Git.

```sh
python -I -B run.py prepare --cache /absolute/external/b1-review-cache
python -I -B run.py setup --cache /absolute/external/b1-review-cache --out /absolute/fresh/b1-setup
python -I -B run.py demo --cache /absolute/external/b1-review-cache --out /absolute/fresh/b1-demo
python -I -B run.py model --cache /absolute/external/b1-review-cache --out /absolute/fresh/b1-model
python -I -B run.py verify-model --cache /absolute/external/b1-review-cache --out /absolute/fresh/b1-model
```

Each run uses a fresh output directory; `verify-model` checks the existing completed output. Commands contain no multiline OS argument. The unchanged helper reconstructs the exact instruction inside Python. Model execution uses official Dream-1 at20Hz/four rows per request and the3870-control limit. Only official transient transport retries within the same session are allowed. Session expiry, restart, missing/changed session identity, scorer or embodiment errors invalidate the run.

## Source and evidence

`b1-bimanual-interface-v11` is the exact frozen core used on Brev, fingerprint `0d5b3276c7873feb72cea378e91eb0c166ebce7d32ec24a722fad1cbc41e072d`. Its draft/internal README preserve earlier development-state provenance; current public results and reproduction instructions are in this top-level README and task metadata. `envelopes/credited_demo.py` and `envelopes-v2/model_entry.py` are byte-identical to the actual scored entrypoints. `published-source-sha256.json` binds all78 source files; the external review inventory binds all33 evidence files. No calibration video or raw observation is committed.

The source evolves from a genuine uncredited v9 replay whose public score stayed0. A verifier-only change accepts exactly the unscored zero-sample reset sentinel; all scored rows remain strict. The v11 interface removes a port-added right-target parked-pose veto because the original native both-arm step and scorer support both arms. All native physics, scoring, controller, finite14D checks, joint limits, total motor/force/contact guards and3870 budget remain. A genuine20-action prefix demonstrated right-joint motion/return and actual27/7Nm,40N caps before the final demo/model. Historical evidence qualifies channels through exact source/nonbinding-guard lineage; the fresh public1 demo was rerun on the final entrypoint.

## Physical specification and limits

`b1-bimanual-interface-v11/candidate/fixture.json` and `b1-bimanual-interface-v11/candidate/physical-inventory.json` specify the complete primitive inventory, material estimates, masses, collision exclusions and transformed poses. Five free steel tools offer12mm,1/2in,13mm,14mm and adjustable openings. The unmarked nut is12.70mm across flats; source geometry yaw180deg/translation(0.40,0.10,-0.0525)m is retained. Two YAM arms keep their unchanged SDK mounts, with actual measured bases world(0,+0.305,0.75)m and(0,-0.305,0.75)m; no table/camera-stand or extra support change. The teacher uses the left arm and starts the right parked; the model interface accepts both genuine seven-channel arm/gripper halves without masking or projection.

Physics samples every2ms; genuine robot controls arrive at20Hz. Total motor torques including gravity are27/27/27/7/7/7Nm, with40N per finger. Wrist load>80N for100ms aborts;>30N for20ms, or any nonfinger-link20ms-average load>50N, terminates the port at the reporting boundary. Dropped/out-of-bounds tools and invalid measurements veto credit.

## Scoring, reset and fidelity

Score the current final state. Full1.0 requires nut backoff>=60deg, arm free/open>=85deg and no hand/tool contact on nut/arm continuously20s. Partial0.5 requires the released nut and contact-free20s with no open-angle requirement. Rounding at any time or a safety/measurement veto yields0. The passive clamp frees only after strictly>60deg. Rack return is instructed but is not a separate pose-based scoring condition in the preserved native rubric.

Reset restores all five free tool poses/velocities, mechanism joints/locks, appearance/damage and all timers/counters. Harness seeds are accepted; this declared single physical case is fixed, not randomized. Rigid primitive contacts, a convex hex and passive joint limits approximate mechanics. Nut torque is approximately1/100 service scale; no thread translation or material-deformation mesh is simulated. Damage is an irreversible torque/clearance event, and the adjustable worm is visual. Force sensing/caps were verified in simulation, not on physical hardware. The teacher reads simulator state. The overhead shows the full setup at native320x176, with a small fixture; the left wrist gives closer views with some occlusion, and the parked-right view often shows table.

## Relevance and related tasks

Industrial machinery mechanics repair and maintain equipment ([O*NET49-9041](https://www.onetonline.org/link/details/49-9041.00)). Correct tool fit protects fastener corners; the manufacturer's open-end wrench design discusses corner-rounding prevention ([Snap-on hand-tool catalogue](https://www.snapon.com/display/901/WWW/Downloads/Files/HandTools/Catalogs/Hand_Tools_Part_3.pdf)). This task combines close metric/imperial fit selection, irreversible clearance-sensitive rounding and passive clamp release followed by hands-free proof. Existing Robocurve wrench tasks include pipe-union backup wrench TK-CW7VYS, click-torque TK-NP6A85 and confined ratchet TK-GMVZWK. The distinction is this combined fit/damage/release mechanism; ordinary wrench use alone is not a novelty claim.

The current six-suite primary-catalog comparison and the fresh1493-task Robocurve neighbor assessment are in [importance-review.md](importance-review.md). The nearest wrong-direction ratchet, confined ratchet, click-torque, backup wrench, seized-pulley and compression-latch goals are explicitly compared there.
