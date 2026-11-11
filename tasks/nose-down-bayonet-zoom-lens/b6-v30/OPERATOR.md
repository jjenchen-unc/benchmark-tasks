B6 v16 private portable inputs. ROOT alone reviews, dispatches and promotes. Brev remains OFF; no model execution is authorized by this file. No request has been submitted.

Extract `b6-v16.tar.gz` anywhere. The task needs no original source path; installed SDK and YAM robot assets remain external dependencies. `candidate/` is byte-exact v14, including its embedded manifest, all goals/variants provenance, assets and controller. Runtime wrappers bootstrap their own directories under `python -I`. Exact setup is `candidate/setup-inventory.json` and `fixture.json`: task rigid translation [0.25,0,-0.0525] m; station bases [0,+/-0.305,0.90] m, 0.20 x 0.20 x 0.1525 m pedestals. No geometry or physical parameters change.

Commands below use the SDK Python available inside the existing container. Set BUNDLE to the extracted bundle absolute path and PY to that Python. Use fresh output paths outside `candidate`. On SERVER, ROOT uses the supplied UNSUBMITTED request with canonical chdir, GPU1/CPU4/RAM32GB/30min, existing `containers/unc/run.sh --gpu`, process-only driver override and OMP/BLAS threads1. ROOT reviewed actual43967 PASS for first-lift plus10 retention controls only (297 controls/7425 samples,230.362s wall); whole goal remains unverified.

```bash
"$PY" -I -B "$BUNDLE/runtime.py" reference --out "$BUNDLE/trial-reference" --rtx-driver-check-override
"$PY" -I -B "$BUNDLE/runtime.py" replay --actions "$BUNDLE/trial-reference/actions.json" --out "$BUNDLE/trial-replay" --rtx-driver-check-override
"$PY" -I -B "$BUNDLE/runtime.py" setup --out "$BUNDLE/trial-setup" --rtx-driver-check-override
```

Reference defaults to the complete genuine controller; no first-lift cutoff is selected. `candidate/run.py --case first-lift` remains the unchanged explicit diagnostic only. Replay consumes 14D joint targets from the fresh reference without object/score/timer writes. Wrapper captures actual three observation cameras at 20Hz H.264, one RGB setup PNG at reset, every action/status, strict numeric seed contract and the native 500Hz trace. Encoder completion/size are checked by `protocol.fresh`; final media still needs visual review and full decode/ffprobe before public use. Setup mode advances no controls after normal reset/settle.

Conditional whole bound 2423/3034 controls (121.15/151.7 sim s), including native continuous 20s release. No measured whole-route walltime exists. Estimate reference/replay 1100–2300 s EACH from component actual43897 257.66 s for401 controls plus cold startup; 30-minute request may timeout at1680s. This is an honest planning range, not proof of full throughput. No repeated automatic dispatch or restart. Setup estimate30–120s. Physical timeouts/failures retain all evidence and reopen work.

Public score remains zero in reference/replay. ROOT must independently review a fresh complete genuine reference AND action replay, native20s hold and all nine channels. Historical43897 is evaluator placement/load component evidence only; old43955 failed. CPU conditional proof and actual43967 first-lift PASS establish neither whole success nor channel approval.

`protocol.receipt(path)` accepts only schema `b6-v16-root-physical-review`, reviewer `ROOT`, approved true, exact current `manifest`, relative `reference` and `replay` evidence folders beside the receipt, matching `reference_summary`/`replay_summary` from `fresh`, complete `{relative filename: sha256}` `reference_sha256`/`replay_sha256`, identical reference/replay actions, mounted_support_approved true, and exactly the nine CHANNELS. Each channel requires approved true, nonempty rationale and proofs [{path: relative-to-receipt-folder, sha256: ...}]. It replays native History against every .002s physics row, checks every control/journal/action and source before/after, native full goal and20s hold. No approval receipt is included. Root component receipts copied under provenance are historical source-bound records, never accepted as this receipt.

Only after ROOT supplies that physical review and separately authorizes final model validation on the user-opened official runtime:

```bash
"$PY" -I -B "$BUNDLE/model_child.py" --review /absolute/ROOT-review.json --out "$BUNDLE/final-model"
```

This actual Inspect CLI entry uses exact task.draft instruction, Dream-1 beta, rows_per_request4, 3034 controls,20Hz, CPU PhysX,320x176 unchanged policy cameras, seed42 with harness-derived reset seed passed unchanged. One episode, one continuous session, attempts1; no retry/restart. Runtime installed source pins are checked before simulator/provider. Model task opens public numeric scorer through reviewed History channels only; native VALIDATED remains empty and no task-state/timer/geometry writes occur. Model/media and official runtime remain unverified.

CPU proof commands use persistent `pre_sdk.py` with `-I -B`: reference/replay/setup, model-missing, model-synthetic, escape-execv/execve/Popen/raw/cleanup. model-synthetic substitutes receipt only in memory for API construction; it never creates an approval file or imports simulator/provider. No guard sys.path injection; actual launchers bootstrap themselves. Actual task loads twice before blocked isaaclab.app. See checks for measured CPU times and consumed-source maps. Preserve those proofs; no need to repeat unchanged scans for metadata.

V16 repair: strict typed physics-row validation now runs in protocol.fresh before native History.update. Native candidate38 files and embedded manifest remain byte-exact; no physics/controller changes. test_fresh_boundary.py feeds actual43967 rows and malformed mutants through fresh itself. These CPU diagnostics do not establish physical failure or success. Existing unchanged escape/API/geometry/source proofs are retained.
