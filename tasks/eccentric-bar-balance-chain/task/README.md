# B4 offset stack: calibrated portable candidate

Fixed public physical case: native seed42, setting S, stack_ABC. Inspect's CLI
seed42 is an episode seed; Inspect may derive another reset seed. That value
passes unchanged to SDK reset and does not change fixed physical parameters.
The exact instruction is in task.draft.json and task/task.py; replay and model
use identical text. Other native goals/settings/seeds remain in native-provenance.

The source-bound receipt enables seven measurement channels only on the pinned
runtime and matching immutable source. Missing or mismatched receipt stays gated.
Actual43744 remains historical public score0; no new public score is claimed.
A future actual saved-action replay must establish its own public score.
Actual43765 supplies paired signed10N wrist readings, raw/processed bar pose and
COM-corrected actor velocity, contact buffer ranges/coverage and120N abort.
Nonfinger load uses a conservative sum(abs(normal))+sum(norm(friction)) upper
bound, not exact native per-contact resultant equivalence. Full12x30 coverage
and the same positive bar/table Contacts reader support the selected historical
zero-nonfinger-contact run; no hardware/FEM/all-branch calibration is claimed.

Use the existing pinned Python3.12.16 YAM/Arena stack. Installed roots are
resolved dynamically from package specs; versions, checkout heads and consumed
runtime source hashes remain bound. No install or historical workspace required.
Device cpu physics; GPU cameras;20Hz/25x2ms substeps;1896control limit (94.8s).
Saved actual1441 controls cover72.05physical seconds; walltime depends on GPU.
From this folder with a fresh external absolute output directory:

    python -I -B package_checks.py
    python -I -B task/probe.py --mode replay --out /absolute/new-replay
    python -I -B task/probe.py --mode setup --out /absolute/new-setup
    python -I -B run_model.py --preflight-only --out /absolute/new-model
    python -I -B run_model.py --out /absolute/new-model

Set B4_LEGACY_RTX_OVERRIDE=1 only for the legacy driver process; it only passes
Kit's driver-check flag. No host/image change. ROOT dispatches actual work under
its existing quota/deadline guard. Never run after the existing08:11:32UTC Brev
hard deadline; no extension or new service allocation is authorized here.

Replay consumes actual saved14D controls only, never task pose/state writes;
it fails on missing channels, measurement error, early termination, clock mismatch
or terminal score!=1. It encodes three actual observation videos H264/yuv420p,
20fps, <=50MB each. Setup uses render-only warmup and checks body/joint states,
physics step count, episode samples and controls stay unchanged. Dream helper
uses actual Inspect/Dream1, four rows/request, zero retry/fallback/session reset,
exact1896budget/instruction, and verifies source, score, continuous session,
record and frames. All helpers are inside this task folder.

Calibration/evaluator modes are diagnostics only, separate from replay/reference.
No task geometry, law, mass, friction, goal, numeric threshold, scorer, reference,
motor/finger ceiling or budget changed. Genuine physical implementation files
are byte-identical to43744. Contract math is AST-identical except receipt lookup.
Native snapshots, source notices, licenses, provenance, actual1441actions and
CPU analysis receipts are bundled. See task/evidence and manifests.

Submission readiness is FALSE until new actual replay, genuine model evidence,
blind media, metadata and current ownership/upload/PR conditions are satisfied.
CPU checks are conditional/static/synthetic; historical actual scores do not
become a new replay or submission-ready result.
