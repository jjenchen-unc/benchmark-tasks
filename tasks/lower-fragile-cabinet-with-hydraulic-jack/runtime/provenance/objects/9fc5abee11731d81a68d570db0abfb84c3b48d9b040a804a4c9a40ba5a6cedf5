Canonical cwd: /mnt/bum/jjenchen/robocurve. Brev stays OFF.

Frozen input verification, host Python (no SDK):

    python3 artifacts/submission-readiness/hydraulic-supported-yam-whole-v4-20261110-0920/verify_inputs.py

CPU conditional controller/geometry commands, existing container, no simulator:

    containers/unc/run.sh env OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 /opt/IsaacLab-Arena/.venv/bin/python -I -B artifacts/submission-readiness/hydraulic-supported-yam-whole-v4-20261110-0920/controller_contract.py
    containers/unc/run.sh env OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 /opt/IsaacLab-Arena/.venv/bin/python -I -B artifacts/submission-readiness/hydraulic-supported-yam-whole-v4-20261110-0920/command_route.py
    containers/unc/run.sh env OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 /opt/IsaacLab-Arena/.venv/bin/python -I -B artifacts/submission-readiness/hydraulic-supported-yam-whole-v4-20261110-0920/self_audit.py

Do not rerun unchanged diagnostics; retain their outputs and consumed map. full_route.py describes the rejected lower grip and is historical failure evidence.

Actual GPU request is GPU_REQUEST.sh. It is unsubmitted and disabled pending ROOT's full-path review. After ROOT review only, exact request:

    sbatch --export=ALL,ROOT_REVIEWED_HYDRAULIC_V4=1 artifacts/submission-readiness/hydraulic-supported-yam-whole-v4-20261110-0920/GPU_REQUEST.sh

Do not execute that command from this unit. No new allocation or Brev start is authorized here. Source channels remain disabled; fresh affected physical evidence, plus later model/media/submission work, cannot be replaced by CPU results.
