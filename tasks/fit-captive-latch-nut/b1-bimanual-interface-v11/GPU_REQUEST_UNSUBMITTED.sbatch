#!/bin/bash
#SBATCH --partition=a6000
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=4
#SBATCH --mem=32G
#SBATCH --time=00:30:00
#SBATCH --chdir=/mnt/bum/jjenchen/robocurve
#SBATCH --output=/mnt/bum/jjenchen/robocurve/artifacts/submission-readiness/b1-fullportable-v8-20261110-1440/slurm-%j.log
set -euo pipefail
# UNSUBMITTED. ROOT dispatch only. Whole-run walltime is unmeasured.
# Prefer tomorrow's user-opened Brev full affected replay; no UNC run is required.
export APPTAINERENV_B1B2_LEGACY_RTX=1
export APPTAINERENV_OMP_NUM_THREADS=1 APPTAINERENV_OPENBLAS_NUM_THREADS=1 APPTAINERENV_MKL_NUM_THREADS=1
# Process-only override; no host driver/config change.
timeout --signal=TERM --kill-after=20 1740 containers/unc/run.sh --gpu /opt/IsaacLab-Arena/.venv/bin/python -B -I /mnt/bum/jjenchen/robocurve/artifacts/submission-readiness/b1-fullportable-v8-20261110-1440/run_phase.py replay --out /mnt/bum/jjenchen/robocurve/artifacts/submission-readiness/b1-fullportable-v8-20261110-1440/work/actual-root-dispatched
