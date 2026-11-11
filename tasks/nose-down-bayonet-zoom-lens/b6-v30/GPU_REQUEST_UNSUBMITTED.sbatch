#!/bin/bash
#SBATCH --job-name=b6-portable-v16
#SBATCH --partition=a6000
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=4
#SBATCH --mem=32G
#SBATCH --time=00:30:00
#SBATCH --chdir=/mnt/bum/jjenchen/robocurve
#SBATCH --output=/mnt/bum/jjenchen/robocurve/artifacts/submission-readiness/b6-fullportable-v16-20261110-1538/slurm-%j.log
set -euo pipefail
: "${SLURM_JOB_ID:?ROOT dispatch only}"
: "${CUDA_VISIBLE_DEVICES:?allocation required}"
TASK_REPO=/mnt/bum/jjenchen/robocurve
BUNDLE=$TASK_REPO/artifacts/submission-readiness/b6-fullportable-v16-20261110-1538
timeout --signal=TERM --kill-after=20s 1680s "$TASK_REPO/containers/unc/run.sh" --gpu env OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 /opt/IsaacLab-Arena/.venv/bin/python -I -B "$BUNDLE/runtime.py" reference --rtx-driver-check-override --out "$BUNDLE/gpu-$SLURM_JOB_ID-reference"
