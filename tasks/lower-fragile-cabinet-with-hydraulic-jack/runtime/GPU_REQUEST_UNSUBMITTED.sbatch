#!/bin/bash
#SBATCH --job-name=Hydraulic-portable-v5
#SBATCH --partition=a6000
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=4
#SBATCH --mem=32G
#SBATCH --time=00:30:00
#SBATCH --chdir=/mnt/bum/jjenchen/robocurve
#SBATCH --output=/mnt/bum/jjenchen/robocurve/artifacts/submission-readiness/hydraulic-fullportable-v5-20261110-1442/work/GPU-%j.log
set -euo pipefail
# UNSUBMITTED. ROOT review/authorization required. Never duplicate pending43834.
exec timeout --signal=TERM --kill-after=20s 1740s containers/unc/run.sh --gpu env OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 HYDRAULIC_LEGACY_RTX=1 /opt/IsaacLab-Arena/.venv/bin/python -B -I /mnt/bum/jjenchen/robocurve/artifacts/submission-readiness/hydraulic-fullportable-v5-20261110-1442/run_phase.py replay --out /mnt/bum/jjenchen/robocurve/artifacts/submission-readiness/hydraulic-fullportable-v5-20261110-1442/work/GPU-${SLURM_JOB_ID}-trial
