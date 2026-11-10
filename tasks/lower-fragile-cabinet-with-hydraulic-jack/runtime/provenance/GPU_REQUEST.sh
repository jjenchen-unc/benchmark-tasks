#!/bin/bash
#SBATCH --partition=a6000
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=4
#SBATCH --mem=32G
#SBATCH --time=00:30:00
#SBATCH --chdir=/mnt/bum/jjenchen/robocurve
#SBATCH --output=/mnt/bum/jjenchen/robocurve/artifacts/submission-readiness/hydraulic-supported-yam-whole-v4-20261110-0920/slurm-%j.log
# UNSUBMITTED / REVIEW REQUIRED. ROOT ONLY: disabled until full route review passes.
# Exact unsubmitted request. No dispatch authorization is asserted by this file.
set -euo pipefail
# ROOT must review CPU route and fresh affected physical validation plan first.
[[ ${ROOT_REVIEWED_HYDRAULIC_V4:-0} == 1 ]] || exit 78
exec timeout --signal=TERM --kill-after=20s 1680s /mnt/bum/jjenchen/robocurve/containers/unc/run.sh --gpu env OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 /opt/IsaacLab-Arena/.venv/bin/python -I -B /mnt/bum/jjenchen/robocurve/artifacts/submission-readiness/hydraulic-supported-yam-whole-v4-20261110-0920/launch.py /mnt/bum/jjenchen/robocurve/artifacts/submission-readiness/hydraulic-supported-yam-whole-v4-20261110-0920/job-${SLURM_JOB_ID}/trial
