#!/usr/bin/env bash
#SBATCH --job-name=S1-portable-v6-ROOT-only
#SBATCH --partition=a6000
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=4
#SBATCH --mem=32G
#SBATCH --time=00:30:00
#SBATCH --chdir=/mnt/bum/jjenchen/robocurve
#SBATCH --output=/mnt/bum/jjenchen/robocurve/artifacts/submission-readiness/s1-fullportable-v6-20261110-1457/gpu-%j.log
set -euo pipefail
# UNSUBMITTED; do not duplicate pending43823. ROOT decides dispatch.
root=/mnt/bum/jjenchen/robocurve
pkg="$root/artifacts/submission-readiness/s1-fullportable-v6-20261110-1457"
exec timeout --signal=TERM --kill-after=15s 1650 \
 "$root/containers/unc/run.sh" --gpu env \
 OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1 \
 PYTHONDONTWRITEBYTECODE=1 S1_LEGACY_RTX=1 \
 /opt/IsaacLab-Arena/.venv/bin/python -I -B "$pkg/run_phase.py" replay \
 --out "$pkg/work/root-gpu-${SLURM_JOB_ID}"
