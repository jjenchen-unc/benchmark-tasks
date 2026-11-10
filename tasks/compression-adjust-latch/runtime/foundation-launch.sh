#!/usr/bin/env bash
# Execute within ROOT's existing allocation. Does not submit/cancel any job.
set -euo pipefail
: "${SLURM_JOB_ID:?Root must dispatch within its allocated job}"
: "${CUDA_VISIBLE_DEVICES:?Allocated GPU visibility is required}"
if [[ $# -ne 1 ]]; then echo 'Usage: foundation-launch.sh NEW_OUTPUT_DIRECTORY' >&2; exit 2; fi
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)
cd -- "$root"
# 28 min plus30s termination cleanup fits the root30min allocation ceiling.
exec timeout --signal=TERM --kill-after=30s 28m \
  "$root/containers/unc/run.sh" --gpu /usr/bin/env OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 python \
  "$root/ports/arena-submission/s4-compression-latch/driver.py" \
  --mode foundation --driver-check-override --out "$1"
