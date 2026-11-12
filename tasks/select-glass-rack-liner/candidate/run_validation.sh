#!/usr/bin/env bash
# Root calls this INSIDE an allocated GPU job; it does not submit/cancel Slurm.
set -euo pipefail
: "${SLURM_JOB_ID:?root must supply a Slurm allocation}"
: "${CUDA_VISIBLE_DEVICES:?root must supply allocated GPU visibility}"
port=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
root=$(cd -- "$port/../../.." && pwd)
suite=${1:?foundation|friction|wet|guard|reset|reference|wet-reference}
out=${2:?new output directory}
# Uses the pinned runtime; process-only RTX check override in frozen bootstrap.
cd -- "$root"
"$root/containers/unc/run.sh" --gpu /usr/bin/env OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 /opt/IsaacLab-Arena/.venv/bin/python -B "$port/validation_entry.py" --suite "$suite" --out "$out"

python3 - "$out" "$suite" <<'PY'
import json,sys
from pathlib import Path
p=Path(sys.argv[1]);r=json.loads((p/"result.json").read_text())
assert not(p/"error.json").exists(),"runtime error artifact"
if sys.argv[2]=="foundation":assert r.get("all_checks_passed") is True,"foundation checks incomplete or failed"
assert (p/"source-sha256.json").is_file()
if sys.argv[2]=="foundation":assert (p/"checkpoints.jsonl").is_file()
PY
