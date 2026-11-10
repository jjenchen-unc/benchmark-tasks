#!/usr/bin/env bash
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec containers/unc/run.sh --gpu /opt/IsaacLab-Arena/.venv/bin/python -I -B "$here/driver.py" "$@"
