#!/usr/bin/env bash
# Evidence (or development) run on a frozen source snapshot.
#   run_frozen.sh <run-name> <command run inside the snapshot's isaac/ dir, as one string>
# Copies the task folder's {isaac,src,task.json} into $OUT/<run>/source (read-only afterwards), writes
# source-sha256.txt and source-combined-sha256.txt, then runs the command in the Isaac container with the snapshot
# mounted at /tb and the run directory at /out/tb/<run>. Log: $OUT/<run>/run.log.
# Environment (host-specific):
#   RC_PODMAN       container launcher (Isaac Sim 6.1 + Isaac Lab-Arena podman wrapper; mounts its output root at /out)
#   RC_SLOT_CMD     optional GPU slot allocator prefix (sets RC_GPU); else RC_GPU (default 1) is used
#   ARENA_VENV_BIN  bin/ directory of the Isaac Lab-Arena Python venv as seen inside the container
#   TB_ISAAC_OUT    host directory that the launcher's /out maps to plus the sub path (default: <task folder>/runs)
#   TB_OUT_IN       the same directory as seen inside the container (default /out/tb)
#   TB_ENV          extra "-e K=V" podman args (e.g. "-e TB_PROBE=taut")
set -euo pipefail
run=$1; shift
cmd=$1
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
OUT=${TB_ISAAC_OUT:-$here/runs}
OUT_IN=${TB_OUT_IN:-/out/tb}
E=${RC_PODMAN:?set RC_PODMAN to the Isaac container launcher script}
V=${ARENA_VENV_BIN:?set ARENA_VENV_BIN to the Arena venv bin directory}
dir=$OUT/$run
[[ -e $dir ]] && { echo "exists: $dir" >&2; exit 1; }
mkdir -p "$dir/source"
cp -r "$here/isaac" "$here/src" "$here/task.json" "$dir/source/"
find "$dir/source" -name __pycache__ -prune -exec rm -rf {} +
rm -rf "$dir/source/isaac/runs"
(cd "$dir/source" && find . -type f | LC_ALL=C sort | xargs sha256sum) > "$dir/source-sha256.txt"
sha256sum "$dir/source-sha256.txt" | cut -d' ' -f1 > "$dir/source-combined-sha256.txt"
chmod -R a-w "$dir/source"
echo "$cmd" > "$dir/command.txt"
# shellcheck disable=SC2086
timeout "${TB_TIMEOUT:-7200}" ${RC_SLOT_CMD:-env RC_GPU=${RC_GPU:-1}} bash "$E" --gpu -- -v "$dir/source:/tb:ro" \
  -e PATH="$V:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" -e TMPDIR=$OUT_IN/$run/tmp ${TB_ENV:-} \
  -- bash -c "mkdir -p $OUT_IN/$run/tmp && cd /tb/isaac && $cmd" > "$dir/run.log" 2>&1
