#!/usr/bin/env bash
# Evidence run on a frozen source snapshot.
#   run_frozen.sh <run-name> "<command run inside the snapshot's isaac/ dir>"
# Copies the task folder's {isaac,tests,task.json} into $OUT/<run>/source (read-only afterwards), writes
# source-sha256.txt (+ combined hash), then runs the command in the Isaac container (shared GPU slot via rc_slot.sh)
# with the snapshot mounted at /task and the run directory at $OUT_IN/<run>. Log: $OUT/<run>/run.log.
# Environment (host-specific):
#   RC_ENV_DIR      directory with rc_slot.sh and rc_podman.sh (Isaac Sim 6.1 + Isaac Lab-Arena podman wrapper that
#                   mounts its own output root at /out)
#   ARENA_VENV_BIN  bin/ directory of the Isaac Lab-Arena Python venv as seen inside the container
#   RECYCLE_OUT     host directory for runs; RECYCLE_OUT_IN = the same directory as seen in the container
set -uo pipefail
run=$1; cmd=$2
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
E=${RC_ENV_DIR:?set RC_ENV_DIR}
V=${ARENA_VENV_BIN:?set ARENA_VENV_BIN}
OUT=${RECYCLE_OUT:?set RECYCLE_OUT}
OUT_IN=${RECYCLE_OUT_IN:?set RECYCLE_OUT_IN}
dir=$OUT/$run
[[ -e $dir ]] && { echo "exists: $dir" >&2; exit 1; }
mkdir -p "$dir/source"
cp -r "$here/isaac" "$here/tests" "$here/task.json" "$dir/source/"
find "$dir/source" -name __pycache__ -prune -exec rm -rf {} +
(cd "$dir/source" && find . -type f | LC_ALL=C sort | xargs sha256sum) > "$dir/source-sha256.txt"
sha256sum "$dir/source-sha256.txt" | cut -d' ' -f1 > "$dir/source-combined-sha256.txt"
chmod -R a-w "$dir/source"
echo "$cmd" > "$dir/command.txt"
for attempt in 1 2 3 4; do
  RC_SLOTS=${RC_SLOTS:-10} timeout "${RECYCLE_TIMEOUT:-7200}" bash "$E/rc_slot.sh" bash "$E/rc_podman.sh" --gpu -- \
    -v "$dir/source:/task:ro" -e PATH="$V:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
    -e TMPDIR="$OUT_IN/tmp" -- bash -c "mkdir -p $OUT_IN/tmp && cd /task/isaac && $cmd" > "$dir/run.log" 2>&1
  code=$?
  # Kit start-up crash (before the environment exists) when several containers start at once: retry
  if [[ $code -ne 0 ]] && ! grep -q "Completed setting up the environment" "$dir/run.log" \
      && grep -q "Crash detected in pid" "$dir/run.log"; then
    mv "$dir/run.log" "$dir/run.log.startup-crash$attempt"; sleep $((20 + RANDOM % 40)); continue
  fi
  echo "$code" > "$dir/exit-code.txt"; exit $code
done
exit 1
