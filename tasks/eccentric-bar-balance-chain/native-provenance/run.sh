#!/usr/bin/env bash
set -euo pipefail
candidate_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$candidate_dir"
python_bin="${ROBOCURVE_NATIVE_PYTHON:-$repo_root/.venv/bin/python}"
if [[ ! -x "$python_bin" && -z "${ROBOCURVE_NATIVE_PYTHON:-}" ]]; then
  python_bin="$(command -v python3)"
fi
if [[ ! -x "$python_bin" ]]; then
  printf 'Native Python is not executable: %s\nSet ROBOCURVE_NATIVE_PYTHON to the prepared native environment Python.\n' "$python_bin" >&2
  exit 2
fi
exec "$python_bin" - "$candidate_dir/candidate.json" "$repo_root" "$@" <<'PYCODE'
import argparse
import json
import os
from pathlib import Path
import sys

metadata_path, root_arg, *argv = sys.argv[1:]
metadata = json.loads(Path(metadata_path).read_text())
root = Path(root_arg)
p = argparse.ArgumentParser(description=metadata['candidate_id'] + ': development wrapper; no submission or model calls')
p.add_argument('command', nargs='?', default='build', choices=['build', 'reset', 'step', 'reference', 'render'])
p.add_argument('--target', help='Goal ID in this candidate family')
p.add_argument('--setting', choices=['A', 'S', 'T-new', 'T-combo'])
p.add_argument('--list-variants', action='store_true', help='Print variants without importing the simulator')
p.add_argument('--dry-run', action='store_true', help='Print resolved command without running the simulator')
args, extra = p.parse_known_args(argv)
if args.list_variants:
    print(json.dumps(metadata['variants'], indent=2))
    raise SystemExit(0)
for flag in extra:
    if flag in ('--module', '--root') or flag.startswith(('--module=', '--root=')):
        p.error('Module and source root are fixed by candidate.json')
variants = {v['goal_id']: v for v in metadata['variants']}
target = args.target or metadata['default_representative']['goal_id']
if target not in variants:
    p.error('Target is outside this candidate family: ' + target)
setting = args.setting or (metadata['default_representative']['setting'] if args.target is None else variants[target]['settings'][0])
if setting not in variants[target]['settings']:
    p.error('Unsupported goal/setting pair: ' + target + ' / ' + setting)
module = root / metadata['source_module']
if not module.is_file():
    p.error('Missing common bench snapshot: ' + str(module))
command = [sys.executable, '-m', 'robocurve_native', args.command, '--module', str(module), '--target', target, '--setting', setting, *extra]
if args.dry_run:
    print(json.dumps({'candidate_id': metadata['candidate_id'], 'command': command, 'submission_exclusion': variants[target]['submission_exclusion'], 'development_only': True}, indent=2))
    raise SystemExit(0)
os.environ['PYTHONPATH'] = str(root / 'runtime')
os.environ.setdefault('MUJOCO_GL', 'osmesa')
if variants[target]['submission_exclusion']:
    print('Submission-blocked variant remains available for local source testing; this command cannot submit.', file=sys.stderr)
os.execv(sys.executable, command)
PYCODE
