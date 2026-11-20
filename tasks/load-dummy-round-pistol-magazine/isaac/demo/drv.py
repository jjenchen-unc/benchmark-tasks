import functools
import runpy
import subprocess
import sys

drv = subprocess.check_output(['nvidia-smi', '--query-gpu=name,driver_version', '--format=csv,noheader'], text=True).strip()
print('GPU', drv, flush=True)
if '--driver-check-override' in sys.argv:
    sys.argv.remove('--driver-check-override')
    from isaaclab.app import AppLauncher
    orig = AppLauncher.__init__

    @functools.wraps(orig)
    def init(self, *a, **k):
        k['kit_args'] = (k.get('kit_args', '') + ' --/rtx/verifyDriverVersion/enabled=false').strip()
        return orig(self, *a, **k)
    AppLauncher.__init__ = init
    print('DEVELOPMENT ONLY: RTX driver-version check disabled for this process', flush=True)
sys.argv = sys.argv[1:]
sys.path.insert(0, str(__import__('pathlib').Path(sys.argv[0]).resolve().parent))
runpy.run_path(sys.argv[0], run_name='__main__')
