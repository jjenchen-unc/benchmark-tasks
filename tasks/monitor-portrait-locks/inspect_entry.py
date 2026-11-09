"""Official Inspect CLI with an explicit process-only legacy RTX driver override."""
import functools
import sys


def main():
    args=sys.argv[1:]
    if '--driver-check-override' in args:
        args.remove('--driver-check-override')
        from isaaclab.app import AppLauncher
        original=AppLauncher.__init__
        @functools.wraps(original)
        def init(self,*positional,**kwargs):
            kwargs['kit_args']=(kwargs.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip()
            return original(self,*positional,**kwargs)
        AppLauncher.__init__=init
        print('UNC DEVELOPMENT ONLY: RTX driver-version check disabled for this process; host driver unchanged.',flush=True)
    sys.argv=['inspect-robots',*args]
    from inspect_robots.cli import main as inspect_main
    return inspect_main()


if __name__=='__main__':main()
