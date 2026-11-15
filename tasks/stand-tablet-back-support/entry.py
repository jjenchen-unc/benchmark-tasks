"""Process-local driver compatibility switch for UNC diagnostic runs."""
import functools,runpy,sys
from isaaclab.app import AppLauncher
previous=AppLauncher.__init__
@functools.wraps(previous)
def launch(self,*a,**k):
 k['kit_args']=(k.get('kit_args','')+' --/rtx/verifyDriverVersion/enabled=false').strip()
 return previous(self,*a,**k)
AppLauncher.__init__=launch
script=sys.argv[1];sys.argv=sys.argv[1:];runpy.run_path(script,run_name='__main__')
