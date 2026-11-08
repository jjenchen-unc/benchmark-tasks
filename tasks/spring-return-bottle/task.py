"""Official YAM task-file entry point."""
from pathlib import Path
import runpy
_definition=runpy.run_path(str(Path(__file__).resolve().parent/'yam/task.py'))
for _name in ('SCENE','OBJECTS','SUCCESS_HOLD_S','SCORE_OVER','success','score','demo'):
    globals()[_name]=_definition[_name]
