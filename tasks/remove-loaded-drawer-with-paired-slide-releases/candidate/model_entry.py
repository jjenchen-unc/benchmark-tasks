'Take the red-tab drawer completely out of the chest and set it level on the table. Keep its sealed box inside and both jars intact. The target jar may stand upright on the table outside cabinets. Release the drawer for one second.'
from pathlib import Path
import os,sys,runpy
HERE=Path(__file__).resolve().parent
# Bootstrap is owned by this actual isolated task entry.
sys.path.insert(0,str(HERE))
_native=runpy.run_path(str(HERE/'task.py'))
for _name in ('SCENE','OBJECTS','SCORE_OVER','score'):
 globals()[_name]=_native[_name]
from model_credit import configure_model
CREDIT_BINDING=configure_model(HERE,os.environ.get('DRAWERS_CREDIT_REVIEW'))
