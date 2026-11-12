'Take the red-tab drawer completely out of the chest and set it level on the table. Keep its sealed box inside and both jars intact. The target jar may stand upright on the table outside cabinets. Release the drawer for one second.'
from pathlib import Path
import sys,math
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from drawers_kit import DrawerAsset,ASSETS,install,current_score
from drawers_math import world
install()
SCENE='tabletop'
OBJECTS=[DrawerAsset(name=a['name'],pos=tuple(world(a['native_pos'])),yaw=math.pi,mass=None if a['fixed'] else a['mass'],size=tuple(a['size'])) for a in ASSETS]
SCORE_OVER='end'
def score(objects):return current_score()
