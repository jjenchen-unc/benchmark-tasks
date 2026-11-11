"""Open box 1's lid past 60 degrees using its swing handle; leave all parts of that
box untouched by both robot arms for 20 seconds. Press the red cap to release the
lever, lift the lever then turn to retract the multipoint lock. Three clamped
steel enclosures lie lid-up on the table. Development port: not submission-ready.
"""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from s1_kit import Enclosures,install,current_score,FIXTURE
install()
SCENE='station'
OBJECTS=[Enclosures(name='enclosures',pos=tuple(FIXTURE['translation']),size=(.2,.7,.4))]
SCORE_OVER='end'
def score(objects):return current_score()
