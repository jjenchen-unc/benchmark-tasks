"""A loose tablet and a flat-sided magnetic stylus on a desk, with two YAM arms."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parent))
import runtime
from runtime import MagneticBody
from magnet import IPAD_SIZE,IPAD_MASS,PENCIL_MASS
INSTRUCTION='Attach the stylus by its flat magnetic face to the magnetic side of the tablet. Release the stylus, then lift the entire tablet at least 5 cm above the desk and hold it there for 5 seconds without supporting the stylus with either hand.'
OBJECTS=[MagneticBody(name='ipad',pos=(-.04,.10,.75065),size=tuple(IPAD_SIZE),mass=IPAD_MASS,
                      color=(.4,.43,.46),quat=(0.,0.,1.,0.)),
         MagneticBody(name='pencil',pos=(.20,-.17,.75055),size=(.166,.0089,.0089),mass=PENCIL_MASS,
                      color=(.9,.9,.88),quat=(2**-.5,0.,0.,2**-.5))]
SUCCESS_HOLD_S=0.
SCORE_OVER='end'
def score(objects):return runtime.ACTIVE.scorer.score if runtime.ACTIVE else 0.
def success(objects):return score(objects)>=1.
