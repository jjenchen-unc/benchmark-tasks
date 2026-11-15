"""Deploy a pre-attached folded-panel support and leave a tablet standing."""
import sys,os,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parent))
import numpy as np
from scipy.spatial.transform import Rotation
from mechanism import TABLET_SIZE,TABLET_MASS,PANEL_SIZE,PANEL_MASS,initial_poses,folded_rest_height
from runtime import StandBody
import runtime
INSTRUCTION='Raise the tablet, unfold its attached back stand, and leave it standing in portrait orientation facing you on the desk, with its screen about 60 degrees above the desk. Release both hands and leave it stable for 5 seconds.'
# Yaw-90 maps portrait local +Y to station-forward +X. Screen remains up.
# Fixtures are initialized only by the diagnostic harness after normal reset.
# An inherited environment variable must never alter a public task layout.
if os.environ.get('STAND_DIAGNOSTIC_FIXTURE'):
 raise ValueError('STAND_DIAGNOSTIC_FIXTURE is forbidden; use explicit harness fixture stage')
DIAGNOSTIC_FIXTURE=False
_initial=initial_poses(np.array([-.04,.10,.7475+folded_rest_height()]),Rotation.from_euler('z',-90,degrees=True).as_quat())

OBJECTS=[StandBody(name=n,pos=tuple(float(x) for x in pos),size=tuple(float(x) for x in (TABLET_SIZE if n=='tablet' else PANEL_SIZE)),mass=TABLET_MASS if n=='tablet' else PANEL_MASS,color=(.23,.27,.30) if n=='tablet' else (.43,.46,.49),quat=tuple(float(x) for x in quat)) for n,(pos,quat) in _initial.items()]
MAX_CONTROL_STEPS=3600  # 180 seconds at 20 Hz, fixed before full native validation.
SUCCESS_HOLD_S=0.
SCORE_OVER='end'
def score(objects):return runtime.ACTIVE.scorer.score if runtime.ACTIVE else 0.
def success(objects):return score(objects)>=1.
