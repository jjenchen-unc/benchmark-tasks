"""Remove the purple three-bar sleeve from its fixed shaft and rest it wholly
inside the green tray. Leave it clear of the shaft and both robots for20seconds.
"""
import sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from kit import Coupling,install,current_score,FIX
install()
SCENE='station'
OBJECTS=[Coupling(name='coupling_fixture',pos=tuple(FIX['stand_world']),size=(.21,.50,.15),mass=None),Coupling(name='coupling_sleeve',pos=tuple(FIX['sleeve_world']),size=(.059,.059,.0804),mass=.25)]
SCORE_OVER='end'
def score(objects):return current_score()
def demo():return {}
