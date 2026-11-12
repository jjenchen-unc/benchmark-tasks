"""Stack orange A, blue B, purple C in that bottom-to-top order on the steel
point pedestal. Bars are320 x50 x50mm with sealed eccentric steel weights;
black caps mark -X. Long axes along X within10deg, level within0.06rad,
six-component velocity norm below0.025; contact only adjacent required bars
and pedestal; no robot contact with ANY bar. Release for20continuous seconds
at episode end. Partial credit1/3 or2/3 for an independently valid ordered
prefix. No damage flag is invented; toppling costs time. Wrist30N/20ms stops
motion;80N/100ms aborts.1896 controls at20Hz. Fixed physical case seed42/S;
harness seeds identify episodes and never expose hidden mass to Dream1.
"""
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from kit import Bar,install,current_score
from yam_arena.task_file import Box
from fixture import POSITIONS,PARAMETERS,COLORS,SUPPORT,SUPPORT_SIZE
install()
SCENE='station'
OBJECTS=[Bar(name='bar_'+c,pos=tuple(map(float,POSITIONS[c])),size=(.32,.05,.05),mass=round(PARAMETERS[c]['mass'],6),color=COLORS[c],label=c) for c in 'ABC']+[Box(name='support',pos=tuple(map(float,SUPPORT)),size=SUPPORT_SIZE,color=(.55,.58,.62))]
SCORE_OVER='end'
def score(objects):return current_score()

# Bolted steel risers physically support both elevated arms; geometric clearance unverified.
OBJECTS += [Box(name=side+"_mount_riser",pos=(0.,y,.854),size=(.14,.14,.213),color=(.25,.26,.28)) for side,y in (("left",.305),("right",-.305))]

PUBLIC_PHYSICAL_SEED=42
PUBLIC_SETTING="S"
PUBLIC_GOAL="stack_ABC"
