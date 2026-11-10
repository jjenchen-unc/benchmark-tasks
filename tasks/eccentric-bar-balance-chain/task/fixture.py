"""Task-local, pure fixed reference case derived from native seed42/S.
Harness seed changes episode identity, not this disclosed fixed case.
"""
import numpy as np
BARS='ABC'
SHIFT=np.array([.25,-.15,-.0525])
POSITIONS={c:np.array([0.,y,.825])+SHIFT for c,y in zip(BARS,[-.30,-.19,-.08])}
SUPPORT=np.array([0.,.18,.88])+SHIFT
SUPPORT_SIZE=(.028,.030,.160)
COLORS={'A':(.96,.56,.14),'B':(.12,.35,.90),'C':(.65,.14,.80)}
def parameters():
 p={}
 for i,c in enumerate(BARS):
  r=np.random.default_rng(np.random.SeedSequence([42,4404,i]));sign=float(r.choice([-1.,1.]));p[c]={'mass':float(r.uniform(.55,.85)),'com':sign*float(r.uniform(.010,.090))}
 return p
PARAMETERS=parameters()
def inertia(m,c):
 ms=m-.19;xs=m*c/ms
 ix=.19*.05**2/6+ms*.048**2/6
 iy=.19*(.32**2+.05**2)/12+.19*c*c+ms*(xs-c)**2+ms*.048**2/12
 # Native XML rounds COM to6decimals/inertia to7; preserve actual compiled specification.
 return round(c,6),[round(ix,7),round(iy,7),round(iy,7)]

ROBOT_BASES={"left":(0.,.305,.97),"right":(0.,-.305,.97)}
