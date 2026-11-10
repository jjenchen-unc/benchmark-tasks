"""Portable C1 constants/parameter distribution. No simulation imports."""
import math
import numpy as np
ANGLE=math.radians(20)
DT=.001
CONTROL_HZ=20
BUDGET=2268
TABLE_Z=.7475
# Proper rotation + translation, no rescaling: native +y -> station +x,
# native +x -> station -y. Six source liners stay on the left side.
A=np.array([[0.,1.,0.],[-1.,0.,0.],[0.,0.,1.]])
OFFSET=np.array([.40,0.,TABLE_Z-.80])
def world(p):return A@np.asarray(p,dtype=float)+OFFSET
def native(p):return A.T@(np.asarray(p,dtype=float)-OFFSET)
NATIVE_RACK_C=np.array([.05,.25,.9004])
RACK_C=world(NATIVE_RACK_C)
N_RACK=A@np.array([0.,-math.sin(ANGLE),math.cos(ANGLE)])
U_SLOPE=A@np.array([0.,math.cos(ANGLE),math.sin(ANGLE)])
RACK_TOP_C=RACK_C+N_RACK*.006
RACK_LOW_Z=float((RACK_TOP_C-U_SLOPE*.12)[2])
LINER_XY=[[-.05+.10*(i%3),(-.33,-.14)[i//3]] for i in range(6)]
CUP_XY=[[.28,-.30],[.28,-.16]]
LINER_START=[world([x,y,.803]) for x,y in LINER_XY]
CUP_START=[world([x,y,.8355]) for x,y in CUP_XY]
PAN_C=world([.05,.035,.80]);PAN_IN=np.array([.07,.085]);WATER_TOP=TABLE_Z+.024
WORKSPACE=np.array([[-.30,.35],[-.40,.40],[.825,1.18]])
TARGETS={'A_dry_cup':(1,False),'C1-T1':(1,False),'C1-T2':(1,True),'C1-T3':(2,False),'C1-T4':(1,False)}
def parameters(seed,transfer=False):
 r=np.random.default_rng(np.random.SeedSequence([seed,7102 if transfer else 7101]))
 n=int(r.integers(2,4));hold=np.array([True]*n+[False]*(6-n));r.shuffle(hold)
 mu=[float(r.uniform(*((.46,1.) if h else (.15,.27)))) for h in hold]
 holders=[i for i in range(6) if hold[i]];nw=int(r.integers(1,min(2,n-1)+1));wh=set(r.choice(holders,nw,replace=False).tolist())
 wet=[float(r.uniform(*( (max(.46,.3*mu[i]),min(1.,1.2*mu[i])) if i in wh else (.3*mu[i],min(.31,1.2*mu[i])) ))) for i in range(6)]
 clip=lambda x:min(round(x,4),.3099) if x<.31 else round(x,4)
 return {'mu_dry':list(map(clip,mu)),'mu_wet':list(map(clip,wet))}
def slot_point(slot):
 p=RACK_TOP_C.copy();p+=A@np.array([(-.04,.12)[slot]-.05,0.,0.]);return p

def rotation_xyzw(q):
 x,y,z,w=q
 return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def supported(touch,include_cups=True):
 mobile={1,32,*range(20,26)}
 if include_cups:mobile.update(range(10,12))
 reached={1};pending=[1]
 while pending:
  a=pending.pop()
  for u,v in touch:
   if u==a and v in mobile and v not in reached:reached.add(v);pending.append(v)
   if v==a and u in mobile and u not in reached:reached.add(u);pending.append(u)
 return reached

# v2: real supported task-local mounts; station cameras remain world-fixed.
ROBOT_BASES={'left':(.36,.20,.8975),'right':(.40,-.36,.8975)}
ROBOT_YAW={'left':-math.pi/2,'right':math.pi/2}
PARK_POINTS=[np.array([.25,-.05,.98]),np.array([.55,-.15,.98])]
BASE_BOTTOM_Z=-.0011011120300485344
