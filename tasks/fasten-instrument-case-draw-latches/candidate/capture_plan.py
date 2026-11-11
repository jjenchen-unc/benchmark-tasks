"""Measured native planar hook inverse geometry; reads only, no state writes.
The target is selected by native link/slide elastic energy, then approached by
bounded cap pushes. This predicts geometry, not contact/capture success.
"""
import numpy as np
from yam_arena.urdf import axis_angle_matrix,quat_xyzw_to_matrix

def plan(fixture,bodies,joints,n):
 meta={b['name']:b for b in fixture['bodies']};hook=meta[n+'_hook'];link=meta[n+'_link']
 pivot,lr=bodies[n+'_link'];hp,hr=bodies[n+'_hook'];lp,lR=bodies['lid_'+n[0]]
 slide=next(j for j in hook['joints'] if j['kind']=='slide');hinge=link['joints'][0]
 tiplocal=np.array(next(s['pos'] for s in hook['sites'] if s['name']==n+'_hook_tip'))
 cap=next(g for g in hook['geoms'] if g['name']==n+'_hook_cap')
 tip=hp+hr@tiplocal;center=hp+hr@np.array(cap['pos']);axis=lr@np.array(hinge['axis']);radial=tip-pivot
 candidates=[]
 for pin in meta['lid_'+n[0]]['sites']:
  if not pin['name'].startswith(n+'_pin'):continue
  target=lp+lR@np.array(pin['pos']);desired=target-pivot
  a=radial-axis*np.dot(radial,axis);b=desired-axis*np.dot(desired,axis)
  delta=float(np.arctan2(np.dot(axis,np.cross(a,b)),np.dot(a,b)))
  theta=float(joints[n+'_link']+delta);s=float(joints[n+'_slide']+np.linalg.norm(b)-np.linalg.norm(a))
  if not hinge['range'][0]<=theta<=hinge['range'][1] or not slide['range'][0]<=s<=slide['range'][1]:continue
  Q=axis_angle_matrix(axis,delta);translation=hr@np.array(slide['axis'])*(s-joints[n+'_slide'])
  predicted=pivot+Q@(radial+translation)
  residual=float(np.linalg.norm(predicted-target))
  if residual>=.0025:continue
  cap_goal=pivot+Q@(center-pivot+translation)
  energy=.5*hinge['stiffness']*(theta-hinge['ref'])**2+.5*slide['stiffness']*(s-slide['ref'])**2
  energy+=.5*fixture['springs'][n]*max(0.,s+.004)**2
  candidates.append((energy,pin['name'],theta,s,residual,cap_goal))
 if not candidates:raise RuntimeError('No native-range hook inverse geometry: '+n)
 energy,pin,theta,s,residual,goal=min(candidates,key=lambda x:x[0]);delta=goal-center;distance=float(np.linalg.norm(delta))
 direction=delta/distance if distance>.001 else bodies['case_'+n[0]][1]@np.array([0.,1.,0.])
 w,x,y,z=cap['quat'];cr=hr@quat_xyzw_to_matrix(np.array([x,y,z,w]));extent=float(np.sum(abs(cr.T@direction)*cap['size']))
 return {'front':center-extent*direction,'direction':direction,'cap_goal':goal,'remaining_m':distance,'pin':pin,'link_target_rad':theta,'slide_target_m':s,'tip_residual_m':residual,'elastic_energy_J':float(energy)}
