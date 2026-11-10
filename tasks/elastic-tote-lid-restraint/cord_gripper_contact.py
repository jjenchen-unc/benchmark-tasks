"""Classify temporary own-end softcord/gripper contact in omitted contact model.
No physics APIs. Surface contact is not a no-contact claim or force prediction.
"""
def classify(check,segment=0,pad_proxy=False):
 issues=[];surface=[];proxy=[]
 for c in check['exact_cord_flags']:
  if not c['centreline_inside'] and c['distance_lower_m']>=.004-1e-7:continue
  own=c['segment']==segment and c['link'] in ('gripper','tip_left','tip_right')
  if not own:issues.append(c)
  elif c['centreline_inside']:
   if pad_proxy and c['link'] in ('tip_left','tip_right'):proxy.append(c)
   else:issues.append(c)
  else:surface.append(c)
 return dict(issues=issues,surface_contacts=surface,pad_local_proxy=proxy)
