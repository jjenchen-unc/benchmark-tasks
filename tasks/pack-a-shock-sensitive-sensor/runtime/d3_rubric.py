"""Source-scoped D3 final rubric. Pure measured-history evaluation, never advances physics.
Synthetic contract checks are not actual task/robot/model evidence.
"""
import math
LIMIT_CONTROLS=1604
LIMIT_SECONDS=80.2
G_LIMIT=50.
HOLD_S=1.
def finite(x):
 return isinstance(x,(int,float)) and not isinstance(x,bool) and math.isfinite(x)
def boolean(x):
 if type(x) is not bool:raise ValueError('Boolean channel required')
 return x
def evaluate(h):
 try:
  required=tuple(h['required'])
  if not required or len(set(required))!=len(required) or any(x not in 'AB' or len(x)!=1 for x in required):raise ValueError('Required box set invalid')
  if not boolean(h['valid']) or not boolean(h['fresh']) or type(h['samples']) is not int or not h['samples']>0:return {'score':0.,'stage':'fail_closed'}
  if any(not boolean(h['verified_channels'][k]) for k in ['clock','shock','contact','wrist','package_force','latch','drop']):return {'score':0.,'stage':'unverified_measurement'}
  controls,seconds=h['controls'],h['physics_s']
  if type(controls) is not int or controls<0 or not finite(seconds) or seconds<0:raise ValueError('Invalid episode clock')
  if controls>LIMIT_CONTROLS or seconds>LIMIT_SECONDS+1e-9:return {'score':0.,'stage':'over_budget'}
  peak=h['peak_any_g']
  if set(peak)!=set('AB') or any(not finite(x) or x<0 for x in peak.values()):raise ValueError('Missing global shock history')
  if any(x>G_LIMIT for x in peak.values()) or boolean(h['diverged']) or boolean(h['damage_veto']):return {'score':0.,'stage':'damage_veto'}
  if boolean(h['wrist_veto']) or boolean(h['crush_veto']):return {'score':0.,'stage':'force_veto'}
  if boolean(h['drop_contact_veto']):return {'score':0.,'stage':'drop_interference'}
  if abs(seconds-controls/20.)>1e-8:return {'score':0.,'stage':'clock_action_disagreement'}
  source,independent=h['source_drops'],h['independent_drops']
  for drop in source+independent+h['open_drop_records']:
   if type(drop['carton']) is not int or not 1<=drop['carton']<=4:raise ValueError('Unknown carton in drop history')
   if not isinstance(drop['boxes'],(list,tuple)) or len(set(drop['boxes']))!=len(drop['boxes']) or any(x not in ['A','B'] for x in drop['boxes']):raise ValueError('Invalid drop payload inventory')
  for box in required:
   carton=h['final'][box]['carton']
   if carton is not None and (type(carton) is not int or not 1<=carton<=4):raise ValueError('Invalid final carton')
  matched=[];agreement=True
  for n in range(1,5):
   a=[x for x in source if x['carton']==n];b=[x for x in independent if x['carton']==n]
   # A trailing in-progress record is never credited. Any unexplained extra completed record fails closed.
   if len(a)!=len(b):
    longer=a if len(a)>len(b) else b
    if abs(len(a)-len(b))!=1 or boolean(longer[-1]['completed']):agreement=False
   for x,y in zip(a,b):
    if set(x['boxes'])!=set(y['boxes']) or boolean(x['robot_contact'])!=boolean(y['robot_contact']):agreement=False
    for box in set(x['boxes'])|set(y['boxes']):
     px,py=x['peak_g'][box],y['peak_g'][box]
     if not finite(px) or not finite(py) or px<0 or py<0:raise ValueError('Invalid drop shock')
     if abs(px-py)>max(3.0,0.15*px):agreement=False
    matched.append((x,y))
  if not agreement:return {'score':0.,'stage':'drop_disagreement'}
  passed={};tested={}
  for box in required:
   tested[box]=any(box in x['boxes'] for x in source+independent+h['open_drop_records'])
   good=set()
   for x,y in matched:
    if box not in y['boxes']:continue
    if not boolean(x['completed']) or not boolean(y['completed']):continue
    numeric=[x['rise_m'],y['rise_m'],x['postlanding_s'],y['postlanding_s']]
    if any(not finite(v) or v<0 for v in numeric):raise ValueError('Invalid drop timing/rise')
    if (min(x['rise_m'],y['rise_m'])>=.25 and min(x['postlanding_s'],y['postlanding_s'])>=2.
        and boolean(x['released']) and boolean(y['released']) and not x['robot_contact'] and not y['robot_contact']
        and x['peak_g'][box]<G_LIMIT and y['peak_g'][box]<G_LIMIT and boolean(x['ok'][box])):
     good.add(y['carton'])
   final=h['final'][box];hold=final['hold_s']
   if not finite(hold) or hold<0:raise ValueError('Invalid final hold')
   passed[box]=bool(final['carton'] in good and boolean(final['packed_now']) and boolean(final['hands_off']) and hold>=HOLD_S-1e-9)
  if len(required)==2 and all(passed.values()) and h['final']['A']['carton']==h['final']['B']['carton']:passed={x:False for x in passed}
  if all(passed.values()):return {'score':1.,'stage':'full','passed':passed}
  if len(required)==2 and sum(passed.values())==1:
   other=next(x for x in passed if not passed[x])
   if not tested[other]:return {'score':.5,'stage':'one_of_two_unshipped','passed':passed}
  return {'score':0.,'stage':'incomplete','passed':passed}
 except (KeyError,TypeError,ValueError,OverflowError,IndexError):
  return {'score':0.,'stage':'fail_closed'}
