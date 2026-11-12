"""Prepare UNAPPROVED source-bound review from fresh actual replay. Never approve."""
from pathlib import Path
import argparse,sys,os,shutil
O=Path(__file__).resolve().parent;C=O/'candidate';sys.path.insert(0,str(C))
from runtime_credit import load,sha
from strict_json import dumps
from fresh_evidence import check,NAMES

def build(trial,out):
 trial=Path(trial).resolve();out=Path(out).resolve()
 if not out.is_relative_to(O/'work') or out.exists():raise ValueError('use fresh review directory within portable bundle/work')
 summary=check(C,trial)
 out.mkdir(parents=True);dest=out/'fresh';dest.mkdir()
 files=[*NAMES,'live-stage-ownership.json','pre-action.json','cold-task.json', 'top_cam.mp4','left_cam.mp4','right_cam.mp4']
 for n in files:shutil.copy2(trial/n,dest/n)
 review=load(O/'reviewed-actual-portable/ROOT-review.json')
 for row in review['actual_inputs'].values():row['path']=os.path.relpath((O/'reviewed-actual-portable'/row['path']).resolve(),out)
 review.update(model_scope='general-native-pad-mapping',**{key+'_sha256':sha(C/file) for key,file in [('model_entry','model_entry.py'),('model_credit','model_credit.py'),('pad_identity','pad-identity.json')]})
 review['affected_pad_validation']={'schema':'drawers-fresh-pad-replay-v19','approved':False,'physical':False,'controls':summary['controls'],'hold_s':summary['hold_s'],'source_sha256':{name:sha(C/name) for name in ('model_physics.py','pad_region.py','pad-region.json','pad_foundation.py','runtime_credit.py')},'evidence':[{'name':n,'path':'fresh/'+n,'sha256':sha(dest/n)} for n in files],'review_notes':'UNAPPROVED: ROOT must inspect fresh measurement/journal and mapping. Set approved=true and physical=true only after actual evidence review. Native score1 is not public/model score1.'}
 (out/'ROOT-general-pad-UNAPPROVED.json').write_text(dumps(review,indent=2)+'\n')
 (out/'numeric-summary.json').write_text(dumps(summary,indent=2)+'\n')
 print(dumps({'status':'UNAPPROVED_REVIEW_PACKET','review':str(out/'ROOT-general-pad-UNAPPROVED.json'),'numeric':summary}))
if __name__=='__main__':
 p=argparse.ArgumentParser();p.add_argument('--trial',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();build(a.trial,a.out)
