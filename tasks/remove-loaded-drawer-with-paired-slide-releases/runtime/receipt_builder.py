"""Build a receipt from exact selected actual inputs and explicit ROOT review only."""
from pathlib import Path
import argparse,sys,json
O=Path(__file__).resolve().parent;sys.path.insert(0,str(O/'candidate'))
from runtime_credit import validate
from model_credit import validate_model
p=argparse.ArgumentParser();p.add_argument('--root-review',type=Path);p.add_argument('--route',choices=['reference','model'],default='reference');p.add_argument('--out',type=Path,required=True);a=p.parse_args()
r={'status':'PENDING_ROOT_PHYSICAL_REVIEW','job_id':43920,'runtime_channels_promoted':[],'physical_fullgoal_verified':False,'submission_ready':False,'model_score':None,'route':a.route,'missing':['matching actual43920 full result/observed rows and explicit source-bound ROOT physical review'] if a.route=='reference' else ['fresh affected physical validation of implemented geometric pad mapping','portable source-bound ROOT general-model review; selected no-contact reference review insufficient']}
if a.root_review:
 try:r=validate_model(O/'candidate',__import__('os').path.relpath(a.root_review.resolve(),O/'candidate')) if a.route=='model' else validate(O/'candidate',a.root_review)
 except (ValueError,KeyError,TypeError,OSError) as e:
  r['reason']=str(e);a.out.write_text(json.dumps(r,indent=2,allow_nan=False)+'\n');raise SystemExit(2)
a.out.write_text(json.dumps(r,indent=2,allow_nan=False)+'\n')
print(json.dumps(r,allow_nan=False))
