"""Produce an UNAPPROVED review form from fresh complete actual replay only."""
from pathlib import Path
import sys,argparse,os
O=Path(__file__).resolve().parent;sys.path.insert(0,str(O))
from protocol import fresh,sha,CHANNELS
from strict_json import dumps
p=argparse.ArgumentParser();p.add_argument('--replay',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
f=fresh(a.replay);out=a.out.resolve()
if out.exists():raise ValueError('fresh review form required')
rel=os.path.relpath(a.replay.resolve()/'result.json',out.parent)
if rel.startswith('../'):raise ValueError('review must be in common ancestor of replay and evidence')
r={'kind':'HYDRAULIC_ROOT_PHYSICAL_REVIEW','approved':False,'reviewer':None,'manifest':f['manifest'],'fresh_replay':f,'replay_result':{'path':rel,'sha256':sha(a.replay/'result.json')},'channels':list(CHANNELS),'decisions':{k:{'approved':False,'scope':'','proofs':[]} for k in (*CHANNELS,'supported_mount')}}
out.write_text(dumps(r,indent=2)+'\n')
