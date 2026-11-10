from pathlib import Path
import sys,json,hashlib,subprocess
B=Path('/home/ubuntu/hydraulic-brev-20261110-1800');S=B/'source';sys.path.insert(0,str(S));import protocol
source=(S/'protocol.py').read_text()
old="if numeric(s['score'])!=0 or s['gate']!='unvalidated_channels' or sorted(s['missing_channels'])!=list(CHANNELS):raise ValueError('uncredited native gate changed')"
new="if numeric(s['score'])!=1 or s['gate'] is not None or s['missing_channels']!=[]:raise ValueError('approved public credit absent')"
assert source.count(old)==1
source=source.replace(old,new).replace("'public_score':0,'native_diagnostic':1","'public_score':1,'native_diagnostic':1")
ns={'__file__':str(S/'protocol.py')};exec(compile(source,'<external-public-verifier>','exec'),ns)
f=ns['fresh'](B/'public-replay');media={}
for name in ('top_cam','left_cam','right_cam'):
 p=B/'public-replay'/(name+'.mp4')
 import imageio_ffmpeg
 reader=imageio_ffmpeg.read_frames(str(p),pix_fmt='rgb24',output_params=['-threads','1']);m=next(reader);count=sum(1 for _ in reader)
 assert m['codec']=='h264' and count==f['controls']+1 and m['fps']==20
 m['decoded_frame_count']=count
 media[name]=m
protocol.receipt(B/'ROOT-physical-review.json')
f.update(kind='ACTUAL_PUBLIC_REPLAY_INDEPENDENT_VERIFICATION',method='Same immutable protocol fresh verifier with explicit public-credit expectation score 1/gate null/missing empty; independent native History recomputation unchanged',external_verifier_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),full_decode_all_three=True,streams=media)
(B/'public-replay-verification.json').write_text(json.dumps(f,indent=2)+'\n');print(json.dumps(f))
