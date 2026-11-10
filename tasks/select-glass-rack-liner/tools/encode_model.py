"""Encode complete actual Inspect RGB observations without changing the trial."""
from pathlib import Path
import argparse,json,sys
import numpy as np
HERE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(HERE/"candidate"))
from video_capture import Capture

def main():
    p=argparse.ArgumentParser();p.add_argument('output',type=Path);a=p.parse_args()
    out=a.output.resolve();proof=json.loads((out/'model-verification.json').read_text())
    assert proof['valid_completed_trial'] is True
    frames={n:sorted((out/'model/frames').rglob('*'+n+'_*.npy')) for n in ['top_cam','left_cam','right_cam']}
    assert all(len(fs)==proof['steps']+1 for fs in frames.values())
    for fs in frames.values():
        ids=[int(f.stem.rsplit('_',1)[1]) for f in fs]
        assert ids==list(range(len(ids)))
    dest=out/'encoded-media';dest.mkdir(exist_ok=False)
    def images(i):return {n:np.load(fs[i],allow_pickle=False) for n,fs in frames.items()}
    capture=Capture(dest,images(0))
    try:
        for i in range(1,proof['steps']+1):capture.append(images(i))
    finally:result=capture.close()
    assert result['all_encoded'] and result['frames_per_camera']==proof['steps']+1
    print(json.dumps(result))

if __name__=='__main__':main()
