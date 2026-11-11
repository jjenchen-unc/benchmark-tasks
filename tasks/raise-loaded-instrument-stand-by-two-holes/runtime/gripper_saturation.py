"""Task-local model conversion: saturate both public grippers on every returned row.

Only the existing normalized physical gripper endpoints [0,1] are applied.
Every finite arm value, row and ordering passes unchanged to the original SDK
station decoder. This is neither a new model response nor an inference retry.
"""
from collections.abc import Mapping
from pathlib import Path
import hashlib,json
import numpy as np
POLICY_LABEL='Dream-1 + public gripper saturation [0,1] on all returned rows'
class PublicGripperRows:
    def __init__(self,original,log_path):
        self.original=original;self.log_path=Path(log_path);self.calls=0
    def __call__(self,reply):
        # Malformed/nonfinite inputs receive the original complete validation;
        # projection never rescues missing/invalid arm data or row structure.
        if not isinstance(reply,Mapping):return self.original(reply)
        arrays={}
        for key in ('q_l_chunk','q_r_chunk'):
            if key not in reply:return self.original(reply)
            try:a=np.array(reply[key],dtype=np.float64)
            except (TypeError,ValueError):return self.original(reply)
            if a.ndim!=2 or a.shape[1]!=7 or not 1<=len(a)<=48 or not np.isfinite(a).all():return self.original(reply)
            arrays[key]=a
        if arrays['q_l_chunk'].shape!=arrays['q_r_chunk'].shape:return self.original(reply)
        pre={};clipped=[];counts={'left':0,'right':0};before=np.hstack([arrays['q_l_chunk'][:,:6],arrays['q_r_chunk'][:,:6]])
        for side,key in [('left','q_l_chunk'),('right','q_r_chunk')]:
            a=arrays[key];g=a[:,6].copy();pre[side]=[float(g.min()),float(g.max())]
            for i in np.flatnonzero((g<0.)|(g>1.)):
                clipped.append({'side':side,'row_index':int(i),'before':float(g[i]),'after':0. if g[i]<0 else 1.});counts[side]+=1
            a[:,6]=np.clip(g,0.,1.)
        bounded=dict(reply);bounded.update(arrays);rows=self.original(bounded)
        after=rows[:,[0,1,2,3,4,5,7,8,9,10,11,12]]
        if before.tobytes()!=after.tobytes():raise RuntimeError('decoder changed arm bytes/row order')
        self.calls+=1
        record={'kind':'public_gripper_projection','chunk_index':self.calls,'rows':len(rows),'bounds':[0.,1.],'preprojection_grip_minmax':pre,'clipped_values':len(clipped),'clipped_by_side':counts,'clipped_rows':clipped,'arm_columns_sha256_before':hashlib.sha256(before.tobytes()).hexdigest(),'arm_columns_sha256_after':hashlib.sha256(after.tobytes()).hexdigest(),'all_rows_preserved':True}
        with self.log_path.open('a') as f:f.write(json.dumps(record,allow_nan=False,separators=(',',':'))+'\n')
        return rows
