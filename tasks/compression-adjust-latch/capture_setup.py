"""Capture actual reset RGB after render-only warmup; assert state preservation."""
from pathlib import Path
import argparse,json,sys
from package_checks import preflight,dumps,digest
HERE=Path(__file__).resolve().parent

def main():
    p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);p.add_argument('--preflight-only',action='store_true');p.add_argument('--driver-check-override',action='store_true');a=p.parse_args()
    preflight(HERE)
    if a.preflight_only:
        print(dumps(dict(valid=True,model_calls=0,simulation_starts=0)));return
    a.out=a.out.resolve()
    if a.out==HERE or HERE in a.out.parents:raise ValueError('Output must be outside package')
    a.out.mkdir(parents=True,exist_ok=False)
    sys.path.insert(0,str(HERE/'runtime'))
    import driver
    sys.meta_path.insert(0,driver.FrozenImports());driver.install_override(a.driver_check_override)
    from inspect_robots import Scene
    from yam_arena.inspect_embodiment import yam_arena_embodiment
    import numpy as np
    from PIL import Image
    emb=yam_arena_embodiment(task=str(HERE/'runtime/task.py'),device='cpu',object_jitter=0,control_hz=20,cam_width=640,cam_height=360)
    try:
        obs=emb.reset(Scene(id='compression-latch-reset-setup',instruction=json.loads((HERE/'task.draft.json').read_text())['instruction']),seed=42)
        import kit
        base=emb._base
        def state():
            result={}
            for group in ('articulations','rigid_objects'):
                for name,obj in getattr(base.scene,group).items():
                    for field in ('body_state_w','root_state_w','joint_pos','joint_vel'):
                        if hasattr(obj.data,field):result[name+'/'+field]=kit.arr(getattr(obj.data,field)).copy()
            assert any('joint_pos' in k for k in result) and any('compression_latch/' in k for k in result)
            return result
        before=state();s0=kit.current_status()['samples'];assert s0==0 and kit._STEPS==0
        for _ in range(40):base.sim.render()
        captures=[]
        for name in ('top_cam','left_cam','right_cam'):
            cam=base.scene[name];cam.update(0.,force_recompute=True)
            rgb=kit.arr(cam.data.output['rgb'])[0,...,:3].astype(np.uint8)
            assert rgb.shape==(360,640,3) and rgb.std()>5
            path=a.out/(name+'.png');Image.fromarray(rgb).save(path)
            captures.append(dict(camera=name,shape=list(rgb.shape),sha256=digest(path)))
        after=state();unchanged=before.keys()==after.keys() and all(np.array_equal(v,after[k]) for k,v in before.items())
        s1=kit.current_status()['samples'];assert unchanged and s1==s0==0 and kit._STEPS==0
        preflight(HERE)
        (a.out/'result.json').write_text(dumps(dict(captures=captures,all_robot_body_joint_states_unchanged=unchanged,state_channels=sorted(before),samples_before=s0,samples_after=s1,episode_controls_before=0,episode_controls_after=kit._STEPS,model_calls=0))+'\n')
    finally:emb.close()
if __name__=='__main__':main()
