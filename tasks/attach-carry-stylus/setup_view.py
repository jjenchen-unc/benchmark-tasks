"""Render the unchanged initial setup from a supplementary review camera."""
import json,sys
from pathlib import Path
import numpy as np
import imageio
import demo,runtime

def main():
    out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=True)
    trial=demo.Trial(out,record=False)
    camera=trial.emb._base.scene['overview_cam']
    before={n:runtime.ACTIVE.views[n].get_transforms().tolist() for n in ('ipad','pencil')}
    views={'front':([.65,-.65,1.6],[.08,0,.76]),'back':([.65,.65,1.6],[.08,0,.76]),'high':([.55,0,1.65],[.08,0,.76])}
    for name,(eye,target) in views.items():
        camera.set_world_poses_from_view([eye],[target])
        for _ in range(12):
            trial.emb._base.sim.render()
            camera.update(.05,force_recompute=True)
        rgb=runtime.arr(camera.data.output['rgb'])[0,:,:,:3].astype(np.uint8)
        imageio.imwrite(out/(name+'.png'),rgb)
    after={n:runtime.ACTIVE.views[n].get_transforms().tolist() for n in ('ipad','pencil')}
    assert before==after,'Rendering must not change object states'
    (out/'setup-render.json').write_text(json.dumps(dict(views=views,before=before,after=after,objects_unchanged=True,scope='Initial reset scene; only supplementary review camera moved. No physics steps or robot/object commands.'),indent=2))
    trial.trace.close()
if __name__=='__main__':main()
