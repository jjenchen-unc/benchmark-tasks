"""Stream the existing three RGB cameras. No camera/physics/scorer writes."""
from pathlib import Path
import shutil,subprocess
import numpy as np
from PIL import Image,ImageOps
class Recorder:
    def __init__(self,emb,out,kit):
        self.base=emb._base;self.out=Path(out);self.kit=kit;self.count=0
        images=self.frames()
        assert all(v.size and float(v.std())>1 for v in images.values()),"Missing/black render"
        for n,v in images.items():Image.fromarray(v).save(self.out/("initial-"+n+".png"))
        Image.fromarray(images["top_cam"]).save(self.out/"setup.png")
        ff=shutil.which("ffmpeg")
        if not ff:
            import imageio_ffmpeg
            ff=imageio_ffmpeg.get_ffmpeg_exe()
        self.log=(self.out/"encode.log").open("wb")
        self.writer=subprocess.Popen([ff,"-y","-loglevel","error","-f","rawvideo","-pix_fmt","rgb24",
            "-s","1280x720","-r","20","-i","pipe:0","-an","-c:v","libx264","-threads","1",
            "-pix_fmt","yuv420p","-crf","22","-maxrate","3000k","-bufsize","6000k",
            "-movflags","+faststart",str(self.out/"demo.mp4")],stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,stderr=self.log)
    def frames(self):
        return {n:np.asarray(self.kit.arr(self.base.scene[n].data.output["rgb"])[0,...,:3],dtype=np.uint8)
                for n in ("top_cam","left_cam","right_cam")}
    def write(self,obs):
        assert set(obs.images)=={"top_cam","left_cam","right_cam"}
        images=self.frames();canvas=Image.new("RGB",(1280,720))
        for n,xy in (("top_cam",(320,0)),("left_cam",(0,360)),("right_cam",(640,360))):
            im=ImageOps.contain(Image.fromarray(images[n]),(640,360));canvas.paste(im,xy)
        self.writer.stdin.write(np.asarray(canvas).tobytes());self.count+=1
    def close(self):
        for n,v in self.frames().items():Image.fromarray(v).save(self.out/("final-"+n+".png"))
        self.writer.stdin.close();code=self.writer.wait(timeout=60);self.log.close()
        return dict(encoder_returncode=code,frames=self.count,width=1280,height=720,fps=20,cameras=["top_cam","left_cam","right_cam"])
