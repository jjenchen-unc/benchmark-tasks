"""Record actual observation RGB only, without changing any physics/action."""
from pathlib import Path
import json,subprocess,shutil
import numpy as np
class Capture:
    def __init__(self,out,images):
        self.out=Path(out);self.frames=0;self.writers={};self.logs={}
        self.ff=shutil.which("ffmpeg")
        if not self.ff:
            import imageio_ffmpeg
            self.ff=imageio_ffmpeg.get_ffmpeg_exe()
        if set(images)!={"top_cam","left_cam","right_cam"}:raise ValueError("Expected three actual cameras")
        self.shapes={name:rgb.shape for name,rgb in images.items()}
        for name,rgb in images.items():
            h,w,c=rgb.shape
            if c!=3 or h%2 or w%2 or rgb.dtype!=np.uint8:raise ValueError("Invalid camera RGB")
            log=(self.out/(name+"-encoder.log")).open("wb");self.logs[name]=log
            cmd=[self.ff,"-y","-loglevel","error","-f","rawvideo","-pix_fmt","rgb24","-s",str(w)+"x"+str(h),"-r","20","-i","pipe:0","-an","-c:v","libx264","-threads","1","-pix_fmt","yuv420p","-crf","24","-maxrate","650k","-bufsize","1300k","-movflags","+faststart",str(self.out/(name+".mp4"))]
            self.writers[name]=subprocess.Popen(cmd,stdin=subprocess.PIPE,stdout=subprocess.DEVNULL,stderr=log)
        self.append(images)
    def append(self,images):
        if set(images)!=set(self.writers):raise ValueError("Camera inventory changed")
        for name,rgb in images.items():
            if rgb.shape!=self.shapes[name] or rgb.dtype!=np.uint8:raise ValueError("Camera frame format changed")
            self.writers[name].stdin.write(np.ascontiguousarray(rgb).tobytes())
        self.frames+=1
    def close(self):
        results={}
        for name,w in self.writers.items():
            try:w.stdin.close();results[name]=w.wait(timeout=30)
            except BaseException as exc:results[name]=type(exc).__name__;w.terminate()
            self.logs[name].close()
        proof={"kind":"Actual synchronized simulator observations, no added frames/labels/holds","frames_per_camera":self.frames,"encoder_returncodes":results,"all_encoded":len(results)==3 and all(v==0 for v in results.values()),"files":{name+".mp4":{"bytes":(self.out/(name+".mp4")).stat().st_size} for name in self.writers if (self.out/(name+".mp4")).exists()}}
        (self.out/"media-finalization.json").write_text(json.dumps(proof,indent=2)+"\n")
        return proof
