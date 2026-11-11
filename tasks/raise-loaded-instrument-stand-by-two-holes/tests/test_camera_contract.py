import hashlib,sys,unittest
from pathlib import Path
R=Path(__file__).resolve().parent.parent;sys.path[:0]=[str(R/"runtime"),str(R/"source")]
import guard as G,contract,overview_camera as oc
EIGHT={"clock","joint_state","robot_contacts","wrist_force","instrument_acceleration","pin_mechanism","total_motor_limits","reset"}
h=lambda p:hashlib.sha256((R/p).read_bytes()).hexdigest()
class T(unittest.TestCase):
 def test_contract_eight(self):self.assertEqual(set(contract.REQUIRED),EIGHT);self.assertEqual(set(G.ROOT_CHANNELS),EIGHT);self.assertEqual(len(G.ROOT_CHANNELS),8)
 def test_reference_hash(self):self.assertEqual(h("source/reference.py"),G.REFERENCE_SHA256)
 def test_adapter_exact(self):self.assertEqual(h("runtime/gripper_saturation.py"),G.ADAPTER_SHA256)
 def test_overview_covers_all(self):
  o=oc.overview();self.assertTrue(o["coverage"]["all_inside"]);self.assertGreater(o["coverage"]["z_min"],.5);self.assertFalse(o["model_observation"])
  self.assertEqual((o["width"],o["height"]),(640,360))
 def test_moving_extremes_included(self):
  p=oc.source_points();self.assertGreater(len(p),1500);self.assertGreater(p[:,2].max(),1.25)
 def test_old_top_clips_detectable(self):
  import numpy as np
  # calibrated SDK top cam (pinned station.py) values; source clipped at top
  pos=np.array([-0.16649487853050232,-0.008999999612569809,1.6918205499649046]);x,y,z,w=0.18301085645621015,-0.18301085645621015,-0.6830131963726359,0.6830131963726355
  Rm=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
  self.assertFalse(oc.coverage(oc.project(oc.source_points(),pos,Rm,392.195617675781,391.722351074219,318.389434814453,177.876312255859))["all_inside"])
if __name__=="__main__":unittest.main()
