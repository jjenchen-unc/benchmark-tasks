"""Regression for the contact made by the finite-radius bends at the tab stroke."""
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
import layout as L

def load(side_mm):
    loads=L.flange_springs(L.H+L.hinge_point()+[0.,side_mm/1000.,0.],L.clip_rot(.2288),L.H,np.eye(3))
    for point,force in loads:
        assert abs(point[0]-(L.H[0]-L.FLANGE_T))<1e-10
        radius=np.linalg.norm((point-L.H)[1:])
        assert L.PILOT_R-1e-12<=radius<=L.FLANGE_R+1e-12
        assert force[0]>=0 and abs(force[1])+abs(force[2])<1e-10
    return sum(f[0] for _,f in loads)

def test_wire_edge_support():
    assert 19<load(0)<20
    sweep=np.array([load(s) for s in np.arange(1.8,2.3,.005)])
    assert sweep.min()>16
    assert np.abs(np.diff(sweep)).max()<.1
    assert load(10)==0

if __name__=='__main__':
    test_wire_edge_support()
    print('finite wire support and annular contact positions passed')
