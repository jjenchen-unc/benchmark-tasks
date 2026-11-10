"""Pure/static tests only. Never load an Isaac app or step physics."""
import ast,hashlib,json,math,sys,unittest,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from mechanism import cam,knob_for,Mechanism,History,REQUIRED,DT,LEAD,TH_OC,REV,PITCH
HERE=Path(__file__).resolve().parent
class Contracts(unittest.TestCase):
    def row(self,**kw):
        r=dict(handle=math.pi,gap=.0009,clamp=21.,cam_torque=1.,cam_error=0.,wrist_force=[0.,0.],stripped=False,touch=False);r.update(kw);return r
    def history(self,row,n=10000):
        h=History()
        for _ in range(n):h.update(row,DT)
        return h
    def test_exact_continuous_hold(self):
        h=self.history(self.row(),9999);self.assertEqual(h.result(REQUIRED)['score'],0.);h.update(self.row(),DT);self.assertEqual(h.result(REQUIRED)['score'],1.)
        h.update(self.row(touch=True),DT);self.assertEqual(h.result(REQUIRED)['score'],0.)
    def test_strict_gap_boundaries(self):
        self.assertEqual(self.history(self.row(gap=.001)).result(REQUIRED)['score'],.5)
        self.assertEqual(self.history(self.row(gap=.002)).result(REQUIRED)['score'],.25)
    def test_terminal_clamp_strict(self):self.assertEqual(self.history(self.row(clamp=20.)).result(REQUIRED)['score'],0.)
    def test_lock_angle(self):self.assertEqual(self.history(self.row(handle=math.radians(174.99))).result(REQUIRED)['score'],0.)
    def test_missing_channels(self):self.assertEqual(self.history(self.row()).result()['score'],0.)
    def test_damage_sticky(self):
        h=self.history(self.row());h.update(self.row(stripped=True),DT);h.update(self.row(),DT);self.assertEqual(h.result(REQUIRED)['score'],0.)
    def test_measurement_fail_closed(self):
        h=self.history(self.row(gap=float('nan')),1);self.assertEqual(h.result(REQUIRED)['score'],0.);self.assertIsNotNone(h.failure)
    def test_force_exact_persistence(self):
        h=self.history(self.row(wrist_force=[80.01,0]),49);self.assertIsNone(h.failure);h.update(self.row(wrist_force=[80.01,0]),DT);self.assertEqual(h.failure,'wrist_force_80N_100ms')
    def test_force_resets(self):
        h=self.history(self.row(wrist_force=[81,0]),49);h.update(self.row(),DT);h.update(self.row(wrist_force=[81,0]),DT);self.assertIsNone(h.failure)
    def test_dt_not_control_period(self):self.assertEqual(self.history(self.row(),1).dt,DT);h=History();h.update(self.row(),.05);self.assertEqual(h.failure,'measurement_unavailable')
    def test_cam_continuity_slope(self):
        g=.001942;self.assertAlmostEqual(cam(TH_OC-1e-10,g,0)[0],cam(TH_OC+1e-10,g,0)[0],places=10);self.assertEqual(cam(math.pi,g,0)[1],-REV)
        self.assertAlmostEqual(cam(TH_OC,g,0)[0]-cam(math.pi/2,g,0)[0],.0064)
    def test_screw_pitch(self):self.assertAlmostEqual(cam(0,0,2*math.pi)[0]-cam(0,0,0)[0],.001)
    def test_reference_target_is_fullgap(self):
        s=json.loads((HERE/'fixture.json').read_text())['spec'];c=cam(math.pi,s['grip'],knob_for(s))[0];self.assertLess(s['g0']-c,.001);self.assertGreater(s['g0']-c,.0005)
    def test_cam_virtual_work(self):
        m=Mechanism(dict(grip=.001,g0=.0068,k=33416.5));q=dict(pawl=-.003,handle=1.,knob=0.,lid_slide=0.);v=dict.fromkeys(q,0.);f=m.efforts(q,v,0,DT);self.assertAlmostEqual(f['handle'],-f['pawl']*LEAD);self.assertAlmostEqual(f['knob'],-f['pawl']*PITCH)
    def test_loaded_screw_hysteresis(self):
        m=Mechanism(dict(grip=0,g0=.0068,k=30000));q=dict(pawl=cam(0,0,0)[0],handle=0.,knob=0.,lid_slide=0.);v=dict.fromkeys(q,0.);m.efforts(q,v,2.01,DT);self.assertEqual(m.lock,0.);m.efforts(q,v,.6,DT);self.assertEqual(m.lock,0.);m.efforts(q,v,.49,DT);self.assertIsNone(m.lock)
    def test_policy_no_state_mutators(self):
        tree=ast.parse((HERE/'reference.py').read_text());calls={n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)}
        self.assertFalse(calls & {'write_joint_state_to_sim','write_root_pose_to_sim','write_root_pose_to_sim_index','set_joint_effort_target','set_joint_position_target','add_forces_and_torques','efforts','update'})
        self.assertTrue('step' in calls)
    def test_native_fixture_geometry(self):
        f=json.loads((HERE/'compiled-fixture.json').read_text());self.assertEqual(f['njnt'],4);self.assertEqual(f['ngeom'],13);self.assertEqual(f['provenance']['simulation_steps'],0);self.assertEqual(f['provenance']['fixture_xml_sha256'],hashlib.sha256((HERE/'fixture.xml').read_bytes()).hexdigest())
        self.assertEqual(f['names']['joint'],['lid_slide','handle','knob','pawl']);self.assertEqual(f['dof_armature'],[20.,.005,.005,250.])
    def test_portable_no_absolute_workspace(self):
        for p in HERE.glob('*.py'):self.assertNotIn('/'+'mnt'+'/'+'bum',p.read_text())
    def test_no_publication_claim(self):
        t=json.loads((HERE/'task.draft.json').read_text());self.assertNotIn('media',t);self.assertTrue(t['not_submission_ready'])
if __name__=='__main__':unittest.main(verbosity=2)
