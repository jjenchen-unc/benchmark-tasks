"""Exercise the runtime thread model with measured poses. Requires the pure-Python yam_arena task-file package."""

import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "isaac"))
try:
    import panelkit as P
except ModuleNotFoundError as exc:
    if exc.name not in ("yam_arena", "yam_arena.task_file"):
        raise
    raise unittest.SkipTest("yam_arena is needed to import the runtime") from exc


class TestThread(unittest.TestCase):
    def setUp(self):
        self.rt = P.Runtime.__new__(P.Runtime)
        self.rt.setup = 0
        self.rt._init_state()
        self.axis = self.rt.screw_axis(0)
        self.pc, self.rc = P.L.cover_installed_pose(0)

    def sample(self, phi, dy=0., dx=0.):
        m = self.axis + [P.L.head_under_x(self.rt.adv[0]) - P.L.HEAD_T + dx, dy, 0.]
        self.rt.update_fastener(1/240, self.pc, self.rc, m, P.L.driver_rot_x(phi))

    def test_misalignment_stationary_and_reverse_cannot_start(self):
        for a in np.arange(0.,3.,.05):
            self.sample(a, dy=.003)
        for a in np.arange(3.,6.,.05):
            self.sample(a, dx=-.004)
        self.sample(6.)
        self.sample(6.)
        self.sample(5.9)
        self.assertFalse(self.rt.thread_started)
        self.assertEqual(self.rt.adv, [0.])

    def test_ratchet_advance_and_release(self):
        self.sample(0.)
        self.sample(.8)
        self.assertTrue(self.rt.thread_started)
        self.assertAlmostEqual(self.rt.adv[0], .8*P.L.PITCH/(2*math.pi))
        distance = self.rt.adv[0]
        self.sample(0.)
        self.sample(.6, dy=.004)
        self.assertEqual(self.rt.adv[0], distance)
        for a in np.arange(.6, 2*math.pi*11.5, .1):
            self.sample(a)
        self.assertTrue(self.rt.seated[0])
        self.assertLess(abs(self.rt.adv[0] - P.L.START_GAP), 2e-5)
        self.sample(0., dx=-.05)
        self.assertTrue(self.rt.seated[0])

    def test_screw_in_bare_tub_cannot_earn_seating(self):
        self.pc, self.rc = P.L.cover_stand_pose(0)
        for a in np.arange(0., 2*math.pi*12., .1):
            self.sample(a)
        self.assertTrue(self.rt.thread_started)
        self.assertFalse(self.rt.seated[0])
        self.assertGreater(self.rt.adv[0], P.L.START_GAP)


class TestLoadedArmDynamics(unittest.TestCase):
    def test_station_compensates_arm_gravity_and_reference_compensates_held_tool(self):
        import demo as D
        from driver_dynamics import joint_torque

        q = np.array([.8, 2.0, .8, .5, 1.5, -1.2])
        zero = np.zeros(6)
        gp = np.array([0., -.047, .095])
        gr = np.eye(3)
        rcom = gp+gr[:,2]*(P.L.DRV_HANDLE_Z0+.040)
        def potential(joints):
            tcp, rot = D.fastik.tcp_pose(joints)
            return P.L.DRV_MASS*9.81*(tcp+rot@rcom)[2]
        expected = []
        for delta in np.eye(6)*1e-6:
            expected.append((potential(q+delta)-potential(q-delta))/2e-6)
        gravity = joint_torque(q, zero, zero, (gp, gr), .016)
        np.testing.assert_allclose(gravity, expected, atol=1e-7)
        mass = np.column_stack([joint_torque(q, zero, accel, (gp, gr), .016)-gravity
                                for accel in np.eye(6)])
        np.testing.assert_allclose(mass, mass.T, atol=1e-9)
        self.assertGreater(min(np.linalg.eigvalsh(mass)), 0.)


class TestCarriedScrewCollision(unittest.TestCase):
    def test_only_the_driver_pair_changes_at_thread_start_and_reset(self):
        try:
            from pxr import Usd, UsdPhysics
        except ImportError:
            self.skipTest("OpenUSD schema is needed to test pair filtering")
        stage = Usd.Stage.CreateInMemory()
        screw = stage.DefinePrim("/World/screw_r", "Xform")
        driver = stage.DefinePrim("/World/driver", "Xform")
        external = [stage.DefinePrim("/World/"+name, "Xform") for name in ("cover", "tub", "finger")]
        for prim in [screw, driver] + external:
            UsdPhysics.RigidBodyAPI.Apply(prim)
        rt = P.Runtime.__new__(P.Runtime)
        rt.screw_pair_filter = UsdPhysics.FilteredPairsAPI.Apply(screw).CreateFilteredPairsRel()
        rt.driver_path = driver.GetPath()
        for carrying in (True, True, False, False, True):
            rt.set_screw_carried(carrying)
            self.assertEqual(rt.screw_pair_filter.GetTargets(), [driver.GetPath()] if carrying else [])
            for prim in external:
                self.assertFalse(prim.HasAPI(UsdPhysics.FilteredPairsAPI))


class TestCoverSupport(unittest.TestCase):
    def test_r7_wrist_pose_is_blocked_by_stand(self):
        from tool_clearance import arm_fixture_clearance, fixture_boxes
        from yam_arena import station

        setup = ("seed", 1415158438)
        boxes = fixture_boxes(P.L.stand_pos(setup), P.L.ry(P.L.LEAN))
        measured = [1.0331, 1.9836, .7995, .5322, 1.5417, -1.3287]
        target = [1.0329, 2.0002, .7878, .5180, 1.5204, -1.2215]
        for pose in (measured, target):
            gap, link, fixture = arm_fixture_clearance(pose, station.BASES["right"], .016, boxes)
            self.assertLess(gap, .004)
            self.assertEqual((link, fixture), ("gripper", "stand backrest"))

    def test_driver_transit_waits_for_lagging_arm(self):
        from unittest.mock import patch
        import demo as D

        c = D.Ctx()
        c.GD = (np.array([0., 0., .1]), np.eye(3))
        arm = c.right
        goal = arm.q.copy()
        goal[5] += .15
        mouth, _, _ = D._driver_of_q(c, arm.q)
        # Hold the measured arm stationary to reproduce loss of tracking during a wrist turn.
        with patch.object(D, "driver_ik", return_value=(goal, 0., 0., 0.)), patch.object(D, "checked_driver_grasp"), patch.object(c, "check_tool_route"):
            transit = D.goto_driver(c, mouth, .05, "driver to screw")
            previous = arm.q.copy()
            with self.assertRaisesRegex(D.Stop, "joint lag"):
                for _ in transit:
                    self.assertLessEqual(max(abs(arm.q-previous)), D.TOOL_CARRY_JOINT_RATE*D.DT+1e-9)
                    previous = arm.q.copy()
        self.assertLess(arm.q[5], goal[5])

    def test_measured_r4_tool_grasp_and_ejection(self):
        import demo as D
        def fit(mouth_mm, axis):
            axis = np.array(axis)
            return D.driver_grasp_fit(np.array(mouth_mm)/1000, -axis/np.linalg.norm(axis))
        radial, axial = fit([.91,-47.93,96.29], [.009,-.4965,.868])
        self.assertLess(radial, P.L.DRV_HANDLE_R)
        self.assertTrue(P.L.DRV_HANDLE_Z0 < axial < P.L.DRV_HANDLE_Z0+P.L.DRV_HANDLE_L)
        radial, axial = fit([-73.41,103.18,-421.9], [-.328,-.808,.4895])
        self.assertGreater(radial, P.L.DRV_HANDLE_R+.002)

    def test_seating_correction_brings_both_edges_closer_to_flush(self):
        import demo as D
        # Measured in panel-r2 after the tub already carried 14.86 N.
        gaps = np.array([.863, .863, -1.265, -1.265])
        top_to_pivot = P.L.COVER_H/2-P.L.GUIDE_Z
        bottom_to_pivot = -P.L.COVER_H/2-P.L.GUIDE_Z
        dx, angle = 0., 0.
        for _ in range(5):
            next_dx, next_angle = D.seating_adjustment(gaps, dx, angle)
            displacement = next_dx-dx + np.array([top_to_pivot]*2+[bottom_to_pivot]*2)*(
                np.sin(np.radians(next_angle))-np.sin(np.radians(angle)))
            gaps += displacement*1000
            dx, angle = next_dx, next_angle
        self.assertLess(max(abs(gaps)), .6)
        for _ in range(100):
            dx, angle = D.seating_adjustment([3., 3., -3., -3.], dx, angle)
        self.assertLessEqual(abs(dx), .002)
        self.assertEqual(angle, -.65)

    def test_joint_bias_matches_virtual_work(self):
        import demo as D
        from yam_arena import station
        from yam_arena.constants import SHOULDER, WRIST

        arm = D.Arm("left", station.BASES["left"])
        pt, rt = arm.measured_tcp()
        centre_in_hand = np.array([.024, -.011, .054])
        centre = pt+rt@centre_in_hand
        load_n = P.L.cover_mass_props()[0]*9.81/2
        arm.support(centre, load_n)
        stiffness = np.array([SHOULDER["stiffness"]]*3+[WRIST["stiffness"]]*3)
        expected = []
        def potential(q):
            p, r = D.fastik.tcp_pose(q, arm.base)
            return load_n*(p+r@centre_in_hand)[2]
        for i in range(6):
            dq = np.eye(6)[i]*1e-6
            expected.append((potential(arm.q+dq)-potential(arm.q-dq))/(2e-6))
        np.testing.assert_allclose(stiffness*arm.payload_offset, expected, atol=1e-7)
        arm.support(centre, 0.)
        np.testing.assert_array_equal(arm.payload_offset, np.zeros(6))
