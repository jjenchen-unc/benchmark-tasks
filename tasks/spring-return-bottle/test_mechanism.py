"""Checks of the passive return mechanism, independent of the simulator."""
import unittest
import xml.etree.ElementTree as ET
from spring_bottle.bottle_model import scene_xml,LOOP_STIFFNESS,LOOP_DAMPING


class MechanismTests(unittest.TestCase):
    def test_loop_is_a_zero_rest_passive_spring(self):
        root=ET.fromstring(scene_xml());joint=root.find(".//joint[@name='catch_hinge']")
        self.assertGreater(float(joint.get('stiffness')),0)
        self.assertEqual(float(joint.get('springref')),0)
        self.assertEqual(float(joint.get('stiffness')),LOOP_STIFFNESS)
        self.assertEqual(float(joint.get('damping')),LOOP_DAMPING)
        self.assertIsNone(root.find('actuator'))

    def test_original_loop_and_body_geometry_are_retained(self):
        root=ET.fromstring(scene_xml());body=root.find("worldbody/body[@name='bottle']")
        self.assertEqual(len(body.findall('body')),3)
        loop=body.find("body[@name='safety_catch']")
        self.assertEqual(len([g for g in loop.findall('geom') if g.get('name').startswith('catch_arc_')]),18)
        self.assertFalse(any('handle' in g.get('name','') for g in root.findall('.//geom')))


if __name__=='__main__':unittest.main()
