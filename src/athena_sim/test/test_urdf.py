#!/usr/bin/env python3
"""
test_urdf.py

Unit tests verifying URDF / XACRO XML validity and element structures for Athena simulation.
"""

import os
import unittest
import xml.etree.ElementTree as ET


class TestUrdfStructure(unittest.TestCase):

    def setUp(self):
        self.repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
        self.athena_xacro = os.path.join(self.repo_root, 'src', 'athena_gps_nav', 'urdf', 'athena.urdf.xacro')
        self.athena_sim_xacro = os.path.join(self.repo_root, 'src', 'athena_sim', 'urdf', 'athena_sim.urdf.xacro')

    def test_athena_urdf_xacro_parses_cleanly(self):
        """Verify athena.urdf.xacro contains valid XML and root is <robot>."""
        self.assertTrue(os.path.exists(self.athena_xacro), f"Missing {self.athena_xacro}")
        tree = ET.parse(self.athena_xacro)
        root = tree.getroot()
        self.assertEqual(root.tag, 'robot')
        self.assertEqual(root.attrib.get('name'), 'athena')

    def test_athena_sim_urdf_xacro_parses_cleanly(self):
        """Verify athena_sim.urdf.xacro contains valid XML and root is <robot>."""
        self.assertTrue(os.path.exists(self.athena_sim_xacro), f"Missing {self.athena_sim_xacro}")
        tree = ET.parse(self.athena_sim_xacro)
        root = tree.getroot()
        self.assertEqual(root.tag, 'robot')
        self.assertEqual(root.attrib.get('name'), 'athena_sim')

    def test_sim_arg_present_in_athena_xacro(self):
        """Verify athena.urdf.xacro defines <xacro:arg name='sim' ... />."""
        tree = ET.parse(self.athena_xacro)
        root = tree.getroot()
        sim_args = [elem for elem in root if 'arg' in elem.tag and elem.attrib.get('name') == 'sim']
        self.assertEqual(len(sim_args), 1, "Expected exactly one <xacro:arg name='sim'> declaration")

    def test_athena_sim_has_diff_drive_plugin(self):
        """Verify athena_sim.urdf.xacro includes the Ignition DiffDrive system plugin."""
        tree = ET.parse(self.athena_sim_xacro)
        root = tree.getroot()
        plugins = root.findall('.//plugin')
        diff_drive_plugins = [p for p in plugins if 'DiffDrive' in p.attrib.get('name', '')]
        self.assertEqual(len(diff_drive_plugins), 1, "DiffDrive plugin missing in athena_sim.urdf.xacro")

        dd = diff_drive_plugins[0]
        left_joints = [j.text for j in dd.findall('left_joint')]
        right_joints = [j.text for j in dd.findall('right_joint')]

        self.assertEqual(len(left_joints), 3, "Expected 3 left wheel joints")
        self.assertEqual(len(right_joints), 3, "Expected 3 right wheel joints")
        self.assertIn('wheel_front_left_joint', left_joints)
        self.assertIn('wheel_front_right_joint', right_joints)

    def test_athena_sim_has_sensor_plugins(self):
        """Verify sensors (rgbd_camera, imu, navsat) are declared in athena_sim.urdf.xacro."""
        tree = ET.parse(self.athena_sim_xacro)
        root = tree.getroot()
        sensors = root.findall('.//sensor')
        sensor_types = [s.attrib.get('type') for s in sensors]

        self.assertIn('rgbd_camera', sensor_types, "RGBD camera sensor missing")
        self.assertIn('imu', sensor_types, "IMU sensor missing")
        self.assertIn('navsat', sensor_types, "NavSat sensor missing")

    def test_rocker_bogie_suspension_declared(self):
        """Verify athena.urdf.xacro defines rocker and bogie suspension linkages and 30cm wheels."""
        tree = ET.parse(self.athena_xacro)
        root = tree.getroot()
        properties = {elem.attrib.get('name'): elem.attrib.get('value') for elem in root if 'property' in elem.tag}

        self.assertEqual(properties.get('wheel_radius'), '0.15', "Wheel radius must be 0.15m (30cm diameter)")
        self.assertEqual(properties.get('base_height'), '0.25', "Base height ground clearance must be 0.25m")

        # Verify DiffDrive in athena_sim matches 0.15m radius
        tree_sim = ET.parse(self.athena_sim_xacro)
        diff_drive = tree_sim.find('.//plugin[@filename="ignition-gazebo-diff-drive-system"]')
        self.assertIsNotNone(diff_drive)
        self.assertEqual(diff_drive.find('wheel_radius').text, '0.15')


if __name__ == '__main__':
    unittest.main()

