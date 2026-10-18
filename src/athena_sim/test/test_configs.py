#!/usr/bin/env python3
"""
test_configs.py

Unit tests verifying the structure, syntax, and contents of configuration files:
  - config/bridge.yaml
  - config/nav2_params_sim.yaml
  - config/delivery_waypoints.yaml
  - worlds/sproscape_zones.yaml
"""

import os
import unittest
import yaml


class TestConfigs(unittest.TestCase):

    def setUp(self):
        self.pkg_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        self.bridge_path = os.path.join(self.pkg_dir, 'config', 'bridge.yaml')
        self.nav2_sim_path = os.path.join(self.pkg_dir, 'config', 'nav2_params_sim.yaml')
        self.waypoints_path = os.path.join(self.pkg_dir, 'config', 'delivery_waypoints.yaml')
        self.zones_path = os.path.join(self.pkg_dir, 'worlds', 'sproscape_zones.yaml')

    def test_bridge_yaml_validity(self):
        """Verify bridge.yaml is valid YAML and defines all required ROS <-> Gazebo topics."""
        self.assertTrue(os.path.exists(self.bridge_path))
        with open(self.bridge_path, 'r') as f:
            entries = yaml.safe_load(f)

        self.assertIsInstance(entries, list)
        self.assertGreaterEqual(len(entries), 8)

        ros_topics = [e.get('ros_topic_name') for e in entries]
        self.assertIn('/clock', ros_topics)
        self.assertIn('/cmd_vel', ros_topics)
        self.assertIn('/joint_states', ros_topics)
        self.assertIn('/camera/camera/color/image_raw', ros_topics)
        self.assertIn('/camera/camera/aligned_depth_to_color/image_raw', ros_topics)
        self.assertIn('/camera/camera/depth/color/points', ros_topics)
        self.assertIn('/imu/data', ros_topics)
        self.assertIn('/gps/fix_raw', ros_topics)
        self.assertIn('/sim/odom', ros_topics)

        for entry in entries:
            self.assertIn('ros_topic_name', entry)
            self.assertIn('gz_topic_name', entry)
            self.assertIn('ros_type_name', entry)
            self.assertIn('gz_type_name', entry)
            self.assertIn(entry.get('direction'), ['GZ_TO_ROS', 'ROS_TO_GZ', 'BIDIRECTIONAL'])

    def test_nav2_params_sim_all_nodes_use_sim_time(self):
        """Verify that every node in nav2_params_sim.yaml has use_sim_time: True."""
        self.assertTrue(os.path.exists(self.nav2_sim_path))
        with open(self.nav2_sim_path, 'r') as f:
            content = f.read()

        # There must be no 'use_sim_time: False' remaining
        self.assertNotIn('use_sim_time: False', content)
        self.assertGreaterEqual(content.count('use_sim_time: True'), 11)

    def test_delivery_waypoints_validity(self):
        """Verify delivery_waypoints.yaml defines valid WGS-84 coordinates."""
        self.assertTrue(os.path.exists(self.waypoints_path))
        with open(self.waypoints_path, 'r') as f:
            data = yaml.safe_load(f)

        self.assertIn('waypoints', data)
        wps = data['waypoints']
        self.assertGreaterEqual(len(wps), 3)

        for wp in wps:
            self.assertIn('name', wp)
            self.assertIn('latitude', wp)
            self.assertIn('longitude', wp)
            self.assertGreater(wp['latitude'], 0.0)
            self.assertGreater(wp['longitude'], 0.0)

    def test_sproscape_zones_validity(self):
        """Verify sproscape_zones.yaml contains arena dimensions, delivery markers, and red zones."""
        self.assertTrue(os.path.exists(self.zones_path))
        with open(self.zones_path, 'r') as f:
            data = yaml.safe_load(f)

        self.assertIn('datum', data)
        self.assertIn('delivery_markers', data)
        self.assertIn('red_zones', data)
        self.assertIn('abex_sites', data)


if __name__ == '__main__':
    unittest.main()
