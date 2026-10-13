"""test_goal_manager.py: tests for goal_manager frame transformations and auto-fixing."""
import math
import pytest
from unittest.mock import MagicMock

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from athena_remote.goal_manager import GoalManager, yaw_to_quat, compass_deg_to_enu_rad


def test_yaw_to_quat():
    # 0 yaw -> (0, 0, 0, 1)
    q = yaw_to_quat(0.0)
    assert q == (0.0, 0.0, 0.0, 1.0)
    # pi yaw -> (0, 0, 1, 0)
    q_pi = yaw_to_quat(math.pi)
    assert pytest.approx(q_pi[2], abs=1e-5) == 1.0
    assert pytest.approx(q_pi[3], abs=1e-5) == 0.0


def test_compass_deg_to_enu_rad():
    # North (0 deg) -> East + 90 deg (pi/2)
    assert pytest.approx(compass_deg_to_enu_rad(0.0), abs=1e-5) == math.pi / 2.0
    # East (90 deg) -> 0 rad
    assert pytest.approx(compass_deg_to_enu_rad(90.0), abs=1e-5) == 0.0
    # South (180 deg) -> -pi/2
    assert pytest.approx(compass_deg_to_enu_rad(180.0), abs=1e-5) == -math.pi / 2.0


def test_auto_fix_goal_pose_ignores_map():
    rclpy.init()
    try:
        node = GoalManager()
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = 5.0
        msg.pose.position.y = 2.0
        node._send_nav_goal = MagicMock()
        node._goal_pose_cb(msg)
        # Should not re-send or intercept map-frame goal
        node._send_nav_goal.assert_not_called()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_auto_fix_goal_pose_nan_rejected():
    rclpy.init()
    try:
        node = GoalManager()
        msg = PoseStamped()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = float('nan')
        msg.pose.position.y = 2.0
        node._send_nav_goal = MagicMock()
        node._say = MagicMock()
        node._goal_pose_cb(msg)
        node._send_nav_goal.assert_not_called()
        node._say.assert_called_with('refused /goal_pose: NaN coordinates')
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_gps_no_fix_refusal():
    rclpy.init()
    try:
        node = GoalManager()
        node.have_fix = False
        node.require_fix = True
        node._say = MagicMock()
        node._go_to_ll(25.26, 82.98, None)
        assert any('waiting for valid 3D/RTK satellite fix' in call[0][0] for call in node._say.call_args_list)
    finally:
        node.destroy_node()
        rclpy.shutdown()
