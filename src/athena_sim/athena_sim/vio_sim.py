#!/usr/bin/env python3
"""
vio_sim.py

Option B Visual-Inertial Odometry (VIO) emulator for the Athena rover.
Generates realistic drifting /vio/odometry from ground-truth motion.

Mimics the output of rtabmap rgbd_odometry without requiring GPU image rendering:
  - Linear velocity scaling errors (scale factor bias)
  - Gaussian measurement noise on linear and angular velocities
  - Cumulative yaw random walk drift proportional to distance travelled
  - Featureless terrain dropout simulation (temporary covariance inflation)

Feeds directly into the unchanged vio_gate node in athena_gps_nav.
"""

import math
import random
import time
from typing import Dict, List, Optional, Tuple


class VioErrorModel:
    """
    Pure Python model for Visual-Inertial Odometry drift and measurement noise.
    Independent of ROS 2 runtime for unit testing.
    """

    def __init__(
        self,
        scale_error: float = 1.015,     # 1.5% scale factor bias
        vel_noise_std: float = 0.02,     # 0.02 m/s linear velocity noise
        yaw_rate_noise_std: float = 0.01,# 0.01 rad/s angular velocity noise
        yaw_drift_rate: float = 0.001,   # rad per meter travelled (~0.057 deg/m)
        seed: Optional[int] = None,
    ):
        self.scale_error = scale_error
        self.vel_noise_std = vel_noise_std
        self.yaw_rate_noise_std = yaw_rate_noise_std
        self.yaw_drift_rate = yaw_drift_rate
        self._rng = random.Random(seed)

        # Integrated drifting state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.total_distance = 0.0

        # Terrain feature quality flag (1.0 = normal, 0.0 = completely lost)
        self.feature_quality = 1.0

    def step(self, true_vx: float, true_wz: float, dt: float) -> Tuple[float, float, float, float, float, float, List[float]]:
        """
        Advance VIO integration by dt seconds given true body velocities.

        Returns:
            est_x, est_y, est_yaw, meas_vx, meas_wz, feature_quality, pose_covariance
        """
        if dt <= 0.0:
            dt = 0.02

        # Apply scale error and noise to velocity measurements
        noise_vx = self._rng.gauss(0.0, self.vel_noise_std)
        noise_wz = self._rng.gauss(0.0, self.yaw_rate_noise_std)

        meas_vx = (true_vx * self.scale_error + noise_vx) * self.feature_quality
        meas_wz = (true_wz + noise_wz) * self.feature_quality

        # Distance step
        ds = abs(meas_vx) * dt
        self.total_distance += ds

        # Yaw drift grows with distance travelled plus small random walk
        drift_dyaw = self._rng.gauss(0.0, self.yaw_drift_rate * ds)
        self.yaw += meas_wz * dt + drift_dyaw

        # Integrate forward in drifting estimated frame
        self.x += meas_vx * math.cos(self.yaw) * dt
        self.y += meas_vx * math.sin(self.yaw) * dt

        # Covariance grows with distance and feature degradation
        pos_var = max(0.01, 0.005 * (1.0 + self.total_distance * 0.02)) / max(self.feature_quality, 0.01)
        yaw_var = max(0.005, 0.002 * (1.0 + self.total_distance * 0.01)) / max(self.feature_quality, 0.01)

        pose_cov = [
            pos_var, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, pos_var, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e4, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e4, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e4, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, yaw_var,
        ]

        return self.x, self.y, self.yaw, meas_vx, meas_wz, self.feature_quality, pose_cov

    def set_feature_quality(self, quality: float):
        """Simulate camera tracking loss in featureless sand (0.0 to 1.0)."""
        self.feature_quality = max(0.0, min(1.0, quality))


# ROS 2 Node integration
try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Quaternion, Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float32

    def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * cy - sr * sp * sy
        return q

    class VioSimNode(Node):
        """ROS 2 Node publishing simulated /vio/odometry."""

        def __init__(self):
            super().__init__('vio_sim')

            self.declare_parameter('scale_error', 1.015)
            self.declare_parameter('vel_noise_std', 0.02)
            self.declare_parameter('yaw_rate_noise_std', 0.01)
            self.declare_parameter('yaw_drift_rate', 0.001)

            self.model = VioErrorModel(
                scale_error=self.get_parameter('scale_error').value,
                vel_noise_std=self.get_parameter('vel_noise_std').value,
                yaw_rate_noise_std=self.get_parameter('yaw_rate_noise_std').value,
                yaw_drift_rate=self.get_parameter('yaw_drift_rate').value,
            )

            self.last_time = self.get_clock().now()
            self.current_vx = 0.0
            self.current_wz = 0.0

            # Subscriptions
            self.sub_odom = self.create_subscription(
                Odometry, '/sim/odom', self._odom_cb, 10
            )
            self.sub_cmd = self.create_subscription(
                Twist, '/cmd_vel', self._cmd_cb, 10
            )
            self.sub_quality = self.create_subscription(
                Float32, '/sim/vio_quality', self._quality_cb, 10
            )

            # Publisher
            self.pub_vio = self.create_publisher(Odometry, '/vio/odometry', 10)

            # Periodic update timer at 20 Hz
            self.timer = self.create_timer(0.05, self._tick)
            self.get_logger().info('VIO Simulator node running at 20 Hz')

        def _odom_cb(self, msg: Odometry):
            # Read ground-truth body velocities from Gazebo diff-drive
            self.current_vx = msg.twist.twist.linear.x
            self.current_wz = msg.twist.twist.angular.z

        def _cmd_cb(self, msg: Twist):
            # Fallback kinematics if Gazebo odom not yet received
            if self.current_vx == 0.0 and self.current_wz == 0.0:
                self.current_vx = msg.linear.x
                self.current_wz = msg.angular.z

        def _quality_cb(self, msg: Float32):
            self.model.set_feature_quality(msg.data)

        def _tick(self):
            now = self.get_clock().now()
            dt = (now - self.last_time).nanoseconds * 1e-9
            self.last_time = now

            x, y, yaw, vx, wz, qual, cov = self.model.step(
                self.current_vx, self.current_wz, dt
            )

            msg = Odometry()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = 'odom'
            msg.child_frame_id = 'base_link'

            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            msg.pose.pose.position.z = 0.0
            msg.pose.pose.orientation = euler_to_quaternion(0.0, 0.0, yaw)
            msg.pose.covariance = cov

            msg.twist.twist.linear.x = vx
            msg.twist.twist.linear.y = 0.0
            msg.twist.twist.linear.z = 0.0
            msg.twist.twist.angular.z = wz
            msg.twist.covariance = [
                0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1e4, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1e4, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1e4, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.01,
            ]

            self.pub_vio.publish(msg)

    def main(args=None):
        rclpy.init(args=args)
        node = VioSimNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

except ImportError:
    def main(args=None):
        print("vio_sim ROS 2 node requires rclpy, geometry_msgs, and nav_msgs.")
