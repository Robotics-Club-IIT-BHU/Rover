#!/usr/bin/env python3
"""
pointcloud_downsampler.py

Subscribes to /camera/camera/depth/color/points, keeps every Nth point
(N = 'scarcity' parameter, default 5), and republishes on
/camera/camera/depth/color/points_downsampled.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2


class PointCloudDownsampler(Node):
    def __init__(self):
        super().__init__('pointcloud_downsampler')

        # Declare the 'scarcity' parameter
        self.declare_parameter('scarcity', 30)

        # Declare the 'max_distance' parameter (meters)
        self.declare_parameter('max_distance', 1.5)

        # RealSense publishes best-effort, so we match that QoS on the sub.
        # Downsampled output is republished reliable so RViz doesn't drop it.
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.cloud_callback,
            sub_qos,
        )

        self.pub = self.create_publisher(
            PointCloud2,
            '/camera/camera/depth/color/points_downsampled',
            pub_qos,
        )

        self.get_logger().info(
            "PointCloud downsampler started. "
            "Subscribed to /camera/camera/depth/color/points, "
            "publishing to /camera/camera/depth/color/points_downsampled "
            "(scarcity=%d, max_distance=%.2fm by default)"
            % (
                self.get_parameter('scarcity').value,
                self.get_parameter('max_distance').value,
            )
        )

    def cloud_callback(self, msg: PointCloud2):
        #scarcity = self.get_parameter('scarcity').get_parameter_value().integer_value
        scarcity = 30
        if scarcity < 1:
            scarcity = 1

        # max_distance = self.get_parameter('max_distance').get_parameter_value().double_value
        max_distance = 1.5

        point_step = msg.point_step

        # View the raw byte buffer as one row per point (works for organized
        # RealSense clouds where row_step == width * point_step, i.e. no
        # row padding, which is the standard case for this driver).
        points = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, point_step)

        # Look up the byte offset of the x/y/z fields rather than hardcoding
        # them, so this keeps working even if the field order/layout changes.
        offsets = {f.name: f.offset for f in msg.fields}
        x_off, y_off, z_off = offsets['x'], offsets['y'], offsets['z']

        x = points[:, x_off:x_off + 4].copy().view('<f4').reshape(-1)
        y = points[:, y_off:y_off + 4].copy().view('<f4').reshape(-1)
        z = points[:, z_off:z_off + 4].copy().view('<f4').reshape(-1)

        distance = np.sqrt(x * x + y * y + z * z)

        # Keep only finite, in-range points (drops NaN/invalid depth too)
        valid = np.isfinite(distance) & (distance <= max_distance)
        filtered = points[valid]

        # Keep every Nth point of what's left
        downsampled = filtered[::scarcity]

        new_msg = PointCloud2()
        new_msg.header = msg.header
        new_msg.height = 1
        new_msg.width = downsampled.shape[0]
        new_msg.fields = msg.fields
        new_msg.is_bigendian = msg.is_bigendian
        new_msg.point_step = point_step
        new_msg.row_step = point_step * downsampled.shape[0]
        new_msg.is_dense = msg.is_dense
        new_msg.data = downsampled.tobytes()

        self.pub.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudDownsampler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
