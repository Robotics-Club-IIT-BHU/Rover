#!/usr/bin/env python3
"""
coordinate_extractor.py
Subscribes to /camera/camera/depth/color/points_downsampled (PointCloud2,
raw bytes), decodes x/y/z, drops non-finite points, and republishes as a
clean PointCloud2 on /camera/camera/depth/color/points_coordinates.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class CoordinateExtractor(Node):
    def __init__(self):
        super().__init__('coordinate_extractor')
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points_downsampled',
            self.cloud_callback,
            qos,
        )
        self.pub = self.create_publisher(
            PointCloud2,
            '/camera/camera/depth/color/points_coordinates',
            qos,
        )
        self.get_logger().info(
            "Coordinate extractor started. "
            "Subscribed to /camera/camera/depth/color/points_downsampled, "
            "publishing to /camera/camera/depth/color/points_coordinates"
        )

    def cloud_callback(self, msg: PointCloud2):
        point_step = msg.point_step
        points = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, point_step)
        offsets = {f.name: f.offset for f in msg.fields}
        x_off, y_off, z_off = offsets['x'], offsets['y'], offsets['z']
        x = points[:, x_off:x_off + 4].copy().view('<f4').reshape(-1)
        y = points[:, y_off:y_off + 4].copy().view('<f4').reshape(-1)
        z = points[:, z_off:z_off + 4].copy().view('<f4').reshape(-1)

        # Drop any remaining non-finite points (NaN/inf depth)
        valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        x, y, z = x[valid], y[valid], z[valid]

        xyz = np.column_stack((x, y, z)).astype(np.float32)

        cloud_msg = point_cloud2.create_cloud_xyz32(msg.header, xyz)
        self.pub.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CoordinateExtractor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
