#!/usr/bin/env python3

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


class PointCloudVisualizer(Node):
    def __init__(self):
        super().__init__('pointcloud_visualizer')

        self.declare_parameter('window_size', 480)
        self.declare_parameter('scale', 100.0)
        self.declare_parameter('topic', '/camera/camera/depth/color/points_coordinates')

        self.size = self.get_parameter('window_size').get_parameter_value().integer_value
        self.scale = self.get_parameter('scale').get_parameter_value().double_value
        topic = self.get_parameter('topic').get_parameter_value().string_value

        self.latest_img = np.zeros((self.size, self.size, 3), dtype=np.uint8)
        self.last_msg_count = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub = self.create_subscription(
            PointCloud2,
            topic,
            self.cloud_callback,
            qos,
        )

        cv2.namedWindow("Point Cloud (z=0 projection)", cv2.WINDOW_NORMAL)
        cv2.imshow("Point Cloud (z=0 projection)", self.latest_img)
        cv2.waitKey(1)

        self.timer = self.create_timer(0.03, self.display_callback)
        self.watchdog = self.create_timer(3.0, self.watchdog_callback)

        self.get_logger().info(f"Subscribed to {topic}")
        self.get_logger().info("Press 'q' in the OpenCV window to quit.")

    def cloud_callback(self, msg: PointCloud2):
        self.last_msg_count += 1

        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array(list(points), dtype=np.float32)

        img = np.zeros((self.size, self.size, 3), dtype=np.uint8)

        if points.size == 0:
            self.latest_img = img
            return

        xs = points[:, 0]
        ys = points[:, 1]

        cx, cy = self.size // 2, self.size // 2
        px = (cx + xs * self.scale).astype(np.int32)
        py = (cy + ys * self.scale).astype(np.int32)

        valid = (px >= 0) & (px < self.size) & (py >= 0) & (py < self.size)
        px, py = px[valid], py[valid]

        if px.size > 0:
            img[py, px] = (0, 255, 0)

        self.latest_img = img

    def watchdog_callback(self):
        if self.last_msg_count == 0:
            self.get_logger().warn("No messages received yet on this topic.")
        self.last_msg_count = 0

    def display_callback(self):
        cv2.imshow("Point Cloud (z=0 projection)", self.latest_img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Quit requested from OpenCV window.")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
