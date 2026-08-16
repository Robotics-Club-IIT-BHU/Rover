#!/usr/bin/env python3
"""
ROS2 node that subscribes to /camera/camera/depth/color/points_coordinates
(sensor_msgs/PointCloud2) and plots a top-down (bird's-eye) view of the
points in an OpenCV window. Points outside a +/- y_limit horizontal slab
are discarded; the surviving points are plotted by their x/z coordinates.
"""
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class PointPlotterNode(Node):
    def __init__(self):
        super().__init__('point_plotter_node')

        # --- Parameters ---
        self.declare_parameter('width', 800)
        self.declare_parameter('height', 800)
        self.declare_parameter('point_radius', 2)
        # pixels per metre -> tune to how "zoomed in" you want the plot
        self.declare_parameter('scale', 100.0)
        # keep only points within +/- this many metres in the camera y axis
        self.declare_parameter('y_limit', 1.0)

        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.point_radius = self.get_parameter('point_radius').value
        self.scale = self.get_parameter('scale').value
        self.y_limit = self.get_parameter('y_limit').value

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points_coordinates',
            self.cloud_callback,
            qos,
        )

        self.window_name = 'Point Plotter'
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)

        self.get_logger().info(
            'PointPlotterNode started. Subscribed to '
            '/camera/camera/depth/color/points_coordinates'
        )

    def cloud_callback(self, msg: PointCloud2):
        # White background, redrawn fresh each message
        frame = np.full((self.height, self.width, 3), 255, dtype=np.uint8)

        points = point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True
        )
        if points.shape[0] > 0:
            x = points['x']
            y = points['y']
            z = points['z']

            # Keep only the horizontal slab within +/- y_limit of the camera.
            slab = np.abs(y) <= self.y_limit
            x, z = x[slab], z[slab]

            # Top view: camera x -> image column (centred),
            # camera z (depth) -> image row, growing upward from the bottom
            # edge so the camera sits at bottom-centre.
            px = (x * self.scale + self.width / 2.0).astype(np.int32)
            pz = (self.height - 1 - z * self.scale).astype(np.int32)

            in_bounds = (
                (px >= 0) & (px < self.width) &
                (pz >= 0) & (pz < self.height)
            )
            px, pz = px[in_bounds], pz[in_bounds]

            frame[pz, px] = (0, 0, 0)
            self.get_logger().debug(f'Plotted {len(px)} points')

        self.draw_reference(frame)

        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)

    def draw_reference(self, frame):
        """Range rings every metre plus a marker at the camera origin."""
        cx = self.width // 2
        cy = self.height - 1
        max_range = int(self.height / self.scale) + 1
        for r in range(1, max_range):
            cv2.circle(frame, (cx, cy), int(r * self.scale),
                       (200, 200, 200), 1)
        cv2.line(frame, (cx, 0), (cx, cy), (200, 200, 200), 1)
        cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PointPlotterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
