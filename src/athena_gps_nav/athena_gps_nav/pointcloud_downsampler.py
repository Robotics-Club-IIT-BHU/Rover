#!/usr/bin/env python3
"""
pointcloud_downsampler.py

Range-filters and stride-downsamples the RealSense depth cloud so the Nav2
voxel costmap layer gets a light-weight cloud.

    /camera/camera/depth/color/points              sensor_msgs/PointCloud2  in
    /camera/camera/depth/color/points_downsampled  sensor_msgs/PointCloud2  out

Both topics are parameters, so the node can be pointed at another camera.
Started by sensors.launch.py; standalone:

    ros2 run athena_gps_nav pointcloud_downsampler

Based on athena_slam/point_cloud_filter.py with the parameter-handling bug
fixed (the original hard-coded scarcity=30 / max_distance=1.5 and ignored its
own parameters; 1.5 m is also too short for obstacle marking at 3 m).
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from sensor_msgs.msg import PointCloud2

# Best-effort matches what the RealSense driver publishes.  Subscribing
# RELIABLE to a BEST_EFFORT publisher is a silent QoS mismatch: the
# subscription is created without error and simply never receives anything.
CLOUD_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST, depth=5,
    durability=DurabilityPolicy.VOLATILE)


class PointCloudDownsampler(Node):
    def __init__(self):
        super().__init__('pointcloud_downsampler')
        self.declare_parameter('scarcity', 8)          # keep every Nth point
        self.declare_parameter('max_distance', 3.5)     # m
        self.declare_parameter('min_distance', 0.25)    # m, drop self-hits
        # The camera's cloud already arrives slowly (~5 Hz after decimation),
        # so don't skip frames: obstacle marking needs every one of them.
        self.declare_parameter('frame_skip', 1)         # process every Nth cloud
        self._frame_count = 0
        self.declare_parameter(
            'input_topic', '/camera/camera/depth/color/points')
        self.declare_parameter(
            'output_topic', '/camera/camera/depth/color/points_downsampled')

        self.sub = self.create_subscription(
            PointCloud2, self.get_parameter('input_topic').value,
            self.cloud_callback, CLOUD_QOS)
        self.pub = self.create_publisher(
            PointCloud2, self.get_parameter('output_topic').value, CLOUD_QOS)

        self.get_logger().info(
            'Downsampler: %s -> %s (scarcity=%d, range=[%.2f, %.2f] m)' % (
                self.get_parameter('input_topic').value,
                self.get_parameter('output_topic').value,
                self.get_parameter('scarcity').value,
                self.get_parameter('min_distance').value,
                self.get_parameter('max_distance').value))

    def cloud_callback(self, msg: PointCloud2):
        # Parameters are read per message, not cached, so `ros2 param set`
        # retunes the filter on a running rover without a restart.
        self._frame_count += 1
        # the costmap updates at 5 Hz; no point burning Jetson CPU on 30 Hz
        if self._frame_count % max(1, self.get_parameter('frame_skip').value):
            return
        scarcity = max(1, self.get_parameter('scarcity').value)
        max_d = self.get_parameter('max_distance').value
        min_d = self.get_parameter('min_distance').value

        point_step = msg.point_step
        points = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, point_step)

        offsets = {f.name: f.offset for f in msg.fields}
        x = points[:, offsets['x']:offsets['x'] + 4].copy().view('<f4').reshape(-1)
        y = points[:, offsets['y']:offsets['y'] + 4].copy().view('<f4').reshape(-1)
        z = points[:, offsets['z']:offsets['z'] + 4].copy().view('<f4').reshape(-1)

        dist = np.sqrt(x * x + y * y + z * z)
        valid = np.isfinite(dist) & (dist <= max_d) & (dist >= min_d)
        downsampled = points[valid][::scarcity]

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = downsampled.shape[0]
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = point_step
        out.row_step = point_step * downsampled.shape[0]
        out.is_dense = True
        out.data = downsampled.tobytes()
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudDownsampler()
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
