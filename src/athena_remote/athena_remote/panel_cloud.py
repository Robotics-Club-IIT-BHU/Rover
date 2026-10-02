#!/usr/bin/env python3
"""
panel_cloud.py, a thinned copy of the depth cloud for the Foxglove 3D panel.

The panel used to show /camera/camera/depth/color/points_downsampled, the
cloud the costmaps consume: about 4,850 points of 16 bytes at 11-12 Hz,
measured 2026-10-02 at 78 KB a message and 0.9 MB/s through the bridge.
Once the raw camera image was replaced by panel_camera's JPEG, that cloud
was about 80 % of everything the panel pulled down the SSH tunnel.

This node publishes /athena/points_preview: the same points thinned on a
voxel grid (one point per `voxel_size` cube, so walls and obstacles keep
their shape instead of losing random points), capped at `max_points`, x/y/z
only (12 bytes a point), at `rate_hz`.  The costmaps keep the full cloud;
nothing here touches obstacle detection.

Like panel_camera it costs nothing while nobody is watching: it subscribes
to the input only while something subscribes to the output (checked once a
second), and frames it skips are dropped as raw bytes, never deserialized.
"""

import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.serialization import deserialize_message

from sensor_msgs.msg import PointCloud2, PointField

# The downsampler publishes best effort; only the newest cloud matters.
IN_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                    history=HistoryPolicy.KEEP_LAST, depth=1)
OUT_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=1)

OUT_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]


class PanelCloud(Node):

    def __init__(self):
        super().__init__('panel_cloud')

        self.declare_parameter('input_topic',
                               '/camera/camera/depth/color/points_downsampled')
        self.declare_parameter('output_topic', '/athena/points_preview')
        self.declare_parameter('rate_hz', 2.0)
        self.declare_parameter('voxel_size', 0.10)     # m; 0 = no voxel thinning
        self.declare_parameter('max_points', 2000)     # 0 = no cap

        self.in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.period = 1.0 / max(0.1, float(self.get_parameter('rate_hz').value))
        self.voxel = max(0.0, float(self.get_parameter('voxel_size').value))
        self.max_points = max(0, int(self.get_parameter('max_points').value))

        self.pub = self.create_publisher(PointCloud2, out_topic, OUT_QOS)
        self.sub = None
        self._next = 0.0

        self.create_timer(1.0, self._check_demand)
        self.get_logger().info(
            f'{self.in_topic} -> {out_topic}: voxel {self.voxel:.2f} m, '
            f'max {self.max_points or "unlimited"} points, '
            f'{1.0 / self.period:.1f} Hz; idle until something subscribes')

    # ------------------------------------------------------------- demand
    def _check_demand(self):
        wanted = self.pub.get_subscription_count() > 0
        if wanted and self.sub is None:
            self.sub = self.create_subscription(
                PointCloud2, self.in_topic, self._cloud_cb, IN_QOS, raw=True)
            self.get_logger().info(f'viewer connected, reading {self.in_topic}')
        elif not wanted and self.sub is not None:
            self.destroy_subscription(self.sub)
            self.sub = None
            self.get_logger().info(f'no viewer, released {self.in_topic}')

    # ------------------------------------------------------------- clouds
    def _cloud_cb(self, raw):
        now = time.monotonic()
        if now < self._next:
            return                       # throttled: dropped as raw bytes
        self._next = max(self._next + self.period, now)

        msg = deserialize_message(raw, PointCloud2)
        xyz = self._xyz(msg)
        if xyz is None:
            return
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if self.voxel > 0.0 and len(xyz):
            keys = np.floor(xyz / self.voxel).astype(np.int32)
            _, first = np.unique(keys, axis=0, return_index=True)
            xyz = xyz[np.sort(first)]
        if self.max_points and len(xyz) > self.max_points:
            step = int(np.ceil(len(xyz) / self.max_points))
            xyz = xyz[::step]

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = len(xyz)
        out.fields = OUT_FIELDS
        out.is_bigendian = False
        out.point_step = 12
        out.row_step = 12 * len(xyz)
        out.is_dense = True
        out.data = np.ascontiguousarray(xyz, dtype='<f4').tobytes()
        self.pub.publish(out)

    def _xyz(self, msg):
        """(N, 3) float32 array of x, y, z, whatever else the points carry."""
        offs = {f.name: f.offset for f in msg.fields
                if f.datatype == PointField.FLOAT32 and f.name in ('x', 'y', 'z')}
        if len(offs) != 3 or msg.is_bigendian:
            self.get_logger().warn(
                'input cloud needs little-endian float32 x/y/z fields',
                throttle_duration_sec=30.0)
            return None
        n = msg.width * msg.height
        dtype = np.dtype({'names': ['x', 'y', 'z'],
                          'formats': ['<f4'] * 3,
                          'offsets': [offs['x'], offs['y'], offs['z']],
                          'itemsize': msg.point_step})
        pts = np.frombuffer(bytes(msg.data), dtype=dtype, count=n)
        return np.stack([pts['x'], pts['y'], pts['z']], axis=1)


def main(args=None):
    rclpy.init(args=args)
    node = PanelCloud()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
