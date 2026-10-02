#!/usr/bin/env python3
"""
costmap_drift_check.py

Quantifies "the costmap is drifting": with the robot and scene stationary,
the occupied-cell centroid expressed in the ROBOT frame must stay constant.
Any movement of that relative centroid is exactly the drift you see in
RViz (obstacles sliding/rotating around the robot).

    ros2 run athena_gps_nav costmap_drift_check --ros-args \
        -p duration:=120.0 -p costmap_topic:=/local_costmap/costmap

Subscribes to the costmap named by `costmap_topic` (nav_msgs/OccupancyGrid)
and reads TF; publishes nothing, prints a report and exits.

Reports centroid drift (cm and cm/min) and the number of occupied cells
over time.  Run one instance per costmap (local: odom frame; global: map
frame. Expect the global one to move indoors, GPS multipath moves the
map frame itself).
"""

import math
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid

import tf2_ros

# Below this the centroid is dominated by whichever handful of cells
# happened to be marked, so it says nothing about drift.
MIN_OCCUPIED_CELLS = 5

# Compare the mean of the first and last few samples rather than single
# frames, so one noisy costmap update cannot masquerade as drift.
EDGE_SAMPLES = 5


class CostmapDriftCheck(Node):
    def __init__(self):
        super().__init__('costmap_drift_check')
        self.declare_parameter('costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('duration', 120.0)
        self.declare_parameter('occupied_threshold', 65)

        self.topic = self.get_parameter('costmap_topic').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.duration = float(self.get_parameter('duration').value)
        self.occ_thresh = int(self.get_parameter('occupied_threshold').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.samples = []   # (t, rel_cx, rel_cy, n_occupied)
        self.t0 = time.time()

        self.create_subscription(OccupancyGrid, self.topic, self._cb, 2)
        self.create_timer(self.duration, self._finish)
        self._done = False
        self.get_logger().info(
            f'Watching {self.topic} for {self.duration:.0f} s ...')

    def _cb(self, grid: OccupancyGrid):
        res = grid.info.resolution
        ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
        w = grid.info.width

        sx = sy = 0.0
        n = 0
        for i, v in enumerate(grid.data):
            if v >= self.occ_thresh:
                sx += ox + ((i % w) + 0.5) * res
                sy += oy + ((i // w) + 0.5) * res
                n += 1
        if n < MIN_OCCUPIED_CELLS:
            return
        cx, cy = sx / n, sy / n           # centroid in costmap frame

        try:
            # non-blocking: latest available transform (a blocking lookup
            # inside the callback starves the executor)
            tf = self.tf_buffer.lookup_transform(
                grid.header.frame_id, self.robot_frame, rclpy.time.Time())
        except Exception:
            return
        rx = tf.transform.translation.x
        ry = tf.transform.translation.y
        q = tf.transform.rotation
        ryaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # centroid expressed in the robot frame
        dx, dy = cx - rx, cy - ry
        rel_x = math.cos(-ryaw) * dx - math.sin(-ryaw) * dy
        rel_y = math.sin(-ryaw) * dx + math.cos(-ryaw) * dy

        self.samples.append((time.time() - self.t0, rel_x, rel_y, n))
        if time.time() - self.t0 > self.duration:
            self._finish()

    def _finish(self):
        if self._done:
            return
        self._done = True
        s = self.samples
        print('=' * 56, flush=True)
        print(f'costmap drift check  {self.topic}', flush=True)
        if len(s) < EDGE_SAMPLES:
            print(f'  not enough samples ({len(s)}). Is the costmap '
                  'publishing and TF available?')
            rclpy.shutdown()
            return
        n = float(EDGE_SAMPLES)
        x0 = sum(r[1] for r in s[:EDGE_SAMPLES]) / n
        y0 = sum(r[2] for r in s[:EDGE_SAMPLES]) / n
        x1 = sum(r[1] for r in s[-EDGE_SAMPLES:]) / n
        y1 = sum(r[2] for r in s[-EDGE_SAMPLES:]) / n
        net = math.hypot(x1 - x0, y1 - y0)
        peak = max(math.hypot(r[1] - x0, r[2] - y0) for r in s)
        window = s[-1][0] - s[0][0]
        print(f'  samples          : {len(s)} over {window:.0f} s', flush=True)
        print(f'  occupied cells   : {s[0][3]} -> {s[-1][3]}', flush=True)
        print(f'  centroid rel to {self.robot_frame}:', flush=True)
        print(f'    net drift      : {net * 100:6.1f} cm '
              f'({net * 100 / max(window, 1) * 60:5.1f} cm/min)')
        print(f'    peak deviation : {peak * 100:6.1f} cm', flush=True)
        print('=' * 56, flush=True)
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CostmapDriftCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._finish()


if __name__ == '__main__':
    main()
