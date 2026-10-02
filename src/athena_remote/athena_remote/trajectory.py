#!/usr/bin/env python3
"""
trajectory.py, the breadcrumb trail of where the rover has actually been.

Nav2 publishes /plan, which is where the rover INTENDS to go.  Nothing in the
stack publishes where it HAS gone, and that is the line an operator watching a
run actually wants: it shows drift, it shows whether a turn was taken wide,
and it shows the shape of the route without needing a rosbag.

Two views of the same trail, because the two panels speak different languages:
    /athena/trajectory          nav_msgs/Path          -> 3D panel / RViz
    /athena/trajectory_geojson  foxglove_msgs/GeoJSON  -> satellite map panel

Distance-gated sampling
    A point is appended only after the rover has moved min_distance since the
    last one.  Sampling on a timer instead would pile thousands of identical
    points on top of each other while parked, and a Path with 50k coincident
    points is both a bandwidth problem over WiFi and a rendering problem in the
    browser.  Gating on distance makes the cost proportional to ground covered
    rather than to time switched on.

Author: Jashan
"""

import json
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from foxglove_msgs.msg import GeoJSON
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Empty

# Transient-local so a Foxglove panel that connects late still gets the trail
# immediately, instead of staying blank until the rover next moves.
LATCHED = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)

EARTH_RADIUS_M = 6371000.0

# Map-panel styling.  Green rover with a fix, red without, so a position
# shown from a NO_FIX message cannot be mistaken for a working GPS.
TRAIL_COLOR = '#48dbfb'
TRAIL_WIDTH = 4
ROVER_FIX_COLOR = '#1dd1a1'
ROVER_NOFIX_COLOR = '#ff5f56'


class Trajectory(Node):

    def __init__(self):
        super().__init__('trajectory')

        self.declare_parameter('odom_topic', '/odometry/global')
        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('min_distance', 0.05)     # m between samples
        self.declare_parameter('min_gps_distance', 0.25)  # m between geo samples
        self.declare_parameter('max_points', 4000)
        self.declare_parameter('publish_period', 0.5)

        self.min_d = float(self.get_parameter('min_distance').value)
        self.min_gd = float(self.get_parameter('min_gps_distance').value)
        self.max_points = int(self.get_parameter('max_points').value)

        self.path = Path()
        self.geo = []          # list of [lon, lat]
        self._last_xy = None
        self._last_ll = None
        self._latest = None    # (lon, lat, status) of the most recent message

        self.path_pub = self.create_publisher(Path, '/athena/trajectory', 10)
        self.geo_pub = self.create_publisher(
            GeoJSON, '/athena/trajectory_geojson', LATCHED)

        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value, self._odom_cb, 10)
        self.create_subscription(
            NavSatFix, self.get_parameter('gps_topic').value, self._gps_cb, 10)
        self.create_subscription(Empty, '/athena/trajectory_clear', self._clear_cb, 10)

        self.create_timer(
            float(self.get_parameter('publish_period').value), self._publish)

        self.get_logger().info(
            f'tracing {self.get_parameter("odom_topic").value} '
            f'every {self.min_d} m (max {self.max_points} points)')

    # ------------------------------------------------------------- inputs
    def _odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._last_xy is not None and math.dist((x, y), self._last_xy) < self.min_d:
            return
        self._last_xy = (x, y)

        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.header.frame_id = msg.header.frame_id
        self.path.poses.append(pose)

        # Oldest-first trim: the recent trail is what is being watched, and an
        # unbounded Path eventually stalls the browser.
        if len(self.path.poses) > self.max_points:
            self.path.poses = self.path.poses[-self.max_points:]

    def _gps_cb(self, msg):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return

        # Keep the newest position even when it is not a fix, so the map can
        # show WHERE the receiver thinks it is and, just as importantly, say
        # that it does not actually know.  Foxglove's map panel plots any
        # NavSatFix it is given without looking at the status field, so a
        # no-fix position renders as a confident-looking dot; labelling it
        # here is what stops that from being read as a working GPS.
        self._latest = (msg.longitude, msg.latitude, int(msg.status.status))

        if msg.status.status < 0:
            return                      # NO_FIX: not worth a trail point
        ll = (msg.latitude, msg.longitude)
        if self._last_ll is not None and self._haversine(ll, self._last_ll) < self.min_gd:
            return
        self._last_ll = ll
        self.geo.append([round(msg.longitude, 8), round(msg.latitude, 8)])
        if len(self.geo) > self.max_points:
            self.geo = self.geo[-self.max_points:]

    def _clear_cb(self, _msg):
        self.path.poses.clear()
        self.geo.clear()
        self._last_xy = self._last_ll = None
        self.get_logger().info('trajectory cleared')
        self._publish()

    @staticmethod
    def _haversine(a, b):
        """Great-circle distance in metres between two (lat, lon) pairs."""
        r = EARTH_RADIUS_M
        p1, p2 = math.radians(a[0]), math.radians(b[0])
        dp = p2 - p1
        dl = math.radians(b[1] - a[1])
        h = (math.sin(dp / 2) ** 2
             + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2)
        return 2 * r * math.asin(min(1.0, math.sqrt(h)))

    # ------------------------------------------------------------- output
    def _publish(self):
        self.path.header.stamp = self.get_clock().now().to_msg()
        if not self.path.header.frame_id:
            self.path.header.frame_id = 'map'
        self.path_pub.publish(self.path)

        features = []

        # A LineString needs two points; with fewer, emit no line rather than
        # invalid GeoJSON the map panel would reject outright.
        if len(self.geo) >= 2:
            features.append({
                'type': 'Feature',
                'geometry': {'type': 'LineString', 'coordinates': self.geo},
                'properties': {'stroke': TRAIL_COLOR,
                               'stroke-width': TRAIL_WIDTH,
                               'stroke-opacity': 0.9, 'name': 'travelled'},
            })

        # Always mark the current position, so the map is never blank while the
        # rover is parked, and colour it by fix quality.
        if self._latest is not None:
            lon, lat, status = self._latest
            fixed = status >= 0
            features.append({
                'type': 'Feature',
                'geometry': {'type': 'Point', 'coordinates': [lon, lat]},
                'properties': {
                    'name': 'rover' if fixed else 'rover (NO FIX)',
                    'marker-color': ROVER_FIX_COLOR if fixed else ROVER_NOFIX_COLOR,
                    'marker-symbol': 'car',
                },
            })

        self.geo_pub.publish(GeoJSON(geojson=json.dumps(
            {'type': 'FeatureCollection', 'features': features})))


def main(args=None):
    rclpy.init(args=args)
    node = Trajectory()
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
