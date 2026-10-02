#!/usr/bin/env python3
"""
waypoint_manager.py. Mark places now, drive back to them later.

Marking a spot is the one thing an operator wants that neither Nav2 nor
Foxglove provides: Nav2 only knows the goal you are driving to right now, and
it forgets it the moment the goal ends.  This node keeps a named list on disk
so a place marked during one run is still there after a reboot, and draws that
list into both the 3D view and the satellite map.

Inputs  (all std_msgs/String)
    /athena/waypoint_save    name to store the rover's current position under
                             (empty string -> auto-named wp1, wp2, ...)
    /athena/waypoint_goto    name to navigate to
    /athena/waypoint_delete  name to forget

Outputs
    /athena/waypoints          visualization_msgs/MarkerArray   3D panel
    /athena/waypoints_geojson  foxglove_msgs/GeoJSON            map panel
    /athena/waypoint_list      std_msgs/String                  readable list
    /athena/goal_gps           sensor_msgs/NavSatFix   (goto, when geo-tagged)
    /goal_pose                 geometry_msgs/PoseStamped (goto, map-only)

Storage
    ~/.config/athena_remote/waypoints.yaml

Every waypoint always stores its map-frame pose, and additionally stores
lat/lon when the GPS had a fix at the time.  That split matters: map
coordinates are exact but only meaningful until the datum moves (a restart
without GPS re-origins the map), while lat/lon survives anything but needs a
fix to record.  Storing both means marking still works with no GPS, and
anything marked with a fix stays valid forever.

Author: Jashan
"""

import json
import math
import os

import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from foxglove_msgs.msg import GeoJSON
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

DEFAULT_STORE = os.path.expanduser('~/.config/athena_remote/waypoints.yaml')

# 3D-panel marker styling: a yellow post with the name floating above it.
PIN_HEIGHT = 0.5              # m
PIN_DIAMETER = 0.18           # m
PIN_CENTRE_Z = 0.25           # m, so the post sits on the ground
PIN_RGBA = (1.0, 0.85, 0.2, 0.85)
LABEL_Z = 0.7                 # m, clear of the top of the post
LABEL_TEXT_HEIGHT = 0.28      # m
LABEL_RGBA = (1.0, 1.0, 1.0, 0.95)

# Map-panel marker colour, matching the 3D pins.
GEOJSON_MARKER_COLOR = '#ffd633'


class WaypointManager(Node):

    def __init__(self):
        super().__init__('waypoint_manager')

        self.declare_parameter('store', DEFAULT_STORE)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('publish_period', 1.0)

        self.store = os.path.expanduser(self.get_parameter('store').value)
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value

        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)

        self.fix = None
        self.create_subscription(NavSatFix, '/gps/fix', self._fix_cb, 10)

        self.marker_pub = self.create_publisher(MarkerArray, '/athena/waypoints', 10)
        self.geo_pub = self.create_publisher(GeoJSON, '/athena/waypoints_geojson', 10)
        self.list_pub = self.create_publisher(String, '/athena/waypoint_list', 10)
        self.goal_gps_pub = self.create_publisher(NavSatFix, '/athena/goal_gps', 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.create_subscription(String, '/athena/waypoint_save', self._save_cb, 10)
        self.create_subscription(String, '/athena/waypoint_goto', self._goto_cb, 10)
        self.create_subscription(String, '/athena/waypoint_delete', self._del_cb, 10)

        self.waypoints = self._load()
        self.get_logger().info(
            f'{len(self.waypoints)} waypoint(s) loaded from {self.store}')

        self.create_timer(
            float(self.get_parameter('publish_period').value), self._publish)

    # ------------------------------------------------------------- storage
    def _load(self):
        try:
            with open(self.store) as f:
                data = yaml.safe_load(f) or {}
            return list(data.get('waypoints', []))
        except FileNotFoundError:
            return []
        except Exception as e:
            self.get_logger().warn(f'could not read {self.store}: {e}')
            return []

    def _persist(self):
        os.makedirs(os.path.dirname(self.store), exist_ok=True)
        tmp = self.store + '.tmp'
        # Write-then-rename so an interrupted save cannot leave a half-written
        # file behind; the rover may lose power at any moment.
        with open(tmp, 'w') as f:
            yaml.safe_dump({'waypoints': self.waypoints}, f, sort_keys=False)
        os.replace(tmp, self.store)

    # ------------------------------------------------------------- inputs
    def _fix_cb(self, msg):
        # status >= 0 means an actual fix; -1 is NO_FIX and carries junk
        self.fix = msg if msg.status.status >= 0 else None

    def _save_cb(self, msg):
        name = msg.data.strip() or self._auto_name()

        try:
            tr = self.tf_buf.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().error(
                f'cannot save "{name}": no {self.map_frame} -> '
                f'{self.robot_frame} transform ({type(e).__name__})')
            return

        t, q = tr.transform.translation, tr.transform.rotation
        wp = {
            'name': name,
            'x': round(float(t.x), 3),
            'y': round(float(t.y), 3),
            'yaw': round(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z)), 4),
        }
        if self.fix is not None:
            wp['latitude'] = round(float(self.fix.latitude), 8)
            wp['longitude'] = round(float(self.fix.longitude), 8)

        self.waypoints = [w for w in self.waypoints if w.get('name') != name]
        self.waypoints.append(wp)
        self._persist()

        where = (f'{wp["latitude"]:.7f}, {wp["longitude"]:.7f}'
                 if 'latitude' in wp else f'map ({wp["x"]:+.2f}, {wp["y"]:+.2f}), no GPS fix')
        self.get_logger().info(f'saved "{name}" at {where}')
        self._publish()

    def _goto_cb(self, msg):
        name = msg.data.strip()
        wp = next((w for w in self.waypoints if w.get('name') == name), None)
        if wp is None:
            self.get_logger().error(
                f'no waypoint named "{name}" (have: {self._names() or "none"})')
            return

        if 'latitude' in wp:
            fix = NavSatFix()
            fix.header.stamp = self.get_clock().now().to_msg()
            fix.header.frame_id = 'gps_link'
            fix.latitude = float(wp['latitude'])
            fix.longitude = float(wp['longitude'])
            self.goal_gps_pub.publish(fix)
            self.get_logger().info(f'going to "{name}" by GPS')
        else:
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(wp['x'])
            pose.pose.position.y = float(wp['y'])
            yaw = float(wp.get('yaw', 0.0))
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            self.goal_pose_pub.publish(pose)
            self.get_logger().info(f'going to "{name}" by map pose')

    def _del_cb(self, msg):
        name = msg.data.strip()
        before = len(self.waypoints)
        self.waypoints = [w for w in self.waypoints if w.get('name') != name]
        if len(self.waypoints) == before:
            self.get_logger().warn(f'no waypoint named "{name}"')
            return
        self._persist()
        self.get_logger().info(f'deleted "{name}"')
        self._publish()

    def _auto_name(self):
        n = 1
        existing = set(self._names_list())
        while f'wp{n}' in existing:
            n += 1
        return f'wp{n}'

    def _names_list(self):
        return [w.get('name', '') for w in self.waypoints]

    def _names(self):
        return ', '.join(self._names_list())

    # ------------------------------------------------------------- outputs
    def _publish(self):
        self._publish_markers()
        self._publish_geojson()
        self.list_pub.publish(String(data=self._readable()))

    def _readable(self):
        if not self.waypoints:
            return 'no waypoints saved'
        lines = []
        for w in self.waypoints:
            if 'latitude' in w:
                lines.append(f'{w["name"]}: {w["latitude"]:.7f}, {w["longitude"]:.7f}')
            else:
                lines.append(f'{w["name"]}: map ({w["x"]:+.2f}, {w["y"]:+.2f})')
        return '\n'.join(lines)

    def _publish_markers(self):
        arr = MarkerArray()

        # A DELETEALL first, so a deleted waypoint actually disappears from the
        # 3D panel instead of lingering as a stale marker id forever.
        clear = Marker()
        clear.action = Marker.DELETEALL
        clear.header.frame_id = self.map_frame
        arr.markers.append(clear)

        # Ids are allocated in pairs (post, label) so a waypoint's two markers
        # can never collide with another waypoint's.
        for i, w in enumerate(self.waypoints):
            pin = self._pin_marker(i, w)
            arr.markers.append(pin)
            arr.markers.append(self._label_marker(i, w, pin))

        self.marker_pub.publish(arr)

    def _pin_marker(self, i, w):
        pin = Marker()
        pin.header.frame_id = self.map_frame
        pin.header.stamp = self.get_clock().now().to_msg()
        pin.ns = 'waypoints'
        pin.id = i * 2
        pin.type = Marker.CYLINDER
        pin.action = Marker.ADD
        pin.pose.position.x = float(w.get('x', 0.0))
        pin.pose.position.y = float(w.get('y', 0.0))
        pin.pose.position.z = PIN_CENTRE_Z
        pin.pose.orientation.w = 1.0
        pin.scale.x = pin.scale.y = PIN_DIAMETER
        pin.scale.z = PIN_HEIGHT
        (pin.color.r, pin.color.g,
         pin.color.b, pin.color.a) = PIN_RGBA
        return pin

    @staticmethod
    def _label_marker(i, w, pin):
        label = Marker()
        label.header = pin.header
        label.ns = 'waypoint_labels'
        label.id = i * 2 + 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = pin.pose.position.x
        label.pose.position.y = pin.pose.position.y
        label.pose.position.z = LABEL_Z
        label.pose.orientation.w = 1.0
        label.scale.z = LABEL_TEXT_HEIGHT
        (label.color.r, label.color.g,
         label.color.b, label.color.a) = LABEL_RGBA
        label.text = w.get('name', '?')
        return label

    def _publish_geojson(self):
        features = []
        for w in self.waypoints:
            # Map-only waypoints have no lat/lon, so there is nowhere on a
            # satellite map to draw them. They still appear in the 3D view.
            if 'latitude' not in w:
                continue
            features.append({
                'type': 'Feature',
                'geometry': {
                    'type': 'Point',
                    'coordinates': [float(w['longitude']), float(w['latitude'])],
                },
                'properties': {
                    'name': w.get('name', '?'),
                    'marker-color': GEOJSON_MARKER_COLOR,
                },
            })
        self.geo_pub.publish(GeoJSON(geojson=json.dumps(
            {'type': 'FeatureCollection', 'features': features})))


def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
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
