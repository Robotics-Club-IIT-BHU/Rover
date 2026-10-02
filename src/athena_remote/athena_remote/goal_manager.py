#!/usr/bin/env python3
"""
goal_manager.py. Turns an operator's goal request into a Nav2 goal.

Nav2 already accepts a map-frame goal on /goal_pose, which is what the
Foxglove 3D panel publishes when you click.  What it does NOT accept is a
latitude/longitude, and on a GPS rover that is the natural way to say where
you want to go.  This node fills that gap and gives the front end one place
to send every kind of goal request.

Inputs
    /athena/goal_gps    sensor_msgs/NavSatFix   lat/lon, e.g. a map click
    /athena/goal_text   std_msgs/String         typed command, see below
    /athena/goal_cancel std_msgs/Empty          stop the current goal
    /athena/goal_click  geometry_msgs/PoseStamped  a pose clicked in the
                                                Foxglove 3D panel, any frame

Why clicks come through here and not straight to /goal_pose
    Foxglove stamps a click in the 3D panel's display frame (odom by default,
    base_link when following the rover).  bt_navigator in Humble keeps that
    frame AND that stamp, and the planner re-transforms the goal into map at
    the ORIGINAL stamp on every 1 Hz replan.  TF only keeps 10 s of history,
    so 10 s after the click the planner can no longer read the goal ("Could
    not transform the start or goal pose"), and after six recovery rounds the
    goal fails.  Every clicked goal on 2026-10-02 that needed more than 10 s
    (3.5 m at cruise speed) failed exactly like that.  Converting the click
    into map ONCE, here, at the moment it was clicked, removes the problem for
    every display frame; map-frame goals are never re-transformed.

Text commands (whitespace or comma separated, case-insensitive):
    "25.2629548, 82.9838284"        go to this lat/lon, face along the path
    "25.2629548 82.9838284 90"      ... and finish facing 90 deg (compass)
    "cancel"                        abort the running goal

Outputs
    /athena/goal_status std_msgs/String         human-readable state
    /athena/goal_marker visualization_msgs/Marker   where the goal is

How lat/lon becomes a map coordinate
    robot_localization's navsat_transform node offers a /fromLL service that
    converts a geodetic point into the map frame using the datum it locked at
    startup.  That is the same conversion the global EKF uses, so a goal sent
    this way lands in exactly the frame Nav2 plans in.

    The trap: with no GPS fix the datum was never set, and /fromLL does NOT
    refuse -- it answers cheerfully with map (0, 0) for every coordinate on
    earth.  Nav2 then accepts a goal at the map origin and reports "reached"
    immediately, so the operator sees a success message for a goal that was
    never understood; with motors attached it would drive somewhere arbitrary.
    So this node requires a real fix before it will convert anything, and says
    why on /athena/goal_status when it refuses.

Author: Jashan
"""

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from robot_localization.srv import FromLL
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Empty, String
from visualization_msgs.msg import Marker
import tf2_geometry_msgs
import tf2_ros


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def compass_deg_to_enu_rad(deg):
    """Compass bearing (0=North, clockwise) -> ENU yaw (0=East, counter-clockwise).

    Operators think in compass headings; REP-103 and every frame in this stack
    think in ENU.  Doing the conversion here means the text command reads the
    way a person expects while Nav2 still receives a standards-compliant pose.
    """
    return math.radians(90.0 - deg)


class GoalManager(Node):

    def __init__(self):
        super().__init__('goal_manager')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('fromll_timeout', 5.0)
        # Set false only for bench testing without a GPS; see the docstring for
        # what /fromLL does when the datum was never set.
        self.declare_parameter('require_gps_fix', True)

        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.fromll_timeout = float(self.get_parameter('fromll_timeout').value)
        self.require_fix = bool(self.get_parameter('require_gps_fix').value)
        self.have_fix = False

        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)

        self.fromll = self.create_client(FromLL, '/fromLL')
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.status_pub = self.create_publisher(String, '/athena/goal_status', 10)
        self.marker_pub = self.create_publisher(Marker, '/athena/goal_marker', 10)

        self.create_subscription(NavSatFix, '/athena/goal_gps', self._gps_cb, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self._fix_cb, 10)
        self.create_subscription(String, '/athena/goal_text', self._text_cb, 10)
        self.create_subscription(Empty, '/athena/goal_cancel', self._cancel_cb, 10)
        self.create_subscription(PoseStamped, '/athena/goal_click', self._click_cb, 10)

        self._goal_handle = None

        self._say('ready. Send a lat/lon on /athena/goal_gps or /athena/goal_text')

    # ------------------------------------------------------------- helpers
    def _say(self, text):
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def _fix_cb(self, msg):
        # status >= 0 is STATUS_FIX or better; -1 is NO_FIX, whose lat/lon are
        # placeholders.  Latching on first fix rather than tracking live state:
        # once navsat_transform has a datum it keeps it, so a later dropout
        # does not invalidate the conversion.
        if msg.status.status >= 0 and not self.have_fix:
            self.have_fix = True
            self._say('GPS fix acquired. Lat/lon goals are now accepted')

    def _robot_xy(self):
        """Current position in the map frame, or None if TF is not ready."""
        try:
            tr = self.tf_buf.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time())
            return tr.transform.translation.x, tr.transform.translation.y
        except Exception:
            return None

    # ------------------------------------------------------------- inputs
    def _gps_cb(self, msg):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            self._say('ignored: goal had NaN latitude/longitude')
            return
        self._go_to_ll(msg.latitude, msg.longitude, None)

    def _text_cb(self, msg):
        raw = msg.data.strip().lower()
        if not raw:
            return
        if raw.startswith('cancel') or raw.startswith('stop'):
            self._cancel_cb(None)
            return

        parts = [p for p in raw.replace(',', ' ').split() if p]
        try:
            nums = [float(p) for p in parts]
        except ValueError:
            self._say(f'could not parse "{msg.data}". Expected "lat, lon [heading]"')
            return

        if len(nums) == 2:
            self._go_to_ll(nums[0], nums[1], None)
        elif len(nums) == 3:
            self._go_to_ll(nums[0], nums[1], compass_deg_to_enu_rad(nums[2]))
        else:
            self._say(f'expected 2 or 3 numbers, got {len(nums)}')

    def _click_cb(self, msg):
        """A pose clicked in the 3D panel: re-express it in map, once.

        Looked up at the click's own stamp first, which is what the operator
        saw (the panel can lag the rover by a second over the tunnel); the
        latest transform is the fallback when that stamp is outside the TF
        buffer, e.g. a laptop clock that disagrees with the Jetson's.
        """
        frame = msg.header.frame_id.lstrip('/') or self.map_frame
        if frame == self.map_frame:
            pose, how = PoseStamped(pose=msg.pose), 'map'
        else:
            tr, err = None, None
            for when, label in ((rclpy.time.Time.from_msg(msg.header.stamp), 'at click time'),
                                (rclpy.time.Time(), 'at the latest pose')):
                try:
                    tr = self.tf_buf.lookup_transform(self.map_frame, frame, when)
                    how = f'{frame} -> {self.map_frame} {label}'
                    break
                except tf2_ros.TransformException as e:
                    err = e
            if tr is None:
                self._say(f'refused clicked goal: no {self.map_frame} <- {frame} '
                          f'transform ({type(err).__name__})')
                return
            pose = tf2_geometry_msgs.do_transform_pose_stamped(msg, tr)

        # The planner is 2D: keep the heading, drop roll, pitch and height.
        q = pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        pose.pose.position.z = 0.0
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = yaw_to_quat(yaw)
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()

        self._publish_marker(pose)
        self._send_nav_goal(pose, f'clicked pose ({how})')

    def _cancel_cb(self, _msg):
        if self._goal_handle is None:
            self._say('nothing to cancel')
            return
        self._goal_handle.cancel_goal_async()
        self._goal_handle = None
        self._say('goal cancelled')

    # ------------------------------------------------------------- action
    def _go_to_ll(self, lat, lon, yaw):
        if self.require_fix and not self.have_fix:
            self._say(
                'refused: no GPS fix yet, so the map has no geographic origin. '
                'Every lat/lon would convert to map (0,0) and the rover would '
                'drive to the wrong place. Wait for a fix, or send a map-frame '
                'goal by clicking in the 3D panel instead.')
            return
        if not self.fromll.wait_for_service(timeout_sec=self.fromll_timeout):
            self._say('/fromLL unavailable. Navsat_transform is not running, '
                      'or the GPS has no fix yet so the datum was never set')
            return

        req = FromLL.Request()
        req.ll_point.latitude = lat
        req.ll_point.longitude = lon
        req.ll_point.altitude = 0.0

        self._say(f'converting {lat:.7f}, {lon:.7f} to the {self.map_frame} frame')
        future = self.fromll.call_async(req)
        future.add_done_callback(
            lambda f: self._on_converted(f, lat, lon, yaw))

    def _on_converted(self, future, lat, lon, yaw):
        try:
            res = future.result()
        except Exception as e:
            self._say(f'/fromLL failed: {e}')
            return

        x, y = res.map_point.x, res.map_point.y

        # No heading asked for: finish pointing the way we travelled, which is
        # what a person means by "drive there".  Falling back to 0 rad would
        # make the rover spin on the spot at the end of every goal.
        if yaw is None:
            here = self._robot_xy()
            yaw = math.atan2(y - here[1], x - here[0]) if here else 0.0

        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quat(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self._publish_marker(pose)
        self._send_nav_goal(pose, f'{lat:.7f}, {lon:.7f}')

    def _publish_marker(self, pose):
        m = Marker()
        m.header = pose.header
        m.ns = 'athena_goal'
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose = pose.pose
        m.scale.x, m.scale.y, m.scale.z = 1.0, 0.15, 0.15
        m.color.r, m.color.g, m.color.b, m.color.a = 0.15, 0.9, 0.4, 0.9
        self.marker_pub.publish(m)

    def _send_nav_goal(self, pose, what):
        if not self.nav.wait_for_server(timeout_sec=5.0):
            self._say('Nav2 navigate_to_pose action not available')
            return
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self._say(f'goal sent: {what} '
                  f'-> map ({pose.pose.position.x:+.1f}, {pose.pose.position.y:+.1f})')
        self.nav.send_goal_async(goal).add_done_callback(self._on_accepted)

    def _on_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            # Humble's bt_navigator rejects only while it is not ACTIVE: still
            # starting, or deactivated after a server lost its heartbeat.
            self._say('Nav2 rejected the goal: bt_navigator is not active '
                      '(still starting, or a Nav2 server crashed)')
            return
        self._goal_handle = handle
        self._say('Nav2 accepted the goal. Driving')
        handle.get_result_async().add_done_callback(
            lambda f, h=handle: self._on_result(f, h))

    def _on_result(self, future, handle):
        if self._goal_handle is not None and handle is not self._goal_handle:
            return          # one of ours, already replaced by a newer one of ours
        self._goal_handle = None
        try:
            status = future.result().status
        except Exception as e:
            self._say(f'goal ended with an error: {e}')
            return
        # action_msgs/GoalStatus.  Humble's result carries no reason: the
        # nav_status node works it out and shows it on /athena/nav_status.
        self._say({4: 'goal reached',
                   5: 'goal cancelled',
                   6: 'goal aborted by Nav2 (or replaced by a newer goal): '
                      'see Nav status for why'}.get(status, f'goal ended, status={status}'))


def main(args=None):
    rclpy.init(args=args)
    node = GoalManager()
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
