#!/usr/bin/env python3
"""
gps_waypoint_follower.py

Follows GPS (lat/lon) waypoints with Nav2 on ROS 2 Humble.

Humble's nav2_waypoint_follower has no FollowGPSWaypoints action (Iron+ only),
so this node does the conversion itself:

    lat/lon --(robot_localization /fromLL service)--> map x/y --> Nav2
    followWaypoints via nav2_simple_commander.

Usage:
    # follow a YAML file of waypoints
    ros2 run athena_gps_nav gps_waypoint_follower --ros-args \
        -p waypoints_file:=/path/to/waypoints.yaml

    # or a single goal
    ros2 run athena_gps_nav gps_waypoint_follower --ros-args \
        -p goal_lat:=25.2629548 -p goal_lon:=82.9838284

waypoints.yaml format (yaw optional, radians ENU; default: face next wp):
    waypoints:
      - {latitude: 25.2629548, longitude: 82.9838284, yaw: 0.0}
      - {latitude: 25.2629321, longitude: 82.9838343}
"""

import math
import time

import rclpy
import yaml

from geometry_msgs.msg import PoseStamped
from robot_localization.srv import FromLL
from lifecycle_msgs.srv import GetState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def quaternion_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class GpsWaypointFollower(BasicNavigator):
    def __init__(self):
        super().__init__()
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('goal_lat', float('nan'))
        self.declare_parameter('goal_lon', float('nan'))
        self.declare_parameter('goal_yaw', float('nan'))

        self.fromll_client = self.create_client(FromLL, '/fromLL')

    # -------------------------------------------------------------- helpers
    def wait_for_nav2(self, timeout=120.0):
        """Wait for bt_navigator to be active (can't use waitUntilNav2Active:
        it waits on amcl, which this stack doesn't run)."""
        client = self.create_client(GetState, 'bt_navigator/get_state')
        deadline = time.time() + timeout
        while time.time() < deadline:
            if client.wait_for_service(timeout_sec=2.0):
                req = GetState.Request()
                fut = client.call_async(req)
                rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
                if fut.result() is not None and \
                        fut.result().current_state.label == 'active':
                    self.get_logger().info('bt_navigator is active')
                    return True
            self.get_logger().info('waiting for bt_navigator...')
        raise RuntimeError('bt_navigator did not become active')

    def wait_for_fromll(self, timeout=120.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.fromll_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().info('/fromLL service available')
                return True
            self.get_logger().info(
                'waiting for /fromLL (navsat_transform needs a GPS fix '
                'before it initializes)...')
        raise RuntimeError('/fromLL not available')

    def ll_to_map_pose(self, lat, lon, yaw=0.0):
        req = FromLL.Request()
        req.ll_point.latitude = float(lat)
        req.ll_point.longitude = float(lon)
        req.ll_point.altitude = 0.0
        fut = self.fromll_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        if fut.result() is None:
            raise RuntimeError(f'/fromLL call failed for ({lat}, {lon})')
        p = fut.result().map_point

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = p.x
        pose.pose.position.y = p.y
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.get_logger().info(
            f'({lat:.7f}, {lon:.7f}) -> map ({p.x:.2f}, {p.y:.2f})')
        return pose

    # -------------------------------------------------------------- main
    def load_waypoints(self):
        wp_file = self.get_parameter('waypoints_file').value
        lat = self.get_parameter('goal_lat').value
        lon = self.get_parameter('goal_lon').value
        gyaw = self.get_parameter('goal_yaw').value

        wps = []
        if wp_file:
            with open(wp_file, 'r') as f:
                data = yaml.safe_load(f)
            for wp in data['waypoints']:
                wps.append((wp['latitude'], wp['longitude'],
                            wp.get('yaw', None)))
        elif not math.isnan(lat) and not math.isnan(lon):
            wps.append((lat, lon, None if math.isnan(gyaw) else gyaw))
        else:
            raise RuntimeError(
                'Provide waypoints_file:=... or goal_lat/goal_lon params')
        return wps

    @staticmethod
    def face_next_waypoint(wps, poses):
        """Point every waypoint that gave no yaw at the one after it.

        Without this the rover arrives at each intermediate waypoint facing
        whatever the last one specified, then has to turn on the spot before
        it can set off again - slower, and hard on a skid-steer chassis.
        The final waypoint is left alone: there is nothing after it to aim at.
        """
        for i, (_, _, yaw) in enumerate(wps):
            if yaw is None and i + 1 < len(poses):
                dx = poses[i + 1].pose.position.x - poses[i].pose.position.x
                dy = poses[i + 1].pose.position.y - poses[i].pose.position.y
                qx, qy, qz, qw = quaternion_from_yaw(math.atan2(dy, dx))
                poses[i].pose.orientation.x = qx
                poses[i].pose.orientation.y = qy
                poses[i].pose.orientation.z = qz
                poses[i].pose.orientation.w = qw

    def run(self):
        wps = self.load_waypoints()
        self.get_logger().info(f'{len(wps)} GPS waypoint(s) loaded')

        self.wait_for_nav2()
        self.wait_for_fromll()

        poses = []
        for lat, lon, yaw in wps:
            poses.append(self.ll_to_map_pose(lat, lon, yaw or 0.0))
        self.face_next_waypoint(wps, poses)

        self.followWaypoints(poses)
        while not self.isTaskComplete():
            fb = self.getFeedback()
            if fb:
                self.get_logger().info(
                    f'executing waypoint {fb.current_waypoint + 1}/{len(poses)}',
                    throttle_duration_sec=5.0)
            time.sleep(0.5)

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('All GPS waypoints reached.')
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('Waypoint following canceled.')
        else:
            self.get_logger().error('Waypoint following failed.')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = GpsWaypointFollower()
    try:
        node.run()
    except KeyboardInterrupt:
        node.cancelTask()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
