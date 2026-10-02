#!/usr/bin/env python3
"""
gps_waypoint_logger.py

Drive the rover (teleop) to each spot, press ENTER to record the current GPS
fix + heading into a waypoints YAML usable by gps_waypoint_follower.

    ros2 run athena_gps_nav gps_waypoint_logger --ros-args \
        -p output_file:=/home/robo/waypoints.yaml
"""

import math
import threading

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix


class GpsWaypointLogger(Node):
    def __init__(self):
        super().__init__('gps_waypoint_logger')
        self.declare_parameter('output_file', '/home/robo/athena_waypoints.yaml')
        self.output_file = self.get_parameter('output_file').value

        self.last_fix = None
        self.last_yaw = 0.0
        self.waypoints = []

        self.create_subscription(NavSatFix, '/gps/fix', self._gps_cb, 10)
        self.create_subscription(Imu, '/imu/data', self._imu_cb, 10)

        threading.Thread(target=self._stdin_loop, daemon=True).start()
        self.get_logger().info(
            f'Press ENTER to record a waypoint -> {self.output_file} '
            '(Ctrl-C to finish)')

    def _gps_cb(self, msg):
        self.last_fix = msg

    def _imu_cb(self, msg):
        q = msg.orientation
        self.last_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _stdin_loop(self):
        while True:
            try:
                input()
            except EOFError:
                return
            if self.last_fix is None:
                self.get_logger().warn('No GPS fix received yet!')
                continue
            wp = {
                'latitude': float(self.last_fix.latitude),
                'longitude': float(self.last_fix.longitude),
                'yaw': float(self.last_yaw),
            }
            self.waypoints.append(wp)
            with open(self.output_file, 'w') as f:
                yaml.dump({'waypoints': self.waypoints}, f,
                          default_flow_style=False)
            self.get_logger().info(
                f'wp {len(self.waypoints)}: {wp["latitude"]:.7f}, '
                f'{wp["longitude"]:.7f}, yaw {wp["yaw"]:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = GpsWaypointLogger()
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
