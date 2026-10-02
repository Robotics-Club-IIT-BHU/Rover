#!/usr/bin/env python3
"""
teleop_mux.py. Manual driving with a live speed limit, safely mixed with Nav2.

The Foxglove Teleop panel can only publish a fixed Twist per button; its
values are baked into the layout, so there is no way to slow down from the
browser without editing the panel.  This node fixes that by treating the
panel's output as a DIRECTION (-1..+1) and applying the speed limit here,
where it can be changed at runtime from another panel.

Inputs
    /athena/teleop/cmd_vel    geometry_msgs/Twist   direction, -1..+1 per axis
    /athena/teleop/max_speed  std_msgs/Float32      m/s   at full stick
    /athena/teleop/max_turn   std_msgs/Float32      rad/s at full stick

Output
    /cmd_vel                  geometry_msgs/Twist   what the motors receive
    /athena/teleop/status     std_msgs/String       current limits and mode

Why this cancels Nav2 instead of blending with it
    Nav2's velocity_smoother also publishes /cmd_vel.  Two publishers on one
    topic do not merge, they interleave, and a rover receiving alternating
    autonomous and manual commands does neither thing properly.  So the first
    time a real teleop input arrives this node cancels every running Nav2
    goal.  Taking the controls should end autonomy, which is also what an
    operator reaching for the stick actually wants.  While teleop is idle
    this node publishes nothing at all, leaving /cmd_vel entirely to Nav2.

Started by athena_remote/launch/foxglove.launch.py; standalone:
    ros2 run athena_remote teleop_mux

Author: Jashan
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String

DEADBAND = 0.02          # below this a stick axis counts as centred


class TeleopMux(Node):

    def __init__(self):
        super().__init__('teleop_mux')

        self.declare_parameter('max_speed', 0.35)     # m/s at full stick
        self.declare_parameter('max_turn', 0.8)       # rad/s at full stick
        self.declare_parameter('speed_ceiling', 1.0)  # hard cap, m/s
        self.declare_parameter('turn_ceiling', 2.0)   # hard cap, rad/s
        self.declare_parameter('idle_timeout', 0.6)   # s of silence -> release
        self.declare_parameter('publish_rate', 20.0)  # Hz while active
        self.declare_parameter('cancel_nav_on_input', True)

        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_turn = float(self.get_parameter('max_turn').value)
        self.speed_ceiling = float(self.get_parameter('speed_ceiling').value)
        self.turn_ceiling = float(self.get_parameter('turn_ceiling').value)
        self.idle_timeout = float(self.get_parameter('idle_timeout').value)
        self.cancel_nav = bool(self.get_parameter('cancel_nav_on_input').value)

        self._dir = (0.0, 0.0)
        self._last_input = None
        self._active = False

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/athena/teleop/status', 10)

        self.create_subscription(Twist, '/athena/teleop/cmd_vel', self._cmd_cb, 10)
        self.create_subscription(Float32, '/athena/teleop/max_speed', self._speed_cb, 10)
        self.create_subscription(Float32, '/athena/teleop/max_turn', self._turn_cb, 10)

        # Cancelling with an empty goal_id and zero stamp is the ROS 2 action
        # protocol's "cancel everything", so this works on goals sent by RViz,
        # goal_manager, or a script, not just ones we know about.
        self.cancel_cli = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')

        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)
        self.create_timer(1.0, self._report)

        self.get_logger().info(
            f'teleop mux up: {self.max_speed} m/s, {self.max_turn} rad/s at full stick')

    # ------------------------------------------------------------- inputs
    def _cmd_cb(self, msg):
        self._dir = (
            max(-1.0, min(1.0, msg.linear.x)),
            max(-1.0, min(1.0, msg.angular.z)),
        )
        self._last_input = self.get_clock().now()

        moving = abs(self._dir[0]) > DEADBAND or abs(self._dir[1]) > DEADBAND
        if moving and not self._active:
            self._active = True
            self.get_logger().info('manual control taken')
            if self.cancel_nav:
                self._cancel_all_nav_goals()

    def _speed_cb(self, msg):
        self.max_speed = max(0.0, min(float(msg.data), self.speed_ceiling))
        self.get_logger().info(f'max speed -> {self.max_speed:.2f} m/s')
        self._report()

    def _turn_cb(self, msg):
        self.max_turn = max(0.0, min(float(msg.data), self.turn_ceiling))
        self.get_logger().info(f'max turn -> {self.max_turn:.2f} rad/s')
        self._report()

    def _cancel_all_nav_goals(self):
        if not self.cancel_cli.service_is_ready():
            return          # Nav2 is not running; nothing to cancel
        self.cancel_cli.call_async(CancelGoal.Request())

    # ------------------------------------------------------------- output
    def _tick(self):
        if self._last_input is None:
            return

        age = (self.get_clock().now() - self._last_input).nanoseconds * 1e-9
        if age > self.idle_timeout:
            if self._active:
                # One explicit zero on the way out, so the rover stops even if
                # the last thing the browser sent was a nonzero command that
                # never got its release event (a closed tab, a dropped WiFi).
                self.cmd_pub.publish(Twist())
                self._active = False
                self._dir = (0.0, 0.0)
                self.get_logger().info('manual control released')
            return

        t = Twist()
        t.linear.x = self._dir[0] * self.max_speed
        t.angular.z = self._dir[1] * self.max_turn
        self.cmd_pub.publish(t)

    def _report(self):
        mode = 'MANUAL' if self._active else 'idle (Nav2 has the wheels)'
        self.status_pub.publish(String(data=(
            f'{mode} | max {self.max_speed:.2f} m/s, {self.max_turn:.2f} rad/s')))


def main(args=None):
    rclpy.init(args=args)
    node = TeleopMux()
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
