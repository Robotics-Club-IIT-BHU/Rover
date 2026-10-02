#!/usr/bin/env python3
"""
teleop.py — keyboard teleop for the Athena rover.

Arrow keys drive, and speed is adjustable on the fly (the bridge + v2
firmware take real velocities now, not fixed-speed F/B/L/R):

    up / down       forward / reverse at current linear speed
    left / right    turn at current angular speed
    q / z           linear speed  +/- 0.05 m/s
    e / c           angular speed +/- 0.10 rad/s
    space           stop
    Ctrl-C          quit (sends stop)

Dead-man behavior: releasing the keys stops the rover after `hold_timeout`
(default 0.4 s) — no more runaway rover when you let go.  The old teleop
kept the last command until space was pressed.
"""

import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TeleopTwist(Node):
    def __init__(self, settings):
        super().__init__('teleop_twist_node')
        self.settings = settings

        self.declare_parameter('linear_speed', 0.35)   # matches Nav2 cruise
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('hold_timeout', 0.4)    # s, dead-man release

        self.lin = float(self.get_parameter('linear_speed').value)
        self.ang = float(self.get_parameter('angular_speed').value)
        self.hold_timeout = float(self.get_parameter('hold_timeout').value)

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.get_logger().info('--- Twist Teleop ---')
        self.get_logger().info(
            'arrows: drive | q/z: lin +/- | e/c: ang +/- | space: stop | Ctrl-C: quit')
        self._show_speeds()

    def _show_speeds(self):
        self.get_logger().info(
            f'linear {self.lin:.2f} m/s   angular {self.ang:.2f} rad/s')

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
            if key == '\x1b':  # multi-byte arrow keys
                key += sys.stdin.read(2)
            return key
        return ''

    def run(self):
        last_key_time = 0.0
        moving = False
        try:
            while True:
                key = self.get_key()
                twist = Twist()
                publish = False

                if key == '\x1b[A':      # UP
                    twist.linear.x = self.lin
                    publish = True
                elif key == '\x1b[B':    # DOWN
                    twist.linear.x = -self.lin
                    publish = True
                elif key == '\x1b[D':    # LEFT
                    twist.angular.z = self.ang
                    publish = True
                elif key == '\x1b[C':    # RIGHT
                    twist.angular.z = -self.ang
                    publish = True
                elif key == ' ':
                    publish = True       # zero twist = stop
                elif key == 'q':
                    self.lin = min(self.lin + 0.05, 1.0)
                    self._show_speeds()
                elif key == 'z':
                    self.lin = max(self.lin - 0.05, 0.05)
                    self._show_speeds()
                elif key == 'e':
                    self.ang = min(self.ang + 0.1, 2.0)
                    self._show_speeds()
                elif key == 'c':
                    self.ang = max(self.ang - 0.1, 0.1)
                    self._show_speeds()
                elif key == '\x03':      # Ctrl-C
                    break

                now = time.time()
                if publish:
                    self.publisher_.publish(twist)
                    moving = (twist.linear.x != 0.0 or twist.angular.z != 0.0)
                    last_key_time = now
                elif moving and (now - last_key_time) > self.hold_timeout:
                    # dead-man: keys released -> stop
                    self.publisher_.publish(Twist())
                    moving = False

        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            self.publisher_.publish(Twist())
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = TeleopTwist(settings)
    node.run()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
