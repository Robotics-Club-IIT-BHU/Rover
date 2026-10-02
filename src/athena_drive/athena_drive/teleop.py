#!/usr/bin/env python3
"""
teleop.py. Keyboard driving, for shaking the rover down before Nav2.

    arrows        drive / turn
    q / z         linear speed  +/- 0.05 m/s
    e / c         angular speed +/- 0.10 rad/s
    space         stop
    Ctrl-C        quit (sends stop)

Publishes geometry_msgs/Twist on cmd_vel (relative, so a launch remap can
point it elsewhere); subscribes to nothing.  Run it in a terminal you can
see, since it reads raw keystrokes from stdin:

    ros2 run athena_drive teleop

Dead-man: releasing the keys stops the rover after hold_timeout.  A teleop
that latches the last command is how rovers drive off benches - you look
away, and it is still going.
"""

import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Terminal escape sequences for the arrow keys, and Ctrl-C, which arrives as
# a raw byte rather than a signal because the terminal is in raw mode.
KEY_UP = '\x1b[A'
KEY_DOWN = '\x1b[B'
KEY_LEFT = '\x1b[D'
KEY_RIGHT = '\x1b[C'
KEY_STOP = ' '
KEY_QUIT = '\x03'

# How far one keypress moves each limit, and the range it may be moved in.
LIN_STEP, LIN_MIN, LIN_MAX = 0.05, 0.05, 1.0
ANG_STEP, ANG_MIN, ANG_MAX = 0.1, 0.1, 2.0

KEY_POLL_TIMEOUT = 0.1   # s; also sets how often the dead-man is checked


class Teleop(Node):
    def __init__(self, settings):
        super().__init__('athena_teleop')
        self.settings = settings
        self.declare_parameter('linear_speed', 0.25)
        self.declare_parameter('angular_speed', 0.6)
        self.declare_parameter('hold_timeout', 0.4)
        self.lin = float(self.get_parameter('linear_speed').value)
        self.ang = float(self.get_parameter('angular_speed').value)
        self.hold = float(self.get_parameter('hold_timeout').value)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        print('arrows drive | q/z linear | e/c angular | space stop | Ctrl-C quit')
        self._show()

    def _show(self):
        print(f'  linear {self.lin:.2f} m/s   angular {self.ang:.2f} rad/s')

    def _key(self):
        tty.setraw(sys.stdin.fileno())
        r, _, _ = select.select([sys.stdin], [], [], KEY_POLL_TIMEOUT)
        if not r:
            return ''
        k = sys.stdin.read(1)
        if k == '\x1b':
            k += sys.stdin.read(2)   # arrows arrive as ESC + two more bytes
        return k

    def run(self):
        last = 0.0
        moving = False
        try:
            while rclpy.ok():
                k = self._key()
                t = Twist()
                send = False
                if k == KEY_UP:
                    t.linear.x = self.lin
                    send = True
                elif k == KEY_DOWN:
                    t.linear.x = -self.lin
                    send = True
                elif k == KEY_LEFT:
                    t.angular.z = self.ang
                    send = True
                elif k == KEY_RIGHT:
                    t.angular.z = -self.ang
                    send = True
                elif k == KEY_STOP:
                    send = True          # an all-zero Twist
                elif k == 'q':
                    self.lin = min(self.lin + LIN_STEP, LIN_MAX)
                    self._show()
                elif k == 'z':
                    self.lin = max(self.lin - LIN_STEP, LIN_MIN)
                    self._show()
                elif k == 'e':
                    self.ang = min(self.ang + ANG_STEP, ANG_MAX)
                    self._show()
                elif k == 'c':
                    self.ang = max(self.ang - ANG_STEP, ANG_MIN)
                    self._show()
                elif k == KEY_QUIT:
                    break

                now = time.time()
                if send:
                    self.pub.publish(t)
                    moving = (t.linear.x != 0.0 or t.angular.z != 0.0)
                    last = now
                elif moving and (now - last) > self.hold:
                    self.pub.publish(Twist())     # dead-man
                    moving = False
        finally:
            self.pub.publish(Twist())
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = Teleop(settings)
    node.run()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
