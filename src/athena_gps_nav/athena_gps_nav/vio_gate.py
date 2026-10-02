#!/usr/bin/env python3
"""
vio_gate.py

Physical-plausibility gate between rgbd_odometry and the EKFs.

Visual odometry occasionally reports large velocities WITHOUT losing
tracking. E.g. a person walking through the camera's view reads as
0.6-0.8 m/s "self-motion" (measured on the bench: two such spikes turned
into a 93 cm EKF position jump).  The statistical (Mahalanobis) gate in
robot_localization cannot reject these because VO genuinely measured
them with good confidence.

This node drops samples the PLATFORM cannot physically produce:
  - |v| > max_linear or |w| > max_angular  (rover tops out ~0.7 m/s)
  - |v - v_prev| > max_delta_v within one sample (impossible acceleration)

Dropped samples are simply not republished, the EKF predicts through
the gap, which is exactly the desired behavior.

ZERO-VELOCITY UPDATE (ZUPT)
---------------------------
Visual odometry never reports exactly zero on a stationary rover: depth
noise and feature drift leave a small residual velocity, and integrating
it walks the pose steadily in one direction.  rtabmap has no built-in
zero-velocity detection (its ZUPT parameters exist only for the OpenVINS
backend, which is not built here), so it is added at this seam.

When the rover is known to be stationary, the twist is republished as
exact zero with a small covariance rather than suppressed.  That
distinction matters: suppressing the message lets the EKF coast on its
last velocity estimate, which is the very drift being removed, whereas a
zeroed measurement actively pulls the filter's velocity state to zero.

Stationarity is taken from /cmd_vel when it is available, because a
commanded zero is unambiguous and needs no thresholds.  With no commands
arriving at all (bench testing, no motors) the node falls back to
treating the rover as stationary, which is the correct assumption when
nothing can drive it.  Set require_cmd_vel:=true to disable that fallback
if something else can move the rover.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class VioGate(Node):
    def __init__(self):
        super().__init__('vio_gate')
        self.declare_parameter('input_topic', '/vio/odometry_raw')
        self.declare_parameter('output_topic', '/vio/odometry')
        self.declare_parameter('max_linear', 1.0)     # m/s, > rover top speed
        self.declare_parameter('max_angular', 2.5)    # rad/s
        self.declare_parameter('max_delta_v', 0.5)    # m/s between samples
        # --- ZUPT ---
        self.declare_parameter('zupt_enabled', True)
        # seconds of continuous zero command before declaring stationary
        self.declare_parameter('zupt_settle', 0.5)
        # a commanded velocity below this counts as zero
        self.declare_parameter('zupt_cmd_epsilon', 0.01)
        # if no /cmd_vel has EVER arrived, assume stationary (bench default)
        self.declare_parameter('require_cmd_vel', False)
        # variance reported on the zeroed twist: small, so the EKF believes it
        self.declare_parameter('zupt_variance', 1.0e-4)

        self.max_lin = float(self.get_parameter('max_linear').value)
        self.max_ang = float(self.get_parameter('max_angular').value)
        self.max_dv = float(self.get_parameter('max_delta_v').value)

        self.pub = self.create_publisher(
            Odometry, self.get_parameter('output_topic').value, 10)
        self.create_subscription(
            Odometry, self.get_parameter('input_topic').value, self._cb, 10)

        # start from rest: the rover is stationary at launch, so the first
        # accepted sample must also be near zero (blocks VO init spikes)
        self._prev_v = 0.0
        self._passed = 0
        self._dropped = 0
        self._zupted = 0

        self.zupt_enabled = bool(self.get_parameter('zupt_enabled').value)
        self.zupt_settle = float(self.get_parameter('zupt_settle').value)
        self.zupt_eps = float(self.get_parameter('zupt_cmd_epsilon').value)
        self.require_cmd = bool(self.get_parameter('require_cmd_vel').value)
        self.zupt_var = float(self.get_parameter('zupt_variance').value)
        self._last_moving_cmd = None   # time of last non-zero command
        self._cmd_v = 0.0              # last COMMANDED linear speed, m/s
        self._seen_cmd = False
        self.create_subscription(Twist, 'cmd_vel', self._cmd_cb, 10)

        self.create_timer(30.0, self._report)

    def _cmd_cb(self, msg: Twist):
        self._seen_cmd = True
        self._cmd_v = math.hypot(msg.linear.x, msg.linear.y)
        moving = (abs(msg.linear.x) > self.zupt_eps
                  or abs(msg.linear.y) > self.zupt_eps
                  or abs(msg.angular.z) > self.zupt_eps)
        if moving:
            self._last_moving_cmd = self.get_clock().now()

    def _is_stationary(self):
        if not self.zupt_enabled:
            return False
        if not self._seen_cmd:
            # nothing is commanding the rover at all
            return not self.require_cmd
        if self._last_moving_cmd is None:
            return True
        age = (self.get_clock().now() - self._last_moving_cmd).nanoseconds * 1e-9
        return age > self.zupt_settle

    def _cb(self, msg: Odometry):
        v = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        w = abs(msg.twist.twist.angular.z)

        ok = v <= self.max_lin and w <= self.max_ang
        if ok and self._prev_v is not None and \
                abs(v - self._prev_v) > self.max_dv:
            ok = False

        if ok and self._is_stationary():
            # Zero-velocity update: republish an exact zero twist so the
            # filter is actively held at rest instead of integrating noise.
            msg.twist.twist.linear.x = 0.0
            msg.twist.twist.linear.y = 0.0
            msg.twist.twist.linear.z = 0.0
            msg.twist.twist.angular.x = 0.0
            msg.twist.twist.angular.y = 0.0
            msg.twist.twist.angular.z = 0.0
            cov = list(msg.twist.covariance)
            for i in (0, 7, 14, 21, 28, 35):
                cov[i] = self.zupt_var
            msg.twist.covariance = cov
            self._prev_v = 0.0
            self._passed += 1
            self._zupted += 1
            self.pub.publish(msg)
        elif ok:
            self._prev_v = v
            self._passed += 1
            self.pub.publish(msg)
        else:
            self._dropped += 1
            self.get_logger().warn(
                f'dropped implausible VO sample: |v|={v:.2f} m/s '
                f'|w|={w:.2f} rad/s',
                throttle_duration_sec=5.0)
            # Decay the reference toward what we COMMANDED, never toward the
            # sample we just rejected.  Decaying toward the rejected value is a
            # ratchet: a sustained stream of garbage drags _prev_v upward until
            # the garbage itself falls inside max_delta_v and starts being
            # ACCEPTED.  Measured 2026-09-02 with the rover physically parked --
            # the gate held for 38 s, then let a burst through and the EKF
            # integrated 10.35 m of phantom odometry in the next 40 s.
            #
            # The commanded speed is an honest reference and preserves the
            # original intent: it is large exactly when a genuine hard
            # acceleration could legitimately need to re-enter, and ~0 when the
            # rover is parked and every large sample is by definition garbage.
            if self._prev_v is not None:
                self._prev_v = (0.7 * self._prev_v
                                + 0.3 * min(self._cmd_v, self.max_lin))

    def _report(self):
        total = self._passed + self._dropped
        if not total:
            return
        bits = [f'{self._dropped}/{total} dropped']
        if self._zupted:
            bits.append(f'{self._zupted} zero-velocity held')
        if self._dropped or self._zupted:
            self.get_logger().info('gate: ' + ', '.join(bits))


def main(args=None):
    rclpy.init(args=args)
    node = VioGate()
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
