#!/usr/bin/env python3
"""
move.py — differential-drive serial bridge for the Athena rover.

Converts /cmd_vel (geometry_msgs/Twist) into per-side signed PWM and streams
it to the Pico W motor controller (firmware/sketch_feb10a v2):

    V <left> <right>\\n      left/right in -255..255, positive = forward

Replaces the old bang-bang F/B/L/R/S quantizer so Nav2's regulated
velocities (RPP slowing into turns, approach scaling, velocity smoother
ramps) actually reach the wheels.

Design:
  - Skid-steer kinematics:  v_l = v - w*W/2,  v_r = v + w*W/2
  - Speeds normalize against max_wheel_speed (m/s at PWM 255) — calibrate
    this once by driving 'V 255 255' for a measured distance/time.
  - min_pwm is the static-friction deadband: any nonzero wheel speed maps
    into [min_pwm, 255] so small corrections still turn the wheels.
  - If a wheel saturates, BOTH sides scale down together so the commanded
    curvature is preserved (turning radius stays right, speed drops).
  - Commands stream at send_rate (20 Hz) regardless of cmd_vel timing;
    the firmware watchdog (500 ms) then covers host/cable death, and this
    node's own watchdog zeros the target if cmd_vel stops arriving.
  - legacy_protocol:=true falls back to F/B/L/R/S for un-flashed firmware.
  - dry_run:=true logs instead of opening serial (bench testing).

The old move.py opened /dev/ttyACM0 — that is the PIXHAWK on this rover.
This version uses the Pico's stable /dev/serial/by-id/ path.
"""

import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

PICO_BY_ID = '/dev/ttyACM1'


class DiffDriveBridge(Node):
    def __init__(self):
        super().__init__('move')

        self.declare_parameter('port', PICO_BY_ID)
        self.declare_parameter('baud', 115200)
        self.declare_parameter('send_rate', 20.0)        # Hz
        self.declare_parameter('track_width', 0.40)      # m, wheel separation
        self.declare_parameter('max_wheel_speed', 0.7)   # m/s at PWM 255
        self.declare_parameter('min_pwm', 40)            # static-friction deadband
        self.declare_parameter('watchdog_timeout', 0.5)  # s without cmd_vel -> stop
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)
        self.declare_parameter('legacy_protocol', False)
        self.declare_parameter('dry_run', False)

        self.track = float(self.get_parameter('track_width').value)
        self.vmax = float(self.get_parameter('max_wheel_speed').value)
        self.min_pwm = int(self.get_parameter('min_pwm').value)
        self.wdt = float(self.get_parameter('watchdog_timeout').value)
        self.inv_l = -1 if self.get_parameter('invert_left').value else 1
        self.inv_r = -1 if self.get_parameter('invert_right').value else 1
        self.legacy = bool(self.get_parameter('legacy_protocol').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)

        self._lock = threading.Lock()
        self._target = (0.0, 0.0)          # (v, w) latest command
        self._last_cmd_time = None
        self._last_line = None

        self.ser = None
        if not self.dry_run:
            self._open_serial()
        else:
            self.get_logger().warn('DRY RUN: serial disabled, commands are logged.')

        self.create_subscription(Twist, 'cmd_vel', self._cmd_cb, 10)

        rate = float(self.get_parameter('send_rate').value)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)

        if self.ser is not None:
            threading.Thread(target=self._reader_loop, daemon=True).start()

        self.get_logger().info(
            f'diff-drive bridge up: track={self.track} m, '
            f'max_wheel_speed={self.vmax} m/s, min_pwm={self.min_pwm}, '
            f'protocol={"legacy F/B/L/R" if self.legacy else "V <l> <r>"}')

    # ------------------------------------------------------------- serial
    def _open_serial(self):
        import serial
        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)
        try:
            # dsrdtr=None avoids auto-reset on open (from the old move.py)
            self.ser = serial.Serial(port, baud, timeout=0.2, dsrdtr=None)
            time.sleep(2)
            self.get_logger().info(f'Serial open: {port}')
        except Exception as e:
            self.get_logger().error(
                f'Failed to open {port}: {e} — running as if dry_run')
            self.ser = None

    def _reader_loop(self):
        """Surface firmware messages (WDT stop, ERR ...) in the ROS log."""
        while rclpy.ok() and self.ser is not None:
            try:
                line = self.ser.readline().decode('utf-8', 'ignore').strip()
            except Exception:
                return
            if not line:
                continue
            if line.startswith('ERR') or 'WDT stop' in line:
                self.get_logger().warn(f'pico: {line}')
            else:
                self.get_logger().debug(f'pico: {line}')

    def _send_line(self, line: str):
        if line != self._last_line:
            self.get_logger().info(f'-> {line}', throttle_duration_sec=0.5)
            self._last_line = line
        if self.dry_run or self.ser is None:
            return
        try:
            self.ser.write((line + '\n').encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f'serial write failed: {e}')

    # ------------------------------------------------------------- control
    def _cmd_cb(self, msg: Twist):
        with self._lock:
            self._target = (msg.linear.x, msg.angular.z)
            self._last_cmd_time = self.get_clock().now()

    def _twist_to_pwm(self, v, w):
        """(v, w) -> (left_pwm, right_pwm), curvature-preserving."""
        vl = v - w * self.track / 2.0
        vr = v + w * self.track / 2.0

        # saturate together so the turn radius is preserved
        peak = max(abs(vl), abs(vr))
        if peak > self.vmax:
            scale = self.vmax / peak
            vl *= scale
            vr *= scale

        def to_pwm(ws):
            if abs(ws) < 1e-3:
                return 0
            mag = self.min_pwm + (abs(ws) / self.vmax) * (255 - self.min_pwm)
            return int(round(mag)) * (1 if ws >= 0 else -1)

        return to_pwm(vl) * self.inv_l, to_pwm(vr) * self.inv_r

    def _quantize_legacy(self, v, w):
        if v > 0.03:
            return 'F'
        if v < -0.03:
            return 'B'
        if w > 0.05:
            return 'L'
        if w < -0.05:
            return 'R'
        return 'S'

    def _tick(self):
        with self._lock:
            v, w = self._target
            last = self._last_cmd_time

        # host-side watchdog: no cmd_vel -> command zero
        if last is None:
            v, w = 0.0, 0.0
        else:
            age = (self.get_clock().now() - last).nanoseconds * 1e-9
            if age > self.wdt:
                v, w = 0.0, 0.0

        if self.legacy:
            self._send_line(self._quantize_legacy(v, w))
        else:
            left, right = self._twist_to_pwm(v, w)
            self._send_line(f'V {left} {right}')

    def destroy_node(self):
        if self.ser is not None:
            try:
                self.ser.write(b'S\n')
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveBridge()
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
