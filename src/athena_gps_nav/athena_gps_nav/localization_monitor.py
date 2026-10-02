#!/usr/bin/env python3
"""
localization_monitor.py

Records the localization pipeline for `duration` seconds, then writes a
report (summary + CSV traces) and prints the summary.  Used both for bench
tests (compares /odometry/global to /bench/ground_truth if present) and on
the real rover (topic rates, stationary drift, GPS-vs-EKF residuals).

    ros2 run athena_gps_nav localization_monitor --ros-args \
        -p duration:=60.0 -p output_dir:=/home/robo/athena/loc_reports

Subscribes (publishes nothing; writes files and shuts itself down):
    /vio/odometry, /odometry/local, /odometry/global, /odometry/gps,
    /bench/ground_truth                             nav_msgs/Odometry
    /gps/fix                                        sensor_msgs/NavSatFix
    /imu/data                                       sensor_msgs/Imu

Writes report_<stamp>.txt plus one CSV per stream into output_dir.
"""

import math
import os
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix

# A step this fast is not motion, it is the estimator jumping.  The rover
# tops out near 0.7 m/s, so 3 m/s cannot be real.
DISCONTINUITY_SPEED = 3.0        # m/s

# Two samples further apart than this are not the same instant, so pairing
# them would measure clock skew rather than position error.
PAIR_WINDOW = 0.25               # s

MIN_SAMPLES = 10                 # below this a stream is not worth reporting
MIN_PAIRS = 5                    # residuals need this many matched samples

# Distance below which the rover counts as parked, so any net displacement
# in the window is drift rather than travel.
STATIONARY_PATH_LENGTH = 0.5     # m


def _yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class LocalizationMonitor(Node):
    TOPICS = {
        'vio': ('/vio/odometry', Odometry),
        'local': ('/odometry/local', Odometry),
        'global': ('/odometry/global', Odometry),
        'gps_odom': ('/odometry/gps', Odometry),
        'gps_fix': ('/gps/fix', NavSatFix),
        'imu': ('/imu/data', Imu),
        'truth': ('/bench/ground_truth', Odometry),
    }

    def __init__(self):
        super().__init__('localization_monitor')
        self.declare_parameter('duration', 60.0)
        self.declare_parameter('output_dir', '/home/robo/athena/loc_reports')
        self.duration = self.get_parameter('duration').value
        self.output_dir = self.get_parameter('output_dir').value

        self.data = {k: [] for k in self.TOPICS}
        self.t0 = time.time()

        for key, (topic, mtype) in self.TOPICS.items():
            self.create_subscription(
                mtype, topic,
                lambda msg, k=key: self._cb(k, msg), 20)

        self.create_timer(5.0, self._progress)
        self.create_timer(self.duration, self._finish)
        self.get_logger().info(
            f'Recording localization for {self.duration:.0f} s ...')
        self._done = False

    def _cb(self, key, msg):
        # use header stamps (not arrival time): arrival bursts would create
        # false velocity/discontinuity readings
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if isinstance(msg, Odometry):
            p = msg.pose.pose.position
            self.data[key].append(
                (t, p.x, p.y, _yaw_of(msg.pose.pose.orientation),
                 msg.twist.twist.linear.x, msg.twist.twist.angular.z))
        elif isinstance(msg, NavSatFix):
            self.data[key].append(
                (t, msg.latitude, msg.longitude,
                 msg.position_covariance[0], float(msg.status.status), 0.0))
        elif isinstance(msg, Imu):
            self.data[key].append(
                (t, _yaw_of(msg.orientation), msg.angular_velocity.z,
                 0.0, 0.0, 0.0))

    def _progress(self):
        left = self.duration - (time.time() - self.t0)
        counts = {k: len(v) for k, v in self.data.items() if v}
        self.get_logger().info(f'{left:5.0f} s left | samples: {counts}')

    # ------------------------------------------------------------------ report
    def _finish(self):
        if self._done:
            return
        self._done = True
        os.makedirs(self.output_dir, exist_ok=True)
        stamp = time.strftime('%Y%m%d_%H%M%S')

        lines = ['=' * 64,
                 f'Athena localization report  {stamp}',
                 f'window: {self.duration:.0f} s',
                 '=' * 64]
        lines += self._rate_lines()
        lines += self._local_odom_lines()
        lines += self._yaw_wander_lines()
        lines += self._residual_lines()

        report = '\n'.join(lines)
        rpt_path = self._write_files(report, stamp)

        print('\n' + report)
        print(f'\nreport written to {rpt_path}')
        rclpy.shutdown()

    def _rate_lines(self):
        lines = ['\n[topic rates]']
        for key, (topic, _) in self.TOPICS.items():
            n = len(self.data[key])
            rate = n / self.duration
            lines.append(f'  {topic:35s} {n:6d} msgs  ({rate:6.1f} Hz)')
        return lines

    def _local_odom_lines(self):
        """Smoothness of /odometry/local, and drift if the rover was parked."""
        loc = self.data['local']
        if len(loc) <= MIN_SAMPLES:
            return []

        dist = 0.0
        jumps = 0
        for a, b in zip(loc[:-1], loc[1:]):
            step = math.hypot(b[1] - a[1], b[2] - a[2])
            dist += step
            dt = b[0] - a[0]
            if dt > 1e-4 and step / dt > DISCONTINUITY_SPEED:
                jumps += 1
        net = math.hypot(loc[-1][1] - loc[0][1], loc[-1][2] - loc[0][2])

        lines = ['\n[/odometry/local]',
                 f'  path length     : {dist:8.2f} m',
                 f'  net displacement: {net:8.2f} m',
                 f'  discontinuities : {jumps} (steps '
                 f'>{DISCONTINUITY_SPEED:.0f} m/s)']
        if dist < STATIONARY_PATH_LENGTH:
            lines.append(
                f'  STATIONARY DRIFT: {net * 100:6.1f} cm over '
                f'{self.duration:.0f} s '
                f'({net * 100 / self.duration * 60:.1f} cm/min)')
        yaw_stats = self._yaw_stats(loc, idx=3)
        if yaw_stats:
            net_deg, span_deg = yaw_stats
            lines.append(
                f'  yaw drift       : {net_deg:+7.2f} deg net, '
                f'{span_deg:6.2f} deg span '
                f'({net_deg / self.duration * 60:+.2f} deg/min)')
        return lines

    def _yaw_wander_lines(self):
        """Heading wander of the two independent yaw sources.

        The IMU column diagnoses the magnetometer environment (steel, motors,
        power cabling); the VIO column diagnoses visual tracking.  Comparing
        them tells you which one to distrust when the fused yaw is wrong.
        """
        lines = []
        for key, idx, heading, label in (
                ('imu', 1, '\n[/imu/data compass yaw]', '  wander          :'),
                ('vio', 3, '\n[/vio/odometry yaw]', '  drift           :')):
            rows = self.data[key]
            if len(rows) <= MIN_SAMPLES:
                continue
            yaw_stats = self._yaw_stats(rows, idx=idx)
            if not yaw_stats:
                continue
            net_deg, span_deg = yaw_stats
            lines.append(heading)
            lines.append(
                f'{label} {net_deg:+7.2f} deg net, '
                f'{span_deg:6.2f} deg span over {self.duration:.0f} s')
        return lines

    def _residual_lines(self):
        """Global EKF against GPS, and against bench ground truth if present."""
        lines = []
        res = self._residual('global', 'gps_odom')
        if res:
            rms, mean, mx = res
            lines += ['\n[/odometry/global vs /odometry/gps]',
                      f'  residual RMS  : {rms:6.2f} m',
                      f'  residual mean : {mean:6.2f} m',
                      f'  residual max  : {mx:6.2f} m']

        res = self._residual('global', 'truth')
        if res:
            rms, mean, mx = res
            lines += ['\n[/odometry/global vs ground truth (bench)]',
                      f'  abs error RMS : {rms:6.2f} m',
                      f'  abs error mean: {mean:6.2f} m',
                      f'  abs error max : {mx:6.2f} m']
            # A constant offset here is the datum initialising a metre or two
            # off, not the filter tracking badly; removing it separates the
            # two so a good tracker is not condemned by a bad start.
            res2 = self._residual('global', 'truth', remove_offset=True)
            if res2:
                lines.append(
                    f'  offset-removed RMS: {res2[0]:6.2f} m '
                    '(datum-init error removed)')
        return lines

    def _write_files(self, report, stamp):
        """Write the text report plus one CSV per stream. Returns the report path."""
        rpt_path = os.path.join(self.output_dir, f'report_{stamp}.txt')
        with open(rpt_path, 'w') as f:
            f.write(report + '\n')
        for key in ('local', 'global', 'gps_odom', 'truth', 'vio', 'imu'):
            if self.data[key]:
                csv_path = os.path.join(
                    self.output_dir, f'{key}_{stamp}.csv')
                with open(csv_path, 'w') as f:
                    f.write('t,x,y,yaw,vx,wz\n')
                    for row in self.data[key]:
                        f.write(','.join(f'{v:.6f}' for v in row) + '\n')
        return rpt_path

    @staticmethod
    def _yaw_stats(rows, idx):
        """Unwrapped net change and span (deg) of a yaw column."""
        if len(rows) < MIN_SAMPLES:
            return None
        unwrapped = [rows[0][idx]]
        prev_raw = rows[0][idx]
        for row in rows[1:]:
            d = row[idx] - prev_raw
            d = math.atan2(math.sin(d), math.cos(d))  # shortest arc
            unwrapped.append(unwrapped[-1] + d)
            prev_raw = row[idx]
        net = math.degrees(unwrapped[-1] - unwrapped[0])
        span = math.degrees(max(unwrapped) - min(unwrapped))
        return net, span

    def _residual(self, key_a, key_b, remove_offset=False):
        """Position residual between two Odometry streams (nearest-time)."""
        a, b = self.data[key_a], self.data[key_b]
        if len(a) < MIN_PAIRS or len(b) < MIN_PAIRS:
            return None
        pairs = []
        j = 0
        for row in b:
            while j + 1 < len(a) and a[j + 1][0] <= row[0]:
                j += 1
            if abs(a[j][0] - row[0]) < PAIR_WINDOW:
                pairs.append((a[j][1] - row[1], a[j][2] - row[2]))
        if len(pairs) < MIN_PAIRS:
            return None
        if remove_offset:
            mx = sum(p[0] for p in pairs) / len(pairs)
            my = sum(p[1] for p in pairs) / len(pairs)
            pairs = [(p[0] - mx, p[1] - my) for p in pairs]
        errs = [math.hypot(px, py) for px, py in pairs]
        rms = math.sqrt(sum(e * e for e in errs) / len(errs))
        return rms, sum(errs) / len(errs), max(errs)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._finish()


if __name__ == '__main__':
    main()
