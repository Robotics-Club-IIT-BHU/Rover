#!/usr/bin/env python3
"""
tf_check.py

Verifies that the TF tree is not just CONNECTED but correctly ORIENTED.

A tree can look perfect in RViz and still be wrong: a camera rotated 90
degrees, an IMU mounted upside down, or an optical frame that was never
rotated into the robot frame all produce a connected tree in which the
rover drives sideways relative to what it sees.  Each check below tests a
physical fact, not just the presence of a transform.

    ros2 run athena_gps_nav tf_check
    ros2 run athena_gps_nav tf_check --ros-args -p settle:=10.0

Subscribes (publishes nothing; `settle` seconds of listening, then it
prints a report and shuts itself down):
    /imu/data                                          sensor_msgs/Imu
    /camera/camera/depth/color/points_downsampled      sensor_msgs/PointCloud2
    /tf, /tf_static                                    via tf2_ros

Checks:
  * every expected edge exists, with its translation printed
  * REP-103 compliance: x forward, y left, z up
  * the camera optical frame is the standard -90/0/-90 rotation from its
    body frame (z forward, x right becomes x forward, y left)
  * gravity measured by the IMU points DOWN once rotated into base_link
  * the depth cloud lands in front of the robot (+x), not behind or beside
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu, PointCloud2
import tf2_ros

BE = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=5)

EXPECTED = [
    ('base_footprint', 'base_link'),
    ('base_link', 'camera_link'),
    ('base_link', 'imu_link'),
    ('base_link', 'gps_link'),
    ('camera_link', 'camera_camera_link'),
    ('odom', 'base_link'),
    ('map', 'odom'),
]

# REP-103 optical frame: roll -90, pitch 0, yaw -90 from the camera body
# frame.  Anything outside this tolerance rotates the whole depth cloud.
OPTICAL_TOLERANCE_DEG = 5

GRAVITY = 9.81
GRAVITY_TOLERANCE = 0.4     # m/s2
MAX_TILT_DEG = 30           # beyond this the rover is on a slope, not level
MIN_IMU_SAMPLES = 5

# The camera looks forward, so the cloud's centroid must sit ahead of
# base_link.  0.2 m clears the origin without demanding a distant scene.
MIN_CLOUD_FORWARD_M = 0.2
MIN_FINITE_POINTS = 10


def quat_to_rpy(q):
    sinr = 2 * (q.w * q.x + q.y * q.z)
    cosr = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)
    sinp = 2 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny = 2 * (q.w * q.z + q.x * q.y)
    cosy = 1 - 2 * (q.y * q.y + q.z * q.z)
    return roll, pitch, math.atan2(siny, cosy)


def rotate_by_quat(q, v):
    """Rotate vector v by quaternion q."""
    x, y, z = v
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    tx = 2 * (qy * z - qz * y)
    ty = 2 * (qz * x - qx * z)
    tz = 2 * (qx * y - qy * x)
    return (x + qw * tx + qy * tz - qz * ty,
            y + qw * ty + qz * tx - qx * tz,
            z + qw * tz + qx * ty - qy * tx)


class TfCheck(Node):
    def __init__(self):
        super().__init__('tf_check')
        self.declare_parameter('settle', 6.0)
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, self)
        self.imu = []
        self.cloud = None
        self.create_subscription(Imu, '/imu/data', self._imu, 20)
        self.create_subscription(
            PointCloud2, '/camera/camera/depth/color/points_downsampled',
            self._cloud, BE)
        self.create_timer(float(self.get_parameter('settle').value), self._run)
        self._done = False
        self.fails = 0

    def _imu(self, m):
        a = m.linear_acceleration
        self.imu.append((a.x, a.y, a.z))

    def _cloud(self, m):
        self.cloud = m

    def _line(self, ok, label, detail=''):
        if not ok:
            self.fails += 1
        print(f'  [{"PASS" if ok else "FAIL"}]  {label:<34s} {detail}', flush=True)

    def _run(self):
        if self._done:
            return
        self._done = True
        print('=' * 68)
        print('TF ORIENTATION CHECK')
        print('=' * 68)

        self._check_edges()
        self._check_optical_frame()
        self._check_gravity()
        self._check_cloud_forward()

        print('\n' + '=' * 68)
        print('  ALL TRANSFORMS OK' if not self.fails else f'  {self.fails} PROBLEM(S)')
        print('=' * 68)
        rclpy.shutdown()

    def _check_edges(self):
        """Every edge in EXPECTED exists, with its translation and rotation."""
        print('\nEDGES  (translation in metres, rotation in degrees)')
        for parent, child in EXPECTED:
            try:
                tr = self.buf.lookup_transform(parent, child, rclpy.time.Time())
            except Exception as e:
                self._line(False, f'{parent} -> {child}', f'MISSING ({type(e).__name__})')
                continue
            t = tr.transform.translation
            r, p, y = [math.degrees(v) for v in quat_to_rpy(tr.transform.rotation)]
            self._line(True, f'{parent} -> {child}',
                       f'xyz=({t.x:+.3f},{t.y:+.3f},{t.z:+.3f}) '
                       f'rpy=({r:+.1f},{p:+.1f},{y:+.1f})')

    def _check_optical_frame(self):
        """The camera optical frame must be the standard REP-103 rotation.

        If it is not, every point in the cloud is rotated, so obstacles land
        in the costmap beside or behind the rover instead of in front of it.
        """
        print('\nCAMERA OPTICAL FRAME  (must be z-forward vs body x-forward)')
        try:
            tr = self.buf.lookup_transform(
                'camera_link', 'camera_color_optical_frame', rclpy.time.Time())
            r, p, y = [math.degrees(v) for v in quat_to_rpy(tr.transform.rotation)]
            ok = (abs(abs(r) - 90) < OPTICAL_TOLERANCE_DEG
                  and abs(p) < OPTICAL_TOLERANCE_DEG
                  and abs(abs(y) - 90) < OPTICAL_TOLERANCE_DEG)
            self._line(ok, 'camera_link -> optical',
                       f'rpy=({r:+.1f},{p:+.1f},{y:+.1f}) '
                       f'{"standard" if ok else "NON-STANDARD - cloud will be rotated"}')
        except Exception:
            self._line(False, 'camera_link -> optical', 'not found')

    def _check_gravity(self):
        """Accelerometer, rotated into base_link, must point up (+z at rest).

        A wrong sign here means the IMU is mounted upside down or the MAVLink
        NED reading was never converted to ENU - either way the EKF's idea of
        which way is down is inverted.
        """
        # UP, not down. An accelerometer at rest measures specific force, so
        # it reads +g along the axis pointing away from the ground - REP-145
        # says roughly (0, 0, +9.81) in a z-up frame. The header used to say
        # DOWN, which contradicted this method's own docstring, its comment,
        # the test below (g[2] > 0) and its own pass message, so a correct
        # PASS read like a fault.
        print('\nGRAVITY  (accelerometer rotated into base_link must point UP)')
        if len(self.imu) < MIN_IMU_SAMPLES:
            self._line(False, 'imu samples', f'only {len(self.imu)}')
            return

        n = len(self.imu)
        mean = tuple(sum(s[i] for s in self.imu) / n for i in range(3))
        try:
            tr = self.buf.lookup_transform('base_link', 'imu_link',
                                           rclpy.time.Time())
        except Exception:
            self._line(False, 'base_link -> imu_link', 'not found')
            return

        g = rotate_by_quat(tr.transform.rotation, mean)
        mag = math.hypot(math.hypot(g[0], g[1]), g[2])
        # accelerometer at rest reads +g along "up"
        tilt = math.degrees(math.acos(
            max(-1.0, min(1.0, g[2] / mag)))) if mag > 1e-6 else 99
        self._line(abs(mag - GRAVITY) < GRAVITY_TOLERANCE, 'gravity magnitude',
                   f'|g| = {mag:.3f} m/s2')
        sign_note = ('(up-positive: correct for ROS)' if g[2] > 0
                     else '(INVERTED - IMU upside down, or NED '
                          'never converted to ENU)')
        self._line(g[2] > 0, 'gravity sign', f'z = {g[2]:+.2f} {sign_note}')
        self._line(tilt < MAX_TILT_DEG, 'chassis tilt',
                   f'{tilt:.1f} deg from vertical '
                   f'{"(level enough)" if tilt < MAX_TILT_DEG else "(is the rover on a slope?)"}')

    def _check_cloud_forward(self):
        """The depth cloud's centroid must land ahead of base_link.

        This is the end-to-end version of the optical-frame check: it uses
        the transform the costmap will actually use, on real points.
        """
        print('\nDEPTH CLOUD  (must appear in FRONT of the robot, +x)')
        if self.cloud is None:
            self._line(False, 'cloud received', 'no cloud on downsampled topic')
            return
        try:
            # Imported here so a missing numpy reports as one FAILED check
            # rather than stopping the whole node from starting.
            import numpy as np
            m = self.cloud
            pts = np.frombuffer(m.data, dtype=np.uint8).reshape(-1, m.point_step)
            off = {f.name: f.offset for f in m.fields}
            xs = pts[:, off['x']:off['x'] + 4].copy().view('<f4').reshape(-1)
            ys = pts[:, off['y']:off['y'] + 4].copy().view('<f4').reshape(-1)
            zs = pts[:, off['z']:off['z'] + 4].copy().view('<f4').reshape(-1)
            good = np.isfinite(xs) & np.isfinite(ys) & np.isfinite(zs)
            if good.sum() < MIN_FINITE_POINTS:
                self._line(False, 'cloud points', 'too few finite points')
                return

            tr = self.buf.lookup_transform(
                'base_link', m.header.frame_id, rclpy.time.Time())
            q = tr.transform.rotation
            t = tr.transform.translation
            cx = float(xs[good].mean())
            cy = float(ys[good].mean())
            cz = float(zs[good].mean())
            bx, by, bz = rotate_by_quat(q, (cx, cy, cz))
            bx += t.x
            by += t.y
            bz += t.z
            front = ('in front' if bx > MIN_CLOUD_FORWARD_M else
                     'NOT IN FRONT - camera frame rotated wrong')
            self._line(bx > MIN_CLOUD_FORWARD_M, 'cloud centroid in base_link',
                       f'({bx:+.2f},{by:+.2f},{bz:+.2f}) m  {front}')
            self._line(m.header.frame_id.endswith('optical_frame'),
                       'cloud frame', m.header.frame_id)
        except Exception as e:
            self._line(False, 'cloud transform', f'{type(e).__name__}: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TfCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
