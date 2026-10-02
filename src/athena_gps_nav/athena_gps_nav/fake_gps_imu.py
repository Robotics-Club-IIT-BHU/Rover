#!/usr/bin/env python3
"""
fake_gps_imu.py

Bench-test sensor simulator: drives a virtual rover around a rectangular
outdoor loop and publishes exactly the topics the real sensor stack produces:

    /gps/fix        5 Hz   NavSatFix, Gaussian noise, real covariance
    /imu/data      50 Hz   Imu (ENU orientation + yaw-rate + accel)
    /vio/odometry  20 Hz   Odometry (noisy body velocities, drifting pose)
    /bench/ground_truth 20 Hz  Odometry (true ENU pose rel. to datum)

This validates the dual-EKF + navsat_transform + Nav2 pipeline indoors,
without GPS reception or the camera.  Compare /odometry/global against
/bench/ground_truth (localization_monitor does this automatically).

The virtual rover: 20 m x 10 m rectangle, 0.5 m/s cruise, turns in place.
Datum matches the real test site so /fromLL sanity checks are realistic.
"""

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus

R_EARTH = 6378137.0
G = 9.80665

# Rectangle corners (ENU metres from the datum) the virtual rover drives
# around, and the tolerances for deciding it has arrived / finished turning.
LOOP_CORNERS = [(0.0, 0.0), (20.0, 0.0), (20.0, 10.0), (0.0, 10.0)]
CORNER_REACHED_M = 0.15
HEADING_REACHED_RAD = 0.05

# One 50 Hz master timer drives everything; the slower streams are published
# every Nth tick so their rates stay in a fixed ratio to the IMU.
TICK_DT = 0.02          # s, 50 Hz
GPS_EVERY_N_TICKS = 10  # 5 Hz
VIO_EVERY_N_TICKS = 2   # 25 Hz, close enough to the real ~20 Hz VIO


class FakeGpsImu(Node):
    def __init__(self):
        super().__init__('fake_gps_imu')
        self.declare_parameter('datum_lat', 25.2629548)
        self.declare_parameter('datum_lon', 82.9838284)
        self.declare_parameter('gps_noise_std', 1.2)     # m
        self.declare_parameter('yaw_noise_std', 0.03)    # rad (~1.7 deg)
        self.declare_parameter('vio_vel_noise_std', 0.02)  # m/s
        self.declare_parameter('vio_scale_error', 1.02)  # 2% VIO scale bias
        self.declare_parameter('speed', 0.5)             # m/s
        self.declare_parameter('turn_rate', 0.5)         # rad/s

        self.lat0 = self.get_parameter('datum_lat').value
        self.lon0 = self.get_parameter('datum_lon').value
        self.gps_std = self.get_parameter('gps_noise_std').value
        self.yaw_std = self.get_parameter('yaw_noise_std').value
        self.vio_std = self.get_parameter('vio_vel_noise_std').value
        self.vio_scale = self.get_parameter('vio_scale_error').value
        self.speed = self.get_parameter('speed').value
        self.turn_rate = self.get_parameter('turn_rate').value

        self.corners = LOOP_CORNERS
        self.leg = 0            # current corner index
        self.state = 'drive'    # 'drive' | 'turn'
        self.x, self.y = 0.0, 0.0
        self.yaw = 0.0
        self.v, self.w = 0.0, 0.0

        # deterministic noise (reproducible runs)
        self._rng_state = 12345

        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.vio_pub = self.create_publisher(Odometry, '/vio/odometry', 10)
        self.gt_pub = self.create_publisher(Odometry, '/bench/ground_truth', 10)

        # VIO integrates its own noisy velocities -> drifting pose like real VO
        self.vio_x, self.vio_y, self.vio_yaw = 0.0, 0.0, 0.0

        self.dt = TICK_DT
        self.create_timer(self.dt, self._step)          # 50 Hz master clock
        self._tick = 0
        self.get_logger().info('Fake sensor simulator running (rectangle loop)')

    # simple LCG + Box-Muller for reproducible gaussian noise
    def _rand(self):
        self._rng_state = (1103515245 * self._rng_state + 12345) % (2 ** 31)
        return self._rng_state / (2 ** 31)

    def _gauss(self, std):
        u1 = max(self._rand(), 1e-9)
        u2 = self._rand()
        return std * math.sqrt(-2.0 * math.log(u1)) * \
            math.cos(2.0 * math.pi * u2)

    # ------------------------------------------------------------- dynamics
    def _step(self):
        """One 50 Hz tick: advance the virtual rover, then publish."""
        self._advance()

        now = self.get_clock().now().to_msg()
        self._publish_imu(now)
        self._tick += 1
        if self._tick % GPS_EVERY_N_TICKS == 0:
            self._publish_gps(now)
        if self._tick % VIO_EVERY_N_TICKS == 0:
            self._publish_vio(now)
            self._publish_gt(now)

    def _advance(self):
        """Drive-then-turn-in-place around the rectangle, one tick at a time.

        Turning in place rather than arcing is deliberate: it is what a
        skid-steer rover does, and it exercises the yaw path of the EKFs
        with no translation to hide behind.
        """
        target = self.corners[(self.leg + 1) % 4]
        if self.state == 'drive':
            dx, dy = target[0] - self.x, target[1] - self.y
            dist = math.hypot(dx, dy)
            if dist < CORNER_REACHED_M:
                self.leg = (self.leg + 1) % 4
                self.state = 'turn'
            else:
                self.v, self.w = self.speed, 0.0
                self.x += self.v * math.cos(self.yaw) * self.dt
                self.y += self.v * math.sin(self.yaw) * self.dt
        if self.state == 'turn':
            target = self.corners[(self.leg + 1) % 4]
            desired = math.atan2(target[1] - self.y, target[0] - self.x)
            err = math.atan2(math.sin(desired - self.yaw),
                             math.cos(desired - self.yaw))
            if abs(err) < HEADING_REACHED_RAD:
                self.state = 'drive'
                self.v, self.w = self.speed, 0.0
            else:
                self.v = 0.0
                self.w = self.turn_rate if err > 0 else -self.turn_rate
                self.yaw += self.w * self.dt
                self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

    # ------------------------------------------------------------- publishers
    def _publish_gps(self, stamp):
        nx = self._gauss(self.gps_std)
        ny = self._gauss(self.gps_std)
        lat = self.lat0 + (self.y + ny) / R_EARTH * 180.0 / math.pi
        lon = self.lon0 + (self.x + nx) / (
            R_EARTH * math.cos(math.radians(self.lat0))) * 180.0 / math.pi

        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = 'gps_link'
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = lat
        fix.longitude = lon
        fix.altitude = 80.0
        v = self.gps_std ** 2
        fix.position_covariance = [v, 0.0, 0.0, 0.0, v, 0.0, 0.0, 0.0, 4.0 * v]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.gps_pub.publish(fix)

    def _publish_imu(self, stamp):
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'imu_link'
        yaw_meas = self.yaw + self._gauss(self.yaw_std)
        imu.orientation.z = math.sin(yaw_meas / 2.0)
        imu.orientation.w = math.cos(yaw_meas / 2.0)
        imu.orientation_covariance = [
            0.005, 0.0, 0.0, 0.0, 0.005, 0.0, 0.0, 0.0, 0.05]
        imu.angular_velocity.z = self.w + self._gauss(0.01)
        imu.angular_velocity_covariance = [
            0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
        imu.linear_acceleration.z = G + self._gauss(0.05)
        imu.linear_acceleration.x = self._gauss(0.05)
        imu.linear_acceleration.y = self._gauss(0.05)
        imu.linear_acceleration_covariance = [
            0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.05]
        self.imu_pub.publish(imu)

    def _publish_vio(self, stamp):
        v_meas = self.v * self.vio_scale + self._gauss(self.vio_std)
        w_meas = self.w + self._gauss(0.005)

        # VIO's own drifting dead-reckoned pose (like real VO output)
        vdt = self.dt * 2
        self.vio_yaw += w_meas * vdt
        self.vio_x += v_meas * math.cos(self.vio_yaw) * vdt
        self.vio_y += v_meas * math.sin(self.vio_yaw) * vdt

        od = Odometry()
        od.header.stamp = stamp
        od.header.frame_id = 'odom'
        od.child_frame_id = 'base_link'
        od.pose.pose.position.x = self.vio_x
        od.pose.pose.position.y = self.vio_y
        od.pose.pose.orientation.z = math.sin(self.vio_yaw / 2.0)
        od.pose.pose.orientation.w = math.cos(self.vio_yaw / 2.0)
        od.twist.twist.linear.x = v_meas
        od.twist.twist.angular.z = w_meas
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.01
        cov[35] = 0.01
        od.pose.covariance = cov
        tcov = [0.0] * 36
        tcov[0] = tcov[7] = self.vio_std ** 2
        tcov[35] = 0.005 ** 2
        od.twist.covariance = tcov
        self.vio_pub.publish(od)

    def _publish_gt(self, stamp):
        gt = Odometry()
        gt.header.stamp = stamp
        gt.header.frame_id = 'map'
        gt.child_frame_id = 'base_link'
        gt.pose.pose.position.x = self.x
        gt.pose.pose.position.y = self.y
        gt.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        gt.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        gt.twist.twist.linear.x = self.v
        gt.twist.twist.angular.z = self.w
        self.gt_pub.publish(gt)


def main(args=None):
    rclpy.init(args=args)
    node = FakeGpsImu()
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
