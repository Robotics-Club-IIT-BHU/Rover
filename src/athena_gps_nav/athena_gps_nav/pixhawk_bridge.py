#!/usr/bin/env python3
"""
pixhawk_bridge.py

Bridges the flight controller (ArduPilot FC with a DroneCAN RTK GPS; the old
PX4 FMU on /dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00 still works)
to ROS 2 via pymavlink and publishes:

    /gps/fix   (sensor_msgs/NavSatFix)  - from GPS_RAW_INT; covariance from the
                                          receiver's hAcc when RTK, else HDOP
    /imu/data  (sensor_msgs/Imu)        - from HIGHRES_IMU / SCALED_IMU2 /
                                          SCALED_IMU + ATTITUDE

RTK: RTCM3 corrections are pulled from the base station's TCP stream
(STRSVR / str2str) and injected into the FC as GPS_RTCM_DATA over this same
MAVLink connection (only one process may own the serial port).  See rtcm.py.
FC params: GPS_INJECT_TO=127, GPS1_TYPE=9 (DroneCAN), MAVLink 2 on the port.

Based on athena_slam/gpsimusplit.py + imupub.py, with two important fixes for
robot_localization / navsat_transform compatibility (REP-103 / REP-145):

  1. MAVLink reports body rates/accels in FRD (front-right-down) and attitude
     relative to NED (yaw 0 = North, clockwise-positive).  ROS expects FLU body
     axes and ENU-referenced orientation (yaw 0 = East, counter-clockwise).
     This node converts:  ENU yaw = pi/2 - NED yaw,  y/z body axes negated.
  2. This firmware streams HIGHRES_IMU (already SI units), not RAW_IMU.  The
     node requests both HIGHRES_IMU and SCALED_IMU2 and uses whichever arrives.

No fusion is done here - this sits upstream of the EKFs.
"""

import glob
import math
import os
import threading
import time

# MAVLink 2 BEFORE pymavlink is imported: the default v1.0 dialect does not
# decode the GPS_RAW_INT extension fields (h_acc, v_acc) that RTK needs.
os.environ.setdefault('MAVLINK20', '1')

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # noqa: E402

from geometry_msgs.msg import Twist  # noqa: E402
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus  # noqa: E402
from pymavlink import mavutil  # noqa: E402

from athena_gps_nav.rtcm import RtcmSender, RtcmTcpClient  # noqa: E402

G = 9.80665

FIX_NAMES = {0: 'NO GPS', 1: 'NO FIX', 2: '2D', 3: '3D', 4: 'DGPS',
             5: 'RTK FLOAT', 6: 'RTK FIXED', 7: 'STATIC', 8: 'PPP'}


def _patch_pymavlink_instances():
    """Work around a pymavlink crash in mavutil.add_message().

    If the first message of an instance-carrying type arrives with a NULL
    instance field, pymavlink stores it via the simple path, leaving
    `_instances` as None.  The next message of that type with a real
    instance value then does `None[value] = msg` and raises
    TypeError, killing the receive loop.  (Seen on this Pixhawk within
    seconds of connecting; pymavlink 2.4.49.)
    """
    original = mavutil.add_message

    def safe_add_message(messages, mtype, msg):
        try:
            return original(messages, mtype, msg)
        except TypeError:
            prev = messages.get(mtype)
            if prev is not None and getattr(prev, '_instances', 0) is None:
                prev._instances = {}
                return original(messages, mtype, msg)
            raise

    # mavutil calls add_message() as a module global, so rebinding here
    # is picked up by the receive loop.
    mavutil.add_message = safe_add_message


_patch_pymavlink_instances()

PIXHAWK_BY_ID = '/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00'


def find_fc_port():
    """Pick the flight controller's serial device.

    Prefers a stable /dev/serial/by-id name (an ArduPilot-family FC, then the
    old PX4 FMU) over /dev/ttyACM*, whose numbering moves around when the
    Pico or other CDC devices enumerate first.  Override with the `port`
    parameter.
    """
    ids = sorted(glob.glob('/dev/serial/by-id/*'))
    for pat in ('ArduPilot', 'Ardu', 'CubeOrange', 'Cube', 'Pixhawk', 'PX4'):
        for p in ids:
            if pat.lower() in os.path.basename(p).lower():
                return p
    return PIXHAWK_BY_ID


def _msg_id(name, fallback):
    """MAVLink message ID by name, falling back to the standard numeric ID.

    See the comment in _request_streams: which symbols exist depends on the
    dialect pymavlink happened to load, but the numbers are fixed by the
    protocol, so the fallback is always correct.
    """
    return getattr(mavutil.mavlink, f'MAVLINK_MSG_ID_{name}', fallback)


class PixhawkBridge(Node):
    def __init__(self):
        super().__init__('pixhawk_bridge')

        # '' = auto-detect via find_fc_port()
        self.declare_parameter('port', '')
        # 921600, NOT 115200. Measured on this FMU: at 115200 the link
        # saturates and a THIRD of all MAVLink packets arrive corrupt
        # (33% BAD_DATA, ~5 usable IMU samples/s); at 921600 corruption
        # drops to 8% and throughput rises ~12x. The old athena_slam
        # imupub.py/gpspub.py used 115200, which is why they were flaky.
        self.declare_parameter('baud', 921600)
        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('gps_frame_id', 'gps_link')
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('imu_rate_hz', 50)
        self.declare_parameter('gps_rate_hz', 5)
        # User Equivalent Range Error: sigma_pos ~= HDOP * UERE.
        # GPS_RAW_INT.eph is HDOP*100 on ArduPilot/PX4 (not centimetres).
        #
        # Error model:  sigma_h^2 = (HDOP * gps_uere)^2 + gps_sigma_floor^2
        #
        # The usual sigma = HDOP * UERE is NOT enough on this receiver.  Two
        # stationary 240 s runs (1202 and 1200 samples) measured:
        #     HDOP 0.98 -> actual 1-sigma 3.50 m
        #     HDOP 1.91 -> actual 1-sigma 4.52 m
        # A purely multiplicative model needs k=3.57 at the first and k=2.36
        # at the second, so no single UERE fits.  The reason is that a large
        # part of the error is slow multipath/ionospheric WANDER that HDOP
        # does not predict: within any 30 s window sigma is only ~1.3-1.8 m,
        # while the solution walks several metres over minutes.
        #
        # Splitting it into an HDOP-scaled term plus an HDOP-independent
        # floor reproduces both runs to the centimetre.  With the old flat
        # UERE=2.0 the EKF was told GPS was ~1.8x better than it really was
        # at good HDOP -- worst exactly when the filter trusts it most --
        # which is what let the map frame chase the wander.
        #
        # Re-derive both numbers if the antenna or site changes; see
        # "Setting the GPS error model honestly" in the README.
        self.declare_parameter('gps_uere', 1.74)
        self.declare_parameter('gps_sigma_floor', 3.06)
        # --- RTK ---------------------------------------------------------
        # With an RTK fix (fix_type >= 4) the receiver's own horizontal /
        # vertical accuracy estimate (GPS_RAW_INT.h_acc / v_acc, mm) is a
        # far better covariance than HDOP*UERE, which describes standalone
        # GPS.  The floor stops the EKF from trusting a centimetre-claimed
        # solution absolutely.
        self.declare_parameter('gps_rtk_sigma_floor', 0.02)    # metres
        self.declare_parameter('rtcm_enabled', True)
        default_rtcm_host = (
            os.environ.get('RTCM_HOST')
            or os.environ.get('BASE_STATION_IP')
            or '127.0.0.1'
        )
        self.declare_parameter('rtcm_host', default_rtcm_host)
        self.declare_parameter('rtcm_port', 2101)
        # seconds of base-stream silence before warning
        self.declare_parameter('rtcm_stale_warn', 10.0)
        # --- gyro zero-rate bias compensation ---------------------------
        # HIGHRES_IMU carries the raw rate; nothing upstream removes the
        # gyro's zero-rate offset.  Measured on this airframe 2026-09-02:
        # z bias = +0.0234 rad/s = +1.34 deg/s.  ekf_local integrates that
        # straight into heading, so the filter believed the rover was
        # turning left at ~80 deg/min while it sat still, and every goal's
        # heading error was computed against a yaw that was wrong and
        # growing -> the rover turned instead of driving.
        #
        # This is a stationary (ZUPT) bias estimator: while nothing is
        # commanding motion, the true rate is zero by definition, so the
        # mean of the gyro IS the bias.  It is the standard fix and it also
        # tracks the thermal drift a one-off calibration cannot.
        # It does NOT replace calibrating the board - 1.34 deg/s is huge for
        # a MEMS gyro and should be a few hundredths after a proper
        # PX4 gyro calibration - but it keeps the stack usable meanwhile.
        self.declare_parameter('gyro_bias_correction', True)
        # samples of continuous stillness before the estimate is trusted
        self.declare_parameter('gyro_bias_min_samples', 100)
        # window length; also the EMA time constant once it is full
        self.declare_parameter('gyro_bias_window', 1000)
        # a sample further than this from the current estimate is real
        # motion (someone pushed the rover), not bias - do not learn from it
        self.declare_parameter('gyro_bias_reject', 0.15)   # rad/s
        # seconds of zero /cmd_vel before we believe the rover is stopped
        self.declare_parameter('gyro_bias_settle', 0.5)
        self.declare_parameter('publish_no_fix', True)
        self.declare_parameter('trim_unused_streams', True)
        # Legacy REQUEST_DATA_STREAM fallback: only needed on old firmware
        # that ignores SET_MESSAGE_INTERVAL. This FMU honours the modern
        # command, and the legacy request re-enables the very streams
        # trim_unused_streams just turned off.
        self.declare_parameter('legacy_stream_request', False)

        self.gps_frame_id = self.get_parameter('gps_frame_id').value
        self.imu_frame_id = self.get_parameter('imu_frame_id').value
        self._gps_uere = float(self.get_parameter('gps_uere').value)
        self._gps_sigma_floor = float(
            self.get_parameter('gps_sigma_floor').value)
        self._publish_no_fix = bool(self.get_parameter('publish_no_fix').value)
        self._rtk_floor = float(self.get_parameter('gps_rtk_sigma_floor').value)
        self._rtcm_stale = float(self.get_parameter('rtcm_stale_warn').value)
        self._last_h_acc = 0.0
        # every MAVLink write (stream requests, RTCM fragments) takes this
        self._tx_lock = threading.Lock()
        self._rtcm = None
        self._rtcm_sender = None
        self._rtcm_tx = 0
        self._t_start = time.time()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.gps_pub = self.create_publisher(
            NavSatFix, self.get_parameter('gps_topic').value, qos)
        self.imu_pub = self.create_publisher(
            Imu, self.get_parameter('imu_topic').value, qos)

        self._latest_attitude = None      # (roll, pitch, yaw) NED/FRD as sent
        self._lock = threading.Lock()
        self._bias_on = bool(self.get_parameter('gyro_bias_correction').value)
        self._bias_min_n = int(self.get_parameter('gyro_bias_min_samples').value)
        self._bias_window = int(self.get_parameter('gyro_bias_window').value)
        self._bias_reject = float(self.get_parameter('gyro_bias_reject').value)
        self._bias_settle = float(self.get_parameter('gyro_bias_settle').value)
        self._gyro_bias = [0.0, 0.0, 0.0]
        self._bias_n = 0
        self._bias_logged = False
        self._last_moving_cmd = None
        self._seen_cmd = False
        if self._bias_on:
            self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
            self.create_timer(10.0, self._report_bias)
        self._imu_source = None           # 'HIGHRES_IMU' or 'SCALED_IMU2'
        self._imu_count = 0
        self._gps_count = 0
        self._last_fix_type = -1
        self._last_sats = -1
        self._bad_data = 0
        self._good_data = 0

        port = self.get_parameter('port').value or find_fc_port()
        baud = int(self.get_parameter('baud').value)
        self.get_logger().info(f'Connecting to Pixhawk at {port} (baud={baud})...')
        self.mav = mavutil.mavlink_connection(port, baud=baud)
        self.mav.wait_heartbeat()
        self.get_logger().info(
            f'Heartbeat OK (sys={self.mav.target_system}, '
            f'comp={self.mav.target_component})')

        self._request_streams()
        self._start_rtcm()

        self._stop = False
        self._thread = threading.Thread(target=self._mavlink_loop, daemon=True)
        self._thread.start()

        # Low-rate health log so `ros2 launch` output shows liveness.
        self.create_timer(10.0, self._report_health)

    def _start_rtcm(self):
        if not self.get_parameter('rtcm_enabled').value:
            self.get_logger().info('RTCM injection disabled (rtcm_enabled=false)')
            return
        host = self.get_parameter('rtcm_host').value
        rport = int(self.get_parameter('rtcm_port').value)
        self._rtcm_sender = RtcmSender(self.mav, lock=self._tx_lock)
        self._rtcm = RtcmTcpClient(
            host, rport, self._on_rtcm_frame,
            log=lambda s: self.get_logger().info(s))
        self._rtcm.start()
        self.get_logger().info(
            f'RTK: injecting RTCM from {host}:{rport} as GPS_RTCM_DATA')

    def _on_rtcm_frame(self, frame):
        if self._rtcm_sender.send(frame):
            self._rtcm_tx += 1

    def _request_streams(self):
        imu_rate = int(self.get_parameter('imu_rate_hz').value)
        gps_rate = int(self.get_parameter('gps_rate_hz').value)
        # Resolve message IDs by NAME rather than referencing the constants
        # directly.  pymavlink picks its dialect at import time from the
        # MAVLINK_DIALECT/MAVLINK20 environment, and the older v1.0
        # ardupilotmega dialect genuinely lacks some of these symbols --
        # touching a missing one raises AttributeError and kills the node at
        # startup, which is how this bridge died once already.  The numeric
        # IDs are fixed by the MAVLink standard, so falling back to the
        # literal is safe and keeps the request working on any dialect.
        requests = [
            (_msg_id('HIGHRES_IMU', 105), imu_rate),
            (_msg_id('SCALED_IMU2', 116), imu_rate),
            # ArduPilot: neither of the above is guaranteed; SCALED_IMU is
            # the one it always has.  Only used as a last resort (see loop).
            (_msg_id('SCALED_IMU', 26), imu_rate),
            (_msg_id('ATTITUDE', 30), imu_rate),
            (_msg_id('GPS_RAW_INT', 24), gps_rate),
        ]
        for msg_id, rate in requests:
            self.mav.mav.command_long_send(
                self.mav.target_system, self.mav.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, int(1e6 / max(rate, 1)), 0, 0, 0, 0, 0)

        # Silence streams this node never reads. The FMU otherwise pushes
        # ~10 extra high-rate topics (ODOMETRY, LOCAL_POSITION_NED,
        # ATTITUDE_QUATERNION, targets, servo output...) which is what
        # drives the link into overrun and corrupts the frames we DO want.
        if self.get_parameter('trim_unused_streams').value:
            for msg_id in (
                _msg_id('ATTITUDE_QUATERNION', 31),
                _msg_id('LOCAL_POSITION_NED', 32),
                _msg_id('ODOMETRY', 331),
                _msg_id('POSITION_TARGET_LOCAL_NED', 85),
                _msg_id('ATTITUDE_TARGET', 83),
                _msg_id('SERVO_OUTPUT_RAW', 36),
                _msg_id('VFR_HUD', 74),
                _msg_id('ALTITUDE', 141),
            ):
                self.mav.mav.command_long_send(
                    self.mav.target_system, self.mav.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    msg_id, -1,  # -1 = disable
                    0, 0, 0, 0, 0)
        if self.get_parameter('legacy_stream_request').value:
            for stream in (mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
                           mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,     # attitude
                           mavutil.mavlink.MAV_DATA_STREAM_POSITION):  # gps
                self.mav.mav.request_data_stream_send(
                    self.mav.target_system, self.mav.target_component,
                    stream, min(imu_rate, 10), 1)
        self.get_logger().info('Requested HIGHRES_IMU/SCALED_IMU2/ATTITUDE/GPS_RAW_INT streams')

    # ------------------------------------------------------------------ mavlink
    def _mavlink_loop(self):
        while not self._stop and rclpy.ok():
            try:
                msg = self.mav.recv_match(blocking=True, timeout=1.0)
            except Exception as e:
                self._bad_data += 1
                time.sleep(0.01)
                continue
            if msg is None:
                continue
            t = msg.get_type()
            if t == 'BAD_DATA':
                self._bad_data += 1
                continue
            self._good_data += 1
            if t == 'ATTITUDE':
                with self._lock:
                    self._latest_attitude = (msg.roll, msg.pitch, msg.yaw)
            elif t == 'HIGHRES_IMU':
                if self._imu_source is None:
                    self._imu_source = 'HIGHRES_IMU'
                    self.get_logger().info('IMU source: HIGHRES_IMU (SI units)')
                if self._imu_source == 'HIGHRES_IMU':
                    # already m/s^2 and rad/s, body FRD
                    self._publish_imu(msg.xacc, msg.yacc, msg.zacc,
                                      msg.xgyro, msg.ygyro, msg.zgyro)
            elif t == 'SCALED_IMU2':
                if self._imu_source is None:
                    self._imu_source = 'SCALED_IMU2'
                    self.get_logger().info('IMU source: SCALED_IMU2 (mg / mrad/s)')
                if self._imu_source == 'SCALED_IMU2':
                    self._publish_imu(
                        msg.xacc / 1000.0 * G, msg.yacc / 1000.0 * G,
                        msg.zacc / 1000.0 * G,
                        msg.xgyro / 1000.0, msg.ygyro / 1000.0,
                        msg.zgyro / 1000.0)
            elif t == 'SCALED_IMU':
                # Last-resort source: wait 3 s so HIGHRES_IMU / SCALED_IMU2,
                # which the PX4 FMU streams, win when they exist.
                if (self._imu_source is None
                        and time.time() - self._t_start > 3.0):
                    self._imu_source = 'SCALED_IMU'
                    self.get_logger().info(
                        'IMU source: SCALED_IMU (mg / mrad/s)')
                if self._imu_source == 'SCALED_IMU':
                    self._publish_imu(
                        msg.xacc / 1000.0 * G, msg.yacc / 1000.0 * G,
                        msg.zacc / 1000.0 * G,
                        msg.xgyro / 1000.0, msg.ygyro / 1000.0,
                        msg.zgyro / 1000.0)
            elif t == 'GPS_RAW_INT':
                self._publish_gps(msg)

    # ------------------------------------------------- gyro bias (ZUPT)
    def _cmd_cb(self, msg: Twist):
        self._seen_cmd = True
        if (abs(msg.linear.x) > 0.01 or abs(msg.linear.y) > 0.01
                or abs(msg.angular.z) > 0.01):
            self._last_moving_cmd = self.get_clock().now()

    def _is_stationary(self):
        """True when nothing is commanding the rover to move.

        With no /cmd_vel publisher at all we assume stationary: that is the
        bench case, and it is also the launch window before Nav2 is up,
        which is exactly when we most want a clean bias estimate.
        """
        if not self._seen_cmd:
            return True
        if self._last_moving_cmd is None:
            return True
        age = (self.get_clock().now() - self._last_moving_cmd).nanoseconds * 1e-9
        return age > self._bias_settle

    def _update_gyro_bias(self, gx, gy, gz):
        """Refine the zero-rate estimate from a known-stationary sample.

        Running mean until the window is full, then an EMA with that window
        as its time constant - fast to converge at startup, still able to
        follow thermal drift later.
        """
        sample = (gx, gy, gz)
        if self._bias_n >= self._bias_min_n:
            # Reject real motion: if the rover was pushed or knocked, the
            # sample is not bias and learning from it would corrupt the
            # estimate with whatever it was actually doing.
            if any(abs(sample[i] - self._gyro_bias[i]) > self._bias_reject
                   for i in range(3)):
                return
        self._bias_n = min(self._bias_n + 1, self._bias_window)
        alpha = 1.0 / self._bias_n
        for i in range(3):
            self._gyro_bias[i] += alpha * (sample[i] - self._gyro_bias[i])
        if not self._bias_logged and self._bias_n >= self._bias_min_n:
            self._bias_logged = True
            self.get_logger().info(
                'gyro bias estimate ready: '
                f'x={self._gyro_bias[0]:+.5f} y={self._gyro_bias[1]:+.5f} '
                f'z={self._gyro_bias[2]:+.5f} rad/s '
                f'(z = {math.degrees(self._gyro_bias[2]):+.3f} deg/s, '
                f'{math.degrees(self._gyro_bias[2]) * 60:+.1f} deg/min of '
                'heading drift if left uncorrected)')

    def _report_bias(self):
        if not self._bias_logged:
            self.get_logger().warn(
                'gyro bias not yet estimated - keep the rover still. '
                f'({self._bias_n}/{self._bias_min_n} samples)')
            return
        z = self._gyro_bias[2]
        if abs(math.degrees(z)) > 0.5:
            self.get_logger().warn(
                f'gyro z bias is {math.degrees(z):+.3f} deg/s - large for a '
                'MEMS gyro. It is being compensated, but run a PX4 gyro '
                'calibration: compensation cannot fix scale-factor error, '
                'only offset.',
                throttle_duration_sec=120.0)

    # ------------------------------------------------------------------ imu
    def _publish_imu(self, ax, ay, az, gx, gy, gz):
        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.imu_frame_id

        # body FRD -> FLU: x unchanged, y and z negated
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = -ay
        imu.linear_acceleration.z = -az
        # Subtract the zero-rate offset BEFORE the FRD->FLU sign flip, so
        # the estimate lives in the same frame as the raw samples it was
        # measured from.
        if self._bias_on:
            if self._is_stationary():
                self._update_gyro_bias(gx, gy, gz)
            if self._bias_n >= self._bias_min_n:
                gx -= self._gyro_bias[0]
                gy -= self._gyro_bias[1]
                gz -= self._gyro_bias[2]

        imu.angular_velocity.x = gx
        imu.angular_velocity.y = -gy
        imu.angular_velocity.z = -gz

        with self._lock:
            att = self._latest_attitude

        if att is not None:
            roll_ned, pitch_ned, yaw_ned = att
            # NED/FRD attitude -> ENU/FLU attitude (REP-103)
            roll = roll_ned
            pitch = -pitch_ned
            yaw = math.pi / 2.0 - yaw_ned
            # normalize to [-pi, pi]
            yaw = math.atan2(math.sin(yaw), math.cos(yaw))
            qx, qy, qz, qw = self._euler_to_quat(roll, pitch, yaw)
            imu.orientation.x = qx
            imu.orientation.y = qy
            imu.orientation.z = qz
            imu.orientation.w = qw
            # Pixhawk EKF attitude: roll/pitch good, yaw is compass-based
            imu.orientation_covariance = [
                0.005, 0.0, 0.0,
                0.0, 0.005, 0.0,
                0.0, 0.0, 0.05,
            ]
        else:
            imu.orientation_covariance[0] = -1.0  # orientation unknown

        imu.angular_velocity_covariance = [
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001,
        ]
        imu.linear_acceleration_covariance = [
            0.05, 0.0, 0.0,
            0.0, 0.05, 0.0,
            0.0, 0.0, 0.05,
        ]
        self.imu_pub.publish(imu)
        self._imu_count += 1

    # ------------------------------------------------------------------ gps
    def _publish_gps(self, msg):
        self._last_fix_type = msg.fix_type
        self._last_sats = msg.satellites_visible

        # "No GPS device" sentinel: the FMU reports fix_type 0 with every
        # field at its UINT_MAX/unknown value. Publishing that as a
        # NavSatFix (lat/lon 0,0 = Null Island) would be actively
        # misleading, so drop it and let the health log report it.
        if (msg.fix_type == 0 and msg.satellites_visible == 255
                and msg.lat == 0 and msg.lon == 0):
            return

        has_fix = msg.fix_type >= 3
        if not has_fix and not self._publish_no_fix:
            return

        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = self.gps_frame_id

        if msg.fix_type >= 5:
            status = NavSatStatus.STATUS_GBAS_FIX      # RTK float / fixed
        elif msg.fix_type == 4:
            status = NavSatStatus.STATUS_SBAS_FIX      # DGPS
        elif has_fix:
            status = NavSatStatus.STATUS_FIX
        else:
            status = NavSatStatus.STATUS_NO_FIX
        fix.status.status = status
        fix.status.service = NavSatStatus.SERVICE_GPS

        fix.latitude = msg.lat / 1e7
        fix.longitude = msg.lon / 1e7
        fix.altitude = msg.alt / 1e3  # mm -> m

        # eph/epv are HDOP/VDOP * 100 (65535 = unknown)
        hdop = msg.eph / 100.0 if msg.eph != 65535 else 99.9
        vdop = msg.epv / 100.0 if msg.epv != 65535 else 99.9
        # sigma^2 = (DOP * uere)^2 + floor^2   (see gps_uere above)
        floor = self._gps_sigma_floor
        sh = math.sqrt((hdop * self._gps_uere) ** 2 + floor ** 2)
        # vertical is empirically far worse: 25 m of altitude range was seen
        # over 240 s stationary, so give the floor twice the weight there.
        sv = math.sqrt((vdop * self._gps_uere) ** 2 + (2.0 * floor) ** 2)

        # RTK / DGPS: the receiver's own accuracy estimate (mm) replaces the
        # standalone-GPS model.  h_acc/v_acc are 0 when unknown, in which
        # case the HDOP model above stays in force.
        h_acc = getattr(msg, 'h_acc', 0) / 1000.0
        v_acc = getattr(msg, 'v_acc', 0) / 1000.0
        self._last_h_acc = h_acc
        if msg.fix_type >= 4 and h_acc > 0.0:
            sh = max(h_acc, self._rtk_floor)
            sv = max(v_acc, 2.0 * self._rtk_floor) if v_acc > 0.0 else 2.0 * sh
        fix.position_covariance = [
            sh * sh, 0.0, 0.0,
            0.0, sh * sh, 0.0,
            0.0, 0.0, sv * sv,
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        self.gps_pub.publish(fix)
        self._gps_count += 1

    def _report_health(self):
        total = self._good_data + self._bad_data
        corrupt = 100.0 * self._bad_data / total if total else 0.0
        self.get_logger().info(
            f'imu msgs: {self._imu_count} (src={self._imu_source}), '
            f'gps msgs: {self._gps_count} '
            f'(fix_type={self._last_fix_type}, sats={self._last_sats}), '
            f'link corruption: {corrupt:.1f}%')
        if self._rtcm is not None:
            counts = self._rtcm.drain_counts()
            summary = ' '.join(f'{k}:{v}' for k, v in sorted(counts.items())) \
                or 'none'
            age = self._rtcm.data_age()
            fix_name = FIX_NAMES.get(self._last_fix_type, self._last_fix_type)
            acc = (f'{self._last_h_acc:.3f} m' if self._last_h_acc > 0
                   else 'n/a')
            self.get_logger().info(
                f'RTK: {fix_name}, hAcc {acc}, RTCM last 10 s [{summary}], '
                f'data age {age:.1f}s, bad CRC {self._rtcm.bad_crc}')
            if age > self._rtcm_stale:
                self.get_logger().warn(
                    f'no RTCM from base {self._rtcm.host}:{self._rtcm.port} '
                    f'for {age:.0f}s (connected={self._rtcm.connected}) - '
                    'fix will fall back to standalone GPS. Is STRSVR running '
                    'and reachable?', throttle_duration_sec=30.0)
        if corrupt > 15.0:
            self.get_logger().warn(
                f'MAVLink link is {corrupt:.0f}% corrupt - the serial link is '
                'overrun. Check the baud parameter (921600 works on this '
                'FMU; 115200 does not) and reduce requested stream rates.')
        self._good_data = self._bad_data = 0

    @staticmethod
    def _euler_to_quat(roll, pitch, yaw):
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

    def destroy_node(self):
        self._stop = True
        if self._rtcm is not None:
            self._rtcm.stop()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PixhawkBridge()
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
