#!/usr/bin/env python3
"""
motor_bridge.py. /cmd_vel to the Pico motor controller.

Converts geometry_msgs/Twist into signed per-side PWM and streams it to the
Pico running athena_drive_fw:

    V <left> <right>\\n     -255..255 per side, positive = forward

Design notes
------------
Skid-steer kinematics:  v_left = v - w*W/2,  v_right = v + w*W/2

* **Curvature-preserving saturation.**  If either side exceeds the maximum
  wheel speed, BOTH sides scale down by the same factor.  Clipping them
  independently would change the ratio between them, which is what sets the
  turn radius - the rover would silently drive a different arc than Nav2
  asked for, and worse at exactly the moments it is trying hardest to turn.

* **Deadband compensation.**  A DC motor does not move at all below some
  duty cycle.  Any non-zero wheel speed is mapped into [min_pwm, 255] so
  small corrections actually reach the ground instead of being swallowed by
  static friction.

* **Streaming, not event-driven.**  Commands go out at send_rate regardless
  of when /cmd_vel arrives, so the firmware watchdog sees a steady heartbeat
  and only trips on a genuine failure.

* **Two independent watchdogs.**  This node zeroes the target if /cmd_vel
  goes stale; the firmware stops the motors if the serial link goes quiet.
  Either alone leaves a gap - a live node sending stale commands, or a dead
  cable with the motors latched on.

Ports are auto-detected by USB id.  Never hard-code /dev/ttyACM<n>: those
numbers are assigned in enumeration order and swap between boots.  On this
rover the Pixhawk and the Pico have each held ttyACM0 on different days, and
sending motor commands to a flight controller is the failure this prevents.
"""

import glob
import os
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from athena_drive.velocity_profile import VelocityProfile

PICO_GLOB = '/dev/serial/by-id/usb-Raspberry_Pi_Pico*'

# Written by `ros2 run athena_drive calibrate` and read here at startup, so
# calibrated values apply to every launch without command-line arguments.
# Kept outside the package so a rebuild or reinstall cannot wipe it.
#
# Motor DIRECTIONS are deliberately NOT here: they live in the Pico's own
# flash. Direction is a property of the wiring, so it belongs to the board -
# swap in a different Jetson and the motors still spin the right way.
CONFIG_PATH = os.path.expanduser('~/.config/athena_drive/params.yaml')

# Values the calibration wizard writes. Side inversion is NOT among them -
# that is per-motor and lives in the Pico's flash.
_CALIBRATED = ('track_width', 'max_wheel_speed', 'min_pwm', 'max_pwm')


def load_calibration(logger=None):
    """Return saved calibration as a dict, or {} if there is none."""
    if not os.path.exists(CONFIG_PATH):
        if logger:
            logger.warn(
                f'No calibration at {CONFIG_PATH} - using built-in estimates. '
                'Run:  ros2 run athena_drive calibrate')
        return {}
    try:
        import yaml
        with open(CONFIG_PATH) as f:
            data = yaml.safe_load(f) or {}
        out = {k: v for k, v in data.items() if k in _CALIBRATED}
        if logger:
            logger.info(f'loaded calibration from {CONFIG_PATH}: {out}')
        return out
    except Exception as e:
        if logger:
            logger.error(f'could not read {CONFIG_PATH}: {e}')
        return {}


def find_pico(logger=None):
    """Stable by-id path of the Pico, or None with a logged reason.

    Globs rather than pinning one serial: the by-id name embeds the flash
    serial, which changes if the board itself is swapped.
    """
    hits = sorted(glob.glob(PICO_GLOB))
    if not hits:
        if logger:
            logger.error(f'No Pico found matching {PICO_GLOB} - is it plugged in?')
        return None
    if len(hits) > 1 and logger:
        logger.warn(f'{len(hits)} Picos present, using {hits[0]}: {hits}')
    return hits[0]


class MotorBridge(Node):
    def __init__(self):
        super().__init__('motor_bridge')

        self.declare_parameter('port', '')               # '' = auto-detect
        self.declare_parameter('baud', 115200)
        self.declare_parameter('send_rate', 20.0)        # Hz
        # -1 means "not given" -> fall back to the saved calibration, then
        # to the built-in estimate. An explicit value always wins, so a
        # one-off experiment never silently becomes permanent.
        self.declare_parameter('track_width', -1.0)      # m between wheel centres
        self.declare_parameter('max_wheel_speed', -1.0)  # m/s at PWM 255
        self.declare_parameter('min_pwm', -1)            # static-friction deadband
        self.declare_parameter('max_pwm', -1)            # cap for cautious testing
        self.declare_parameter('watchdog_timeout', 0.5)  # s of stale cmd_vel
        # Velocity profile. Limiting acceleration alone still lets the
        # acceleration itself change instantly - that step is the lurch you
        # feel starting and the nod you see stopping, and it shakes the very
        # camera the visual odometry depends on. Bounding jerk too gives an
        # S-curve: the pull eases in, holds, then eases out.
        self.declare_parameter('profile_enabled', True)
        self.declare_parameter('max_accel', 0.5)       # m/s^2
        self.declare_parameter('max_jerk', 1.0)        # m/s^3
        self.declare_parameter('max_ang_accel', 1.5)   # rad/s^2
        self.declare_parameter('max_ang_jerk', 3.0)    # rad/s^3
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)
        self.declare_parameter('dry_run', False)

        # precedence: explicit argument > saved calibration > estimate
        cal = load_calibration(self.get_logger())
        FALLBACK = {'track_width': 0.40, 'max_wheel_speed': 0.7,
                    'min_pwm': 40, 'max_pwm': 255}

        # Which values ended up as guesses.  Tracked per key, not per file:
        # a calibration that exists but is INCOMPLETE used to count as fully
        # calibrated, so the rover would silently run on the 0.7 m/s estimate
        # with no warning anywhere.  That is the live state of this rover -
        # the saved file has min_pwm and track_width but not max_wheel_speed
        # or max_pwm - and max_wheel_speed scales every velocity Nav2 asks
        # for, so it is exactly the value you least want quietly guessed.
        self._estimated = []

        def pick(name, cast):
            given = self.get_parameter(name).value
            if given is not None and cast(given) >= 0:
                return cast(given)                  # explicitly passed
            if name in cal:
                return cast(cal[name])              # from calibration
            self._estimated.append(name)
            return cast(FALLBACK[name])             # built-in estimate

        g = self.get_parameter
        self.track = pick('track_width', float)
        self.vmax = pick('max_wheel_speed', float)
        self.min_pwm = pick('min_pwm', int)
        self.max_pwm = pick('max_pwm', int)
        self.wdt = float(g('watchdog_timeout').value)
        self.inv_l = -1 if g('invert_left').value else 1
        self.inv_r = -1 if g('invert_right').value else 1
        self.dry_run = bool(g('dry_run').value)
        self.calibrated = not self._estimated

        self.profile_on = bool(g('profile_enabled').value)
        self.profile = VelocityProfile(
            float(g('max_accel').value), float(g('max_jerk').value),
            float(g('max_ang_accel').value), float(g('max_ang_jerk').value))
        self._last_tick = None

        self._lock = threading.Lock()
        self._target = (0.0, 0.0)
        self._last_cmd = None
        self._last_line = None
        self.ser = None
        # Firmware liveness.  The Pico is SILENT while it is happy - it only
        # speaks on ERR or a watchdog trip - so "no messages" is not evidence
        # of anything.  We therefore ping it with '?' (a pure status read)
        # and track the replies, which is the only way to tell a working link
        # from a dead one without moving a wheel.
        self._last_reply = None      # monotonic time of the last firmware line
        self._fw_status = None       # last parsed '?' reply
        self._ping_due = 0.0
        self._link_warned = False
        self._reopen_due = 0.0

        if self.dry_run:
            self.get_logger().warn('DRY RUN - commands are logged, not sent.')
        else:
            self._open_serial()

        self.create_subscription(Twist, 'cmd_vel', self._cmd_cb, 10)
        self.status_pub = self.create_publisher(String, '~/firmware', 10)

        rate = float(g('send_rate').value)
        self.create_timer(1.0 / max(rate, 1.0), self._tick)
        self._start_reader()

        self.get_logger().info(
            f'motor_bridge up: track={self.track} m, '
            f'max_wheel_speed={self.vmax} m/s, pwm=[{self.min_pwm},{self.max_pwm}]')
        if self._estimated:
            # Name the specific keys. "run calibrate" alone is not actionable
            # when the file already exists and looks fine.
            self.get_logger().warn(
                'UNCALIBRATED, using built-in estimates for: '
                + ', '.join(self._estimated)
                + f'.  Nothing in {CONFIG_PATH} supplies them.'
                + ('  max_wheel_speed scales every velocity Nav2 commands, so'
                   ' distances and speeds will be wrong until it is measured:'
                   '  ros2 run athena_drive calibrate_speed'
                   ' --ros-args -p mode:=topspeed'
                   if 'max_wheel_speed' in self._estimated else ''))

    # ------------------------------------------------------------- serial
    def _open_serial(self):
        import serial
        port = self.get_parameter('port').value or find_pico(self.get_logger())
        if not port:
            return
        try:
            # dsrdtr=None: some boards reset when DTR is asserted on open
            self.ser = serial.Serial(port, int(self.get_parameter('baud').value),
                                     timeout=0.2, dsrdtr=None)
            time.sleep(2.0)          # let the board finish booting
            self.get_logger().info(f'serial open: {port}')
        except Exception as e:
            self.get_logger().error(f'could not open {port}: {e} - running dry')
            self.ser = None
            return
        self._handshake()

    def _handshake(self):
        """Prove the firmware is actually alive before we trust the link.

        Opening a CDC port succeeds against a wedged board, a board running
        different firmware, or one that enumerated but never finished booting.
        Without this check motor_bridge logged a cheerful 'serial open' and
        then streamed 'V 0 0' into the void at 20 Hz forever, with nothing
        anywhere saying the rover could not possibly move.  That is what
        "drive does not work at all and there is no error" looks like.

        '?' is a pure status read - it sets no target and moves nothing.
        """
        try:
            self.ser.reset_input_buffer()
            self.ser.write(b'?\n')
            self.ser.flush()
        except Exception as e:
            self.get_logger().error(f'firmware handshake write failed: {e}')
            return
        deadline = time.time() + 2.0
        reply = ''
        while time.time() < deadline:
            try:
                chunk = self.ser.read(200).decode('utf-8', 'ignore')
            except Exception:
                break
            if chunk:
                reply += chunk
                if 'wdt=' in reply:
                    break
            else:
                time.sleep(0.05)
        reply = reply.strip()
        if not reply:
            self.get_logger().error(
                'FIRMWARE NOT RESPONDING. The serial port opened but the Pico '
                'did not answer "?" within 2 s. Nothing this node sends will '
                'reach the motors. Check that athena_drive_fw is flashed and '
                'the board is not wedged (replug the USB), then relaunch.')
            self._fw_status = None
            return
        self._last_reply = time.time()
        self._fw_status = reply
        self.get_logger().info(f'firmware alive: {reply}')
        # Surface the saved invert flags: a wrong one here is invisible at
        # runtime and shows up only as a wheel fighting the others.
        if 'inv=' in reply:
            inv = reply.split('inv=')[1].split()[0]
            if set(inv) - {'0'}:
                self.get_logger().warn(
                    f'per-motor invert flags are not all zero: inv={inv} '
                    '(motor order 1=FL 2=ML 3=RL 4=FR 5=MR 6=RR). '
                    'That is fine if it matches your wiring - and the usual '
                    'cause of one wheel driving against the others if it does not.')

    def _start_reader(self):
        """Run one reader per serial handle.

        The reader is bound to the handle it was started with and exits as
        soon as that handle is replaced, so a reconnect cannot end up with
        two threads racing on the same port - or, as before this was added,
        with ZERO readers after the first drop, which silently disabled
        every firmware error message for the rest of the session.
        """
        ser = self.ser
        if ser is None:
            return
        threading.Thread(target=self._reader, args=(ser,), daemon=True).start()

    def _reader(self, ser):
        """Surface firmware messages so watchdog trips are visible in ROS."""
        while rclpy.ok() and self.ser is ser:
            try:
                line = ser.readline().decode('utf-8', 'ignore').strip()
            except Exception:
                return
            if not line:
                continue
            self._last_reply = time.time()
            if 'wdt=' in line:
                self._fw_status = line
            self.status_pub.publish(String(data=line))
            if line.startswith('ERR') or 'WDT stop' in line:
                self.get_logger().warn(f'pico: {line}')

    def _send(self, line):
        if line != self._last_line:
            self.get_logger().info(f'-> {line}', throttle_duration_sec=1.0)
            self._last_line = line
        if self.dry_run or self.ser is None:
            return
        try:
            self.ser.write((line + '\n').encode())
        except Exception as e:
            # The USB link on this rover is intermittently flaky. Drop the
            # handle rather than raising on every tick from here on; _tick
            # retries the open, so a replug recovers on its own instead of
            # needing the whole stack relaunched.
            self.get_logger().error(f'serial write failed: {e} - dropping link')
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self._fw_status = None
            self._reopen_due = time.time() + 2.0

    # ------------------------------------------------------------- control
    def _cmd_cb(self, msg):
        with self._lock:
            self._target = (msg.linear.x, msg.angular.z)
            self._last_cmd = self.get_clock().now()

    def twist_to_pwm(self, v, w):
        """(v, w) -> (left_pwm, right_pwm), preserving commanded curvature."""
        vl = v - w * self.track / 2.0
        vr = v + w * self.track / 2.0

        peak = max(abs(vl), abs(vr))
        if peak > self.vmax:                 # scale together, not independently
            vl *= self.vmax / peak
            vr *= self.vmax / peak

        def to_pwm(ws):
            if abs(ws) < 1e-3:
                return 0
            span = self.max_pwm - self.min_pwm
            mag = self.min_pwm + (abs(ws) / self.vmax) * span
            return int(round(mag)) * (1 if ws >= 0 else -1)

        return to_pwm(vl) * self.inv_l, to_pwm(vr) * self.inv_r

    def _tick(self):
        with self._lock:
            v, w = self._target
            last = self._last_cmd

        now = self.get_clock().now()
        if last is None:
            v = w = 0.0
        elif (now - last).nanoseconds * 1e-9 > self.wdt:
            v = w = 0.0

        if self.profile_on:
            # Real elapsed time, not the nominal period: a late tick that
            # integrated a full period would overshoot the accel limit.
            dt = 0.0 if self._last_tick is None else \
                (now - self._last_tick).nanoseconds * 1e-9
            self._last_tick = now
            if 0.0 < dt < 0.5:          # ignore startup and long stalls
                v, w = self.profile.step(v, w, dt)
            elif dt >= 0.5:
                self.profile.reset()    # resync after a stall

        left, right = self.twist_to_pwm(v, w)
        self._send(f'V {left} {right}')
        self._check_link(left, right)

    def _check_link(self, left, right):
        """Keep the firmware link honest: reopen it, and prove it still talks.

        Two failure modes this catches, both of which used to be silent:
          * the link dropped (USB glitch) - we reopen instead of dying
          * the link is open but the board stopped answering - we say so
        """
        if self.dry_run:
            return
        now = time.time()

        # 1. link is down -> retry the open, but not on every 50 ms tick
        if self.ser is None:
            if now >= self._reopen_due:
                self._reopen_due = now + 2.0
                self._open_serial()
                if self.ser is not None:
                    self._start_reader()
                    self.get_logger().info('serial link recovered')
                    self._link_warned = False
            return

        # 2. link is up -> ping with a pure status read every 2 s. The reader
        #    thread timestamps the reply. '?' sets no target and moves nothing,
        #    and it also refreshes _fw_status so a watchdog trip is visible.
        if now >= self._ping_due:
            self._ping_due = now + 2.0
            try:
                self.ser.write(b'?\n')
            except Exception:
                pass    # _send's handler owns link teardown

        # 3. no reply for 6 s means the board is not listening. Say it once,
        #    loudly, and keep saying it every 10 s - a rover that cannot move
        #    and does not complain is the worst of both worlds.
        if self._last_reply is not None and (now - self._last_reply) > 6.0:
            if not self._link_warned or (now - self._last_reply) % 10.0 < 0.06:
                self._link_warned = True
                self.get_logger().error(
                    f'firmware has not answered for '
                    f'{now - self._last_reply:.0f}s - commands are going '
                    f'nowhere (last sent: V {left} {right}). Replug the Pico.')
        elif self._link_warned and self._last_reply is not None \
                and (now - self._last_reply) <= 6.0:
            self._link_warned = False
            self.get_logger().info('firmware is answering again')

    def destroy_node(self):
        if self.ser is not None:
            try:
                self.ser.write(b'S\n')
                self.ser.flush()
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorBridge()
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
