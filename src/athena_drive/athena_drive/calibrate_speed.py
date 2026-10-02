#!/usr/bin/env python3
"""
calibrate_speed.py. Measure max_wheel_speed and min_pwm for real.

Both parameters are guesses until measured, and both matter:

  max_wheel_speed  scales every velocity Nav2 asks for.  Wrong by 2x and
                   the rover drives at half or double the commanded speed,
                   so the controller's distance-to-goal maths is wrong too.
  min_pwm          the duty below which the motors do not turn at all.
                   Too low and small corrections do nothing; too high and
                   the rover lurches instead of easing in.

    # CLEAR SPACE AHEAD - the rover WILL drive
    ros2 run athena_drive calibrate_speed --ros-args -p mode:=deadband
    ros2 run athena_drive calibrate_speed --ros-args -p mode:=topspeed -p seconds:=3.0

No ROS topics: it talks to the Pico over serial directly, so it works with
the rest of the stack shut down.  It only prints the numbers - unlike
`ros2 run athena_drive calibrate`, it does not save them anywhere.
"""

import time

import rclpy
from rclpy.node import Node

from athena_drive.motor_bridge import find_pico

# The Pico resets when the serial port opens; wait for it to boot.
PICO_BOOT_WAIT = 2.0     # s

# Deadband sweep: step up until the wheels first move.
DEADBAND_SWEEP = range(10, 130, 10)
DEADBAND_HOLD = 1.2      # s driving at each step
DEADBAND_REST = 1.2      # s between steps, to see it stop


class Calibrate(Node):
    def __init__(self):
        super().__init__('calibrate_speed')
        self.declare_parameter('port', '')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('mode', 'deadband')   # deadband | topspeed
        self.declare_parameter('seconds', 3.0)
        self.declare_parameter('pwm', 255)

        import serial
        port = self.get_parameter('port').value or find_pico(self.get_logger())
        if not port:
            raise SystemExit('no Pico found')
        self.ser = serial.Serial(port, int(self.get_parameter('baud').value),
                                 timeout=0.5, dsrdtr=None)
        time.sleep(PICO_BOOT_WAIT)
        self.ser.reset_input_buffer()

    def _hold(self, cmd, seconds):
        """Stream a command so the firmware watchdog stays fed."""
        end = time.time() + seconds
        while time.time() < end:
            self.ser.write((cmd + '\n').encode())
            self.ser.flush()
            time.sleep(0.05)

    def deadband(self):
        print('=' * 62)
        print('DEADBAND - lowest PWM that actually turns the wheels')
        print('Wheels OFF the ground. Watch for the first movement.')
        print('=' * 62)
        for pwm in DEADBAND_SWEEP:
            print(f'  pwm {pwm:3d} ... ', end='', flush=True)
            self._hold(f'V {pwm} {pwm}', DEADBAND_HOLD)
            self.ser.write(b'S\n')
            self.ser.flush()
            print('did the wheels turn?')
            time.sleep(DEADBAND_REST)
        print('\nSet min_pwm to the FIRST value where every wheel turned.')
        print('  ros2 run athena_drive motor_bridge --ros-args -p min_pwm:=<n>')

    def topspeed(self):
        secs = float(self.get_parameter('seconds').value)
        pwm = int(self.get_parameter('pwm').value)
        print('=' * 62)
        print(f'TOP SPEED - full throttle (pwm {pwm}) for {secs:.1f} s')
        print('CLEAR SPACE AHEAD. Mark the start, measure where it stops.')
        print('=' * 62)
        for i in (3, 2, 1):
            print(f'  starting in {i}...', flush=True)
            time.sleep(1.0)
        t0 = time.time()
        self._hold(f'V {pwm} {pwm}', secs)
        self.ser.write(b'S\n')
        self.ser.flush()
        actual = time.time() - t0
        print(f'\n  drove for {actual:.2f} s at pwm {pwm}')
        print('  measure the distance, then:')
        print(f'    max_wheel_speed = distance_m / {actual:.2f}')
        print('  (the ramp costs ~0.3 s of that, so this reads slightly low;')
        print('   use a longer run for a tighter number)')

    def run(self):
        mode = self.get_parameter('mode').value
        try:
            if mode == 'topspeed':
                self.topspeed()
            else:
                self.deadband()
        finally:
            self.ser.write(b'S\n')
            self.ser.flush()


def main(args=None):
    rclpy.init(args=args)
    node = Calibrate()
    try:
        node.run()
    except KeyboardInterrupt:
        node.ser.write(b'S\n')
    finally:
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
