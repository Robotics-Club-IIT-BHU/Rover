#!/usr/bin/env python3
"""
wiring_check.py. Verify every motor is wired and spinning the right way.

Talks straight to the Pico, bypassing ROS.  Three modes:

    each   (default)  one motor at a time via the firmware's "M" command.
                      Each burst auto-stops after ~1.5 s in the firmware, so
                      a mis-wired motor cannot run away.
    sides             a whole side at a time via "V", to compare left/right.
    spin              the full rover: forward, then spin left.

Nothing here reads your keyboard - it runs the motors on a schedule and you
do the judging.  `ros2 run athena_drive calibrate` is the interactive one
that asks questions and writes the answers to the board.

Nothing else may hold the Pico port while this runs: stop bringup or any
standalone motor_bridge first, and check with `fuser -v /dev/ttyACM*`.

    # PROP THE ROVER UP FIRST - wheels must be off the ground
    ros2 run athena_drive wiring_check                 # interactive
    ros2 run athena_drive wiring_check --ros-args -p motor:=1 -p pwm:=120
    ros2 run athena_drive wiring_check --ros-args -p mode:=sides

Why one motor at a time: if you drive both sides and the rover pulls left,
you cannot tell whether the left pair is weak, the right pair is reversed,
or one of six is dead.  Testing individually turns one ambiguous symptom
into six unambiguous answers.

What "correct" means, in the robot's own frame (REP-103):
    +x forward, +y LEFT, +z up, yaw counter-clockwise seen from above.
So a positive PWM on any motor must roll the TOP of that wheel toward the
FRONT of the rover.  Get this right for all six and `V 150 150` drives
straight forward, which is what the ROS side assumes.
"""

import time

import rclpy
from rclpy.node import Node

from athena_drive.motor_bridge import find_pico

MOTORS = [
    (1, 'front-left'), (2, 'mid-left'), (3, 'rear-left'),
    (4, 'front-right'), (5, 'mid-right'), (6, 'rear-right'),
]

# The Pico resets when the serial port opens; wait for it to boot before
# sending anything.
PICO_BOOT_WAIT = 2.0     # s

# The firmware's own auto-stop for an "M" burst.  Wait it out rather than
# sending a stop, so the check also proves the auto-stop works.
BURST_SECONDS = 1.6


class WiringCheck(Node):
    def __init__(self):
        super().__init__('wiring_check')
        self.declare_parameter('port', '')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('pwm', 120)
        self.declare_parameter('motor', 0)     # 0 = all, 1..6 = just that one
        self.declare_parameter('mode', 'each')  # each | sides | spin
        self.declare_parameter('pause', 3.0)   # s between motors

        import serial
        port = self.get_parameter('port').value or find_pico(self.get_logger())
        if not port:
            raise SystemExit('no Pico found')
        self.ser = serial.Serial(port, int(self.get_parameter('baud').value),
                                 timeout=0.5, dsrdtr=None)
        time.sleep(PICO_BOOT_WAIT)
        self.ser.reset_input_buffer()
        print(f'connected: {port}\n', flush=True)

    def send(self, line, wait=0.4):
        self.ser.write((line + '\n').encode())
        self.ser.flush()
        time.sleep(wait)
        out = self.ser.read(400).decode('utf-8', 'ignore').strip()
        return out

    def run(self):
        pwm = int(self.get_parameter('pwm').value)
        one = int(self.get_parameter('motor').value)
        mode = self.get_parameter('mode').value
        pause = float(self.get_parameter('pause').value)

        print('=' * 66)
        print('  WHEELS OFF THE GROUND?  Each burst runs ~1.5 s then stops.')
        print('=' * 66)
        print()

        if mode == 'sides':
            self._sides(pwm)
        elif mode == 'spin':
            self._spin(pwm)
        else:
            self._each(pwm, one, pause)

        self.send('S')
        print('\nall stopped.')

    def _each(self, pwm, one, pause):
        """Drive each motor in turn and pause so the operator can watch it.

        Nothing is read back: this only runs the motors on a schedule, the
        judging is the operator's.  `ros2 run athena_drive calibrate` is the
        version that asks and acts on the answers.
        """
        targets = [m for m in MOTORS if one == 0 or m[0] == one]
        print(f'Driving {len(targets)} motor(s) FORWARD at pwm={pwm}.')
        print('For each: does the TOP of that wheel roll toward the FRONT?\n')
        for idx, name in targets:
            print(f'  [{idx}] {name:12s} ... ', end='', flush=True)
            self.send(f'M {idx} {pwm}', wait=0.3)
            time.sleep(BURST_SECONDS)
            # Not a prompt: nothing here reads stdin.  It used to print
            # "forward? (y/n)", which looks like it is waiting for an answer
            # and is not.  `calibrate` is the interactive one.
            print('done  (watch it, then note down: forward or backward?)')
            # `pause` is measured from the start of the burst, so subtract
            # what the burst and its command already consumed.
            time.sleep(max(pause - 1.9, 0.3))
        print('\nAny wheel that turned BACKWARD: swap that motor\'s two power')
        print('leads at the driver, or flip its DIR level in the firmware.')

    def _sides(self, pwm):
        """Drive a whole side at once, using V rather than three M commands.

        This used to send 'M 1', 'M 2', 'M 3' 50 ms apart and expect all
        three left motors to run together.  They never did: the firmware's
        M handler calls hardStop() before driving its motor, so each command
        cancelled the previous one and only the LAST motor of the side was
        ever turning.  Two of every three wheels sat still and the operator
        was invited to conclude they were dead - the exact opposite of what
        a wiring check is for.

        V <left> <right> is the right instruction here: applySide() writes
        the same PWM to all three motors on a side in one go, and it honours
        the per-motor invert flags, which is what makes "does this side roll
        forward" a meaningful question.  Like _spin, it has to be repeated
        because the firmware watchdog stops the motors after 500 ms of
        silence.
        """
        print('Left side forward, then right side forward.')
        print('Both should roll the rover FORWARD.')
        print('All THREE wheels on the active side should turn together.\n')
        for side, cmd in (('LEFT ', f'V {pwm} 0'), ('RIGHT', f'V 0 {pwm}')):
            print(f'  {side} side ...', flush=True)
            end = time.time() + BURST_SECONDS
            while time.time() < end:      # keep feeding the firmware watchdog
                self.send(cmd, wait=0.05)
            self.send('S')
            time.sleep(1.0)      # a beat between sides, so they are told apart
        print('\nIf one side rolls backward, all three of its motors are')
        print('reversed - set invert_left/invert_right on motor_bridge, or')
        print('fix the wiring, but do ONE of the two, not both.')
        print('If ONE wheel of a side stays still while the other two turn,')
        print('that is never software: V drives all three identically.')

    def _spin(self, pwm):
        print(f'Full-rover test at pwm={pwm}: forward, then spin left.')
        print('Forward = both sides same way. Spin left = left back, right fwd.')
        print('(uses V so the firmware ramp applies)\n')
        for label, cmd, dur in (('FORWARD   ', f'V {pwm} {pwm}', 2.0),
                                ('SPIN LEFT ', f'V -{pwm} {pwm}', 2.0)):
            print(f'  {label} ...', flush=True)
            end = time.time() + dur
            while time.time() < end:      # keep feeding the firmware watchdog
                self.send(cmd, wait=0.05)
            self.send('S')
            time.sleep(1.0)
        print('\nSpin left must rotate COUNTER-CLOCKWISE seen from above.')
        print('That is +yaw in ROS. If it spins the other way, the two sides')
        print('are swapped: exchange the left and right driver connections.')


def main(args=None):
    rclpy.init(args=args)
    node = WiringCheck()
    try:
        node.run()
    except KeyboardInterrupt:
        node.send('S')
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
