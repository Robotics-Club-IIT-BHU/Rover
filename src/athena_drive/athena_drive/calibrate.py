#!/usr/bin/env python3
"""
calibrate.py. Interactive calibration and motor testing. Settings persist.

    ros2 run athena_drive calibrate

Opens a MENU, and you jump straight to whichever test you want:

  m  Manual motor test     pick one or more motors, pick forward/backward,
                           confirm, and only that runs
  d  Direction check       stage 1 for just the motors you pick
  w  Wiring map            which Pico pin goes to which motor driver channel
  s  Board status          live state and invert flags from the Pico
  1  Motor directions      all six, guided   -> saved in the PICO'S FLASH
                           (it belongs to the board, which is where the
                           wiring lives)
  2  Side check            confirms left/right are not swapped
  3  PWM deadband          -> saved to the config file below
  4  Top speed             -> saved to the config file below
  5  Track width           -> saved to the config file below
  a  Run stages 1-5 in order

Anything a stage measures is saved as soon as that stage finishes.

Config file: ~/.config/athena_drive/params.yaml
`motor_bridge` loads it automatically at startup, so calibrated values are
used by every launch without passing arguments.

Skip the menu and run chosen entries straight away (any menu letters work):
    ros2 run athena_drive calibrate --ros-args -p stages:=3,4
    ros2 run athena_drive calibrate --ros-args -p stages:=m
    ros2 run athena_drive calibrate --ros-args -p stages:=all   # 1-5 in order
"""

import os
import re
import sys
import time

import rclpy
from rclpy.node import Node

from athena_drive.motor_bridge import find_pico, CONFIG_PATH

MOTORS = [
    (1, 'front-left'), (2, 'mid-left'), (3, 'rear-left'),
    (4, 'front-right'), (5, 'mid-right'), (6, 'rear-right'),
]
MOTOR_NAME = dict(MOTORS)

# Pico -> driver wiring. Mirrors the #defines in athena_drive_fw.ino, which is
# the only place the pins are set: move a wire to another GPIO and that file
# must change too. (motor: (driver board, its channel, PWM GPIO, DIR GPIO)).
# The board/channel names are the firmware's FRONT_M1 / MID_M2 ... labels.
WIRING = {
    1: ('FRONT', 'M1', 8, 9),
    2: ('MID',   'M1', 18, 19),
    3: ('REAR',  'M1', 10, 11),
    4: ('FRONT', 'M2', 6, 7),
    5: ('MID',   'M2', 20, 21),
    6: ('REAR',  'M2', 12, 13),
}

# Physical header pin for each GPIO we use, on the Pico / Pico W (pin 1 is
# top-left with the USB connector at the top; 1-20 run down the left edge,
# 21-40 run UP the right edge). GND sits at pins 3, 8, 13, 18, 23, 28, 33,
# 38 on the standard Pico header.
GPIO_TO_PIN = {6: 9, 7: 10, 8: 11, 9: 12, 10: 14, 11: 15, 12: 16, 13: 17,
               18: 24, 19: 25, 20: 26, 21: 27}

# Short names accepted when picking motors.
MOTOR_ALIASES = {'fl': 1, 'ml': 2, 'rl': 3, 'fr': 4, 'mr': 5, 'rr': 6}
LEFT_SIDE = (1, 2, 3)
RIGHT_SIDE = (4, 5, 6)

# The Pico resets when the serial port opens; it needs this long to boot and
# start listening before anything sent to it is read.
PICO_BOOT_WAIT = 2.0            # s

# Long enough for the wheel to visibly turn, short enough that a mis-wired
# motor cannot go anywhere before it stops.
PULSE_SECONDS = 1.5

# Deadband sweep: step up in coarse increments until every wheel moves.
DEADBAND_SWEEP = range(10, 141, 10)


def ask(prompt, default=None):
    """Yes/no question. Returns True/False. Ctrl-C safe."""
    suffix = ' [y/n] ' if default is None else (
        ' [Y/n] ' if default else ' [y/N] ')
    while True:
        try:
            a = input(prompt + suffix).strip().lower()
        except EOFError:
            return bool(default)
        if not a and default is not None:
            return default
        if a in ('y', 'yes'):
            return True
        if a in ('n', 'no'):
            return False


def ask_float(prompt, default=None):
    while True:
        try:
            a = input(f'{prompt} ').strip()
        except EOFError:
            return default
        if not a and default is not None:
            return default
        try:
            return float(a)
        except ValueError:
            print('  need a number')


def prompt(text):
    """input() that returns None at EOF instead of raising."""
    try:
        return input(text).strip()
    except EOFError:
        return None


def parse_motors(text):
    """'1,4' / 'left' / 'fl mr' / 'all' -> sorted motor numbers, else None."""
    chosen = set()
    for tok in re.split(r'[,\s]+', text.strip().lower()):
        if not tok:
            continue
        if tok in ('all', 'a'):
            chosen.update(range(1, 7))
        elif tok in ('left', 'l'):
            chosen.update(LEFT_SIDE)
        elif tok in ('right', 'r'):
            chosen.update(RIGHT_SIDE)
        elif tok in MOTOR_ALIASES:
            chosen.add(MOTOR_ALIASES[tok])
        elif tok.isdigit() and 1 <= int(tok) <= 6:
            chosen.add(int(tok))
        else:
            return None
    return sorted(chosen) or None


def wire_text(idx):
    """'PWM GP10 (pin 14), DIR GP11 (pin 15)' for one motor."""
    _, _, pwm, dr = WIRING[idx]
    return (f'PWM GP{pwm} (pin {GPIO_TO_PIN[pwm]}), '
            f'DIR GP{dr} (pin {GPIO_TO_PIN[dr]})')


class Calibrate(Node):
    def __init__(self):
        super().__init__('calibrate')
        self.declare_parameter('port', '')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('test_pwm', 120)
        self.declare_parameter('stages', 'menu')

        import serial
        port = self.get_parameter('port').value or find_pico(self.get_logger())
        if not port:
            raise SystemExit('no Pico found - is it plugged in?')
        self.ser = serial.Serial(port, int(self.get_parameter('baud').value),
                                 timeout=0.5, dsrdtr=None)
        time.sleep(PICO_BOOT_WAIT)
        self.ser.reset_input_buffer()
        self.port = port
        self.results = {}
        self._group = None      # does the firmware have the G command?

    # ---------------------------------------------------------------- serial
    def send(self, line, wait=0.35):
        self.ser.write((line + '\n').encode())
        self.ser.flush()
        time.sleep(wait)
        return self.ser.read(500).decode('utf-8', 'ignore').strip()

    def hold(self, line, seconds):
        """Stream a command so the firmware watchdog stays fed."""
        end = time.time() + seconds
        while time.time() < end:
            self.ser.write((line + '\n').encode())
            self.ser.flush()
            time.sleep(0.05)
        self.send('S', 0.1)

    def pulse_motor(self, idx, pwm):
        self.send(f'M {idx} {pwm}', 0.2)
        time.sleep(PULSE_SECONDS)

    def check_motor_direction(self, idx, name, pwm):
        """Run one motor FORWARD then REVERSE and classify what it did.

        Returns 'ok' | 'flipped' | 'stuck' | 'dead'.

        Testing both directions is the whole point.  Until 2026-09-02 this
        stage only ever ran the motor forward and only asked "did it roll
        FORWARD?", which means a motor that physically CANNOT reverse - a
        broken DIR line, a half-dead H-bridge - passed calibration every
        time and then failed the moment Nav2 asked for a turn.  That is not
        hypothetical: it is the fault this check was written for.

        An invert flag cannot cause 'stuck'.  The firmware applies invert as
        `spwm = -spwm`, a symmetric sign flip, so it can make a motor run the
        WRONG way in both directions but never the SAME way in both.  A
        motor that drives forward on both commands is a wiring or driver
        fault and no amount of calibration will fix it - so say so plainly
        rather than letting the operator flip a flag that cannot help.
        """
        print(f'\n  [{idx}] {name}')
        print('      FORWARD ...', flush=True)
        self.pulse_motor(idx, pwm)
        fwd = ask('      did it roll FORWARD (top of wheel toward the front)?')

        print('      REVERSE ...', flush=True)
        self.pulse_motor(idx, -pwm)
        back = ask('      did it roll BACKWARD this time?')

        if fwd and back:
            return 'ok'
        if not fwd and not back:
            # Ran one way on both commands, or the operator saw it move
            # backward then forward: consistent reversal, just mirrored.
            moved = ask('      did it move at all, just the wrong way each time?',
                        True)
            return 'flipped' if moved else 'dead'
        # Exactly one of the two answers was "yes" -> same direction both
        # times -> the direction input is not being honoured.
        return 'stuck'

    def read_invert(self):
        """Current per-motor invert flags on the Pico, as {idx: 0|1}."""
        out = self.send('I', 0.5)
        state = {}
        for line in out.splitlines():
            line = line.strip()
            if line.startswith('invert:'):
                for pair in line[len('invert:'):].split():
                    k, _, v = pair.partition('=')
                    try:
                        state[int(k)] = int(v)
                    except ValueError:
                        pass
        return state

    # ---------------------------------------------------------------- stages
    def group_supported(self):
        """Does the firmware have `G <mask> <pwm>` (several motors at once)?

        Asked of the board itself via its help text, once, so an un-reflashed
        Pico degrades to running the motors one after another instead of
        answering `ERR use ...` to every multi-motor test.
        """
        if self._group is None:
            self.ser.reset_input_buffer()
            self.ser.write(b'H\n')
            self.ser.flush()
            time.sleep(0.6)
            text = self.ser.read(2000).decode('utf-8', 'ignore')
            self._group = 'G <mask>' in text
        return self._group

    def run_motors(self, motors, pwm):
        """Run the chosen motors at signed pwm for one auto-stopping pulse."""
        if len(motors) == 1:
            reply = self.send(f'M {motors[0]} {pwm}', 0.2)
            time.sleep(PULSE_SECONDS)
        elif self.group_supported():
            mask = sum(1 << (m - 1) for m in motors)
            reply = self.send(f'G {mask} {pwm}', 0.2)
            time.sleep(PULSE_SECONDS)
        else:
            reply = ''
            for m in motors:
                reply += self.send(f'M {m} {pwm}', 0.2)
                time.sleep(PULSE_SECONDS)
        if 'ERR' in reply:
            print(f'  Pico said: {reply}')
        time.sleep(0.2)
        self.ser.reset_input_buffer()    # drop the trailing "motor test ended"

    # ------------------------------------------------------- manual testing
    def _pick_motors(self):
        """Ask which motors. Returns a list, or None for 'back'."""
        print('\n  Motors:  ' + '   '.join(
            f'{i}={n}' for i, n in MOTORS))
        print('  Pick by number (3 or 1,4), name (fl ml rl fr mr rr), or '
              'left / right / all.')
        while True:
            text = prompt('  Which motor(s)?  [q = back] > ')
            if text is None or text.lower() in ('q', 'quit', 'back'):
                return None
            motors = parse_motors(text)
            if motors:
                return motors
            print('  not understood - e.g.  3   1,4   fl mr   left   all')

    def _pick_direction(self, pwm):
        """Returns (sign, pwm), or None for 'back'."""
        while True:
            text = prompt('  Direction: [f]orward or [b]ackward'
                          f'  (pwm {pwm}; add a number to change, e.g. '
                          '"f 150")  [q = back] > ')
            if text is None or text.lower() in ('q', 'quit', 'back'):
                return None
            toks = text.lower().split()
            if toks and toks[0] in ('f', 'fwd', 'forward', 'b', 'back',
                                    'bwd', 'backward', 'r', 'rev', 'reverse'):
                sign = 1 if toks[0].startswith('f') else -1
                if len(toks) == 1:
                    return sign, pwm
                if len(toks) == 2 and toks[1].isdigit() \
                        and 1 <= int(toks[1]) <= 255:
                    return sign, int(toks[1])
            print('  not understood - e.g.  f   b   f 150')

    def _print_plan(self, motors, sign, pwm):
        way = 'FORWARD' if sign > 0 else 'BACKWARD'
        print(f'\n  About to run {way} at pwm {pwm} for '
              f'{PULSE_SECONDS:g} s:')
        for m in motors:
            print(f'    [{m}] {MOTOR_NAME[m]:<12} {wire_text(m)}')
        if len(motors) == 1:
            print('  (If it does not move, those two wires are the ones to '
                  'check.)')
        elif self.group_supported():
            print('  All of them run together.')
        else:
            print('  NOTE: this Pico firmware has no G command yet, so they '
                  'run ONE AFTER\n  ANOTHER, not together. Reflash '
                  'athena_drive_fw to run them together.')

    def manual_test(self):
        print('\n' + '=' * 66)
        print('MANUAL MOTOR TEST')
        print('=' * 66)
        print('Pick the motors, pick a direction, confirm - only that runs.')
        print('Every run auto-stops after ~1.5 s. Wheels OFF the ground.\n')
        if not ask('  Wheels off the ground and ready?', True):
            print('  skipped')
            return
        pwm = int(self.get_parameter('test_pwm').value)
        while True:
            motors = self._pick_motors()
            if motors is None:
                return
            got = self._pick_direction(pwm)
            if got is None:
                continue
            sign, pwm = got
            need_confirm = True
            while True:
                if need_confirm:
                    self._print_plan(motors, sign, pwm)
                    if not ask('  Confirm and run?', True):
                        break
                    need_confirm = False
                self.run_motors(motors, sign * pwm)
                nxt = prompt('  [Enter] run again   [r] reverse direction   '
                             '[n] new selection   [q] back to menu > ')
                nxt = 'q' if nxt is None else nxt.lower()
                if nxt == '':
                    continue
                if nxt in ('r', 'rev', 'reverse'):
                    sign = -sign
                    need_confirm = True
                    continue
                if nxt == 'n':
                    break
                return

    def wiring_map(self):
        print('\n' + '=' * 66)
        print('WIRING MAP - Pico GPIO to motor driver')
        print('=' * 66)
        print('Each motor takes TWO signal wires: PWM (speed) and DIR '
              '(direction).')
        print('"pin" is the physical header pin: pin 1 is top-left with the '
              'USB connector\nat the top, 1-20 run down the left edge, '
              '21-40 run UP the right edge.\n')
        print('  #  motor        driver  ch   PWM wire         DIR wire')
        print('  ' + '-' * 60)
        for idx, name in MOTORS:
            board, ch, pwm, dr = WIRING[idx]
            print(f'  {idx}  {name:<12} {board:<6}  {ch}   '
                  f'GP{pwm:<2} (pin {GPIO_TO_PIN[pwm]:>2})   '
                  f'GP{dr:<2} (pin {GPIO_TO_PIN[dr]:>2})')
        print('\n  The same wires in the order they sit on the Pico header:')
        by_pin = sorted(
            (GPIO_TO_PIN[g], g, idx, kind)
            for idx, (_, _, p, d) in WIRING.items()
            for g, kind in ((p, 'PWM'), (d, 'DIR')))
        for pin, gpio, idx, kind in by_pin:
            print(f'    pin {pin:>2}  GP{gpio:<2}  {kind}  -> motor {idx} '
                  f'({MOTOR_NAME[idx]})')
        print('\n  Left forward = DIR LOW, right forward = DIR HIGH. '
              'PWM is 20 kHz, 3.3 V.')
        print('  Also needed, though not in the firmware: every driver '
              'board\'s signal GND\n  tied to a Pico GND pin (3, 8, 13, 18, '
              '23, 28 ...), and motor power on the\n  driver itself.')
        print('  Board/channel names come from the firmware\'s FRONT_M1 / '
              'MID_M2 ... pin\n  labels; check them against your actual '
              'driver boards.')
        print('\n  Moving a wire to a different GPIO? The pin #defines in '
              'athena_drive_fw.ino\n  must change to match, then reflash.')

    def board_status(self):
        print('\n  Board status:')
        for line in self.send('?', 0.5).splitlines():
            print(f'    {line}')
        for line in self.send('I', 0.5).splitlines():
            print(f'    {line}')

    def stage_pick_directions(self):
        motors = self._pick_motors()
        if motors:
            self.stage_directions(only=motors)

    def stage_directions(self, only=None):
        print('\n' + '=' * 66)
        print('STAGE 1 - motor directions'
              + (f'  (motors {",".join(map(str, only))} only)'
                 if only else ''))
        print('=' * 66)
        print('Each motor runs FORWARD, then REVERSE, ~1.5 s each, one at a time.')
        print('Watch the TOP of that wheel: forward = rolls toward the FRONT.')
        print('Both directions are tested: a motor that cannot reverse is a')
        print('wiring/driver fault, not something an invert flag can fix.\n')
        if not ask('  Wheels off the ground and ready?', True):
            print('  skipped')
            return
        pwm = int(self.get_parameter('test_pwm').value)
        # Read the board's actual current flags rather than assuming 0: a
        # motor already flipped by a prior run (or still wrong after one)
        # must TOGGLE from its real state, not get the same value re-sent.
        inv = self.read_invert()
        flipped, faults = [], []
        for idx, name in MOTORS:
            if only and idx not in only:
                continue
            # Retest after each flip: the operator is the sensor here, and
            # confirming the fix is the only way to know it took.
            while True:
                verdict = self.check_motor_direction(idx, name, pwm)

                if verdict == 'ok':
                    break

                if verdict in ('stuck', 'dead'):
                    self._explain_fault(idx, name, verdict)
                    faults.append((idx, name, verdict))
                    break

                # 'flipped' - this one an invert flag genuinely does fix.
                self._toggle_invert(idx, inv, flipped)
                print('      re-testing...')

        inv_line = self.send('I', 0.5)
        print(f'\n  done. {len(flipped)} motor(s) flipped: '
              f'{flipped if flipped else "none"}')
        print(f'  {inv_line.splitlines()[-1] if inv_line else ""}')
        print('  These live in the Pico. You never need to redo this.')
        if faults:
            self._summarize_faults(faults)

    @staticmethod
    def _explain_fault(idx, name, verdict):
        """Say what is physically wrong, so nobody retries calibration on it."""
        if verdict == 'stuck':
            print(f'      *** FAULT: motor {idx} ({name}) ran the '
                  f'SAME way in both directions.')
            print('          The direction input is not reaching '
                  'the driver. An invert')
            print('          flag is a symmetric sign flip and '
                  'CANNOT fix this - check')
            print(f'          the DIR wiring and the H-bridge for '
                  f'{name}.')
        else:
            print(f'      *** FAULT: motor {idx} ({name}) did not '
                  f'move at all.')
            print('          Check power, the motor leads and the '
                  'driver channel.')
        print(f'          Signal wires: {wire_text(idx)}')

    def _toggle_invert(self, idx, inv, flipped):
        """Flip one motor's invert flag on the Pico and record that we did."""
        desired = 0 if inv.get(idx, 0) else 1
        out = self.send(f'I {idx} {desired}', 1.0)
        if 'config saved' in out:
            inv[idx] = desired
            print(f'      flipped motor {idx} -> invert={desired}, '
                  f'saved to Pico flash')
            if idx not in flipped:
                flipped.append(idx)
        else:
            print(f'      WARN: save may have failed: {out}')

    @staticmethod
    def _summarize_faults(faults):
        """Repeat hardware faults at the end, where they will not scroll past."""
        print('\n  *** ' + '=' * 58)
        print(f'  *** {len(faults)} MOTOR(S) FAILED IN HARDWARE - '
              f'calibration cannot fix these:')
        for idx, name, why in faults:
            reason = ('runs one way only (DIR/driver fault)'
                      if why == 'stuck' else 'does not move (power/leads)')
            print(f'  ***   motor {idx}  {name:<12} {reason}')
        print('  *** Fix the wiring before driving. Nav2 will command')
        print('  *** reverse on every turn and these wheels will fight it.')
        print('  *** ' + '=' * 58)

    def stage_sides(self):
        print('\n' + '=' * 66)
        print('STAGE 2 - are the sides swapped?')
        print('=' * 66)
        if not ask('  Wheels still off the ground?', True):
            print('  skipped')
            return
        pwm = int(self.get_parameter('test_pwm').value)
        print('\n  Spinning LEFT (left side back, right side forward)...')
        self.hold(f'V -{pwm} {pwm}', 2.0)
        ccw = ask('  Did it rotate COUNTER-CLOCKWISE seen from above?')
        if ccw:
            print('  correct: that is +yaw in ROS.')
            self.results['sides_ok'] = True
        else:
            print('\n  The two sides are exchanged. Fix ONE of these:')
            print('    - swap the left and right driver connections, or')
            print('    - flip all six invert flags (I 1 1 ... I 6 1)')
            print('  Do not do both. Re-run stage 1 afterwards.')
            self.results['sides_ok'] = False

    def stage_deadband(self):
        print('\n' + '=' * 66)
        print('STAGE 3 - PWM deadband (min_pwm)')
        print('=' * 66)
        print('Below some duty a DC motor does not turn at all. Too low and')
        print('small course corrections do nothing; too high and the rover')
        print('lurches instead of easing in.\n')
        if not ask('  Wheels off the ground?', True):
            print('  skipped')
            return
        found = None
        for pwm in DEADBAND_SWEEP:
            print(f'\n  pwm {pwm:3d} ...', flush=True)
            self.hold(f'V {pwm} {pwm}', 1.5)
            if ask('      did ALL SIX wheels turn?'):
                found = pwm
                break
        if found is None:
            print('\n  no value moved every wheel - check power and wiring')
            return
        self.results['min_pwm'] = found
        print(f'\n  min_pwm = {found}')

    def stage_topspeed(self):
        print('\n' + '=' * 66)
        print('STAGE 4 - top speed (max_wheel_speed)')
        print('=' * 66)
        print('This scales EVERY velocity Nav2 requests. Wrong by 2x and the')
        print('rover drives at half or double the commanded speed, which')
        print('also breaks the controller\'s distance-to-goal maths.\n')
        print('  *** THE ROVER WILL DRIVE. Put it on the floor, clear space,')
        print('      and mark where the front edge starts. ***\n')
        if not ask('  Ready to drive?', False):
            print('  skipped - keeping the current estimate')
            return
        secs = ask_float('  How many seconds should it run? [3]', 3.0)
        for i in (3, 2, 1):
            print(f'    {i}...', flush=True)
            time.sleep(1.0)
        t0 = time.time()
        self.hold('V 255 255', secs)
        actual = time.time() - t0
        print(f'\n  drove {actual:.2f} s at full PWM.')
        dist = ask_float('  Measured distance in METRES (0 to skip):', 0.0)
        if dist and dist > 0:
            speed = dist / actual
            self.results['max_wheel_speed'] = round(speed, 3)
            print(f'  max_wheel_speed = {speed:.3f} m/s')
            print('  (the ~0.3 s ramp makes short runs read a little low)')
        else:
            print('  skipped')

    def stage_track(self):
        print('\n' + '=' * 66)
        print('STAGE 5 - track width')
        print('=' * 66)
        print('Distance between the LEFT and RIGHT wheel centre lines.')
        print('Sets how fast a commanded turn rate spins the rover.\n')
        w = ask_float('  Track width in METRES (0 to skip):', 0.0)
        if w and w > 0:
            self.results['track_width'] = round(w, 3)
            print(f'  track_width = {w} m')
        else:
            print('  skipped')

    # ---------------------------------------------------------------- save
    def save(self, quiet=False):
        if not self.results:
            if not quiet:
                print('\nNothing new to save.')
            return
        import yaml
        existing = {}
        if os.path.exists(CONFIG_PATH):
            # Merge rather than overwrite, so running one stage does not
            # discard values measured by an earlier run.
            try:
                with open(CONFIG_PATH) as f:
                    existing = yaml.safe_load(f) or {}
            except Exception:
                existing = {}
        merged = dict(existing)
        for k, v in self.results.items():
            # sides_ok is an operator answer, not a motor_bridge parameter.
            if k != 'sides_ok':
                merged[k] = v
        os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
        with open(CONFIG_PATH, 'w') as f:
            f.write('# athena_drive calibration - written by `ros2 run '
                    'athena_drive calibrate`\n')
            f.write('# motor_bridge loads this automatically at startup.\n')
            f.write('# Motor DIRECTIONS are not here: they live in the Pico\'s\n'
                    '# flash, because they belong to the board and its wiring.\n')
            yaml.safe_dump(merged, f, default_flow_style=False)
        print(f'\nSaved to {CONFIG_PATH}:')
        for k, v in sorted(merged.items()):
            print(f'  {k}: {v}')
        print('\nmotor_bridge picks these up on its next start. Nothing to')
        print('pass on the command line.')

    # ---------------------------------------------------------------- menu
    def _entries(self):
        """(key, label, action) in menu order; keys also work in `stages:=`."""
        return [
            ('m', 'Manual motor test   - pick motors + direction, confirm, run',
             self.manual_test),
            ('d', 'Direction check     - stage 1, only the motors you pick',
             self.stage_pick_directions),
            ('w', 'Wiring map          - Pico pin -> motor driver channel',
             self.wiring_map),
            ('s', 'Board status        - Pico state and invert flags',
             self.board_status),
            ('1', 'Stage 1  motor directions, all six, guided',
             self.stage_directions),
            ('2', 'Stage 2  side check (left/right swapped?)',
             self.stage_sides),
            ('3', 'Stage 3  PWM deadband', self.stage_deadband),
            ('4', 'Stage 4  top speed (drives on the floor)',
             self.stage_topspeed),
            ('5', 'Stage 5  track width', self.stage_track),
        ]

    def run_stages(self, keys):
        actions = {k: fn for k, _, fn in self._entries()}
        for k in keys:
            if k in actions:
                actions[k]()
            else:
                print(f'  unknown entry "{k}" - valid: '
                      f'{",".join(actions)}, all')

    def menu(self):
        entries = self._entries()
        while True:
            print('\n' + '-' * 66)
            for key, label, _ in entries:
                print(f'  {key}  {label}')
            print('  a  Run stages 1-5 in order')
            print('  q  Quit')
            choice = prompt('\n  Choose > ')
            if choice is None or choice.lower() in ('q', 'quit', 'exit'):
                return
            choice = choice.lower()
            try:
                if choice == 'a':
                    self.run_stages(['1', '2', '3', '4', '5'])
                elif choice in {k for k, _, _ in entries}:
                    self.run_stages([choice])
                else:
                    print('  pick a letter or number from the list')
                    continue
            except KeyboardInterrupt:
                # Ctrl-C abandons the current test, not the whole session.
                self.send('S', 0.2)
                print('\n  interrupted - motors stopped, back at the menu')
            # Save what a stage measured right away, not only on quit, so a
            # later Ctrl-C or crash cannot throw it away.
            self.save(quiet=True)
            self.results.clear()

    def run(self):
        want = str(self.get_parameter('stages').value).replace(' ', '').lower()
        print('=' * 66)
        print('  ATHENA DRIVE CALIBRATION')
        print(f'  Pico: {self.port}')
        print('=' * 66)
        print('Anything a stage measures is saved as soon as it finishes.')
        try:
            in_menu = want in ('', 'menu')
            if in_menu:
                self.menu()
            else:
                keys = (['1', '2', '3', '4', '5'] if want == 'all'
                        else [s for s in want.split(',') if s])
                self.run_stages(keys)
            self.save(quiet=in_menu)
        finally:
            self.send('S', 0.2)
        print('\ndone. Next:  ros2 launch athena_drive drive.launch.py')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = Calibrate()
    except SystemExit as e:
        print(e, file=sys.stderr)
        rclpy.shutdown()
        return
    try:
        node.run()
    except KeyboardInterrupt:
        node.send('S', 0.2)
        print('\ninterrupted - motors stopped')
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
