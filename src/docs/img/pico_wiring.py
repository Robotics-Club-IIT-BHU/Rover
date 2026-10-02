#!/usr/bin/env python3
"""Draw the Pico -> motor driver wiring diagram (pico_wiring.png).

Run from anywhere:  python3 pico_wiring.py

The wiring is NOT typed in here. It is read from the two files that own it:

  * the #defines in athena_drive/firmware/athena_drive_fw/athena_drive_fw.ino
  * the WIRING and GPIO_TO_PIN tables in athena_drive/athena_drive/calibrate.py

and the script refuses to draw if the two disagree. If you move a wire, change
both of those files, then re-run this to refresh the picture.
"""
import ast
import os
import re

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                      # noqa: E402
from matplotlib.patches import FancyBboxPatch, Rectangle   # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
DRIVE = os.path.join(HERE, '..', '..', 'athena_drive')
CALIBRATE = os.path.join(DRIVE, 'athena_drive', 'calibrate.py')
FIRMWARE = os.path.join(DRIVE, 'firmware', 'athena_drive_fw',
                        'athena_drive_fw.ino')
OUT = os.path.join(HERE, 'pico_wiring.png')

# Ground pins on the standard Pico header (same list as calibrate.py's comment).
GND_PINS = (3, 8, 13, 18, 23, 28, 33, 38)

NAMES = {1: 'front-left', 2: 'mid-left', 3: 'rear-left',
         4: 'front-right', 5: 'mid-right', 6: 'rear-right'}


def load_tables():
    """WIRING and GPIO_TO_PIN, straight out of calibrate.py."""
    tree = ast.parse(open(CALIBRATE).read())
    found = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
            name = node.targets[0].id
            if name in ('WIRING', 'GPIO_TO_PIN'):
                found[name] = ast.literal_eval(node.value)
    return found['WIRING'], found['GPIO_TO_PIN']


def check_against_firmware(wiring):
    """Fail loudly if the .ino #defines and calibrate.py disagree."""
    text = open(FIRMWARE).read()
    defs = {(b, c, k): int(n) for b, c, k, n in re.findall(
        r'#define\s+(FRONT|MID|REAR)_(M1|M2)_(PWM|DIR)\s+(\d+)', text)}
    for motor, (board, ch, pwm, dr) in wiring.items():
        if defs[(board, ch, 'PWM')] != pwm or defs[(board, ch, 'DIR')] != dr:
            raise SystemExit(
                f'motor {motor}: calibrate.py says PWM GP{pwm} DIR GP{dr} but '
                f'the firmware says PWM GP{defs[(board, ch, "PWM")]} '
                f'DIR GP{defs[(board, ch, "DIR")]}. Fix one, then re-run.')


def main():
    wiring, gpio_to_pin = load_tables()
    check_against_firmware(wiring)

    # pin number -> (motor, board, channel, kind, gpio)
    used = {}
    for motor, (board, ch, pwm, dr) in wiring.items():
        used[gpio_to_pin[pwm]] = (motor, board, ch, 'PWM', pwm)
        used[gpio_to_pin[dr]] = (motor, board, ch, 'DIR', dr)

    # Geometry. y grows downward in the picture, so pin 1 is at the top.
    row_h = 0.62
    top = 0.0

    def y_of(pin):
        row = pin - 1 if pin <= 20 else 40 - pin       # right edge: 40 at top
        return top - row * row_h

    fig, ax = plt.subplots(figsize=(12.5, 9.8), dpi=100)
    ax.set_xlim(-11.0, 11.0)
    ax.set_ylim(top - 20 * row_h - 1.1, 1.9)
    ax.axis('off')

    # Pico body and USB connector.
    body_w = 3.0
    ax.add_patch(FancyBboxPatch((-body_w / 2, y_of(20) - 0.45), body_w,
                                (y_of(1) - y_of(20)) + 0.9,
                                boxstyle='round,pad=0.02,rounding_size=0.15',
                                fc='#2e7d32', ec='#1b4d1f', lw=1.5))
    ax.add_patch(Rectangle((-0.55, y_of(1) + 0.45), 1.1, 0.55,
                           fc='#b0b0b0', ec='#555555', lw=1.2))
    ax.text(0, y_of(1) + 0.73, 'USB', ha='center', va='center', fontsize=10)
    ax.text(0, (y_of(1) + y_of(20)) / 2, 'Raspberry\nPi Pico W\n\n(USB at top,\nviewed from\nthe top)',
            ha='center', va='center', color='white', fontsize=12)

    colour = {'FRONT': '#1565c0', 'MID': '#ef6c00', 'REAR': '#6a1b9a'}

    # Pins.
    for pin in range(1, 41):
        y = y_of(pin)
        left = pin <= 20
        x = -body_w / 2 - 0.18 if left else body_w / 2 + 0.18
        if pin in used:
            motor, board, ch, kind, gpio = used[pin]
            c = colour[board]
            ax.plot([x], [y], 'o', ms=11, mfc=c, mec='black', zorder=5)
            lab = f'{pin}  GP{gpio}' if left else f'GP{gpio}  {pin}'
            ax.text(x + (-0.3 if left else 0.3), y, lab,
                    ha='right' if left else 'left', va='center',
                    fontsize=10.5, fontweight='bold',
                    bbox=dict(fc='white', ec='none', pad=1.5), zorder=6)
        elif pin in GND_PINS:
            ax.plot([x], [y], 'o', ms=9, mfc='#222222', mec='black', zorder=5)
            lab = f'{pin}  GND' if left else f'GND  {pin}'
            ax.text(x + (-0.3 if left else 0.3), y, lab,
                    ha='right' if left else 'left', va='center',
                    fontsize=9, color='#222222')
        else:
            ax.plot([x], [y], 'o', ms=6, mfc='#cfd8dc', mec='#78909c', zorder=5)
            ax.text(x + (-0.3 if left else 0.3), y, str(pin),
                    ha='right' if left else 'left', va='center',
                    fontsize=8, color='#90a4ae')

    # Driver boards: one box per board. Terminal rows keep the same order and
    # spacing as the Pico pins they connect to, so no wire crosses another.
    # FRONT and REAR both hang off the left edge, so they are pushed apart
    # (FRONT up, REAR down) to leave room for their titles.
    shift = {'FRONT': 1.3, 'REAR': -1.3, 'MID': 0.0}
    boards = {}
    for pin, (motor, board, ch, kind, gpio) in used.items():
        boards.setdefault(board, []).append(pin)

    for board, pins in boards.items():
        on_left = pins[0] <= 20
        c = colour[board]
        ys = [y_of(p) + shift[board] for p in pins]
        y_hi, y_lo = max(ys) + 0.95, min(ys) - 0.9
        bx0, bx1 = (-10.7, -5.1) if on_left else (5.1, 10.7)
        ax.add_patch(FancyBboxPatch((bx0, y_lo), bx1 - bx0, y_hi - y_lo,
                                    boxstyle='round,pad=0.02,rounding_size=0.12',
                                    fc='white', ec=c, lw=2.5, zorder=1))
        ax.text((bx0 + bx1) / 2, y_hi - 0.33, f'{board} driver board',
                ha='center', va='center', fontsize=12.5, fontweight='bold',
                color=c)
        for pin in pins:
            motor, _, ch, kind, gpio = used[pin]
            y_pin = y_of(pin)
            y_term = y_pin + shift[board]
            px = -body_w / 2 - 0.18 if on_left else body_w / 2 + 0.18
            tx = bx1 if on_left else bx0
            ax.plot([px, tx], [y_pin, y_term], '-', color=c, lw=2.2, zorder=2)
            ax.plot([tx], [y_term], 's', ms=8, mfc=c, mec='black', zorder=5)
            ax.text(tx + (-0.2 if on_left else 0.2), y_term,
                    f'{ch} {kind}   motor {motor} {NAMES[motor]}',
                    ha='right' if on_left else 'left', va='center',
                    fontsize=9.5, fontweight='bold', zorder=6)
        ax.text((bx0 + bx1) / 2, y_lo + 0.3,
                'signal GND  ->  any Pico GND pin',
                ha='center', va='center', fontsize=8.5, color='#444444',
                style='italic')

    ax.text(0, 1.55,
            'Pico to motor driver wiring  (generated from calibrate.py WIRING)',
            ha='center', va='center', fontsize=14, fontweight='bold')
    ax.text(0, 1.1,
            'M1 channels drive the LEFT side, M2 the RIGHT.   '
            'Left forward = DIR LOW, right forward = DIR HIGH.',
            ha='center', va='center', fontsize=10)
    ax.text(0, top - 20 * row_h - 0.55,
            'Pin 1 is top-left with USB at the top: pins 1-20 run down the left '
            'edge, 21-40 run up the right.\n'
            'Black = GND pins (3, 8, 13, 18, 23, 28, ...). Each driver\'s signal '
            'GND must reach one of them; the firmware does not show it.',
            ha='center', va='center', fontsize=9.5, color='#333333')

    fig.savefig(OUT, dpi=100, bbox_inches='tight', facecolor='white')
    print('wrote', OUT)


if __name__ == '__main__':
    main()
