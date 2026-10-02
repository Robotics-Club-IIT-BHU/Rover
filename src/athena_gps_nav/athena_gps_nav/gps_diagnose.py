#!/usr/bin/env python3
"""
gps_diagnose.py

Talks to the Pixhawk directly (no ROS) and reports what the GPS receiver
is actually doing: fix type, satellites visible, HDOP, and whether the
autopilot's own EKF accepts the fix.  Use this when /gps/fix shows
NO FIX and you need to tell "no sky" apart from "broken/absent module".

    ros2 run athena_gps_nav gps_diagnose
    ros2 run athena_gps_nav gps_diagnose --seconds 60

This is a plain argparse script, NOT a ROS node, so it takes ordinary flags:
--port, --baud, --seconds.  The ROS-style `--ros-args -p seconds:=60` that
used to be documented here does nothing at all: parse_known_args silently
discards it and you get the 30 s default while believing you asked for 60.

Nothing else may hold the serial port while this runs. Stop the
pixhawk_bridge / bringup first.

Exit codes:  0 = working (3D fix seen), 1 = no GPS telemetry at all,
2 = telemetry but no usable fix.
"""

import argparse
import sys
import time

from pymavlink import mavutil

PORT = '/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00'

# MAV_SYS_STATUS_SENSOR_GPS, the bit SYS_STATUS uses to say a GPS exists.
SENSOR_GPS_BIT = 0x20

# GPS_RAW_INT sends eph as 65535 when it has no HDOP to report.
EPH_UNKNOWN = 65535

# One line per ~0.4 s keeps the output readable while still showing the
# receiver flapping in and out, which is the fault this tool exists to catch.
PRINT_PERIOD = 0.4

# Two or more crossings between "no GPS device" and "device present" inside
# one window is an electrical fault, not a sky-view problem.
INTERMITTENT_TRANSITIONS = 2

FIX_TYPES = {
    0: 'NO GPS   (receiver not reporting / not connected)',
    1: 'NO FIX   (receiver alive, no satellite lock yet)',
    2: '2D FIX   (position only, no altitude)',
    3: '3D FIX   (usable)',
    4: 'DGPS     (differential, better)',
    5: 'RTK FLOAT',
    6: 'RTK FIXED',
}


def connect(port, baud):
    """Open the MAVLink link and wait for the autopilot's heartbeat."""
    print(f'connecting to {port} ...', flush=True)
    mav = mavutil.mavlink_connection(port, baud=baud)
    mav.wait_heartbeat()
    print(f'heartbeat OK (sys={mav.target_system})\n', flush=True)
    return mav


def request_gps_streams(mav):
    """Ask the autopilot for GPS telemetry.

    Both mechanisms are used because firmware varies in which it honours:
    SET_MESSAGE_INTERVAL is the modern per-message request, and the legacy
    data-stream request covers autopilots that ignore it.
    """
    for msg_id, hz in ((mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 2),
                       (mavutil.mavlink.MAVLINK_MSG_ID_GPS_STATUS, 1),
                       (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1)):
        mav.mav.command_long_send(
            mav.target_system, mav.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)
    mav.mav.request_data_stream_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION, 2, 1)
    mav.mav.request_data_stream_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2, 1)


def collect(mav, seconds):
    """Watch GPS telemetry for `seconds`, printing each sample.

    Returns a dict of the facts the verdict is drawn from.  `transitions`
    counts how often fix_type crossed between 0 ("no GPS device") and
    non-zero, which is what separates a flaky cable from a poor sky view.
    """
    stats = {
        'best_fix': -1,
        'best_sats': -1,
        'samples': 0,
        'seen_gps_msg': False,
        'gps_present_flag': None,
        'transitions': 0,
        'no_gps_samples': 0,
    }
    prev_fix = None
    deadline = time.time() + seconds

    while time.time() < deadline:
        msg = mav.recv_match(
            type=['GPS_RAW_INT', 'GPS_STATUS', 'SYS_STATUS'],
            blocking=True, timeout=2.0)
        if msg is None:
            continue
        t = msg.get_type()

        if t == 'SYS_STATUS':
            stats['gps_present_flag'] = bool(
                msg.onboard_control_sensors_present & SENSOR_GPS_BIT)
            continue

        if t != 'GPS_RAW_INT':
            continue

        stats['seen_gps_msg'] = True
        stats['samples'] += 1
        fix = msg.fix_type
        sats = msg.satellites_visible
        hdop = msg.eph / 100.0 if msg.eph != EPH_UNKNOWN else float('nan')
        stats['best_fix'] = max(stats['best_fix'], fix)
        stats['best_sats'] = max(stats['best_sats'], sats)
        if fix == 0:
            stats['no_gps_samples'] += 1
        if prev_fix is not None and (fix == 0) != (prev_fix == 0):
            stats['transitions'] += 1
        prev_fix = fix

        print(f'  fix={fix} ({FIX_TYPES.get(fix, "?").split()[0]:8s}) '
              f'sats={sats:2d}  hdop={hdop:5.2f}  '
              f'lat={msg.lat/1e7:11.7f} lon={msg.lon/1e7:11.7f}',
              flush=True)
        time.sleep(PRINT_PERIOD)

    return stats


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default=PORT)
    # 921600: at 115200 this FMU corrupts ~33% of MAVLink frames
    ap.add_argument('--baud', type=int, default=921600)
    ap.add_argument('--seconds', type=float, default=30.0)
    # tolerate ROS-style args when run via `ros2 run`
    args, _ = ap.parse_known_args(argv if argv is not None else sys.argv[1:])

    mav = connect(args.port, args.baud)
    request_gps_streams(mav)
    stats = collect(mav, args.seconds)
    return print_verdict(stats)


def print_verdict(stats):
    """Turn the collected counters into a diagnosis and an exit code."""
    best_fix = stats['best_fix']
    best_sats = stats['best_sats']
    samples = stats['samples']
    transitions = stats['transitions']

    print('\n' + '=' * 62)
    print('GPS DIAGNOSIS')
    print('=' * 62)
    if stats['gps_present_flag'] is not None:
        print(f'  autopilot reports GPS sensor present : '
              f'{stats["gps_present_flag"]}')
    if not stats['seen_gps_msg']:
        print('  NO GPS_RAW_INT messages at all.')
        print('  -> The autopilot is not producing GPS telemetry. Check that')
        print('     a GPS module is wired to a GPS/SERIAL port and that the')
        print('     port is configured for GPS in the autopilot firmware.')
        return 1

    print(f'  messages received : {samples}')
    print(f'  best fix_type     : {best_fix} - {FIX_TYPES.get(best_fix, "?")}')
    print(f'  max satellites    : {best_sats}')
    print()
    if transitions:
        print(f'  fix_type 0 <-> non-0 transitions : {transitions}')
        print(f'  samples reporting "no GPS" (0)   : '
              f'{stats["no_gps_samples"]}/{samples}')
    print()

    # fix_type 0 means "no GPS device", which is a DIFFERENT failure from
    # fix_type 1 ("device present, no satellite lock"). Sky view can never
    # produce 0 - only a missing/disconnected/unconfigured receiver can.
    if transitions >= INTERMITTENT_TRANSITIONS:
        print('  VERDICT: the GPS is INTERMITTENT - the autopilot keeps')
        print('  losing the receiver entirely (fix_type drops to 0, "no GPS",')
        print('  then comes back). No amount of sky view causes that; it is')
        print('  an electrical fault. Check, in order:')
        print('    1. the GPS connector at the Pixhawk GPS port (reseat it)')
        print('    2. the cable, especially near strain points/connectors')
        print('    3. power: a browning-out 5V rail drops the module off')
        print('    4. the module itself (swap it to confirm)')
        print('  Navigation will be unreliable until this is fixed: every')
        print('  dropout stalls the global EKF and the map frame freezes.')
    elif best_fix >= 3:
        print('  VERDICT: GPS is working. navsat_transform will initialize')
        print('  and /odometry/gps will publish.')
        return 0
    elif best_fix == 0:
        print('  VERDICT: fix_type 0 ("no GPS device") for the whole window.')
        print()
        print('  On THIS rover that is ambiguous: a PX4 receiver still')
        print('  searching also reports fix_type 0 / sats 0 / sensor-absent,')
        print('  and one was measured sitting like that for over two hours')
        print('  before acquiring normally. So before suspecting hardware:')
        print('    1. give it 30+ minutes with clear sky (cold start)')
        print('    2. re-run this check - sats climbing above 0 means it')
        print('       is alive and acquiring')
        print('  If it is still 0 after that, then suspect the hardware:')
        print('    - GPS cable unplugged or broken (most common)')
        print('    - the autopilot serial port is not configured for GPS')
        print('    - the module has failed')
    elif best_sats == 0:
        print('  VERDICT: the receiver is connected and talking, but sees ZERO')
        print('  satellites. Almost always one of:')
        print('    - indoors / no sky view  (most likely: take it outside)')
        print('    - GPS antenna unplugged or damaged')
        print('    - cold start: a module that has been off for weeks needs')
        print('      5-15 minutes of clear sky for the first fix')
    else:
        print(f'  VERDICT: receiver sees {best_sats} satellites but has no 3D')
        print('  fix. It is acquiring - a 3D fix needs at least 4 satellites.')
        print('  Give it clear sky and a few more minutes.')
    print()
    print('  Until fix_type reaches 3, the GLOBAL EKF has no absolute')
    print('  position: map->odom stays at its initial guess and')
    print('  /odometry/gps does not publish. The LOCAL EKF (odom frame,')
    print('  VIO+IMU) is unaffected and Nav2 obstacle avoidance still works.')
    return 2


if __name__ == '__main__':
    sys.exit(main())
