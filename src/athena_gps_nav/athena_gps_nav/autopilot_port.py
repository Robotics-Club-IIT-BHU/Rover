"""
autopilot_port.py

Finds the autopilot's MAVLink serial port.  No ROS imports, so the plain
argparse tools (gps_diagnose) can use it too.

Order matters: the CubeOrange+ (current) first, then the legacy PX4 FMU v2
this rover used before.  Only the `-if00` interface carries MAVLink; the
Cube's `-if02` is its second (SLCAN/secondary) interface.
"""

import glob

CANDIDATE_GLOBS = (
    '/dev/serial/by-id/usb-CubePilot_*-if00',
    '/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00',
)


def find_autopilot_port(candidates=CANDIDATE_GLOBS):
    """Return the first matching serial path, or the first candidate pattern
    if nothing is plugged in (so the connect error names a sensible path)."""
    for pattern in candidates:
        matches = sorted(glob.glob(pattern))
        if matches:
            return matches[0]
    return candidates[0]
