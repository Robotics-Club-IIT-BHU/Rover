# Troubleshooting and known limitations

Start with `ros2 run athena_gps_nav stack_check` (see
[TESTING.md](TESTING.md#health-check)); it names the broken link. Tuning
symptoms (overshoot, yawing in place, smeared obstacles) are in
[TUNING.md](TUNING.md#symptom-table). Back to the [package README](../README.md).

| Symptom | Go to |
|---|---|
| `ros2 topic list` is empty from a non-interactive shell | the shell is on domain 0: `export ROS_DOMAIN_ID=42` ([why](../../athena_remote/README.md#option-b-ros-cli-over-ssh)) |
| IMU rate low, MAVLink errors, `link corruption` above a few percent | [Pixhawk link](#check-the-pixhawk-link-before-blaming-gps-or-imu) |
| Pixhawk missing, or disconnects | [Pixhawk missing or silent](#pixhawk-missing-or-silent) |
| `fix_type=0`, no satellites | [GPS has no fix](#gps-has-no-fix) |
| Both costmaps empty, everything else healthy | [Costmap is empty](#costmap-is-empty) |
| `Frames didn't arrived within 5 seconds` | [Camera stalls silently](#camera-stalls-silently) |
| `std::bad_array_new_length` from rtabmap | [rtabmap crashes instantly](#rtabmap-crashes-instantly) |
| Rover turns exactly 90 degrees left | the stock behaviour tree is in use ([TUNING.md](TUNING.md#recovery-behaviour)); or a phantom obstacle, check with `mount_check.py costmap` |
| `/cmd_vel` publishes but wheels do not turn | the Pico's serial port is held by another process, or the Pico is not answering: [athena_drive README](../../athena_drive/README.md#safety) |

## Check the Pixhawk link before blaming GPS or IMU

`pixhawk_bridge` prints this every 10 s:

```
imu msgs: 489 ..., gps msgs: 50 (fix_type=3, sats=11), link corruption: 0.0%
```

Anything above a few percent means the serial link is overrunning and **every**
sensor downstream degrades. Measured on this FMU:

| baud | good frames / 18 s | corrupt | usable IMU |
|---|---|---|---|
| 115200 | 91 | **33.1 %** | 11 |
| **921600** | **1082** | **8.0 %** | **129** |

After also silencing streams the node never reads, corruption drops to **0.0 %**.
`baud` therefore defaults to **921600**. The old `athena_slam` nodes used 115200,
which is why they behaved erratically.

## Pixhawk missing or silent

Signature of a failing USB cable seen on this rover:

```
usb 1-2-port1: Cannot enable. Maybe the USB cable is bad?
usb 1-2.1: can't read configurations, error -32
usb 1-2-port1: unable to enumerate USB device
```

```bash
sudo dmesg | grep 'usb 1-2.1'         # disconnect history
lsusb | grep 26ac                     # on the bus?
ls /dev/serial/by-id/ | grep -i px4   # serial node present?
```

Visible in `lsusb` but **no serial node**, or a node with no heartbeat, means the
physical link is failing. Every disconnect reboots the FMU and cold-restarts the
GPS, which is the usual reason a fix never holds. Swap the cable first, then try
another port. Re-binding the parent hub sometimes revives it temporarily:

```bash
sudo sh -c 'echo 1-2 > /sys/bus/usb/drivers/usb/unbind'
sudo sh -c 'echo 1-2 > /sys/bus/usb/drivers/usb/bind'
```

## GPS has no fix

```bash
# stop the stack first: this needs exclusive access to the serial port
ros2 run athena_gps_nav gps_diagnose --seconds 120
```

`gps_diagnose` is **not a ROS node**: it talks to the FMU with pymavlink and
parses its own `argparse` flags (`--seconds`, `--port`, `--baud`).
`--ros-args -p seconds:=120` is silently ignored and you get the 30 s default.
It exits 0 on a 3D fix, 1 with no GPS telemetry at all, 2 with telemetry but no
usable fix.

| fix_type | Meaning |
|---|---|
| **0** | "no GPS device": read the caveat below |
| **1** | receiver present, no lock: go outside and wait |
| 2 | 2D only, no altitude |
| **3+** | 3D fix: what `navsat_transform` needs |

**Caveat:** on this PX4 build a receiver that is merely *searching* also reports
`fix_type=0`, `sats=0` and "GPS not present", indistinguishable from an unplugged
module. It once sat like that for over two hours and then acquired normally. **Do
not call it a hardware fault from `fix_type=0` alone**: give it 30+ minutes with
sky view.

**Current state on this rover:** the module powers up and talks to the Pixhawk
cleanly (thousands of messages, zero link corruption) but reports zero satellites
indoors and outdoors. The software path is ruled out; the remaining candidates are
the antenna and the RF path. `/gps/fix` still carries a lat/lon with `status: -1`
and a large covariance, so a position appears on the map that is not a fix.

Already ruled out, no need to retest: pymavlink's 1200-baud probe does **not**
reboot the FMU; heavy telemetry load does **not** starve the GPS task; the bridge is
**not** at fault (MAVROS reports identically; use it as a referee):

```bash
ros2 launch mavros px4.launch \
  fcu_url:=/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00:921600
ros2 topic echo /mavros/global_position/raw/fix --once
```

## Costmap is empty

The RealSense pointcloud filter's parameter namespace is named after the
librealsense processing block, and on this Jetson it is **`pointcloud__neon_`**,
not `pointcloud`. Setting `pointcloud.enable: True` fails **silently**: no cloud,
both costmaps empty, everything else healthy. `sensors.launch.py` sets both names.

```bash
ros2 topic info /camera/camera/depth/color/points   # Publisher count must be 1
ros2 param list /camera/camera | grep -i pointcloud # find the real namespace
```

## Camera stalls silently

Streams stop with `Frames didn't arrived within 5 seconds` despite a healthy USB 3
link. USB autosuspend was the cause; a udev rule now pins `power/control=on`. To
recover:

```bash
sudo sh -c 'echo 2-1.3 > /sys/bus/usb/drivers/usb/unbind'
sudo sh -c 'echo 2-1.3 > /sys/bus/usb/drivers/usb/bind'
```

## rtabmap crashes instantly

`std::bad_array_new_length` on startup means `~/rtab` is being sourced. That
source build has been ABI-broken since the Aug 2026 apt upgrade. Use the apt
package (`ros-humble-rtabmap-odom`) and remove `~/rtab` from `.bashrc`.

## Known limitations

- **GPS absolute accuracy is +-3.5 m 1-sigma, measured**, even at HDOP 0.98: a
  receiver floor, not a tuning one; RTK is the only route to sub-metre. It is
  *absolute* error: goal and robot pose live in the same map frame and wander
  together, so Nav2 still converges on `xy_goal_tolerance: 0.5`; the rover just
  ends up within +-3.5 m of where you meant. If it hunts near a goal, widen the
  tolerance rather than tightening it.
- Indoor GPS wanders tens of metres from multipath; every map-frame view inherits
  that. Only odom-frame behaviour is meaningful indoors.
- Visual odometry needs texture and depth in view; a blank wall at 30 cm resets
  tracking repeatedly. People moving in the camera's view register as robot motion
  (`vio_gate` drops the big spikes; keep the scene still for clean numbers).
- The depth camera has a ~0.2 m minimum range and ~87 degree horizontal field of
  view, and the downsampler also drops returns closer than 0.25 m as self-hits.
  Anything closer, directly behind, or outside that cone is invisible. The costmap
  remembers what it saw, but a rover turning in place only clears what the camera
  sees now.
- **There is no rear-facing sensor.** That is why Spin and BackUp were removed from
  the behaviour tree and a failed goal now stops rather than manoeuvring. Reversing
  this rover is an operator decision.
- Absolute heading is not fused; map yaw is dead-reckoned
  ([TUNING.md](TUNING.md#heading-the-compass-is-not-used)). GPS waypoint work needs a
  real heading source first.
- `gps_link` (0, 0, 0.25) in the URDF is an **assumption**, not a measurement.
  `navsat_transform` uses it to remove the antenna lever arm: harmless for local
  obstacle avoidance, worth a tape measure before waypoint work.
- **RViz on the Jetson costs a full CPU core.** Close it for serious measurements.
- The Pico must run the `V <left> <right>` firmware in
  `athena_drive/firmware/athena_drive_fw`. The `athena_drive` bridge speaks only
  that protocol and has no legacy fallback. The deprecated `drive` package has one
  (`legacy_protocol:=true`) but `bringup` deliberately does not use that package: it
  hardcodes a `/dev/ttyACM*` number that enumerates as the Pixhawk about half the
  time ([drive/README.md](../../drive/README.md)).
