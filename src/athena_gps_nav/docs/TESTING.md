# System Testing & RTK Verification Guide

Comprehensive testing workflows, diagnostic checks, and verification procedures for the Athena autonomy stack.

---

## 1. Automated Health Check (`stack_check`)

The `stack_check` utility evaluates active message rates, transform integrity, IMU physics, and Nav2 lifecycle states across a configurable sampling window (default 10 seconds).

```bash
ros2 run athena_gps_nav stack_check
ros2 run athena_gps_nav stack_check --ros-args -p window:=30.0
```

### Expected Output on a Fully Operational System

```
==============================================================
ATHENA STACK HEALTH CHECK
==============================================================
SENSORS
  [PASS]  pixhawk imu                 48.2 Hz
  [PASS]  imu physics (at rest)      |g|=9.81 m/s2, attitude-vs-accel residual 0.003
  [PASS]  pixhawk gps                  5.0 Hz, RTK FIXED (22 sats, hAcc 0.018m)
  [PASS]  depth cloud (camera)         7.0 Hz

OBSTACLE PERCEPTION  (depth camera -> costmap)
  [PASS]  downsampled cloud            7.4 Hz, ~4300 pts/frame
  [PASS]  local costmap marking      780 lethal + 890 inflated cells
  [PASS]  global costmap marking     750 lethal + 910 inflated cells

ODOMETRY / LOCALIZATION
  [PASS]  VIO raw (rtabmap)           10.5 Hz
  [PASS]  VIO gated                   10.2 Hz (0 implausible dropped)
  [PASS]  EKF local (odom)            22.4 Hz
  [PASS]  EKF global (map)            22.1 Hz
  [PASS]  navsat gps->map              5.0 Hz

TF TREE
  [PASS]  odom -> base_link           present, continuous
  [PASS]  map -> odom                 present, continuous

NAV2
  [PASS]  planner / controller        lifecycle active
  [INFO]  cmd_vel                      0.0 Hz (idle - no goal sent)

==============================================================
ALL CHECKS PASS
==============================================================
```

> [!NOTE]
> - `navsat gps->map` requires an active 3D or RTK fix and must wait until `navsat_transform` finishes its `delay: 45.0` s datum settlement window.
> - Under `local_only:=true` (indoor mode), `EKF global (map)` and `navsat gps->map` are reported as inactive, which is expected.

---

## 2. RTK GNSS Verification Workflow

Follow these four steps to verify end-to-end RTK operation from base station injection to centimeter-level localization.

### Step 1: Base Station Connectivity Test

Confirm that the RTCM3 stream on the base station is broadcasting and reachable over the local network:

```bash
# Verify TCP port 2101 accessibility on base station
nc -zv $BASE_STATION_IP 2101
```

*Expected output:* `Connection to <IP> 2101 port [tcp/*] succeeded!`

```bash
# Optional: inspect raw RTCM3 stream byte flow
timeout 3 nc $BASE_STATION_IP 2101 | xxd -p -l 32
```

*Expected output:* Hex stream starting with `d3` (RTCM3 frame sync byte `0xD3`).

### Step 2: Direct GNSS Diagnostics (`gps_diagnose`)

`gps_diagnose.py` establishes a direct MAVLink serial link with the Cube Orange, connects to the RTCM TCP stream, and monitors satellite lock, accuracy, and RTK state transitions in real time.

> [!IMPORTANT]
> The flight controller serial port is strictly exclusive. Stop the ROS stack (`bringup.launch.py` or `sensors.launch.py`) before running `gps_diagnose`.

```bash
# Run 30-second RTK injection diagnostic
ros2 run athena_gps_nav gps_diagnose \
  --rtcm-host $BASE_STATION_IP \
  --rtcm-port 2101 \
  --seconds 30
```

#### Real-Time Telemetry Stream
```
connecting to /dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange* ...
heartbeat OK (sys=1)
  [rtcm] connected to $BASE_STATION_IP:2101
  fix=3 (3D FIX  ) sats=18  hdop= 0.85  hAcc= 0.820m  lat=25.2629548 lon=82.9838284
  fix=4 (DGPS    ) sats=20  hdop= 0.72  hAcc= 0.450m  lat=25.2629548 lon=82.9838284
  fix=5 (RTK FLOA) sats=22  hdop= 0.65  hAcc= 0.180m  lat=25.2629548 lon=82.9838284
  fix=6 (RTK FIXE) sats=24  hdop= 0.58  hAcc= 0.018m  lat=25.2629548 lon=82.9838284
```

#### Final Verdict Interpretation

```
==============================================================
GPS DIAGNOSIS
==============================================================
  autopilot reports GPS sensor present : True
  messages received : 75
  best fix_type     : 6 - RTK FIXED
  max satellites    : 24

  RTCM frames injected: 142, bad CRC: 0

  VERDICT: RTK is working (RTK FIXED).
  navsat_transform will initialize with centimetre-level data.
==============================================================
```

| Exit Code | Meaning | Action |
|---|---|---|
| `0` | **PASS:** 3D Fix or RTK Fix achieved. | System ready for global navigation. |
| `1` | **FAIL:** No MAVLink `GPS_RAW_INT` messages. | Check DroneCAN GPS wiring to CAN1/CAN2 and ArduPilot parameters (`GPS1_TYPE=9`). |
| `2` | **WARN:** Messages present, but no usable fix (`fix_type < 3`). | Check antenna sky view, multipath, and RF cables. |

### Step 3: `/gps/fix` Topic Validation

With `bringup.launch.py` running, inspect the published ROS 2 topic:

```bash
# Inspect GNSS status code (2 = RTK Fix)
ros2 topic echo /gps/fix --field status
```

*Expected output:*
```yaml
status: 2    # NavSatStatus.STATUS_GBAS_FIX (RTK Float / Fixed)
service: 1   # SERVICE_GPS
```

```bash
# Inspect position covariance (diagonal meters^2)
ros2 topic echo /gps/fix --field position_covariance
```

*Expected output in RTK Fixed:*
```
[0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.0016]
```
$(0.02\text{ m})^2 = 0.0004\text{ m}^2$ (centimeter accuracy clamped to `gps_rtk_sigma_floor: 0.02`).

### Step 4: `navsat_transform` & Metric Odometry Check

Verify that metric Cartesian coordinates are publishing into the global EKF:

```bash
ros2 topic echo /odometry/gps --field pose.pose.position
```

*Expected output:* Stable $x, y$ coordinates with variation $< 3\text{ cm}$ over consecutive readings.

---

## 3. Visual Verification with RViz

Run RViz on an operator laptop connected to the rover's network domain:

```bash
# On operator laptop (ensure ROS_DOMAIN_ID=42)
rviz2 -d ~/athena_net/athena_bench.rviz
```

### Essential Displays Checklist

| Display | Topic | Expected Visual Representation |
|---|---|---|
| **RobotModel** | `/robot_description` | Fully connected rover chassis model (+x forward). |
| **Local Costmap** | `/local_costmap/costmap` | 8x8 m obstacle grid centered on rover; dark lethal cells, light inflation halo. |
| **Depth Cloud** | `/camera/camera/depth/color/points_downsampled` | 3D points aligned with physical obstacles in view. |
| **Odometry Local** | `/odometry/local` | Green trajectory arrow; smooth, stays pinned at rest. |
| **Odometry Global** | `/odometry/global` | Orange trajectory arrow; tracks RTK position. |
| **TF Tree** | `/tf`, `/tf_static` | Fully connected `map -> odom -> base_link` tree. |

---

## 4. Camera Mounting Geometry (`mount_check.py`)

Verify camera extrinsics and ground plane separation using RANSAC floor estimation:

```bash
python3 ~/athena/indoor_test/mount_check.py camera
```

| Mode | Command | Output & Purpose |
|---|---|---|
| `camera` | `mount_check.py camera` | Estimates camera height ($z=0.55\text{ m}$), pitch, and roll relative to the physical floor. |
| `costmap` | `mount_check.py costmap` | ASCII grid showing obstacle proximity and footprint clearance. |
| `watch` | `mount_check.py watch` | Continuous real-time monitor; alerts if floor points begin marking as obstacles. |
| `yaw` | `mount_check.py yaw` | Measures 90° manual rotation agreement between gyro, VIO, and EKF. |

---

## 5. Hands-On Operational Tests

1. **Stationary Drift Verification:**
   Leave the rover stationary for 120 seconds:
   ```bash
   ros2 topic echo /odometry/local --field pose.pose.position
   ```
   *Expected result:* **0.0 cm net drift**. Third decimal remains static due to ZUPT pinning.
2. **Obstacle Dynamic Insertion & Clearing:**
   Place an obstacle (box) 1.5 m in front of the camera:
   - Lethal cells immediately appear in the local costmap.
   - Remove the box: STVL frustum clearing wipes the cells within ~1.0 second.
3. **90-Degree Manual Rotation:**
   Rotate the rover 90° counter-clockwise by hand:
   ```bash
   ros2 run tf2_ros tf2_echo odom base_link
   ```
   *Expected result:* Yaw increases by $\approx +1.57\text{ rad}$ (+90°).

---

## 6. Performance Benchmarks & Logging Tools

### Drift & Metric Characterization (`localization_monitor`)

Record continuous state estimation quality for 120 seconds:

```bash
ros2 run athena_gps_nav localization_monitor --ros-args -p duration:=120.0
```

Reports saved to `~/athena/loc_reports/`:
- **Net displacement:** $< 5\text{ cm}$
- **Discontinuities:** `0`
- **Net yaw drift:** $< 0.1^\circ$

### Transform Tree Integrity Check (`tf_check`)

```bash
ros2 run athena_gps_nav tf_check
```

Validates:
- Gravitational acceleration magnitude $\approx 9.81\text{ m/s}^2$ with **positive sign (+z)** in `base_link`.
- Camera optical frame rotations conform to REP-103.
- Depth cloud centroid resides in forward (+x) half-space.
