# athena_gps_nav

Autonomous navigation stack for the Athena rover: integrates Intel RealSense D435i depth sensing, CubePilot Cube Orange flight controller (ArduPilot + DroneCAN RTK GNSS with MAVLink 2 RTCM injection), dual-EKF state estimation (`robot_localization`), visual odometry (`rtabmap`), and Nav2 path planning.

---

## Quick Start

```bash
# 1. Launch complete navigation stack (allow ~40 seconds to settle)
ros2 launch athena_gps_nav bringup.launch.py

# 2. In another terminal, verify node and topic health
ros2 run athena_gps_nav stack_check

# 3. Check RTK fix and satellite telemetry (with optional RTCM injection)
ros2 run athena_gps_nav gps_diagnose --rtcm-host $BASE_STATION_IP --rtcm-port 2101
```

---

## Operating Modes & Launch Arguments

`bringup.launch.py` accepts four arguments:

```bash
# Indoor / No-GPS Mode (Disables ekf_global & navsat_transform; maps map -> odom as identity)
ros2 launch athena_gps_nav bringup.launch.py local_only:=true

# Desk / Dry-Run Mode (Motors receive zero command; safe on a table)
ros2 launch athena_gps_nav bringup.launch.py dry_run:=true

# Camera Disabled (Runs without RealSense depth cloud or VIO)
ros2 launch athena_gps_nav bringup.launch.py use_camera:=false

# Foxglove Disabled (Skips WebSocket bridge to conserve Jetson CPU)
ros2 launch athena_gps_nav bringup.launch.py foxglove:=false
```

> [!IMPORTANT]
> **Indoors or without GPS fix, always set `local_only:=true`.**
> Without satellites, the global EKF dead-reckons identical inputs to the local EKF, causing synthetic filter divergence (`map -> odom` drift). In `local_only` mode, a static transform `map -> odom` is published and Nav2 navigates reliably against visual odometry.

---

## Running Individual Subsystems (Debugging)

For targeted testing or diagnostics, launch individual layers independently:

| Layer | Command | Function |
|---|---|---|
| **Sensors & RTK** | `ros2 launch athena_gps_nav sensors.launch.py` | RealSense camera, camera IMU filter, Cube Orange MAVLink bridge (with RTCM injection), downsampler |
| **Visual Odometry** | `ros2 launch athena_gps_nav vio.launch.py` | rtabmap RGB-D odometry + `vio_gate` |
| **Localization** | `ros2 launch athena_gps_nav localization.launch.py` | Dual EKFs and `navsat_transform` (supports `local_only:=true`) |
| **Navigation** | `ros2 launch athena_gps_nav navigation.launch.py` | Nav2 planner, controller, costmaps, smoother |
| **Motor Drive** | `ros2 launch athena_drive drive.launch.py` | Serial bridge for Raspberry Pi Pico |
| **Control Panel** | `ros2 launch athena_remote foxglove.launch.py` | Foxglove WebSocket bridge and operator nodes |
| **Bench Simulation** | `ros2 launch athena_gps_nav bench_test.launch.py` | Fake sensors feeding the real Nav2 pipeline without hardware |

---

## Sending Navigation Goals

### 1. Clicked Goal (Indoor & Outdoor)
- Open Foxglove Studio or RViz2.
- Use the **Pose Publish Tool** (`/athena/goal_click` in Foxglove or `2D Goal Pose` in RViz).
- Click target point and drag arrow to set target orientation.
- The planner immediately calculates `/plan` and begins moving.

### 2. GPS Coordinate (Requires Satellite Fix)
- Send latitude, longitude, and optional compass heading via string topic:
  ```bash
  ros2 topic pub --once /athena/goal_text std_msgs/String "{data: '25.2629548, 82.9838284'}"
  ```
- Or input via the Foxglove *Go to* tab.

### 3. Record & Replay GPS Waypoint Routes
```bash
# Record waypoints while driving manually (press ENTER at each waypoint):
ros2 run athena_gps_nav gps_waypoint_logger --ros-args -p output_file:=/home/robo/route.yaml

# Replay and drive the recorded route autonomously:
ros2 run athena_gps_nav gps_waypoint_follower --ros-args -p waypoints_file:=/home/robo/route.yaml
```

---

## Node & Tool Reference

Executable targets available via `ros2 run athena_gps_nav <node>`:

| Node | Description |
|---|---|
| `pixhawk_bridge` | Communicates with Cube Orange over MAVLink 2 at 921600 baud -> `/imu/data`, `/gps/fix`. Injects RTCM3 corrections over TCP (`$RTCM_HOST`:2101) into Cube Orange via `GPS_RTCM_DATA`, and publishes centimeter-accurate covariance from `h_acc`. |
| `vio_gate` | Filters visual odometry, rejecting impossible velocity steps and clamping drift when stationary |
| `pointcloud_downsampler` | Voxel-decimates camera point cloud for low-CPU costmap marking |
| `stack_check` | Automated system-wide health diagnostic tool |
| `gps_diagnose` | Direct serial inspection of Cube Orange MAVLink packets with optional RTCM injection (`--rtcm-host`, `--rtcm-port`) to diagnose RTK Float/Fixed state, horizontal accuracy, and satellites. |
| `costmap_drift_check` | Checks whether stationary obstacles migrate across costmap frames |
| `tf_check` | Validates orientation and consistency across the TF tree |
| `gps_waypoint_logger` | Logs visited GPS coordinates to YAML |
| `gps_waypoint_follower` | Sequences autonomous navigation goals across saved GPS coordinates |

---

## Key Parameters & Configuration

| Configuration File | Parameter | Value | Description |
|---|---|---|---|
| `nav2_params.yaml` | `desired_linear_vel` | `0.35` m/s | Nominal cruising speed |
| | `lookahead_dist` | `1.0` m | Fixed Pure Pursuit lookahead distance |
| | `xy_goal_tolerance` | `0.5` m | Distance threshold to declare goal reached |
| | `yaw_goal_tolerance` | `3.14` rad | Heading tolerance (allows ending at any orientation) |
| | `local_costmap` | 8x8 m @ 0.05m | Rolling obstacle avoidance grid |
| | `global_costmap` | 100x100 m @ 0.1m | Rolling long-range route planning grid |
| `dual_ekf_navsat.yaml` | `delay` | `45.0` s | Datum initialization delay |
| | `imu0_config` | Yaw disabled | Yaw is dead-reckoned from gyro rate to prevent compass magnetic interference |

---

## Documentation Links

- [Detailed Navigation Architecture](ARCHITECTURE.md)
- [Sensor & Stack Testing Guide](docs/TESTING.md)
- [Parameter Tuning & Behaviour](docs/TUNING.md)
- [Troubleshooting & Known Issues](docs/TROUBLESHOOTING.md)
- [Overall System Architecture](../docs/ARCHITECTURE.md)
