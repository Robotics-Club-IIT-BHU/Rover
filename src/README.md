# Athena Autonomous Rover

Athena is a six-wheeled outdoor autonomous rover running **ROS 2 Humble** on an **NVIDIA Jetson**. It navigates to GPS or clicked map coordinates, builds live local/global obstacle costmaps, and safely steers around detected obstacles using skid-steer drive.

---

## System Overview

```
                      +----------------------------------------+
                      |               OPERATOR                 |
                      |   Foxglove Studio / Teleop / Laptop    |
                      +-------------------+--------------------+
                                          | SSH Tunnel (:8765)
+-----------------------------------------v-----------------------------------------+
|                                   NVIDIA JETSON                                   |
|                                                                                   |
|  [Sensors & RTK GNSS]         [Perception & Odometry]            [Navigation]        |
|  - RealSense D435i  --------> Pointcloud Downsampler ------->  Nav2 Costmaps         |
|  - Cube Orange (IMU/RTK) ---> rtabmap VIO + vio_gate ------->  NavFn Planner         |
|    (RTCM3 TCP Injected) ----> Dual EKFs (Local & Global) --->  Pure Pursuit Control  |
|                                                                    |              |
|                                                                /cmd_vel           |
|                                                                    |              |
|  [Drive Bridge]                                                    v              |
|  - athena_drive::motor_bridge (Velocity profiling & PWM deadband)                  |
+-----------------------------------------+-----------------------------------------+
                                          | USB Serial (115200)
+-----------------------------------------v-----------------------------------------+
|                              RASPBERRY PI PICO W                                  |
|         Firmware: PWM generation (20 kHz), slew rate limiting, watchdog          |
+-----------------------------------------+-----------------------------------------+
                                          |
+-----------------------------------------v-----------------------------------------+
|                              6x DC DRIVE MOTORS                                   |
+-----------------------------------------------------------------------------------+
```

---

## Documentation Index

| Resource | Scope |
|---|---|
| **[System Architecture](docs/ARCHITECTURE.md)** | Package breakdown, TF transform tree, data flow rates, and design decisions |
| **[Navigation Stack (`athena_gps_nav`)](athena_gps_nav/README.md)** | Full bringup, indoor vs outdoor modes, parameter tuning, sensor diagnostics |
| **[Motor Drive (`athena_drive`)](athena_drive/README.md)** | Pico pinout, motor driver wiring, interactive calibration wizard, firmware |
| **[Remote Access (`athena_remote`)](athena_remote/README.md)** | SSH configuration, CycloneDDS multi-machine setup, headless portal login |
| **[Foxglove Studio UI Guide](athena_remote/docs/FOXGLOVE_PANEL.md)** | Layout setup, 3D visualization, waypoint storage, and teleoperation |
| **[Contributing Guide](CONTRIBUTING.md)** | Build instructions (`colcon`), workspace layout, git workflows |

---

## 🚀 Quick Start Flow

Run through these steps to get from initial power-on to driving:

```
[1. Connect SSH] ---> [2. WiFi Login & Time Sync] ---> [3. Bringup Stack] ---> [4. Verify Health] ---> [5. Open Foxglove]
```

### 1. Connect to the Jetson via SSH
Set the Jetson IP on your workstation (or use `ubuntu.local` via mDNS):

```bash
# On your workstation:
export JETSON_IP=<jetson-ip>    # e.g., export JETSON_IP=192.168.1.50 or export JETSON_IP=ubuntu.local
ssh robo@$JETSON_IP
```

> [!TIP]
> Add an alias to your workstation's `~/.ssh/config` for seamless single-command login and automatic port forwarding:
> ```ssh-config
> Host athena
>     HostName <jetson-ip>
>     User robo
>     ForwardAgent yes
>     LocalForward 8765 localhost:8765
> ```
> Then connect anytime with: `ssh athena`

---

### 2. Campus WiFi Login & Time Synchronization
If connected to a captive network requiring browser authentication, log in headlessly and synchronize system time:

```bash
# 1. Discover login form parameters (ONE TIME)
ros2 run athena_remote captive_login --ros-args -p mode:=discover   # writes ~/.config/athena_remote/portal.yaml

# 2. Add your username and password (i to type, Esc then :wq to save)
vim ~/.config/athena_remote/portal.yaml

# 3. Log into the network (skips if already online)
ros2 run athena_remote captive_login

# 4. Synchronize clock immediately (CRITICAL for ROS TF, SSL certs, and apt)
sudo systemctl restart systemd-timesyncd
```

Verify connection and time sync:
```bash
timedatectl                                                         # check "System clock synchronized: yes"
```

---

### 3. Launch the Autonomy Stack

```bash
# Full outdoor autonomous stack (sensors + VIO + Nav2 + Drive + Foxglove bridge)
ros2 launch athena_gps_nav bringup.launch.py
```

Allow **~40 seconds** for all staggered layers to initialize:
- **t = 0s:** Sensors (RealSense, Pixhawk, Robot description) + Motor bridge
- **t = 5s:** Visual Odometry (rtabmap + `vio_gate`)
- **t = 8s:** State Estimation (Local EKF, Global EKF, `navsat_transform`)
- **t = 12s:** Nav2 (Costmaps, NavFn planner, Regulated Pure Pursuit)
- **t = 15s:** Foxglove Bridge & Operator telemetry nodes

#### Key Launch Options
```bash
# Indoors or without valid GPS fix (prevents global filter drift):
ros2 launch athena_gps_nav bringup.launch.py local_only:=true

# Bench testing (motors receive no commands; safe with wheels on table):
ros2 launch athena_gps_nav bringup.launch.py dry_run:=true

# Without RealSense camera:
ros2 launch athena_gps_nav bringup.launch.py use_camera:=false

# Without Foxglove bridge (saves Jetson CPU):
ros2 launch athena_gps_nav bringup.launch.py foxglove:=false
```

---

### 4. Verify System Health
Run the automated diagnostic check in a separate terminal:

```bash
ros2 run athena_gps_nav stack_check
```

Example healthy output:
```
SENSORS
  [PASS]  pixhawk imu                 48.5 Hz
  [PASS]  imu physics (at rest)      |g|=9.81 m/s2, attitude-vs-accel residual 0.002
  [PASS]  pixhawk gps                  5.2 Hz
  [PASS]  depth cloud (camera)         6.6 Hz

OBSTACLE PERCEPTION  (depth camera -> costmap)
  [PASS]  downsampled cloud            6.6 Hz, 4812 pts/frame
  [PASS]  local costmap marking      767 lethal + 888 inflated cells
  [PASS]  global costmap marking     731 lethal + 902 inflated cells

ODOMETRY / LOCALIZATION
  [PASS]  VIO raw (rtabmap)           11.4 Hz
  [PASS]  VIO gated                   11.2 Hz
  [PASS]  EKF local (odom)            29.8 Hz
  [PASS]  EKF global (map)            27.3 Hz
```

> [!NOTE]
> When running with `local_only:=true` or indoors, `EKF global (map)` and `navsat gps->map` report `FAIL`. This is expected because the global GPS filter is disabled by design in local mode.

---

### 5. Connect Foxglove Studio UI

1. **Open SSH Tunnel (on workstation):**
   ```bash
   ssh -N -L 8765:localhost:8765 robo@$JETSON_IP
   ```
2. **Download layout file (on workstation, first time):**
   ```bash
   scp robo@$JETSON_IP:/home/robo/athena/src/athena_remote/config/athena_foxglove_layout.json .
   ```
3. **Open browser:** Navigate to **[app.foxglove.dev](https://app.foxglove.dev)**.
4. **Connect:** Select **Foxglove WebSocket** -> `ws://localhost:8765`.
5. **Import layout:** Top right **Layout menu** -> **Import from file...** -> select `athena_foxglove_layout.json`.

---

## 🎮 Driving & Sending Goals

| Operation | Method | Notes |
|---|---|---|
| **Click Goal** | 3D view toolbar -> **Publish tool (Pose)** | Click & drag heading. Published on `/athena/goal_click`. Works indoors and outdoors. |
| **GPS Coordinate** | Controls tab -> *Go to* field | Send JSON `{"data": "25.2629548, 82.9838284"}`. Requires valid GPS fix. |
| **Teleoperation** | Teleop tab arrow buttons | Manual nudge instantly overrides and cancels autonomous goals. |
| **Cancel Goal** | Controls tab -> *Go to* field | Send `{"data": "cancel"}`. |
| **Save Waypoint** | Controls tab -> *Mark* field | Send `{"data": "site_A"}`. Saved in `~/.config/athena_remote/waypoints.yaml`. |
| **Return to Waypoint** | Controls tab -> *Return* field | Send `{"data": "site_A"}`. |

---

## 💻 Direct ROS 2 Operations from Base Station (Without Foxglove)

If you are operating from a standard Ubuntu base station laptop or SSH terminal without running Foxglove Studio, you can control the rover, dispatch goals, and monitor live positioning using standard ROS 2 CLI tools.

### 1. Base Station Environment Setup
Ensure your workstation and the Jetson are on the same network subnet, and configure ROS 2 DDS discovery:

```bash
# On your Ubuntu base station laptop terminal:
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# Verify ROS 2 sees the rover over the network:
ros2 node list
ros2 topic list
```
*(Alternatively, SSH into the Jetson directly: `ssh robo@$JETSON_IP` where `ROS_DOMAIN_ID=42` is preset).*

---

### 2. Manual Driving & Teleoperation

#### A. Interactive Keyboard Teleoperation
Run the dedicated teleop node with safety deadman stop (releasing keys halts after 0.4s):
```bash
ros2 run athena_drive teleop
```
| Key | Action |
|---|---|
| **Arrow Up / Down** | Drive forward / reverse |
| **Arrow Left / Right** | Rotate counter-clockwise / clockwise |
| **q / z** | Linear speed $\pm 0.05$ m/s (default: 0.25 m/s) |
| **e / c** | Angular speed $\pm 0.10$ rad/s (default: 0.60–0.80 rad/s) |
| **Space** | Instant emergency brake (zero velocity) |
| **Ctrl-C** | Quit teleop |

#### B. Single-Command `/cmd_vel` Pulses (Testing)
```bash
# Rotate in place counter-clockwise at 0.8 rad/s (matches teleop turn torque):
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.8}}"

# Drive forward at 0.25 m/s:
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.25}, angular: {z: 0.0}}"

# Emergency stop:
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

---

### 3. Sending Autonomous Navigation Goals

#### A. Send 2D Metric Goal Pose (`map` frame)
Send an autonomous Cartesian target ($x, y$ in meters relative to map origin):
```bash
# Send rover to x = +2.0m, y = +1.0m in map frame:
ros2 topic pub --once /athena/goal_click geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}
}"
```
*(Or send directly via Nav2 action client)*:
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{
  pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}
}"
```

#### B. Send GPS Coordinate Goal (Latitude, Longitude)
Dispatches an outdoor geographic coordinate (converted into `map` frame by `goal_manager`):
```bash
# Format: "latitude, longitude"
ros2 topic pub --once /athena/goal_text std_msgs/msg/String "{data: '25.262955, 82.983828'}"
```

#### C. Cancel Active Autonomous Goal
```bash
ros2 topic pub --once /athena/goal_text std_msgs/msg/String "{data: 'cancel'}"
```

---

### 4. Monitoring Rover Position, Sensors & Decisions

```bash
# 1. Rover Local Metric Position (X, Y, Z in odom frame):
ros2 topic echo /odometry/local --field pose.pose.position

# 2. Rover Geographic Position & RTK Status (status 0=No fix, 1=Autonomous, 2=RTK Float/Fixed):
ros2 topic echo /gps/fix --field latitude --field longitude --field status.status

# 3. Transparent Rover Decision & Activity Status Card (Activity, Decision, Why, Distance, ETA):
ros2 topic echo /athena/decision_status --field data

# 4. Summary Navigation Status:
ros2 topic echo /athena/nav_status --field data

# 5. Live Commanded Motor Output:
ros2 topic echo /cmd_vel
```

---

### 5. Native RViz2 Visualization (Workstation)
If your Ubuntu laptop has ROS 2 Humble installed, you can visualize the pointcloud, local/global costmaps, footprint, and planned paths natively in RViz2:

```bash
rviz2 -d ~/athena/src/athena_gps_nav/config/athena_bench.rviz
```
To set a goal visually: select the **2D Goal Pose** tool on the top toolbar, then click and drag a target heading on the map grid.

---

## 🛠️ Everyday Commands Cheat Sheet

```bash
# --- Autonomy Stack ---
ros2 launch athena_gps_nav bringup.launch.py local_only:=true        # Launch indoor stack
ros2 run athena_gps_nav stack_check                                  # Check health of all nodes & topics
ros2 run athena_gps_nav gps_diagnose --rtcm-host $BASE_STATION_IP    # Diagnose RTK fix, hAcc & satellites
ros2 topic echo /cmd_vel_nav                                         # Inspect controller velocity output

# --- Motors & Calibration ---
fuser -v /dev/ttyACM*                                                # Check who holds Pico serial port
ros2 run athena_drive calibrate                                      # Interactive motor calibration wizard
ros2 run athena_drive teleop                                         # Keyboard teleop (arrows drive, space stop)

# --- Remote Access & Network ---
ssh robo@$JETSON_IP                                                  # Connect to Jetson
ssh -N -L 8765:localhost:8765 robo@$JETSON_IP                        # Forward Foxglove WebSocket
sudo systemctl restart systemd-timesyncd                             # Force NTP clock resynchronization
```

---

## Hardware Reference

| Subsystem | Device | Interface / Port | Driver Node |
|---|---|---|---|
| **Depth Camera** | Intel RealSense D435i | USB 3.0 | `realsense2_camera_node` |
| **IMU / RTK GNSS** | CubePilot Cube Orange (ArduPilot + DroneCAN RTK GPS) | USB Serial (`921600` baud, MAVLink 2) | `pixhawk_bridge` (`/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange*` / auto-detect) |
| **Motor Controller** | Raspberry Pi Pico W | USB Serial (`115200` baud) | `motor_bridge` (`/dev/serial/by-id/usb-Raspberry_Pi_Pico*`) |
| **Actuators** | 6x DC Motors + 3x Dual H-Bridges | 20 kHz PWM + Direction GPIO | Pico Firmware (`athena_drive_fw.ino`) |

> [!WARNING]
> The Raspberry Pi Pico's USB serial port is strictly exclusive. Always stop running launch stacks before using `calibrate` or other standalone drive tools. Check ownership using `fuser -v /dev/ttyACM*`.
