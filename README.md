# Athena Autonomous Rover 

Athena is a high-mobility, six-wheeled autonomous outdoor rover designed and developed by the **Robotics Club at IIT (BHU) Varanasi**. 

The platform is engineered to autonomously navigate extreme, unstructured Martian-analog terrain—traversing boulders, soft sand dunes, steep terraced ridges, and amphi-craters. It performs centimeter-accurate RTK GNSS waypoint navigation, real-time 3D voxel obstacle perception (STVL), and multi-sensor fused state estimation without relying on external infrastructure.

---

## What is the Athena Rover?

Athena is a complete end-to-end robotics platform integrating heavy-duty mechanical design, custom motor drive electronics, multi-sensor embedded compute, and an autonomous software stack running **ROS 2 Humble Hawksbill** on an **NVIDIA Jetson**:

- **Autonomous Waypoint Navigation**: Fuses RTK GNSS satellite fixes, 6-DOF inertial measurement, and visual-inertial odometry into a Dual-EKF localization pipeline. Waypoints can be dispatched via GPS coordinates, operator click in Foxglove Studio, or mission-clock autonomous scripts.
- **3D Depth Perception & Obstacle Avoidance**: Utilizes an active-stereo Intel RealSense D435i RGB-D sensor to generate dense 3D point clouds. Obstacles, steep drop-offs, and negative terrain are tracked in real-time using a Spatio-Temporal Voxel Layer (STVL) costmap for dynamic Nav2 local path planning.
- **Heavy-Duty 6-Wheel Skid-Steer Drivetrain**: Six independent high-torque planetary-geared DC drive motors controlled by a custom Raspberry Pi Pico W microcontroller executing 20 kHz PWM generation, slew-rate current limiting, and emergency watchdog cutoffs.
- **Operator Cockpit & Web Telemetry**: Remote mission control interface powered by Foxglove Studio over high-bandwidth WebSocket bridges (`foxglove_bridge`), providing live 3D terrain visualization, path trails, manual speed-limited teleoperation, and one-click goal dispatch.
- **High-Fidelity Simulation Harness (`athena_sim`)**: An accurate Gazebo simulation (Harmonic 8.x / Fortress 6.x) replicating the 120m x 120m IRC 2027 Sproscape competition arena with procedural heightmaps, amphi-crater, Barchan dunes, Promontory ridge, RTK noise models, dual HD spectator cameras, and one-flag RViz2 visualization (`rviz:=true`).

---

## System Architecture

```
                                  +---------------------------------------+
                                  |            OPERATOR STATION           |
                                  |   Foxglove Studio / Teleop / Laptop   |
                                  +-------------------+-------------------+
                                                      |
                                                      | WebSockets / SSH (:8765)
+-----------------------------------------------------v-----------------------------------------------------+
|                                              NVIDIA JETSON                                                |
|                                                                                                           |
|   [Sensor Feeds]                   [Perception & Odometry]                  [Autonomous Navigation]       |
|   - RealSense D435i RGB-D -------> PointCloud Downsampler --------------->  Nav2 Local/Global Costmaps    |
|   - Pixhawk PX4 Cube Orange -----> rtabmap VIO + vio_gate --------------->  (STVL 3D Spatio-Temporal)    |
|     (Dual IMU + RTK GNSS) -------> Dual EKF (ekf_local + ekf_global) ---->  NavFn Global Planner          |
|                                                                          |  Regulated Pure Pursuit        |
|                                                                          |            |                   |
|                                                                          |         /cmd_vel               |
|                                                                          |            |                   |
|   [Drive Subsystem]                                                      |            v                   |
|   - athena_drive::motor_bridge <-----------------------------------------+-- athena_remote::teleop_mux    |
|     (Velocity profiling, PWM deadband, safety watchdog)                                                   |
+-----------------------------------------------------+-----------------------------------------------------+
                                                      |
                                                      | USB Serial @ 115200 baud (/dev/serial/by-id/...)
+-----------------------------------------------------v-----------------------------------------------------+
|                                          RASPBERRY PI PICO W                                              |
|                     Firmware: 20 kHz PWM generation, differential mixing, safety watchdog                 |
+-----------------------------------------------------+-----------------------------------------------------+
                                                      |
                                                      | High-current PWM motor driver lines
+-----------------------------------------------------v-----------------------------------------------------+
|                                        6x HIGH-TORQUE DC MOTORS                                           |
|                            Heavy-duty planetary gearboxes with high-traction cleated tires                |
+-----------------------------------------------------------------------------------------------------------+
```

---

## Package Directory & Documentation Index

| Package | Description | Reference Guide |
| :--- | :--- | :--- |
| **`athena_sim`** | Gazebo Harmonic/Fortress simulation harness, Sproscape 120x120m arena, RTK GPS noise model, VIO emulator, dual spectator cameras, RViz2 bringup | [src/athena_sim/README.md](src/athena_sim/README.md) |
| **`athena_gps_nav`** | Production navigation stack: Dual-EKF (`robot_localization`), Nav2 bringup, STVL 3D costmaps, RTK GPS waypoint follower, Pixhawk bridge | [src/athena_gps_nav/README.md](src/athena_gps_nav/README.md) |
| **`athena_drive`** | Six-wheel skid-steer motor bridge, interactive motor calibration wizard, wiring validator, and Raspberry Pi Pico C++ firmware | [src/athena_drive/README.md](src/athena_drive/README.md) |
| **`athena_remote`** | Operator telemetry suite: Foxglove WebSocket bridge, remote goal manager, path visualizer, and headless campus WiFi portal login | [src/athena_remote/README.md](src/athena_remote/README.md) |
| **`athena_slam`** | 3D Visual SLAM and point cloud filtering utilizing RTAB-Map and Intel RealSense RGB-D streams | [src/athena_slam/README.md](src/athena_slam/README.md) |
| **`docs/`** | Architectural specifications, TF frame tree, coordinate datums, and design decisions | [src/docs/ARCHITECTURE.md](src/docs/ARCHITECTURE.md) |
| **`CONTRIBUTING`**| Workspace conventions, colcon workflows, and branch management | [src/CONTRIBUTING.md](src/CONTRIBUTING.md) |

---

## Quick Table of Contents

1. [Plug-and-Play Setup & Installation](#1-plug-and-play-setup--installation)
2. [Simulation Bringup (Gazebo & RViz2)](#2-simulation-bringup-gazebo--rviz2)
3. [Physical Rover Bringup](#3-physical-rover-bringup)
4. [System Health & Diagnostics](#4-system-health--diagnostics)
5. [Operator Interface (Foxglove Studio)](#5-operator-interface-foxglove-studio)
6. [Everyday Operational Commands](#6-everyday-operational-commands)
7. [Hardware & Electrical Interfaces](#7-hardware--electrical-interfaces)
8. [Current System Status & Verification](#8-current-system-status--verification)

---

## 1. Plug-and-Play Setup & Installation

The repository is built for **Ubuntu 22.04 LTS** and **ROS 2 Humble**. All ROS and system dependencies are packaged for clean, single-command installation via `rosdep`.

### Step 1: Install Dependencies in One Go
From the workspace root (`~/Rover`):

```bash
cd ~/Rover

# 1. Update rosdep package database
rosdep update

# 2. Install all system and ROS packages across all repository modules
rosdep install --from-paths src --ignore-src -r -y

# 3. Install Pixhawk MAVLink Python interface (required for physical GPS bridge)
pip3 install pymavlink
```

### Step 2: Run Pre-Flight Environment Diagnostic Audit
Before launching, run the automated diagnostic auditor to verify OS compatibility, ROS packages, Gazebo version, Python modules, and serial access permissions:

```bash
python3 scripts/check_env.py
```
*(Can also be executed after building as: `ros2 run athena_sim check_env`)*

The script checks:
- **Host OS & Architecture:** Ubuntu 22.04 LTS (Jammy) on x86_64 or aarch64 (Jetson).
- **ROS 2 Distribution:** Active `ROS_DISTRO=humble` environment.
- **Gazebo Simulator:** Verifies `gz sim` (Harmonic 8.x) or `ign gazebo` (Fortress 6.x) and `ros_gz_bridge`.
- **Serial Permissions:** Confirms user is in the `dialout` group for `/dev/ttyACM*` access.
- **Python Libraries:** Confirms `numpy`, `PIL` (Pillow), `yaml`, `matplotlib`, `cv2`, `serial`, and `pymavlink`.
- **GPU Acceleration:** Validates DRI hardware acceleration (`/dev/dri/renderD128`) and display availability.

### Step 3: Build the Workspace
```bash
colcon build --symlink-install
source install/setup.bash
```

---

## 2. Simulation Bringup (Gazebo & RViz2)

The `athena_sim` package spawns Athena in the IRC 2027 Sproscape competition arena with full sensor simulation (IMU, RTK GNSS, RealSense RGB-D, wheel encoders, and dual HD spectator cameras).

### Launch Modes

```bash
# 1. Full Simulation (Gazebo 3D GUI + ROS Bridge + Dual EKF)
ros2 launch athena_sim sim.launch.py

# 2. Simulation WITH RViz2 Pre-Configured Layout (Includes RobotModel, PointCloud, Costmaps, Odom)
ros2 launch athena_sim sim.launch.py rviz:=true

# 3. Headless Simulation (For CI, remote servers, or headless laptops)
ros2 launch athena_sim sim.launch.py headless:=true

# 4. Headless Simulation + RViz2 (Run physics headlessly, view in RViz2)
ros2 launch athena_sim sim.launch.py headless:=true rviz:=true
```

### Key Launch Arguments

| Argument | Default | Description |
| :--- | :--- | :--- |
| `rviz` | `false` | When `true`, automatically launches RViz2 with `athena_sim.rviz`. |
| `rviz_config` | `athena_sim.rviz` | Path to custom RViz2 configuration file. |
| `headless` | `false` | When `true`, runs Gazebo physics server headless (`-s -r`). |
| `run_navigation`| `false` | When `true`, brings up the full Nav2 stack with STVL costmaps. |
| `run_mission` | `false` | When `true`, runs the IRC 2027 25-minute competition rules engine. |
| `gps_mode` | `fixed` | RTK GNSS noise model: `fixed` (2cm), `float` (35cm), `standalone` (1.8m), `no_fix`. |

For complete simulation documentation, spectator camera feeds, and orthographic topographic maps, see [src/athena_sim/README.md](src/athena_sim/README.md).

---

## 3. Physical Rover Bringup

On the physical rover (NVIDIA Jetson over SSH):

```bash
# 1. SSH into the Jetson
ssh robo@ubuntu.local    # or ssh robo@<jetson-ip>

# 2. Launch the master bringup stack
ros2 launch athena_gps_nav bringup.launch.py
```

One command initializes the entire hardware and software stack in a staggered sequence:
1. **0 s**: RealSense D435i camera, Pixhawk MAVLink bridge, robot description, motor bridge.
2. **5 s**: Visual-Inertial Odometry (`rtabmap rgbd_odometry`) and safety gate.
3. **8 s**: Dual EKF state estimation (`ekf_local` and `ekf_global`) and GPS datum transform.
4. **12 s**: Nav2 planner, controller, and 3D STVL costmaps.
5. **15 s**: Foxglove WebSocket bridge and operator telemetry nodes.

### Hardware Bringup Options

```bash
# Bench testing (motors receive commands but do not send them to wheels)
ros2 launch athena_gps_nav bringup.launch.py dry_run:=true

# Indoor navigation without GPS (disables global EKF drift, sets map = odom)
ros2 launch athena_gps_nav bringup.launch.py local_only:=true

# Camera-less dead-reckoning (disables visual odometry and costmaps)
ros2 launch athena_gps_nav bringup.launch.py use_camera:=false
```

---

## 4. System Health & Diagnostics

Verify that all sensors, transforms, and odometry pipelines are healthy using the automated diagnostic auditor:

```bash
ros2 run athena_gps_nav stack_check
```

Sample output from a healthy system:
```
SENSORS
  [PASS]  pixhawk imu                 48.5 Hz
  [PASS]  imu physics (at rest)      |g|=9.81 m/s2, attitude residual 0.002, r=+0 p=+0 deg
  [PASS]  pixhawk gps                  5.2 Hz, RTK FIXED (18 satellites)
  [PASS]  depth cloud (camera)         8.4 Hz

OBSTACLE PERCEPTION  (depth camera -> costmap)
  [PASS]  downsampled cloud            8.4 Hz, ~4500 pts/frame
  [PASS]  local costmap marking      540 lethal + 720 inflated cells
  [PASS]  global costmap marking     510 lethal + 680 inflated cells

ODOMETRY / LOCALIZATION
  [PASS]  VIO raw (rtabmap)           12.1 Hz
  [PASS]  VIO gated                   12.0 Hz
  [PASS]  EKF local (odom)            30.0 Hz
  [PASS]  EKF global (map)            28.5 Hz
  [PASS]  navsat gps->map              5.0 Hz
```

---

## 5. Operator Interface (Foxglove Studio)

Foxglove Studio provides the primary operator telemetry dashboard:

1. **Open SSH Port Forwarding** from your workstation:
   ```bash
   ssh -N -L 8765:localhost:8765 robo@<jetson-ip>
   ```
2. **Launch Foxglove Studio** and connect to `ws://localhost:8765`.
3. Open the pre-configured layout: [`src/athena_remote/config/athena_panel.json`](src/athena_remote/config/athena_panel.json).
4. Features:
   - **3D World View**: Live point clouds, robot footprint, local costmap, and Nav2 planned paths.
   - **Interactive Goal Tool**: Click anywhere on the map or enter WGS-84 GPS coordinates to dispatch goals.
   - **Emergency Stop & Teleop**: Speed-limited teleoperation override with emergency stop.

---

## 6. Everyday Operational Commands

| Objective | Command |
| :--- | :--- |
| **Launch Simulation** | `ros2 launch athena_sim sim.launch.py` |
| **Simulation with RViz2** | `ros2 launch athena_sim sim.launch.py rviz:=true` |
| **Pre-Flight Environment Check**| `python3 scripts/check_env.py` |
| **Bring Up Physical Rover** | `ros2 launch athena_gps_nav bringup.launch.py` |
| **Check System Health** | `ros2 run athena_gps_nav stack_check` |
| **Cancel Active Navigation Goal**| `ros2 topic pub --once /athena/goal_text std_msgs/String '{data: "cancel"}'` |
| **Calibrate Drive Motors** | `ros2 run athena_drive calibrate` *(elevate wheels off ground first)* |
| **Diagnose GPS / RTK Issues** | `ros2 run athena_gps_nav gps_diagnose` |
| **Inspect Motor Controller Port** | `fuser -v /dev/serial/by-id/usb-Raspberry_Pi_Pico*` |
| **Campus WiFi Headless Login** | `ros2 run athena_remote captive_login` |

---

## 7. Hardware & Electrical Interfaces

| Component | Interface / Connection | Role in Stack |
| :--- | :--- | :--- |
| **Intel RealSense D435i** | USB 3.0 Type-C | Active IR stereo depth point clouds (30 Hz) + RGB vision |
| **Pixhawk PX4 Cube Orange** | USB Serial @ **921600 baud** MAVLink (`/dev/serial/by-id/usb-3D_Robotics_PX4*`) | Dual IMUs (100 Hz) + RTK GNSS satellite fixes |
| **Raspberry Pi Pico W** | USB Serial @ 115200 baud (`/dev/serial/by-id/usb-Raspberry_Pi_Pico*`) | Motor controller PWM bridge; firmware in [`athena_drive`](src/athena_drive) |
| **6x Planetary DC Motors** | High-power H-Bridge drivers | Skid-steer drivetrain (independent PWM left/right banks) |

> [!IMPORTANT]
> Always reference serial devices via persistent `/dev/serial/by-id/...` symlinks. Direct `ttyACM*` enumeration can swap across reboots.

---

## 8. Current System Status & Verification

- **Autonomous Navigation & Costmaps**: Verified end-to-end on both hardware and simulation. STVL 3D voxel costmaps dynamically mark and clear obstacles in real-time.
- **Simulation Fidelity**: Verified against IRC 2027 Sproscape world specifications. Zero floating objects, grounded terrain heightmap, and multi-sensor streaming verified via automated QA audits.
- **Drive Kinematics**: Variable-frequency 20 kHz PWM motor bridge ensures smooth low-speed crawling and skid-steer pivoting with zero motor whine.
- **Visual-Inertial Odometry**: Gated VIO prevents EKF divergence during camera occlusion or rapid rotation.

---

## Contributors & Maintainers

- **Robotics Club, IIT (BHU) Varanasi**
- **Jashan**: Autonomy Stack, Simulation Harness, Motor Bridge, Remote Operator Interface

For development workflows and pull request guidelines, please refer to [CONTRIBUTING.md](src/CONTRIBUTING.md).
