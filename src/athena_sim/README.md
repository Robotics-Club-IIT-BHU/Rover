# athena_sim: Simulation Harness for Athena Rover (IRC 2027)

Comprehensive Gazebo Fortress (gz-sim 6) simulation harness for the Athena rover, built for the International Rover Challenge (IRC 2027) finals at Dayananda Sagar University, Bengaluru.

Enables end-to-end validation of the real autonomy stack (Dual EKF, Nav2, STVL 3D costmaps, RTK GNSS waypoint follower) in realistic Martian-analog terrain without hardware dependencies.

---

## Architecture Overview

```
                                      +--------------------------+
                                      |   Gazebo Fortress Sim    |
                                      | (Sproscape 120x120m SDF) |
                                      +--------------------------+
                                                   |
                     +-----------------------------+-----------------------------+
                     | (Ignition Transport)        | (Ignition Transport)        | (Ignition Transport)
                     v                             v                             v
           [/camera/camera/points]            [/imu/data_raw]             [/gps/fix_raw]
                     |                             |                             |
                     | ros_gz_bridge               | ros_gz_bridge               | ros_gz_bridge
                     v                             v                             v
        [/camera/camera/depth/points]         [/imu/data]                        |
                     |                             |                             v
                     | (STVL 3D Costmap)           | (Dual EKF yaw)       +---------------+
                     v                             v                      |  sim_gps_rtk  | (RTK error model)
               +-----------+                 +-----------+                +---------------+
               |   Nav2    | <-------------- |  ekf_loc  |                       |
               | (Planner/ |   /odom->base   |  ekf_glob | <--- [/gps/fix] <----+
               |Recovery)  |   /map->odom    +-----------+      (RTK Fixed/Float)
               +-----------+                       ^
                     |                             |
                     | /cmd_vel                    | [/vio/odometry]
                     v                             |
          +---------------------+            +---------------+
          |  Gazebo DiffDrive   |            |    vio_sim    | (Option B Drift model)
          | (6 continuous joint)|            +---------------+
          +---------------------+
```

---

## Package Structure

```
src/athena_sim/
├── athena_sim/
│   ├── sim_gps_rtk.py        # RTK GNSS error injector (Fixed/Float/Standalone/NoFix)
│   ├── vio_sim.py            # Visual-Inertial Odometry emulator with scale & yaw drift
│   ├── briefing_tool.py      # WGS-84 coordinate briefing generator from arena metadata
│   ├── mission_runner.py     # IRC 2027 rules engine (25m clock, 7m abort, red zones, scoring)
│   └── generate_terrain.py   # Procedural heightmap (257x257 PNG) & SDF world generator
├── config/
│   ├── bridge.yaml           # ros_gz_bridge topic definitions & QoS profiles
│   ├── nav2_params_sim.yaml  # Production Nav2 params with use_sim_time: True overrides
│   └── delivery_waypoints.yaml # Target waypoints for gps_waypoint_follower
├── docs/
│   └── MISSION_BRIEFING.md   # Generated operator briefing sheet
├── launch/
│   ├── sim.launch.py         # Full bringup (Gazebo + Bridge + Sensors + EKF + Nav2)
│   └── test_world.launch.py  # Minimal smoke test launch (Flat world + Robot)
├── models/                   # IRC 2027 3D models (SDF + config)
│   ├── rado_hammer/          # Graspable tool
│   ├── rado_wrench/          # Graspable tool
│   ├── rado_toolbox/         # 4.5kg toolbox
│   ├── delivery_marker/      # Target post with circular landing pad
│   ├── red_zone_marker/      # Contaminated boundary cone
│   ├── abex_sample_site/     # Science sampling patch
│   └── idmo_panel/           # 1.8m mock maintenance panel
├── test/
│   ├── run_all_tests.py      # Unified standalone test runner
│   ├── test_urdf.py          # URDF/XACRO validity & joint checks
│   ├── test_gps_rtk.py       # RTK error model & covariance verification
│   ├── test_vio_sim.py       # VIO drift kinematics verification
│   ├── test_mission_runner.py# IRC 2027 rules engine, abort & scoring tests
│   ├── test_terrain_generator.py # Heightmap & SDF syntax tests
│   └── test_configs.py       # YAML syntax & configuration checks
├── urdf/
│   └── athena_sim.urdf.xacro # Sim description extending athena.urdf.xacro
├── worlds/
│   ├── sproscape.sdf         # 120m x 120m Mars analog arena SDF
│   ├── sproscape_heightmap.png # 257x257 8-bit heightmap
│   ├── sproscape_zones.yaml  # Zone definitions and coordinates
│   └── test_flat.sdf         # Lightweight flat ground SDF
├── package.xml
├── setup.py
└── setup.cfg
```

---

## Key Capabilities & Realism

### 1. RTK GNSS Error Model (`sim_gps_rtk.py`)
Replaces ideal simulator GPS with realistic multi-state RTK behavior matching `pixhawk_bridge.py`:
- **RTK Fixed**: $\sigma \approx 0.02$ m horizontal accuracy, `NavSatStatus.STATUS_GBAS_FIX` (2), covariance matrix diagonal $\approx 4 \times 10^{-4}$ m².
- **RTK Float**: $\sigma \approx 0.35$ m horizontal accuracy, `NavSatStatus.STATUS_SBAS_FIX` (1), covariance matrix diagonal $\approx 0.12$ m².
- **Standalone 3D**: $\sigma \approx 1.80$ m horizontal accuracy, `NavSatStatus.STATUS_FIX` (0), covariance $\approx 3.24$ m².
- **No Fix**: Status $-1$, covariance $> 10^4$ m².
- **Runtime Override**: Send mode to `/sim/set_gps_mode` (`"fixed"`, `"float"`, `"standalone"`, `"no_fix"`) to test failovers mid-run.

### 2. VIO Emulator (`vio_sim.py`)
Injects `/vio/odometry` without GPU image rendering overhead:
- Simulates $+1.5\%$ velocity scale factor bias.
- Simulates cumulative yaw random walk ($0.05^\circ / \text{meter}$).
- Supports featureless terrain dropout via `/sim/vio_quality` to test degraded navigation in sand dunes.

### 3. Sproscape Procedural Arena
- Seeded $120 \times 120$ m heightmap.
- Includes flat base station starting line ($y \in [0, 15]$ m), sand dunes with $15-20^\circ$ slopes, rock garden ($0.3-1.0$ m roughness), impact crater with elevated rim, trench rift crossing, and a $0.5$ m vertical drop.

### 4. IRC 2027 Rules Engine & Scorer (`mission_runner.py`)
- **25-minute mission clock**.
- **7-minute abort rule**: Aborts automatically if 0 points are scored in the first 420 seconds.
- **15-minute gate**: Requires $>30\%$ score to proceed to remaining 10 minutes.
- **Delivery proximity rings**:
  - Ring 1 ($<0.5$ m): 100 points
  - Ring 2 ($<1.0$ m): 70 points
  - Ring 3 ($<2.0$ m): 40 points
- **Autonomy Multiplier**: $1.0\times$ autonomous, $0.5\times$ teleoperated.
- **Penalties**: $-10\%$ of max score per red-zone penetration, $-30\%$ per human intervention (max 2).

---

## Running Unit Tests (In WSL)

All units tests run directly with Python standard library (no ROS 2 or Gazebo binaries required):

```bash
python3 src/athena_sim/test/run_all_tests.py
```

Expected result:
```
Ran 28 tests in 0.046s
OK
>>> ALL ATHENA_SIM TESTS PASSED SUCCESSFULLY! (EXIT 0)
```

---

## Launching Full Simulation (In ROS 2 + Gazebo Environment)

### Option A: Headless Simulation (Default, Foxglove Visualization)
```bash
ros2 launch athena_sim sim.launch.py headless:=true run_localization:=true
```

### Option B: With Autonomous Nav2 Navigation
```bash
ros2 launch athena_sim sim.launch.py headless:=true run_localization:=true run_navigation:=true
```

### Option C: Minimal Smoke Test
```bash
ros2 launch athena_sim test_world.launch.py
```
