# Athena Gazebo Simulation Harness — Progress & Delivery Report

This report documents the completion of the simulation harness for the Athena rover in Gazebo Fortress (Ignition Gazebo 6) for the IRC (International Rover Challenge) 2027 finals.

---

## 1. Executive Summary

All 8 planned milestone items have been implemented, verified, and committed to branch `feature/sim` at commit `fb29418`:

| Item | Task Description | Status | Verification |
|---|---|---|---|
| **0** | Create `feature/sim` branch from `origin/main` | **Completed** | Checked out at commit `4b61424` |
| **1** | Verify Gazebo Fortress SDF/plugin semantics (DiffDrive, RGB-D, IMU, NavSat) | **Completed** | Clean SDF 1.8 syntax verified |
| **2** | Patch `athena.urdf.xacro` with `sim:=true` mode (continuous joints, inertials) | **Completed** | XML lint & joint checks pass |
| **3** | Write `athena_sim` package (launch scripts, bridge config, RTK GPS, VIO sim) | **Completed** | Configs and nodes verified |
| **4** | Procedural terrain & heightmap generator + 3D mission models | **Completed** | 257x257 PNG & 7 SDF models generated |
| **5** | IRC 2027 mission clock, rules engine, proximity scorer & briefing tool | **Completed** | Rules & scorer unit-tested |
| **6** | WSL unit test suite (no ROS 2 binary dependency required) | **Completed** | 28 / 28 unit tests pass (exit 0) |
| **7** | Git commit and sync state on `feature/sim` | **Completed** | Committed and synced |

---

## 2. Technical Implementation Details

### 2.1 URDF Simulation Patch (`athena.urdf.xacro` & `athena_sim.urdf.xacro`)
- **URDF Parameter**: Added `<xacro:arg name="sim" default="false"/>` and `<xacro:property name="is_sim" value="$(arg sim)"/>`.
- **Physics Fidelity**: When `is_sim:=true`, `base_link` receives a collision box and 45.0 kg inertia tensor. The 6 visual-only fixed wheel joints switch to `continuous` joints with cylinder collisions, 2.5 kg inertias, and friction parameters ($mu_1=1.0$, $mu_2=0.2$ lateral slip reduction for skid-steer rotation).
- **Gazebo Plugins**: `athena_sim.urdf.xacro` attaches `ignition-gazebo-diff-drive-system` (subscribed to `/cmd_vel`, publishing `/sim/odom` and `/sim/tf` to avoid conflicting with `ekf_local`'s ownership of `odom->base_link`), RealSense D435i RGB-D camera ($87^\circ \times 58^\circ$ FOV), Pixhawk Cube Orange IMU (50 Hz), and RTK GNSS NavSat (5 Hz).

### 2.2 ROS 2 $\leftrightarrow$ Gazebo Bridge (`config/bridge.yaml`)
Configured bidirectional and unidirectional bridging via `ros_gz_bridge`:
- `/clock` (`ignition.msgs.Clock` $\rightarrow$ `rosgraph_msgs/msg/Clock`)
- `/cmd_vel` (`geometry_msgs/msg/Twist` $\rightarrow$ `ignition.msgs.Twist`)
- `/camera/camera/color/image_raw` (`ignition.msgs.Image` $\rightarrow$ `sensor_msgs/msg/Image`)
- `/camera/camera/aligned_depth_to_color/image_raw` (`ignition.msgs.Image` $\rightarrow$ `sensor_msgs/msg/Image`)
- `/camera/camera/depth/color/points` (`ignition.msgs.PointCloudPacked` $\rightarrow$ `sensor_msgs/msg/PointCloud2`)
- `/imu/data` (`ignition.msgs.IMU` $\rightarrow$ `sensor_msgs/msg/Imu`)
- `/gps/fix_raw` (`ignition.msgs.NavSat` $\rightarrow$ `sensor_msgs/msg/NavSatFix`)
- `/sim/odom` (`ignition.msgs.Odometry` $\rightarrow$ `nav_msgs/msg/Odometry`)

### 2.3 RTK GNSS Error Model (`sim_gps_rtk.py`)
Replaces ideal simulator GPS with realistic multi-state RTK behavior matching `pixhawk_bridge.py`:
- `RTK_FIXED`: $\sigma \approx 0.02$ m horizontal accuracy, `STATUS_GBAS_FIX` (2), covariance diagonal $0.0004$ m².
- `RTK_FLOAT`: $\sigma \approx 0.35$ m horizontal accuracy, `STATUS_SBAS_FIX` (1), covariance diagonal $0.12$ m².
- `STANDALONE`: $\sigma \approx 1.80$ m horizontal accuracy, `STATUS_FIX` (0), covariance diagonal $3.24$ m².
- `NO_FIX`: Status $-1$, covariance $> 10^4$ m².
- Dynamic override via `/sim/set_gps_mode`.

### 2.4 Option B VIO Emulator (`vio_sim.py`)
Generates realistic `/vio/odometry` without GPU image rendering:
- $+1.5\%$ velocity scale factor bias.
- Cumulative yaw random walk drift ($0.05^\circ / \text{meter}$).
- Covariance expansion in featureless sand dunes.

### 2.5 Procedural Arena & Heightmap Generator (`generate_terrain.py`)
Generates an authentic $120 \times 120$ m Mars-analog arena ("Sproscape-Sim"):
- $257 \times 257$ 8-bit grayscale PNG heightmap generated via pure Python `zlib` and `struct`.
- Flat base station start line ($y \in [0, 15]$ m).
- Sand dunes with $15-20^\circ$ slopes.
- Rock garden ($0.3-1.0$ m roughness).
- Impact crater ($6$ m diameter, $1.4$ m depth, $0.6$ m raised rim).
- Trench crossing ($2.5$ m width, $0.8$ m depth).
- Negative obstacle ($0.5$ m vertical drop).

### 2.6 IRC 2027 Rules Engine & Live Scorer (`mission_runner.py`)
- **25-minute mission clock**.
- **7-minute checkpoint**: Aborts automatically if 0 points scored in first 420 seconds.
- **15-minute gate**: Requires $>30\%$ score to proceed to remaining 10 minutes.
- **Proximity rings**: Ring 1 ($<0.5$ m) = 100 pts, Ring 2 ($<1.0$ m) = 70 pts, Ring 3 ($<2.0$ m) = 40 pts.
- **Autonomy multiplier**: $1.0\times$ autonomous, $0.5\times$ teleoperated.
- **Penalties**: $-10\%$ per red-zone penetration, $-30\%$ per human intervention (max 2).

---

## 3. Test Suite & Verification Results

All 28 unit tests pass cleanly in WSL under Python 3.14:

```
test_bridge_yaml_validity ... ok
test_delivery_waypoints_validity ... ok
test_nav2_params_sim_all_nodes_use_sim_time ... ok
test_sproscape_zones_validity ... ok
test_dynamic_mode_switch ... ok
test_invalid_mode_raises_error ... ok
test_no_fix_mode ... ok
test_rtk_fixed_mode ... ok
test_rtk_float_mode ... ok
test_standalone_mode ... ok
test_clock_increment_and_cap ... ok
test_delivery_scoring_rings ... ok
test_intervention_limits_and_penalties ... ok
test_red_zone_violation_penalty ... ok
test_report_generation ... ok
test_seven_minute_abort_rule ... ok
test_flat_world_sdf_xml_syntax ... ok
test_heightmap_generation_dimensions_and_normalization ... ok
test_png_writer_creates_valid_png_header ... ok
test_world_sdf_xml_syntax ... ok
test_athena_sim_has_diff_drive_plugin ... ok
test_athena_sim_has_sensor_plugins ... ok
test_athena_sim_urdf_xacro_parses_cleanly ... ok
test_athena_urdf_xacro_parses_cleanly ... ok
test_sim_arg_present_in_athena_xacro ... ok
test_feature_quality_dropout ... ok
test_forward_integration ... ok
test_scale_factor_bias ... ok

Ran 28 tests in 0.046s
OK
>>> ALL ATHENA_SIM TESTS PASSED SUCCESSFULLY! (EXIT 0)
```
