# Sensor, Filter, and Navigation Tuning Guide

Parameter references, configuration mechanics, and diagnostic procedures for `athena_gps_nav`.

Configuration files:
- Costmaps and Nav2 Controller/Planner: `config/nav2_params.yaml`
- Dual-EKF and `navsat_transform`: `config/dual_ekf_navsat.yaml`
- Sensor Drivers & Bridge: `launch/sensors.launch.py` and `pixhawk_bridge.py`

> [!TIP]
> Under `--symlink-install`, modifications to YAML files in `src/athena_gps_nav/config/` take effect immediately on the next node launch without requiring a re-compilation step.

---

## 1. RTK GNSS and Covariance Modeling

`pixhawk_bridge.py` dynamically adjusts `/gps/fix` position covariance based on whether the Cube Orange and DroneCAN receiver report an RTK solution or standalone GPS.

```
                    ┌─ Fix >= 4 (RTK) ────> sigma = max(h_acc, gps_rtk_sigma_floor)
GPS_RAW_INT ────────┤
                    └─ Fix < 4 (Auto) ────> sigma^2 = (HDOP * gps_uere)^2 + gps_sigma_floor^2
```

### RTK & GPS Parameters in `pixhawk_bridge`

| Parameter | Default | Range | Description & Tuning Guidance |
|---|---|---|---|
| `gps_rtk_sigma_floor` | `0.02` m | 0.01 – 0.05 m | Clamps the minimum horizontal standard deviation when RTK Float or Fixed (`fix_type >= 4`). Prevents infinite filter weighting when the receiver reports sub-centimeter accuracy. |
| `gps_uere` | `1.74` | 1.0 – 3.0 | User Equivalent Range Error multiplier for standalone GPS. Scales HDOP: $\sigma_{hdop} = \text{HDOP} \times \text{gps\_uere}$. |
| `gps_sigma_floor` | `3.06` m | 2.0 – 5.0 m | Baseline floor for standalone GPS. Captures low-frequency ionospheric and multipath wander that HDOP cannot predict. |
| `rtcm_enabled` | `true` | bool | Enables TCP client that pulls RTCM3 correction frames and injects them as MAVLink `GPS_RTCM_DATA` into the Cube Orange. |
| `rtcm_host` | `$RTCM_HOST` or `$BASE_STATION_IP` | IP / Hostname | IP address of the RTCM3 TCP streamer (e.g., STRSVR / `str2str` / NTRIP proxy). Defaults to `127.0.0.1` if variables are unset. |
| `rtcm_port` | `2101` | 1 – 65535 | TCP port of the base station RTCM stream. |
| `rtcm_stale_warn` | `10.0` s | 3.0 – 30.0 s | Interval of base station silence before logging warnings that corrections have stalled. |

### Nav2 Goal Tolerances vs GNSS Fix Type

In `config/nav2_params.yaml`, the goal tolerance must align with the current GNSS capability:

| Fix State | Typical $1\sigma$ Accuracy | Recommended `xy_goal_tolerance` | Recommended `yaw_goal_tolerance` |
|---|---|---|---|
| **RTK Fixed (`fix_type=6`)** | **1.5 – 3.0 cm** | **0.15 – 0.25 m** | 3.14 rad (unconstrained) |
| **RTK Float (`fix_type=5`)** | 15 – 40 cm | 0.35 – 0.50 m | 3.14 rad (unconstrained) |
| **Autonomous GPS (`fix_type=3`)** | 3.50 m (wander up to 10 m) | 0.50 – 1.0 m | 3.14 rad (unconstrained) |

> [!WARNING]
> Do **not** set `xy_goal_tolerance` tighter than `0.50 m` when operating on Autonomous GPS without RTK. The rover will circle or hunt around the goal point indefinitely because sensor wander exceeds the acceptance threshold.

### Map Datum Initialization in `navsat_transform`

`navsat_transform` anchors the Cartesian `map` frame origin from the first received GPS fix:
- **Automatic Latched Origin (`wait_for_datum: false`, `delay: 45.0` s):** Waits 45 seconds after launch to ensure the GNSS receiver has finished convergence before latching the origin.
- **Repeatable Static Origin (`wait_for_datum: true`):** For waypoint reproducibility across multiple operational runs, pin the datum explicitly in `config/dual_ekf_navsat.yaml`:

```yaml
navsat_transform:
  ros__parameters:
    wait_for_datum: true
    datum: [25.2629548, 82.9838284, 0.0]  # [Latitude, Longitude, Heading rad]
```

---

## 2. Heading and IMU Configuration

### Gyroscope Rate Fusion vs Compass Attitude

- `ekf_local` and `ekf_global` fuse **angular rate ($v_{yaw}$)** only (index 11 in `imu0_config`).
- Absolute attitude yaw (index 5) is set to **`false`** in both filters.
- **Rationale:** Onboard DC motor currents, steel chassis elements, and uncalibrated magnetometer offsets cause compass heading errors of 20° to 90°. Fusing compass attitude introduces large permanent yaw offsets between `map` and `odom`.
- **Operating Behavior:** Map yaw and odom yaw stay aligned via dead-reckoning. Global heading naturally aligns with the rover's trajectory once motion begins under RTK tracking.

### Online Gyro Bias Compensation (ZUPT)

`pixhawk_bridge.py` includes an automatic Zero-Velocity Update (ZUPT) estimator that compensates for MEMS gyro zero-rate bias:

| Parameter | Value | Description |
|---|---|---|
| `gyro_bias_correction` | `true` | Enables background ZUPT bias estimation. |
| `gyro_bias_min_samples` | `100` | Number of stationary samples (~2 s @ 50 Hz) required before activating bias subtraction. |
| `gyro_bias_window` | `1000` | Rolling sample window length and EMA time constant. |
| `gyro_bias_reject` | `0.15` rad/s | Discards samples exceeding threshold (rejects human jostles or external movement). |
| `gyro_bias_settle` | `0.5` s | Minimum seconds of zero `/cmd_vel` required before classifying rover as stationary. |

---

## 3. Visual Odometry & Stationary Drift Control

Stationary drift is suppressed via two independent mechanisms:

```
[Camera RGB-D] -> rtabmap (Odom/FilteringStrategy=0) -> vio_gate (ZUPT Injection) -> EKFs
```

1. **`Odom/FilteringStrategy = 0` (`vio.launch.py`):**
   - Must stay set to `0` (Frame-to-Frame / Frame-to-Map without internal Kalman feedback).
   - Setting to `1` closes a feedback loop in `rtabmap` where previous motion estimates contaminate subsequent guesses, inducing a synthetic ~2.8 cm/min drift.
2. **`vio_gate` Zero-Velocity Update (ZUPT):**
   - Monitors `/cmd_vel`. When linear and angular velocity commands are zero for >0.5 s, `vio_gate` republishes linear and angular twist velocities as **exact zero** with tight variance (`zupt_variance: 1e-4`).
   - Guarantees `<0.5 cm` net drift over 150 seconds when stationary.

---

## 4. Obstacle Detection & STVL Costmaps

Obstacle perception is implemented using `spatio_temporal_voxel_layer` (STVL) reading `/camera/camera/depth/color/points_downsampled`.

### Voxel Marking Layer (`realsense_mark`)

| Parameter | Value | Purpose |
|---|---|---|
| `topic` | `/camera/camera/depth/color/points_downsampled` | Downsampled point cloud (7.4 Hz). |
| `filter` | `passthrough` | **Mandatory.** Without this, STVL ignores height limits and marks the floor as lethal. |
| `min_obstacle_height` | `0.10` m | Obstacles below this are treated as traversable ground. Measured relative to `base_link` (z=0.10 m), meaning 0.20 m above floor. |
| `max_obstacle_height` | `1.8` m | Truncates overhanging structures and ceilings. |
| `obstacle_range` | `3.0` m | Maximum marking range for obstacle insertion. |
| `expected_update_rate` | `2.0` s | Watchdog interval: halts motion if camera point clouds stall for >2 s. |

### Frustum Clearing Layer (`realsense_clear`)

| Parameter | Value | Purpose |
|---|---|---|
| `vertical_fov_angle` | `1.05` rad (~60.1°) | Horizontal & vertical clearing bounds set safely inside RealSense physical FOV (measured ±32.2° V, ±39.5° H). |
| `horizontal_fov_angle` | `1.17` rad (~67.0°) | Prevents clearing obstacles outside actual camera field of view. |
| `min_z` / `max_z` | `0.45` m / `4.2` m | Near and far frustum clipping planes. |
| `decay_acceleration` | `10` (local), `5` (global) | Speed of voxel clearing: removed obstacles clear in ~1.0 s locally and ~1.5 s globally. |
| `clear_after_reading` | `true` | **Mandatory.** Ensures stale frustums are not repeatedly re-cleared during camera stalls. |

### Costmap Dimensions & Inflation

| Property | Local Costmap | Global Costmap |
|---|---|---|
| Frame | `odom` | `map` |
| Dimensions | 8.0 m $\times$ 8.0 m @ 0.05 m | 100.0 m $\times$ 100.0 m @ 0.10 m |
| Inflation Radius | `0.9` m | `1.2` m |
| Cost Scaling Factor | `3.0` (steep gradient) | `2.0` (gentle gradient) |
| Update Frequency | 1.5 – 2.0 Hz | 1.0 Hz |
| `always_send_full_costmap` | `true` (160 $\times$ 160 cells) | `false` (incremental updates save bandwidth) |

---

## 5. Speed, Steering, & Controller Tuning

Parameters located in `config/nav2_params.yaml` under `controller_server` (Regulated Pure Pursuit):

| Parameter | Value | Operational Reason |
|---|---|---|
| `desired_linear_vel` | `0.35` m/s | Normal cruise velocity. |
| `lookahead_dist` | **`1.0` m** | **Fixed lookahead.** Must remain $\ge 0.9$ m to stay clear of the 0.65 m front footprint. |
| `use_velocity_scaled_lookahead_dist` | **`false`** | Velocity scaling pins lookahead to minimum (0.4 m) at standstill, trapping the carrot inside the chassis and causing permanent yaw-in-place locks. |
| `rotate_to_heading_angular_vel` | `0.4` rad/s | Reduced from 0.8 rad/s. Prevents camera motion blur and costmap obstacle smearing during in-place rotation. |
| `max_rotational_vel` | `0.5` rad/s | Angular ceiling across behaviors. |
| `odom_topic` | `/odometry/local` | Set explicitly across `bt_navigator`, `controller_server`, and `velocity_smoother`. |

---

## 6. Diagnostic Symptom Resolution Table

| Symptom | Probable Cause | Action / Verification |
|---|---|---|
| **Fix stays at 3D / DGPS, never reaches RTK** | Base station RTCM stream unreachable or not injected. | 1. Check TCP connection: `nc -zv $BASE_STATION_IP 2101`.<br/>2. Verify bridge logs: `RTK: RTCM last 10 s [...]`.<br/>3. Verify Cube Orange parameter: `GPS_INJECT_TO=127`. |
| **Fix flaps between RTK Float and RTK Fixed** | Sky obstructions, high multipath, or weak satellite geometry. | 1. Move rover clear of metal structures/walls.<br/>2. Run `ros2 run athena_gps_nav gps_diagnose --rtcm-host $BASE_STATION_IP`.<br/>3. Ensure $\ge 16$ satellites in common view. |
| **Global odometry jumps violently** | Standalone GPS noise or RTK stream dropouts. | 1. Confirm `gps_rtk_sigma_floor: 0.02` is active.<br/>2. If on standalone GPS, check `gps_uere` (1.74) and `gps_sigma_floor` (3.06).<br/>3. Check `fix_type` via `ros2 topic echo /gps/fix --field status`. |
| **Rover yaws in place instead of driving to goal** | Lookahead carrot trapped inside footprint, or phantom floor obstacle. | 1. Confirm `use_velocity_scaled_lookahead_dist: false` and `lookahead_dist: 1.0`.<br/>2. Run `python3 ~/athena/indoor_test/mount_check.py costmap` to check for phantom obstacles. |
| **Costmap turns red / rover halts after 2 s** | RealSense depth stream dropped or USB autosuspend engaged. | Check `expected_update_rate: 2.0`. Verify camera frame arrival with `ros2 topic hz /camera/camera/depth/color/points_downsampled`. |
| **Rover turns 90° counter-clockwise and stops** | Phantom obstacle ahead triggered recovery. | Check camera tilt extrinsics (`camera_pitch`). Verify obstacle clearances using `mount_check.py costmap`. |
| **Heading drifts over long stationary periods** | Gyro bias learning not settled. | Keep rover stationary for $\ge 10$ seconds immediately after startup so ZUPT estimator converges. |
