# Tuning

Parameter by parameter: what each is now, and why. Costmap and controller
values are in `config/nav2_params.yaml`, filter values in
`config/dual_ekf_navsat.yaml`; with `--symlink-install`, editing either takes
effect on the next relaunch. Back to the [package README](../README.md). For
the design reasoning see [../ARCHITECTURE.md](../ARCHITECTURE.md).

Symptom first? Jump to [Symptom table](#symptom-table).

## Obstacle detection

Both costmaps carry the same obstacle layer, `stvl_layer`
(spatio_temporal_voxel_layer), reading the one cloud twice: `realsense_mark`
marks, `realsense_clear` clears by view frustum. Inflation differs.

Under `stvl_layer` -> `realsense_mark`, identical in both:

| Parameter | Now | What it does |
|---|---|---|
| `topic` | `/camera/camera/depth/color/points_downsampled` | the cloud both costmaps consume (not the raw one) |
| `filter` | `passthrough` | **required**: STVL applies the two height limits only inside this filter. With `""` the floor is marked lethal everywhere |
| `obstacle_range` | 3.0 m | how far obstacles get marked |
| `min_obstacle_height` | 0.10 m | below this counts as ground. Measured in the **costmap** frame where `base_link` is z = 0, so 0.10 is 0.20 m above the actual ground. It is not a camera height; setting it from one is how the floor ends up marked |
| `max_obstacle_height` | 1.8 m | higher is ignored (doorframes, branches) |
| `expected_update_rate` | 2.0 s | if no cloud arrives for this long the costmap reports itself not-current and the controller stops. Unset (0.0) means never checked: a wedged camera would leave stale obstacles frozen with no warning |

Under `stvl_layer` -> `realsense_clear`, identical in both. A voxel inside
this frustum that no new point re-marks is removed, whatever is behind it:

| Parameter | Now | What it does |
|---|---|---|
| `vertical_fov_angle` / `horizontal_fov_angle` | 1.05 / 1.17 rad | must sit inside the real view (measured +/-32.2 / +/-39.5 deg), or obstacles the camera cannot see get erased. STVL's frustum is vertical half-angle = v/2, horizontal half-angle = atan(tan(h/2)/cos(v/2)), so these give 30.0 / 37.4 deg. Re-derive if the depth profile changes |
| `min_z` / `max_z` | 0.45 / 4.2 m | the near and far planes sit at these times cos(v/2)cos(h/2), i.e. 0.32 m and 3.03 m along the optical axis. `max_z` 3.0 would stop clearing at 2.07 m |
| `decay_acceleration` | 10 local, 5 global | how fast unobserved voxels in view die: a removed object clears in about 1.0 s locally and 1.5 s globally. 20 was too aggressive, an obstacle in plain view blinked out in a normal cloud gap |
| `clear_after_reading` | True | **required**: with False the last frustum is re-applied every update during a camera stall and in-view obstacles are erased before `expected_update_rate` trips |

Under `stvl_layer` itself:

| Parameter | local | global | What it does |
|---|---|---|---|
| `voxel_size` | 0.05 | 0.1 | match the costmap resolution |
| `decay_model` / `voxel_decay` | 0 (linear) / 30 s | -1 (persistent) / 15 | memory for obstacles that left the view. Local: expire 30 s after last seen, which keeps the grid bounded. Global: kept until the camera sees the space empty; `voxel_decay` is then only the in-view clearing budget |
| `mark_threshold` | 0 | 0 | voxels per column needed to mark a cell lethal. Raise to 1 or 2 if depth noise makes phantom obstacles, but only with a lower `scarcity` in the downsampler: at every 8th point a real thin obstacle at 3 m may fill only one voxel and vanish |

Inflation differs by design:

| | local | global | Why |
|---|---|---|---|
| `inflation_radius` | 0.9 m | 1.2 m | the global map shapes the route, so a broad gentle gradient makes the planner curve away early. The local map controls, so it stays tight: wide inflation would box the controller in and it would refuse gaps the rover fits through |
| `cost_scaling_factor` | 3.0 | 2.0 | steeper locally, gentler globally |

Both radii must stay at or above the footprint's circumscribed radius,
`hypot(0.65, 0.5)` = 0.82 m for the 1.3 x 1.0 m footprint, or Nav2 warns and
nothing covers the rover's own corner sweep. Sizes: local 8 x 8 m at 0.05 m in
`odom`; global 100 x 100 m at 0.1 m in `map`; both rolling, and a rolling window
sees only width/2 in any direction (60 m capped every leg at ~30 m).
`always_send_full_costmap` is True locally (160 x 160 cells) and **False**
globally: 1e6 cells at 1 Hz is ~1 MB/s down the Foxglove tunnel.

Downsampler (`pointcloud_downsampler`): `scarcity` 8 (keep every 8th point),
`min_distance` 0.25 m (drops self-hits), `max_distance` 3.5 m.

## Speed and steering

| Parameter | Now | Why |
|---|---|---|
| `desired_linear_vel` | 0.35 m/s | cruise |
| `lookahead_dist` | 1.0 m | fixed, see below |
| `use_velocity_scaled_lookahead_dist` | **false** | see below |
| `min_lookahead_dist` / `max_lookahead_dist` | 0.9 / 1.5 m | unused while scaling is off; kept above the 0.65 m footprint so re-enabling scaling cannot put the carrot inside the robot |
| `rotate_to_heading_angular_vel` | 0.4 rad/s | was 0.8. VIO runs ~10 Hz outdoors and 2-4 Hz indoors; at 0.8 rad/s the rover sweeps 12-25 degrees between pose updates, every cloud in that gap is placed with a stale heading, and at 3 m that smears an obstacle by up to 0.8 m. Smear the camera can still see clears in about 1 s; smear outside the view stays until it expires (30 s locally, never globally) |
| `behavior_server: max_rotational_vel` | 0.5 rad/s | was 0.8; matched to the above |
| `velocity_smoother: max_velocity` angular | 0.6 rad/s | was 1.2, far above anything upstream asks for; a high ceiling only matters when something misbehaves, then it lets it misbehave fast |
| `xy_goal_tolerance` / `yaw_goal_tolerance` | 0.5 m / 3.14 rad | GPS-grade: do not chase centimetres with a +-3.5 m sensor. Yaw unconstrained on purpose |
| `required_movement_radius` / `movement_time_allowance` | 0.3 m / 30 s | this drive is slow to get going; be patient before calling it stuck |

**Do not turn velocity-scaled lookahead back on.** With scaling,
`lookahead = clamp(|speed.linear.x| * lookahead_time, min, max)`, and speed is
zero when every goal starts, so the carrot pins to `min_lookahead_dist`. At the
old 0.4 m that put it *inside* the rover (footprint reaches x = +0.65). NavFn
snaps the path start to a 0.1 m cell, which alone put the carrot at -0.58 rad,
past `rotate_to_heading_min_angle` (0.5), so the controller zeroed `linear.x` and
turned in place, and could never recover because building speed is what would
have lengthened the lookahead. Measured on a 2 m goal down a clear corridor:

| | forward command |
|---|---|
| velocity-scaled, 0.4 m carrot | `linear.x` = 0.000 on 160 of 160 cycles |
| fixed 1.0 m carrot | `linear.x` = 0.350 on 178 of 178 cycles |

The other half of that bug was `controller_server` having no `odom_topic`: Nav2's
default is the bare topic `odom`, which nothing here publishes, so the controller's
speed estimate was 0.0 forever. `bt_navigator`, `controller_server` and
`velocity_smoother` now all use `/odometry/local`.

## Recovery behaviour

Nav2 runs Athena's own tree, `behavior_trees/athena_nav_to_pose.xml`, set by
`bt_navigator: default_nav_to_pose_bt_xml`. The main branch is stock (replan at
1 Hz, follow the path, each retrying once). **Spin and BackUp are removed**: the
D435i sees 87 degrees and nothing faces backwards, so both move a 1.3 m rover
through space it cannot see. What remains: clear both costmaps, wait 5 s, repeat
up to 6 rounds, then fail the goal cleanly and hand control back to the operator.
Stock `Spin(1.57 rad)` is exactly 90 degrees counter-clockwise; if the rover
turns exactly 90 degrees left again, the stock tree is in use.

To restore stock behaviour, delete the `default_nav_to_pose_bt_xml` line from
`config/nav2_params.yaml` and relaunch; the `spin` and `backup` plugins are
still loaded. Reasoning: [../ARCHITECTURE.md](../ARCHITECTURE.md#recovery-why-spin-and-backup-were-removed).

## Visual odometry and stationary drift

A stationary rover should report zero motion; visual odometry never does exactly.
Two independent mechanisms hold the drift down.

**1. `Odom/FilteringStrategy` must stay `0`** (`vio.launch.py`). It is documented
as smoothing odometry output but is not a downstream smoother: rtabmap's Kalman
option overwrites the pose increment *and* becomes the next frame's motion guess,
closing a feedback loop that turns small perturbations into a sustained
directional velocity. Set to `1` it produced a steady ~2.8 cm/min walk; at `0`:

| | before | after |
|---|---|---|
| raw VO mean velocity | ~0.00047 m/s | **0.00019 m/s** |
| mean / std | biased | **0.021** |
| implied drift | 2.8 cm/min | ~1.2 cm/min (within noise) |

Mean/std above ~0.3 means a real bias; 0.021 is statistically zero. For in-VO
smoothing use `Odom/GuessSmoothingDelay` (0.15), which averages velocities for
the *guess* only.

**2. Zero-velocity update in `vio_gate`.** rtabmap's built-in ZUPT exists only for
the OpenVINS backend, which the apt package lacks, so `vio_gate` republishes the
twist as **exact zero with small covariance** (`zupt_variance` 1e-4) when the
rover is stationary. Zeroing, not suppressing, is deliberate: a suppressed message
leaves the filter coasting on its last velocity. Stationarity comes from
`/cmd_vel` (a commanded zero within `zupt_cmd_epsilon` 0.01 for `zupt_settle` 0.5
s). With no `/cmd_vel` publisher at all (bench, no motors) it assumes stationary;
`require_cmd_vel:=true` disables that. `zupt_enabled:=false` turns the whole
thing off for comparison:

```bash
ros2 run athena_gps_nav vio_gate --ros-args -p zupt_enabled:=false
```

Keep both: the parameter removes the cause, the ZUPT bounds the residual.

Other `vio.launch.py` values that matter: `approx_sync_max_interval: 0.02` (without
it `approx_sync` pairs RGB and depth frames two frames, 66.7 ms, apart and
`rgbd_odometry` reports phantom velocities of 0.7-1.6 m/s on a stationary rover);
`Vis/MinInliers: 20` (12 accepted badly conditioned geometry); `Reg/Force3DoF:
true`; `Vis/MaxDepth: 5.0`; `Odom/ImageDecimation: 2`; `Vis/MaxFeatures: 600`;
`OdomF2M/MaxSize: 1000` (the last three are Jetson CPU relief).

## How hard GPS pulls the map

If `/odometry/global` jumps, the filter trusts GPS more than it deserves.
`pixhawk_bridge` tells it how noisy GPS is:

```
sigma^2 = (HDOP x gps_uere)^2 + gps_sigma_floor^2        # defaults: gps_uere 1.74, gps_sigma_floor 3.06
```

Raising either makes the filter trust GPS less. Measure, don't guess: the
textbook `sigma = HDOP x UERE` does not fit this receiver. Two stationary runs:

| Run | mean HDOP | actual 1-sigma |
|---|---|---|
| A | 0.98 | 3.50 m |
| B | 1.91 | 4.52 m |

No single UERE fits both (3.57 for A, 2.36 for B), because much of the error is
slow wander that HDOP cannot predict (within 30 s sigma is only 1.3-1.8 m, but the
solution walks several metres over minutes). An HDOP-independent floor reproduces
both to the centimetre: `(0.98 x 1.74)^2 + 3.06^2` -> 3.50 m and `(1.91 x 1.74)^2 + 3.06^2` -> 4.52 m.

To re-derive after moving the antenna or the site, take two stationary runs at
*different* HDOP and solve:

```
k^2 = (s2^2 - s1^2) / (HDOP2^2 - HDOP1^2)   -> gps_uere
f^2 =  s1^2 - k^2 * HDOP1^2                 -> gps_sigma_floor
```

The old flat `UERE = 2.0` claimed 1.96 m against a true 3.50 m at good HDOP: the
filter was told GPS was twice as good as it is, exactly when it trusts it most.

Limits: do **not** fix jumpiness by turning GPS off in `odom1_config` (you lose
the absolute position that makes `map` worth having). Do **not** inflate the floor
until GPS is ignored (waypoints land in the wrong place); if sigma is really tens
of metres, the answer is sky view or RTK. None of this touches the `odom` frame:
if *obstacle avoidance* jitters, the problem is visual odometry.

## Heading: the compass is not used

**Neither** filter fuses absolute yaw. Both take the Pixhawk gyro rate only
(index 11 of `imu0_config`; index 5 is false in `ekf_local` and `ekf_global`).
This FMU's attitude solution is untrustworthy: flat, its quaternion decodes to
roll -22.3 / pitch -15.4 degrees, inconsistent between sessions (27.5 degrees on
2026-08-30, -21 / -17 on 2026-08-31). Fused, it anchored the map frame's rotation
to it and gave a permanent `map -> odom` yaw of +89 degrees, so every absolute
map-frame goal was aimed about 90 degrees off. A rate gyro does not depend on the
attitude solution, so the gyro stays; that also keeps the EKFs near 20 Hz instead
of 7-8.

**Cost:** map yaw is dead-reckoned, so `map` and `odom` agree in rotation and
drift together. Fine for the odom-frame goals this rover drives; **not** good
enough for GPS waypoint work, which needs a real heading again (index 5 against
a calibrated compass, or dual-antenna RTK). `navsat_transform`'s
`use_odometry_yaw` is `false`, so it also reads `/imu/data` yaw for the datum
heading; fix both together. Until then the two heading rows of the symptom table
below apply only if absolute yaw is re-enabled.

## The map origin must come from a settled fix

`navsat_transform` latches the map origin from the first fix it accepts,
`delay` seconds after start. The stock `delay: 3.0` is a trap: a just-booted
receiver is still converging, and the origin landed **110 m** from the settled
solution, so the filter spent minutes crawling toward it (a huge
`/odometry/global` vs `/odometry/gps` residual that looks like a tuning fault).
`delay` is **45 s**. These should agree within a few metres:

```bash
ros2 topic echo /odometry/global --field pose.pose.position
ros2 topic echo /odometry/gps    --field pose.pose.position
```

For anything repeatable (recorded waypoints, comparing runs) pin the datum in
`config/dual_ekf_navsat.yaml`, so `map` means the same thing every boot:

```yaml
navsat_transform:
  ros__parameters:
    wait_for_datum: true
    datum: [25.2629548, 82.9838284, 0.0]    # lat, lon, yaw; the site datum the bench simulator also uses
```

None of this applies under `local_only`, where `navsat_transform` does not run
and `map -> odom` is a static identity.

## Symptom table

| Symptom | Fix |
|---|---|
| Heading rotated by a constant angle | Set `magnetic_declination_radians` (NOAA value, East positive). Only matters once absolute yaw is fused again |
| Heading garbage near the motors | Absolute yaw is not fused, only gyro rate. `imu0_differential` is moot while index 5 is false |
| Filter lags real motion | Raise vx/vy/vyaw process noise (rows 7/8/12) |
| Visual odometry keeps resetting | Keep texture and depth in view. `Vis/MinInliers` is 20 on purpose; lower only as a last resort. Tilting the camera down helps, but re-measure `camera_pitch` after |
| Rover overshoots waypoints | Lower `desired_linear_vel`, raise `xy_goal_tolerance` |
| Rover yaws in place instead of driving | `lookahead_dist` and `odom_topic` (above), then check for a phantom obstacle: `python3 ~/athena/indoor_test/mount_check.py costmap` |
| Costmap misses obstacles | Check the downsampled cloud rate; lower `scarcity` on the downsampler |
| Costmap goes not-current and the rover stops | The camera stopped for more than `expected_update_rate` (2 s). Working as intended; check the RealSense, not Nav2 |
| Robot size or camera mount changed | Edit `footprint` in both costmaps (a rectangular polygon, not `robot_radius`: a circumscribed circle is needlessly conservative at this aspect ratio) and `urdf/athena.urdf.xacro`, relaunch, then re-measure with `mount_check.py camera` |
