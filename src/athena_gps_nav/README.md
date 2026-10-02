# athena_gps_nav

Outdoor navigation for the Athena rover: a depth camera to see obstacles, GPS
and an IMU to know where it is, and Nav2 to drive. ROS 2 Humble, Nav2 1.1.20,
robot_localization 3.5.4, rtabmap 0.23.7, Jetson.

This page is how to run it. How it works: [ARCHITECTURE.md](ARCHITECTURE.md).

| I want to | Read |
|---|---|
| Check the stack is healthy, test with RViz or by hand | [docs/TESTING.md](docs/TESTING.md) |
| Change a costmap, speed, GPS or filter parameter | [docs/TUNING.md](docs/TUNING.md) |
| Fix a sensor, GPS or camera problem; see known limitations | [docs/TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) |
| Understand the design and why it is the way it is | [ARCHITECTURE.md](ARCHITECTURE.md), [system overview](../docs/ARCHITECTURE.md) |

## Set up (once, per machine)

```bash
sudo apt install \
  ros-humble-nav2-bringup ros-humble-nav2-simple-commander \
  ros-humble-robot-localization ros-humble-rtabmap-odom \
  ros-humble-realsense2-camera ros-humble-imu-filter-madgwick \
  ros-humble-foxglove-bridge ros-humble-robot-state-publisher \
  ros-humble-xacro ros-humble-tf2-ros ros-humble-tf-transformations \
  ros-humble-sensor-msgs-py
pip3 install pymavlink

cd ~/athena                                  # the workspace root, never src/ or a package
colcon build --symlink-install
source install/setup.bash
ros2 run athena_drive calibrate              # motors, once; wheels off the ground
```

- `package.xml` does not declare `pymavlink` or `rtabmap_odom`; neither breaks the
  build, both surface as an `ImportError` or missing executable at launch.
- Build from `~/athena` with `--symlink-install` (traps explained in
  [CONTRIBUTING.md](../CONTRIBUTING.md)): `colcon` run from inside a package builds
  nothing and exits 0, and without the flag Python edits are ignored. With it,
  *editing* `config/`, `urdf/`, `launch/` or `behavior_trees/` takes effect on the
  next relaunch; *adding* a file needs a rebuild.
- Calibration is read by the motor bridge; until it exists the rover drives but not
  at the speeds Nav2 thinks it commanded ([athena_drive](../athena_drive/README.md)).
- **Do not** `source ~/rtab/install/setup.bash`: that source build is ABI-broken
  and crashes instantly. rtabmap comes from apt.
- `~/.bashrc` on this Jetson also sources `~/ros2_ws`, an unrelated rover project
  with its own from-source `realsense2_camera`. `sensors.launch.py` therefore
  starts the RealSense driver by absolute path to the apt binary; that is why its
  `executable=` looks odd.

## Run

```bash
ros2 launch athena_gps_nav bringup.launch.py          # everything; give it ~40 s
ros2 run athena_gps_nav stack_check                   # PASS/FAIL per stage, then exits
```

One command starts sensors, visual odometry, both filters, Nav2, the motor bridge
and the Foxglove panel, staggered so each layer has what it needs (order and
reasons: [system overview](../docs/ARCHITECTURE.md#launch-order)). These four are
the only arguments (`ros2 launch athena_gps_nav bringup.launch.py --show-args` is
authoritative; others are silently ignored):

| Argument | Effect |
|---|---|
| `dry_run:=true` | motor commands logged, not sent; safe on a desk |
| `use_camera:=false` | no RealSense: no obstacle avoidance **and** no visual odometry, so the local EKF has no translation source and `odom` will not track motion |
| `foxglove:=false` | no control panel, lower CPU |
| `local_only:=true` | indoor / no-GPS mode (below) |

**Indoors, use `local_only:=true`.** It drops `ekf_global` and `navsat_transform`
and publishes a static identity `map -> odom`; Nav2's config is untouched
(`global_frame` stays `map`) and switching back is dropping the flag. With no fix
`ekf_global` would dead-reckon the same inputs as `ekf_local`, and the two
diverging *is* the indoor `map -> odom` drift
([why](../docs/ARCHITECTURE.md#key-design-decisions)). `pixhawk_bridge` still
publishes `/gps/fix`. `stack_check` then reports `EKF global (map)` and `navsat
gps->map` as FAIL: expected.

Launch one layer at a time to debug it:

| Launch file | Arguments | Starts |
|---|---|---|
| `sensors.launch.py` | `use_camera` | robot description, camera driver, camera IMU filter, Pixhawk bridge, cloud downsampler |
| `vio.launch.py` | none | rtabmap visual odometry + `vio_gate` |
| `localization.launch.py` | `use_sim_time`, `local_only` | two EKFs + navsat_transform, or one EKF + static `map -> odom` |
| `navigation.launch.py` | `use_sim_time` | Nav2 |
| `bringup.launch.py` | `dry_run`, `use_camera`, `foxglove`, `local_only` | all of the above + motor bridge + Foxglove |
| `bench_test.launch.py` | none | fake sensors + the real pipeline; no hardware at all |

## Send it somewhere

- **Click:** RViz **2D Goal Pose** 2-3 m ahead in free space, or the Foxglove 3D
  panel. A path appears on `/plan` and `/cmd_vel` starts (`ros2 topic hz /cmd_vel`).
  With `dry_run:=true` nothing moves; `/cmd_vel` existing proves planning and control.
- **By GPS (needs a fix):** type a coordinate in the Foxglove *Go to* tab
  ([athena_remote](../athena_remote/docs/FOXGLOVE_PANEL.md#driving-it)), or record
  and replay a route (ENTER saves a waypoint; file format in
  `config/waypoints_example.yaml`):

```bash
ros2 run athena_gps_nav gps_waypoint_logger --ros-args -p output_file:=/home/robo/route.yaml
ros2 run athena_gps_nav gps_waypoint_follower --ros-args -p waypoints_file:=/home/robo/route.yaml
ros2 run athena_gps_nav gps_waypoint_follower --ros-args -p goal_lat:=25.26295 -p goal_lon:=82.98383   # one point
```

  Humble's Nav2 has no GPS waypoint action, so the follower converts each lat/lon
  through robot_localization's `/fromLL` and uses the ordinary waypoint follower.
  Absolute yaw is not fused, so map-frame bearings are dead-reckoned: read
  [TUNING.md](docs/TUNING.md#heading-the-compass-is-not-used) before trusting a
  recorded route.
- **A failed goal** (Nav2 here runs its own tree with no Spin or BackUp): clear
  costmaps, wait 5 s, up to 6 rounds, then fail cleanly and hand control back to
  you ([why](docs/TUNING.md#recovery-behaviour)).

## Nodes

The complete list of `ros2 run athena_gps_nav <exe>` targets:

| Node | Job |
|---|---|
| `pixhawk_bridge` | Pixhawk over MAVLink -> `/imu/data` and `/gps/fix`; aviation axes to ROS axes; HDOP to a real covariance |
| `vio_gate` | between visual odometry and the filters: drops motion the rover cannot have made, forces zero when parked |
| `pointcloud_downsampler` | thins the depth cloud so the costmaps are cheap to update |
| `stack_check` | one-shot PASS/FAIL health check |
| `localization_monitor` | records N seconds and writes an accuracy report |
| `costmap_drift_check` | measures whether the costmap slides when it should not |
| `gps_diagnose` | talks to the Pixhawk directly to explain why GPS has no fix. `argparse`, not a ROS node: `--seconds`, not `-p seconds:=` |
| `tf_check` | checks transforms are correctly *oriented*, not just present |
| `gps_waypoint_logger` / `gps_waypoint_follower` | record a route by driving it, then drive it autonomously |
| `fake_gps_imu` | synthetic GPS/IMU/VIO for `bench_test.launch.py`; not used on hardware |

Not in this package: motor control (`athena_drive`) and anything an operator
clicks (`athena_remote`).

## Key parameters

Tuning rationale and the rest: [docs/TUNING.md](docs/TUNING.md).

| Where | Parameter | Value |
|---|---|---|
| `nav2_params.yaml` | `desired_linear_vel` | 0.35 m/s |
| | `lookahead_dist` (fixed; `use_velocity_scaled_lookahead_dist: false`) | 1.0 m |
| | `rotate_to_heading_angular_vel` | 0.4 rad/s |
| | `xy_goal_tolerance` / `yaw_goal_tolerance` | 0.5 m / 3.14 rad |
| | local / global costmap | 8 x 8 m @ 0.05 m (`odom`) / 100 x 100 m @ 0.1 m (`map`) |
| | `inflation_radius` local / global | 0.9 / 1.2 m |
| | `obstacle_max_range` / `min_obstacle_height` / `expected_update_rate` | 3.0 m / 0.10 m / 2.0 s |
| | footprint | 1.3 x 1.0 m rectangle |
| `dual_ekf_navsat.yaml` | `navsat_transform: delay` | 45 s (pin a datum for repeatable runs) |
| | absolute yaw (`imu0_config` index 5) | false in both EKFs; gyro rate only |
| `pixhawk_bridge` | `baud`, `imu_rate_hz`, `gps_uere`, `gps_sigma_floor` | 921600, 50, 1.74, 3.06 |

Other files: `behavior_trees/athena_nav_to_pose.xml` (the tree), `urdf/athena.urdf.xacro`
(sensor positions; the camera offsets are load-bearing, re-measure with
`mount_check.py` after any remount, see [TESTING.md](docs/TESTING.md#camera-mount)),
`config/athena_bench.rviz`, `config/waypoints_example.yaml`.

## Contributors

- **Jashan**: the whole stack: sensor bridges, dual-EKF tuning, VIO gating, costmap work
