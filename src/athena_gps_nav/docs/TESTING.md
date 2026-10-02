# Checking that it works

Eyeballing tells you *whether* something is wrong; the automated checks tell
you *how much*. Back to the [package README](../README.md).

1. [Health check](#health-check): `stack_check`, every link PASS/FAIL
2. [RViz](#rviz): what to look at
3. [Camera mount](#camera-mount): `mount_check.py`
4. [Hands-on tests](#hands-on-tests): no motors needed
5. [Measurements](#measurements): drift, costmap sliding, TF orientation, no-hardware bench

## Health check

```bash
ros2 run athena_gps_nav stack_check                      # 10 s window
ros2 run athena_gps_nav stack_check --ros-args -p window:=30.0
```

It counts messages for the window, prints one block per stage plus `TF TREE`
and `NAV2` sections and a closing `ALL CHECKS PASS` or `n FAIL`, then exits on
its own (nothing to Ctrl-C). Extract from a healthy indoor run:

```
SENSORS
  [PASS]  pixhawk imu                 48.5 Hz
  [PASS]  imu physics (at rest)      |g|=9.81 m/s2, attitude-vs-accel residual 0.002, tilt r=+0 p=+0 deg
  [PASS]  pixhawk gps                  5.2 Hz, NO FIX (indoors?)
  [PASS]  depth cloud (camera)         6.6 Hz

OBSTACLE PERCEPTION  (depth camera -> costmap)
  [PASS]  downsampled cloud            6.6 Hz, 4812 pts/frame
  [PASS]  local costmap marking      767 lethal + 888 inflated cells
  [PASS]  global costmap marking     731 lethal + 902 inflated cells

ODOMETRY / LOCALIZATION
  [PASS]  VIO raw (rtabmap)           11.4 Hz
  [PASS]  VIO gated                   11.2 Hz (2 implausible dropped)
  [PASS]  EKF local (odom)            29.8 Hz
  [PASS]  EKF global (map)            27.3 Hz
  [FAIL]  navsat gps->map              0.0 Hz  (needs a GPS fix)
```

Expected, not bugs:

- `navsat gps->map` FAILs without a satellite fix (indoors, or the GPS problem in
  [TROUBLESHOOTING.md](TROUBLESHOOTING.md#gps-has-no-fix)). `navsat_transform`
  also waits `delay: 45.0` s before latching a datum, so re-run after a minute
  outdoors before believing that line. Obstacle avoidance depends on neither.
- With `local_only:=true`, `EKF global (map)` and `navsat gps->map` both FAIL: those
  nodes are deliberately not running.
- `NAV2 / cmd_vel` reads `0.0 Hz (idle - no goal sent)` and is never a FAIL.
- `imu physics (at rest)` compares the accelerometer against the FMU's own
  attitude quaternion, which is known bad on this FMU (flat, it decodes to roll
  -22.3 / pitch -15.4 degrees). A large attitude-vs-accel residual here is that
  known fault, not new damage; nothing fuses that attitude. `tf_check`'s gravity
  checks are the ones to trust (they use the URDF transform).
- Rate floors are set well below what the hardware achieves, so PASS means
  "alive", not "fast": IMU > 30 Hz, GPS > 1, depth > 3, downsampled cloud > 1,
  VIO > 2, EKFs > 15 Hz (healthy is 19-26, a lost Pixhawk IMU drops them to 7-8),
  navsat > 1.

## RViz

```bash
rviz2 -d ~/athena/src/athena_gps_nav/config/athena_bench.rviz
```

Run it on a **laptop** if you can (on the Jetson itself, directly or over `ssh -X`, it eats a whole CPU core and
roughly halves visual odometry, ~10 Hz to ~5 Hz). That needs `ROS_DOMAIN_ID=42` in
every shell on both ends and a network path that carries DDS; on campus WiFi it
usually does not, so use the setup in
[athena_remote Option C](../../athena_remote/README.md#option-c-native-ros-on-a-laptop)
or use Foxglove. The `.rviz` file is on the Jetson, so `scp` it to the laptop
first (the `athena_remote` guide has the command); plain `rviz2` with no `-d`
works if you add the displays by hand.

**Set Fixed Frame first** (Global Options): it decides what you are testing.

| Fixed frame | Tests |
|---|---|
| `odom` | camera and IMU only; GPS cannot affect anything you see. **Use this indoors and for everything below** |
| `map` | GPS too. Meaningful only with a fix; indoors it wanders metres with multipath, which is the GPS, not a bug |
| `base_link` | the rover's own view, with the world moving around it |

Displays (the `.rviz` file has them; add by hand otherwise):

| Display | Topic | Should look like |
|---|---|---|
| RobotModel | `/robot_description` | the rover, nose block forward |
| Local Costmap (Map) | `/local_costmap/costmap` | dark cells where obstacles are, lighter halo around them |
| Depth Cloud (PointCloud2) | `/camera/camera/depth/color/points_downsampled` | 3D points sitting exactly on those cells |
| Odometry local (green) | `/odometry/local` | an arrow that stays put when the rover does |
| Odometry global (orange) | `/odometry/global` | same, but drifts with GPS indoors; nothing publishes it under `local_only` |
| Path | `/plan`, `/athena/trajectory` | a line to your goal once you set one; the driven trail |
| MarkerArray | `/athena/waypoints` | saved places |
| TF | `/tf`, `/tf_static` | one connected tree, no floating frames |

A second Map display on `/global_costmap/costmap` comes up blank and fills in as
the rover moves: it is 100 x 100 m with `always_send_full_costmap: False`, so it
publishes incremental updates (a full 1e6-cell republish at 1 Hz is ~1 MB/s down
the SSH tunnel). The local costmap keeps `True`; it is only 160 x 160 cells.

Frames the URDF publishes under `base_link`: `base_footprint`, `camera_link`,
`imu_link`, `gps_link`, `nose_marker` and six `wheel_*` links (twelve links with
`base_link`), plus `camera_camera_link` bridged from the RealSense driver by a
static transform in `sensors.launch.py`. There are no mast frames: `camera_mast`
and `gps_mast` were deleted on 2026-09-03, so seeing either name in a config or
script means it is stale.

## Camera mount

The camera's position and tilt in `urdf/athena.urdf.xacro` are what the costmap
believes. Get them wrong and the rover marks the floor as a wall, refuses to
drive and looks like a Nav2 tuning problem. After moving the camera or Pixhawk, or
whenever obstacle marking looks wrong in a way software cannot explain, **measure
the mount** with `mount_check.py`: it fits the floor plane in the camera's own
optical frame and does not read the URDF, so it stays honest whatever the URDF claims.

```bash
python3 ~/athena/indoor_test/mount_check.py camera
```

It is a plain script (not a `ros2 run` target, not executable: run it with
`python3`), lives outside the source tree in `~/athena/indoor_test/`, needs the
stack up, and must run on a **flat floor with clear ground in view**, on the
rover (a desk reproduces a 60 cm height error exactly). Read-only except `yaw`,
which asks you to turn the rover by hand.

| Mode | Answers |
|---|---|
| `camera` | height, pitch and roll from the floor plane; prints `camera_z` and `camera_pitch` lines to paste into the URDF |
| `costmap` | highest cost inside the footprint right now, plus an ASCII map of the 3 m ahead. Tells you whether "sends goal, turns 90 degrees left" is a phantom obstacle under the rover |
| `yaw` | turn the rover by hand through 90 degrees; compares the Pixhawk gyro, VIO and the EKF. Catches an IMU mounted rotated or upside down |
| `watch` | `camera`, continuously: every few seconds, where flat ground 3 m ahead lands relative to `min_obstacle_height`; shouts when the floor starts marking. Leave it running during a test drive |

`camera` cannot measure `camera_x`: tape-measure it (lens to the middle of the
chassis, positive forwards). It supersedes `indoor_test/ground_check.py`, whose
suggested `camera_z` uses a hardcoded 0.20 / 0.30 m baseline from an older
mounting and now prints confident, wrong advice: do not use it.

Current URDF values:

| Property | Value | Established |
|---|---|---|
| `camera_x` | 0.50 | tape, 2026-09-03 |
| `camera_z` | 0.55 | tape, 2026-09-03: lens 0.65 m above ground, minus `base_height` 0.10 |
| `camera_pitch` | 0.0 | mounted level; `mount_check` on 2026-09-04 read -1.25 degrees pitch, +0.52 degrees roll, small enough to leave at zero |
| `imu_link` | (0.50, 0, 0.40), rpy 0 0 0 | same place as the camera, 15 cm lower |
| `gps_link` | (0, 0, 0.25) | **not measured**, assumed over the chassis centre |

**In ROS, positive pitch is nose DOWN.** The old `camera_pitch: -0.10` claimed a
5.7 degree *upward* tilt on a level camera, rotating every depth point upward
with an error that grows with range: flat floor 2 m ahead landed at z = +0.10 m in
`base_link` instead of -0.10 m, exactly `min_obstacle_height`, so the floor was
marked lethal in a band across the view from about 2 m out. Together with
`camera_x` being 0.40 m short, nothing was drivable and Nav2 fell through to Spin
(see [../ARCHITECTURE.md](../ARCHITECTURE.md#where-the-camera-is-bolted-and-why-it-decides-everything)).
When you tilt the camera down, put the **measured** angle here as a **positive**
number; guessing brings the phantom wall back.

## Hands-on tests

Nothing but your hands: the rover need not move, and you can pick up the sensor
board instead. Use RViz with Fixed Frame `odom`.

1. **Does it sit still?** Leave everything alone and watch
   `ros2 topic echo /odometry/local --field pose.pose.position`. With the
   zero-velocity update, expect **0 cm net displacement over 150 s** (only the
   third decimal moves); the raw ungated VO beneath it still walks ~1.2 cm/min.
   If it wanders: someone moving in view, a blank untextured wall, or a saturated CPU
   dropping frames.
2. **Put an obstacle in front of it.** Hold a box ~1 m ahead. Dark lethal cells
   appear exactly there, in a lighter inflation halo (the safety margin), and the
   depth cloud sits on them; if they disagree, TF is wrong. From a terminal:
   `python3 ~/athena/indoor_test/mount_check.py costmap` (ASCII map) or
   `stack_check` (cell counts). `head -20` on the raw costmap topic prints only the
   header, not the cells.
3. **Take it away. Does the map forget?** Cells should clear within a second or
   two: the costmap *raytraces*, erasing any cell the camera sees through. If cells
   linger, check the cloud is arriving (`ros2 topic hz
   /camera/camera/depth/color/points_downsampled`, ~7.4 Hz). If it has stopped
   entirely, `expected_update_rate: 2.0` makes the costmap not-current after 2 s and
   the controller refuses to drive: correct for a blind rover, but a flaky camera
   stops the rover.
4. **Move an obstacle across the view.** Slide the box slowly: marked cells follow,
   appearing ahead and clearing behind. A live map, not a snapshot.
5. **Move the camera.** Slide the whole board sideways. Correct: obstacles stay
   fixed in the world and the robot marker moves. Wrong: obstacles slide *with* the
   robot, so odometry is not tracking.
6. **Rotate it.** Turn the board 90 degrees: the marker turns and stays turned.
   Use `ros2 topic echo /imu/data --field angular_velocity` and `ros2 run tf2_ros
   tf2_echo odom base_link`. Do **not** judge from `/imu/data --field orientation`:
   that is the Pixhawk's attitude, wrong on this FMU and unfused. The full version
   is `python3 ~/athena/indoor_test/mount_check.py yaw`.
7. **Walk a loop.** Tape-mark the board, carry it ~2 m around the desk, set it back
   on the tape. The green trail should return near the origin; start-to-end distance
   is the loop-closure error, good under 5% of distance travelled. Move slowly with
   texture in view: fast motion or a blank wall resets visual odometry, which is
   expected.
8. **GPS.** `ros2 topic echo /gps/fix --field status` (0 = fix, -1 = none) and
   `ros2 topic hz /odometry/gps` (ticks only with a fix). Indoors there is usually no
   fix; everything above still works.

## Measurements

**Is the pose drifting?** Everything still, two minutes. Reports go to
`~/athena/loc_reports/`:

```bash
ros2 run athena_gps_nav localization_monitor --ros-args -p duration:=120.0
```

| Metric | Good | If bad |
|---|---|---|
| net displacement | < 5 cm | the pose estimate drifts |
| path length | < 2 m | micro-jitter accumulates; the costmap smears |
| discontinuities | 0 | VIO resets leak into the filter |
| yaw drift | < 1 degree | gyro bias or compass wander |

**Is the costmap sliding?** Measures where obstacles sit relative to the robot.
Good: under 5 cm total and under 3 cm/min. Measured here: 0.7 cm net, 0.4 cm/min,
peak 0.8 cm.

```bash
ros2 run athena_gps_nav costmap_drift_check --ros-args -p duration:=120.0
```

**Are the transforms correctly *oriented*?** A TF tree can be connected and still
wrong (camera rotated 90 degrees, IMU upside down, optical frame never rotated),
looking perfect in RViz while the rover drives sideways relative to what it sees:

```bash
ros2 run athena_gps_nav tf_check            # -p settle:=15.0 on a slow start (default 6 s)
```

| Check | Proves |
|---|---|
| every expected edge, xyz and rpy printed | nothing missing or silently identity |
| `camera_link -> optical` is rpy (-90, 0, -90) | the optical frame follows REP-103, so the cloud is not rotated |
| gravity magnitude about 9.81 | accelerometer scaling and units are right |
| **gravity sign is +z in `base_link`** | IMU not upside down, NED converted to ENU (a mis-converted IMU still reads 9.81; only the sign shows it) |
| chassis tilt | the rover is on a slope, or the IMU is mounted rotated |
| **depth cloud centroid is +x** | the camera really looks forward, not sideways |

`tf_check` prints every translation but does not know what it *should* be: a
`camera_x` 0.40 m wrong passes every check here while ruining the costmap. Only
`mount_check.py` ([above](#camera-mount)) measures against the floor.

**No hardware at all.**

```bash
ros2 launch athena_gps_nav bench_test.launch.py
ros2 run athena_gps_nav localization_monitor --ros-args -p duration:=90.0
```

`fake_gps_imu` simulates a rover driving a rectangle with realistic noise through
the real filters and Nav2; `athena_drive`'s `motor_bridge` runs too, in `dry_run`.
No camera, Pixhawk or sky view needed, but `athena_drive` and `nav2_bringup` must
be built and installed. Against simulated ground truth with 1.2 m sigma GPS noise
the global filter tracked to 0.25 m RMS (offset removed).

More measured numbers (rates, GPS accuracy runs, obstacle counts) are in
[../ARCHITECTURE.md](../ARCHITECTURE.md#verified-results).
