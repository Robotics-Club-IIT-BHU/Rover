# Contributing to Athena

How to change code in the Athena workspace (ROS 2 Humble on a Jetson): where it goes, how to build
it, what not to break, and the git workflow. To run the rover see [README.md](README.md). For how it
works see [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

Read [section 1](#1-the-five-packages), [section 3](#3-rules-that-protect-the-rover) and [section
4](#4-building-and-the-traps) before your first change. The rest is reference.

## 1. The five packages

| Package | Owns | Live? |
|---|---|---|
| [`athena_gps_nav`](athena_gps_nav/README.md) | Sensors, localization, obstacle maps, Nav2 planning. Output is `/cmd_vel` | yes |
| [`athena_drive`](athena_drive/README.md) | Everything that ends in a motor turning: `/cmd_vel` to PWM over serial, calibration, Pico firmware | yes |
| [`athena_remote`](athena_remote/README.md) | Everything an operator sees or presses, and remote access | yes |
| `athena_slam` | First-generation SLAM/GPS experiments | no |
| [`drive`](drive/README.md) | First-generation motor bridge and teleop | no |

### Where new code goes

The split is by purpose, not technology. Ask what the node's data is *for*.

- **`athena_gps_nav`**: "where am I, what is around me, where do I go". Reads a sensor, estimates a
  pose, keeps a costmap, plans a path, even if the sensor is bolted to the drive system.
- **`athena_drive`**: "how do the wheels do that". `/cmd_vel` in, PWM out. Knows track width,
  deadbands and the Pico; knows nothing about goals, maps or GPS.
- **`athena_remote`**: "how does a person see and steer this". Goal entry, waypoints, travelled
  trail, teleop limits, Foxglove bridge, WiFi portal login, even if the node subscribes to
  navigation topics (`waypoint_manager`, `trajectory`). The rover navigates fine with all of it
  stopped.

### The two legacy packages

`athena_slam` and `drive` are superseded: `bringup.launch.py` starts nothing from them and no live
node imports from them. They stay so older branches and notes resolve. Do not extend or delete them;
if you are editing one you are probably in the wrong package. Six of `athena_slam/setup.py`'s
console scripts point at modules that do not exist (e.g. `linearSim`, `gpsplot`); leave it.

## 2. Where a change goes

File-by-file descriptions:
[athena_gps_nav](athena_gps_nav/ARCHITECTURE.md#every-file-in-this-package),
[athena_drive](athena_drive/README.md#nodes),
[athena_remote](athena_remote/README.md#what-actually-runs). Before tuning Nav2 or the EKFs, read
[TUNING.md](athena_gps_nav/docs/TUNING.md).

| I want to... | Open |
|---|---|
| Change autonomous speed | `athena_gps_nav/config/nav2_params.yaml`: `FollowPath` `desired_linear_vel`, and `rotate_to_heading_angular_vel` for turning in place. This is what Nav2 *requests*. |
| Change how a request turns the wheels | `~/.config/athena_drive/params.yaml`, written by `ros2 run athena_drive calibrate` (stage 4 measures `max_wheel_speed`). Without a saved `max_wheel_speed`, `motor_bridge` assumes 0.7 m/s and logs `UNCALIBRATED`. |
| Change the teleop speed limit | `athena_remote/athena_remote/teleop_mux.py` defaults, or `max_speed` / `max_turn` on `foxglove.launch.py`, or at runtime a `std_msgs/Float32` on `/athena/teleop/max_speed`. |
| Change acceleration smoothness | `athena_drive/athena_drive/velocity_profile.py`; `max_accel` / `max_jerk` in `drive.launch.py`. |
| Change a costmap layer, or obstacle range | `nav2_params.yaml`: `local_costmap` / `global_costmap`, and the `realsense` source (`obstacle_max_range`, `raytrace_max_range`). Also `pointcloud_downsampler`'s `max_distance`: a point must pass the filter before the costmap can mark it. |
| Change where the camera or GPS is mounted | `athena_gps_nav/urdf/athena.urdf.xacro`: `camera_x` / `camera_z` / `camera_pitch` at the top. **Measure, do not eyeball**: `python3 ~/athena/indoor_test/mount_check.py camera` fits the floor plane without reading the URDF ([guide](athena_gps_nav/docs/TESTING.md#camera-mount)). Then `ros2 run athena_gps_nav tf_check`. |
| Change the rover's size | The URDF (`base_length`, `base_width`) and `footprint:` in **both** costmap sections of `nav2_params.yaml`. Keep all three equal. |
| Change what Nav2 does when a goal keeps failing | `athena_gps_nav/behavior_trees/athena_nav_to_pose.xml`, wired in by `default_nav_to_pose_bt_xml` in `nav2_params.yaml`. Read its top comment first. |
| Change EKF fusion, or how hard GPS pulls the map | `athena_gps_nav/config/dual_ekf_navsat.yaml`. |
| Change startup order or timing | `athena_gps_nav/launch/bringup.launch.py`, the `TimerAction` periods ([current order](docs/ARCHITECTURE.md#launch-order)). |
| Add or change a Foxglove panel | `athena_remote/config/athena_foxglove_layout.json`, re-imported in Foxglove (no node reads it). A node the panel needs goes in `athena_remote`, started from `launch/foxglove.launch.py`. |
| Change the serial protocol to the Pico | Both sides together: `athena_drive/firmware/athena_drive_fw/athena_drive_fw.ino` and `athena_drive/athena_drive/motor_bridge.py` ([FIRMWARE.md](athena_drive/docs/FIRMWARE.md)). |
| Fix a motor spinning the wrong way | Not code: `ros2 run athena_drive calibrate --ros-args -p stages:=1`. The invert flags live in the Pico's flash. |
| Run indoors, or with no GPS fix | Nothing to edit: `ros2 launch athena_gps_nav bringup.launch.py local_only:=true`. |

**Adding a sensor** touches five places: its mount in the URDF; its driver in
`launch/sensors.launch.py`; `dual_ekf_navsat.yaml` if it feeds localization; an observation source
in `nav2_params.yaml` if it feeds obstacles; a line in `stack_check.py` so it is health-checked.

`~/athena/indoor_test/` (outside `src/`, not a ROS package) holds loose diagnostic scripts;
`mount_check.py` is the one that matters. Run them with `python3`.

## 3. Rules that protect the rover

These changes can damage hardware, not just software.

**Motors and serial**

- Never launch the old `drive` package. Address serial devices by `/dev/serial/by-id/...`, never
  `ttyACM<n>`. The numbers swap between boots, and the Pixhawk and the Pico have each been
  `ttyACM0`.
- The Pico's serial port has one owner. Bringup already runs a `motor_bridge`, so stop the stack
  before `calibrate`, `wiring_check` or `calibrate_speed`. The loser fails quietly
  (`could not open ... running dry`) and `/cmd_vel` does nothing. See who holds it with
  `fuser -v /dev/ttyACM*`.
- Prop the rover up for every motor test. Only `calibrate` stage 4 and
  `calibrate_speed -p mode:=topspeed` drive on the floor.
- Change the Pico protocol in firmware and `motor_bridge.py` together. Keep both watchdogs (bridge
  0.5 s, firmware 500 ms) and both limiters (host jerk-limited profile, firmware slew limit): each
  covers a failure the other cannot.

**Navigation**

- Keep the no-fix guard in `goal_manager.py`. With no GPS fix `/fromLL` returns map `(0, 0)` for
  every coordinate instead of failing, so Nav2 would accept a goal at the origin and report success.
- Do not add Spin or BackUp back to the behavior tree: the rover has an 87 degree forward camera and
  no rear sensor, and both move a 1.3 m rover through space it cannot see. Keep
  `expected_update_rate: 2.0` on both costmap observation sources too: a camera that stops sending
  must stop the rover, not leave stale obstacles.
- Keep `use_velocity_scaled_lookahead_dist: false` (fixed 1.0 m) and `odom_topic: /odometry/local`
  in `bt_navigator`, `controller_server` and `velocity_smoother`. Speed is zero when a goal starts,
  so a scaled lookahead is 0.4 m, inside the footprint (it reaches x = +0.65): the controller turns
  in place and never drives. A missing `odom_topic` does the same.

**Running it**

- One stack per `ROS_DOMAIN_ID` (42). A second `bringup.launch.py`, or `bench_test.launch.py` beside
  it, starts a second pair of EKFs and TF flips between them.
  `ros2 node list | sort | uniq -c | awk '$1>1'` must print nothing.
- Indoors or with no GPS, use `local_only:=true`. Otherwise `ekf_global` dead-reckons the same
  inputs as `ekf_local` and the two drift apart.
- A misspelled launch argument is silently ignored, so a typo in `dry_run` leaves the wheels live.
  Check with `ros2 launch <pkg> <file> --show-args`.
- Never `source ~/rtab/install/setup.bash` (ABI-broken, crashes instantly; rtabmap comes from apt).

## 4. Building, and the traps

```bash
cd ~/athena
colcon build --symlink-install                                   # everything
colcon build --packages-select athena_drive --symlink-install    # one package
source install/setup.bash
```

**Trap 1: build from `~/athena`, never from `src/` or a package directory.** `colcon` finds packages
by walking down from the current directory, so inside a package it sees only that one. Building
another package there prints a warning (`ignoring unknown package`), **exits 0 and builds nothing**,
and you run code without your changes. It also writes `build/`, `install/` and `log/` into the wrong
place: any of those inside `src/` or a package is stray output from this mistake (gitignored, safe
to delete). If `ros2 run` runs code you deleted, suspect a stray `install/` on `AMENT_PREFIX_PATH`.

**Trap 2: `--symlink-install` is not optional.** Without it `colcon` copies Python sources into
`install/`, and every later edit in `src/` is silently ignored until a rebuild with the flag. With
it, editing a `.py` file and restarting the node is enough. To see which you have,
`ls -l ~/athena/install/athena_gps_nav/lib/python3.10/site-packages/`: a single `.egg-link` is a
symlink install, a directory of `.py` files is a copy.

**Trap 3: data files are linked at build time.** `setup.py` globs them when you build and links each
one `install/` -> `build/` -> `src/` (check the whole chain with `readlink -f`).

| What you did | Rebuild? |
|---|---|
| Edited a `.py` file | No. Restart the node. |
| Edited an existing YAML, launch, URDF, behavior-tree, JSON or script file | No. Relaunch. |
| Added, renamed or deleted a data file | **Yes.** An added file is invisible until rebuilt; a deleted one leaves a dangling symlink. |
| Added a node, or changed `setup.py` or `package.xml` | **Yes.** |

The globs are not "everything in the directory". A file with another extension or in another
directory is silently absent at runtime, and the launch file that wants it fails on a path that
looks right in `src/`. Installed by each `setup.py`:

- `athena_gps_nav`: `launch/*.py`, `config/*.yaml`, `config/*.rviz`, `urdf/*.xacro`,
  `behavior_trees/*.xml`
- `athena_drive`: `launch/*.py`, `firmware/athena_drive_fw/*.ino`
- `athena_remote`: `launch/*.py`, `config/*.xml`, `config/*.json`, `scripts/*.sh`

**Adding a node:** write it in the package whose job it is (section 1); add
`'my_node = athena_remote.my_node:main'` to `console_scripts` in that package's `setup.py`; declare
any new dependency or message package in `package.xml`; then rebuild (`setup.py` changed) and
`ros2 run athena_remote my_node`.

## 5. Testing before you push

The live packages have no unit tests (no `test/` directory), so the checks are these. Docs-only
changes need none of them.

```bash
python3 -m py_compile <file>
python3 -m flake8 --select=F,E9 <file>      # undefined names, unused imports
cd ~/athena
colcon build --packages-select athena_gps_nav athena_drive athena_remote --symlink-install
source install/setup.bash
```

If the change affects how the rover behaves, also bring the stack up with `dry_run:=true` (motor
commands are logged, not sent, so the wheels stay still) and run the health checks:

```bash
ros2 launch athena_gps_nav bringup.launch.py dry_run:=true     # indoors: add local_only:=true
# second shell, after about 40 s:
ros2 run athena_gps_nav stack_check
ros2 run athena_gps_nav tf_check
```

- Expected FAILs, not yours: `navsat gps->map` whenever there is no GPS fix, and under
  `local_only:=true` also `EKF global (map)`. Say which you saw.
- No hardware at all: `ros2 launch athena_gps_nav bench_test.launch.py` runs fake sensors through
  the real EKFs, Nav2 and a dry-run `motor_bridge`.
- Anything that moves wheels: follow section 3. Per-package test guides:
  [gps_nav](athena_gps_nav/docs/TESTING.md), [drive](athena_drive/docs/TESTING.md),
  [remote](athena_remote/TESTING.md).

Say in the pull request which checks you ran and what they reported.

## 6. Working on the Jetson with your own GitHub account

The Jetson is shared (everyone logs in as `robo`), so do not put your personal GitHub key on it. SSH
agent forwarding keeps your private key on your machine; only signatures cross the connection. Once,
on **your own machine**:

```bash
ssh-keygen -t ed25519 -C "you@example.com"   # skip if you already have a key
cat ~/.ssh/id_ed25519.pub                    # paste at https://github.com/settings/keys
ssh-add ~/.ssh/id_ed25519                    # macOS: add --apple-use-keychain
ssh-add -l                                   # must list a key, not an empty agent
```

Add to `~/.ssh/config` on your own machine:

```
Host athena
    HostName <jetson-ip>
    User robo
    ForwardAgent yes
    LocalForward 8765 localhost:8765
```

`LocalForward` also opens the Foxglove tunnel. The Jetson's address changes with DHCP: run
`hostname -I` on it, or see [athena_remote](athena_remote/README.md#getting-a-shell-on-the-jetson).

Check that your identity arrived, from inside `ssh athena`: `ssh -T git@github.com` should say
`Hi <your-username>!`. A password prompt, `Permission denied (publickey)` or someone else's name
means the agent was not forwarded: run `ssh-add -l` locally and reconnect.

Set your commit identity **per repository** (`git config user.name "Your Name"`,
`git config user.email "you@example.com"` inside your checkout), never with `git config --global`.
That writes `/home/robo/.gitconfig`, which every user shares and which already holds one person's
name and email, so commits without a per-repo identity are attributed to them.

While you are connected, anyone with root on the Jetson can use your forwarded agent as you. Do not
forward it to machines you do not trust, and close sessions you are done with.

## 7. Branches, commits, pull requests

**State of this checkout:** `~/athena/src` is not currently a git repository: no `.git` in `src/` or
`~/athena`, and no remote recorded anywhere in the tree. `.gitignore` sits in `src/`, so `src/` is
the intended root. Ask a maintainer where the repository lives before you commit or push. The
commands below assume a checkout of it with remote `origin` and default branch `main`.

`main` should always be in a state the rover can run.

```bash
git checkout main && git pull
git checkout -b <name>/<subject>           # e.g. arya/fix-costmap-decay
git push -u origin <name>/<subject>        # then open the PR from the link it prints
```

- **Commit subject:** imperative ("add", "fix", "remove"), under about 70 characters. Then a blank
  line, then **why**: the diff records what changed, only the message records the reasoning that
  stops the next person reverting it. A typo fix can be one line.
- **Pull request:** open it in a browser. `gh` is not installed on the Jetson, and `gh auth login`
  would store your token under the shared `robo` account, which section 6 exists to avoid. Say what
  problem and symptom, how you solved it and what you rejected, and what shows it works (commands
  and output, section 5).
- `git push` needs internet. On campus WiFi, clear the captive portal first:
  [CAPTIVE_PORTAL.md](athena_remote/docs/CAPTIVE_PORTAL.md).
- Request one review. **Say prominently if the change affects how the rover moves** (drive package,
  Nav2 tuning, velocity profile).

## 8. Conventions

- **Comments explain why, not what.** If a line looks strange, say what breaks without it. Keep the
  comments that document real faults; delete ones that only restate the next line.
- **Every node has a docstring**: purpose, subscriptions and publications, how to run it, and the
  reasoning behind anything non-obvious. Models: `athena_remote/athena_remote/goal_manager.py`,
  `athena_drive/athena_drive/motor_bridge.py`.
- **Flat and explicit beats clever.** No base classes to unify two nodes that merely resemble each
  other.
- **Named constants** at module level with the unit in a comment, only where the meaning is clear.
- **Measured values go in a README or comment, with the measurement** and date.
- **XML comments (URDF, behavior tree) must not contain a double hyphen.** `xacro` fails with
  "invalid token" pointing at the comment, not the cause.

## 9. What not to commit

`.gitignore` covers `build/`, `install/`, `log/`, Python caches, bags and dumps (`*.bag`, `*.db3`,
`*.mcap`, `rosbag2_*/`, `loc_reports*/`), `waypoints.yaml`, `portal.yaml` and editor files.

These live outside the repository on purpose; never commit them:

| File | Holds |
|---|---|
| `~/.config/athena_remote/portal.yaml` | WiFi captive-portal credentials |
| `~/.config/athena_remote/waypoints.yaml`, `~/athena_waypoints.yaml` | Saved and recorded waypoints (the second is `gps_waypoint_logger`'s default output) |
| `~/.config/athena_drive/params.yaml` | Motor calibration: it describes *this* physical rover |
| The Pico's flash | Motor directions |

## Adding yourself

When your first pull request is merged, add a line to the contributors list in `README.md`, and in
the README of any package you worked on substantially: `- **Your Name**: what you worked on`.

## Contributors

- **Jashan**: GPS navigation stack, drive package, remote access and control panel
