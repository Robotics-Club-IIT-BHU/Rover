# Athena

Athena is a six-wheeled outdoor rover (ROS 2 Humble, NVIDIA Jetson). Give it a
GPS coordinate, or click a point, and it drives there, steering around
obstacles it sees. A Pixhawk supplies IMU and GPS, a RealSense D435i supplies
obstacles and visual odometry, and a Raspberry Pi Pico drives the six motors.

This page is **how to run it**. How it is built: [docs/ARCHITECTURE.md](src/docs/ARCHITECTURE.md).

| I want to | Read |
|---|---|
| Understand the packages, data flow, frames, design decisions | [docs/ARCHITECTURE.md](src/docs/ARCHITECTURE.md) |
| Run, tune, test or fix navigation | [athena_gps_nav/README.md](src/athena_gps_nav/README.md) |
| Wire (pin map), calibrate, flash or debug the motors | [athena_drive/README.md](src/athena_drive/README.md) |
| SSH in, get through WiFi login, use `rviz2` from a laptop | [athena_remote/README.md](src/athena_remote/README.md) |
| Every Foxglove panel, goal, waypoint and troubleshooting step | [athena_remote/docs/FOXGLOVE_PANEL.md](src/athena_remote/docs/FOXGLOVE_PANEL.md) |
| Test navigation from the Foxglove panel | [athena_remote/TESTING.md](src/athena_remote/TESTING.md) |
| Change code, build, use git | [CONTRIBUTING.md](src/CONTRIBUTING.md) |

## Contents

1. [Get a shell on the rover](#1-get-a-shell-on-the-rover)
   - [Log in to the institute WiFi](#log-in-to-the-institute-wifi)
2. [Bring the rover up](#2-bring-the-rover-up)
3. [Check it is healthy](#3-check-it-is-healthy)
4. [Connect Foxglove (the control panel)](#4-connect-foxglove-the-control-panel)
5. [Send it somewhere](#5-send-it-somewhere)
6. [Everyday commands](#6-everyday-commands)
7. [Hardware](#7-hardware)
8. [Current state](#8-current-state-hardware-last-verified-september-2026)

---

## 1. Get a shell on the rover

Everything below is typed on the Jetson, over SSH:

```bash
ssh robo@172.20.119.87
```

`172.20.119.87` is the address on 2026-10-02. It changes with DHCP: on the
Jetson, `hostname -I` prints it; from another machine on the same WiFi,
`ssh robo@ubuntu.local` usually works. Finding it by scanning, and a host alias
that also opens the Foxglove tunnel:
[athena_remote](src/athena_remote/README.md#getting-a-shell-on-the-jetson).

`~/.bashrc` on the Jetson sources ROS and the workspace and sets
`ROS_DOMAIN_ID=42`. A shell that skips it (`ssh host 'command'`, cron) is on
domain 0 and sees an empty graph:
[fix](src/athena_remote/README.md#getting-a-shell-on-the-jetson).

### Log in to the institute WiFi

The institute WiFi puts a login page in front of the internet, and the Jetson has
no screen to show it on. Log it in from your SSH session instead. The login page
blocks only the internet, so you can still SSH to the Jetson from a laptop on the
same WiFi before it is logged in.

**1. Is a login needed?**

```bash
curl -s -o /dev/null -w "%{http_code}\n" http://connectivitycheck.gstatic.com/generate_204
```

`204` means it is already online and there is nothing to do. Anything else means
the login page is intercepting it.

**2. Log in by hand (first time, or when the automatic login fails).** `w3m` is a
text web browser, already installed on the Jetson:

```bash
w3m http://connectivitycheck.gstatic.com/generate_204
```

The login page opens in the terminal. Fill it in like a normal page:

| Key | Does |
|---|---|
| `Tab` / arrow keys | move between fields and buttons |
| `Enter` on a field | start typing; `Enter` again to accept |
| `Enter` on the login button | submit |
| `q`, then `y` | quit |

Run the `curl` from step 1 again: `204` means you are through.

**3. Let it log itself in (optional).** `captive_login` reads the login form once,
then logs in with your saved username and password:

```bash
ros2 run athena_remote captive_login --ros-args -p mode:=discover   # ONE TIME: writes ~/.config/athena_remote/portal.yaml
vim ~/.config/athena_remote/portal.yaml                             # fill in your username and password (i to type, Esc then :wq to save)
ros2 run athena_remote captive_login                                # log in (does nothing if already online)
```

Your password stays in `~/.config/athena_remote/portal.yaml` on the Jetson: it is
not in the repo, the file is readable only by you, and it is never logged.

**4. Stay logged in (optional).** Run `crontab -e` and add these two lines. The
first logs in at boot; the second re-logs in every 10 minutes if the session
expired:

```
@reboot      sleep 30 && . /opt/ros/humble/setup.sh && ~/athena/install/athena_remote/lib/athena_remote/captive_login
*/10 * * * * . /opt/ros/humble/setup.sh && ~/athena/install/athena_remote/lib/athena_remote/captive_login
```

Keep the `. /opt/ros/humble/setup.sh` part: without it the cron job fails
silently. Logged in but still no internet (`apt`, `git` fail): check
[the clock](#correcting-the-clock). If the automatic login fails (JavaScript login
pages, changing tokens):
[CAPTIVE_PORTAL.md](src/athena_remote/docs/CAPTIVE_PORTAL.md).

## 2. Bring the rover up

```bash
ros2 launch athena_gps_nav bringup.launch.py
```

One command starts the entire stack, staggered so each layer has what it
depends on before it starts. Give it about 40 seconds to settle.

| Elapsed | Starts | Why it waits |
|---|---|---|
| 0 s | RealSense camera, Pixhawk bridge, robot description | nothing works without sensor data |
| 0 s | Motor bridge | not delayed: it opens the Pico serial link, then waits for a `/cmd_vel` that only arrives once Nav2 is up |
| 5 s | Visual odometry (rtabmap) | needs camera images already flowing |
| 8 s | Both EKFs and the GPS transform | need odometry and IMU to fuse |
| 12 s | Nav2: planner, controller, costmaps | needs a position estimate before it can plan |
| 15 s | Foxglove bridge and the panel's nodes | only useful once there is data to show |

### Launch options

`bringup.launch.py` takes exactly these four arguments. Anything else is
silently accepted by `ros2 launch` and then ignored, so a typo costs you a run
(`--show-args` lists the real ones).

```bash
# motors receive no commands (logged, not sent); safe with the rover on a bench
ros2 launch athena_gps_nav bringup.launch.py dry_run:=true

# no RealSense: no obstacle avoidance AND no visual odometry, so the EKFs
# dead-reckon on the Pixhawk alone
ros2 launch athena_gps_nav bringup.launch.py use_camera:=false

# no control panel: lower CPU load
ros2 launch athena_gps_nav bringup.launch.py foxglove:=false

# indoor / no-GPS mode (explained below)
ros2 launch athena_gps_nav bringup.launch.py local_only:=true
```

**Indoors, use `local_only:=true`.** It drops `ekf_global` and `navsat_transform`
and publishes a static identity `map -> odom` instead. Nav2's configuration is
untouched (`global_frame` stays `map` everywhere), because with that transform
`map` *is* `odom`, so switching back to GPS work is just dropping the flag.

The reason is not CPU. With no GPS fix, `ekf_global` has nothing to fuse, so it
dead-reckons the same visual odometry and gyro that `ekf_local` already uses:
two independent filters on identical inputs, whose estimates slowly diverge for
no reason. That divergence *is* the `map -> odom` drift, and indoors it buys
nothing. Delete the second filter and the drift cannot happen
([more](src/docs/ARCHITECTURE.md#key-design-decisions)).

### Running one layer at a time

For debugging; `bringup.launch.py` is all of these together. Full list and
arguments: [athena_gps_nav](src/athena_gps_nav/README.md#run).

| Launch file | Starts |
|---|---|
| `athena_gps_nav sensors.launch.py` | robot description, camera, Pixhawk bridge, cloud downsampler |
| `athena_gps_nav vio.launch.py` | visual odometry and its gate |
| `athena_gps_nav localization.launch.py` | the EKFs and GPS transform |
| `athena_gps_nav navigation.launch.py` | Nav2 |
| `athena_drive drive.launch.py` | the motor bridge only |
| `athena_remote foxglove.launch.py` | the Foxglove bridge and panel nodes only |
| `athena_gps_nav bench_test.launch.py` | fake sensors and the real pipeline: no hardware at all |

## 3. Check it is healthy

```bash
ros2 run athena_gps_nav stack_check
```

It counts messages for a fixed window (10 s by default), prints one block per
stage, and exits on its own: there is nothing to Ctrl-C. For a longer sample:

```bash
ros2 run athena_gps_nav stack_check --ros-args -p window:=30.0
```

The full report also has a `TF TREE` section, a `NAV2` section, and a closing
`ALL CHECKS PASS` or `n FAIL` line. The part people read first, from a healthy
indoor run:

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

That single failure is expected without a GPS fix. Failures that are **not** faults:

- `navsat gps->map` FAILs whenever there is no fix.
- Launched with `local_only:=true`, `EKF global (map)` and `navsat gps->map` both
  FAIL, because neither node is started in that mode. That is the point of the mode.
- `NAV2 / cmd_vel` reads `0.0 Hz (idle - no goal sent)` and is never a FAIL. An
  idle `/cmd_vel` is the correct state with no goal running.

How to read the rest, and the hands-on tests:
[TESTING.md](src/athena_gps_nav/docs/TESTING.md#health-check).

## 4. Connect Foxglove (the control panel)

[Foxglove](https://foxglove.dev) is the rover's front end: a satellite map with
live GPS, the obstacle costmap in 3D, the camera feed, the planned path, and drive
controls in one window. It draws on your machine, so the Jetson keeps its CPU. You
need a browser (or the desktop app) and SSH access to the rover; nothing else.

- Web app (nothing to install): **[app.foxglove.dev](https://app.foxglove.dev)**
  (it will ask you to sign in)
- Desktop app: **[foxglove.dev/download](https://foxglove.dev/download)**, works
  identically
- Foxglove's own docs: [docs.foxglove.dev](https://docs.foxglove.dev)

```
Your machine                           Jetson
Foxglove --ws://localhost:8765-- ssh -L 8765 --> foxglove_bridge (127.0.0.1:8765)
```

**Use a current Foxglove.** The bridge speaks only the newer `foxglove.sdk.v1`
protocol; older builds are refused at the handshake with a bare HTTP 400 and no
message in the UI.

### Step 1: on the rover

`bringup.launch.py` already starts the bridge (the 15 s step above). Run this
yourself only when bringup is **not** running; a second copy gives duplicate nodes
and a bridge that cannot bind port 8765:

```bash
ros2 launch athena_remote foxglove.launch.py
```

It starts `foxglove_bridge` on port **8765**, bound to `127.0.0.1` only (nothing is
exposed to the network, which is why step 2 is not optional), plus six nodes the
panel depends on:

| Node | Used for |
|---|---|
| `goal_manager` | turning a typed or clicked lat/lon into a Nav2 goal |
| `waypoint_manager` | marking places and driving back to them |
| `trajectory` | the line showing where the rover has actually been |
| `teleop_mux` | manual driving with a speed limit you can change live |
| `panel_camera` | a small JPEG copy of the camera for the panel (5 Hz, about 30 KB a frame) |
| `panel_cloud` | a thinned copy of the depth cloud for the 3D view (2 Hz, about 1,500 points) |

The last two exist because the raw camera image (13.8 MB/s) and the costmap's
point cloud (0.9 MB/s) are far more than campus WiFi carries through the tunnel.
They do nothing while no panel is open, and the costmap and visual odometry still
use the full-quality data.

### Step 2: on your machine, open the tunnel

Leave this running in its own terminal:

```bash
ssh -N -L 8765:localhost:8765 robo@172.20.119.87      # or just: ssh athena  (alias, see section 1)
```

If it fails with `bind: Permission denied`, something else holds port 8765 on your
machine (common on Windows, where Hyper-V reserves port ranges). Use another local
port, and the same one in Foxglove:

```bash
ssh -N -L 18765:localhost:8765 robo@172.20.119.87     # then connect to ws://localhost:18765
```

### Step 3: connect

1. Open [app.foxglove.dev](https://app.foxglove.dev) (or the desktop app) and sign in.
2. **Open connection -> Foxglove WebSocket**.
3. URL: `ws://localhost:8765` -> **Open**.

Use `localhost`, not the Jetson's address: the web app is served over HTTPS, and
browsers block a plain `ws://` connection from an HTTPS page except to `localhost`.
`ws://172.20.119.87:8765` would be blocked even if it were reachable.

### Step 4: load the layout

The layout is the saved arrangement of panels, already pointed at the right
topics. Copy it from the rover (the import dialog reads from *your* machine):

```bash
scp robo@172.20.119.87:/home/robo/athena/src/athena_remote/config/athena_foxglove_layout.json .
```

Then in Foxglove: **Layout menu (top right) -> Import from file** ->
`athena_foxglove_layout.json`. That is the whole panel.

```
+-------------------------------+-----------------+-------------+
|                               |  SATELLITE MAP  |   CAMERA    |
|      3D VIEW                  |                 |             |
|                               +-----------------+-------------+
|                               |        |  CONTROLS  | READOUTS |
|                               | TELEOP |  (4 tabs)  | (8 tabs) |
+-------------------------------+--------+------------+----------+
```

| Panel | Shows |
|---|---|
| 3D | rover model, obstacle costmap, depth cloud, **orange** planned path, **cyan** driven trail, yellow waypoint pins |
| Satellite map | GPS position on satellite imagery, the trail, waypoint pins |
| Camera | the colour camera, as JPEG at 5 Hz |
| Teleop | arrow buttons to drive by hand |
| Controls | tabs *Mark*, *Return*, *Speed*, *Go to* |
| Readouts | tabs *Goal* (with the *Nav status* line: why the last goal ended), *Speed*, *Teleop*, *GPS fix*, *Saved places*, *Log*, *Goal pose*, *Nav feedback* |

What every marker in the 3D view means, and the Controls tabs one by one:
[FOXGLOVE_PANEL.md](src/athena_remote/docs/FOXGLOVE_PANEL.md#what-each-panel-shows).

### Adding or rebuilding panels by hand

If you would rather build a panel yourself, or one went missing, add it from
Foxglove's **Add panel** menu and give it these settings:

| Panel type | Settings |
|---|---|
| **3D** | Follow frame `odom`. Publish tool: pose topic `/athena/goal_click` (never `/goal_pose`). Topics on: `/local_costmap/costmap`, `/athena/points_preview` (thinned depth cloud), `/plan`, `/athena/trajectory`, `/athena/waypoints`, `/athena/goal_marker`, `/odometry/local`, `/local_costmap/published_footprint`. Rover model: panel settings -> **Custom layers -> Add URDF**, source **topic**, `/robot_description` |
| **Map** | Custom tile layer, URL `https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}` (satellite), follow topic `/gps/fix`; also show `/athena/trajectory_geojson` and `/athena/waypoints_geojson` |
| **Image** | Image topic `/athena/camera/compressed`, calibration `/camera/camera/color/camera_info` |
| **Teleop** | Topic `/athena/teleop/cmd_vel`, publish rate 15; up = `linear-x` +1, down = `linear-x` -1, left = `angular-z` +1, right = `angular-z` -1 |
| **Publish** (one per button) | Topic and type, then the JSON value: `/athena/goal_text` `std_msgs/msg/String` `{"data": "25.2629548, 82.9838284"}` (Go to); `/athena/waypoint_save` and `/athena/waypoint_goto` `std_msgs/msg/String` (Mark, Return); `/athena/teleop/max_speed` `std_msgs/msg/Float32` `{"data": 0.35}` (Speed) |
| **Raw Messages** | Any topic, for readouts, e.g. `/gps/fix.status.status` for the fix state |
| **Log** | rosout, for the Log readout |

Panel names and menu wording shift a little between Foxglove versions; the topics
above are what matters.

### If it will not connect

| Symptom | Cause |
|---|---|
| Refused instantly, HTTP 400, nothing in the bridge log | Foxglove too old (see above). Update it |
| `ConnectionRefusedError` / nothing on 8765 | bridge not running: bringup has not reached 15 s, or it was launched with `foxglove:=false` |
| Connects, panels blank | wrong layout or topics: re-import the layout; check the topic names above |
| Map blank, model missing, camera blank, teleop does nothing, no path lines | [troubleshooting table](src/athena_remote/docs/FOXGLOVE_PANEL.md#troubleshooting) |
| Still laggy on a slow link | send less: lower the defaults of `camera_rate`, `camera_width` (e.g. 424) and `cloud_rate` in `athena_remote/launch/foxglove.launch.py`, then relaunch. Raw everything, on a fast wired LAN only: `topic_whitelist:="['.*']"` |
| Camera panel blank | the Image panel must use `/athena/camera/compressed`: re-import the layout (copy it over again first, it changed on 2026-10-02) |

Test the bridge itself from the Jetson, and the full table:
[FOXGLOVE_PANEL.md](src/athena_remote/docs/FOXGLOVE_PANEL.md#setup).

## 5. Send it somewhere

Three ways, once the panel is open:

- **Click in the 3D view.** Toolbar **Publish** tool, **Pose**; click the
  destination and drag to set heading. Works with or without a GPS fix: use this
  indoors. The layout publishes clicks on `/athena/goal_click`, where `goal_manager`
  converts them into the `map` frame once and sends the goal. Do **not** point the
  tool at `/goal_pose` directly: Foxglove stamps a click in the panel's display
  frame (`odom` or `base_link`), Nav2 Humble re-reads it at that stamp on every
  replan, and TF only keeps 10 s, so any goal not reached within about 10 s fails.
  Until 2026-10-02 that was the cause of nearly every "rejected" clicked goal.
- **Type a coordinate** in the *Go to* tab: `{ "data": "25.2629548, 82.9838284" }`.
  An optional third number is the final compass heading (0 = north, 90 = east);
  omit it and the rover finishes facing the way it travelled. Needs a GPS fix.
- **Drive by hand** with the teleop arrows. This cancels any running Nav2 goal,
  so autonomy and the operator never fight over the wheels.

**Why did a goal fail?** Read the *Nav status* line at the top of the *Goal*
readout tab (the same line appears in the bringup terminal from `nav_status`). It
names the reason, for example `FAILED ...: blocked: lethal costmap cells in the
rover's path and no detour found`, `goal is outside the planner's 100 x 100 m
window`, or `rover did not move 0.3 m in 30 s`, and says what to do. Every reason
it can give: [FOXGLOVE_PANEL.md](src/athena_remote/docs/FOXGLOVE_PANEL.md).

**Cancel a goal:** send `{ "data": "cancel" }` on the *Go to* topic, or nudge a
teleop arrow. The teleop STOP button does **not** cancel a goal.

**Mark a place and come back.** Drive there, open the *Mark* tab, name it; the
*Return* tab drives back. Waypoints are saved in
`~/.config/athena_remote/waypoints.yaml` on the Jetson and survive reboots.
GPS-tagged waypoints (a fix was available when saved) stay valid forever;
map-only ones last until the next restart, because the map origin moves when the
datum is re-locked.

**Record and follow a GPS route** from a terminal, no panel:
[athena_gps_nav](src/athena_gps_nav/README.md#send-it-somewhere). Not ready for
unattended use: absolute heading is not fused
([why](src/athena_gps_nav/docs/TUNING.md#heading-the-compass-is-not-used)).

The same actions as `ros2 topic pub`, and every topic:
[TOPICS.md](src/athena_remote/docs/TOPICS.md).

## 6. Everyday commands

| I want to | Do this |
|---|---|
| Start everything | `ros2 launch athena_gps_nav bringup.launch.py` (section 2) |
| Check it works | `ros2 run athena_gps_nav stack_check` (section 3) |
| Open the panel | `ssh -N -L 8765:localhost:8765 robo@172.20.119.87`, then Foxglove -> `ws://localhost:8765` (section 4) |
| Cancel a goal from a terminal | `ros2 topic pub --once /athena/goal_text std_msgs/String '{data: "cancel"}'` |
| See what the controller asks for | `ros2 topic echo /cmd_vel_nav` (`/cmd_vel` has six publishers) |
| Open RViz | `rviz2 -d ~/athena/src/athena_gps_nav/config/athena_bench.rviz`: [setup](src/athena_gps_nav/docs/TESTING.md#rviz) |
| Test or calibrate the motors | `ros2 run athena_drive calibrate` (a menu). **Stop the stack first** (the Pico's port is exclusive); wheels off the ground: [guide](src/athena_drive/README.md#calibration) |
| Check how the sensors are mounted | `python3 ~/athena/indoor_test/mount_check.py camera` (also `costmap`, `yaw`, `watch`): [guide](src/athena_gps_nav/docs/TESTING.md#camera-mount) |
| Find out why GPS has no fix | stop bringup, then `ros2 run athena_gps_nav gps_diagnose`: [guide](src/athena_gps_nav/docs/TROUBLESHOOTING.md#gps-has-no-fix) |
| Run ROS commands over SSH with no panel | [athena_remote, Option B](src/athena_remote/README.md#which-way-in) |
| Use a native `rviz2` / `ros2` on a laptop | [athena_remote, Option C](src/athena_remote/README.md#which-way-in) |
| Get the Jetson through campus WiFi login | `w3m http://connectivitycheck.gstatic.com/generate_204`, or [captive_login](src/athena_remote/docs/CAPTIVE_PORTAL.md) |

### Correcting the clock

A Jetson that boots offline starts with a wrong clock: TF lookups report
extrapolation into the future, and certificate validation fails, so `apt` and
`git` stop working.

```bash
timedatectl                                                          # want: System clock synchronized: yes
sudo systemctl restart systemd-timesyncd && sudo timedatectl set-ntp true     # once a network is available
```

With no network, set it by hand: `sudo timedatectl set-ntp false`, then
`sudo timedatectl set-time "YYYY-MM-DD HH:MM:SS"`, then `sudo timedatectl set-ntp true`.
Restart the stack after any manual clock change: nodes cache timestamps.

## 7. Hardware

| Device | Connection | Used by |
|---|---|---|
| RealSense D435i | USB 3 | camera driver |
| Pixhawk PX4 FMU v2 | USB serial, MAVLink **921600** baud (`/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v2.x_0-if00`) | `pixhawk_bridge` |
| Raspberry Pi Pico W | USB serial, 115200 baud (`/dev/serial/by-id/usb-Raspberry_Pi_Pico*`) | `motor_bridge`; firmware in [athena_drive](src/athena_drive/docs/FIRMWARE.md) |

- **Motor wiring and the Pico pin map** (which GPIO goes to which driver channel,
  with a wiring diagram): [athena_drive/README.md](src/athena_drive/README.md#pico-to-motor-driver-pin-map).
- Always address serial devices by `/dev/serial/by-id/...`. The `ttyACM*` numbers
  swap between boots, and the Pixhawk and the Pico have each held `ttyACM0`.
  **Never launch the old `drive` package**: it hardcodes `/dev/ttyACM1`, which can
  be the Pixhawk ([drive/README.md](src/drive/README.md)).
- The Pico's serial port has one owner. `motor_bridge`, `calibrate`, `wiring_check`
  and `calibrate_speed` all open it, and bringup already starts a bridge. Check
  with `fuser -v /dev/ttyACM*`.
- Prop the rover up (wheels off the ground) for every motor test, except
  `calibrate` stage 4 and `calibrate_speed -p mode:=topspeed`, which drive on the floor.

## 8. Current state (hardware last verified September 2026)

- **Working, verified on hardware:** IMU, camera, visual odometry, both EKFs,
  obstacle costmaps, Nav2 planning and execution of goals.
- **GPS has no satellite fix.** The module talks to the Pixhawk cleanly but reports
  zero satellites indoors and outdoors; the software path is ruled out and the
  antenna / RF path is suspected. `/gps/fix` still carries a lat/lon with
  `status: -1`, which is not a fix. Everything except GPS-referenced navigation
  works: use `local_only:=true` and click goals. Diagnosis:
  [TROUBLESHOOTING.md](src/athena_gps_nav/docs/TROUBLESHOOTING.md#gps-has-no-fix).
- **GPS waypoint following is not ready:** it also needs an absolute heading source
  (absolute yaw is not fused), see
  [TUNING.md](src/athena_gps_nav/docs/TUNING.md#heading-the-compass-is-not-used).
- **Motor calibration is half done.** `~/.config/athena_drive/params.yaml` holds
  `min_pwm: 20` and `track_width: 0.85` (measured), but not `max_wheel_speed` or
  `max_pwm`, so `motor_bridge` falls back to 0.7 m/s and 255 and logs
  `UNCALIBRATED ... max_wheel_speed, max_pwm` at startup. A `max_wheel_speed` of
  exactly `0.7` is the fallback, not a measurement. Run stage 4 on the floor before
  trusting a commanded distance:
  `ros2 run athena_drive calibrate --ros-args -p stages:=4`.
- **The Pico has not been reflashed with the `G` command**, so `calibrate` runs
  multi-motor tests one motor at a time
  ([FIRMWARE.md](src/athena_drive/docs/FIRMWARE.md#flashing)).
- **Recovery is deliberately reduced:** no Spin or BackUp, so a goal Nav2 cannot
  rescue fails cleanly and hands control back to the operator
  ([why](src/athena_gps_nav/docs/TUNING.md#recovery-behaviour)).

## Contributors

- **Jashan**: GPS navigation stack, drive package, remote access and control panel

Add your name here when you contribute. See [CONTRIBUTING.md](src/CONTRIBUTING.md).
