# Foxglove control panel

A browser front end for the rover: satellite map with live GPS, costmap and
depth cloud in 3D, camera feed, planned path, goals, waypoints, teleop. It
renders on your machine, so the Jetson keeps its CPU. One TCP port, works
anywhere SSH works, nothing to install but a browser. Back to the
[package README](../README.md).

```
Your machine                            Jetson
app.foxglove.dev  --ws://localhost:8765--  ssh -L 8765  -->  foxglove_bridge (127.0.0.1:8765)
```

## Setup

**1. On the Jetson.** `bringup.launch.py` already starts the bridge. Run this
only when bringup is *not* running (a second copy gives duplicate nodes and a
bridge that cannot bind 8765):

```bash
ros2 launch athena_remote foxglove.launch.py
```

It starts `foxglove_bridge` (port 8765, loopback only) and six nodes:

| Node | Used for |
|---|---|
| `goal_manager` | turning a typed lat/lon, or a pose clicked in 3D, into a Nav2 goal in `map` |
| `waypoint_manager` | marking places and driving back to them |
| `trajectory` | the line showing where the rover has actually been |
| `teleop_mux` | manual driving with a speed limit you can change live |
| `nav_status` | one line saying why the last Nav2 goal ended, for every goal whoever sent it |
| `panel_camera` | the colour camera as throttled JPEG, so video fits through the tunnel |
| `panel_cloud` | a voxel-thinned depth cloud for the 3D view, for the same reason |

`operator_nodes:=false` starts the bridge alone. All launch arguments are in
the [README](../README.md#foxglove-launch-arguments).

**2. On your machine**, open the tunnel and leave it running:

```bash
ssh -N -L 8765:localhost:8765 robo@172.20.119.87       # or just: ssh athena
```

`bind: Permission denied` means something else holds the local port (common
on Windows, where Hyper-V and WSL reserve ranges). Pick another local port and
use it in the browser too: `ssh -N -L 18765:localhost:8765 ...` then
`ws://localhost:18765`.

**3. In a browser**, open [app.foxglove.dev](https://app.foxglove.dev) (the
desktop app works the same):

- **Open connection -> Foxglove WebSocket -> `ws://localhost:8765` -> Open**
- **Layout menu (top right) -> Import from file** -> `athena_foxglove_layout.json`.
  The file is on the Jetson and the dialog reads from your machine, so copy it first:

```bash
scp robo@172.20.119.87:/home/robo/athena/src/athena_remote/config/athena_foxglove_layout.json .
```

The tunnel must end at `localhost`: the web app is HTTPS, and browsers block
`ws://` from an HTTPS page except to `localhost`. Connecting straight to
`ws://172.20.119.87:8765` is blocked as mixed content.

**Use a current Foxglove.** The bridge is `foxglove_bridge` 3.4.3, which speaks
only the `foxglove.sdk.v1` subprotocol. Older Studio builds offering just
`foxglove.websocket.v1` are refused at the handshake with a bare HTTP 400 and
nothing in the UI or the bridge log. To test the bridge from the Jetson:

```bash
python3 -c "import socket,base64,os; s=socket.create_connection(('127.0.0.1',8765),timeout=5); k=base64.b64encode(os.urandom(16)).decode(); s.sendall(('GET / HTTP/1.1\r\nHost: 127.0.0.1:8765\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: '+k+'\r\nSec-WebSocket-Version: 13\r\nSec-WebSocket-Protocol: foxglove.sdk.v1\r\n\r\n').encode()); print(s.recv(120).decode(errors='replace').splitlines()[0])"
```

`HTTP/1.1 101 Switching Protocols` = bridge is fine, the client is the
problem. `ConnectionRefusedError` = nothing listens on 8765: bringup has not
reached 15 s yet, or it was launched with `foxglove:=false`.

## What each panel shows

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
| **3D** | rover model, obstacle costmap, depth cloud, planned path, travelled trail, saved waypoints |
| **Satellite map** | GPS position on Google imagery, travelled track, waypoint pins |
| **Camera** | `/athena/camera/compressed`: JPEG copy of the colour camera, 5 Hz by default |
| **Teleop** | arrow buttons: direction only, speed applied on the rover |
| **Controls** | *Mark*, *Return*, *Speed*, *Go to* |
| **Readouts** | *Speed*, *Goal*, *Teleop*, *GPS fix*, *Saved places*, *Log*, *Goal pose*, *Nav feedback* |

Each Controls tab is one Publish panel with one button: *Mark* ->
`/athena/waypoint_save`, *Return* -> `/athena/waypoint_goto`, *Speed* ->
`/athena/teleop/max_speed`, *Go to* -> `/athena/goal_text`. Cancel, delete,
clear-trail and turn rate have no button: use the command line
([TOPICS.md](TOPICS.md)).

In the 3D view: dark cells = lethal obstacles, lighter halo = inflation (local
costmap; the global one is listed but off). Coloured points = depth cloud
(a thinned preview, one point per 10 cm cube; the costmap uses the full cloud).
**Orange line = planned path, cyan line = path actually driven.** Cyan
rectangle = `published_footprint` (the footprint Nav2 really uses). Yellow
pins = saved waypoints. White arrow = pose from `/odometry/local`, green arrow
= current goal (`/athena/goal_marker`). Grid = 1 m squares in `odom`. The cyan
nose block on the model points forward.

Rover model missing: 3D panel settings -> **Custom layers -> Add URDF**, source
**topic**, `/robot_description`. Some Foxglove versions do not add it themselves.

## Driving it

| Way | How | Notes |
|---|---|---|
| Click a point | 3D toolbar -> **Publish** tool, **Pose** on `/athena/goal_click`; click sets position, drag sets heading | `goal_manager` converts it into `map` once, at click time, then sends it to Nav2. Works with or without a GPS fix and in any Follow frame. Use this indoors |
| Type a coordinate | *Go to* tab: `{ "data": "25.2629548, 82.9838284" }` | Needs a GPS fix. Optional third number = final heading in **compass degrees** (0 = north, 90 = east): `"25.2629548, 82.9838284, 90"`. Omitted = finish facing the way it travelled |
| Drive by hand | teleop arrows | Cancels any running Nav2 goal |

Abort a goal: `{ "data": "cancel" }` (or `stop`) on the *Go to* topic, or
`/athena/goal_cancel`. **The teleop STOP button does not abort a goal.** It
publishes a zero direction, `teleop_mux` only cancels Nav2 when an input crosses
its 0.02 deadband, and zero is below it. Nudging an arrow cancels; STOP does not.

Teleop details: while idle it publishes nothing, so Nav2 owns the wheels. On
release it sends one explicit zero, so a closed tab or dropped WiFi stops the
rover instead of latching the last command. The buttons send a direction (+-1);
the limit is applied on the rover and changed live from the *Speed* tab
(`{ "data": 0.35 }`). `/athena/teleop/max_turn` does the same for turn rate but
no tab publishes it: send it from the command line. Both are clamped by the
`speed_ceiling` (1.0 m/s) and `turn_ceiling` (2.0 rad/s) parameters. The
`goal_manager` reply appears in the *Goal* readout, under the *Nav status*
line from `nav_status`, which says why the last goal ended (`FAILED ...: no path`,
`... blocked: collision ahead`, `REPLACED`, `CANCELED: manual control taken`, ...).

## Marking places

Drive somewhere, *Mark* tab, name it, **Mark this spot** (`{ "data":
"charging_point" }`; an empty name auto-numbers `wp1`, `wp2`, ...). *Return*
with the same name drives back. Stored in
`~/.config/athena_remote/waypoints.yaml` on the Jetson; survives reboots. Each
waypoint stores its map pose always, and lat/lon only if there was a fix:

- **GPS-tagged**: valid forever.
- **Map-only**: valid until the next restart (the map origin moves when the datum is re-locked).

*Saved places* lists them and shows which kind.

## No-GPS trap

With no fix, `navsat_transform`'s `/fromLL` does not refuse: it answers map
`(0, 0)` for every coordinate on earth. Nav2 would accept a goal at the origin and
report "reached", so the panel would show success for a goal that was never
understood. `goal_manager` therefore refuses lat/lon goals until it has seen a
real fix and says so on `/athena/goal_status`. Click a 3D goal instead. Set
`require_gps_fix:=false` only for bench testing.

## Which frame to view in

The 3D panel's **Follow** setting decides what "still" means:

- **`odom`** (layout default). World fixed, rover moves. Use this to judge
  odometry: obstacles must stay put. Also the frame a clicked goal is stamped in.
- **`base_link`**. Camera rides the rover. Comfortable to drive, poor for goal
  testing: a world sliding under a fixed rover hides odometry faults.
- **`map`**. Adds GPS. Indoors it wanders metres; that is the GPS, not a bug.

Follow also sets the frame of clicked goals (Foxglove stamps the panel's display
frame). Clicks go to `/athena/goal_click`, and `goal_manager` converts them into
`map` at click time, so any Follow frame works. Never point the Publish tool back
at `/goal_pose` with Follow on `odom` or `base_link`: Nav2 Humble keeps the click's
frame and stamp, re-reads it at that stamp on every replan, and TF holds only 10 s,
so the goal fails about 10 s after the click (every such goal on 2026-10-02 did).

## Bandwidth

The tunnel is the bottleneck, not the Jetson. Measured 2026-10-02 over campus
WiFi, the SSH tunnel carried **4.7-7.0 MB/s**. What the panel pulls by default:

| Stream | Size x rate | MB/s |
|---|---|---|
| Camera, `/athena/camera/compressed` (JPEG q70, 640x480) | ~28 KiB x 5 Hz (indoors) | ~0.15 (up to ~0.3 outdoors) |
| Depth cloud, `/athena/points_preview` (~1,500 points) | ~18 KiB x 2 Hz | ~0.035 |
| Local costmap | 25 KiB x 1.7 Hz | 0.04 |
| TF, odometry, trail, everything else | | ~0.05 |

**Never serve `/camera/camera/color/image_raw` over the tunnel.** It is
921,672 bytes a frame at 15 Hz, 13.8 MB/s, and foxglove_bridge's WebSocket
server sends it raw: it does not transcode to H.264 (that is only done by its
remote-access gateway, which is off). When the panel asked for it, every
buffer between the bridge and the laptop filled (bridge send 4 MB, sshd
receive 6 MB, SSH about 2 MB, roughly 2 s at link speed) plus the bridge's own
message backlog, and the costmap, TF and everything else waited behind
the video in the same TCP stream. That is why the costmap looked slow too.

The default `topic_whitelist` leaves out the raw colour image, the raw depth
image and the full point cloud. It serves TF, `/robot_description`, GPS,
`/plan`, both odometry topics, both costmaps, every `/athena/*` topic
(including the camera JPEG), the diagnostic set (`/goal_pose`,
`/cmd_vel_nav`, `/vio/odometry{,_raw}`, `/imu/data`,
`/local_costmap/published_footprint`, `/rosout`, `/diagnostics`, the
`navigate_to_pose` action feedback and status), and the point cloud only as
`/athena/points_preview`. The costmap's own cloud, `points_downsampled`, is
78 KB at 11-12 Hz = 0.9 MB/s; once the camera was compressed it was about 80 %
of the panel's traffic, so `panel_cloud` thins it on a 10 cm voxel grid (shape
kept, about 1,500 points), x/y/z only, at 2 Hz. Measured end to end through the
bridge on 2026-10-02: camera 147 KB/s + cloud 35 KB/s + costmap 44 KB/s, about
0.23 MB/s in total, against roughly 14.8 MB/s before.

On a weak link, tune the camera without touching the real camera stream:
`camera_rate:=2.0`, `camera_quality:=50`, `camera_width:=320`, and for the cloud
`cloud_rate:=1.0` or `cloud_voxel:=0.2`. Hiding a layer in the 3D panel also
unsubscribes it, so it costs nothing. Do not lower
the RealSense profile for the panel's sake. rgbd_odometry and the costmap use
the same streams.

`num_threads` is 3, not one per core, so the bridge leaves the Jetson's 6 cores
free for rtabmap, the EKFs and Nav2. Nav2's action topics are *hidden* topics
(`ros2 topic list` omits them without `--include-hidden-topics`, and the bridge
drops them too); the launch file sets `include_hidden: True` for them, and
whitelisting alone is not enough. For everything, for example on a wired LAN:

```bash
ros2 launch athena_remote foxglove.launch.py topic_whitelist:="['.*']"
```

To check the link is keeping up, on the Jetson: `ss -tni sport = :8765`. A
`Send-Q` that sits near 4 MB means the panel is asking for more
than the link carries.

## Troubleshooting

| Symptom | Cause |
|---|---|
| Connection refused instantly, HTTP 400, nothing in the bridge log | Foxglove too old for `foxglove.sdk.v1` (see Setup) |
| No path lines in 3D | Orange needs an active Nav2 goal; the cyan trail needs the rover to have moved 5 cm. Neither shows on a stationary rover with no goal |
| Cyan trail never appears while driving | `trajectory` follows `/odometry/global`, which exists only with `ekf_global`. Bringup picks `/odometry/local` under `local_only:=true`; a standalone `foxglove.launch.py` does not: pass `trajectory_source:=/odometry/local` |
| Satellite map blank | No GPS position. `ros2 topic echo /gps/fix --field status.status` (`0` = fix, `-1` = none; the number is in a nested `NavSatStatus`, so `--field status` prints the whole sub-message). If your network blocks Google tiles, switch the layer to OpenStreetMap |
| Map shows a position but GPS is not working | The panel plots lat/lon without checking status. The *GPS fix* readout turns the marker red and labels it `NO FIX` when `status` is `-1` |
| 3D view empty | Wrong Follow frame: use `odom` with no GPS fix |
| Rover model missing | Add the URDF layer (above) |
| Camera panel blank | The camera driver did not start; check launch output for RealSense errors |
| Teleop has no effect | The bridge needs the `clientPublish` capability (set by default). Reconnect Foxglove |
| Everything laggy, costmap and camera seconds behind | The panel is asking for more than the tunnel carries, usually a raw image topic. See Bandwidth; check `ss -tni sport = :8765` |
| Camera panel blank but the camera runs | Image panel must point at `/athena/camera/compressed` (re-import the layout); `panel_camera` needs `operator_nodes:=true` |
