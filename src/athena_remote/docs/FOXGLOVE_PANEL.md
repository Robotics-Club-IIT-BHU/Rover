# Foxglove Control Panel

Foxglove Studio provides a browser-based UI for monitoring and teleoperating the Athena rover. It renders on your local workstation, preserving the Jetson's CPU resources for autonomy.

```
Workstation / Laptop                     Jetson (Rover)
[Foxglove Studio] --ws://localhost:8765-- [SSH Tunnel] --> [foxglove_bridge: 127.0.0.1:8765]
```

---

## 1. Quick Connection Workflow

### Step 1: Start Bridge on Jetson
> [!NOTE]
> If `bringup.launch.py` is already running, the bridge is already active (starts at t=15s). Do **not** launch it twice.

If running standalone without bringup:
```bash
ros2 launch athena_remote foxglove.launch.py
```

### Step 2: Open SSH Tunnel on Workstation
Set your Jetson's IP variable, then open port forwarding:

```bash
# On your laptop:
export JETSON_IP=<jetson-ip>   # e.g., 192.168.1.50 or ubuntu.local

# Open background SSH tunnel (keep terminal open)
ssh -N -L 8765:localhost:8765 robo@$JETSON_IP
```

> [!TIP]
> If port 8765 is in use locally (`bind: Permission denied`), forward another port (e.g., `18765`):
> `ssh -N -L 18765:localhost:8765 robo@$JETSON_IP` and connect Foxglove to `ws://localhost:18765`.

### Step 3: Copy Panel Layout (One Time)
Transfer the preconfigured layout JSON from the rover to your laptop:

```bash
# On your laptop:
scp robo@$JETSON_IP:/home/robo/athena/src/athena_remote/config/athena_foxglove_layout.json .
```

### Step 4: Open and Connect Foxglove
1. Open **[app.foxglove.dev](https://app.foxglove.dev)** (or Foxglove Desktop App).
2. Click **Open connection** -> **Foxglove WebSocket**.
3. Set URL to: `ws://localhost:8765` -> Click **Open**.
   *(Always use `localhost`, as HTTPS web browsers block non-localhost insecure WebSockets).*
4. Import layout: Top right **Layout menu** -> **Import from file...** -> select `athena_foxglove_layout.json`.

---

## 2. Panel Overview

```
+-------------------------------+-----------------+-------------+
|                               |  SATELLITE MAP  |   CAMERA    |
|           3D VIEW             |                 |             |
|                               +-----------------+-------------+
|                               |        |  CONTROLS  | READOUTS |
|                               | TELEOP |  (4 tabs)  | (8 tabs) |
+-------------------------------+--------+------------+----------+
```

| Panel | Description | Topics Displayed |
|---|---|---|
| **3D View** | Costmaps, point cloud, plans, TF, and robot model | `/local_costmap/costmap`, `/athena/points_preview`, `/plan`, `/athena/trajectory`, `/robot_description` |
| **Satellite Map** | GPS location on satellite imagery | `/gps/fix`, `/athena/trajectory_geojson`, `/athena/waypoints_geojson` |
| **Camera** | Compressed JPEG colour stream (5 Hz, low bandwidth) | `/athena/camera/compressed` |
| **Teleop** | Manual driving buttons (cancels active autonomous goal) | `/athena/teleop/cmd_vel` |
| **Controls** | Action tabs: *Mark*, *Return*, *Speed*, *Go to* | `/athena/waypoint_save`, `/athena/waypoint_goto`, `/athena/goal_text`, `/athena/teleop/max_speed` |
| **Readouts** | Status tabs: *Goal*, *Decisions*, *Speed*, *Teleop*, *GPS*, *Log*, etc. | `/athena/nav_status`, `/athena/decision_status`, `/athena/goal_status`, `/gps/fix`, `/rosout` |

### 3D Visualizer Color Legend
- **Orange line:** Currently planned Nav2 path (`/plan`).
- **Cyan line:** Driven trajectory history (`/athena/trajectory`).
- **Cyan rectangle:** Actual footprint evaluated by Nav2 costmaps (`/local_costmap/published_footprint`).
- **Dark grid cells:** Lethal obstacles.
- **Light halo:** Costmap inflation layer.
- **Yellow markers:** Saved persistent waypoints.

---

## 3. Driving the Rover

### Method A: Click Goal in 3D View (Recommended Indoors / Bench)
1. Select the **Publish** tool (arrow icon) on the 3D panel toolbar.
2. Publish to `/athena/goal_click` or directly to `/goal_pose` (clicks on `/goal_pose` stamped in `base_link` or `odom` are auto-fixed to `map` with fresh timestamp to prevent 10s TF buffer expiration).
3. Click target position and drag to orient heading.
4. Autonomous navigation plans and initiates immediately.

### Method B: GPS Coordinates (*Go to* Tab)
1. In the Controls *Go to* tab, enter coordinate JSON:
   ```json
   {"data": "25.2629548, 82.9838284"}
   ```
   *(Optional 3rd value specifies final heading in compass degrees: `"25.2629548, 82.9838284, 90"`).*
2. Requires valid GPS fix (`status >= 0`).

### Method C: Manual Teleoperation
- Use arrow controls in the **Teleop** tab.
- Nudging any teleop arrow **immediately cancels** any running Nav2 autonomous goal to prevent command conflicts.
- Speed limits are adjusted in the **Speed** tab (`{"data": 0.35}`).

### Aborting Active Goals
- Publish `cancel` via the *Go to* tab: `{"data": "cancel"}`.
- Or nudge any manual arrow in Teleop.

---

## 4. Saving and Returning to Waypoints

- **Mark Spot:** Open *Mark* tab -> send `{"data": "sample_site_1"}`.
- **Return to Spot:** Open *Return* tab -> send `{"data": "sample_site_1"}`.
- Stored permanently in `~/.config/athena_remote/waypoints.yaml` on the Jetson.

---

## 5. Troubleshooting & Diagnostics

| Symptom | Cause | Solution |
|---|---|---|
| **Connection refused immediately / HTTP 400** | Outdated Foxglove client | Update Foxglove Studio to a version supporting protocol `foxglove.sdk.v1`. |
| **Port 8765 Connection refused** | Bridge not running | Ensure `bringup.launch.py` has finished starting (takes 15s) or run `ros2 launch athena_remote foxglove.launch.py`. |
| **Panels blank or topics missing** | Layout not loaded | Import `athena_foxglove_layout.json` via Layout menu -> Import from file. |
| **High latency / freezing streams** | Bandwidth saturation | Never stream raw camera topics (`/camera/.../image_raw`). Use compressed `/athena/camera/compressed` and thinned `/athena/points_preview`. |
| **Robot model missing in 3D** | URDF layer not added | 3D panel settings -> Custom layers -> Add URDF -> source topic `/robot_description`. |
| **Satellite map empty** | No GPS fix or tiles blocked | Verify `/gps/fix` has status >= 0 (`ros2 topic echo /gps/fix --field status.status`). If Google tiles blocked, switch tile provider in Map panel settings. |

### Test Bridge Health from Jetson
```bash
python3 -c "import socket,base64,os; s=socket.create_connection(('127.0.0.1',8765),timeout=5); k=base64.b64encode(os.urandom(16)).decode(); s.sendall(('GET / HTTP/1.1\r\nHost: 127.0.0.1:8765\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: '+k+'\r\nSec-WebSocket-Version: 13\r\nSec-WebSocket-Protocol: foxglove.sdk.v1\r\n\r\n').encode()); print(s.recv(120).decode(errors='replace').splitlines()[0])"
# Expected: HTTP/1.1 101 Switching Protocols
```
