# Topic Reference & Telemetry Specifications

Topic specifications, message definitions, and interface behaviors across the `athena_remote` subsystem and Foxglove Studio interface.

For general operator instructions, see the [athena_remote README](../README.md).  
For web panel setup, see [FOXGLOVE_PANEL.md](FOXGLOVE_PANEL.md).

---

## 1. Remote Navigation & Teleoperation Topics

Topics routed between operator interfaces (Foxglove Studio, web portal, or CLI) and the Athena autonomy core:

| Topic | Message Type | Direction | Functional Purpose |
|---|---|---|---|
| `/athena/goal_gps` | `sensor_msgs/NavSatFix` | Input | Dispatches a goal as geographic coordinates (lat/lon). Validated by `goal_manager`. |
| `/athena/goal_text` | `std_msgs/String` | Input | Dispatches a text coordinate goal (`"lat, lon [heading]"`) or string command (`"cancel"`). |
| `/athena/goal_cancel` | `std_msgs/Empty` | Input | Immediately aborts the current Nav2 navigation action. |
| `/athena/goal_click` | `geometry_msgs/PoseStamped` | Input | Point clicked in Foxglove 3D scene. Converted to `map` frame by `goal_manager`. |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Input | Direct click topic from Foxglove/RViz. Intercepted by `goal_manager` if stamped in moving frames (`base_link`, `odom`) and auto-fixed to `map` with fresh timestamp. |
| `/athena/goal_status` | `std_msgs/String` | Output | Real-time human-readable feedback on goal parsing and acceptance. |
| `/athena/nav_status` | `std_msgs/String` | Output | Comprehensive structured status card (Activity, Decision, Root Cause, Target & Metrics). |
| `/athena/decision_status` | `std_msgs/String` | Output | Dedicated transparent decision status card for Foxglove and ROS 2 operators. |
| `/athena/goal_marker` | `visualization_msgs/Marker` | Output | 3D visual target marker rendered in the operator display. |
| `/athena/waypoint_save` | `std_msgs/String` | Input | Records current rover pose into persistent memory under the provided name. |
| `/athena/waypoint_goto` | `std_msgs/String` | Input | Navigates to a previously recorded named waypoint. |
| `/athena/waypoint_delete` | `std_msgs/String` | Input | Removes a named waypoint from persistent storage. |
| `/athena/waypoint_list` | `std_msgs/String` | Output | JSON list of currently saved waypoints. |
| `/athena/waypoints` | `visualization_msgs/MarkerArray` | Output | 3D map pins representing saved waypoint locations. |
| `/athena/waypoints_geojson` | `foxglove_msgs/GeoJSON` | Output | Geographic pins rendered on the Foxglove Satellite Map panel. |
| `/athena/trajectory` | `nav_msgs/Path` | Output | Driven history path rendered in the 3D scene. |
| `/athena/trajectory_geojson` | `foxglove_msgs/GeoJSON` | Output | Driven trail rendered on the Foxglove Satellite Map panel. |
| `/athena/trajectory_clear` | `std_msgs/Empty` | Input | Clears the active visualization trail. |
| `/athena/teleop/cmd_vel` | `geometry_msgs/Twist` | Input | Normalized stick velocity inputs ($\pm 1.0$) from the web/Foxglove teleop widget. |
| `/athena/teleop/max_speed` | `std_msgs/Float32` | Input | Velocity scale parameter defining full-stick linear velocity (m/s). |
| `/athena/teleop/max_turn` | `std_msgs/Float32` | Input | Velocity scale parameter defining full-stick angular velocity (rad/s). |
| `/athena/teleop/status` | `std_msgs/String` | Output | Teleoperation operational status and currently commanded limits. |
| `/athena/camera/compressed` | `sensor_msgs/CompressedImage` | Output | JPEG-compressed forward video stream for low-bandwidth web streaming. |
| `/athena/points_preview` | `sensor_msgs/PointCloud2` | Output | Downsampled preview point cloud (~1,500 pts, 2 Hz) for remote 3D monitoring. |

---

## 2. GNSS & RTK Telemetry Interface (`/gps/fix`)

The `/gps/fix` topic carries absolute WGS-84 coordinates and dynamic error covariance from the Cube Orange and DroneCAN RTK GNSS receiver.

- **Topic:** `/gps/fix`
- **Type:** `sensor_msgs/msg/NavSatFix`
- **Publishing Node:** `athena_gps_nav/pixhawk_bridge`
- **Rate:** 5.0 Hz

### Message Structure & RTK Status Mapping

```yaml
header:
  stamp: {sec: ..., nanosec: ...}
  frame_id: "gps_link"
status:
  status: 2      # GNSS Fix Quality (see table below)
  service: 1     # SERVICE_GPS
latitude: 25.2629548
longitude: 82.9838284
altitude: 82.4
position_covariance: [0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.0016]
position_covariance_type: 2  # COVARIANCE_TYPE_APPROXIMATED
```

### Status Code Definitions (`status.status`)

| Value | ROS 2 Constant | GNSS Operational Fix State | Typical Accuracy ($1\sigma$) |
|---|---|---|---|
| `-1` | `STATUS_NO_FIX` | No satellites acquired / indoors. | N/A |
| `0` | `STATUS_FIX` | Autonomous 3D GNSS lock. | ~3.50 m |
| `1` | `STATUS_SBAS_FIX` | Differential GNSS (DGPS / SBAS). | ~0.80 m |
| `2` | `STATUS_GBAS_FIX` | **RTK Float or RTK Fixed** solution. | **0.015 – 0.030 m (Fixed)**<br/>0.15 – 0.40 m (Float) |

### Covariance Derivation (`position_covariance`)

The 9-element array represents the spatial covariance matrix $\Sigma = \text{diag}(\sigma_x^2, \sigma_y^2, \sigma_z^2)$:
- **RTK Active (`status.status == 2`):** Covariance is computed directly from MAVLink 2 extension field $h_{acc}$ reported by the receiver, bounded by `gps_rtk_sigma_floor` (0.02 m):
  $$\sigma_x = \sigma_y = \max\left(\frac{h_{acc}}{1000}, 0.02\right)$$
  In RTK Fixed, this evaluates to $\sigma_x^2 \approx 0.0004\text{ m}^2$ (centimeter confidence).
- **Autonomous GPS (`status.status == 0`):** Covariance falls back to the HDOP error model:
  $$\sigma_x^2 = \sigma_y^2 = (\text{HDOP} \times 1.74)^2 + 3.06^2$$

---

## 3. Related Localization Topics

| Topic | Type | Provider | Description |
|---|---|---|---|
| `/odometry/local` | `nav_msgs/Odometry` | `ekf_local` | Continuous, jump-free local odometry in `odom` frame (fuses VIO + Gyro). |
| `/odometry/global` | `nav_msgs/Odometry` | `ekf_global` | Global map-frame odometry with RTK GNSS corrections. |
| `/odometry/gps` | `nav_msgs/Odometry` | `navsat_transform` | Metric coordinates in `map` frame converted from `/gps/fix`. |
| `/imu/data` | `sensor_msgs/Imu` | `pixhawk_bridge` | 50 Hz body rates (FLU) and linear accelerations (SI units). |

---

## 4. Command-Line Telemetry & Control

Direct CLI examples for monitoring and commanding without a browser interface:

```bash
# Monitor real-time RTK fix status and accuracy
ros2 topic echo /gps/fix --field status.status
ros2 topic echo /gps/fix --field position_covariance

# Dispatch a geographic waypoint goal
ros2 topic pub --once /athena/goal_text std_msgs/String '{data: "25.2629548, 82.9838284"}'

# Monitor goal feedback status
ros2 topic echo /athena/goal_status

# Cancel active goal
ros2 topic pub --once /athena/goal_cancel std_msgs/Empty '{}'

# Save a named waypoint at current location
ros2 topic pub --once /athena/waypoint_save std_msgs/String '{data: "docking_station"}'

# Command the rover to drive to a named waypoint
ros2 topic pub --once /athena/waypoint_goto std_msgs/String '{data: "docking_station"}'
```
