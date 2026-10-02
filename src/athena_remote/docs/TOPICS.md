# Control panel topics

Everything `athena_remote` publishes or subscribes to. Back to the
[package README](../README.md).

| Topic | Type | Direction |
|---|---|---|
| `/athena/goal_gps` | `sensor_msgs/NavSatFix` | in: go to this lat/lon |
| `/athena/goal_text` | `std_msgs/String` | in: `"lat, lon [heading]"` or `"cancel"` |
| `/athena/goal_cancel` | `std_msgs/Empty` | in: abort |
| `/athena/goal_click` | `geometry_msgs/PoseStamped` | in: pose clicked in the 3D panel, any frame |
| `/athena/goal_status` | `std_msgs/String` | out: what just happened |
| `/athena/nav_status` | `std_msgs/String` | out: why the last Nav2 goal ended (latched) |
| `/athena/goal_marker` | `visualization_msgs/Marker` | out: the goal, in 3D |
| `/athena/waypoint_save` | `std_msgs/String` | in: name to save here |
| `/athena/waypoint_goto` | `std_msgs/String` | in: name to drive to |
| `/athena/waypoint_delete` | `std_msgs/String` | in: name to forget |
| `/athena/waypoint_list` | `std_msgs/String` | out: readable list |
| `/athena/waypoints` | `visualization_msgs/MarkerArray` | out: pins in 3D |
| `/athena/waypoints_geojson` | `foxglove_msgs/GeoJSON` | out: pins on the map |
| `/athena/trajectory` | `nav_msgs/Path` | out: trail in 3D |
| `/athena/trajectory_geojson` | `foxglove_msgs/GeoJSON` | out: trail on the map |
| `/athena/trajectory_clear` | `std_msgs/Empty` | in: wipe the trail |
| `/athena/teleop/cmd_vel` | `geometry_msgs/Twist` | in: direction, +-1 |
| `/athena/teleop/max_speed` | `std_msgs/Float32` | in: m/s at full stick |
| `/athena/teleop/max_turn` | `std_msgs/Float32` | in: rad/s at full stick |
| `/athena/teleop/status` | `std_msgs/String` | out: mode and limits |
| `/athena/camera/compressed` | `sensor_msgs/CompressedImage` | out: colour camera as JPEG for the Image panel |
| `/athena/points_preview` | `sensor_msgs/PointCloud2` | out: thinned depth cloud (x/y/z, ~1,500 points, 2 Hz) for the 3D panel |

Everything works from the command line, the quickest way to test without a
browser:

```bash
ros2 topic pub --once /athena/waypoint_save std_msgs/String '{data: "gate"}'
ros2 topic pub --once /athena/goal_text std_msgs/String '{data: "25.2629548, 82.9838284"}'
ros2 topic echo /athena/goal_status

# the controls that have no panel button
ros2 topic pub --once /athena/goal_text std_msgs/String '{data: "cancel"}'
ros2 topic pub --once /athena/waypoint_delete std_msgs/String '{data: "gate"}'
ros2 topic pub --once /athena/teleop/max_turn std_msgs/Float32 '{data: 0.8}'
ros2 topic pub --once /athena/trajectory_clear std_msgs/Empty '{}'
```

`teleop_mux` publishes the final `/cmd_vel` and calls the
`/navigate_to_pose/_action/cancel_goal` service when an input crosses its 0.02
deadband. When told to drive to a saved place, `waypoint_manager` publishes
`/athena/goal_gps` (GPS-tagged waypoint) or `/goal_pose` (map-only waypoint).
