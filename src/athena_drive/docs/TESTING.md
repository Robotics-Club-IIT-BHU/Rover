# End-to-End Drive & Motor Verification

Step-by-step verification procedures for the Athena drive system, from single-motor rotation to autonomous Nav2 path execution.

Coordinate conventions adhere strictly to **REP-103**:
- **+x:** Forward
- **+y:** Left
- **+z:** Up
- **+yaw:** Counter-clockwise viewed from above

> [!IMPORTANT]
> Lift the rover off the ground so all six wheels spin freely before executing any motor rotation tests. Stop the ROS navigation stack to release the serial port lock.

---

## 1. Single Motor & Polarity Verification

Test all six motor channels sequentially:

```bash
ros2 run athena_drive wiring_check
```

The script spins Motors 1 through 6 forward in order.
**Verify:** The top of each wheel rolls toward the front bumper of the rover.

| Observed Wheel Motion | Root Cause | Resolution |
|---|---|---|
| Single wheel spins backward | Inverted motor channel polarity | Invert flag in firmware via `ros2 run athena_drive calibrate` (Stage 1), or swap the motor leads at the driver board. |
| Single wheel does not spin | Hardware or electrical failure | Check 12V motor supply rail, screw terminals, and driver board fuse. The software cannot address individual dead wheels. |
| All three wheels on a side spin backward | Side polarity reversed | Set `invert_left: true` or `invert_right: true` in `drive.launch.py`. |

### Advanced Wiring Checks

```bash
# Drive left side wheels, then right side wheels
ros2 run athena_drive wiring_check --ros-args -p mode:=sides

# Spin left test (verifies side mapping)
ros2 run athena_drive wiring_check --ros-args -p mode:=spin
```
*Expected Spin Left Result:* The rover spins counter-clockwise viewed from above. If it spins clockwise, the left and right driver channel connections are reversed.

---

## 2. ROS 2 Coordinate & Odometry Agreement

With the full stack running, confirm that motor motion maps accurately into ROS 2 odometry:

```bash
# Terminal 1: Monitor local odometry
ros2 topic echo /odometry/local --field pose.pose.position

# Terminal 2: Monitor TF yaw orientation
ros2 run tf2_ros tf2_echo odom base_link
```

- When commanded **forward**: $x$ position increases.
- When commanded **spin left**: yaw angle increases.

Run the automated coordinate frame validator:

```bash
ros2 run athena_gps_nav tf_check
```

*Expected results:*
- Gravity points **+z in `base_link`** (verifies IMU orientation and REP-103 axis conversion).
- Camera point cloud centroid is located in **+x** (verifies camera looks forward).

---

## 3. Staged Bringup Verification

### Stage 1: Manual Teleoperation (No Localization)

```bash
ros2 launch athena_drive drive.launch.py
ros2 run athena_drive teleop
```

- Arrow keys drive the rover (Up = Forward, Left = Spin Left).
- Releasing arrow keys halts motion within ~0.4 s.
- Unplugging the Pico W USB cable immediately trips the hardware watchdog and cuts motor power.

### Stage 2: Nav2 with RTK Global Localization

```bash
# Terminal 1: Full system bringup
ros2 launch athena_gps_nav bringup.launch.py

# Terminal 2: Stack health check
ros2 run athena_gps_nav stack_check
```

In RViz (Fixed Frame `odom` or `map`):
1. Send a **2D Goal Pose** a few meters ahead.
2. Confirm the path is generated on `/plan`.
3. Confirm velocity commands publish on `/cmd_vel_nav` and the rover tracks the trajectory smoothly.

### Stage 3: Dynamic Obstacle Avoidance

1. Place a test obstacle (e.g. cardboard box) 1.5 m ahead in the planned trajectory.
2. Nav2 re-plans a path around the obstacle.
3. The local costmap visualizes dark lethal cells with an inflated clearance safety halo.
4. Step directly in front of the rover: the controller halts forward motion immediately without colliding.

---

## 4. Recommended First Field Drive Procedure

1. **Set Initial Speed Ceiling:**
   For cautious indoor testing, limit the maximum PWM:
   ```bash
   ros2 launch athena_drive drive.launch.py max_pwm:=100
   ```
2. **Teleop in open space:** Validate forward, reverse, and turn controls.
3. **Teleop near obstacles:** Confirm costmap updates in real time.
4. **Short Nav2 Goal (2 meters):** Clear open ground.
5. **Nav2 Goal with Obstacle:** Path bends smoothly around the object.
6. **Outdoors RTK Waypoints:** Run `gps_waypoint_follower` to verify centimeter-accurate stopping.
