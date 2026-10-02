# Testing the drive end to end

Checks, in order, from bare wheels to Nav2 driving around an obstacle. Prop
the rover up so the wheels spin free for every step except where noted, and
stop the stack before using `wiring_check` or `calibrate` (the Pico's serial
port is exclusive). Back to the [package README](../README.md).

Everything downstream assumes ROS conventions (REP-103) in the robot's frame:
**+x forward, +y left, +z up, and +yaw is counter-clockwise seen from above.**

## 1. Each motor turns the right way

```bash
ros2 run athena_drive wiring_check          # wheels off the ground
```

It drives motors 1 to 6 forward one at a time, naming each. For every one: **does
the top of that wheel roll toward the front of the rover?**

- Wrong direction on one wheel: flip that motor's invert flag (`I <n> 1`, or let
  `calibrate` do it), **or** swap its two power leads at the driver. One, not both.
- **No movement on one wheel: hardware.** The bridge sends one PWM per side and
  the firmware writes it to all three motors on that side, so no software path
  can single one wheel out. Check power, leads, driver channel (the
  [pin map](../README.md#pico-to-motor-driver-pin-map) says which wires).
- Wrong on all three of a side: set `invert_left` or `invert_right` on
  `motor_bridge`. Fix it in the wiring **or** in software, never both.

Options: `-p motor:=3` (one motor), `-p pwm:=150` (stiff gearbox), `-p pause:=5.0`.
`-p mode:=sides` drives the whole left side, then the right, with `V`; all three
wheels on the active side should turn together. If **one** wheel of a side stays
still while the other two turn, that is never software.

## 2. The sides are not swapped

```bash
ros2 run athena_drive wiring_check --ros-args -p mode:=spin
```

Drives forward, then spins left. **Spin left must rotate counter-clockwise seen
from above.** If it spins the other way the left and right driver connections
are exchanged.

## 3. It matches what ROS believes

With the nav stack running, drive and watch `/odometry/local`:

```bash
ros2 topic echo /odometry/local --field pose.pose.position
ros2 run tf2_ros tf2_echo odom base_link          # roll/pitch/yaw, live
```

- Drive **forward**: `x` increases.
- Spin **left**: yaw increases. `/imu/data` carries a quaternion, not an angle,
  so read yaw off TF as above.

Whole-system check, which catches what the above missed:

```bash
ros2 run athena_gps_nav tf_check
```

It verifies gravity points **+z** in `base_link` (catches an upside-down or
unconverted IMU; a wrong IMU still reads 9.81, only the *sign* gives it away) and
that the depth cloud lands at **+x** (catches a camera rotated sideways).

In RViz, Fixed Frame `odom`, drive forward a metre: the robot marker moves along
its own +x and obstacles stay **fixed in the world**. If obstacles slide along
with the robot, odometry is not tracking the motion.

## 4. Bring-up in three stages

Each stage proves one thing before the next depends on it.

**Stage 1, teleop only** (no localization involved):

```bash
ros2 launch athena_drive drive.launch.py
ros2 run athena_drive teleop
```

Arrows drive, releasing them stops the rover. Verify: forward goes forward and
left goes left; releasing the keys stops it within ~0.4 s; unplugging the Pico USB
stops it (the firmware watchdog).

**Stage 2, Nav2 with GPS.** Stop the stage 1 launch first: bringup includes
`drive.launch.py`, and a second `motor_bridge` cannot open the port, so it streams
into nothing while the first one drives.

```bash
ros2 launch athena_gps_nav bringup.launch.py
ros2 run athena_gps_nav stack_check                 # every line PASS
```

In RViz (`rviz2 -d ~/athena/src/athena_gps_nav/config/athena_bench.rviz`, Fixed
Frame `odom`) use **2D Goal Pose** a few metres ahead. Watch for a path on
`/plan`, `/cmd_vel` publishing and the rover following. Before trusting a goal,
drive forward a metre by hand and confirm `x` increases in `/odometry/local`;
if the rover drives forward while odometry says backwards, a goal drives it away
from the target.

Watch `/cmd_vel_nav`, not `/cmd_vel`, to see what the *controller* asked for.
`/cmd_vel` has six publishers (`teleop_mux`, the velocity smoother and several
`behavior_server` behaviours), so Foxglove teleop contaminates a reading taken
there. Only `controller_server` publishes `/cmd_vel_nav`;
`ros2 topic info /cmd_vel -v` lists the rest.

Outdoors, with sky, record and follow a GPS route:

```bash
ros2 run athena_gps_nav gps_waypoint_logger --ros-args -p output_file:=/home/robo/route.yaml
ros2 run athena_gps_nav gps_waypoint_follower --ros-args -p waypoints_file:=/home/robo/route.yaml
```

Expect arrival within a few metres, not centimetres: GPS here is **+-3.5 m
1-sigma**, a receiver limit, not a tuning one.

**Stage 3, local obstacle avoidance.** This is the depth camera's job and works
with **no GPS at all** (the local costmap lives in `odom`).

- *Static:* put a box ~1.5 m ahead, slightly off the line to a goal and send the
  goal. The path should bend around the box. RViz shows dark lethal cells where
  the box is, with a lighter inflation halo (the clearance Nav2 keeps).
- *Dynamic:* with the rover driving, step into its path a few metres ahead. It
  should re-plan around you and cells behind you should clear. Stand still directly
  in front: it should stop, not push through.
- *If it does not stop:* `ros2 topic hz /camera/camera/depth/color/points_downsampled`
  (camera alive?) and `ros2 run athena_gps_nav stack_check`. `local costmap
  marking` at 0 cells means the camera is not reaching the costmap: a perception
  fault, not a controller one.

Blind spots: the D435i sees nothing closer than ~0.2 m, nothing outside its ~87
degree cone and nothing behind. The costmap remembers what it saw, but a rover
turning in place only clears what the camera sees now. Approach obstacles head-on.

## Recommended order for the first real drive

1. `max_pwm:=100`: deliberately slow while you build confidence. It is an argument
   of `drive.launch.py`; `bringup.launch.py` does not forward it, so for the Nav2
   steps temporarily add `max_pwm: 100` to `~/.config/athena_drive/params.yaml`
   (and remove it after)
2. Teleop in open space
3. Teleop near an obstacle, watching the costmap
4. A short Nav2 goal, ~2 m, clear ground
5. A Nav2 goal with one obstacle in the way
6. GPS waypoints outdoors

Keep a hand near the Pico's USB cable throughout; pulling it is the fastest
stop you have.
