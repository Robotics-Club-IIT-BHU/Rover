# Testing the nav stack from Foxglove

How to send a goal from Foxglove and tell, from the panel alone, *why* it did
what it did. [README.md](README.md) and [docs/FOXGLOVE_PANEL.md](docs/FOXGLOVE_PANEL.md) cover connecting; this covers testing.

Written 2026-09-02, after a session where the rover yawed instead of driving.
The diagnostic panels described here exist because that bug was invisible
without them.

---

## 0. Before you connect

One stack, and only one. Two `bringup.launch.py` instances on the same
`ROS_DOMAIN_ID` will each run their own `ekf_global` broadcasting `map -> odom`
and their own `ekf_local` broadcasting `odom -> base_link`, and TF flips
between two independent estimates at 30 Hz. The plan looks fine and execution
is garbage. It is not obvious from any panel.

```bash
ros2 node list | sort | uniq -c | awk '$1>1'
```

Any output at all means duplicates. Nothing printed is what you want.

Run that on the Jetson, or over ssh with the domain set explicitly —
`ssh robo@172.20.119.87 'export ROS_DOMAIN_ID=42; source ~/athena/install/setup.bash && ros2 node list'`.
A non-interactive shell returns early from `~/.bashrc` and lands on domain 0,
where the list is empty and every duplicate is invisible.

**The Pico's serial port is exclusive**, and bringup starts its own
`motor_bridge`. A second bridge, or a stray `calibrate` / `wiring_check` /
`calibrate_speed` left running, takes the port or fails to get it, and
either way `/cmd_vel` stops reaching the wheels. The only sign is one
`could not open ... - running dry` line at startup, which scrolls past in
the launch output; after that the node behaves exactly like a healthy one.
Check before launching:

```bash
fuser -v /dev/ttyACM*
```

Then:

```bash
ros2 launch athena_gps_nav bringup.launch.py
```

Bringup already includes the Foxglove bridge and the motor bridge. Give it
~45 s: sensors at t=0, VIO at t=5, localization at t=8, Nav2 at t=12, panel
at t=15.

Four arguments, and these are all of them: `dry_run`, `use_camera`,
`foxglove`, `local_only`.

---

## 1. Static checks, before anything moves

Do these in order. Each one takes seconds and each has caught a real fault.

**TF tree is a single chain.** In the 3D panel the rover model should appear
once, upright, with the cyan nose block pointing forward. From the shell:

```bash
ros2 run tf2_tools view_frames
```

You want exactly one root (`map`), one `map -> odom`, one `odom -> base_link`,
and every sensor hanging off `base_link`. 11 frames hang directly off
`base_link` (the six wheels, `base_footprint`, the nose marker, `camera_link`,
`imu_link`, `gps_link`; the two masts were deleted on 2026-09-03), and the
RealSense's own frames hang below `camera_link`. The total frame count (27 when
this was written, before the masts were removed) will be lower now: count with
`view_frames`.

**The footprint is the real one.** The 3D panel draws
`/local_costmap/published_footprint` in cyan. It should be a 1.3 m x 1.0 m
rectangle around the rover — *the footprint Nav2 is actually using*, not the
one you think is configured. It comes from
`footprint: "[[0.65, 0.5], [0.65, -0.5], [-0.65, -0.5], [-0.65, 0.5]]"` in
`nav2_params.yaml`. If it looks like a small square, the params did not
reload.

> **Check how your configs are actually installed before blaming a stale
> build.** On this workspace `install/.../config/*.yaml` is a symlink to
> `build/.../config/*.yaml`, which is itself a symlink to `src/`. So the
> chain lands in `src` and an edit there *does* take effect on the next
> node start, no rebuild needed. Confirm rather than assume:
>
> ```bash
> readlink -f ~/athena/install/athena_gps_nav/share/athena_gps_nav/config/nav2_params.yaml
> ```
>
> If that prints a path under `src/`, editing is enough. If it prints a path
> under `build/` (a plain `colcon build`, no `--symlink-install`), you have
> a real copy and the edit changes nothing until you rebuild. Adding a
> *new* config file always needs a rebuild either way: the `glob()` in
> `setup.py` runs at build time.

**Odometry is not running away while parked.** Watch the *Speed* tab with the
rover stationary. `measured m/s` and `measured rad/s` should sit on zero. Any
sustained non-zero reading on a parked rover is phantom odometry, and every
goal will fail in a confusing way until it is fixed.

The 90-second version, which is what you should actually trust:

```bash
python3 - <<'EOF'
import rclpy, math, time, tf2_ros
from rclpy.node import Node
rclpy.init(); n=Node('drift'); b=tf2_ros.Buffer(); tf2_ros.TransformListener(b,n)
t0=time.time(); first=last=None
while time.time()-t0 < 90:
    rclpy.spin_once(n, timeout_sec=0.2)
    try: t=b.lookup_transform('odom','base_link',rclpy.time.Time()).transform.translation
    except Exception: continue
    if first is None: first=(t.x,t.y)
    last=(t.x,t.y)
if first is None: print("no odom -> base_link transform in 90 s - nothing is publishing it")
else: print(f"moved {math.hypot(last[0]-first[0], last[1]-first[1]):.3f} m while parked")
EOF
```

`0.000 m` is the pass. This test read **10.350 m** before the 2026-09-02 fixes.

**VIO can actually see.** Point the camera at a textured scene — a blank wall
or the ceiling is not enough. `/vio/odometry_raw` carries what rtabmap
computed; `/vio/odometry` carries what survived `vio_gate`. If the gate is
dropping nearly everything, the *Log* tab says so — the message text is
`gate: ...`, from the `vio_gate` node:

```
gate: 93/105 dropped, 12 zero-velocity held
```

That is correct behaviour when the camera sees nothing, but it means
`ekf_local` has **no linear-velocity source at all**. The rover will command
forward motion and never register progress. Fix the scene, not the gate.

---

## 2. Teleop first, always

Before any autonomous goal, drive it manually from the Teleop panel. This
proves the whole chain below Nav2 — `/cmd_vel` -> `motor_bridge` -> serial ->
Pico -> motors — and it proves the direction conventions.

- Forward should go forward.
- Positive turn (left) should turn left.

If teleop works and goals do not, the fault is in Nav2 or localization, not
wiring. That single split saves a lot of time.

**Teleop is also the stop button.** `teleop_mux` cancels the running Nav2 goal
on first real input, so nudging a teleop *arrow* aborts an autonomous run. It
is not an e-stop in the safety sense — there is no hardware cut in this stack —
but it is the fastest software stop you have. The panel's STOP button is not
that: it publishes a zero direction, which sits below the mux's 0.02 deadband
and therefore cancels nothing.

**That cuts the other way when you are measuring.** `teleop_mux` publishes on
`/cmd_vel`, so anyone touching the Foxglove teleop panel silently
contaminates any test that reads it — and it cancels the goal you were
watching at the same time. `ros2 topic info /cmd_vel -v` legitimately shows
six publishers here: `teleop_mux`, the velocity smoother, and several
`behavior_server` behaviours. `/cmd_vel_nav` is the clean signal; only
`controller_server` publishes there.

---

## 3. Sending the goal

In the 3D panel, use the **click-to-publish pose** tool. The layout publishes
`geometry_msgs/PoseStamped` on `/athena/goal_click`; `goal_manager` converts it into
`map` at click time and sends the goal (a click sent straight to `/goal_pose` in
`odom` or `base_link` expires after about 10 s). Click the position, drag to set
the final heading, release. Why a goal ended: the *Nav status* line in the *Goal* tab.

**Check the frame the first time.** Foxglove stamps the pose with the panel's
**display frame** (the *Follow* setting). The layout ships with Follow = `odom`,
so a click 2 m ahead of the rover is genuinely 2 m ahead. Open the *Goal pose*
readout tab and confirm:

```
header:
  frame_id: "odom"
```

This matters more than it sounds. If Follow is `map` and `map -> odom` carries
a yaw offset, a goal you *meant* as "2 m forward" is aimed somewhere else
entirely, the planner produces a perfectly correct path to the wrong place, and
the rover turns before driving. That is not a controller bug; the goal was
never where you thought.

Check `map -> odom` yaw whenever you are unsure:

```bash
ros2 run tf2_ros tf2_echo map odom
```

Near-zero yaw is healthy for the current configuration. If you see something
like +89°, `ekf_global` is fusing absolute yaw from a compass — see §6.

---

## 4. Reading what happened

Four places, in the order worth checking.

**Nav feedback tab** (`/navigate_to_pose/_action/feedback`) — is Nav2 even
running the goal? `distance_remaining` should fall. `number_of_recoveries`
climbing means it is stuck and cycling recovery behaviours.

**Speed tab** — the diagnostic plot. Five traces:

| Trace | Topic | Means |
|---|---|---|
| `controller linear m/s` | `/cmd_vel_nav.linear.x` | what the controller wants |
| `controller ANGULAR rad/s` | `/cmd_vel_nav.angular.z` | what the controller wants |
| `commanded m/s` | `/cmd_vel.linear.x` | what actually reached the motor bridge |
| `measured m/s` | `/odometry/local.twist.twist.linear.x` | what the EKF believes happened |
| `measured rad/s` | `/odometry/local.twist.twist.angular.z` | what the EKF believes happened |

`cmd_vel_nav` is the controller's raw output, **before** the velocity smoother.
That distinction is the whole point: it separates "Nav2 is asking for the wrong
thing" from "Nav2 is asking correctly and the request is not reaching the
wheels."

Reading the three levels together is what localises a fault. `cmd_vel_nav`
non-zero with `commanded` flat means the smoother or the mux is eating it.
Both non-zero with `measured` flat means the request reached the bridge and
the rover did not move, or moved and was not seen. And `commanded` non-zero
while `cmd_vel_nav` is flat means someone is driving from the teleop panel.

The signature to watch for:

> **angular saturating while linear sits at exactly 0.00, indefinitely.**

That is `RegulatedPurePursuit` in rotate-to-heading, refusing to translate
until it is pointed at the carrot. It is correct behaviour for a real heading
error, and a bug symptom when the heading error is not real — when it is
phantom odometry, or a lookahead so short that ordinary position noise trips
`rotate_to_heading_min_angle` (0.5 rad).

The lookahead half of that was fixed on 2026-09-04. With velocity scaling
on, the carrot is `clamp(|speed.linear.x| * lookahead_time, min, max)` and
speed is zero at the instant every goal starts, so it pinned at
`min_lookahead_dist` — 0.4 m at the time, which is *inside* a rover whose
footprint reaches x = +0.65. NavFn's 0.1 m grid snap alone was then enough
to put the carrot at −0.58 rad and zero `linear.x`, and it could never
recover, because building speed was the thing that would have lengthened the
lookahead. Measured down a clear corridor with a 2 m goal:

```
velocity-scaled, 0.4 m carrot : linear.x = 0.000 on 160/160 cycles
fixed 1.0 m carrot            : linear.x = 0.350 on 178/178 cycles
```

So `use_velocity_scaled_lookahead_dist` is now `false` with a fixed
`lookahead_dist: 1.0`. If the symptom comes back, check that it is still
false before looking anywhere else.

**Log tab** (`/rosout`) — planner failures, `vio_gate` drop counts, recovery
announcements, costmap warnings. Filter on `WARN`.

**3D panel** — the orange `/plan` line is what the planner produced. If the
orange line is sensible and the rover does not follow it, the problem is the
controller or odometry, not planning. If the orange line itself is wrong, the
goal or the costmap is wrong. This one visual split tells you which half of
the stack to investigate.

---

## 5. Failure modes and what they look like

| What you see | Where to look | Likely cause |
|---|---|---|
| Rover rotates forever, never translates | Speed tab: angular high, linear 0.00 | Phantom odometry (run the parked-drift test in §1), or a lookahead shorter than the footprint — check `use_velocity_scaled_lookahead_dist` is still `false`, see §4 |
| Plan is correct, rover drives somewhere else | *Goal pose* tab | Goal stamped in a frame with a yaw offset |
| Goal instantly reports success | *Nav feedback*, `/gps/fix` | No GPS fix, `/fromLL` returned map (0,0). `goal_manager` guards this; a raw `/goal_pose` click does not |
| Rover commands motion, never registers progress | *Speed* tab: commanded non-zero, measured flat 0 | VIO blind — camera on a blank surface. `ekf_local` has no linear-velocity source |
| Obstacles slide with the rover in 3D | Set Follow to `odom` | Odometry not tracking |
| Nothing marked in the costmap | 3D depth cloud layer | Camera height/pitch wrong in the URDF, or cloud not arriving |
| `number_of_recoveries` climbing | *Log* tab | Stuck; read which behaviour fired |
| Nothing moves, teleop or goal, and no errors | `fuser -v /dev/ttyACM*`, *Log* tab | Something else holds the Pico's port, or `motor_bridge` logged `FIRMWARE NOT RESPONDING` and is streaming into a dead link |
| One wheel does not turn | — | Always hardware. The bridge sends one PWM per side and the firmware writes it to all three motors on that side, so no software path can single a wheel out |
| Connection refused instantly, HTTP 400 | — | Foxglove too old for `foxglove.sdk.v1`. See [docs/FOXGLOVE_PANEL.md](docs/FOXGLOVE_PANEL.md#setup) |
| Empty node/topic list over ssh | `echo $ROS_DOMAIN_ID` | Non-interactive shell, domain 0 instead of 42. See §0 |

---

## 6. Known limitations right now

Read this before trusting a result.

- **`max_wheel_speed` is an unmeasured estimate (0.7 m/s), and so is
  `max_pwm` (255).** `~/.config/athena_drive/params.yaml` holds only
  `min_pwm: 20` and `track_width: 0.85`, so `motor_bridge` falls back to the
  built-in estimate for the other two. `max_wheel_speed` scales every
  velocity Nav2 commands, so distance-to-goal behaviour is only as good as
  that guess.

  Measuring it is two steps, not one. `ros2 run athena_drive calibrate_speed
  --ros-args -p mode:=topspeed` drives the rover and prints the number, but
  **saves nothing** — either run `ros2 run athena_drive calibrate --ros-args
  -p stages:=4`, which asks for the measured distance and writes the file,
  or add the line to `params.yaml` by hand. `motor_bridge` names the
  offenders at startup — `UNCALIBRATED, using built-in estimates for:
  max_wheel_speed, max_pwm` — so check the launch log rather than the file.

- **Absolute heading is not available.** `ekf_global` no longer fuses absolute
  yaw, because the Pixhawk's attitude reads roll −22° / pitch −15° with the
  rover flat. `map` yaw is now dead-reckoned, which is right for local goals
  and **not** sufficient for GPS waypoints. Heading needs to come back from a
  calibrated compass or dual-antenna RTK before waypoint work.
  `navsat_transform.use_odometry_yaw` is still `false` and still reads
  `/imu/data` yaw for the datum — fix both together.

- **Goals beyond ~50 m cannot be planned.** The global costmap is
  `rolling_window: true` at 100 x 100 m, so anything outside ±50 m of the
  rover is off the map. It was 60 x 60 m, which capped legs at ~30 m; at
  0.1 m resolution the larger map is 1e6 cells, about 1 MB, which is nothing
  on this Jetson.

- **A wedged RealSense now invalidates observations rather than going
  unnoticed.** Both costmaps set `expected_update_rate: 2.0` on the depth
  cloud source, so a camera that stops publishing for two seconds gets
  flagged instead of leaving stale obstacles marked forever. It is still
  worth watching the depth cloud in the 3D panel: the costmap has no time
  decay, so anything marked while the camera was healthy stays marked until
  something clears it.

- **Small turns may not move the rover outdoors.** `min_pwm: 20` was measured
  driving *straight*. Skid-steer scrub needs more torque, so corrections at
  ω ≤ 0.2 rad/s (PWM 27–49) may stall on grass or gravel while working fine
  on a hard floor.

- **The rover is blind behind and to the sides.** One forward-facing camera,
  87° wide, nothing rearward. This is why the stack runs its own behaviour
  tree, `behavior_trees/athena_nav_to_pose.xml`: the stock Nav2 tree's
  `Spin` (1.57 rad through 273° it cannot see) and `BackUp` (0.30 m
  reversing a 1.3 m rover into space no sensor has ever observed) are both
  removed. What is left is clear-the-costmaps and wait. Both plugins are
  still loaded by `behavior_server`, so a direct action call can still fire
  them — the tree just never does.

  The practical consequence when reading `number_of_recoveries`: a stuck
  goal now cycles clearing and waiting, and then fails cleanly and hands
  control back, instead of blindly reversing.

---

## 7. Bench testing without moving the rover

```bash
ros2 launch athena_gps_nav bringup.launch.py dry_run:=true
```

`dry_run` logs motor commands instead of sending them. Every panel behaves
normally and you can watch `cmd_vel_nav` respond to a goal. It also leaves
the Pico's serial port free, so `wiring_check` or `calibrate` can run in
another terminal at the same time.

Indoors, add `local_only:=true`. That drops `ekf_global` and
`navsat_transform` and pins `map` to `odom`, which is the right shape for
testing local obstacle avoidance with no sky. `/odometry/global` never
publishes in that mode, so bringup also switches the operator trail to
`/odometry/local` for you — the cyan trail keeps working. That only happens
when bringup starts the panel: a standalone
`ros2 launch athena_remote foxglove.launch.py` still defaults to
`/odometry/global` and needs `trajectory_source:=/odometry/local` passed by
hand.

One caveat that looks exactly like a bug: with `dry_run` a goal needing an
initial turn produces **pure rotation, linear.x pinned at 0.00, forever**. The
rover cannot physically turn, so the heading error never closes and the
controller keeps asking. That is correct. Only judge forward driving with
`dry_run` off.
