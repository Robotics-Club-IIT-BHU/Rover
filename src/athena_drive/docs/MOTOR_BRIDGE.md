# motor_bridge: parameters, kinematics, velocity profile

`motor_bridge` turns `/cmd_vel` into `V <left> <right>` for the Pico. It is the
only node that runs during navigation. Back to the [package README](../README.md).

## Parameters

`drive.launch.py` passes `-1` for the four calibrated numbers, meaning "use the
saved value" ([precedence](CALIBRATION.md#where-the-settings-live)). The column
below is the **built-in estimate**, used only when nothing is saved and nothing
is passed:

| Parameter | Estimate | Meaning |
|---|---|---|
| `port` | `''` | empty = auto-detect the Pico by USB id |
| `baud` | 115200 | matches the firmware |
| `send_rate` | 20.0 | Hz; the heartbeat the firmware watchdog counts on |
| `watchdog_timeout` | 0.5 | s of stale `/cmd_vel` before the target is zeroed |
| `track_width` | 0.40 | m between left and right wheel centres (saved: 0.85) |
| `max_wheel_speed` | 0.7 | m/s at full PWM: **unmeasured** |
| `min_pwm` | 40 | static-friction deadband (saved: 20) |
| `max_pwm` | 255 | cap; lower it for cautious indoor testing. **Unmeasured** |
| `invert_left` / `invert_right` | false | flip a whole side in software |
| `profile_enabled` | true | jerk-limited velocity shaping |
| `max_accel` / `max_jerk` | 0.5 / 1.0 | m/s^2, m/s^3 |
| `max_ang_accel` / `max_ang_jerk` | 1.5 / 3.0 | rad/s^2, rad/s^3 |
| `dry_run` | false | log commands instead of sending them |

`drive.launch.py` exposes ten as launch arguments: `dry_run`, `track_width`,
`max_wheel_speed`, `min_pwm`, `max_pwm`, `profile_enabled`, `max_accel`,
`max_jerk`, `max_ang_accel`, `max_ang_jerk`. The rest are `--ros-args -p` only.
The bridge also publishes the last firmware status on `~/firmware`.

## Serial port and firmware handshake

- **Never hard-code `/dev/ttyACM<n>`.** Numbers are assigned in USB
  enumeration order and swap between boots; the Pixhawk and the Pico have each
  held `ttyACM0` on different days. `port: ''` globs
  `/dev/serial/by-id/usb-Raspberry_Pi_Pico*`, stable per board. Sending motor
  commands to a flight controller is the failure this prevents.
- Opening a CDC port succeeds against a wedged board, one running other
  firmware, or one that never finished booting. So the bridge sends `?` and
  waits 2 s for a reply before trusting the link, re-pings every 2 s, and
  complains if 6 s pass with no answer. Without that it logged a cheerful
  `serial open` and streamed `V 0 0` into the void.
- `FIRMWARE NOT RESPONDING`: the port is right and the board is not talking.
  Replug it, or reflash ([FIRMWARE.md](FIRMWARE.md#flashing)).
- `could not open ... - running dry`: another process holds the port, or the
  Pico is absent. `fuser -v /dev/ttyACM*` names the holder. If the link drops
  later, the bridge retries the open every 2 s.
- With `inv=` flags non-zero in the firmware status, the bridge warns at
  startup (see [FIRMWARE.md](FIRMWARE.md#commands)).

## Kinematics

Skid steer:

```
v_left  = v - w*W/2
v_right = v + w*W/2
```

**Saturation scales both sides together.** If one side exceeds
`max_wheel_speed`, both are scaled by the same factor. Clipping them
independently would change their ratio, which is the turn radius, so the rover
would drive a different arc than Nav2 asked for, worst exactly when turning hardest.

**Deadband is added, not multiplied.** Any non-zero wheel speed maps into
`[min_pwm, max_pwm]` rather than `[0, max_pwm]`, so small corrections clear
static friction instead of being swallowed by it.

## Velocity profile

A step in commanded velocity is a step in torque: a **jerk**. The wheels
break traction, the chassis rocks, and the camera visual odometry depends on
gets shaken just when the rover is working out how it moved. Limiting
acceleration alone still lets acceleration change instantly (the lurch on
starting, the nod on stopping), so the bridge bounds both:

| | Parameter | Default |
|---|---|---|
| acceleration | `max_accel` | 0.5 m/s^2 |
| jerk | `max_jerk` | 1.0 m/s^3 |
| angular acceleration | `max_ang_accel` | 1.5 rad/s^2 |
| angular jerk | `max_ang_jerk` | 3.0 rad/s^3 |

The result is an S-curve. Measured for a step command to 0.4 m/s:

```
   t(s)    cmd   shaped          PWM
   0.00   0.40    0.003   V  21  21     ease in
   0.30   0.40    0.070   V  44  44
   0.60   0.40    0.212   V  91  91     full acceleration
   0.90   0.40    0.348   V 137 137
   1.20   0.40    0.400   V 154 154     eased out, settled
```

About 1.2 s to reach speed and the same to stop, with no step in the PWM trace.
The PWM column used `min_pwm` 20 and `max_wheel_speed` 0.7, that is `pwm = 20 +
(v / 0.7) x 235`; other values map the same velocities elsewhere.

This is separate from the firmware's slew limit: the firmware ramps raw PWM as a
safety net that works even if the host misbehaves; this shapes *velocity*, in the
units Nav2 reasons in, before the kinematics. Keep both.

Tuning: lower `max_accel` if the wheels slip on start; lower `max_jerk` if the
chassis rocks or visual odometry loses tracking when you set off; raise both if
the rover feels sluggish to Nav2. Compare with it off:

```bash
ros2 launch athena_drive drive.launch.py profile_enabled:=false
```

`velocity_profile.py` is self-testing and imports nothing from ROS, so it runs
in any shell. It prints the profile, `peak |accel|`, `peak |jerk|` and
`LIMITS RESPECTED`, and exits non-zero if either bound is violated:

```bash
cd ~/athena && python3 src/athena_drive/athena_drive/velocity_profile.py
```

## Known limitations

- **Open loop.** No encoders on this board, so commanded speed is not verified.
  Skid-steer turns slip by design, so heading from the wheels alone would drift
  badly: the nav stack takes rotation from the gyro, never from the wheels.
- A previous Pico ran closed-loop PID firmware with encoders. Going back to
  that board would make wheel odometry possible, a real upgrade: it works in the
  dark and against blank walls, exactly where visual odometry struggles.
- `max_wheel_speed` and `max_pwm` are unmeasured estimates
  ([details](CALIBRATION.md#what-is-in-the-file-today)).
- The Pico must run the `V <left> <right>` firmware in `firmware/athena_drive_fw`.
  The bridge speaks only that protocol and has no legacy fallback (no
  `legacy_protocol` argument on `drive.launch.py` or `bringup`).
