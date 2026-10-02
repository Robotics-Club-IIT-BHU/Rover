# drive (deprecated)

**Do not use this package. Use [`athena_drive`](../athena_drive/README.md).**

`move.py` defaults `port` to a **hardcoded `/dev/ttyACM1`**. On this rover the
Pico and the Pixhawk swap `ttyACM` numbers between boots, and the autopilot lands
on `ttyACM1` about half the time, so this package can send PWM commands to a flight
controller. `athena_drive/motor_bridge` globs
`/dev/serial/by-id/usb-Raspberry_Pi_Pico*`, which is stable per board.
Nothing in the current stack runs this package: `bringup.launch.py` includes
`athena_drive/launch/drive.launch.py`. It remains only so older branches still build.

`athena_drive` also has what this lacks: the calibration wizard (this package's
parameters are not persisted), the jerk-limited velocity profile, the firmware
handshake and per-motor wiring checks.

If you must run it, pass the port explicitly and stop anything else holding the
Pico's serial port (including bringup's `motor_bridge`):

```bash
ros2 run drive move --ros-args -p port:=$(readlink -f /dev/serial/by-id/usb-Raspberry_Pi_Pico*)
ros2 run drive move --ros-args -p dry_run:=true            # log instead of driving
ros2 run drive move --ros-args -p legacy_protocol:=true    # Pico on the pre-v2 sketch (F/B/L/R/S)
ros2 run drive teleop                                      # arrows, q/z linear speed, e/c angular speed, space stop; releasing stops
```

Firmware here is `firmware/sketch_feb10a/`; flash `athena_drive/firmware/athena_drive_fw/`
instead (`~/Desktop/sketch_feb10a` is an older, diverged copy). `move` parameters
(`max_wheel_speed` 0.7, `track_width` 0.40, `min_pwm` 40, `invert_left`/`invert_right`)
have the same meanings as in `athena_drive`'s [`motor_bridge`](../athena_drive/docs/MOTOR_BRIDGE.md#parameters),
but there is no `max_pwm` here.
