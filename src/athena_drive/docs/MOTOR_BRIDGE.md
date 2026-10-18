# Motor Bridge Architecture & Kinematics

`motor_bridge` translates standard ROS 2 `/cmd_vel` velocity commands into low-level `V <left> <right>` serial commands for the Raspberry Pi Pico W firmware.

---

## 1. Parameters & Configuration

Parameters are loaded at startup from `~/.config/athena_drive/params.yaml` or launch arguments:

| Parameter | Default / Saved | Unit | Description |
|---|---|---|---|
| `port` | `''` (Auto-detect) | Path | Globs `/dev/serial/by-id/usb-Raspberry_Pi_Pico*` for deterministic USB binding. |
| `baud` | `115200` | Baud | Baud rate matching Pico W firmware UART. |
| `send_rate` | `20.0` | Hz | Telemetry transmission rate (serves as firmware watchdog heartbeat). |
| `watchdog_timeout` | `0.5` | s | Timeout threshold: zeroes targets if `/cmd_vel` goes stale. |
| `track_width` | `0.85` | m | Distance between left and right wheel centerline. |
| `max_wheel_speed` | `0.70` | m/s | Maximum forward linear wheel speed at PWM 255. |
| `min_pwm` | `20` | PWM (0–255) | Static friction deadband threshold. Non-zero speeds map into `[min_pwm, max_pwm]`. |
| `max_pwm` | `255` | PWM (0–255) | Upper PWM ceiling (reduce for low-speed bench testing). |
| `profile_enabled` | `true` | bool | Enables jerk-limited S-curve velocity shaping. |
| `max_accel` / `max_jerk` | `0.5` / `1.0` | m/s$^2$, m/s$^3$ | Linear acceleration and jerk limits. |
| `max_ang_accel` / `max_ang_jerk` | `1.5` / `3.0` | rad/s$^2$, rad/s$^3$ | Angular acceleration and jerk limits. |
| `dry_run` | `false` | bool | Logs serial commands without writing to USB. |

---

## 2. Serial Port & Liveness Handshake

- **Deterministic Device Binding:**
  Addressing is restricted to `/dev/serial/by-id/usb-Raspberry_Pi_Pico*`. Never use `/dev/ttyACM*`, which can swap with the Cube Orange across reboots.
- **Heartbeat & Liveness Handshake:**
  The bridge sends a `?` query every 2 seconds. If no response is received within 6 seconds, the bridge logs a warning and marks the connection dead.
- **Mutual Exclusion:**
  Only one host process may hold the serial connection. If another utility holds the port, the bridge reports `could not open ... - running dry` and retries every 2 seconds.

---

## 3. Kinematics & Wheel Scaling

### Skid-Steer Differential Model

For linear velocity $v$ and angular velocity $\omega$ with track width $W$:

$$v_{left} = v - \frac{\omega \cdot W}{2}, \quad v_{right} = v + \frac{\omega \cdot W}{2}$$

### Coordinated Saturation Scaling

If either wheel speed exceeds `max_wheel_speed`, **both** sides are scaled down proportionally by $\frac{v_{max}}{\max(|v_{left}|, |v_{right}|)}$.
This preserves the exact turn radius ratio commanded by Nav2 rather than distorting the robot's heading trajectory.

### Deadband Addition

Any non-zero speed maps into `[min_pwm, max_pwm]` rather than `[0, max_pwm]`:

$$\text{PWM} = \text{sign}(v_{wheel}) \cdot \left( \text{min\_pwm} + \frac{|v_{wheel}|}{\text{max\_wheel\_speed}} \cdot (\text{max\_pwm} - \text{min\_pwm}) \right)$$

---

## 4. Jerk-Limited S-Curve Profiling

Instantaneous changes in commanded acceleration induce mechanical jerk, rocking the chassis and degrading visual odometry tracking.

The profiler applies an S-curve profile bounding both acceleration and jerk:

```
Command Step (0.4 m/s) ──> Jerk Limit (1.0 m/s^3) ──> Accel Limit (0.5 m/s^2) ──> Smooth PWM Output
```

| Time (s) | Commanded ($v$) | Shaped ($v$) | Left/Right PWM | Stage |
|---|---|---|---|---|
| `0.00` | 0.40 m/s | 0.003 m/s | `V  21  21` | Initial smooth ramp |
| `0.30` | 0.40 m/s | 0.070 m/s | `V  44  44` | Accelerating |
| `0.60` | 0.40 m/s | 0.212 m/s | `V  91  91` | Peak acceleration |
| `0.90` | 0.40 m/s | 0.348 m/s | `V 137 137` | Tapering acceleration |
| `1.20` | 0.40 m/s | 0.400 m/s | `V 154 154` | Steady cruise |

> [!NOTE]
> This host-side profiler shapes velocities in Nav2 physical units. It operates alongside the firmware's hardware PWM slew rate (~0.3 s ramp) to provide redundant motor and gearbox protection.

---

## 5. Standalone Velocity Profile Verification

Run the host-side velocity profile test (requires no ROS runtime):

```bash
python3 /home/robo/athena/src/athena_drive/athena_drive/velocity_profile.py
```

*Expected output:* Confirms peak acceleration and jerk stay strictly within configured limits and exits with code 0.
