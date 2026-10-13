# Troubleshooting & Fault Resolution Guide

Systematic fault isolation and diagnostic procedures for the Athena autonomy platform.

---

## Quick Symptom Reference

| Symptom | Diagnostic Tool | Primary Root Cause & Resolution Section |
|---|---|---|
| **Fix stuck at 3D / DGPS, never engages RTK** | `gps_diagnose --rtcm-host` | [RTCM Correction Stream Failure](#1-rtcm-correction-stream-failure) |
| **Fix flaps between RTK Float and RTK Fixed** | `gps_diagnose` | [RTK Float vs Fixed Instability](#2-rtk-float-vs-fixed-instability) |
| **`fix_type: 0` (No GPS Device)** | `gps_diagnose` | [DroneCAN GPS Wiring & CAN Bus](#3-dronecan-gps-wiring--hardware-issues) |
| **Flight controller disconnected or not found** | `ls /dev/serial/by-id` | [Serial Device Identification](#4-cube-orange-serial-identification) |
| **High MAVLink corruption (>5%) / low IMU rate** | `pixhawk_bridge` log | [MAVLink Serial Overrun](#5-mavlink-serial-link-corruption) |
| **Costmaps completely empty despite camera running** | `ros2 topic info` | [RealSense NEON Point Cloud Namespace](#6-costmaps-empty--camera-issues) |
| **Camera stalls silently (`Frames didn't arrive`)** | `dmesg` | [USB Autosuspend Power Lock](#6-costmaps-empty--camera-issues) |
| **Rover yaws in place instead of driving to goal** | `mount_check.py costmap` | [Lookahead & Footprint Obstacle Trap](#7-motion--controller-faults) |
| **Pico serial port held / motors not turning** | `fuser` | [Pico W Serial Contention](#7-motion--controller-faults) |

---

## 1. RTCM Correction Stream Failure

### Symptoms
- `pixhawk_bridge` logs: `no RTCM from base $BASE_STATION_IP:2101 for Xs - fix will fall back to standalone GPS`.
- Fix remains at `3D FIX` (`fix_type: 3`) despite clear sky view.

### Diagnostic Steps

1. **Verify Base Station Network Reachability:**
   ```bash
   nc -zv $BASE_STATION_IP 2101
   ```
   If connection is refused or times out:
   - Confirm the base station computer is powered on and STRSVR / `str2str` / NTRIP caster is running.
   - Ensure `$BASE_STATION_IP` environment variable is defined:
     ```bash
     echo "Target RTCM Host: $BASE_STATION_IP"
     ```

2. **Inspect Bridge RTCM Injection Telemetry:**
   Review the periodic `pixhawk_bridge` 10-second health log:
   ```
   RTK: 3D FIX, hAcc 0.820 m, RTCM last 10 s [1005:1 1077:10 1087:10], data age 0.4s, bad CRC 0
   ```
   - **`RTCM last 10 s [none]` or `data age > 10.0s`:** No RTCM frames received from the network.
   - **`bad CRC > 0`:** Network MTU fragmentation or byte corruption.

3. **Verify Flight Controller Forwarding Parameters (Cube Orange):**
   Ensure ArduPilot is configured to forward incoming RTCM packets to the DroneCAN bus:
   - `GPS_INJECT_TO = 127` (relays RTCM packets to all GNSS receivers).
   - `GPS1_TYPE = 9` (DroneCAN driver).
   - `MAVLINK20 = 1` (MAVLink 2 packet framing required for RTCM injection).

---

## 2. RTK Float vs Fixed Instability

### Symptoms
- Fix type alternates between 5 (`RTK FLOAT`) and 6 (`RTK FIXED`).
- Horizontal accuracy `h_acc` hovers between 0.15 m and 0.40 m instead of dropping $<0.03\text{ m}$.

### Root Causes & Solutions

| Cause | Mechanism | Resolution |
|---|---|---|
| **Multipath & Occlusion** | Concrete walls, metallic structures, or tree foliage reflect carrier-phase signals. | Move the rover into an open area with an unobstructed horizon ($\ge 15^\circ$ elevation angle). |
| **Low Satellite Count** | Fewer than 16 common satellites tracked between rover and base. | Inspect satellite count with `gps_diagnose.py`. Verify base station tracks GPS + GLONASS + Galileo + BeiDou. |
| **Baseline Distance** | Rover is $>15\text{ km}$ from the base station antenna. | Ionospheric divergence prevents integer ambiguity resolution. Shorten baseline distance. |
| **Antenna Cable Noise** | Loose SMA / u.FL connector or pinched coaxial line. | Inspect RF cable and ensure antenna ground plane is securely mounted. |

---

## 3. DroneCAN GPS Wiring & Hardware Issues

### Symptoms
- `gps_diagnose.py` reports: `best fix_type : 0 - NO GPS (receiver not reporting)`.
- `pixhawk_bridge` reports `fix_type=0, sats=255`.

### Isolation Procedure

```
[DroneCAN GPS] === CAN Bus (120 Ohm Termination) ===> [Cube Orange CAN1]
```

1. **Verify Physical Bus Connections:**
   - The DroneCAN RTK GNSS module must connect to the **CAN1** port on the Cube Orange carrier board (or CAN2 if configured).
   - Verify CAN bus termination resistors: exactly **120 $\Omega$** termination at each end of the CAN bus (~60 $\Omega$ across CAN_H and CAN_L with power off).
2. **Verify ArduPilot CAN Driver Parameters:**
   Connect via Mission Planner or MAVProxy:
   - `CAN_P1_DRIVER = 1` (Enables CAN Port 1).
   - `CAN_D1_PROTOCOL = 1` (DroneCAN protocol enabled on Port 1).
   - `GPS1_TYPE = 9` (DroneCAN GNSS).
3. **Electrical Intermittent Faults:**
   If `gps_diagnose` reports `transitions >= 2` between fix 0 and non-zero:
   - Reseat the 4-pin JST-GH CAN connector.
   - Check the 5V power supply rail on the carrier board.

---

## 4. Cube Orange Serial Identification

### Symptoms
- `pixhawk_bridge` fails with: `No such file or directory: /dev/serial/by-id/...`
- Serial link maps to the wrong USB device.

### Diagnostic Procedure

List all connected USB serial devices:

```bash
ls -l /dev/serial/by-id/
```

*Expected output:*
```
usb-Hex_ProfiCNC_CubeOrange_XXXXXX-if00 -> ../../ttyACM0
usb-Raspberry_Pi_Pico_XXXXXX-if00      -> ../../ttyACM1
```

> [!IMPORTANT]
> `pixhawk_bridge.py` auto-detects ports by matching substrings in `/dev/serial/by-id/`:
> `('ArduPilot', 'Ardu', 'CubeOrange', 'Cube', 'Pixhawk', 'PX4')`.
> Never hardcode `/dev/ttyACM0` in launch files.

If the device does not enumerate in `/dev/serial/by-id/`:
```bash
dmesg | tail -n 25
lsusb | grep -i "hex\|proficnc\|26ac"
```
If absent from `lsusb`, replace the USB-C telemetry cable and ensure the Jetson USB port provides adequate power.

---

## 5. MAVLink Serial Link Corruption

### Symptoms
- `pixhawk_bridge` logs: `link corruption: X%` where $X > 5\%$.
- Sensor update rates drop ($<30\text{ Hz}$ on IMU).

### Technical Cause & Fix

At standard 115200 baud, high-rate IMU (50 Hz), GPS (5 Hz), and RTCM injection saturate the serial link, resulting in 33% packet corruption.
- **Enforce 921600 Baud:**
  Both `pixhawk_bridge` and ArduPilot serial parameters (`SERIALx_BAUD=921`) must operate at **921600 baud**.
- **Unused Stream Suppression:**
  `pixhawk_bridge` automatically disables unused high-bandwidth MAVLink streams (`ODOMETRY`, `LOCAL_POSITION_NED`, `ATTITUDE_QUATERNION`) via `MAV_CMD_SET_MESSAGE_INTERVAL` to keep serial bandwidth utilization low.

---

## 6. Costmaps Empty & Camera Issues

### 1. Jetson NEON Point Cloud Parameter Namespace
On NVIDIA Jetson platforms, `librealsense` utilizes NEON-accelerated point cloud processing blocks named **`pointcloud__neon_`**, whereas standard x86 systems use `pointcloud`.
- If only `pointcloud.enable: True` is configured, point cloud generation fails **silently**.
- `sensors.launch.py` explicitly enables both namespaces:
  ```python
  'pointcloud.enable': True,
  'pointcloud__neon_.enable': True,
  'pointcloud__neon_.ordered_pc': False,
  ```

### 2. Camera Autosuspend Disconnection
If the RealSense driver logs `Frames didn't arrived within 5 seconds`:
Recover the USB bus immediately without rebooting:
```bash
sudo sh -c 'echo 2-1.3 > /sys/bus/usb/drivers/usb/unbind'
sudo sh -c 'echo 2-1.3 > /sys/bus/usb/drivers/usb/bind'
```

---

## 7. Motion & Controller Faults

### 1. Lookahead Trap (Rover Yaws in Place)
- **Symptom:** When a goal is dispatched, the rover spins left/right in place instead of advancing along the path.
- **Cause:** Lookahead distance set too small or scaled with velocity ($v_x = 0$ at start $\to$ carrot lands inside 0.65 m chassis footprint).
- **Check:** Ensure `config/nav2_params.yaml` has `use_velocity_scaled_lookahead_dist: false` and `lookahead_dist: 1.0`.

### 2. Pico W Serial Port Contention
- **Symptom:** `motor_bridge` outputs: `could not open serial port ... - running dry`.
- **Cause:** Another script holds the serial port (e.g. `calibrate`, `wiring_check`, or another bridge instance).
- **Resolution:**
  ```bash
  # Identify PID holding the Pico serial link
  fuser -v /dev/serial/by-id/usb-Raspberry_Pi_Pico*
  # Terminate contending process
  kill -9 <PID>
  ```
