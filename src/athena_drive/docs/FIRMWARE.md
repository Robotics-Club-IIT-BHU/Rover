# Pico W Motor Firmware Reference

Firmware implementation, serial protocol commands, and flashing procedures for `athena_drive_fw` running on the Raspberry Pi Pico W.

Source: `firmware/athena_drive_fw/athena_drive_fw.ino`  
Serial Configuration: 115200 baud, 8N1, newline-terminated (`\n`).

---

## 1. Serial Protocol Commands

| Command | Arguments | Functional Description |
|---|---|---|
| `V` | `<left> <right>` | Variable skid-steer drive command. Signed PWM ($-255$ to $+255$) per side (+ is forward). Active drive command used by `motor_bridge`. |
| `M` | `<motor> <pwm>` | Drives an individual motor ($1$ to $6$). Automatically halts after 1.5 s. Used for isolated wiring checks. |
| `G` | `<mask> <pwm>` | Drives a bitmask combination of motors simultaneously ($1$ to $63$, where bit 0 = Motor 1). Halts after 1.5 s. |
| `I` | `[none]` | Prints the 6-character hardware invert string (e.g. `inv=000000`). |
| `I` | `<motor> <0\|1>` | Configures motor polarity ($0$ = normal, $1$ = inverted). Immediately commits to onboard EEPROM/flash. |
| `?` | `[none]` | Queries board telemetry and watchdog status without affecting motor state. |
| `W` | `[none]` | Commits active configuration to onboard flash memory. |
| `H` | `[none]` | Prints command help text. |

### Firmware Status Telemetry String

Sending `?` returns the status line:

```
L=0/0 R=0/0 spd=200 pwmHz=20000 wdt=ok inv=000000
```

- **`L` / `R`:** Target vs active slewed PWM for left and right channels.
- **`pwmHz=20000`:** Active 20 kHz ultrasonic PWM frequency.
- **`wdt=ok`:** Hardware watchdog status (`ok` or `tripped`).
- **`inv`:** Direction invert flags for motors 1 through 6 (`1=FL, 2=ML, 3=RL, 4=FR, 5=MR, 6=RR`).

> [!NOTE]
> The `V` command controls wheels **per side** (`applySide()`). All 3 motors on a side receive the identical PWM command. Therefore, if 5 wheels turn and 1 does not, the failure is strictly hardware (motor leads, driver board channel, or power rail), not software.

---

## 2. Built-In Firmware Protections

- **20 kHz Ultrasonic PWM:**
  The RP2040 hardware timer generates 20 kHz PWM. This eliminates audible coil whine and ensures continuous motor winding current for smooth low-speed torque.
- **Hardware Slew Rate Limiter:**
  PWM duty cycle ramps smoothly over ~0.3 s from 0 to full scale. This protects mechanical gearboxes from shock loading and wheel spin.
- **500 ms Hardware Watchdog:**
  If no valid serial command is received for 500 ms, the firmware automatically zeroes PWM output across all channels.

---

## 3. Firmware Flashing Procedures

### Method 1: Serial Upload (via Arduino CLI)

Use this method when the Pico W enumerates normally on `/dev/serial/by-id/`:

```bash
# Verify the Pico W serial link
ls -l /dev/serial/by-id/usb-Raspberry_Pi_Pico*

# Compile and upload firmware
cd ~/athena/src/athena_drive
~/bin/arduino-cli compile --fqbn rp2040:rp2040:rpipicow \
  --port $(readlink -f /dev/serial/by-id/usb-Raspberry_Pi_Pico*) \
  --upload firmware/athena_drive_fw
```

### Method 2: USB Mass Storage Bootloader (UF2)

Use this method if the serial port fails to enumerate or after flashing corrupt firmware:

1. Disconnect the Pico W USB cable.
2. Hold down the white **BOOTSEL** button on the Pico board while plugging the USB cable into the Jetson.
3. The board mounts as a mass storage drive at `/media/$USER/RPI-RP2`.
4. Compile the UF2 binary and copy it to the mounted volume:

```bash
cd ~/athena/src/athena_drive
~/bin/arduino-cli compile --fqbn rp2040:rp2040:rpipicow \
  --output-dir ~/athena/firmware_build firmware/athena_drive_fw

cp ~/athena/firmware_build/athena_drive_fw.ino.uf2 /media/$USER/RPI-RP2/
```

5. The Pico W automatically unmounts, flashes the firmware, and reboots into normal CDC serial mode.
6. Verify operation:
   ```bash
   ros2 run athena_drive calibrate --ros-args -p stages:=s
   ```
