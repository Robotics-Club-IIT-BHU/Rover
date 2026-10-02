# Pico firmware

`firmware/athena_drive_fw/athena_drive_fw.ino`, running on a Raspberry Pi
Pico W. Serial protocol: 115200 baud, newline-terminated. For which GPIO drives
which motor see the [pin map](../README.md#pico-to-motor-driver-pin-map). Back
to the [package README](../README.md).

## Commands

| Command | Does |
|---|---|
| `V <left> <right>` | variable drive, signed PWM -255..255 **per side**, + = forward. The only drive command the bridge uses |
| `M <n> <pwm>` | ONE motor, n = 1..6; auto-stops after 1.5 s |
| `G <mask> <pwm>` | several motors at once (bit 0 = motor 1, mask 1..63, e.g. `G 5 120` = motors 1 and 3); same auto-stop. Used by the `calibrate` menu. **In the source, not yet flashed to the Pico** (see below) |
| `F` `B` `L` `R` `S` | legacy fixed-speed forward / back / left / right / stop |
| `P <0-255>` | speed used by F/B/L/R (saved to flash) |
| `I` | print the six invert flags |
| `I <n> <0\|1>` | flip motor n's forward direction, saved to flash immediately |
| `W` | save config to flash |
| `?` | status line |
| `H` | help |

`I` is how `calibrate` stage 1 writes a direction. `?` is what `motor_bridge`
uses as a liveness ping: a pure read that sets no target and moves nothing.
Status line:

```
L=0/0 R=0/0 spd=200 pwmHz=20000 wdt=ok inv=000000
```

`inv` is the six invert flags in motor order (1=FL 2=ML 3=RL 4=FR 5=MR 6=RR).
`motor_bridge` warns at startup if any is non-zero: a wrong one is invisible at
runtime and shows up only as a wheel fighting the other five.

**`V` carries one value per SIDE.** `applySide()` writes that value to all three
motors on the side. There is no per-motor speed and no per-motor enable; the
only per-motor state is `invert[6]`, which flips direction and nothing else.
**So one dead wheel is never a software fault.** If five wheels turn and one
does not, it is the motor, its leads, its driver channel or its power. `M <n>
<pwm>` proves it one motor at a time (`wiring_check` wraps it).

## Built-in behaviour

- **20 kHz PWM.** The RP2040 core defaults to ~1 kHz, which is audible and gives
  poor low-speed torque (each pulse is long against the motor's electrical time
  constant, so current collapses between pulses). 20 kHz is inaudible and keeps
  winding current continuous, so slow speeds are smooth.
- **Slew limiting.** Commanded PWM ramps instead of stepping (about 0.3 s from 0
  to full). It protects the gearboxes, stops the wheels breaking traction on every
  change and keeps motion smooth enough for visual odometry. Separate from the
  bridge's [velocity profile](MOTOR_BRIDGE.md#velocity-profile); keep both.
- **Watchdog.** No valid command for 500 ms and the motors stop. The bridge
  streams at 20 Hz, so a crashed node or unplugged cable halts the rover in
  under half a second. A running `M`/`G` test is deliberately exempt, or the
  watchdog would cut every wiring check short at 500 ms instead of 1.5 s.

## Flashing

**The `G` command is compiled in the current source but the Pico has not been
reflashed since it was added.** Until it is, `calibrate` runs multi-motor tests
one motor after another. The prebuilt image
`~/athena/firmware_build/athena_drive_fw.ino.uf2` is dated 2026-08-30 and does
not contain `G`: rebuild it before using it.

Two routes. The first needs the board to enumerate as a serial port; the second
works when it does not (the case you will hit after flashing something broken).

**Over serial**, from the package directory (the `.ino` path is relative):

```bash
cd ~/athena/src/athena_drive
ls /dev/serial/by-id/        # check the Pico is listed first
arduino --upload --board rp2040:rp2040:rpipicow \
  --port $(readlink -f /dev/serial/by-id/usb-Raspberry_Pi_Pico*) \
  firmware/athena_drive_fw/athena_drive_fw.ino
```

The rp2040 core (5.5.1) is installed for both `/usr/bin/arduino` and
`~/bin/arduino-cli`. If the glob matches nothing, `$(...)` expands to empty and
`--port` silently gets no argument, hence the `ls` first. Stop `motor_bridge`
first (the port is exclusive).

**Over USB mass storage**, needing no serial port and no working firmware: hold
BOOTSEL while plugging the board in; it mounts as a drive. Copy a UF2 onto it
and it reboots into the new firmware when the copy finishes:

```bash
cd ~/athena/src/athena_drive
~/bin/arduino-cli compile --fqbn rp2040:rp2040:rpipicow \
  --output-dir ~/athena/firmware_build firmware/athena_drive_fw     # rebuild the image
cp ~/athena/firmware_build/athena_drive_fw.ino.uf2 /media/$USER/RPI-RP2/
```

After flashing, run `ros2 run athena_drive calibrate` and press `s` to confirm
the board answers and to check the invert flags (`inv`) still match your
wiring; they are stored in the Pico's flash, and the config falls back to all
zeros if it is not found.
