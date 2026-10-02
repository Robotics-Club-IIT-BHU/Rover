# Calibration and motor tests

`ros2 run athena_drive calibrate` is the one tool for testing motors and
saving what it measures. Run it once per hardware change, not per drive.
Wheels off the ground for everything except stage 4. Stop the stack first (the
Pico's serial port is exclusive). Back to the [package README](../README.md).

## The menu

`calibrate` opens a menu. Keys also work in `-p stages:=...`, which skips the menu:

| Key | Does |
|---|---|
| `m` | **Manual motor test**: pick one or more motors, a direction, confirm; only that runs. Then repeat, reverse or pick again |
| `d` | Stage 1 direction check, only for the motors you pick |
| `w` | **Wiring map**: Pico GPIO and header pin to motor driver channel (the [pin map](../README.md#pico-to-motor-driver-pin-map)) |
| `s` | Board status and invert flags, read from the Pico |
| `1`-`5` | The five stages below, each on its own |
| `a` | Stages 1-5 in order |
| `q` | Quit |

```bash
ros2 run athena_drive calibrate                                    # menu
ros2 run athena_drive calibrate --ros-args -p stages:=m            # straight to the manual test
ros2 run athena_drive calibrate --ros-args -p stages:=3,4          # these stages, no menu
ros2 run athena_drive calibrate --ros-args -p stages:=all          # stages 1-5
```

Other parameters: `test_pwm` (default 120, the PWM used by the tests), `port`
(default: auto-detect the Pico), `baud` (115200).

**When results are saved.** In the menu, as soon as each stage finishes. With
`stages:=...`, once when the whole run ends (Ctrl-C mid-run saves nothing).

### `m`: manual motor test

- Pick motors by number (`3`, `1,4`), name (`fl ml rl fr mr rr`) or `left` / `right` / `all`.
- Direction is `f` or `b`, with an optional PWM (`f 150`).
- It prints the plan, including each motor's PWM and DIR wires, and asks to
  confirm. Each run auto-stops after ~1.5 s. After a run: Enter repeats, `r`
  reverses, `n` picks new motors, `q` returns to the menu.
- Several motors run **together** through the firmware's `G <mask> <pwm>`
  command (bit 0 = motor 1). `G` is compiled in the current firmware source
  but **not yet flashed to the Pico**. `calibrate` asks the board (via its help
  text) and, if it has no `G`, runs the chosen motors one after another and says
  so before you confirm. Reflash `athena_drive_fw` to run them together
  ([FIRMWARE.md](FIRMWARE.md#flashing)).

## The five stages

| Stage | Does | Saved to |
|---|---|---|
| 1 motor directions | runs each motor forward **then reverse**, ~1.5 s each; you answer what the wheel did | **Pico flash** (invert flags) |
| 2 side check | spins left; "counter-clockwise from above?" | nothing; tells you the fix |
| 3 PWM deadband | steps PWM 10, 20 ... 140, asks "did all six turn?", stops at the first yes | `min_pwm` in the config file |
| 4 top speed | **drives on the floor** at full PWM for N s (default 3); you enter the measured distance | `max_wheel_speed` |
| 5 track width | you enter the distance between left and right wheel centre lines | `track_width` |

Stage 4 needs clear floor and a mark at the front edge. The ~0.3 s firmware
ramp makes short runs read a little low. Stage 2 failure fix: swap the left
and right driver connections, **or** flip all six invert flags (`I 1 1` ...
`I 6 1`), never both; then redo stage 1.

### Stage 1 verdicts

Both directions are tested and each motor is classified from the two answers:

| "rolled forward?" | "rolled backward?" | Verdict | What happens |
|---|---|---|---|
| yes | yes | ok | next motor |
| no | no, but it did move | flipped | invert flag toggled on the Pico, saved, motor re-tested |
| no | no, and it never moved | **dead** | power, motor leads or driver channel |
| exactly one yes | | **stuck** | ran the same way both times: DIR line or H-bridge fault |

Only `flipped` is fixable by calibration. The firmware applies invert as
`spwm = -spwm`, a symmetric sign flip: it can make a motor run the wrong way
in both directions but never the same way in both. So `stuck` is reported as
a hardware fault, not handed a flag that cannot help. Faults are repeated in a
block at the end so they do not scroll past. Stage 1 starts from the board's real
flags, so a motor already flipped toggles back rather than being re-sent the
same value. A motor that cannot reverse passes a forward-only test and then
fails the moment Nav2 asks for a turn, which is why both directions are tested.

## Where the settings live

| What | Where | Why |
|---|---|---|
| Motor directions (invert flags) | **the Pico's flash** | Direction is a property of the wiring, so it belongs to the board: swap the Jetson, reinstall ROS, and the motors still spin the right way. It must be there anyway: the bridge sends one PWM value per *side*, so it cannot flip a single wheel |
| `min_pwm`, `max_wheel_speed`, `track_width` | `~/.config/athena_drive/params.yaml` | Loaded by `motor_bridge` at startup, so calibrated values apply to every launch with no arguments. Outside the package so a rebuild cannot wipe it |

Precedence: **explicit argument > saved calibration > built-in estimate**, so
a one-off experiment never silently becomes permanent:

```bash
ros2 launch athena_drive drive.launch.py max_pwm:=120    # cautious, this run only
```

## What is in the file today

```yaml
min_pwm: 20
track_width: 0.85
```

`max_wheel_speed` and `max_pwm` are absent, so the rover runs on the built-in
estimates of **0.7 m/s and 255**. `max_wheel_speed` scales every velocity Nav2
commands, so commanded speeds and distances are only as good as that guess:
run stage 4 on the floor (`calibrate --ros-args -p stages:=4`). A sanity check
on 0.7 m/s: wheels about 20 cm across give 0.628 m of circumference, so about
67 wheel RPM. `max_pwm` has no measuring tool; 255 is the default, lower it
only deliberately.

`motor_bridge` warns per missing key at startup (it reports each key
separately, so an incomplete file does not look calibrated):

```
loaded calibration from /home/robo/.config/athena_drive/params.yaml: {'min_pwm': 20, 'track_width': 0.85}
motor_bridge up: track=0.85 m, max_wheel_speed=0.7 m/s, pwm=[20,255]
[WARN] UNCALIBRATED, using built-in estimates for: max_wheel_speed, max_pwm. ...
```

No `UNCALIBRATED` line means all four values are either saved or passed
explicitly. A `max_wheel_speed` of exactly `0.7` is the fallback, not a measurement.

## Single-purpose tools

`calibrate` covers all of this. These re-check one thing without the wizard:

```bash
ros2 run athena_drive wiring_check                           # each motor forward in turn
ros2 run athena_drive wiring_check --ros-args -p mode:=sides # left side, then right side
ros2 run athena_drive wiring_check --ros-args -p mode:=spin  # forward, then spin left
ros2 run athena_drive calibrate_speed --ros-args -p mode:=deadband
ros2 run athena_drive calibrate_speed --ros-args -p mode:=topspeed -p seconds:=3.0
```

- `wiring_check` options: `-p motor:=3` (one motor), `-p pwm:=150` (default
  120), `-p pause:=5.0` (default 3 s between motors). It runs on a schedule and
  reads no answers; `calibrate` is the interactive one. See
  [TESTING.md](TESTING.md#1-each-motor-turns-the-right-way) for what to look for.
- `calibrate_speed` `deadband` sweeps PWM 10 to 120 in tens, wheels off the
  ground. `topspeed` **drives the rover on the floor** at full PWM (`-p pwm:=`
  to change) for `seconds`, then tells you to divide your measured distance by
  the time.
- These print results and **do not save them**. Only `calibrate` writes
  `params.yaml`. Either use `calibrate`, or hand-edit the file (the bridge reads
  it at the next launch):

```yaml
min_pwm: 20
track_width: 0.85
max_wheel_speed: 0.62      # add the number you measured
```
