# Motor Calibration & Tuning Guide

Calibration procedures, manual motor diagnostics, and parameter persistence for `athena_drive`.

Run this calibration once following any hardware, wiring, or chassis modifications.
All motor testing except Stage 4 must be performed with the **wheels lifted off the ground**.

> [!IMPORTANT]
> The Raspberry Pi Pico W serial interface is strictly exclusive. Stop `bringup.launch.py` and `drive.launch.py` before launching calibration tools.

---

## 1. Interactive Calibration Menu (`calibrate`)

Launch the unified calibration wizard:

```bash
ros2 run athena_drive calibrate
```

### Interactive Menu Key Actions

| Key | Mode / Action | Description |
|---|---|---|
| `m` | **Manual Motor Test** | Test individual or grouped motors in forward/reverse with specified PWM. |
| `d` | **Direction Verification** | Verifies forward/reverse direction on chosen motor channels. |
| `w` | **Wiring Pin Map** | Displays the GPIO pin mapping from Pico W to motor driver boards. |
| `s` | **Firmware Status** | Reads active invert flags and watchdog state from Pico W flash. |
| `1`–`5` | **Individual Stages** | Executes one specific calibration stage (see below). |
| `a` | **Full Sequence** | Runs Stages 1 through 5 sequentially. |
| `q` | **Quit** | Exits the calibration utility. |

### Direct Command-Line Execution (Skip Menu)

```bash
# Direct manual motor test
ros2 run athena_drive calibrate --ros-args -p stages:=m

# Run specific calibration stages (e.g. Stage 3 deadband and Stage 4 top speed)
ros2 run athena_drive calibrate --ros-args -p stages:=3,4

# Run all five stages sequentially
ros2 run athena_drive calibrate --ros-args -p stages:=all
```

---

## 2. The Five Calibration Stages

| Stage | Procedure | What It Measures | Target Storage Location |
|---|---|---|---|
| **1. Motor Directions** | Runs each motor forward then reverse (~1.5 s each); prompts for wheel direction. | Direction polarity per motor channel. | **Pico W Flash** (Hardware Invert Flags) |
| **2. Side Polarity Check** | Commands a left spin; confirms counter-clockwise rotation from above. | Left vs Right driver channel wiring. | Physical driver wiring check |
| **3. PWM Deadband** | Steps PWM from 10 to 140 in increments of 10 until all 6 wheels spin. | Minimum PWM to overcome static friction. | `min_pwm` in `params.yaml` |
| **4. Ground Top Speed** | Drives on floor at full PWM for $N$ seconds; prompts for measured distance. | True maximum forward speed (m/s). | `max_wheel_speed` in `params.yaml` |
| **5. Track Width** | Prompts for measured distance between left and right wheel centerline. | Effective kinematic track width (m). | `track_width` in `params.yaml` |

### Stage 1: Direction Verdicts & Fault Isolation

Each motor is tested forward and backward to detect wiring or H-bridge faults:

| Forward Test | Reverse Test | Verdict | Action Taken |
|---|---|---|---|
| Rolled Forward | Rolled Backward | **OK** | Proceeds to next motor channel. |
| Rolled Backward | Rolled Forward | **Flipped** | Firmware invert flag toggled on Pico W and saved to flash. |
| No Movement | No Movement | **Dead** | Hardware fault: check 12V motor power, driver channel, or wiring. |
| Movement in 1 Direction Only | Movement in 1 Direction Only | **Stuck** | Hardware fault: defective DIR control line or damaged H-bridge MOSFET. |

> [!NOTE]
> The firmware applies direction inversion as $PWM = -PWM$. A motor that rolls the wrong way in both directions is inverted; a motor that only moves in one direction has a hardware failure that software cannot fix.

---

## 3. Configuration Hierarchy & Persistence

Settings are resolved using a strict three-tier precedence:
$$\text{Explicit Launch Argument} > \text{Saved } \texttt{params.yaml} > \text{Built-in Estimate}$$

| Setting | Storage Location | Default Fallback | Purpose |
|---|---|---|---|
| **Invert Flags (`inv`)** | **Pico W Onboard Flash** | `000000` (all un-inverted) | Hardware property of the chassis wiring. Persists across host re-installs. |
| **`min_pwm`** | `~/.config/athena_drive/params.yaml` | `40` (typical: `20`) | Static friction deadband threshold. |
| **`max_wheel_speed`** | `~/.config/athena_drive/params.yaml` | `0.7` m/s | Maximum physical wheel speed at PWM 255. |
| **`track_width`** | `~/.config/athena_drive/params.yaml` | `0.40` m (actual: `0.85` m) | Kinematic track width between wheel centers. |
| **`max_pwm`** | `~/.config/athena_drive/params.yaml` | `255` | Maximum allowable PWM duty cycle. |

### Current Calibrated Configuration (`~/.config/athena_drive/params.yaml`)

```yaml
min_pwm: 20
track_width: 0.85
max_wheel_speed: 0.62   # Example measured value from Stage 4
```

---

## 4. Standalone Diagnostic Utilities

For targeted testing without entering the interactive wizard:

```bash
# Spin each motor forward sequentially (isolated test)
ros2 run athena_drive wiring_check

# Test left side together, then right side together
ros2 run athena_drive wiring_check --ros-args -p mode:=sides

# Forward test followed by left spin test
ros2 run athena_drive wiring_check --ros-args -p mode:=spin

# Measure static friction deadband without saving
ros2 run athena_drive calibrate_speed --ros-args -p mode:=deadband

# Floor speed test: 3 seconds at full PWM
ros2 run athena_drive calibrate_speed --ros-args -p mode:=topspeed -p seconds:=3.0
```
