# Athena Gazebo Simulation Plan — IRC 2027

Plan for validating the Athena autonomy stack in simulation before the IRC 2027
finals (Feb 2–5, 2027, Dayananda Sagar University, Bengaluru). Scope agreed with
the team: **no arm on the rover yet; the full real sensor stack is simulated.**

Sources: IRC 2027 Rulebook (attached, extracted), the Athena repo
(`feature/rtk-integration` @ 715a9e5), SPROS/IRC public pages, Fictionlab's ERC
Mars-yard simulation (open-source reference).

---

## 1. Why we are doing this (tied to IRC 2027)

The three IRC 2027 field missions and what our stack must do in each:

| Mission | What the rulebook demands | What Athena must prove |
|---|---|---|
| **RADO** — Reconnaissance & Autonomous Delivery | 10 min recon: locate scattered objects (tools, containers, rocks ≤5 kg, graspable ≤7 cm), photograph each + log GPS coords. Then 15 min: pick up and deliver objects to designated GPS locations; **delivery must be autonomous for full points** (50% if teleoperated). Terrain: soft sand → rough stony, boulders, vertical drops, dunes, slopes. | RTK-GPS waypoint autonomy end-to-end (this is what the `feature/rtk-integration` branch exists for), object spotting on camera, obstacle avoidance at speed. |
| **ABEx** — Astrobiology Expedition | Rover as mobile science lab: panoramas (min 1:3 aspect, cardinal directions, GPS/elevation), close-up photos with scale, ≥10 g sample from ≥10 cm depth, onboard soil/atmosphere tests, cache a sealed sample at a designated drop point (retrieved later during RADO). | Precise *positioning* at science sites, panorama workflow, GPS-tagged photo logging. (Sample collection hardware is out of sim scope; we simulate reaching and documenting sites.) |
| **IDMO** — Instrument Deployment & Maintenance | Short traverse to a mock instrument panel (≤1.8 m tall): drawer, buttons, switches, knobs, joystick, latches, 3-pin plug; read codes on/near the panel. Needs an arm. | Arm is **not** in sim scope now — but we simulate driving to the panel and framing it with the camera, so arm work later starts from a solved positioning problem. |

Mission rules that the simulator should also rehearse (operators + code):

- 25 min mission cap, 8 min setup; **abort if no points in first 7 min**; full time
  only if >30% points in first 15 min; bonus tasks only after 15 min.
- **Red = contaminated no-go zone**: entry costs 10% of mission points, per violation.
- Max 2 interventions across the finals, 30% penalty each, restart at start line.
- Max 500 m from base station; WGS-84 decimal degrees; base station operators have
  **no direct line of sight** to the field — camera + telemetry only.

Hard deadlines that gate this work (from the rulebook):

| Date | Event | Sim relevance |
|---|---|---|
| **Nov 5, 2026** | Registration closes (early bird was Oct 15) | Register NOW, independent of sim |
| **Nov 25, 2026** | SDDR report + 3-min video (16 pages) | Phases 0–2 done by then = real "prototype testing strategy with verification" content for the SDDR |
| Dec 5, 2026 | Comms standards declaration | Not sim-related |
| **Feb 2–5, 2027** | Finals | Everything validated by mid-January |

### What the real competition terrain looks like

The finals run in **"Sproscape"**, described in the rulebook as one of the world's
largest arenas of its kind with craters, mounts, rocky gardens, rifts and quarry
fines; SPROS publicity puts past IRC fields at ~**20,000 m²** of simulated Martian
terrain. RADO specifically adds sand dunes, slopes, boulders and vertical drops.
Past IRC editions used GPS waypoints/gates for autonomous segments, and the
science task used marked sampling patches. We mirror all of this at reduced scale
(§4).

---

## 2. Ground rules for the simulation

1. **Simulate the sensors, not the brain.** Every node that ships on the Jetson —
   `vio_gate`, `pointcloud_downsampler`, both EKFs, `navsat_transform`, Nav2 with
   our STVL costmaps, `goal_manager`, `waypoint_manager`, `teleop_mux`,
   `motor_bridge` — must run **unmodified** in sim. Only the hardware layer is
   replaced: RealSense driver → Gazebo depth camera; `pixhawk_bridge`+Cube Orange
   → Gazebo IMU/GNSS with an RTK error model; Pico+motors → Gazebo drivetrain.
   The repo already has this pattern (`bench_test.launch.py`: fake sensors +
   real pipeline) — we are upgrading that pattern from a rectangle-loop fake to
   physics.
2. **Same topics, same frames, same params.** Topic names, TF tree
   (`map/odom/base_link/camera_link/imu_link/gps_link`), QoS profiles and config
   files stay identical, so `stack_check`, `localization_monitor`,
   `costmap_drift_check` and the Foxglove panel work in sim without edits.
   Sensor plugins are remapped onto the real names, e.g. the simulated depth
   cloud must appear on `/camera/camera/depth/color/points`.
3. **Repeatable and headless.** Seeded worlds, fixed GPS datum, launchable
   without a GUI (`gz sim -s`), visualized through the existing Foxglove setup.
   This makes every mission rehearsal a regression test.
4. **Sim is for autonomy, ops and integration — not for mechanism design.** We do
   not trust Gazebo wheel-soil numbers; we trust it for software behaviour,
   sensor fusion, navigation logic and operator training.

---

## 3. Tooling and where it runs

- **ROS 2 Humble** (what the rover runs) + **Gazebo Fortress (gz-sim 6)** via
  `ros_gz`. Recommended over Gazebo Classic 11 because Classic is end-of-life
  (no fixes through our 2027 cycle) and Fortress is the Humble-paired LTS with
  everything we need: `rgbd_camera`, `imu`, `navsat` sensors, `DiffDriveSystem`,
  heightmap terrain, headless server. All our sensors/topics bridge with
  `ros_gz_bridge`. (Fallback if Fortress pain becomes real: Classic 11 — same
  plan applies, swap plugin names.)
- **Where to run** (this WSL instance installs nothing, per team decision):
  - Option A (preferred): a team PC/laptop with Ubuntu 22.04 — native, or
    dual-boot, or Docker (`ros:humble` + Fortress layer). Any machine with a
    modest GPU runs this world comfortably.
  - Option B: this same WSL2 later, using WSLg + GPU paravirtualization — gz-sim
    works there, or fully headless with Foxglove for visuals (matches our
    existing workflow; the Jetson stays out of it).
  - Option C: headless on any Linux box/server; view only through Foxglove.
- New package: **`athena_sim`** in `src/`, sibling of the others:

```
src/athena_sim/
  urdf/athena_sim.xacro        # sim model extending athena.urdf.xacro frames
  worlds/                      # test_flat.sdf, sproscape.sdf, + generated heightmaps
  models/                      # rocks, tools, markers, panel, cones, cache...
  launch/sim.launch.py         # world + spawn + bridge + (bringup | layers)
  config/bridge.yaml           # ros_gz_bridge topic list
  config/nav2_params_sim.yaml  # use_sim_time overrides ONLY (see §7)
  scripts/                     # terrain generator, object spawner, mission runner, scorer
  test/                        # launch tests (spawn, topics, drive-to-goal)
```

---

## 4. The world — "Sproscape-Sim"

Scale: a **~120 × 120 m** heightmap terrain (the real arena is ~20,000 m²; a
150 m square ≈ 22,500 m² — but sensor/perf budget and our 3.5 m depth range make
120 m the practical start; §5.6 covers extending to full 500 m legs). Generated
**procedurally by a script** (seeded), not hand-modeled, so it is reproducible
and re-tunable. Layout:

```
        N
   ┌────────────────────────────────┐
   │  rocky garden   ABEx site B    │      ● = RADO objects (tools/rocks)
   │  (boulders      (outcrop +     │      ▲ = delivery marker (coloured)
   │   0.3–1.0 m)     stratigraphy) │      ✕ = red no-go zone
   │        \    sand dune field    │
   │   rift/  \   (15–20° slopes)   │
   │   trench  \   ▲     ●          │
   │  ✕          \      ABEx site A │
   │      crater  \   (soil patch + │
   │       rim     \    red rocks)✕ │
   │ IDMO panel     \               │
   │ (wall + zone)   \    ●     ▲   │
   ├────────────────────────────────┤
   │ BASE STATION  | start line     │   rover spawns at start line
   └────────────────────────────────┘
```

Zones and features (each maps to a rulebook requirement):

1. **Base station edge + start line.** Tent/tables cosmetic model, start gate.
   All missions begin here; interventions restart here.
2. **RADO course.** 8–12 scattered graspable objects from the rulebook list:
   screwdriver, hammer, wrench (handles ≤7 cm dia), toolbox (~40 cm), rocks
   (≤5 kg). 3–4 **delivery locations**: coloured marker flags/posts on visible
   pads, GPS coordinates "given to operators" as decimal degrees in the mission
   briefing (sim datum = venue coordinates, §7). One delivery point doubles as
   the **ABEx sample-cache drop site** (rulebook: cached sample is dropped for
   later retrieval during RADO).
3. **ABEx science sites.** 2–3 visually distinct patches — fine-soil basin,
   rocky outcrop with layered texture, "ice/brine" tinted patch — each with a
   marker stake. Distinct textures matter: the panorama/close-up workflow and
   VIO both depend on the camera actually seeing differences.
4. **IDMO zone.** 1.8 m mock instrument panel: drawer, buttons, toggle switches,
   knob, joystick, latch, 3-pin socket, and a printed **code/pattern board**
   (readable by the camera). Included now so drive-up-and-frame-the-panel is
   solved before the arm exists.
5. **Obstacle fields.** Rock garden (boulders 0.3–1.0 m), a **vertical drop /
   ledge (~0.5 m)** (negative obstacles are the classic depth-camera blind spot
   — we want to find our limits in sim, not in Bengaluru), sand dune field with
   15–20° slopes, a shallow **crater** and a **rift/trench** crossing.
6. **Red no-go zones.** Red-painted rocks/cones per the rulebook; the scorer
   penalizes entry 10% per violation. Later: invisible geofence variant to test
   operator discipline.
7. **Extended leg strip (Phase 4b).** A 500 m corridor out the back gate to
  rehearse max-range operations and long GPS legs at real scale.

Terrain generation: a Python script produces (a) an Ogre heightmap PNG from
parametric craters/mounds/dunes + value noise, (b) the matching `.world` SDF with
zone anchors written to a YAML the mission runner reads. One seed = one
identical world forever. Mars palette textures (deliberately including some
featureless sand — VIO must survive it, or we learn it can't).

Physics: Earth gravity (the competition is on Earth). Global medium-friction
ground; individual rock/panel models carry their own friction. We do **not**
attempt per-zone soil physics.

**What we are NOT putting in the world:** RF propagation/congestion (rulebook
emphasises the congested 5.8 GHz environment — real-world testing only), day/night
or weather (missions run in daylight), crowds/spectators, the Buddy Micro Rover
(optional later), the arm.

---

## 5. The rover model — `athena_sim.xacro`

No arm, full sensor stack. Start from the **measured numbers already in
`athena.urdf.xacro`** — those are load-bearing and stay the single source of
truth for frames:

| Thing | Value (from URDF) | Sim treatment |
|---|---|---|
| Chassis | 1.30 × 1.00 m, base_link 0.10 m above ground | collision box + inertia; total mass budget ~50 kg placeholder (rulebook cap 65 kg incl. arm; measure real rover later) |
| Wheels | radius 0.06 m (!), track 0.85 m, axles at x = 0, ±0.45 | convert the 6 visual-only fixed joints to `continuous` joints with collision + inertia. **Measure the real wheel radius** — 6 cm is flagged "not measured" in the URDF and ground clearance over a stony field depends on it |
| `camera_link` | x=0.50, z=0.55, pitch 0 | depth camera sensor (below) |
| `imu_link` | x=0.50, z=0.40 | IMU sensor (below) |
| `gps_link` | x=0, y=0, z=0.25 | navsat sensor (below) |

Structure: `athena.urdf.xacro` gains a `sim:=true` mode (wheels become continuous
joints, collision/inertial blocks included); `athena_sim.xacro` includes it and
adds the Gazebo plugins. One file keeps hardware and sim frames from drifting.

### 5.1 Drivetrain — staged realism

- **Stage 1 (kinematic):** `DiffDriveSystem` (left = 3 left wheels, right = 3
  right wheels) subscribing to `/cmd_vel` via the bridge. Six-wheel skid-steer;
  odometry published from the wheel joints. Gets us driving in a day.
- **Stage 2 (physics honesty):** tune wheel friction (`mu1` longitudinal /
  `mu2` lateral — skid-steer turning needs low lateral scrub to rotate at all);
  verify 0.8 rad/s spins and 15–20° dune climbs behave; accept and record
  differences from the real rover.
- **Stage 3 (full-fidelity, optional):** add a `sim` output mode to the real
  `motor_bridge` (it already has `dry_run`) so `/cmd_vel` runs through the
  **real** deadband compensation, curvature-preserving saturation and
  jerk-limited `VelocityProfile` before hitting the wheels. This is the
  bench_test philosophy applied to sim: exercise shipping code.

### 5.2 Sensors — exact drop-in replacements

| Real hardware | Sim plugin | Must match |
|---|---|---|
| RealSense D435i depth + colour | `rgbd_camera` at `camera_link` | depth 87°×58° H×V FOV (STVL clear frustum is tuned to ±39.5°/±32.2°), 424×240 or 640×480 @ 15 Hz, range 0.25–4.5 m (downsampler clips 0.25–3.5); publish colour, aligned depth and the cloud as `/camera/camera/depth/color/points` so `pointcloud_downsampler` and both STVL costmaps run unmodified |
| D435i IMU → `imu_filter_madgwick` → rtabmap VIO init | include gyro/accel in the IMU below or a second small sensor at `camera_link`; same remaps | `/camera/camera/imu` |
| Cube Orange IMU → `pixhawk_bridge` | `imu` sensor at `imu_link`, bias/noise set to Cube-Orange-class values | publishes `/imu/data` (FLU body, ENU orientation — bridge or sensor config handles the convention) at ~50 Hz; the EKFs only fuse vyaw, but keep the message faithful |
| DroneCAN RTK GNSS → `pixhawk_bridge` | `navsat` sensor at `gps_link` **wrapped by a small `sim_gps_rtk` node** (extends `fake_gps_imu`) | `/gps/fix` at 5 Hz with the full RTK error model (below), frame `gps_link`, covariance exactly like the bridge's h_acc logic |
| rtabmap `rgbd_odometry` + `vio_gate` | **Option A (default): run the real rtabmap on the simulated images.** **Option B (light machines): `vio_sim` node injecting `/vio/odometry` from ground truth + drift/noise model.** Both feed the unchanged `vio_gate` | `/vio/odometry` twist at ~10 Hz |

### 5.3 RTK GNSS error model (`sim_gps_rtk`)

The RTK branch is the reason we're here, so the GPS model is first-class:

- Modes switchable at runtime (topic/service): **RTK Fixed** (σ ≈ 0.02 m,
  matching `gps_rtk_sigma_floor`), **RTK Float** (σ ≈ 0.3–0.5 m, occasional
  jumps), **3D Standalone** (HDOP model, σ ≈ 1.5–3 m, slow wander), **NO FIX**
  (status −1, like today's broken antenna — good for `local_only` drills).
- Fix status + `h_acc`-derived covariance, exactly as `pixhawk_bridge` computes
  them, so `ekf_global` weighting behaviour is the real one.
- Scriptable degradation scenarios: Fixed→Float→Standalone mid-mission,
  multipath-ish jitter near the rock garden, total dropout for N s (rehearse the
  ARCHITECTURE.md §8 failure table in real time).
- RTCM/NTRIP path itself is **not** simulated (bypasses `pixhawk_bridge`); a
  fake NTRIP server on :2101 is an optional Phase 6 nicety.

### 5.4 What is deliberately not on the model yet

Robotic arm (IDMO needs it — model arrives with the real arm design; the panel
zone is ready and waiting), science payload internals (drill/cache/assay —
mission scripts treat them as operator-declared actions), Buddy Micro Rover.

---

## 6. What we simulate — capability checklist

Each item names the IRC risk it retires:

1. **RTK waypoint autonomy (RADO).** Datum handling, lat/lon goal → Nav2 →
   delivery accuracy vs the scorer's proximity rings, behaviour when fix
   degrades mid-leg. Retires: "autonomous delivery for full points".
2. **Obstacle avoidance on rough terrain.** STVL marking/clearing on realistic
   depth data; phantom obstacles; the 0.5 m negative drop (does the rover see
   it, stop, or fall in?). Retires: boulder-field crashes in Bengaluru.
3. **Skid-steer dynamics on slopes.** Whether 6-wheel/0.06 m-wheel geometry
   climbs the dune field; velocity-profile behaviour at the traction limit.
   Retires: "found out at the venue that the rover can't climb".
4. **Mission ops rehearsal.** 25-min clock, 7-min abort, 30% rule, 15-min bonus
   unlock, red-zone penalties, intervention restarts — enforced by the mission
   runner; operators train on the **same Foxglove panel** they'll use in the
   base station. Retires: operator errors, which historically cost more than
   software.
5. **Failure drills.** Camera stall (costmap goes stale → controller refuses to
   drive — verify that's what happens), VIO dropout in featureless sand, GPS
   loss, EKF divergence. Retires: panic during the real mission.
6. **Regression benchmarking.** Seeded worlds + existing tools (`stack_check`,
   `localization_monitor`, `costmap_drift_check`) = a repeatable autonomy
   benchmark for every PR that touches nav/EKF/drive code. Retires: "worked
   last week" regressions.

---

## 7. Integration details that will bite us (handle in Phase 2)

- **`use_sim_time`**: every node in `nav2_params.yaml` hardcodes `use_sim_time:
  False`. Ship `nav2_params_sim.yaml` that overrides **only** sim-time (keep the
  rest inherited/identical), and set `use_sim_time:=true` for EKFs, navsat,
  vio_gate, bridges, foxglove nodes. The Gazebo `/clock` must be bridged first.
- **Topic/name fidelity**: bridge remaps so names match exactly —
  `/camera/camera/depth/color/points` (STVL sources),
  `/camera/camera/color/image_raw` + `/camera/camera/aligned_depth_to_color/image_raw`
  (rtabmap), `/camera/camera/imu`, `/imu/data`, `/gps/fix`, `/cmd_vel`.
  The `camera_link→camera_camera_link` static alias stays harmless.
- **QoS**: ros_gz bridge must publish sensor topics with `sensor_data` QoS
  (best-effort) so `pointcloud_downsampler`/rtabmap subscriptions match.
- **GPS datum**: set `navsat_transform` `wait_for_datum: true` with a fixed
  sim datum near the venue (~12.8°N, 77.5°E) — kills the 45 s settle wait and
  makes runs repeatable; `datum` also defines what lat/lon the mission briefing
  hands the operator.
- **TF ownership**: `ekf_local`/`ekf_global` still own `odom→base_link` and
  `map→odom`; the Gazebo diff-drive odometry must **not** publish TF (or be
  bridged to a scratch topic only), or the tree gets two parents.
- **cmd_vel routing**: `teleop_mux`/Nav2/`velocity_smoother` chain stays the
  only publisher path; the bridge feeds the diff-drive system.
- **Foxglove reuse**: run `foxglove.launch.py` unmodified in sim — same panel,
  same layout JSON, satellite map now showing the sim datum.

---

## 8. Phases

Each phase has an exit test; do not start the next until it passes.

**Phase 0 — Foundations (½ week).**
Tooling decision + install on the sim machine (not this WSL); `athena_sim`
package skeleton; `test_flat.sdf` (flat ground, sky, one box); CI builds it.
*Exit: `gz sim` loads the flat world headless; `colcon test` green.*

**Phase 1 — Rover spawns and drives (1 week).**
`athena_sim.xacro` (sim:=true wheels, collision/inertia, Stage-1 diff drive);
`sim.launch.py` spawns Athena at the start line; `teleop_mux` arrows drive it.
*Exit: drive a lap of the flat world with the Foxglove teleop panel; TF tree
identical to hardware; `stack_check` runs (sensor sections may fail).*

**Phase 2 — Sensor parity (1–1.5 weeks).**
Depth camera → downsampler → STVL costmaps visible in Foxglove; IMU; GNSS with
`sim_gps_rtk` in RTK-Fixed mode; both EKFs + navsat up; Option-B VIO injector to
start, Option A (real rtabmap) once images are stable. All §7 items.
*Exit: `stack_check` reports ALL CHECKS PASS in sim (GPS section included), and
`localization_monitor` shows sub-5 cm error vs ground truth on a driven loop.*

**Phase 3 — Autonomy in sim (1 week).**
Nav2 goals: click-to-goal and lat/lon goals through the real `goal_manager`;
waypoint mark/return; transfer-check one real-world tuning (e.g. fixed 1.0 m
lookahead) — behaviour should match hardware qualitatively.
*Exit: 10/10 consecutive goals ≥25 m away on flat+sparse terrain, zero
interventions, logged by `localization_monitor`.*

**Phase 4 — Sproscape-Sim (1.5–2 weeks).**
Terrain generator + zones + objects + markers + red zones; Stage-2 friction
tuning; dune/climb/rock-garden/drop trials; camera-vs-negative-obstacle study.
*Exit: full 120 m world traverse from start line to the far delivery marker on
RTK autonomy; the 0.5 m drop is detected or its failure is documented and
mitigated; dune climb verdict recorded with data.*

**Phase 5 — Mission rehearsal (1.5 weeks).**
Mission runner + scorer: RADO (10-min recon with photo+GPS logging, 15-min
autonomous delivery with proximity scoring, teleop-lockout enforcement),
ABEx-lite (site visits, panorama capture, cache drop at designated point),
IDMO-lite (panel approach + camera framing + code read). 25-min clock, 7-min
abort, 30% rule, red-zone penalties, intervention simulation. GPS degradation
scenarios on a switch.
*Exit: a full mock RADO mission scored end-to-end with the Foxglove panel as
the only operator interface; a written post-mission report like the real ABEx
report.*

**Phase 6 — Advanced (stretch, December–January).**
Stage-3 motor_bridge-in-the-loop; fake NTRIP server; VIO Option-A as default +
featureless-sand stress tests; 500 m leg strip; automated regression suite in
CI (nightly seeded missions); Buddy micro rover stub; arm stub when the real
arm design exists.

**Calendar fit:** Phases 0–2 by ~Nov 20 → screenshots + results into the SDDR
(due Nov 25) as the "prototype testing strategy and verification" section.
Phases 3–5 through December, Phase 6 + hardware integration in January. Field
rehearsals (the real rover) whenever the GPS antenna fix lands — sim and field
results should be compared run-for-run.

---

## 9. Risks / open questions

- **WSL2 GPU**: if sim lands on this WSL instance, verify `wslg` + GPU
  acceleration before Phase 1; headless+Foxglove is the fallback everywhere.
- **Wheel radius 0.06 m is flagged unmeasured** in the URDF — measure the real
  wheels and update before trusting Phase-4 climb results.
- **Skid-steer physics fidelity**: Gazebo friction models are approximate;
  treat Stage-2 results as directional, validate climb limits on the real rover.
- **gz-sim rgbd camera intrinsics/noise** differ subtly from a real D435i —
  compare a simulated vs real cloud of the same box wall once (Phase 2).
- **rtabmap on rendered images** can behave differently than on real textures
  (too-perfect features) — keep Option B available for nav-only work.
- **Humble + Fortress** is a supported pair, but pin versions in a Dockerfile so
  every teammate runs the same sim.
- Open: do we add the venue's real GPS datum as the sim datum (yes, proposed)?
  Do we model the start-line restart rule in the scorer (proposed yes)?

---

## 10. Sources

- IRC 2027 Rulebook (SPROS) — missions, timing, penalties, rover limits, Sproscape.
- SPROS / IRC public pages (spaceroboticssociety.org, roverchallenge.org) — arena
  scale (~20,000 m²), past formats.
- Fictionlab, "Exploring Mars through simulation" — ERC Leo Rover Gazebo Mars
  yard (open-source reference world).
- NASA Curiosity Gazebo ROSject; aunefyren/mars_gazebo — additional Mars-world
  references.
- Repo: `src/docs/ARCHITECTURE.md`, `src/athena_gps_nav/*` (configs, URDF,
  bridges), `src/athena_drive/*`, `src/athena_remote/*`.
