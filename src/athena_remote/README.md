# athena_remote

Remote control interface, monitoring bridge, and connectivity management for the Athena rover.

Provides:
- **Foxglove Studio Bridge:** Web-based control panel for telemetry, costmaps, camera feed, and teleop.
- **Operator Nodes:** Goal dispatch (`goal_manager`), persistent waypoints (`waypoint_manager`), trajectory trails, and speed-limited teleop mux.
- **Connectivity Utilities:** Headless captive portal login (`captive_login`) and CycloneDDS configuration for native multi-machine ROS 2.

---

## Quick Reference

| Task | Command |
|---|---|
| **Set Jetson IP (on laptop)** | `export JETSON_IP=<jetson-ip>` *(or use `ubuntu.local`)* |
| **Open Foxglove tunnel** | `ssh -N -L 8765:localhost:8765 robo@$JETSON_IP` |
| **Run ROS command over SSH** | `ssh robo@$JETSON_IP 'export ROS_DOMAIN_ID=42; source ~/athena/install/setup.bash && ros2 topic list'` |
| **Portal login (on Jetson)** | `ros2 run athena_remote captive_login` |
| **Sync Jetson clock** | `sudo systemctl restart systemd-timesyncd` |

---

## 1. Connecting to the Jetson

### Finding the Jetson's IP
Because IP addresses change over DHCP, define it as an environment variable on your workstation:

```bash
# 1. If mDNS is working (Jetson default hostname is 'ubuntu'):
export JETSON_IP=ubuntu.local

# 2. Or check the IP directly on the Jetson terminal:
hostname -I

# 3. Or scan your local subnet:
sudo nmap -sn 192.168.1.0/24 | grep -B2 -i nvidia    # replace with your local subnet
```

### SSH Configuration (Recommended)
Add this alias to `~/.ssh/config` on your laptop to automate port forwarding:

```ssh-config
Host athena
    HostName <jetson-ip>           # replace with current Jetson IP or ubuntu.local
    User robo
    ForwardAgent yes
    LocalForward 8765 localhost:8765
```

Now you can simply run:
```bash
ssh athena
```

---

## 2. WiFi Captive Portal Login & Clock Sync

When connected to an institutional network requiring browser authentication, log in headlessly via terminal:

```bash
# 1. Discover portal fields & generate config (ONE TIME)
ros2 run athena_remote captive_login --ros-args -p mode:=discover   # writes ~/.config/athena_remote/portal.yaml

# 2. Enter your credentials (i to type, Esc then :wq to save)
vim ~/.config/athena_remote/portal.yaml

# 3. Authenticate against the portal (skips if already connected)
ros2 run athena_remote captive_login

# 4. Sync clock via NTP immediately (required for ROS TF frames and SSL certs)
sudo systemctl restart systemd-timesyncd
```

For automated reboot login or troubleshooting, see [docs/CAPTIVE_PORTAL.md](docs/CAPTIVE_PORTAL.md).

---

## 3. Remote Access Modes

### Option A: Foxglove Studio (Recommended)
Visual front-end in your browser. Needs only SSH port forwarding:

1. **On Jetson:** Start the bridge (already running if using `bringup.launch.py`):
   ```bash
   ros2 launch athena_remote foxglove.launch.py
   ```
2. **On Laptop:** Open port tunnel:
   ```bash
   ssh -N -L 8765:localhost:8765 robo@$JETSON_IP
   ```
3. **On Laptop Browser:** Open [app.foxglove.dev](https://app.foxglove.dev) -> Connect to `ws://localhost:8765` -> Import `athena_foxglove_layout.json`.

Detailed guide: [docs/FOXGLOVE_PANEL.md](docs/FOXGLOVE_PANEL.md).

---

### Option B: ROS 2 CLI over SSH
Run commands directly on the Jetson without visual tooling.

> [!IMPORTANT]
> Non-interactive SSH sessions skip `~/.bashrc`. Always explicitly export `ROS_DOMAIN_ID=42`:

```bash
ssh robo@$JETSON_IP 'export ROS_DOMAIN_ID=42; source ~/athena/install/setup.bash && ros2 topic list'
ssh robo@$JETSON_IP 'export ROS_DOMAIN_ID=42; source ~/athena/install/setup.bash && ros2 run athena_gps_nav stack_check'
```

**Convenience Laptop Alias:** Add to your laptop's `~/.bashrc`:
```bash
jros() {
    ssh robo@$JETSON_IP "export ROS_DOMAIN_ID=42; source ~/athena/install/setup.bash && ros2 $*"
}
# Usage:
# jros topic list
# jros topic echo /odometry/local
```

---

### Option C: Native ROS 2 on Laptop (CycloneDDS Unicast)
Allows running `rviz2` and `ros2` natively on your laptop against the Jetson's live graph across networks that block multicast.

#### Prerequisites
- Laptop running Ubuntu 22.04 with ROS 2 Humble.
- Install CycloneDDS middleware:
  ```bash
  sudo apt install ros-humble-rmw-cyclonedds-cpp
  ```

#### Setup Steps
1. **Copy network scripts to your laptop:**
   ```bash
   mkdir -p ~/athena_net/scripts ~/athena_net/config
   scp robo@$JETSON_IP:/home/robo/athena/src/athena_remote/scripts/athena_env.sh ~/athena_net/scripts/
   scp robo@$JETSON_IP:/home/robo/athena/src/athena_remote/config/cyclonedds_peers.xml ~/athena_net/config/
   ```

2. **Configure environment on both machines:**
   ```bash
   # On Laptop (point at Jetson):
   source ~/athena_net/scripts/athena_env.sh $JETSON_IP

   # On Jetson (point at Laptop):
   source ~/athena/src/athena_remote/scripts/athena_env.sh <laptop-ip>
   ```

   > [!WARNING]
   > On the Jetson, source `athena_env.sh` **before** launching `bringup.launch.py`. Both sides must run the same RMW (`rmw_cyclonedds_cpp`) and `ROS_DOMAIN_ID=42`.

3. **Verify from laptop:**
   ```bash
   ros2 topic list
   ros2 topic echo /odometry/local
   ```

4. **Launch RViz from laptop:**
   ```bash
   scp robo@$JETSON_IP:/home/robo/athena/src/athena_gps_nav/config/athena_bench.rviz ~/athena_net/
   rviz2 -d ~/athena_net/athena_bench.rviz
   ```

---

## 4. Package Architecture & Nodes

| Node | Purpose | Topics / Function |
|---|---|---|
| `foxglove_bridge` | WebSocket gateway | Exposes ROS graph on port 8765 (`127.0.0.1`) |
| `goal_manager` | Goal translation | Converts clicked poses / GPS coords into Nav2 `map` goals |
| `waypoint_manager` | Named locations | Saves/recalls waypoints in `~/.config/athena_remote/waypoints.yaml` |
| `trajectory` | Path history | Publishes driven path trail on `/athena/trajectory` |
| `teleop_mux` | Manual override | Translates panel controls to `/cmd_vel` while safely overriding autonomous goals |
| `panel_camera` | Video compression | Throttles camera to 5 Hz JPEG (`/athena/camera/compressed`) |
| `panel_cloud` | Cloud thinning | Thins depth cloud to 2 Hz voxel preview (`/athena/points_preview`) |
| `captive_login` | Network auth | Headless login for campus captive portal |

---

## Documentation Links

- [Foxglove Control Panel Guide](docs/FOXGLOVE_PANEL.md)
- [Captive Portal Setup & Automation](docs/CAPTIVE_PORTAL.md)
- [Topic Reference & Publish Commands](docs/TOPICS.md)
- [Navigation Testing Procedures](TESTING.md)
