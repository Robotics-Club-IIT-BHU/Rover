# athena_remote

The operator's side of the rover. Two jobs:

1. **Control panel.** A Foxglove front end to watch and drive the rover, plus
   the nodes behind it: GPS goals, saved waypoints, travelled trail,
   speed-limited teleop.
2. **Getting to the rover at all.** Shell access, WiFi captive-portal login,
   and viewing the ROS graph from a laptop on a network that blocks DDS.

Nothing here is needed for the rover to navigate. Kill all of `athena_remote`
and it still drives a route; it just has no convenient way to be told where to go.

## Everyday commands

```bash
# Jetson: already running if you used bringup.launch.py. Do not run it twice.
ros2 launch athena_remote foxglove.launch.py

# your machine: open and leave running, then connect Foxglove to ws://localhost:8765
ssh -N -L 8765:localhost:8765 robo@172.20.119.87

# any ROS command on the Jetson, no Foxglove (the export is not optional, see Option B)
ssh robo@172.20.119.87 'export ROS_DOMAIN_ID=42; source ~/athena/install/setup.bash && ros2 topic hz /odometry/local'
```

Panel walkthrough, goals, waypoints and troubleshooting:
[docs/FOXGLOVE_PANEL.md](docs/FOXGLOVE_PANEL.md).

### What actually runs

| Process | Started by | Job |
|---|---|---|
| `foxglove_bridge` | `foxglove.launch.py` (and `bringup.launch.py`) | serves ROS topics to Foxglove over one WebSocket, port 8765, loopback only |
| `goal_manager` | `foxglove.launch.py` | typed or clicked lat/lon -> Nav2 goal; understands `cancel` |
| `waypoint_manager` | `foxglove.launch.py` | save a named place, drive back to it |
| `trajectory` | `foxglove.launch.py` | publishes the driven path as the trail |
| `teleop_mux` | `foxglove.launch.py` | panel arrows -> `/cmd_vel`, cancelling any running goal first |
| `panel_camera` | `foxglove.launch.py` | colour camera -> throttled JPEG on `/athena/camera/compressed`; idle while nobody watches |
| `panel_cloud` | `foxglove.launch.py` | depth cloud -> voxel-thinned copy on `/athena/points_preview` for the 3D view; idle while nobody watches |
| `captive_login` | by hand: `ros2 run athena_remote captive_login` | logs the Jetson into a WiFi captive portal |

The first five run on the Jetson; none of them is needed for the rover to
navigate. `ssh` and the tunnel run on your own machine.

## Which way in

All three are verified working.

| You want | Use |
|---|---|
| Drive the rover, watch costmap, GPS, camera | **Option A, Foxglove panel.** Needs only a browser and SSH. Start here. See [docs/FOXGLOVE_PANEL.md](docs/FOXGLOVE_PANEL.md) |
| Run a command, check a rate, launch something | **Option B, ROS CLI over SSH.** Below |
| `rviz2` and native `ros2` tools on your laptop | **Option C, CycloneDDS.** Below. Worth the setup only for native tooling |

Why ROS 2 does not just work across managed WiFi: see the start of Option C.
SSH works because it is one outbound TCP connection, which such networks always
allow; every option here makes ROS look like that.

## Foxglove launch arguments

`foxglove.launch.py` declares sixteen arguments, and these are all of them. Add
`operator_nodes:=false` for the bridge alone.

| Argument | Default | Does |
|---|---|---|
| `port` | 8765 | bridge TCP port |
| `address` | 127.0.0.1 | loopback only; `0.0.0.0` listens on the network, and the bridge has no authentication |
| `topic_whitelist` | light set | regex list of served topics; `"['.*']"` for everything, including the 13.8 MB/s raw image (wired LAN only) |
| `operator_nodes` | true | the five panel nodes |
| `trajectory_source` | `/odometry/global` | use `/odometry/local` with no GPS fix; `bringup.launch.py` sets it from `local_only` |
| `max_speed` | 0.35 | m/s at full teleop stick |
| `max_turn` | 0.8 | rad/s at full teleop stick |
| `num_threads` | 3 | bridge worker threads |
| `max_qos_depth` | 2 | per-topic queue before the bridge sends |
| `message_backlog_size` | 200 | one shared outbox across every channel |
| `camera_rate` | 5.0 | panel camera JPEG frames per second |
| `camera_quality` | 70 | panel camera JPEG quality, 1-100 |
| `camera_width` | 0 | panel camera width in px, 0 = native 640 |
| `cloud_rate` | 2.0 | panel point cloud messages per second |
| `cloud_voxel` | 0.10 | panel point cloud: keep one point per cube of this size, m |
| `cloud_max_points` | 2000 | panel point cloud: cap per message |

## Getting a shell on the Jetson

```bash
ssh robo@172.20.119.87
```

`172.20.119.87` is the address on 2026-10-02 (it was `172.20.58.70` before). It
changes with DHCP: see "The address changes with DHCP" below to find it.

If you will do this often, add a host alias on your own machine. `ssh athena`
then also opens the Foxglove tunnel:

```
Host athena
    HostName 172.20.119.87
    User robo
    ForwardAgent yes
    LocalForward 8765 localhost:8765
```

`ForwardAgent` lets `git push` from the Jetson sign as you rather than as the
shared `robo` account; setup is in
[CONTRIBUTING.md section 6](../CONTRIBUTING.md#6-working-on-the-jetson-with-your-own-github-account).
No password for the account is recorded in this repository: ask a teammate for access.

**The address changes with DHCP.** From a machine already on the network:

```bash
ssh robo@ubuntu.local                      # mDNS: the Jetson's hostname is `ubuntu`, avahi is running
sudo nmap -sn 172.20.119.0/24 | grep -B2 -i nvidia   # or scan; widen to 172.20.0.0/17 if it moved far
```

Look for an NVIDIA vendor string, not Raspberry Pi (the Pico is behind USB, not
on the network). `nmap` is not installed on the Jetson or on most laptops
(`sudo apt install nmap`), and needs `sudo` to print MAC vendors. Without it,
ping-sweep and read the ARP table:

```bash
for i in $(seq 1 254); do ping -c1 -W1 172.20.119.$i >/dev/null & done; wait; ip neigh | grep -i REACHABLE
```

On the Jetson itself: `hostname -I`, or `ip -4 -o addr show scope global | awk '{print $2, $4}'`.
Ask the network admin for a DHCP reservation on the Jetson's MAC so the
address (and the tunnel command) stops changing.

**ROS environment.** `~/.bashrc` on the Jetson sources ROS and the workspace
and sets `ROS_DOMAIN_ID=42`. A shell that skips it (for example
`ssh host 'some command'`, which is non-interactive) lands on domain 0 and sees
nothing. If a shell reports no topics:

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/humble/setup.bash
source ~/athena/install/setup.bash
```

## WiFi captive portal

Campus WiFi intercepts the Jetson with a login page it has no screen to show.
From the SSH session:

```bash
w3m http://connectivitycheck.gstatic.com/generate_204   # a text browser; 204 = through, a page = portal
ros2 run athena_remote captive_login --ros-args -p mode:=detect     # is a portal in the way?
ros2 run athena_remote captive_login --ros-args -p mode:=discover   # ONE TIME: write ~/.config/athena_remote/portal.yaml
ros2 run athena_remote captive_login                                # log in (after filling in portal.yaml)
```

Details, JavaScript portals and the cron re-login:
[docs/CAPTIVE_PORTAL.md](docs/CAPTIVE_PORTAL.md). Still offline after
logging in: check the clock ([root README](../README.md#correcting-the-clock)).

## Option B: ROS CLI over SSH

The simplest thing that works, with one required export:

```bash
ssh robo@172.20.119.87 'export ROS_DOMAIN_ID=42; source ~/athena/install/setup.bash && ros2 topic list'
ssh robo@172.20.119.87 'export ROS_DOMAIN_ID=42; source ~/athena/install/setup.bash && ros2 run athena_gps_nav stack_check'
```

`ssh host 'command'` runs a non-interactive shell, and `~/.bashrc` starts with
the stock Ubuntu guard (`case $- in *i*) ;; *) return;; esac`), so it returns
before reaching `export ROS_DOMAIN_ID=42`. The command then runs on domain 0
while every node is on 42, and you get an empty graph that looks like a
network fault or a stack that is not running. Sourcing `install/setup.bash`
does not help (it sets package paths, not the domain). Diagnose with:

```bash
ssh robo@172.20.119.87 'echo "domain=[$ROS_DOMAIN_ID] rmw=[$RMW_IMPLEMENTATION]"'
# domain=[] rmw=[]   <- the failure, not a network problem
```

A shell function on your laptop makes it feel local:

```bash
jros() { ssh robo@172.20.119.87 "export ROS_DOMAIN_ID=42; source ~/athena/install/setup.bash && ros2 $*"; }
# then:  jros topic list      jros topic echo /gps/fix
```

If the stack was started from a shell that had `athena_env.sh` sourced (Option
C), also `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` in the command: the CLI
must use the same middleware as the nodes it inspects.

## Option C: native ROS on a laptop

The CycloneDDS setup guide. For `rviz2` and `ros2 topic echo` running on your own machine against the
Jetson's live graph. Driving the rover and watching costmap, GPS and camera
needs only Foxglove, which is simpler.

**Why it is needed.** ROS 2 has no master. Nodes find each other by DDS
discovery: a multicast announcement to `239.255.0.1`, direct replies, then data
over a wide range of UDP ports. Managed WiFi breaks each step: it drops
multicast, **AP/client isolation** stops clients reaching each other, and it
firewalls the UDP range. `ROS_DOMAIN_ID` only partitions traffic on a network
that already carries it; it does not help machines find each other. Three
problems must be solved, in order:

1. **IP reachability** between the two machines.
2. **Discovery without multicast**: CycloneDDS with an explicit unicast peer.
3. **The same middleware on both machines.** Two ROS 2 machines on different
   RMWs never discover each other, whatever the domain and peers. Nothing
   reports it: both sides come up clean and see nothing.

**Prerequisite.** The laptop runs ROS 2 Humble on Ubuntu 22.04 and every shell
has `source /opt/ros/humble/setup.bash`. Install `ros-humble-desktop` for
`rviz2` (or at least `ros-humble-rviz2`), or `ros-humble-ros-base` if
`ros2 topic echo` is enough. No version
mixing: Cyclone on Humble will not talk to a Jazzy or Foxy machine.

### Step 1: IP reachability

```bash
ping <jetson-ip>        # from the laptop, e.g. 172.20.119.87
```

If it works, go to step 2. If not, the network isolates clients and you need an
overlay VPN, on **both** machines:

```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo tailscale up
tailscale ip -4                                     # note each 100.x.y.z
```

`ping 100.x.y.z` (the other machine) must work before continuing.

### Step 2: install CycloneDDS on the laptop

Already installed on the Jetson, alongside FastDDS.

```bash
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

### Step 3: copy the helper to the laptop

`athena_env.sh` finds `cyclonedds_peers.xml` at `../config/` relative to itself,
so keep the sibling `scripts/` and `config/` layout. Copy only the `.sh` and it
fails with `cyclonedds_peers.xml not found` (its fallback, `ros2 pkg prefix
athena_remote`, only helps on a machine with this workspace built, which the
laptop is not):

```bash
mkdir -p ~/athena_net/scripts ~/athena_net/config
scp robo@172.20.119.87:/home/robo/athena/src/athena_remote/scripts/athena_env.sh ~/athena_net/scripts/
scp robo@172.20.119.87:/home/robo/athena/src/athena_remote/config/cyclonedds_peers.xml ~/athena_net/config/
```

### Step 4: source it on both machines, pointed at the other one

```bash
# laptop  -> the Jetson's address
source ~/athena_net/scripts/athena_env.sh 172.20.119.87

# Jetson  -> the laptop's address
source ~/athena/src/athena_remote/scripts/athena_env.sh <laptop-ip>
```

**Source it on the Jetson BEFORE launching the stack.** `RMW_IMPLEMENTATION` is
not set in the Jetson's `~/.bashrc`, so a normal shell there runs the default
`rmw_fastrtps_cpp`. A stack launched from such a shell never discovers a
Cyclone laptop, with no error on either side, and running nodes cannot be
switched: restart them under the new environment.

The script is equivalent to these, where the `CYCLONEDDS_URI` path is wherever
you put the XML on *that* machine (`ATHENA_PEER` is substituted into the XML's
`<Peer address="${ATHENA_PEER}"/>`, so the file needs no editing):

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ATHENA_PEER=<the other machine's address>
export CYCLONEDDS_URI=file://$HOME/athena_net/config/cyclonedds_peers.xml
export ROS_DOMAIN_ID=42      # the script defaults to 42 and leaves an existing value alone
ros2 daemon stop             # the daemon caches the old (multicast) middleware config
```

**No files at all.** `CYCLONEDDS_URI` also accepts literal XML, so one export
replaces the script and config on the laptop:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export ROS_DOMAIN_ID=42 && export CYCLONEDDS_URI='<CycloneDDS><Domain id="any"><General><Interfaces><NetworkInterface autodetermine="true" priority="default" multicast="false"/></Interfaces><AllowMulticast>false</AllowMulticast></General><Discovery><Peers><Peer address="172.20.119.87"/></Peers></Discovery></Domain></CycloneDDS>' && ros2 daemon stop
```

Cyclone picks a network interface itself. If a machine has several (`docker0`, a
VPN, WiFi) and it picks wrong, name one in `config/cyclonedds_peers.xml`:
`<NetworkInterface name="tailscale0" priority="default" multicast="false"/>`.

**`ROS_DOMAIN_ID` must match on both sides: the rover is on 42.** The script
keeps a value that is already set, so an inherited wrong one survives it.
Check, and if a shell disagrees, set it and restart the daemon (which caches the
domain too):

```bash
echo $ROS_DOMAIN_ID
export ROS_DOMAIN_ID=42 && ros2 daemon stop
```

### Step 5: verify, from the laptop

```bash
ros2 topic list
ros2 topic echo /odometry/local
```

RViz: `athena_bench.rviz` is in this workspace on the Jetson, not on the laptop.
Copy it, then open it (plain `rviz2` also works; add the displays by hand, see
[athena_gps_nav RViz setup](../athena_gps_nav/docs/TESTING.md#rviz)):

```bash
scp robo@172.20.119.87:/home/robo/athena/src/athena_gps_nav/config/athena_bench.rviz ~/athena_net/
rviz2 -d ~/athena_net/athena_bench.rviz
```

Verified on this rover: with `AllowMulticast=false` and a single unicast peer,
topic discovery and data transfer both work, and the `${ATHENA_PEER}`
substitution in the XML works.

### If it does not work

Test the simplest pair before touching RViz:

```bash
ros2 run demo_nodes_cpp talker      # Jetson
ros2 run demo_nodes_cpp listener    # laptop
```

`demo_nodes_cpp` is on the Jetson and comes with `ros-humble-desktop`; a
`ros-humble-ros-base` laptop needs `ros-humble-demo-nodes-cpp`. If that fails,
so will everything else:

| Symptom | Cause |
|---|---|
| `ros2 topic list` shows only `/rosout`, `/parameter_events` | wrong peer address, a stale daemon (`ros2 daemon stop`), or different RMWs: `echo $RMW_IMPLEMENTATION` on both, including in the shell the stack was launched from |
| nothing at all, and `ping` fails | step 1: no IP route between the machines |
| topics listed but `echo` hangs | discovery works, data does not: usually a firewall on the laptop |
| works, then stops after a reboot | the env vars are per-shell; add the `source` line to `~/.bashrc` |

### Notes

- **FastDDS** (the default RMW) can do this too, through a Discovery Server or
  TCP transport, but neither worked reliably here: with TCP the sockets
  connected and RTPS logical-port negotiation failed (*"Cannot find an
  available logical port"*). CycloneDDS with an explicit peer is simpler and is
  what was verified.
- **The Jetson's actual desktop.** `gnome-remote-desktop` is running: VNC on
  5900, RDP on 3389. Prefer RDP (much faster on GNOME 42 under Wayland):
  `ssh -N -L 3389:localhost:3389 robo@172.20.119.87`, then RDP to `localhost`.
  It costs the Jetson a full CPU core, dropping visual odometry from ~11 Hz to
  ~5 Hz: fine for a look, not for measurements.

## Docs

| Document | Covers |
|---|---|
| [docs/FOXGLOVE_PANEL.md](docs/FOXGLOVE_PANEL.md) | panel setup, what each panel shows, goals, teleop, waypoints, bandwidth, panel troubleshooting |
| [docs/TOPICS.md](docs/TOPICS.md) | every topic the panel nodes publish or subscribe to, plus `ros2 topic pub` equivalents |
| [docs/CAPTIVE_PORTAL.md](docs/CAPTIVE_PORTAL.md) | WiFi portal login: `w3m`, `captive_login`, cron |
| [TESTING.md](TESTING.md) | testing navigation from the panel and telling why it did what it did |

## Contributors

- **Jashan**: control panel, goal and waypoint nodes, remote access, captive-portal login
