# WiFi captive-portal login from a terminal

Campus WiFi that wants a browser login would otherwise mean a trip to the lab
with a monitor every time the lease expires. The Jetson has no screen, so do
it from the SSH session. Back to the [package README](../README.md).

## A real browser in the terminal

`w3m` is installed on the Jetson. It renders forms and keeps cookies, which is
all a portal needs. Use it the first time you meet a portal, or when the
automated login below stops working and you need to see what the page says.

```bash
w3m http://connectivitycheck.gstatic.com/generate_204
```

That URL normally returns an empty `204`, so anything you see is the portal
intercepting you.

| Key | Does |
|---|---|
| `Tab` / arrows | move between fields and links |
| `Enter` on a field | start typing; `Enter` again to accept |
| `Enter` on the button | submit |
| `q` then `y` | quit |

Confirm you are through:

```bash
curl -s -o /dev/null -w "%{http_code}\n" http://connectivitycheck.gstatic.com/generate_204
# 204 = through. Anything else = still captive.
```

## Automated login

```bash
# 1. is a portal in the way?
ros2 run athena_remote captive_login --ros-args -p mode:=detect

# 2. ONE TIME - read the login form and write a config template
ros2 run athena_remote captive_login --ros-args -p mode:=discover

# 3. fill in ~/.config/athena_remote/portal.yaml, then log in
ros2 run athena_remote captive_login
```

- The config lives in `~/.config/athena_remote/portal.yaml`, not in the repo,
  is written `0600`, and credentials are never logged. Keep it that way.
- `discover` reads the real form because every portal names its fields
  differently (`Username`/`user`/`uname`, plus hidden CSRF tokens that must be
  echoed back verbatim); guessing fails silently. The template lists every
  field it found and marks the likely username, password and hidden ones.
- Success is verified: a portal will return `200 OK` for a failed login, so the
  tool re-probes for real internet before reporting success.

If it still fails, in order of likelihood: wrong credentials; a per-page-load
CSRF token (re-run `discover` for a fresh value); the form is built by
JavaScript so there is no `<form>` to read (capture the real POST once in a
browser's dev tools, Network tab with preserve log, write those fields into the
config by hand, and the tool replays it).

## Making it automatic

Re-login on boot and whenever the lease drops:

```bash
# crontab -e
@reboot      sleep 30 && . /opt/ros/humble/setup.sh && ~/athena/install/athena_remote/lib/athena_remote/captive_login
*/10 * * * * . /opt/ros/humble/setup.sh && ~/athena/install/athena_remote/lib/athena_remote/captive_login
```

The `. /opt/ros/humble/setup.sh` is required. That path is a setuptools console
script that imports `rclpy`, and cron has no `PYTHONPATH`, `AMENT_PREFIX_PATH`
or `LD_LIBRARY_PATH`; without it the job dies on `ModuleNotFoundError: rclpy`
and tells you nothing. `captive_login` needs no ROS graph (it is a node only so
`ros2 run` can start it). It exits immediately when already online, so every
ten minutes costs one HTTP probe.

## Still no internet after logging in

Check the clock. A Jetson that booted offline has a wrong clock, which breaks
certificate validation (`apt`, `git`) and TF lookups. See
[Correcting the clock](../../README.md#correcting-the-clock).
