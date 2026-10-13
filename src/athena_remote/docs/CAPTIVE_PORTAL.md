# WiFi Captive Portal Login

Campus WiFi networks intercept internet traffic with a browser login page. Because the Jetson is headless (no monitor), you authenticate directly from your terminal session.

---

## Quick Setup (Terminal Login)

Run these commands inside your SSH session on the Jetson:

```bash
# 1. Inspect the login page and generate configuration template (ONE TIME)
ros2 run athena_remote captive_login --ros-args -p mode:=discover   # writes ~/.config/athena_remote/portal.yaml

# 2. Fill in your username and password (press 'i' to type, 'Esc' then ':wq' to save)
vim ~/.config/athena_remote/portal.yaml

# 3. Log in (authenticates session; exits immediately if already online)
ros2 run athena_remote captive_login

# 4. Synchronize system clock immediately after connection
sudo systemctl restart systemd-timesyncd
```

> [!IMPORTANT]
> **Always sync system time after logging in.** Booting offline leaves the Jetson clock unsynchronized, which causes TF extrapolation errors, ROS node failures, and broken SSL certificates (`apt`/`git`).

---

## Verify Connection & Clock

```bash
# Verify clock synchronization (should show: System clock synchronized: yes)
timedatectl

# Verify internet access (204 means fully connected)
curl -s -o /dev/null -w "%{http_code}\n" http://connectivitycheck.gstatic.com/generate_204
```

---

## Stay Logged In Across Reboots (Crontab)

To automatically log in on boot and refresh expired portal leases:

1. Open crontab editor:
   ```bash
   crontab -e
   ```
2. Append the following lines:
   ```cron
   @reboot      sleep 30 && . /opt/ros/humble/setup.sh && ~/athena/install/athena_remote/lib/athena_remote/captive_login && systemctl restart systemd-timesyncd
   */10 * * * * . /opt/ros/humble/setup.sh && ~/athena/install/athena_remote/lib/athena_remote/captive_login
   ```

> [!NOTE]
> `. /opt/ros/humble/setup.sh` is necessary because cron runs with a minimal environment lacking `PYTHONPATH` and ROS paths.

---

## Fallback: Manual Terminal Browser (`w3m`)

If the portal uses complex dynamic JavaScript or novel anti-bot challenges that `discover` cannot parse, use the installed terminal web browser:

```bash
w3m http://connectivitycheck.gstatic.com/generate_204
```

| Key | Action |
|---|---|
| `Tab` / `Arrow keys` | Navigate between input fields and buttons |
| `Enter` on input | Enter text; press `Enter` again to finish editing |
| `Enter` on button | Submit form / Log in |
| `q` then `y` | Quit browser |

Once logged in, remember to synchronize time:
```bash
sudo systemctl restart systemd-timesyncd
```
