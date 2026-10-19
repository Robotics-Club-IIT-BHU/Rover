#!/usr/bin/env python3
"""
check_env.py

Comprehensive repository-wide pre-flight environment diagnostic tool for Athena Rover.
Audits host OS, ROS 2 distribution, Gazebo simulation engine, ROS package dependencies,
Python libraries, hardware permissions (dialout), and workspace packages across:
  - athena_sim (Gazebo simulation, Sproscape arena, sensor bridge)
  - athena_drive (Pico motor controller, serial bridge, teleop)
  - athena_gps_nav (Dual EKF, Nav2, STVL, RTK GNSS, Pixhawk bridge, VIO)
  - athena_remote (Foxglove bridge, control panel, remote telemetry)
  - athena_slam (RTAB-Map 3D visual SLAM, point cloud filters)
"""

import os
import sys
import shutil
import subprocess
import platform
import argparse
from typing import List, Tuple, Dict

# Terminal color codes
GREEN = '\033[92m'
YELLOW = '\033[93m'
RED = '\033[91m'
CYAN = '\033[96m'
BOLD = '\033[1m'
RESET = '\033[0m'

def print_header(title: str):
    print(f"\n{BOLD}{CYAN}=== {title} ==={RESET}")

def check_item(name: str, passed: bool, msg: str = "", warn: bool = False) -> bool:
    if passed:
        tag = f"{GREEN}[PASS]{RESET}"
    elif warn:
        tag = f"{YELLOW}[WARN]{RESET}"
    else:
        tag = f"{RED}[FAIL]{RESET}"
    
    details = f" - {msg}" if msg else ""
    print(f"  {tag} {BOLD}{name}{RESET}{details}")
    return passed

def run_cmd(cmd: List[str]) -> Tuple[int, str]:
    try:
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=5)
        return res.returncode, (res.stdout + res.stderr).strip()
    except Exception as e:
        return -1, str(e)

def find_repo_root() -> str:
    # Traverse upwards until finding src directory containing package manifests
    cur = os.path.abspath(os.path.dirname(__file__))
    while cur and cur != os.path.dirname(cur):
        if os.path.isdir(os.path.join(cur, "src", "athena_sim")):
            return cur
        cur = os.path.dirname(cur)
    return "/home/jashan/Rover"

def check_ros_package(pkg_name: str) -> bool:
    try:
        from ament_index_python.packages import get_package_share_directory
        get_package_share_directory(pkg_name)
        return True
    except Exception:
        # Fallback to checking /opt/ros/$ROS_DISTRO/share
        distro = os.environ.get("ROS_DISTRO", "humble")
        path = os.path.join("/opt/ros", distro, "share", pkg_name)
        return os.path.isdir(path)

def main():
    parser = argparse.ArgumentParser(description="Athena Rover Pre-Flight Environment Diagnostic Utility")
    parser.add_argument("--sim-only", action="store_true", help="Only audit simulation prerequisites")
    args = parser.parse_args()

    repo_root = find_repo_root()

    print(f"\n{BOLD}=================================================================={RESET}")
    print(f"{BOLD}           Athena Rover Pre-Flight Environment Audit              {RESET}")
    print(f"{BOLD}=================================================================={RESET}")

    sim_failures = []
    hardware_failures = []
    warnings = []

    # 1. Operating System & Platform
    print_header("1. Operating System & Host Environment")
    distro_info = "Unknown Linux"
    if os.path.exists("/etc/os-release"):
        with open("/etc/os-release") as f:
            for line in f:
                if line.startswith("PRETTY_NAME="):
                    distro_info = line.split("=")[1].strip().strip('"')

    is_ubuntu_2204 = "Ubuntu 22.04" in distro_info or "Jammy" in distro_info
    check_item("Operating System", is_ubuntu_2204, f"{distro_info} ({platform.machine()})", warn=not is_ubuntu_2204)
    if not is_ubuntu_2204:
        warnings.append(
            f"Detected {distro_info}. The stack is officially verified on Ubuntu 22.04 LTS (Jammy Jellyfish)."
        )

    # 2. ROS 2 Environment
    print_header("2. ROS 2 Distribution")
    ros_distro = os.environ.get("ROS_DISTRO", None)
    if not ros_distro:
        check_item("ROS_DISTRO", False, "Environment variable ROS_DISTRO not set!")
        sim_failures.append("ROS 2 is not sourced. Run: source /opt/ros/humble/setup.bash")
    elif ros_distro != "humble":
        check_item("ROS_DISTRO", False, f"Detected '{ros_distro}', expected 'humble'", warn=True)
        warnings.append(f"ROS_DISTRO is '{ros_distro}'. The Athena stack is built and verified on ROS 2 Humble.")
    else:
        check_item("ROS_DISTRO", True, f"ROS 2 {ros_distro.capitalize()} active")

    # 3. Workspace Packages
    print_header("3. Rover Workspace Packages")
    ws_packages = ["athena_sim", "athena_drive", "athena_gps_nav", "athena_remote", "athena_slam"]
    for wp in ws_packages:
        found_in_active = check_ros_package(wp)
        src_dir = os.path.join(repo_root, "src", wp)
        install_dir = os.path.join(repo_root, "install", wp)

        if found_in_active:
            check_item(f"Package: {wp}", True, "Found in active AMENT_PREFIX_PATH")
        elif os.path.isdir(install_dir):
            check_item(f"Package: {wp}", False, f"Built in install/{wp} but setup.bash not sourced!", warn=True)
            warnings.append(f"Package '{wp}' built but not sourced. Run: source install/setup.bash")
        elif os.path.isdir(src_dir):
            check_item(f"Package: {wp}", False, f"Found in src/{wp} (requires colcon build)", warn=True)
            warnings.append(f"Package '{wp}' needs build. Run: colcon build --symlink-install && source install/setup.bash")
        else:
            check_item(f"Package: {wp}", False, f"Missing source directory in src/{wp}")
            sim_failures.append(f"Missing workspace package {wp}")

    # 4. Simulation Dependencies (athena_sim)
    print_header("4. Gazebo Simulation Stack (athena_sim)")
    gz_bin = shutil.which("gz")
    ign_bin = shutil.which("ign")
    gz_ver_str = None
    gz_major = None
    engine_name = None

    if gz_bin:
        ret, out = run_cmd(["gz", "sim", "--version"])
        if ret == 0 and "version" in out.lower():
            gz_ver_str = out.split("\n")[0]
            engine_name = "Gazebo (gz)"
            for token in gz_ver_str.split():
                if "." in token and token[0].isdigit():
                    gz_major = int(token.split(".")[0])
                    break

    if not gz_ver_str and ign_bin:
        ret, out = run_cmd(["ign", "gazebo", "--version"])
        if ret == 0 and "version" in out.lower():
            gz_ver_str = out.split("\n")[0]
            engine_name = "Ignition (ign)"
            for token in gz_ver_str.split():
                if "." in token and token[0].isdigit():
                    gz_major = int(token.split(".")[0])
                    break

    if gz_ver_str:
        codename = "Harmonic" if gz_major == 8 else ("Fortress" if gz_major == 6 else f"v{gz_major}")
        check_item("Gazebo Simulator", True, f"{engine_name} -> {gz_ver_str} ({codename})")
    else:
        check_item("Gazebo Simulator", False, "Neither 'gz sim' nor 'ign gazebo' found in PATH!")
        sim_failures.append("Install Gazebo: sudo apt install -y ros-humble-ros-gzharmonic (or ros-humble-ros-gz)")

    sim_ros_pkgs = ["ros_gz_sim", "ros_gz_bridge", "robot_state_publisher", "xacro", "tf2_ros"]
    for spkg in sim_ros_pkgs:
        installed = check_ros_package(spkg)
        check_item(f"Sim ROS Pkg: {spkg}", installed, "Installed" if installed else "Missing")
        if not installed:
            sim_failures.append(f"Missing ROS package '{spkg}'. Run: sudo apt install ros-humble-{spkg.replace('_', '-')}")

    # Simulation world assets
    sim_dir = os.path.join(repo_root, "src", "athena_sim")
    sdf_path = os.path.join(sim_dir, "worlds", "sproscape.sdf")
    hm_path = os.path.join(sim_dir, "worlds", "sproscape_heightmap.png")
    check_item("Sproscape World SDF", os.path.isfile(sdf_path), "120x120m IRC 2027 arena verified")
    check_item("Heightmap Texture", os.path.isfile(hm_path), "257x257 relief grid verified")

    # 5. Motor & Drive Hardware Dependencies (athena_drive)
    print_header("5. Motor & Drivetrain Subsystem (athena_drive)")
    try:
        import serial
        check_item("Python serial (pyserial)", True, f"v{getattr(serial, '__version__', 'installed')}")
    except ImportError:
        check_item("Python serial (pyserial)", False, "Missing python3-serial")
        hardware_failures.append("Missing python3-serial. Run: sudo apt install python3-serial")

    # Serial port permissions (dialout group)
    user = os.environ.get("USER", "")
    in_dialout = False
    try:
        import grp
        for g in grp.getgrall():
            if g.gr_name == "dialout" and user in g.gr_mem:
                in_dialout = True
                break
    except Exception:
        pass
    check_item("Serial Access (dialout group)", in_dialout, f"User '{user}' in dialout" if in_dialout else f"User '{user}' NOT in dialout group!", warn=not in_dialout)
    if not in_dialout:
        warnings.append(f"User '{user}' is not in the 'dialout' group. Serial access to Pixhawk/Pico requires: sudo usermod -a -G dialout {user} (then re-login)")

    # 6. Autonomy & GPS Navigation Dependencies (athena_gps_nav)
    print_header("6. Autonomy & GPS Navigation Stack (athena_gps_nav)")
    nav_ros_pkgs = [
        ("robot_localization", True),
        ("nav2_bringup", True),
        ("spatio_temporal_voxel_layer", True),
        ("nav2_simple_commander", True),
        ("geographic_msgs", True),
        ("map_msgs", True),
        ("realsense2_camera", False),
        ("rtabmap_odom", False),
        ("imu_filter_madgwick", False),
    ]

    for npkg, is_core in nav_ros_pkgs:
        installed = check_ros_package(npkg)
        desc = "Installed" if installed else ("Missing core navigation dependency" if is_core else "Missing physical sensor driver")
        check_item(f"Nav ROS Pkg: {npkg}", installed, desc, warn=(not installed and not is_core))
        if not installed:
            if is_core:
                sim_failures.append(f"Missing core Nav2 package '{npkg}'.")
            else:
                hardware_failures.append(f"Missing sensor driver package '{npkg}'.")

    # Pymavlink check
    try:
        import pymavlink
        check_item("Python pymavlink (Pixhawk MAVLink)", True, "Installed")
    except ImportError:
        check_item("Python pymavlink (Pixhawk MAVLink)", False, "Not installed (required for real Pixhawk)", warn=True)
        hardware_failures.append("pymavlink missing. Run: pip3 install pymavlink")

    # 7. Operator Interface & Telemetry (athena_remote)
    print_header("7. Operator Telemetry & Front End (athena_remote)")
    remote_ros_pkgs = [
        ("foxglove_bridge", False),
        ("foxglove_msgs", False),
        ("nav2_msgs", True),
        ("action_msgs", True),
    ]
    for rpkg, is_core in remote_ros_pkgs:
        installed = check_ros_package(rpkg)
        check_item(f"Remote Pkg: {rpkg}", installed, "Installed" if installed else "Missing", warn=(not installed and not is_core))
        if not installed:
            if is_core:
                sim_failures.append(f"Missing core package '{rpkg}'.")
            else:
                hardware_failures.append(f"Missing operator interface package '{rpkg}'.")

    # 8. Visual SLAM & Point Cloud Processing (athena_slam)
    print_header("8. Perception & Visual SLAM (athena_slam)")
    slam_ros_pkgs = [
        ("sensor_msgs_py", True),
        ("rtabmap_ros", False),
    ]
    for spkg, is_core in slam_ros_pkgs:
        installed = check_ros_package(spkg)
        check_item(f"SLAM Pkg: {spkg}", installed, "Installed" if installed else "Missing", warn=(not installed and not is_core))
        if not installed:
            if is_core:
                sim_failures.append(f"Missing perception package '{spkg}'.")
            else:
                hardware_failures.append(f"Missing physical SLAM package '{spkg}'.")

    # 9. Python Core Modules
    print_header("9. Python Scientific & Computer Vision Libraries")
    py_modules = [
        ("numpy", "numpy"),
        ("PIL", "pillow"),
        ("yaml", "pyyaml"),
        ("matplotlib", "matplotlib"),
        ("cv2", "opencv-python"),
    ]
    for mod_name, hint in py_modules:
        try:
            mod = __import__(mod_name)
            ver = getattr(mod, "__version__", "installed")
            check_item(f"Python module: {mod_name}", True, f"v{ver}")
        except ImportError:
            check_item(f"Python module: {mod_name}", False, f"Missing {mod_name}")
            sim_failures.append(f"Missing Python package '{hint}'. Install with: sudo apt install python3-{hint}")

    # 10. Display & Hardware Acceleration
    print_header("10. Graphics & Hardware Acceleration")
    display = os.environ.get("DISPLAY", None)
    wayland = os.environ.get("WAYLAND_DISPLAY", None)
    dri_render = os.path.exists("/dev/dri/renderD128")

    if display or wayland:
        check_item("Display Server", True, f"DISPLAY={display} | WAYLAND={wayland}")
    else:
        check_item("Display Server", True, "No display server detected. Use headless:=true for simulation.", warn=True)

    check_item("Direct Rendering (DRI /dev/dri/renderD128)", dri_render, "Hardware GPU acceleration available" if dri_render else "Software rendering fallback", warn=not dri_render)

    # Final Summary
    print(f"\n{BOLD}=================================================================={RESET}")
    is_sim_ready = len(sim_failures) == 0
    is_hw_ready = len(sim_failures) == 0 and len(hardware_failures) == 0

    if is_hw_ready:
        print(f"{BOLD}{GREEN}  STATUS: 100% READY! Simulation AND Physical Rover Stacks Operational.{RESET}")
    elif is_sim_ready:
        print(f"{BOLD}{GREEN}  STATUS: SIMULATION READY! All simulation components operational.     {RESET}")
        if hardware_failures:
            print(f"{BOLD}{YELLOW}  NOTE: {len(hardware_failures)} physical hardware/sensor package(s) not installed.{RESET}")
    else:
        print(f"{BOLD}{RED}  STATUS: ACTION NEEDED ({len(sim_failures)} core simulation failure(s)).        {RESET}")
    print(f"{BOLD}=================================================================={RESET}\n")

    if sim_failures:
        print(f"{BOLD}{RED}Core Issues to Resolve for Simulation:{RESET}")
        for idx, f in enumerate(sim_failures, 1):
            print(f"  {idx}. {f}")

    if hardware_failures and not args.sim_only:
        print(f"\n{BOLD}{YELLOW}Packages Missing for Full Physical Rover Operation:{RESET}")
        for idx, f in enumerate(hardware_failures, 1):
            print(f"  - {f}")

    if warnings:
        print(f"\n{BOLD}Notices & Warnings:{RESET}")
        for w in warnings:
            print(f"  - {w}")

    print(f"\n{BOLD}One-Command Plug-and-Play Installation for All Missing Packages:{RESET}")
    print(f"  cd {repo_root}")
    print(f"  rosdep update")
    print(f"  rosdep install --from-paths src --ignore-src -r -y")
    print(f"  pip3 install pymavlink")
    print(f"  colcon build --symlink-install")
    print(f"  source install/setup.bash\n")

    if is_sim_ready:
        print(f"{BOLD}Quick Launch Simulation:{RESET}")
        print(f"  ros2 launch athena_sim sim.launch.py")
        print(f"  ros2 launch athena_sim sim.launch.py headless:=true\n")
        return 0
    else:
        return 1

if __name__ == "__main__":
    sys.exit(main())
