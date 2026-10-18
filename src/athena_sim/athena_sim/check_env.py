#!/usr/bin/env python3
"""
check_env.py

Pre-flight environment diagnostics for athena_sim.
Detects ROS 2 distribution, Gazebo binary and version (Fortress vs Harmonic),
ROS-Gazebo bridge compatibility, Python dependencies, workspace packages,
and simulation assets.
"""

import os
import sys
import shutil
import subprocess
import platform
from typing import List, Tuple

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

def main():
    print(f"\n{BOLD}=================================================================={RESET}")
    print(f"{BOLD}           Athena Simulation Pre-Flight Environment Audit         {RESET}")
    print(f"{BOLD}=================================================================={RESET}")

    failures = []
    warnings = []

    # 1. Operating System
    print_header("1. Operating System & Platform")
    os_name = platform.system()
    distro_info = "Unknown"
    if os.path.exists("/etc/os-release"):
        with open("/etc/os-release") as f:
            for line in f:
                if line.startswith("PRETTY_NAME="):
                    distro_info = line.split("=")[1].strip().strip('"')
    
    is_ubuntu_2204 = "Ubuntu 22.04" in distro_info or "Jammy" in distro_info
    check_item("Operating System", is_ubuntu_2204, f"{distro_info} ({platform.machine()})", warn=not is_ubuntu_2204)
    if not is_ubuntu_2204:
        warnings.append(
            f"Detected {distro_info}. athena_sim is officially validated for Ubuntu 22.04 LTS (Jammy Jellyfish). "
            f"Ensure your ROS 2 and Gazebo libraries are ABI-compatible."
        )

    # 2. ROS 2 Distribution
    print_header("2. ROS 2 Environment")
    ros_distro = os.environ.get("ROS_DISTRO", None)
    if not ros_distro:
        check_item("ROS_DISTRO", False, "Environment variable ROS_DISTRO not set!")
        failures.append("ROS 2 is not sourced. Please run: source /opt/ros/humble/setup.bash")
    elif ros_distro != "humble":
        check_item("ROS_DISTRO", False, f"Detected '{ros_distro}', expected 'humble'", warn=True)
        warnings.append(f"ROS_DISTRO is '{ros_distro}'. athena_sim is built and tuned for ROS 2 Humble.")
    else:
        check_item("ROS_DISTRO", True, f"ROS 2 {ros_distro.capitalize()} active")

    # ROS 2 packages
    required_ros_pkgs = [
        "rclpy",
        "robot_state_publisher",
        "xacro",
        "tf2_ros",
        "ros_gz_bridge",
        "ros_gz_sim",
    ]
    try:
        from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
        for pkg in required_ros_pkgs:
            try:
                path = get_package_share_directory(pkg)
                check_item(f"ROS pkg: {pkg}", True, f"Installed at {path}")
            except PackageNotFoundError:
                check_item(f"ROS pkg: {pkg}", False, "Not found in active ROS environment")
                failures.append(f"Missing ROS package '{pkg}'. Install with: sudo apt install ros-{ros_distro or 'humble'}-{pkg.replace('_', '-')}")
    except ImportError:
        check_item("ament_index_python", False, "Cannot import ament_index_python. Source your ROS setup!")
        failures.append("ament_index_python missing. Source ROS: source /opt/ros/humble/setup.bash")

    # 3. Gazebo Simulator Detection & Version Check
    print_header("3. Gazebo Simulator & Engine")
    gz_bin = shutil.which("gz")
    ign_bin = shutil.which("ign")

    gz_version_str = None
    gz_major_ver = None
    simulator_engine = None

    if gz_bin:
        ret, out = run_cmd(["gz", "sim", "--version"])
        if ret == 0 and "version" in out.lower():
            gz_version_str = out.split("\n")[0]
            simulator_engine = "Gazebo (gz)"
            # Extract major version
            for token in gz_version_str.split():
                if "." in token and token[0].isdigit():
                    gz_major_ver = int(token.split(".")[0])
                    break

    if not gz_version_str and ign_bin:
        ret, out = run_cmd(["ign", "gazebo", "--version"])
        if ret == 0 and "version" in out.lower():
            gz_version_str = out.split("\n")[0]
            simulator_engine = "Ignition (ign)"
            for token in gz_version_str.split():
                if "." in token and token[0].isdigit():
                    gz_major_ver = int(token.split(".")[0])
                    break

    if gz_version_str:
        codename = "Harmonic" if gz_major_ver == 8 else ("Fortress" if gz_major_ver == 6 else f"v{gz_major_ver}")
        check_item("Gazebo Installation", True, f"{simulator_engine} -> {gz_version_str} ({codename})")
    else:
        check_item("Gazebo Installation", False, "No working 'gz sim' or 'ign gazebo' binary found in PATH!")
        failures.append(
            "Gazebo is not installed. Recommended installation:\n"
            "  sudo apt update && sudo apt install -y ros-humble-ros-gzharmonic\n"
            "  or\n"
            "  sudo apt update && sudo apt install -y ros-humble-ros-gz"
        )

    # 4. Python Dependencies
    print_header("4. Python Core Dependencies")
    python_deps = [
        ("numpy", "numpy"),
        ("PIL", "pillow"),
        ("yaml", "pyyaml"),
        ("matplotlib", "matplotlib"),
    ]

    for mod_name, pkg_hint in python_deps:
        try:
            mod = __import__(mod_name)
            ver = getattr(mod, "__version__", "installed")
            check_item(f"Python module: {mod_name}", True, f"v{ver}")
        except ImportError:
            check_item(f"Python module: {mod_name}", False, f"Missing {mod_name}")
            failures.append(f"Missing Python package '{pkg_hint}'. Install with: sudo apt install python3-{pkg_hint} (or pip install {pkg_hint})")

    # 5. Rover Workspace Dependencies
    print_header("5. Athena Workspace Packages")
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ws_root = os.path.dirname(os.path.dirname(base_dir)) # e.g. Rover/src/athena_sim -> Rover
    ws_pkgs = ["athena_drive", "athena_gps_nav", "athena_sim"]
    for wp in ws_pkgs:
        found_in_share = False
        try:
            from ament_index_python.packages import get_package_share_directory
            get_package_share_directory(wp)
            found_in_share = True
        except Exception:
            pass

        if found_in_share:
            check_item(f"Workspace pkg: {wp}", True, "Found in active AMENT_PREFIX_PATH")
        else:
            # Check if present in workspace src or install
            src_pkg = os.path.join(ws_root, "src", wp)
            install_pkg = os.path.join(ws_root, "install", wp)
            if os.path.isdir(install_pkg):
                check_item(f"Workspace pkg: {wp}", False, f"Built in install/{wp} but setup.bash not sourced!", warn=True)
                warnings.append(f"Package '{wp}' is built in install/ but not in active environment. Run: source install/setup.bash")
            elif os.path.isdir(src_pkg):
                check_item(f"Workspace pkg: {wp}", False, f"Found in src/{wp} (needs colcon build)", warn=True)
                warnings.append(f"Package '{wp}' source exists in src/. Run: colcon build --symlink-install && source install/setup.bash")
            else:
                check_item(f"Workspace pkg: {wp}", False, "Package not found in workspace", warn=(wp != "athena_sim"))
                if wp == "athena_sim":
                    failures.append(f"Package '{wp}' not found. Ensure repository is intact.")
                else:
                    warnings.append(f"Optional workspace package '{wp}' not found.")

    # 6. Asset Verification (SDF world, heightmap, textures)
    print_header("6. World & Arena Asset Integrity")
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    worlds_dir = os.path.join(base_dir, "worlds")
    models_dir = os.path.join(base_dir, "models")

    # World files
    sdf_path = os.path.join(worlds_dir, "sproscape.sdf")
    hm_path = os.path.join(worlds_dir, "sproscape_heightmap.png")
    zones_path = os.path.join(worlds_dir, "sproscape_zones.yaml")

    check_item("Sproscape World SDF", os.path.isfile(sdf_path), sdf_path)
    if not os.path.isfile(sdf_path):
        failures.append(f"Missing {sdf_path}. Run generate_terrain to build.")

    check_item("Heightmap Texture", os.path.isfile(hm_path), hm_path)
    if not os.path.isfile(hm_path):
        failures.append(f"Missing heightmap {hm_path}.")

    check_item("IRC Zones Metadata", os.path.isfile(zones_path), zones_path)
    if not os.path.isfile(zones_path):
        failures.append(f"Missing {zones_path}.")

    # Required IRC models
    irc_models = [
        "rado_hammer", "rado_wrench", "rado_toolbox",
        "delivery_marker", "red_zone_marker", "abex_sample_site", "idmo_panel"
    ]
    missing_models = [m for m in irc_models if not os.path.isdir(os.path.join(models_dir, m))]
    if not missing_models:
        check_item("IRC 2027 3D Models", True, f"All {len(irc_models)} models present in models/")
    else:
        check_item("IRC 2027 3D Models", False, f"Missing models: {missing_models}")
        failures.append(f"Missing IRC models: {missing_models}")

    # 7. Display and GPU Diagnostics
    print_header("7. Graphics & Display Environment")
    display = os.environ.get("DISPLAY", None)
    wayland = os.environ.get("WAYLAND_DISPLAY", None)
    dri_render = os.path.exists("/dev/dri/renderD128")

    if display or wayland:
        check_item("Display Server", True, f"DISPLAY={display} | WAYLAND={wayland}")
    else:
        check_item("Display Server", True, "No active X11/Wayland display. Use headless:=true for simulation.", warn=True)

    check_item("Direct Rendering (DRI /dev/dri/renderD128)", dri_render, "Hardware GPU acceleration available" if dri_render else "Software rendering fallback", warn=not dri_render)

    # Summary
    print(f"\n{BOLD}=================================================================={RESET}")
    if not failures and not warnings:
        print(f"{BOLD}{GREEN}  STATUS: ALL CHECKS PASSED! Simulation environment is ready.    {RESET}")
        print(f"{BOLD}=================================================================={RESET}\n")
        print(f"Quick Start:")
        print(f"  ros2 launch athena_sim sim.launch.py")
        print(f"  ros2 launch athena_sim sim.launch.py headless:=true")
        return 0
    elif not failures and warnings:
        print(f"{BOLD}{YELLOW}  STATUS: PASSED WITH WARNINGS ({len(warnings)} warning(s)).                  {RESET}")
        print(f"{BOLD}=================================================================={RESET}\n")
        print(f"{BOLD}Warnings:{RESET}")
        for w in warnings:
            print(f"  - {w}")
        print(f"\nYou may proceed with launching the simulation.")
        return 0
    else:
        print(f"{BOLD}{RED}  STATUS: ENVIRONMENT CHECK FAILED ({len(failures)} critical error(s)).       {RESET}")
        print(f"{BOLD}=================================================================={RESET}\n")
        print(f"{BOLD}{RED}Action Items Required Before Launching:{RESET}")
        for idx, f in enumerate(failures, 1):
            print(f"  {idx}. {f}")
        print(f"\n{BOLD}One-Go Installation Command for Missing Dependencies:{RESET}")
        print(f"  cd ~/Rover")
        print(f"  rosdep update")
        print(f"  rosdep install --from-paths src --ignore-src -r -y")
        print(f"  colcon build --packages-select athena_sim --symlink-install")
        print(f"  source install/setup.bash\n")
        return 1

if __name__ == "__main__":
    sys.exit(main())
