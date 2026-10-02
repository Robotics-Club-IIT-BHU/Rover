# Rover

**Athena rover stack (ROS 2 Humble, Jetson):** start at [src/README.md](src/README.md).
It covers bringing the rover up, the Foxglove control panel, motor calibration and
the Pico pin map, with architecture in [src/docs/ARCHITECTURE.md](src/docs/ARCHITECTURE.md)
and contribution rules in [src/CONTRIBUTING.md](src/CONTRIBUTING.md).
Packages: `athena_gps_nav` (sensors, localization, Nav2), `athena_drive` (motors,
calibration, Pico firmware), `athena_remote` (Foxglove panel, goals, remote access).

The sections below are the earlier simulation notes.

## Drive:
Clone the src file and build the package
then run using : ros2 launch drive sim.launch.py

## RtabMap:
Build RTABMAP ROS: https://github.com/introlab/rtabmap_ros
and run in second terminal using : ros2 launch rtabmap_launch rtabmap.launch.py    rgb_topic:=/camera/color/image_raw    depth_topic:=/camera/depth/image_raw    camera_info_topic:=/camera/camera_info    frame_id:=base_link    odom_topic:=/odom    approx_sync:=true 


