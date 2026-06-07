# Rover
## Drive:
Clone the src file and build the package
then run using : ros2 launch drive sim.launch.py

## RtabMap:
Build RTABMAP ROS: https://github.com/introlab/rtabmap_ros
and run in second terminal using : ros2 launch rtabmap_launch rtabmap.launch.py    rgb_topic:=/camera/color/image_raw    depth_topic:=/camera/depth/image_raw    camera_info_topic:=/camera/camera_info    frame_id:=base_link    odom_topic:=/odom    approx_sync:=true 


