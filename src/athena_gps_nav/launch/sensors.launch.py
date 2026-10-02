"""
sensors.launch.py. Robot model + all sensor drivers.

Starts:
  - robot_state_publisher (athena.urdf.xacro -> static TF tree)
  - realsense2_camera     (D435i: color + aligned depth + pointcloud + IMU)
  - imu_filter_madgwick   (camera IMU -> orientation, for VIO init)
  - pixhawk_bridge        (/gps/fix + /imu/data via pymavlink)
  - pointcloud_downsampler (light cloud for the Nav2 costmaps)

Usage:
  ros2 launch athena_gps_nav sensors.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('athena_gps_nav')
    xacro_file = os.path.join(pkg_share, 'urdf', 'athena.urdf.xacro')

    use_camera = LaunchConfiguration('use_camera')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('use_camera', default_value='true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # --- RealSense D435i ---
        # Pinned to the exact apt-installed binary, NOT resolved via
        # package='realsense2_camera'. This machine also has an unrelated
        # rover project at ~/ros2_ws with its own from-source realsense2_camera
        # build (different robot: mecanum drive, YDLidar, JY901S IMU - nothing
        # to do with Athena's D435i+Pixhawk+Pico setup). ~/.bashrc sources
        # that workspace, so package-based resolution silently picks
        # whichever build is FIRST in AMENT_PREFIX_PATH - which is the wrong
        # one whenever this launches from a normal interactive shell instead
        # of a clean one. Found 2026-08-31 after camera-orientation bugs kept
        # reappearing despite the URDF being correct: the running driver
        # binary was /home/robo/ros2_ws/install/realsense2_camera/..., not
        # /opt/ros/humble's, and there is no guarantee the two builds share
        # the same defaults/extrinsics handling. An absolute path here makes
        # it deterministic regardless of whose shell launches this.
        Node(
            executable='/opt/ros/humble/lib/realsense2_camera/realsense2_camera_node',
            name='camera',
            namespace='camera',
            output='screen',
            condition=IfCondition(use_camera),
            parameters=[{
                'camera_name': 'camera',
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_color': True,
                'enable_depth': True,
                'align_depth.enable': True,
                'enable_sync': True,
                'enable_gyro': True,
                'enable_accel': True,
                'unite_imu_method': 2,            # linear interpolation
                # The pointcloud filter's parameter namespace is named after
                # the librealsense processing block, which differs by build:
                # 'pointcloud' on x86, 'pointcloud__neon_' on this Jetson
                # (NEON-accelerated). Setting the wrong one fails SILENTLY.
                # No cloud is produced and the costmaps stay empty, so set
                # both and let the unused one be ignored.
                'pointcloud.enable': True,
                'pointcloud__neon_.enable': True,
                'pointcloud__neon_.ordered_pc': False,
                # Decimate depth 2x BEFORE the pointcloud block: generating
                # the cloud is the most expensive stage on the Jetson, and
                # obstacle marking at 5 cm costmap resolution does not need
                # full depth resolution.
                'decimation_filter.enable': True,
                'decimation_filter.filter_magnitude': 2,
                # 15 fps: VO only needs ~10 Hz; 30 fps starves the Jetson CPU
                # (VO update times hit 0.2 s and tracking drops out)
                'depth_module.depth_profile': '640x480x15',
                'rgb_camera.color_profile': '640x480x15',
                'initial_reset': True,
            }],
        ),

        # The realsense node roots its TF tree at camera_camera_link
        # (<camera_name>_camera_link naming); tie it to the URDF camera_link.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_link_alias',
            arguments=['--frame-id', 'camera_link',
                       '--child-frame-id', 'camera_camera_link'],
        ),

        # --- Madgwick filter: camera IMU -> orientation (for VIO init) ---
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='camera_imu_filter',
            output='screen',
            parameters=[{
                'use_mag': False,
                'world_frame': 'enu',
                'publish_tf': False,
            }],
            remappings=[
                ('imu/data_raw', '/camera/camera/imu'),
                ('imu/data', '/camera/imu/filtered'),
            ],
        ),

        # --- Pixhawk: GPS + IMU ---
        Node(
            package='athena_gps_nav',
            executable='pixhawk_bridge',
            name='pixhawk_bridge',
            output='screen',
        ),

        # --- Downsampled cloud for costmaps ---
        Node(
            package='athena_gps_nav',
            executable='pointcloud_downsampler',
            name='pointcloud_downsampler',
            output='screen',
        ),
    ])
