"""
bench_test.launch.py. Validate the full pipeline WITHOUT hardware.

Runs fake sensors (rectangle loop) + dual EKF + navsat_transform + Nav2 +
motor bridge in dry_run.  No camera, no Pixhawk, no GPS reception needed.

    ros2 launch athena_gps_nav bench_test.launch.py

Then check accuracy with:
    ros2 run athena_gps_nav localization_monitor --ros-args -p duration:=90.0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('athena_gps_nav')
    launch_dir = os.path.join(pkg_share, 'launch')
    xacro_file = os.path.join(pkg_share, 'urdf', 'athena.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='athena_gps_nav',
            executable='fake_gps_imu',
            name='fake_gps_imu',
            output='screen',
        ),
        # Same motor bridge the real rover uses, in dry_run: the bench test is
        # only worth anything if it exercises the code that actually ships.
        Node(
            package='athena_drive',
            executable='motor_bridge',
            name='motor_bridge',
            output='screen',
            parameters=[{'dry_run': True}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'localization.launch.py')),
        ),
        TimerAction(
            period=6.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'navigation.launch.py')),
                ),
            ],
        ),
    ])
