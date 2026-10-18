"""
test_world.launch.py

Minimal launch file for verifying Athena rover spawning and basic sensor
bridging in a lightweight flat world (test_flat.sdf).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_sim_share = get_package_share_directory('athena_sim')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world = os.path.join(pkg_sim_share, 'worlds', 'test_flat.sdf')
    bridge_cfg = os.path.join(pkg_sim_share, 'config', 'bridge.yaml')
    xacro_file = os.path.join(pkg_sim_share, 'urdf', 'athena_sim.urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' sim:=true']), value_type=str
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -s {world}'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_athena',
        arguments=['-name', 'athena', '-topic', 'robot_description', '-z', '0.2'],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_cfg, 'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
    ])
