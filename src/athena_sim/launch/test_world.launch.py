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
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_sim_share = get_package_share_directory('athena_sim')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    default_world = os.path.join(pkg_sim_share, 'worlds', 'test_flat.sdf')
    bridge_cfg = os.path.join(pkg_sim_share, 'config', 'bridge.yaml')
    xacro_file = os.path.join(pkg_sim_share, 'urdf', 'athena_sim.urdf.xacro')

    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Resource paths for Gazebo to discover local models and textures
    models_dir = os.path.join(pkg_sim_share, 'models')
    worlds_dir = os.path.join(pkg_sim_share, 'worlds')
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[models_dir, ':', worlds_dir]
    )
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[models_dir, ':', worlds_dir]
    )

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' sim:=true']), value_type=str
    )

    gz_args = PythonExpression(["'-r -s ' + '", world, "'"])
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
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

    imu_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sim_imu_frame_bridge',
        arguments=['--frame-id', 'imu_link', '--child-frame-id', 'athena/base_link/imu_sensor'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    camera_frame_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sim_camera_frame_bridge',
        arguments=['--frame-id', 'camera_link', '--child-frame-id', 'athena/base_link/rgbd_camera'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world, description='Path to SDF world file'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gz_resource_path,
        ign_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        imu_frame_bridge,
        camera_frame_bridge,
    ])
