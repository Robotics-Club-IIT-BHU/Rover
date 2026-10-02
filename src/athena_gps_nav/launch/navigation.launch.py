"""
navigation.launch.py. Nav2 (map-less, GPS/EKF localized).

Wraps nav2_bringup/navigation_launch.py with config/nav2_params.yaml.
No AMCL and no map server: map->odom comes from the global EKF.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('athena_gps_nav')
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')
    params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_launch_dir, 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': params,
                'autostart': 'true',
            }.items(),
        ),
    ])
