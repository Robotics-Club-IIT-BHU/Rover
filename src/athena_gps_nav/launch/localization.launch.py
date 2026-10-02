"""
localization.launch.py. Dual EKF + navsat_transform (robot_localization).

  ekf_local   (world_frame=odom): VIO velocities + IMU yaw  -> odom->base_link
                                  output: /odometry/local
  ekf_global  (world_frame=map):  same + GPS odometry       -> map->odom
                                  output: /odometry/global
  navsat_transform: /gps/fix + /imu/data + /odometry/global -> /odometry/gps
                    also serves /fromLL used for GPS waypoints.

Config: config/dual_ekf_navsat.yaml

local_only:=true
    Indoor / no-GPS mode.  Drops ekf_global and navsat_transform, and pins
    map -> odom to identity with a static transform instead.

    Nav2 is left configured exactly as it is for GPS work - global_frame
    stays `map` everywhere - because with the static transform in place, map
    IS odom, exactly, by construction.  Nothing has to be rewritten and
    switching back to GPS is just dropping the flag.

    The point is not to save CPU.  It is that with no fix, ekf_global has
    no GPS to fuse, so it dead-reckons the same VIO and gyro that ekf_local
    already does - two independent filters on identical inputs, whose
    estimates slowly diverge for no reason.  That divergence IS the
    map -> odom drift, and indoors it buys you nothing.  Delete the second
    filter and the drift cannot happen.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('athena_gps_nav')
    params = os.path.join(pkg_share, 'config', 'dual_ekf_navsat.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    local_only = LaunchConfiguration('local_only')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('local_only', default_value='false'),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[params, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', 'odometry/local')],
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            condition=UnlessCondition(local_only),
            parameters=[params, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', 'odometry/global')],
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            condition=UnlessCondition(local_only),
            parameters=[params, {'use_sim_time': use_sim_time}],
            remappings=[
                ('imu', '/imu/data'),
                ('gps/fix', '/gps/fix'),
                ('odometry/filtered', 'odometry/global'),
                ('odometry/gps', 'odometry/gps'),
                ('gps/filtered', 'gps/filtered'),
            ],
        ),

        # local_only: nothing else publishes map -> odom now, and Nav2 still
        # plans in map, so tie the two frames together permanently.  Identity
        # is the honest value: with no GPS there is no information anywhere in
        # the system that could distinguish map from odom.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_odom_identity',
            condition=IfCondition(local_only),
            arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
        ),
    ])
