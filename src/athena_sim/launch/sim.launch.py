"""
sim.launch.py

Master simulation harness bringup for the Athena rover in Gazebo Fortress.

Launches:
  1. Gazebo Fortress server (headless by default: gz sim -r -s <world>)
  2. Robot State Publisher with athena_sim.urdf.xacro
  3. Robot entity spawner (ros_gz_sim create)
  4. ROS 2 <-> Gazebo Bridge (ros_gz_bridge parameter_bridge)
  5. RTK GNSS simulator node (athena_sim sim_gps_rtk)
  6. VIO simulator node (athena_sim vio_sim)
  7. Dual EKF + NavSat localization stack (athena_gps_nav localization.launch.py)
  8. Optional: Nav2 autonomous navigation (nav2_bringup with nav2_params_sim.yaml)
  9. Optional: IRC 2027 Mission Scorer (athena_sim mission_runner)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_sim_share = get_package_share_directory('athena_sim')
    pkg_nav_share = get_package_share_directory('athena_gps_nav')

    # Default paths
    default_world = os.path.join(pkg_sim_share, 'worlds', 'sproscape.sdf')
    default_bridge_cfg = os.path.join(pkg_sim_share, 'config', 'bridge.yaml')
    default_nav2_cfg = os.path.join(pkg_sim_share, 'config', 'nav2_params_sim.yaml')
    xacro_file = os.path.join(pkg_sim_share, 'urdf', 'athena_sim.urdf.xacro')

    # Launch configuration variables
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    spawn_yaw = LaunchConfiguration('spawn_yaw')
    gps_mode = LaunchConfiguration('gps_mode')
    run_localization = LaunchConfiguration('run_localization')
    run_navigation = LaunchConfiguration('run_navigation')
    run_mission = LaunchConfiguration('run_mission')

    # Robot Description
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' sim:=true']), value_type=str
    )

    # Set Gazebo resource path to discover local models
    models_dir = os.path.join(pkg_sim_share, 'models')
    gz_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[models_dir, ':', os.path.join(pkg_sim_share, 'worlds')]
    )

    # Gazebo simulation launch from ros_gz_sim
    # -s runs headless server, -r runs physics immediately
    gz_args = PythonExpression([
        "'-r ' + ('-s ' if '", headless, "'.lower() == 'true' else '') + '", world, "'"
    ])

    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    # Spawn Athena rover in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_athena',
        output='screen',
        arguments=[
            '-name', 'athena',
            '-topic', 'robot_description',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
            '-Y', spawn_yaw,
        ],
    )

    # ROS <-> Gazebo Bridge using config/bridge.yaml
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': default_bridge_cfg,
            'use_sim_time': use_sim_time,
        }],
    )

    # RTK GNSS error model wrapper
    sim_gps_rtk = Node(
        package='athena_sim',
        executable='sim_gps_rtk',
        name='sim_gps_rtk',
        output='screen',
        parameters=[{
            'gps_mode': gps_mode,
            'use_sim_time': use_sim_time,
            'frame_id': 'gps_link',
        }],
    )

    # VIO Simulator (Option B)
    vio_sim = Node(
        package='athena_sim',
        executable='vio_sim',
        name='vio_sim',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Localization Stack (Dual EKF + NavSat Transform)
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav_share, 'launch', 'localization.launch.py')
        ),
        condition=IfCondition(run_localization),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Navigation Stack (Nav2 with STVL Costmaps)
    try:
        nav2_bringup_share = get_package_share_directory('nav2_bringup')
        navigation = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
            ),
            condition=IfCondition(run_navigation),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': default_nav2_cfg,
                'autostart': 'true',
            }.items(),
        )
    except Exception:
        # Fallback placeholder if nav2_bringup not in path
        navigation = GroupAction(actions=[])

    # IRC 2027 Mission Runner & Live Scorer
    mission_runner = Node(
        package='athena_sim',
        executable='mission_runner',
        name='mission_runner',
        output='screen',
        condition=IfCondition(run_mission),
    )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument('world', default_value=default_world, help='Path to SDF world file'),
        DeclareLaunchArgument('headless', default_value='true', help='Run Gazebo headless (no GUI)'),
        DeclareLaunchArgument('use_sim_time', default_value='true', help='Use simulation clock'),
        DeclareLaunchArgument('spawn_x', default_value='0.0', help='Initial spawn X coordinate'),
        DeclareLaunchArgument('spawn_y', default_value='0.0', help='Initial spawn Y coordinate'),
        DeclareLaunchArgument('spawn_z', default_value='0.20', help='Initial spawn Z coordinate'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0', help='Initial spawn Yaw (rad)'),
        DeclareLaunchArgument('gps_mode', default_value='fixed', help='RTK mode: fixed | float | standalone | no_fix'),
        DeclareLaunchArgument('run_localization', default_value='true', help='Launch dual EKF and navsat_transform'),
        DeclareLaunchArgument('run_navigation', default_value='false', help='Launch Nav2 stack'),
        DeclareLaunchArgument('run_mission', default_value='false', help='Launch IRC 2027 rules engine and scorer'),

        # Nodes and Inclusions
        gz_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        sim_gps_rtk,
        vio_sim,
        localization,
        navigation,
        mission_runner,
    ])
