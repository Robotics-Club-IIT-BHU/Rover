"""
bringup.launch.py. Full Athena GPS navigation stack on the real rover.

    sensors  -> vio -> dual EKF + navsat -> Nav2 -> motor bridge

This is THE one command that starts the rover.  It brings up the sensors,
localization, Nav2, the motors, and the Foxglove front end, in that order,
staggered so each layer has what it depends on before it starts.

    ros2 launch athena_gps_nav bringup.launch.py

Args:
    dry_run:=true       motor commands logged, not sent (rover will not move)
    use_camera:=false   skip the RealSense (no VIO. GPS+IMU only, degraded)
    foxglove:=false     no operator front end (headless / lower CPU)
    local_only:=true    indoor / no-GPS mode: drops ekf_global and
                        navsat_transform and pins map to odom. Use this for
                        local obstacle avoidance testing; see
                        localization.launch.py for why it is not just a
                        CPU saving.

After launch, send GPS waypoints with:
    ros2 run athena_gps_nav gps_waypoint_follower \
        --ros-args -p waypoints_file:=<yaml>
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    pkg_share = get_package_share_directory('athena_gps_nav')
    launch_dir = os.path.join(pkg_share, 'launch')

    dry_run = LaunchConfiguration('dry_run')
    use_camera = LaunchConfiguration('use_camera')
    foxglove = LaunchConfiguration('foxglove')
    local_only = LaunchConfiguration('local_only')

    # Which odometry the operator panel draws its travelled trail from.
    # ekf_global only exists when local_only is false, so pick accordingly.
    # Accepts the same spellings IfCondition does, so local_only:=True and
    # local_only:=1 behave like local_only:=true rather than silently
    # falling through to the GPS topic.
    trajectory_source = PythonExpression([
        "'/odometry/local' if '", local_only,
        "'.lower() in ('true', '1') else '/odometry/global'"])

    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='false'),
        DeclareLaunchArgument('use_camera', default_value='true'),
        DeclareLaunchArgument('foxglove', default_value='true'),
        DeclareLaunchArgument(
            'local_only', default_value='false',
            description='indoor / no-GPS mode: drop ekf_global and '
                        'navsat_transform, pin map to odom, and draw the '
                        'operator trail from /odometry/local'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'sensors.launch.py')),
            launch_arguments={'use_camera': use_camera}.items(),
        ),

        # VIO needs the camera streams up first
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'vio.launch.py')),
                    condition=IfCondition(use_camera),
                ),
            ],
        ),

        TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'localization.launch.py')),
                    launch_arguments={'local_only': local_only}.items(),
                ),
            ],
        ),

        TimerAction(
            period=12.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'navigation.launch.py')),
                ),
            ],
        ),

        # athena_drive's motor bridge: jerk-limited /cmd_vel -> "V <l> <r>" PWM
        # over USB to the Pico.  It reads the saved calibration written by
        # `ros2 run athena_drive calibrate`, so motor inversions and measured
        # speeds survive a reboot and do not need re-entering here.
        #
        # This deliberately does NOT use the older `drive` package, which
        # hardcodes /dev/ttyACM1 -- that enumerates as the Pixhawk about half
        # the time, so it would stream motor commands at the autopilot.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('athena_drive'),
                    'launch', 'drive.launch.py')),
            launch_arguments={'dry_run': dry_run}.items(),
        ),

        # Operator front end last: it only serves what already exists, so
        # starting it early would just mean a panel full of missing topics.
        # Binds to 127.0.0.1. Reach it through an SSH tunnel, see
        # athena_remote/README.md.
        TimerAction(
            period=15.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('athena_remote'),
                            'launch', 'foxglove.launch.py')),
                    condition=IfCondition(foxglove),
                    # foxglove.launch.py's trajectory node defaults to
                    # /odometry/global, which is published by ekf_global -
                    # and local_only does not run ekf_global at all.  Left
                    # alone, the travelled-trail panel would sit empty
                    # forever with nothing to indicate why.
                    launch_arguments={
                        'trajectory_source': trajectory_source}.items(),
                ),
            ],
        ),
    ])
