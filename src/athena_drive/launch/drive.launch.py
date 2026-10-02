"""drive.launch.py, the motor bridge on its own.

    ros2 launch athena_drive drive.launch.py                 # live
    ros2 launch athena_drive drive.launch.py dry_run:=true   # log only
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='false'),
        # -1 = use the saved calibration (~/.config/athena_drive/params.yaml).
        # Pass a real value only to override it for one run.
        DeclareLaunchArgument('track_width', default_value='-1.0'),
        DeclareLaunchArgument('max_wheel_speed', default_value='-1.0'),
        DeclareLaunchArgument('min_pwm', default_value='-1'),
        DeclareLaunchArgument('max_pwm', default_value='-1'),
        DeclareLaunchArgument('profile_enabled', default_value='true'),
        DeclareLaunchArgument('max_accel', default_value='0.5'),
        DeclareLaunchArgument('max_jerk', default_value='1.0'),
        DeclareLaunchArgument('max_ang_accel', default_value='1.5'),
        DeclareLaunchArgument('max_ang_jerk', default_value='3.0'),
        Node(
            package='athena_drive',
            executable='motor_bridge',
            name='motor_bridge',
            output='screen',
            parameters=[{
                'dry_run': LaunchConfiguration('dry_run'),
                'track_width': LaunchConfiguration('track_width'),
                'max_wheel_speed': LaunchConfiguration('max_wheel_speed'),
                'min_pwm': LaunchConfiguration('min_pwm'),
                'max_pwm': LaunchConfiguration('max_pwm'),
                'profile_enabled': LaunchConfiguration('profile_enabled'),
                'max_accel': LaunchConfiguration('max_accel'),
                'max_jerk': LaunchConfiguration('max_jerk'),
                'max_ang_accel': LaunchConfiguration('max_ang_accel'),
                'max_ang_jerk': LaunchConfiguration('max_ang_jerk'),
            }],
        ),
    ])
