from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share = get_package_share_directory('drive')

    urdf_path = os.path.join(pkg_share, 'urdf', 'rover.urdf')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description}
            ]
        ),

        Node(
            package='drive',
            executable='move',   # if installed via setup.py entry_points
            name='move',
            output='screen',
        ),
    ])
