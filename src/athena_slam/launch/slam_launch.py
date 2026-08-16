import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Parameters for easy adjustment
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 1. STATIC TRANSFORMS (The "Virtual" Robot Model)
    # Arguments: x y z yaw pitch roll frame_id child_frame_id
    
    # Transform: base_link -> camera_link
    # EDIT THESE: x=0.1 (10cm forward), y=0, z=0.2 (20cm up)
    tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_tf',
        arguments=['0.1', '0', '0.2', '0', '0', '0', 'base_link', 'camera_link']
    )

    # Transform: base_link -> imu_link (Where your Pixhawk/MAVROS IMU is)
    # Usually the IMU is at the center of the rover
    tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_link_frd'] 
    )

    # 2. REALSENSE NODE
    # We use the raw executable since we are in a custom python launch
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        parameters=[{
            'enable_gyro': True,
            'enable_accel': True,
            'unite_imu_method': 2, # 2 = Linear Interpolation
            'enable_infra1': True,
            'enable_infra2': True,
            'enable_color': True,
            'enable_depth': True,
            'align_depth.enable': True,
            'pointcloud.enable': False, # RTAB-Map generates its own
            'depth_module.profile': '640x480x30',
            'rgb_module.profile': '640x480x30',
        }]
    )

    # 3. VISUAL ODOMETRY (The "Fake" Encoders)
    visual_odometry_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'wait_imu_to_init': True,
            'approx_sync': True,
            'queue_size': 20,
            'Vis/MinInliers': '12', # Sensitivity for tracking
        }],
        remappings=[
            ('rgb/image', '/camera/camera/color/image_raw'),
            ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('imu', '/mavros/imu/data')
        ]
    )

    # 4. RTAB-MAP SLAM NODE
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'subscribe_depth': True,
            'subscribe_odom_info': True,
            'approx_sync': True,
            'use_action_for_goal': True,
            'wait_imu_to_init': True,
            'Grid/FromDepth': 'true', # Create 2D map from depth camera
            'Reg/Strategy': '1',      # 0=Visual, 1=ICP (if you had LIDAR), 2=Both
            'Optimizer/GravitySigma': '0.3', # Trust the IMU for leveling
        }],
        remappings=[
            ('rgb/image', '/camera/camera/color/image_raw'),
            ('depth/image', '/camera/camera/aligned_depth_to_color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('imu', '/mavros/imu/data')
        ],
        arguments=['-d'] # Warning: deletes previous map on every launch
    )

    # 5. RVIZ FOR VISUALIZATION
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(os.getcwd(), 'athena.rviz')], # Optional: save a config
        condition=None 
    )

    return LaunchDescription([
        tf_base_to_camera,
        tf_base_to_imu,
        realsense_node,
        visual_odometry_node,
        rtabmap_node,
        #rviz_node
    ])
