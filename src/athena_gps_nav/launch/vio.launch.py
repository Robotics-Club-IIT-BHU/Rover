"""
vio.launch.py. Rtabmap rgbd_odometry as visual-inertial odometry source.

Publishes nav_msgs/Odometry on /vio/odometry.  publish_tf is FALSE: the
local EKF (robot_localization) owns the odom->base_link transform and fuses
the VIO *velocities* (robust to VO resets).

Uses the apt rtabmap (ros-humble-rtabmap-odom).  NOTE: the old source build
in ~/rtab is ABI-broken since the Aug 2026 ROS apt upgrade (crashes with
std::bad_array_new_length). Do not source it.

Key parameters (per rtabmap_ros maintainer guidance for EKF integration):
  - wait_imu_to_init: true    -> gravity-aligned initialization from madgwick
  - Odom/ResetCountdown: 1    -> auto-reset odometry when tracking is lost
  - publish_null_when_lost: false -> don't feed null poses to the EKF
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': False,
                'wait_imu_to_init': True,
                'approx_sync': True,
                # Without this, approx_sync will happily pair an RGB frame with
                # a depth frame 2 whole frames (66.7 ms at 30 fps) away, and
                # rgbd_odometry computes motion from the mismatch.  Observed
                # 2026-09-02: a *constant* 0.0667 s skew and phantom velocities
                # of 0.7-1.6 m/s on a stationary rover.  0.02 s is rtabmap's own
                # recommendation in the warning it prints for this condition.
                'approx_sync_max_interval': 0.02,
                'queue_size': 20,
                'publish_null_when_lost': False,
                'Odom/ResetCountdown': '1',
                # 20 is the rtabmap default; 12 accepted badly-conditioned
                # geometry, which at half resolution is a poorly constrained
                # PnP.  Frames that now fail simply produce no output and the
                # EKF predicts through the gap -- the desired behaviour.
                'Vis/MinInliers': '20',
                # Ground rover: solve in 3 DoF so depth noise cannot leak
                # into z/roll/pitch and back out as horizontal drift.
                'Reg/Force3DoF': 'true',
                # Ignore far depth returns, which are the noisiest and
                # dominate the apparent motion of a static scene.
                'Vis/MaxDepth': '5.0',
                # Jetson CPU relief: half-resolution feature extraction and a
                # feature cap.  Without these VIO runs ~5 Hz with the full
                # stack up; with them ~15+ Hz.
                'Odom/ImageDecimation': '2',
                'Vis/MaxFeatures': '600',
                # Odom/FilteringStrategy MUST stay 0 (the rtabmap default).
                # It was briefly set to 1 ("Kalman") to smooth VO twist noise
                # -- that was a mistake.  In rtabmap's source the filter is
                # not a downstream output smoother: it overwrites the pose
                # increment AND becomes the next frame's motion guess, which
                # then steers guided feature matching (Vis/CorGuessWinSize)
                # and seeds the iterative PnP.  Measurement therefore depends
                # on filter output, closing a feedback loop that can hold a
                # small non-zero velocity indefinitely -- exactly the steady
                # one-direction drift seen while stationary.  Noise belongs in
                # the downstream EKF and vio_gate, not inside the VO's prior.
                'Odom/FilteringStrategy': '0',
                # Averages recent velocities for the GUESS only, leaving the
                # measured transform untouched: smoothing without the loop.
                'Odom/GuessSmoothingDelay': '0.15',
                # Smaller local feature map: halves F2M update time on the
                # Jetson (RViz running locally eats a full core by itself).
                'OdomF2M/MaxSize': '1000',
            }],
            remappings=[
                ('rgb/image', '/camera/camera/color/image_raw'),
                ('depth/image',
                 '/camera/camera/aligned_depth_to_color/image_raw'),
                ('rgb/camera_info', '/camera/camera/color/camera_info'),
                ('imu', '/camera/imu/filtered'),
                ('odom', '/vio/odometry_raw'),
            ],
        ),

        # Physical-plausibility gate: drops VO samples the rover cannot
        # produce (e.g. a person walking through the camera view reads as
        # 0.7 m/s self-motion).  EKFs consume the gated /vio/odometry.
        Node(
            package='athena_gps_nav',
            executable='vio_gate',
            name='vio_gate',
            output='screen',
        ),
    ])
