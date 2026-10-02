"""foxglove.launch.py, the operator front end: bridge + the nodes behind the panels.

Serves the whole ROS graph as a WebSocket on ONE TCP port. That matters on a
network that blocks DDS: DDS needs multicast discovery plus a wide range of
UDP ports, which campus WiFi almost always drops. A single TCP port goes
through an SSH tunnel, so if `ssh` works, this works.

    ros2 launch athena_remote foxglove.launch.py

Then from your laptop:
    ssh -N -L 8765:localhost:8765 robo@<jetson-ip>
and point Foxglove at  ws://localhost:8765

Alongside the bridge this starts the seven nodes the control panel drives.
They live here rather than in athena_gps_nav because they exist to serve an
operator, not to navigate: the rover reaches every goal perfectly well
without them, it just has no convenient way to be told where to go.

    goal_manager      lat/lon (typed) or a 3D click -> Nav2 goal in map
    waypoint_manager  mark a place, drive back to it later
    trajectory        where the rover has actually been, in 3D and on the map
    teleop_mux        manual driving with a speed limit you can change live
    panel_camera      the colour camera as throttled JPEG for the Image panel
    panel_cloud       a thinned depth cloud for the 3D panel
    nav_status        why the last Nav2 goal ended, in one line

Args:
    operator_nodes:=false   bridge only, no panel support nodes
    address:=0.0.0.0        listen on the network instead of loopback
    topic_whitelist:=[...]  override the light default (see below)
    camera_rate:=5.0        panel camera frames per second
    camera_quality:=70      panel camera JPEG quality (1-100)
    camera_width:=0         panel camera width in px, 0 = native 640
    cloud_rate:=2.0         panel point cloud messages per second
    cloud_voxel:=0.10       panel point cloud: one point per voxel of this size (m)
    cloud_max_points:=2000  panel point cloud: point cap per message

Lightweight by default.  The bridge's WebSocket server forwards every
message exactly as published: it does NOT transcode images to H.264.  That
only happens in its separate remote-access (WebRTC) gateway, which needs a
Foxglove device token and is off here.  So the raw colour image,
640x480 rgb8 = 921,672 bytes a frame at 15 Hz = 13.8 MB/s, went down the
tunnel as-is.  Measured 2026-10-02, the SSH tunnel over campus WiFi carried
4.7-7.0 MB/s: every queue between the bridge and the laptop (about 12 MB of
kernel socket buffers) sat full, and the costmap and TF waited seconds
behind the video in the same TCP stream.  The default whitelist therefore
serves the camera only as /athena/camera/compressed (panel_camera, about
0.15-0.3 MB/s), never image_raw.  The depth cloud is served only as
/athena/points_preview (panel_cloud: about 1,500 points at 2 Hz, 35 KB/s),
not as points_downsampled, the costmap's cloud, which measured 78 KB at
11-12 Hz = 0.9 MB/s, about 80 % of the panel's traffic once the camera
was compressed.  `['.*']` serves
everything, raw image included, and is only sensible on a wired LAN:

    ros2 launch athena_remote foxglove.launch.py \\
      topic_whitelist:="['.*']"
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Everything the control panel needs, with the two heavy streams replaced by
# panel-only copies made on the Jetson: the depth cloud is served only as
# /athena/points_preview (panel_cloud, voxel-thinned, 2 Hz) and the colour
# camera only as /athena/camera/compressed (panel_camera, JPEG, 5 Hz), both
# covered by '^/athena/.*'. points_downsampled (the costmap's own cloud,
# 0.9 MB/s) and the raw images are deliberately NOT listed. The raw
# /camera/camera/color/image_raw in particular: the bridge's
# WebSocket server does not transcode it (video_transcode_topic_denylist
# applies to the remote-access gateway only), so listing it would put
# 13.8 MB/s of raw RGB back on a 5-7 MB/s tunnel. Costmaps, TF, GPS, path
# and every athena/* control topic (goals, waypoints, teleop, status) round
# out the rest.
#
# The second block below (added 2026-09-02) is the DIAGNOSTIC set.  Every one
# of these is a tiny message - text, a twist, a pose, a polygon - so together
# they cost a rounding error next to the point cloud and video that dominate
# the link, and they are exactly what you need to tell "Nav2 is commanding the
# wrong thing" from "Nav2 is commanding the right thing and the rover is not
# doing it".  Without them the operator is debugging blind:
#   /goal_pose ................ what frame_id your click ACTUALLY published in
#   /cmd_vel_nav .............. the controller's raw output, BEFORE the
#                               velocity smoother - if linear.x is pinned at 0
#                               while angular.z saturates, that is the
#                               controller refusing to drive, not the motors
#   /vio/odometry{,_raw} ...... VIO health before and after vio_gate
#   /imu/data ................. Pixhawk gyro/attitude
#   /local_costmap/published_footprint  the footprint Nav2 is ACTUALLY using
#   /rosout ................... node warnings in a Log panel (vio_gate drops,
#                               "Rotate to heading", planner failures)
#   /diagnostics .............. both EKFs publish here (print_diagnostics)
#   /navigate_to_pose/_action/* goal accepted/rejected, distance remaining,
#                               and which recovery fired
LIGHT_TOPIC_WHITELIST = (
    "['^/tf$', '^/tf_static$', '^/robot_description$', "
    "'^/odometry/(local|global)$', "
    "'^/(local|global)_costmap/costmap$', "
    "'^/gps/fix$', '^/plan$', '^/cmd_vel$', '^/athena/.*', "
    # --- diagnostic set ---
    "'^/goal_pose$', '^/cmd_vel_nav$', '^/vio/odometry(_raw)?$', "
    "'^/imu/data$', '^/local_costmap/published_footprint$', "
    "'^/rosout$', '^/diagnostics$', "
    "'^/navigate_to_pose/_action/(feedback|status)$']"
)


def generate_launch_description():
    operator = LaunchConfiguration('operator_nodes')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8765'),
        # Bind to loopback by default: the SSH tunnel terminates locally, so
        # nothing needs to listen on the network. Set 0.0.0.0 only on a
        # network you trust - the bridge has no authentication.
        DeclareLaunchArgument('address', default_value='127.0.0.1'),
        DeclareLaunchArgument(
            'topic_whitelist',
            default_value=LIGHT_TOPIC_WHITELIST,
            description='regex list of topics to serve. Default excludes the '
                         'raw camera image, depth image and full point cloud; '
                         "pass ['.*'] for everything"),
        DeclareLaunchArgument('operator_nodes', default_value='true'),
        DeclareLaunchArgument('camera_rate', default_value='5.0',
                              description='panel camera JPEG frames per second'),
        DeclareLaunchArgument('camera_quality', default_value='70',
                              description='panel camera JPEG quality, 1-100'),
        DeclareLaunchArgument('camera_width', default_value='0',
                              description='panel camera width in px, 0 = native'),
        DeclareLaunchArgument('cloud_rate', default_value='2.0',
                              description='panel point cloud messages per second'),
        DeclareLaunchArgument('cloud_voxel', default_value='0.10',
                              description='panel point cloud voxel size in m'),
        DeclareLaunchArgument('cloud_max_points', default_value='2000',
                              description='panel point cloud point cap'),
        DeclareLaunchArgument(
            'trajectory_source', default_value='/odometry/global',
            description='/odometry/local if there is no GPS fix'),
        DeclareLaunchArgument('max_speed', default_value='0.35'),
        DeclareLaunchArgument('max_turn', default_value='0.8'),
        # 0 = one thread per hardware core. On a 6-core Jetson already
        # loaded with rtabmap + 2x EKF + Nav2, letting the bridge grab that
        # many competes with the actual nav stack; 3 gives it more headroom
        # than the bare minimum for transcoding video + serializing the
        # point cloud without fully matching the nav stack's appetite.
        DeclareLaunchArgument('num_threads', default_value='3'),
        # Default 10: how many messages of a topic can queue before the
        # bridge sends them. Lower means less buffering/memory and fresher
        # data reaching the panel instead of catching up through a backlog,
        # at the cost of not replaying missed messages on a stall.
        DeclareLaunchArgument('max_qos_depth', default_value='2'),
        # The lag fix (2026-08-31), take 2. Default 1024 caused ~680 MB of
        # buffered backlog and a client watching it slowly play catch-up
        # instead of seeing live data. But this is ONE SHARED outbox queue
        # across every subscribed channel, not per-topic - dropping it to
        # 10 instantly saturated it (TF alone runs ~15 Hz, plus point
        # cloud, video, costmap, odometry all sharing the same 10 slots)
        # and starved nearly everything ("outbox ... full" in the bridge
        # log, no TF/odom/cloud reaching the client at all). 200 splits
        # the difference: enough headroom across ~20+ channels to not
        # starve anything, far short of the 1024 that caused the original
        # multi-hundred-MB backlog.
        DeclareLaunchArgument('message_backlog_size', default_value='200'),

        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'address': LaunchConfiguration('address'),
                'topic_whitelist': LaunchConfiguration('topic_whitelist'),
                # Nav2's action topics (/navigate_to_pose/_action/feedback and
                # /status - goal accepted/rejected, distance remaining, which
                # recovery fired) are HIDDEN topics: `ros2 topic list` omits
                # them without --include-hidden-topics, and foxglove_bridge
                # drops them too unless this is true.  Whitelisting them alone
                # is not enough.  topic_whitelist still gates what is actually
                # served, so this does not open the floodgates.
                'include_hidden': True,
                'num_threads': LaunchConfiguration('num_threads'),
                'max_qos_depth': LaunchConfiguration('max_qos_depth'),
                'message_backlog_size': LaunchConfiguration('message_backlog_size'),
                # Lowered from 10 MB (2026-08-31). Note that this cannot
                # bound the latency on its own: when the link is saturated,
                # the kernel adds its own ~4 MB send buffer, sshd's ~6 MB
                # receive buffer and SSH's ~2 MB window on top (measured
                # 2026-10-02, ~12 MB = 2 s at 6 MB/s). The only real cure is
                # keeping the panel's total well under the link rate.
                'send_buffer_limit': 4000000,
                'use_compression': True,
                # clientPublish is what lets the panels send goals and teleop;
                # without it the control panel is read-only.
                'capabilities': ['clientPublish', 'connectionGraph',
                                 'assets', 'services', 'parameters'],
            }],
        ),

        Node(
            package='athena_remote', executable='goal_manager',
            name='goal_manager', output='screen',
            condition=IfCondition(operator),
        ),
        Node(
            package='athena_remote', executable='waypoint_manager',
            name='waypoint_manager', output='screen',
            condition=IfCondition(operator),
        ),
        Node(
            package='athena_remote', executable='trajectory',
            name='trajectory', output='screen',
            parameters=[{'odom_topic': LaunchConfiguration('trajectory_source')}],
            condition=IfCondition(operator),
        ),
        Node(
            package='athena_remote', executable='teleop_mux',
            name='teleop_mux', output='screen',
            parameters=[{
                'max_speed': LaunchConfiguration('max_speed'),
                'max_turn': LaunchConfiguration('max_turn'),
            }],
            condition=IfCondition(operator),
        ),
        # Idle (no subscription to the camera at all) until the Image panel
        # subscribes to /athena/camera/compressed.
        Node(
            package='athena_remote', executable='panel_camera',
            name='panel_camera', output='screen',
            parameters=[{
                'rate_hz': ParameterValue(
                    LaunchConfiguration('camera_rate'), value_type=float),
                'jpeg_quality': ParameterValue(
                    LaunchConfiguration('camera_quality'), value_type=int),
                'max_width': ParameterValue(
                    LaunchConfiguration('camera_width'), value_type=int),
            }],
            condition=IfCondition(operator),
        ),
        # Idle until the 3D panel subscribes to /athena/points_preview. The
        # costmaps keep reading the full points_downsampled cloud.
        Node(
            package='athena_remote', executable='panel_cloud',
            name='panel_cloud', output='screen',
            parameters=[{
                'rate_hz': ParameterValue(
                    LaunchConfiguration('cloud_rate'), value_type=float),
                'voxel_size': ParameterValue(
                    LaunchConfiguration('cloud_voxel'), value_type=float),
                'max_points': ParameterValue(
                    LaunchConfiguration('cloud_max_points'), value_type=int),
            }],
            condition=IfCondition(operator),
        ),
        # Reads Nav2's action status, behaviour-tree log and /rosout and says
        # why each goal ended on /athena/nav_status (Goal tab).  Covers every
        # goal, whoever sent it.  Subscribes only to event topics; no /tf.
        Node(
            package='athena_remote', executable='nav_status',
            name='nav_status', output='screen',
            condition=IfCondition(operator),
        ),
    ])
