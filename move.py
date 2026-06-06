import pybullet as p
import pybullet_data
import time
import math
import random
import numpy as np
import threading

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
import tf_transformations  # from tf_transformations pip package


# ══════════════════════════════════════════════════════════════════════════════
#  CONFIG
# ══════════════════════════════════════════════════════════════════════════════

ROVER_URDF    = "/home/user/roversim/src/drive/urdf/rover.urdf"
TERRAIN_OBJ   = "/home/user/roversim/src/drive/drive/mount.blend1.obj"

# Camera intrinsics
CAM_WIDTH     = 640
CAM_HEIGHT    = 480
CAM_FOV_DEG   = 60.0        # vertical FOV in degrees
CAM_NEAR      = 0.05        # near clip (m)
CAM_FAR       = 20.0        # far  clip (m)
CAM_HZ        = 10          # publish rate (Hz) — every N sim steps

# Derived focal length for CameraInfo
CAM_FY = (CAM_HEIGHT / 2.0) / math.tan(math.radians(CAM_FOV_DEG / 2.0))
CAM_FX = CAM_FY             # square pixels
CAM_CX = CAM_WIDTH  / 2.0
CAM_CY = CAM_HEIGHT / 2.0

SIM_HZ        = 240         # PyBullet step rate
WHEEL_JOINTS  = [2, 5, 7, 10, 13, 15]

# Wheel geometry (used for odometry)
WHEEL_RADIUS    = 0.1       # m  (matches URDF)
WHEEL_SEPARATION = 0.71     # m  (matches diff_drive plugin)


# ══════════════════════════════════════════════════════════════════════════════
#  ROS 2 NODE
# ══════════════════════════════════════════════════════════════════════════════

class RoverBridge(Node):
    """
    Bridges PyBullet ↔ ROS 2.

    Publishers
    ----------
    /camera/color/image_raw      sensor_msgs/Image  (RGB  uint8)
    /camera/depth/image_raw      sensor_msgs/Image  (depth float32, metres)
    /camera/camera_info          sensor_msgs/CameraInfo
    /odom                        nav_msgs/Odometry
    /tf                          (odom → base_link transform)

    Subscribers
    -----------
    /cmd_vel                     geometry_msgs/Twist  (teleop_twist_keyboard)
    """

    def __init__(self):
        super().__init__("rover_pybullet_bridge")

        self.bridge = CvBridge()

        # Publishers
        self.pub_rgb   = self.create_publisher(Image,      "/camera/color/image_raw", 10)
        self.pub_depth = self.create_publisher(Image,      "/camera/depth/image_raw", 10)
        self.pub_info  = self.create_publisher(CameraInfo, "/camera/camera_info",     10)
        self.pub_odom  = self.create_publisher(Odometry,   "/odom",                   10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber — teleop_twist_keyboard publishes here
        self.sub_cmd  = self.create_subscription(
            Twist, "/cmd_vel", self._cmd_vel_cb, 10
        )

        # Teleop state (written by ROS callback, read by sim thread)
        self._cmd_lock   = threading.Lock()
        self._linear_x   = 0.0   # m/s  forward/backward
        self._angular_z  = 0.0   # rad/s  turn

        # Odometry state
        self._odom_x   = 0.0
        self._odom_y   = 0.0
        self._odom_yaw = 0.0

        self.get_logger().info("RoverBridge node ready.")

    # ── cmd_vel callback ──────────────────────────────────────────────────────
    def _cmd_vel_cb(self, msg: Twist):
        with self._cmd_lock:
            self._linear_x  = msg.linear.x
            self._angular_z = msg.angular.z

    def get_cmd(self):
        with self._cmd_lock:
            return self._linear_x, self._angular_z

    # ── Camera info (static, only built once) ─────────────────────────────────
    def _make_camera_info(self, stamp):
        ci = CameraInfo()
        ci.header.stamp    = stamp
        ci.header.frame_id = "camera_optical_link"
        ci.width           = CAM_WIDTH
        ci.height          = CAM_HEIGHT
        ci.distortion_model = "plumb_bob"
        ci.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        ci.k = [CAM_FX, 0.0,   CAM_CX,
                0.0,    CAM_FY, CAM_CY,
                0.0,    0.0,    1.0]
        ci.r = [1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0]
        ci.p = [CAM_FX, 0.0,   CAM_CX, 0.0,
                0.0,    CAM_FY, CAM_CY, 0.0,
                0.0,    0.0,    1.0,    0.0]
        return ci

    # ── Publish RGB + Depth + CameraInfo ──────────────────────────────────────
    def publish_camera(self, rgb: np.ndarray, depth: np.ndarray):
        """rgb: (H,W,3) uint8 | depth: (H,W) float32 metres"""
        stamp = self.get_clock().now().to_msg()

        # RGB
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
        rgb_msg.header.stamp    = stamp
        rgb_msg.header.frame_id = "camera_optical_link"
        self.pub_rgb.publish(rgb_msg)

        # Depth — 32FC1 is what rtabmap expects for metric float depth
        depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding="32FC1")
        depth_msg.header.stamp    = stamp
        depth_msg.header.frame_id = "camera_optical_link"
        self.pub_depth.publish(depth_msg)

        # CameraInfo
        self.pub_info.publish(self._make_camera_info(stamp))

    # ── Publish Odometry + TF ─────────────────────────────────────────────────
    def publish_odometry(self, pos, orn_quat, lin_vel, ang_vel):
        """
        pos       – (x, y, z) world position of base_link
        orn_quat  – (x, y, z, w) orientation
        lin_vel   – (vx, vy, vz) linear  velocity in world frame
        ang_vel   – (wx, wy, wz) angular velocity in world frame
        """
        stamp = self.get_clock().now().to_msg()

        # ── Odometry message ──────────────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id  = "base_link"

        odom.pose.pose.position.x = pos[0]
        odom.pose.pose.position.y = pos[1]
        odom.pose.pose.position.z = pos[2]
        odom.pose.pose.orientation.x = orn_quat[0]
        odom.pose.pose.orientation.y = orn_quat[1]
        odom.pose.pose.orientation.z = orn_quat[2]
        odom.pose.pose.orientation.w = orn_quat[3]

        odom.twist.twist.linear.x  = lin_vel[0]
        odom.twist.twist.linear.y  = lin_vel[1]
        odom.twist.twist.linear.z  = lin_vel[2]
        odom.twist.twist.angular.x = ang_vel[0]
        odom.twist.twist.angular.y = ang_vel[1]
        odom.twist.twist.angular.z = ang_vel[2]

        self.pub_odom.publish(odom)

        # ── TF: odom → base_link ──────────────────────────────────────────────
        tf_msg = TransformStamped()
        tf_msg.header.stamp    = stamp
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id  = "base_link"
        tf_msg.transform.translation.x = pos[0]
        tf_msg.transform.translation.y = pos[1]
        tf_msg.transform.translation.z = pos[2]
        tf_msg.transform.rotation.x = orn_quat[0]
        tf_msg.transform.rotation.y = orn_quat[1]
        tf_msg.transform.rotation.z = orn_quat[2]
        tf_msg.transform.rotation.w = orn_quat[3]
        self.tf_broadcaster.sendTransform(tf_msg)


# ══════════════════════════════════════════════════════════════════════════════
#  PYBULLET HELPERS
# ══════════════════════════════════════════════════════════════════════════════

def setup_pybullet():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -9.81)


def spawn_terrain():
    placed = []
    for _ in range(500):
        for _ in range(100):
            x = random.uniform(-5, 5)
            y = random.uniform(-5, 5)
            if all(math.hypot(x - px, y - py) >= 2.0 for px, py in placed):
                placed.append((x, y))
                break
        else:
            continue
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=p.createCollisionShape(
                p.GEOM_MESH, fileName=TERRAIN_OBJ, meshScale=[0.3, 0.14, 0.3]),
            baseVisualShapeIndex=p.createVisualShape(
                p.GEOM_MESH, fileName=TERRAIN_OBJ, meshScale=[0.3, 0.14, 0.3]),
            basePosition=[x, y, 0],
            baseOrientation=p.getQuaternionFromEuler([-math.pi / 2, math.pi, 0])
        )


def load_rover():
    rover_id = p.loadURDF(
        ROVER_URDF,
        [-7, 0, 0.5],
        p.getQuaternionFromEuler([0, 0, 0]),
        useFixedBase=False
    )

    # Disable default motor friction on every joint
    for i in range(p.getNumJoints(rover_id)):
        p.setJointMotorControl2(rover_id, i,
                                p.VELOCITY_CONTROL,
                                targetVelocity=0, force=0)

    # Differential-bar constraints (same as original)
    p.createConstraint(rover_id, 0,  rover_id, 20,
                       p.JOINT_POINT2POINT, [0,0,0],
                       [0, 0, 0.1], [0, 0, 0.225])
    p.createConstraint(rover_id, 8,  rover_id, 18,
                       p.JOINT_POINT2POINT, [0,0,0],
                       [0, 0, 0.1], [0, 0, 0.225])
    return rover_id


def find_link_index(rover_id, link_name):
    for i in range(p.getNumJoints(rover_id)):
        info = p.getJointInfo(rover_id, i)
        if info[12].decode("utf-8") == link_name:
            return info[0]
    return -1


def set_wheel_velocities(rover_id, left_vel, right_vel, force=5.0):
    """
    wheel_joints = [2, 5, 7, 10, 13, 15]
    First 3 indices → left wheels, last 3 → right wheels.
    Positive velocity = forward.
    """
    for j in WHEEL_JOINTS[:3]:
        p.setJointMotorControl2(rover_id, j,
                                p.VELOCITY_CONTROL,
                                targetVelocity=left_vel, force=force)
    for j in WHEEL_JOINTS[3:]:
        p.setJointMotorControl2(rover_id, j,
                                p.VELOCITY_CONTROL,
                                targetVelocity=right_vel, force=force)


def get_rgbd(rover_id, cam_link_idx, proj_matrix):
    """Render one RGB + metric-depth frame from camera_optical_link."""
    ls = p.getLinkState(rover_id, cam_link_idx, computeForwardKinematics=True)
    cam_pos = ls[4]
    cam_orn = ls[5]

    R = p.getMatrixFromQuaternion(cam_orn)
    forward_vec = [R[2], R[5], R[8]]
    up_vec      = [-R[1], -R[4], -R[7]]
    target      = [cam_pos[i] + forward_vec[i] for i in range(3)]

    view_matrix = p.computeViewMatrix(cam_pos, target, up_vec)

    _, _, rgba_raw, depth_raw, _ = p.getCameraImage(
        CAM_WIDTH, CAM_HEIGHT,
        viewMatrix=view_matrix,
        projectionMatrix=proj_matrix,
        renderer=p.ER_TINY_RENDERER
    )

    rgba  = np.array(rgba_raw,  dtype=np.uint8).reshape(CAM_HEIGHT, CAM_WIDTH, 4)
    rgb   = rgba[:, :, :3]

    z_buf = np.array(depth_raw, dtype=np.float32).reshape(CAM_HEIGHT, CAM_WIDTH)
    depth = CAM_FAR * CAM_NEAR / (CAM_FAR - (CAM_FAR - CAM_NEAR) * z_buf)

    return rgb, depth


# ══════════════════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════════════════

def main():
    # ── ROS 2 init ────────────────────────────────────────────────────────────
    rclpy.init()
    node = RoverBridge()

    # Spin ROS 2 in a background thread so cmd_vel callbacks fire without
    # blocking the simulation loop.
    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # ── PyBullet init ─────────────────────────────────────────────────────────
    setup_pybullet()
    spawn_terrain()
    rover_id = load_rover()

    cam_link_idx = find_link_index(rover_id, "camera_optical_link")
    if cam_link_idx == -1:
        node.get_logger().error("camera_optical_link not found in URDF!")
        return

    proj_matrix = p.computeProjectionMatrixFOV(
        fov=CAM_FOV_DEG,
        aspect=CAM_WIDTH / CAM_HEIGHT,
        nearVal=CAM_NEAR,
        farVal=CAM_FAR
    )

    node.get_logger().info(f"camera_optical_link index: {cam_link_idx}")
    node.get_logger().info("Simulation running. Drive with teleop_twist_keyboard on /cmd_vel")

    # ── Simulation loop ───────────────────────────────────────────────────────
    step_count  = 0
    cam_every   = max(1, int(SIM_HZ / CAM_HZ))   # steps between camera publishes
    dt          = 1.0 / SIM_HZ

    while rclpy.ok():
        # ── 1. Read teleop command ────────────────────────────────────────────
        linear_x, angular_z = node.get_cmd()

        # Convert Twist → individual wheel velocities (differential drive)
        # v_left  = (linear_x - angular_z * L/2) / R
        # v_right = (linear_x + angular_z * L/2) / R
        v_left  = (linear_x - angular_z * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS
        v_right = (linear_x + angular_z * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS

        set_wheel_velocities(rover_id, v_left, v_right)

        # ── 2. Step simulation ────────────────────────────────────────────────
        p.stepSimulation()
        step_count += 1

        # ── 3. Odometry from ground-truth base_link pose ──────────────────────
        pos, orn = p.getBasePositionAndOrientation(rover_id)
        lin_vel, ang_vel = p.getBaseVelocity(rover_id)
        node.publish_odometry(pos, orn, lin_vel, ang_vel)

        # ── 4. Camera at reduced rate ─────────────────────────────────────────
        if step_count % cam_every == 0:
            rgb, depth = get_rgbd(rover_id, cam_link_idx, proj_matrix)
            node.publish_camera(rgb, depth)

        time.sleep(dt)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
