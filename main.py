import pybullet as p
import pybullet_data
import time
import os
import math
import random
import numpy as np

# ── RGB-D camera intrinsics ──────────────────────────────────────────────────
CAM_WIDTH      = 640
CAM_HEIGHT     = 480
CAM_FOV        = 60          # vertical field-of-view in degrees
CAM_NEAR       = 0.05        # near clipping plane  (metres)
CAM_FAR        = 20.0        # far clipping plane   (metres)
CAM_CAPTURE_HZ = 10          # how many sim steps between captures
# ─────────────────────────────────────────────────────────────────────────────

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

plane_id = p.loadURDF("plane.urdf")

p.setGravity(0, 0, -9.81)


# p.createMultiBody(
#     baseMass=0,  # static world
#     baseCollisionShapeIndex=p.createCollisionShape(
#     shapeType=p.GEOM_MESH,
#     fileName="terrain1.obj",
#     meshScale=[.01, .01, .005]
# ),
#     baseVisualShapeIndex=p.createVisualShape(
#     shapeType=p.GEOM_MESH,
#     fileName="terrain1.obj",
#     meshScale=[.01, .01, .005]
# ),
#     basePosition=[-20, 5, -.5]
# )
#
# p.createMultiBody(
#     baseMass=0,  # static world
#     baseCollisionShapeIndex=p.createCollisionShape(
#     shapeType=p.GEOM_MESH,
#     fileName="terrain2.obj",
#     meshScale=[.01, .01, .005]
# ),
#     baseVisualShapeIndex=p.createVisualShape(
#     shapeType=p.GEOM_MESH,
#     fileName="terrain2.obj",
#     meshScale=[.01, .01, .005]
# ),
#     basePosition=[5, 0, -1]
# )
#
# p.createMultiBody(
#     baseMass=0,  # static world
#     baseCollisionShapeIndex=p.createCollisionShape(
#     shapeType=p.GEOM_MESH,
#     fileName="terrain1.obj",
#     meshScale=[.01, .01, .005]
# ),
#     baseVisualShapeIndex=p.createVisualShape(
#     shapeType=p.GEOM_MESH,
#     fileName="terrain1.obj",
#     meshScale=[.01, .01, .005]
# ),
#     basePosition=[-10, 15, -.5]
# )
#
# p.createMultiBody(
#     baseMass=0,  # static world
#     baseCollisionShapeIndex=p.createCollisionShape(
#     shapeType=p.GEOM_MESH,
#     fileName="terrain1.obj",
#     meshScale=[.01, .01, .008]
# ),
#     baseVisualShapeIndex=p.createVisualShape(
#     shapeType=p.GEOM_MESH,
#     fileName="terrain1.obj",
#     meshScale=[.01, .01, .008]
# ),
#     basePosition=[-15, 7, -.5]
# )
#
# p.createMultiBody(
#     baseMass=0,  # static world
#     baseCollisionShapeIndex=p.createCollisionShape(
#     shapeType=p.GEOM_MESH,
#     fileName="terrain3.obj",
#     meshScale=[.01, .01, .009]
# ),
#     baseVisualShapeIndex=p.createVisualShape(
#     shapeType=p.GEOM_MESH,
#     fileName="terrain3.obj",
#     meshScale=[.01, .01, .009]
# ),
#     basePosition=[4, 0, -.5],
#     baseOrientation=p.getQuaternionFromEuler([0, 0, 1.57083]),
# )

terrain_path = "/home/user/roversim/src/drive/drive/mount.blend1.obj"

placed_positions = []

min_dist = 2.0  # tune this

for _ in range(500):

    # Try multiple times to find a non-overlapping location
    for _ in range(100):
        x = random.uniform(-5, 5)
        y = random.uniform(-5, 5)

        valid = True
        for px, py in placed_positions:
            if math.hypot(x - px, y - py) < min_dist:
                valid = False
                break

        if valid:
            placed_positions.append((x, y))
            break
    else:
        print("Couldn't find space for another terrain.")
        continue

    p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=p.createCollisionShape(
            shapeType=p.GEOM_MESH,
            fileName=terrain_path,
            meshScale=[0.3, 0.14, 0.3]
        ),
        baseVisualShapeIndex=p.createVisualShape(
            shapeType=p.GEOM_MESH,
            fileName=terrain_path,
            meshScale=[0.3, 0.14, 0.3]
        ),
        basePosition=[x, y, 0],
        baseOrientation=p.getQuaternionFromEuler(
            [-math.pi / 2, math.pi, 0]
        )
    )


rover_path = "/home/user/roversim/src/drive/urdf/rover.urdf"  # update if inside folder

rover_start_pos = [-7, 0, 0.5]
rover_start_orientation = p.getQuaternionFromEuler([0, 0, 0])

rover_id = p.loadURDF(
    rover_path,
    rover_start_pos,
    rover_start_orientation,
    useFixedBase=False
)


num_joints = p.getNumJoints(rover_id)

print("Total joints:", num_joints)
print("\nJoint list:\n")

for i in range(num_joints):
    info = p.getJointInfo(rover_id, i)

    joint_id = info[0]
    joint_name = info[1].decode("utf-8")
    joint_type = info[2]

    link_index = info[0]
    link_name = info[12].decode("utf-8")  # <-- THIS is link name

    p.setJointMotorControl2(bodyIndex=rover_id,
                            jointIndex=i,
                            controlMode=p.VELOCITY_CONTROL,
                            targetVelocity=0,
                            force=0)

    print(f"Link ID: {link_index} | Link Name: {link_name}")

    # print(f"ID: {joint_id} | Name: {joint_name} | Type: {joint_type}")

# ── Resolve camera_optical_link index ────────────────────────────────────────
# camera_optical_link is the child of camera_optical_joint; we need its link
# index to query its world pose each frame.
camera_link_index = -1
for i in range(p.getNumJoints(rover_id)):
    info = p.getJointInfo(rover_id, i)
    if info[12].decode("utf-8") == "camera_optical_link":
        camera_link_index = info[0]
        break

if camera_link_index == -1:
    raise RuntimeError("camera_optical_link not found in URDF — "
                       "make sure rover.urdf contains the camera links.")

print(f"\ncamera_optical_link index: {camera_link_index}")

# Pre-compute the projection matrix once (never changes)
proj_matrix = p.computeProjectionMatrixFOV(
    fov=CAM_FOV,
    aspect=CAM_WIDTH / CAM_HEIGHT,
    nearVal=CAM_NEAR,
    farVal=CAM_FAR
)
# ─────────────────────────────────────────────────────────────────────────────

wheel_joints = [2, 5, 7, 10, 13, 15]



left_constraint_id = p.createConstraint(
    parentBodyUniqueId=rover_id,
    parentLinkIndex=0,
    childBodyUniqueId=rover_id,
    childLinkIndex=20,
    jointType=p.JOINT_POINT2POINT,
    jointAxis=[0, 0, 0],
    parentFramePosition=[0, 0, 0.1],
    childFramePosition=[0, 0, 0.225]
)

right_constraint_id = p.createConstraint(
    parentBodyUniqueId=rover_id,
    parentLinkIndex=8,
    childBodyUniqueId=rover_id,
    childLinkIndex=18,
    jointType=p.JOINT_POINT2POINT,
    jointAxis=[0, 0, 0],
    parentFramePosition=[0, 0, 0.1],
    childFramePosition=[0, 0, 0.225]
)


def forward(velocity, force):
    for j in wheel_joints:
        p.setJointMotorControl2(
            bodyUniqueId=rover_id,
            jointIndex=j,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=velocity,
            force=force
        )

def backward(velocity, force):
    for j in wheel_joints:
        p.setJointMotorControl2(
            bodyUniqueId=rover_id,
            jointIndex=j,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=-1*velocity,
            force=force
        )

def left(velocity, force):
    for j in wheel_joints[:3]:
        p.setJointMotorControl2(
            bodyUniqueId=rover_id,
            jointIndex=j,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=-1*velocity,
            force=force
        )
    for j in wheel_joints[3:]:
        p.setJointMotorControl2(
            bodyUniqueId=rover_id,
            jointIndex=j,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=velocity,
            force=force
        )

def right(velocity, force):
    for j in wheel_joints[:3]:
        p.setJointMotorControl2(
            bodyUniqueId=rover_id,
            jointIndex=j,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=1*velocity,
            force=force
        )
    for j in wheel_joints[3:]:
        p.setJointMotorControl2(
            bodyUniqueId=rover_id,
            jointIndex=j,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=-1*velocity,
            force=force
        )

def get_rgbd_images():
    """
    Query the world pose of camera_optical_link and render one RGB + depth
    frame.  Returns:
        rgb   – (H, W, 3)  uint8  numpy array
        depth – (H, W)     float32 numpy array, metric depth in metres
    """
    # ── 1. Get camera pose from PyBullet ─────────────────────────────────────
    link_state = p.getLinkState(
        rover_id,
        camera_link_index,
        computeForwardKinematics=True
    )
    cam_pos  = link_state[4]   # world position  (x, y, z)
    cam_orn  = link_state[5]   # world orientation (quaternion x,y,z,w)

    # ── 2. Build view matrix ─────────────────────────────────────────────────
    # In camera_optical_link convention: +Z forward, +X right, +Y down.
    # We derive the forward (+Z) and up (-Y) vectors from the quaternion.
    rot_matrix = p.getMatrixFromQuaternion(cam_orn)   # row-major 3×3
    # Forward direction = third column of rotation matrix (+Z in camera frame)
    forward = [rot_matrix[2], rot_matrix[5], rot_matrix[8]]
    # Camera "up" in world = negative second column (−Y in camera frame → +Y world up)
    up = [-rot_matrix[1], -rot_matrix[4], -rot_matrix[7]]

    target = [cam_pos[0] + forward[0],
              cam_pos[1] + forward[1],
              cam_pos[2] + forward[2]]

    view_matrix = p.computeViewMatrix(cam_pos, target, up)

    # ── 3. Render ─────────────────────────────────────────────────────────────
    _, _, rgba_raw, depth_raw, _ = p.getCameraImage(
        width=CAM_WIDTH,
        height=CAM_HEIGHT,
        viewMatrix=view_matrix,
        projectionMatrix=proj_matrix,
        renderer=p.ER_TINY_RENDERER   # CPU renderer, no GPU needed
    )

    # ── 4. Post-process ───────────────────────────────────────────────────────
    # RGB
    rgba  = np.array(rgba_raw, dtype=np.uint8).reshape(CAM_HEIGHT, CAM_WIDTH, 4)
    rgb   = rgba[:, :, :3]                          # drop alpha channel

    # Depth: PyBullet returns normalised depth in [0,1].
    # Convert to metric depth (metres) using the standard formula:
    #   z_eye = far * near / (far - (far - near) * z_buf)
    z_buf = np.array(depth_raw, dtype=np.float32).reshape(CAM_HEIGHT, CAM_WIDTH)
    depth = CAM_FAR * CAM_NEAR / (CAM_FAR - (CAM_FAR - CAM_NEAR) * z_buf)

    return rgb, depth


# ── Simulation loop ───────────────────────────────────────────────────────────
step_count = 0

while True:
    forward(10, 2)
    p.stepSimulation()
    step_count += 1

    # Capture RGB-D at CAM_CAPTURE_HZ rate
    if step_count % max(1, int(240 / CAM_CAPTURE_HZ)) == 0:
        rgb, depth = get_rgbd_images()

        # ── What you can do with rgb and depth ───────────────────────────────
        # rgb   shape: (480, 640, 3)  dtype: uint8
        # depth shape: (480, 640)     dtype: float32, values in metres

        # Example 1 – print centre pixel info
        cy, cx = CAM_HEIGHT // 2, CAM_WIDTH // 2
        print(f"[step {step_count:6d}] "
              f"centre RGB: {rgb[cy, cx]}  "
              f"centre depth: {depth[cy, cx]:.3f} m")

        # Example 2 – save frames to disk (uncomment to enable)
        # import cv2
        # cv2.imwrite(f"rgb_{step_count:06d}.png",
        #             cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        # np.save(f"depth_{step_count:06d}.npy", depth)

        # Example 3 – display with matplotlib (uncomment to enable)
        # import matplotlib.pyplot as plt
        # fig, axes = plt.subplots(1, 2, figsize=(12, 4))
        # axes[0].imshow(rgb);   axes[0].set_title("RGB");   axes[0].axis("off")
        # axes[1].imshow(depth, cmap="plasma"); axes[1].set_title("Depth (m)")
        # plt.tight_layout(); plt.pause(0.001)

    time.sleep(1. / 240.)
