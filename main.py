import pybullet as p
import pybullet_data
import time
import os
import math
import random

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

terrain_path = "mount.blend1.obj"

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
            meshScale=[0.3, 0.2, 0.3]
        ),
        baseVisualShapeIndex=p.createVisualShape(
            shapeType=p.GEOM_MESH,
            fileName=terrain_path,
            meshScale=[0.3, 0.2, 0.3]
        ),
        basePosition=[x, y, 0],
        baseOrientation=p.getQuaternionFromEuler(
            [-math.pi / 2, math.pi, 0]
        )
    )

print("Done")

rover_path = "rover.urdf"  # update if inside folder

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

for i in range(num_joints-5):
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

    print(f"Link ID: {joint_id} | Link Name: {joint_name}")

    # print(f"ID: {joint_id} | Name: {joint_name} | Type: {joint_type}")

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

while True:
    forward(20, 13)
    p.stepSimulation()
    time.sleep(1. / 240.)