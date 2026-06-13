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


terrain_path = "mount.blend1.obj"


def spawn_terrain():
    placed_positions = []
    min_dist = 2.0  # tune this

    for _ in range(-8, 8, 2):
        for j in range(-8, 8, 2):
            if abs(_) < 4 and abs(j) < 4:
                continue
            if abs(_) >= 0 and abs(j) > 3 and abs(_) < 4 and abs(j) < 6:
                p.createMultiBody(
                    baseMass=0,
                    baseCollisionShapeIndex=p.createCollisionShape(
                        shapeType=p.GEOM_MESH,
                        fileName=terrain_path,
                        meshScale=[0.3, 1.2, 0.14]
                    ),
                    baseVisualShapeIndex=p.createVisualShape(
                        shapeType=p.GEOM_MESH,
                        fileName=terrain_path,
                        meshScale=[0.3, 1.2, 0.14]
                    ),
                    basePosition=[_, j, 2],
                    baseOrientation=p.getQuaternionFromEuler(
                        [-math.pi / 2, math.pi, 2]
                    ))
                continue

            foo = random.random() + 0.01

            p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=p.createCollisionShape(
                    shapeType=p.GEOM_MESH,
                    fileName=terrain_path,
                    meshScale=[0.3, foo/3, foo]
                ),
                baseVisualShapeIndex=p.createVisualShape(
                    shapeType=p.GEOM_MESH,
                    fileName=terrain_path,
                    meshScale=[0.3, foo/3, foo]
                ),
                basePosition=[_, j, 2],
                baseOrientation=p.getQuaternionFromEuler(
                    [-math.pi / 2, math.pi, 2]
                )
            )


spawn_terrain()


cone_colors = [
    [0, 0, 1, 1],       # blue
    [1, 1, 0, 1],       # yellow
    [0, 1, 0, 1],       # green
    [0.56, 0, 1, 1],    # violet
    [1, 0.41, 0.71, 1], # pink
    [1, 0.5, 0, 1],     # orange
    [1, 0, 0, 1],       # red
    [0, 1, 1, 1],       # cyan
    [0.5, 0, 0.5, 1],   # purple
    [1, 0.84, 0, 1],    # gold
]

cone_ids = []
for color in cone_colors:
    x = random.uniform(-6, 6)
    y = random.uniform(-6, 6)
    with open('cone.txt', 'a') as f:
        f.write(str(x) + ',' + str(y) + '\n')

    col_shape = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.25, height=0.6)
    vis_shape = p.createVisualShape(p.GEOM_CYLINDER, radius=0.25, length=0.6, rgbaColor=color)

    cone_id = p.createMultiBody(
        baseMass=0,  # static so it won't fall
        baseCollisionShapeIndex=col_shape,
        baseVisualShapeIndex=vis_shape,
        basePosition=[x, y, 2.5]
    )
    cone_ids.append(cone_id)


rover_path = "rover.urdf"  # update if inside folder

rover_start_pos = [-7.5, 0, 3.5]
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
    forward(10, 8)
    p.stepSimulation()
    time.sleep(1. / 240.)
