from controller import Robot, Motor, DistanceSensor
import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, DHLink

TIME_STEP = 64

# Define the UR5e DH parameters using DHLink
ur5e_chain = Chain(name="UR5e", links=[ 
    OriginLink(),
    DHLink(name="Joint1", d=0.089159, a=0, alpha=3.14159 / 2, theta=0),
    DHLink(name="Joint2", d=0, a=-0.425, alpha=0, theta=0),
    DHLink(name="Joint3", d=0, a=-0.39225, alpha=0, theta=0),
    DHLink(name="Joint4", d=0.10915, a=0, alpha=3.14159 / 2, theta=0),
    DHLink(name="Joint5", d=0.09465, a=0, alpha=-3.14159 / 2, theta=0),
    DHLink(name="Joint6", d=0.0823, a=0, alpha=0, theta=0)
])

# Function to compute forward kinematics
def compute_forward_kinematics(joint_angles):
    # Compute the end-effector position using Forward Kinematics
    joint_angles_with_base = [0] + list(joint_angles)  # Add 0 for the OriginLink
    fk = ur5e_chain.forward_kinematics(joint_angles_with_base)
    print("Forward Kinematics - End Effector Position (X, Y, Z):", fk[:3, 3])
    return fk[:3, 3]

# Function to compute inverse kinematics
def compute_inverse_kinematics(target_position):
    # Compute joint angles for the given target position using Inverse Kinematics
    initial_positions = [0] * len(ur5e_chain.links)  # Start with all zeros
    joint_angles = ur5e_chain.inverse_kinematics(target_position, initial_position=initial_positions)
    print("Inverse Kinematics - Joint Angles:", joint_angles[1:])
    return joint_angles[1:]  # Exclude the base joint

# Robot instance
robot = Robot()

# Gripper motor instances
finger1 = robot.getDevice("finger_1_joint_1")
finger2 = robot.getDevice("finger_2_joint_1")
finger3 = robot.getDevice("finger_middle_joint_1")

# Arm motor instances
joints = [
    robot.getDevice("shoulder_lift_joint"),
    robot.getDevice("elbow_joint"),
    robot.getDevice("wrist_1_joint"),
    robot.getDevice("wrist_2_joint")
]

# Set initial position and velocity for the motors
for joint in joints:
    joint.setPosition(float("inf"))
    joint.setVelocity(0.0)

finger1.setPosition(float("inf"))
finger2.setPosition(float("inf"))
finger3.setPosition(float("inf"))
finger1.setVelocity(0.0)
finger2.setVelocity(0.0)
finger3.setVelocity(0.0)

# Distance sensor instance
distance_sensor = robot.getDevice("distance sensor")
distance_sensor.enable(TIME_STEP)

# Variable used to indicate the operational state of the robot
state = 0

# Delay function
def delay(robot, duration):
    start_time = robot.getTime()
    while robot.getTime() < start_time + duration:
        robot.step(1)

# Main loop
while robot.step(TIME_STEP) != -1:
    # Reading input from the distance sensor
    val = distance_sensor.getValue()

    # Operational stages of the robotic arm
    if state == 0:  # Waiting
        print("Waiting")
        if val < 600:
            print("Object detected in range")
            state = 1  # Object detected, ready for gripping

    elif state == 1:  # Gripping
        print("Gripping")
        finger1.setVelocity(3.14)
        finger1.setPosition(1)
        finger2.setVelocity(3.14)
        finger2.setPosition(1)
        finger3.setVelocity(3.14)
        finger3.setPosition(1)
        delay(robot, 0.5)
        print("--> Gripping done, ready for rotation")
        state = 2  # Object gripped, ready to rotate

    elif state == 2:  # Moving to goal
        print("Moving to target position")
        # Target position for the end effector
        target_position = [0.14, 0.7, 0.7]  # Example target (X, Y, Z)
    
        #Compute inverse kinematics to get joint angles
        joint_angles = compute_inverse_kinematics(target_position)
    
        #Move each joint to the calculated angle
        for i, joint in enumerate(joints):
            joint.setVelocity(1.0)
            joint.setPosition(joint_angles[i]-1)
        delay(robot, 1)
    
        #Compute Forward Kinematics for verification
        fk_result = compute_forward_kinematics(joint_angles)

        print(f"Target Position: {target_position}")
        print(f"Forward Kinematics Result: {fk_result}")
    
        #  #Verify if the FK result matches the target position
        error = np.linalg.norm(np.array(target_position) - np.array(fk_result))
        if error < 0.01:  # we can adjust the threshold 
            print("FK Verification: Success! End-effector reached the target position.")
        else:
            print(f"FK Verification: Error! End-effector is off by {error:.4f} meters.")
    
        print("--> Movement to target position complete, ready for release")
        wrist_joint_index = -3  
        joints[wrist_joint_index].setVelocity(0.83)
        joints[wrist_joint_index].setPosition(-2.5307)  # Rotate downward
        delay(robot, 0.5)
    
        print("--> Wrist rotation complete, ready for release")
        state = 3  # Ready for release

    elif state == 3:  # Releasing at goal
        print("Releasing object")
        finger1.setVelocity(3.14)
        finger1.setPosition(0.05)
        finger2.setVelocity(3.14)
        finger2.setPosition(0.05)
        finger3.setVelocity(3.14)
        finger3.setPosition(0.05)
        delay(robot, 0.5)
        print("Object released, returning home")
        state = 4  # Object released, return home

    elif state == 4:  # Returning back to Home
        print("Returning to start position")
        # Start position joint angles
        home_position = [0, 0, 0, 0]  

        for i, joint in enumerate(joints):
            joint.setVelocity(1.0)
            joint.setPosition(home_position[i])
        delay(robot, 1)

        print("--> Returned to start position")
        state = 0  # Back to waiting
