import roboticstoolbox as rtb
from roboticstoolbox.robot.RobotKinematics import *
import spatialmath as sm


# Load the robot file from id7l class
robot = rtb.models.URDF.id7l()

# Display the robot structure
print(robot)

# Optionally: You can use the robot's kinematics and dynamics features
print(robot.qr)  # Initial joint positions

robot.plot(robot.qr, block=True) # Visualize the robot in the joint config

# T = robot.fkine([0.3, 0.3, 0.3, -2, 0.5, 0.5])

# inverse = robot.ik_LM(T)

# print(inverse)