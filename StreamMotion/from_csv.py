import csv
import numpy as np

from src.client import *
from src.utils import *

# For Joint Space based calculations
import roboticstoolbox as rtb
import spatialmath as sm

def derivative_array(array, dt = 0.008):

    # np.diff finds the difference between subsequent rows of the array
    # allowing for efficient calculation for derivatives (pos -> vel -> acc -> jerk)

    d_array = np.diff(array, axis=0) / dt 

    # Pad the beginning with zero to maintain the shape of array
    np.insert(d_array, 0, 0)
    
    return d_array

def joint_angles(xy_point, wpr_pos):
    
    # Obtain XYZ position
    X = 0
    Y = xy_point[0] / 1000
    Z = xy_point[1] / 1000

    # Obtain WPR position in radians
    W = np.radians(wpr_pos[0])
    P = np.radians(wpr_pos[1])
    R = np.radians(wpr_pos[2])

    T = sm.SE3(X, Y, Z) * sm.SE3.RPY([W,P,R], order='zyx') # Create SE3 transform for inverse kinematics

    return T

def rad_to_degrees(rad_array):
    deg_list = [] # Initialize empty list for easy appending
    starting_degrees = [90, -18.8, -10.2, 0, -63.8, -90]

    for i, array in enumerate(rad_array):
        
        deg = np.degrees(array) % 360 # Convert the radian array to degrees within 0 to 360
        deg_list.append(deg) # Append to list

    deg_array = np.array(deg_list) # Convert list to numpy array to keep functionality

    return deg_array


x, y, z = [],[],[] # initialize blank lists for each coordinate
init_pos = [0, 350, 300]
robot = rtb.models.URDF.id7l()

i = input("Enter number for signal file: ") # 1 or 2

# Open csv file and read in x,y,z data
with open(f'signal{i}.csv', mode='r') as file:

    csv_reader = csv.reader(file)
    for row in csv_reader:
        x.append(float(row[1]))
        y.append(float(row[2]))
        z.append(float(row[3])) # just zeros

# Find difference between first point and home position
# Add to all motion points to start from home position
diff_x = init_pos[1] - x[0] 
diff_y = init_pos[2] - y[0]

# convert to np arrays to easily find derivatives
x_array, y_array = np.array(x), np.array(y)

# Find derivative values for arrays
for j in range(3):
     vel_x = derivative_array(x_array)
     acc_x = derivative_array(vel_x)
     jerk_x = derivative_array(acc_x)
     vel_y = derivative_array(y_array)
     acc_y = derivative_array(vel_y)
     jerk_y = derivative_array(acc_y)

# Obtain the joint angles array from the calculated points list
wpr_pos = [-164.003, 0.00, -0.001] # Initial end effector position used to calculate joint positions from ikine

angles_list = []

for p in zip(x, y): # iterate over each point to find the joint angles
    T = joint_angles(p, wpr_pos)  # Convert the cartesian position into an SE3 matrix
    ik_sol = robot.ikine_LM(T)    # Inverse kinematics to find the joint angles (in rad)
    angles_list.append(ik_sol.q)  # Create new list of joint angles for each step

angles_array = np.array(angles_list)

deg_array = rad_to_degrees(angles_array)

# Obtain derivatives for angles
vel_j = derivative_array(deg_array)
acc_j = derivative_array(vel_j)
jerk_j = derivative_array(acc_j)


# Write pos, vel, acc, jerk to .csv file for both x and y

with open(f'output{i}.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    # header
    writer.writerow(["X Position", "X Velocity", "X Acceleration", "X Jerk",
                     "Y Position", "Y Velocity", "Y Acceleration", "Y Jerk"])
    # Write rows
    for row in zip(x_array, vel_x, acc_x, jerk_x, y_array, vel_y,
                   acc_y, jerk_y):
        writer.writerow(row)

# Indexes too far
with open(f'output_{i}_joints.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    # header
    writer.writerow(["J1 Position", "J1 Velocity", "J1 Acceleration", "J1 Jerk",
                     "J2 Position", "J2 Velocity", "J2 Acceleration", "J2 Jerk",
                     "J3 Position", "J3 Velocity", "J3 Acceleration", "J3 Jerk",
                     "J4 Position", "J4 Velocity", "J4 Acceleration", "J4 Jerk",
                     "J5 Position", "J5 Velocity", "J5 Acceleration", "J5 Jerk",
                     "J6 Position", "J6 Velocity", "J6 Acceleration", "J6 Jerk"])
    
    # write rows
    for time in range(len(deg_array) - 1):
        row = [deg_array[time][0], vel_j[time][0], acc_j[time][0], jerk_j[time][0],
               deg_array[time][1], vel_j[time][1], acc_j[time][1], jerk_j[time][1],
               deg_array[time][2], vel_j[time][2], acc_j[time][2], jerk_j[time][2],
               deg_array[time][3], vel_j[time][3], acc_j[time][3], jerk_j[time][3],
               deg_array[time][4], vel_j[time][4], acc_j[time][4], jerk_j[time][4],
               deg_array[time][5], vel_j[time][5], acc_j[time][5], jerk_j[time][5]]
        
        writer.write(row)

# Stream Motion
# Establish UDP connection
client = UDPClient("192.168.0.3")
client.connect()
print("Connection Established to Robot")

resp = client.send_init_pack() 
rob_data = resp[9:18]
print("Initialization Pack Sent")
    
data = commandpack([1, 0, 0, rob_data]) # Initial command pack
resp = client.send_command_pack(data)

# Loop for motion
for i, j in zip(x, y):
        
        rob_data = resp[9:18]

        rob_data[1] = i + diff_x # y direction
        rob_data[2] = j + diff_y # z direction

        data = commandpack([resp[2], 0, 0, rob_data])
        resp = client.send_command_pack(data)

# Final Command Pack
data = commandpack([resp[2], 1, 0, rob_data])
resp = client.send_command_pack(data)

# End of Stream Pack
client.send_end_pack()
print("Stream Complete at command: ", resp[2])



