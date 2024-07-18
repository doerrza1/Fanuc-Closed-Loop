# Fanuc Stream Motion draft for final motion (Test for equation movement)
# Using position equation with determined acceleration/velocity/position to get values as loop goes on
# pos = 1/2*acc*i^2 + vel*i + init_pos
# Needs function to obtain necessary acceleration/velocity/position
# Use i as the number of signals

from src.client import *
from src.utils import *
from src.display import *
import numpy as np
# import serial

# Varible declaration

# Postion equation variables (will have to be different for each cartesian axis X,Y,Z) if more than one is used
# Variables will have to be scaled to per 8 ms
acc = 0  # Acceleration for movement
vel = 0  # Initial Velocity for movement (will most likely stay 0)
init_pos = 0 # Initial Position for movement 

# User Inputs
acc = float(input("Enter the acceleration in mm/s: "))
acc = acc*8/1000 # Scaling acc to mm/8ms^2
sig = int(input("Enter the amount of signals to send per movement: "))

# Create an array from 1 to the value of sig in increments of 1
count = np.linspace(start = 1, stop = sig, num = sig)

# Connection to robot
client = UDPClient("192.168.0.3")
client.connect()

# Initialization Pack
resp = client.send_init_pack()
current_data = resp[9:18]
init_pos = current_data[2] # Initial position for z-movement

# First Command Pack
data = commandpack([1, 0, 0, current_data])
resp = client.send_command_pack(data)

current_data = resp[9:18]

# Forward Movement Loops
for i in count: # Acceleration

    rob_data = current_data

    rob_data[2] = 0.5*acc*(i**2) + init_pos

    data = commandpack([resp[2], 0, 0, rob_data])
        
    if (i == count[-2]): # Second to Last Velocity
        p_2 = rob_data[2]

    if (i == count[-1]): # Final Velocity
        p_fin = rob_data[2]

        v = p_fin - p_2

    # Sends command pack and receives new data
    resp = client.send_command_pack(data)
    current_data = resp[9:18]
    display_cmd_pack(resp)
     
for i in count: # Deceleration
    rob_data = current_data

    rob_data[2] = -0.5*acc*(i**2) + v*i + p_fin

    data = commandpack([resp[2], 0, 0, rob_data])
    if (i == count[-1]):
        p_fin = rob_data[2]

    # Sends command pack and receives new data
    resp = client.send_command_pack(data)
    current_data = resp[9:18]
    display_cmd_pack(resp)

# Reverse Movement Loops 
for i in count: # Acceleration
    rob_data = current_data

    rob_data[2] = -0.5*acc*(i**2) + p_fin

    data = commandpack([resp[2], 0, 0, rob_data])
        
    if (i == count[-2]): # Second to Last Velocity
        p_2 = rob_data[2]

    if (i == count[-1]): # Final Velocity
        p_fin = rob_data[2]

        v = p_fin - p_2

    # Sends command pack and receives new data
    resp = client.send_command_pack(data)
    current_data = resp[9:18]
    display_cmd_pack(resp)

for i in count:
    rob_data = current_data

    rob_data[2] = 0.5*acc*(i**2) - v*i + p_fin

    data = commandpack([resp[2], 0, 0, rob_data])

    # Sends command pack and receives new data
    resp = client.send_command_pack(data)
    current_data = resp[9:18]
    display_cmd_pack(resp)

# Creates/sends final command pack
data = commandpack([resp[2], 1, 0, current_data])
resp = client.send_command_pack(data)

# Sends end pack
client.send_end_pack()





