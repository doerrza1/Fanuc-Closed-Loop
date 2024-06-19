# Full Motion Control (All joints/All axes) for Fanuc Stream Motion

from src.client import *
from src.display import *
from src.utils import *
from src.fullvelo import *
from src.signal import *
import numpy as np


print("Stream Motion Master Control")
print("Do not exceed 75mm/s or 20 deg/s for test purposes")
print("Do not exceed 4 seconds for time")
print("----------------------------")
# Selection for method of movement (C - Cartesian, J - Joint)
while(True):
    movement = input("Select a method of movement (C or J): ").strip().upper()

    if (movement == "C"):
        print("Cartesian Movement Selected")
        print("----------------------------")
        # Sets the bounds for slicing the response packet
        low_bound = 9
        high_bound = 18
        # Sets the value for motion within command packets
        m = 0

        break

    elif (movement == "J"):
        print("Joint Movement Selected")
        print("----------------------------")
        # Sets the bounds for slicing the response packet
        low_bound = 18
        high_bound = 27
        # Sets the value for motion within command packets
        m = 1

        break

    else:
        print("Invalid Selection")
        print("----------------------------")

# Calls velocity funtion from velo.py in order to obtain the velocity (mm/s or deg/s) and
# convert it into 8ms denominations

# ret is list of lists comprised of [velocity, accel/deccel signal count, steady state signal count]
ret = motion(m)

# Creates the lol for signals normalizes to all the same lengths
signal = create(ret)

print(len(signal))

# Obtain the length of the signals
max_len = len(max(signal, key = len))


# Establish Connection
client = UDPClient("192.168.0.3")
client.connect()

resp = client.send_init_pack()

current_rob_data = resp[low_bound:high_bound]

# Cartesian Movement
if (m == 0):
    
    for i in range(max_len):

        rob_data = current_rob_data

        if (i == 0):
            # Creates initial command pack
            data = commandpack([1, 0, m, rob_data])

        elif(i < max_len-2):
            # Creates forward movement packs
            rob_data[0] += signal[0][i]  #X
            rob_data[1] += signal[1][i]  #Y
            rob_data[2] += signal[2][i]  #Z
            
            data = commandpack([resp[2], 0, m, rob_data])

        else:
            # Creates final command pack
            data = commandpack([resp[2], 1, m, rob_data])

        # Sends command pack and receives new data
        resp = client.send_command_pack(data)

        # print out data for every 50 sent
        if (i % 50 == 0):
            display_cmd_pack(resp)

        current_data = resp[low_bound:high_bound]

# Joint Motion
elif (m == 1):
    
    for i in range(max_len):

        rob_data = current_rob_data

        if (i == 0):
            # Creates initial command pack
            data = commandpack([1, 0, m, rob_data])

        elif(i < max_len-2):
            # Creates forward movement packs
            rob_data[0] += signal[0][i]  #J1
            rob_data[1] += signal[1][i]  #J2
            rob_data[2] += signal[2][i]  #J3
            rob_data[3] += signal[3][i]  #J4
            rob_data[4] += signal[4][i]  #J5
            rob_data[5] += signal[5][i]  #J6

            data = commandpack([resp[2], 0, m, rob_data])

        else:
            # Creates final command pack
            data = commandpack([resp[2], 1, m, rob_data])

        # Sends command pack and receives new data
        resp = client.send_command_pack(data)

        # print out data for every 50 sent
        if (i % 50 == 0):
            display_jnt_pack(resp)

        current_data = resp[low_bound:high_bound]


client.send_end_pack()
print("End of Stream")