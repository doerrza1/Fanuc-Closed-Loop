# Fanuc Stream Motion Test for complete control over both cartesian and joint movement

from src.client import *
from src.display import *
from src.utils import *
from src.velo import *
import numpy as np

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

ret = velocity(m)

v, axis = ret

# Inputs for the acceleration/deceleration time and steady state motion time
a_time = int(input("Input a time for accelertion/deceleration in seconds: "))
s_time = int(input("Input a time for steady state motion in seconds: "))

a_time = int(a_time*1000/8)
s_time = int(s_time*1000/8)
print("----------------------------")

a = np.linspace(start = 0, stop = v, num = a_time) #acceleration
s = np.full(s_time, v)                             #steadystate
d = np.flip(a)                                     #deceleration

# Combining the signals
signal = np.append(a, s)
signal = np.append(signal, d)

#Create reverse signal
reverse = np.flip(signal)
reverse = reverse*(-1)

# Completed signal
signal = np.append(signal, reverse)

# Establish Connection
client = UDPClient("192.168.0.3")
client.connect()

resp = client.send_init_pack()

current_rob_data = resp[low_bound:high_bound]

for i, value in enumerate(signal):

    rob_data = current_rob_data

    if (i == 0):
        # Creates initial command pack
        data = commandpack([1, 0, m, rob_data])

    elif(i < len(signal)-1):
    	# Creates forward movement packs
        rob_data[axis-1] += value
        
        data = commandpack([resp[2], 0, m, rob_data])

    else:
        # Creates final command pack
        data = commandpack([resp[2], 1, m, rob_data])

    # Sends command pack and receives new data
    resp = client.send_command_pack(data)

    # print out data for every 50 sent
    if (i % 50 == 0):
        display_jnt_pack(resp)

        limit = client.send_vel_pack(axis)
        display_limit_pack(limit)

        limit = client.send_acc_pack(axis)
        display_limit_pack(limit)

        limit = client.send_jerk_pack(axis)
        display_limit_pack(limit)

    current_data = resp[low_bound:high_bound]
    

client.send_end_pack()
print("End of Stream")

