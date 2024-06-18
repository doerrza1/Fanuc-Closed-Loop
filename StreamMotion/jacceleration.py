# Stream Motion test for controlling acceleration/deceleration to a given velocity for joint motion
# J3 and J2 work in tandem with one another do not increment both

from src.client import *
from src.utils import *
from src.display import *
from src.radius import *
import numpy as np


acceleration = int(input("Input a time for accelertion in seconds: "))

# Grabs and validates target velocity
velocity = 1
while(velocity == True):
    velocity = int(input("Input a value for target velocity in deg/s: "))
    # selects joint for movement, joint-1 is the index for the joint within jnt_data 
    joint = int(input("Input a joint(1-6) for rotation: "))

    # Gets radius of the joint based on the initial position of the arm
    r = radius(joint)

    # Converts deg/s into rad/s 
    v_rads = math.radians(velocity)
    # Converst rad/s into mm/s
    v_mm = v_rads*r

    if (v_mm < 255):
        break
    else:
        print("Velocity is too high!!")
    
    print("----------------------------")

#Sets the velocity per every 8ms
v_ms = velocity*8/1000

# Time for acceleration in seconds, converted into 8ms steps
a_ms = acceleration*1000/8

# Signal Definition
# Forward Signal
acceleration = np.linspace(start=0, stop=v_ms, num=a_ms)

steady_state = np.full(100, v_ms)

deceleration = np.flip(acceleration)

signal = np.append(acceleration, steady_state)
signal = np.append(signal, deceleration)

# Reverse Signal
reverse = np.flip(signal)
reverse = reverse *(-1)

# Full signal
signal = np.append(signal, reverse)
# Length for for loop
length = len(signal)

client = UDPClient("192.168.0.3")
client.connect()

resp = client.send_init_pack()

current_jnt_data = resp[18:27]

for i, value in enumerate(signal):

    jnt_data = current_jnt_data

    if (i == 0):
        # Creates initial command pack
        data = commandpack([1, 0, 1, jnt_data])

    elif(i < length - 1):
    	# Creates movement packs
        jnt_data[joint-1] += value
        data = commandpack([resp[2], 0, 1, jnt_data])

    else:
        # Creates final command pack
        data = commandpack([resp[2], 1, 1, jnt_data])

    # Sends command pack and receives new data
    resp = client.send_command_pack(data)

    # print out data for every 50 sent
    if (i % 50 == 0):
        display_jnt_pack(resp)

        limit = client.send_vel_pack(joint)
        display_limit_pack(limit)

        limit = client.send_acc_pack(joint)
        display_limit_pack(limit)

        limit = client.send_jerk_pack(joint)
        display_limit_pack(limit)

    current_data = resp[18:27]
    

client.send_end_pack()
print("End of Stream")




# Stuff that didn't work may come back to it
#Start at velocity = 0, and accelerate to v_ms with acceleration value
# a_ms = acceleration*8/1000

# num = 0
# acc_array = []

# while(num <= v_ms):
#     acc_array.append(num)
#     num += a_ms

# print("target velo: ", v_ms)
# print("a_ms: ", a_ms)
# print("last velo: ", acc_array[-1])
# print("length: ", len(acc_array))

