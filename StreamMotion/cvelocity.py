# Stream Motion test for controlling a steady state velocity, using z - axis motion
# The y - axis must also be incremented in order to prevent joint 2 from reaching the jerk limit
# Works with values of velocity <= 100 mm/s

from src.utils import *
from src.client import *
import numpy as np
from src.display import *


# 8ms per signal, 125 signals per second
# velocity = 125 mm/s = .125 mm/ms * 8 = 1 mm/8ms

v = int(input("Input a value for maximum velocity in mm/s (0 < v < 255): "))
print("----------------------------")
# conversion from mm/s to mm/ms * 8
# ex. 125 mm/s -> .125 mm/ms -> 1 mm/8ms
v_ms = v*8/1000
print("Ss velocity (calculated): ", v_ms)

#accelerate to steady state velocity in 200 packs
acceleration = np.linspace(start = 0, stop = v_ms, num = 200)
print("Sum acc: ", sum(acceleration))
print("Ss velocity: ", acceleration[-1])

# move at steady state velocity for 100 packs
steady_state = np.full(100, v_ms)
print("Sum ss: ", sum(steady_state))

# decelerate back to 0 in 200 packs
deceleration = np.linspace(start = v_ms, stop = 0, num = 200)
print("Sum Dec: ", sum(deceleration))
print("Dec start: ", deceleration[0])
print("----------------------------")

# combine all arrays to create the signal
signal = np.append(acceleration, steady_state)
signal = np.append(signal, deceleration)
signal = signal * 7.5 # Scaling Factor

# create a flipped array so robot returns to initial position
reverse = np.flip(signal)
signal = np.append(signal, reverse)

# establish connection to robot
client = UDPClient("192.168.0.3")
client.connect()

resp = client.send_init_pack()

# obtain current data to build off of
current_data = resp[9:18]

for i, value in enumerate(signal):

    car_data = current_data

    if (i == 0):
        # Creates initial command pack
        data = commandpack([1, 0, 0, car_data])

    elif(i <= 500):
    	# Creates forward movement packs
        car_data[1] -= value/4
        car_data[2] += value
    	
        data = commandpack([resp[2], 0, 0, car_data])
    	
    	
    elif((i > 500) and (i < 1000)):
    	# Creates reverse movement packs
        car_data[1] += value/4
        car_data[2] -= value
    	
        data = commandpack([resp[2], 0, 0, car_data])
    else:
        # Creates final command pack
        data = commandpack([resp[2], 1, 0, car_data])

    # Sends command pack and receives new data
    resp = client.send_command_pack(data)

    # print out data for every 50 sent
    if (i % 50 == 0):
        display_cmd_pack(resp)

        limit = client.send_vel_pack(3)
        display_limit_pack(limit)
        limit = client.send_acc_pack(3)
        display_limit_pack(limit)
        limit = client.send_jerk_pack(3)
        display_limit_pack(limit)

    current_data = resp[9:18]
    

client.send_end_pack()
print("End of Stream")

        


