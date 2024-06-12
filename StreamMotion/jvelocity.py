# Stream Motion test for controlling a steady state velocity wih joint motion
# J3 and J2 work in tandem with one another

from src.client import *
from src.utils import *
from src.display import *
from src.radius import *
import numpy as np

# Velocity input will be in deg/s, converted to deg/8ms (for signal definition)

# Will also convert the deg/s into mm/s using a calculated radius for the motion to 
# ensure that the input velocity is less than 255 mm/s

# Loops unti an acceptable velocity value is obtained
velocity = 1
while(velocity == True):
    velocity = int(input("Input a value for Max Velocity in deg/s: "))
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

#signal definition
acceleration = np.linspace(start = 0, stop = v_ms, num = 200)

steady_state = np.full(100, v_ms)

deceleration = np.linspace(start = v_ms, stop = 0, num = 200)

#combine signals to create the motion
signal = np.append(acceleration, steady_state)
signal = np.append(signal, deceleration)

#reverse the signal to return to original position
reverse = np.flip(signal)
signal = np.append(signal, reverse)

client = UDPClient("192.168.0.3")
client.connect()

resp = client.send_init_pack()

current_jnt_data = resp[18:27]

for i, value in enumerate(signal):

    jnt_data = current_jnt_data

    if (i == 0):
        # Creates initial command pack
        data = commandpack([1, 0, 1, jnt_data])

    elif(i <= 500):
    	# Creates forward movement packs
        jnt_data[joint] += value
        
        data = commandpack([resp[2], 0, 1, jnt_data])
    		
    elif((i > 500) and (i < 1000)):
    	# Creates reverse movement packs
        jnt_data[joint] += value
    	
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

               


