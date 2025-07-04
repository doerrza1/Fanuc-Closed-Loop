# Full Motion Control (All joints/All axes) for Fanuc Stream Motion
# Same as fullmotion.py with loop integration to continuously run

from src.client import *
from src.display import *
from src.utils import *
from src.fullvelo import *
from src.signal import *
import numpy as np
import csv

def derivative_array(array, dt = 0.008):

    # np.diff finds the difference between subsequent rows of the array
    # allowing for efficient calculation for derivatives (pos -> vel -> acc -> jerk)

    d_array = np.diff(array, axis=0) / dt 

    # Pad the first row with zeros to maintain the shape of array
    zero_array = np.zeros((1, array.shape[1]))
    d_array = np.vstack([zero_array, d_array])
    
    return d_array

print("Stream Motion Master Control now with loops!!!")
print("Do not exceed 75mm/s or 20 deg/s for test purposes")
print("Do not exceed 4 seconds for time")
print("----------------------------")

loops = int(input("How many loops: "))
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
print("Number of signals to be sent: ", max_len)

# # Establish Connection
# client = UDPClient("192.168.0.3")
# client.connect()

# resp = client.send_init_pack()

# current_rob_data = resp[low_bound:high_bound]

# # Create/send initial command pack
# data = commandpack([1, 0, m, current_rob_data])
# resp = client.send_command_pack(data)
# current_rob_data = resp[low_bound:high_bound]

# Variable Declarations
cnt = 0
i = 0
point_list = []
rob_data = [0, 350, 300] #initial position

# Cartesian Movement
if (m == 0):
    
    while(cnt < loops):

        
        if (cnt % 2 == 0):
            
            if(i < max_len-1):
                # Creates forward movement packs
                rob_data[0] += signal[0][i]  #X
                rob_data[1] += signal[1][i]  #Y
                rob_data[2] += signal[2][i]  #Z
                
                #data = commandpack([resp[2], 0, m, rob_data])
                i += 1
            
            elif(i == max_len-1):
                rob_data[0] += signal[0][i]  #X
                rob_data[1] += signal[1][i]  #Y
                rob_data[2] += signal[2][i]  #Z
                
                #data = commandpack([resp[2], 0, m, rob_data])

                i = 0
                cnt += 1 
                print(f"Loop Done {cnt}") 
        else:
            
            if(i < max_len-1):
                # Creates forward movement packs
                rob_data[0] -= signal[0][i]  #X
                rob_data[1] -= signal[1][i]  #Y
                rob_data[2] -= signal[2][i]  #Z
                
                #data = commandpack([resp[2], 0, m, rob_data])
                i += 1
            
            elif(i == max_len-1):
                rob_data[0] -= signal[0][i]  #X
                rob_data[1] -= signal[1][i]  #Y
                rob_data[2] -= signal[2][i]  #Z
                
                #data = commandpack([resp[2], 0, m, rob_data])

                i = 0
                cnt += 1 
                print(f"Loop Done {cnt}")

        point_list.append(np.array(rob_data[:3])) # Append XYZ point to list

        # Sends command pack and receives new data
        #resp = client.send_command_pack(data)
        #current_data = resp[low_bound:high_bound]

# Joint Motion
elif (m == 1):
    
    while(cnt < loops):

        #rob_data = current_rob_data
        if (cnt % 2 == 0):
            
            if(i < max_len-1):
                # Creates forward movement packs
                rob_data[0] += signal[0][i]  #J1
                rob_data[1] += signal[1][i]  #J2
                rob_data[2] += signal[2][i]  #J3
                rob_data[3] += signal[3][i]  #J4
                rob_data[4] += signal[4][i]  #J5
                rob_data[5] += signal[5][i]  #J6

                #data = commandpack([resp[2], 0, m, rob_data])
                i += 1

            elif(i == max_len-1):
                # Creates forward movement packs
                rob_data[0] += signal[0][i]  #J1
                rob_data[1] += signal[1][i]  #J2
                rob_data[2] += signal[2][i]  #J3
                rob_data[3] += signal[3][i]  #J4
                rob_data[4] += signal[4][i]  #J5
                rob_data[5] += signal[5][i]  #J6

                #data = commandpack([resp[2], 0, m, rob_data])
                i = 0
                cnt += 1
                print(f"Loop Done {cnt}") 
        
        else:
            
            if(i < max_len-1):
                # Creates forward movement packs
                rob_data[0] -= signal[0][i]  #J1
                rob_data[1] -= signal[1][i]  #J2
                rob_data[2] -= signal[2][i]  #J3
                rob_data[3] -= signal[3][i]  #J4
                rob_data[4] -= signal[4][i]  #J5
                rob_data[5] -= signal[5][i]  #J6

                
                i += 1

            elif(i == max_len-1):
                # Creates forward movement packs
                rob_data[0] -= signal[0][i]  #J1
                rob_data[1] -= signal[1][i]  #J2
                rob_data[2] -= signal[2][i]  #J3
                rob_data[3] -= signal[3][i]  #J4
                rob_data[4] -= signal[4][i]  #J5
                rob_data[5] -= signal[5][i]  #J6

                #data = commandpack([resp[2], 0, m, rob_data])
                i = 0
                cnt += 1
                print(f"Loop Done {cnt}")

        # Sends command pack and receives new data
        #resp = client.send_command_pack(data)
        #current_data = resp[low_bound:high_bound]

# Send Final Command Pack   
# data = commandpack([resp[2], 1, m, rob_data])
# resp = client.send_command_pack(data)

# client.send_end_pack()
# print("End of Stream")

point_array = np.array(point_list)
vel_array = derivative_array(point_array)
acc_array = derivative_array(vel_array)
jerk_array = derivative_array(acc_array)
print(len(point_array))
print(len(vel_array))
print(len(acc_array))
print(len(jerk_array))

with open('output_full_1.csv', mode='w', newline='') as file:

    writer = csv.writer(file)
    # header
    writer.writerow(["X Position", "X Velocity", "X Acceleration", "X Jerk",
                     "Y Position", "Y Velocity", "Y Acceleration", "Y Jerk",
                     "Z Position", "Z Velocity", "Z Acceleration", "Z Jerk"])
    # Write rows
    for i in range(len(point_array) - 1):

        row = [point_array[i][0], vel_array[i][0], acc_array[i][0], jerk_array[i][0], 
                   point_array[i][1], vel_array[i][1], acc_array[i][1], jerk_array[i][1],
                   point_array[i][2], vel_array[i][2], acc_array[i][2], jerk_array[i][2]]
        
        writer.writerow(row)