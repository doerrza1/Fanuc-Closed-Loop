# Fanuc Stream Motion Test for communication with decoder

from src.client import *
from src.utils import *
from src.display import *
import numpy as np
import serial
import time

# Obtain user input for the value to move robot, When decoder value is greater then input robot will move with motion determined 
# by the magnitude of the input

# initialize serial connection
ser = serial.Serial('/dev/ttyACM0', 9600)

# initialize connection to robot
client = UDPClient("192.168.0.3")
client.connect()

# User input for decoder value
value = float(input("Enter a value to begin motion(1:1000): "))

# Scale input to make it maximum velocity
v_s = value / 8 # velocity(seconds) = value / 8
v_ms = v_s*8/1000 # velocity(8 miliseconds)

# Signal Definition
acc = np.linspace(start = 0, stop = v_ms, num = 125)
dec = np.linspace(start = v_ms, stop = 0, num = 125)

sig = np.append(acc, dec)
reverse = sig * (-1)
sig = np.append(sig, reverse)

# Initialization pack
resp = client.send_init_pack()
current_data = resp[9:18]

# Loop Variables
period = 0.008 # Time for loop
j = 0 # Counter for initial command pack
last_value = 0  # Placeholder for if an error occurs
while(value != 0):
    # Obtain the starting time for the loop
    start_time = time.time()

    car_data = current_data

    if (j == 0):  # Send initial command pack
        data = commandpack([1, 0, 0, car_data])

    # Obtain the encoder value
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    print("Encoder Value: ", line)

    # Convert the encoder string into a float
    try:
        decoder = float(line)
    # If given a bad value will convert it to zero
    except:  # ValueError
        decoder = last_value
        print("False Reading")

    last_value = decoder
    # If the decoder is greater than value start robot motion
    if (decoder > value):
        print("Motion Start")

        # Loop through signal
        for i in sig:
            car_data = current_data

            car_data[2] += i # Increment z-position by value in sig
            data = commandpack([resp[2], 0, 0, car_data])
            
            # Sends command pack and receives new data
            resp = client.send_command_pack(data)
            current_data = resp[9:18]
            display_cmd_pack(resp)
        
        # Creates/sends final command pack
        data = commandpack([resp[2], 1, 0, current_data])
        resp = client.send_command_pack(data)

        # Sends end pack
        client.send_end_pack()
        
        
        break
    
    # If the decoder is less than value send command pack of current position
    elif (decoder < value): 
        
        data = commandpack([resp[2], 0, 0, car_data])
        # Sends command pack and receives new data
        resp = client.send_command_pack(data)
        current_data = resp[9:18]

    time_elapsed = time.time() - start_time
    print("Time Elapsed, ", time_elapsed)
    print("-----------------------------")


print("End of Stream")    



