# Fanuc Stream Motion Test with Decoder

from src.client import *
from src.utils import *
from src.plot import *
from src.display import *
import numpy as np
import serial
import time
import math

def read_serial_data():
    try:
        # Read a line from the serial port
        line = ser.readline().decode('utf-8').strip()
        if line:
            values = line.split(',')
            if len(values) == 3:
                # Split the line into position, velocity, and acceleration
                position, velocity, acceleration = values
                return float(position), float(velocity), float(acceleration)
            else:
                print("Not enough values received")

    except Exception as e:
        print(f"Error reading serial data: {e}")
    return None, None, None

def pause():
    programPause = input("Press the <ENTER> key to continue...")

def zeroBeam():
    print("Place the beam at theta = 0 and maintain stationary")
    pause()
    tf = time.time() + 6 # time to record encoder data at theta = 0
    while time.time() < tf:
        th, dth, d2th = read_serial_data() # establishes the zero position
        last_th = th
        last_dth = dth
    return last_th, last_dth

# Initialize serial connection
ser = serial.Serial('/dev/ttyACM0', 115200)

# Initialize connection to robot
client = UDPClient("192.168.0.3")
client.connect()


period = 0.0079
last_value = 0

last_th, last_dth = zeroBeam() # Sets the zero position for the beam
print(last_th, last_dth)
while last_th != 0:  # If theta is not 0 redo zeroing until it is
    print("redo zeroing")
    last_th, last_dth = zeroBeam()

resp = client.send_init_pack()
current_rob_data = resp[9:18]
init_z = current_rob_data[2] # initial z position

# Create/send initial command pack
data = commandpack([1, 0, 0, current_rob_data])
resp = client.send_command_pack(data)

for i in range (2000):
    start_time = time.time()
    rob_data = resp[9:18]

    pos, vel, acc = read_serial_data()

    print("Pos: ", pos)
    rob_data[2] += math.sin(pos) # Increment value from initial z position

    data = commandpack([resp[2], 0, 0, rob_data])

    resp = client.send_command_pack(data)

    #display_cmd_pack(resp)

    # Time calcs
    time_elapsed = time.time() - start_time
    print("Time Elapsed: ", time_elapsed)
    sleep_time = max(0, period - time_elapsed)
    print("Sleep Time: ", sleep_time)
    time.sleep(sleep_time)
    print('----------------------------')

# Final Command Pack
data = commandpack([resp[2], 1, 0, rob_data])
resp = client.send_command_pack(data)

# End pack
client.send_end_pack()
print("End of Stream")

