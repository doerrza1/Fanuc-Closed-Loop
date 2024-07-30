from src.client import UDPClient
from src.utils import commandpack
from src.display import display_cmd_pack
import numpy as np
import serial
import time

# Initialize serial connection
ser = serial.Serial('/dev/ttyACM0', 9600)

# Initialize connection to robot
client = UDPClient("192.168.0.3")
client.connect()

# Period for the loop
period = 0.07
last_value = 0

# Smoothing factor for exponential moving average
alpha = 0.2  # Adjust this value as needed

# Initial Command Pack
resp = client.send_init_pack()
current_rob_data = resp[9:18]
init_z = current_rob_data[2]  # Initial z position

# Create/send initial command pack
data = commandpack([1, 0, 0, current_rob_data])
resp = client.send_command_pack(data)

for i in range(100000):
    start_time = time.time()
    rob_data = current_rob_data.copy()  # Copy current robot data

    # Read encoder value
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    print("Encoder Value: ", line)
    
    try:
        value = float(line) / 100  # Move robot to 1/100 of the value read
        smoothed_value = alpha * value + (1 - alpha) * last_value
        last_value = smoothed_value
    
    except ValueError:
        smoothed_value = last_value

    print("Smoothed Value: ", smoothed_value)
    rob_data[2] = init_z + smoothed_value  # Add smoothed value to initial z position

    # Create command pack
    data = commandpack([resp[2], 0, 0, rob_data])

    # Send command pack and receive new data
    resp = client.send_command_pack(data)
    current_rob_data = resp[9:18]  # Update current robot data
    display_cmd_pack(resp)

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
