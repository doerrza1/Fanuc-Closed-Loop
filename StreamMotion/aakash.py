# Fanuc Stream Motion Test with Decoder

from src.client import *
from src.utils import *
from src.plot import *
from src.display import *
import numpy as np
import serial
import time

# initialize serial connection
ser = serial.Serial('/dev/ttyACM0', 9600)

# s_list = []
# Buffer to ensure serial reading is correct
for i in range(100):
    line = ser.readline().decode('utf-8').strip()
    print("Encoder Value: ", line)
    if line == '':
        line = 0

    # value = float(line) / 10 + 300
    # s_list.append(value)


# initialize connection to robot
client = UDPClient("192.168.0.3")
client.connect()



period = 0.04

# for i in range(len(s_list)):

#     rob_data = resp[9:18]

#     rob_data[2] = s_list[i]

#     data = commandpack([resp[2], 0, 0, rob_data])
#     resp = client.send_command_pack(data)

#     display_cmd_pack(resp)
#     print("--------------------------")

for i in range (100000):
    start_time = time.time()
    rob_data = resp[9:18]

    if i == 0:
        resp = client.send_init_pack()
        current_rob_data = resp[9:18]
        init_z = resp[11] # initial z position

        # Create/send initial command pack
        data = commandpack([1, 0, 0, current_rob_data])
        resp = client.send_command_pack(data)
    	
        
    
    else:

        line = ser.readline().decode('utf-8').strip()
        print("Encoder Value: ", line)
        if line == '':
            line = 0
            
        value = float(line) / 5 # move robot to 1/5 of the value read
        print("Value: ", value)
        rob_data[2] = init_z + value # Add value to initial z position

        data = commandpack([resp[2], 0, 0, rob_data])

        resp = client.send_command_pack(data)

        display_cmd_pack(resp)

    time_elapsed = time.time() - start_time
    print("Time Elapsed: ", time_elapsed)

    sleep_time = max(0, period - time_elapsed)
    print("Sleep Time: ", sleep_time)
    #time.sleep(sleep_time)
    print('----------------------------')

# Final Command Pack
data = commandpack([resp[2], 1, 0, rob_data])
resp = client.send_command_pack(data)

# End pack
client.send_end_pack()
print("End of Stream")

