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

# initialize connection to robot
client = UDPClient("192.168.0.3")
client.connect()



period = 0.04
last_value = 0

for i in range (100000):
    start_time = time.time()
    rob_data = resp[9:18]

    if i == 0:
        resp = client.send_init_pack()
        current_rob_data = resp[9:18]
        init_z = current_rob_data[2] # initial z position

        # Create/send initial command pack
        data = commandpack([1, 0, 0, current_rob_data])
        resp = client.send_command_pack(data)
    	
    else:

        line = ser.readline().decode('utf-8', errors='ignore').strip()
        print("Encoder Value: ", line)
        try:
            value = float(line) / 100 # move robot to 1/100 of the value read
            last_value = value

        except:
            value = last_value

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

