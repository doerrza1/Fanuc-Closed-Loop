# Fanuc Stream Motion test for alternating half circle motion

from src.client import *
from src.utils import *
from src.display import *
from src.plot import *
import math
import numpy as np

# User Inputs
r = int(input("Radius: "))
sig = int(input("Enter number of signals for acceleration: "))
loops = int(input("Loops: "))

# Signal definition for acceleration/deceleration for degrees
# Acceleration array definition
start1 = 270
end1 = 225

# Creates a signal with increasing spacing (acceleration) over time 
acc_base = np.linspace(1, sig, sig)**2
acc_base = acc_base / acc_base.sum() * (start1 - end1)
accel = start1 - np.cumsum(acc_base)

# Deceleration array definition
start2 = 225
end2 = 180

dec_base = np.linspace(sig, 1, sig)**2
dec_base = dec_base / dec_base.sum() * (start2 - end2)
decel = start2 - np.cumsum(dec_base)


signal = np.append(accel, decel)

reverse = np.flip(signal)
signal = np.append(signal, reverse)

z_begin = 300
z_origin = z_begin + r

x_list = []
z_list = []

for i, value in enumerate(signal):
    
    x = r * math.cos(math.radians(value))
    z = z_origin + r*math.sin(math.radians(value))

    x_list.append(x)
    z_list.append(z)

length = len(x_list)
plot(z_list, x_list)
signal = [x_list, z_list]

# Connection to robot
client = UDPClient("192.168.0.3")
client.connect()

resp = client.send_init_pack()
current_rob_data = resp[9:18]

# Create/send initial command pack
data = commandpack([1, 0, 0, current_rob_data])
resp = client.send_command_pack(data)
current_rob_data = resp[9:18]

cnt = 0

while(cnt < loops):
    
    # Loop to send the signals to the robot
    for i in range(length):
        
        rob_data = resp[9:18]
        if (cnt % 2 == 0):
            rob_data[0] = signal[0][i]
            rob_data[2] = signal[1][i]
        else:
            rob_data[0] = (-1) * signal[0][i]
            rob_data[2] = signal[1][i]

        data = commandpack([resp[2], 0, 0, rob_data])

        resp = client.send_command_pack(data)

        if (i % 50 == 0):
            display_cmd_pack(data)
    
    cnt += 1 # Increment cnt
    


# Final command pack
data = commandpack([resp[2], 1, 0, rob_data])
resp = client.send_command_pack(data)

client.send_end_pack()
print("End of Stream")



    

