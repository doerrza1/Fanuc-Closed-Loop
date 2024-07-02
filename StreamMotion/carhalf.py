# Fanuc Stream Motion Test for Cartesian Half Circle without stop

from src.client import *
from src.utils import *
from src.display import *
import numpy as np
import math

# User Inputs
r = int(input("Radius: "))
sig = int(input("Enter number of signals for acceleration: "))
loops = int(input("Loops: "))
d_sig = sig *2

# Signal Definition for the first motion/z-axis motion
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

signal1 = np.append(accel, decel)

# Signal Definition for Subsequent Motion in x-axis
# Acceleration
start3 = 180
end3 = 270

acc_base = np.linspace(1, d_sig, d_sig)**2
acc_base = acc_base / acc_base.sum() * (start3 - end3)
accel = start3 - np.cumsum(acc_base)


# Deceleration
start4 = 270
end4 = 360

dec_base = np.linspace(d_sig, 1, d_sig)**2
dec_base = dec_base / dec_base.sum() * (start4 - end4)
decel = start4 - np.cumsum(dec_base)
print(decel)

signal2 = np.append(accel, decel)

# Conversion from degrees array into cartesian points for signal1

z_begin = 300
z_origin = z_begin + r

x_list = []
z_list = []
z_signal = []

for i, value in enumerate(signal1):
    
    x = r * math.cos(math.radians(value))
    z = z_origin + r*math.sin(math.radians(value))

    x_list.append(x)
    z_list.append(z)
    z_signal.append(z)

signal1 = [x_list, z_list]

# Reverse the list to start at end point
rev = z_signal[::-1]

z_signal = rev + z_signal

x_signal = []
# Conversion from degrees to cartesian points for signal2
for value in signal2:

    x = r * math.cos(math.radians(value))
    x_signal.append(x)

print("signal1_x: ", len(signal1[0]))
print("signal1_z: ", len(signal1[1]))
print("z_signal: ", len(z_signal))
print("x_signal: ", len(x_signal))
print(x_signal)
# print(x_signal)
# Length used for first loop
length1 = len(signal1[0])

# Length used for second loop
length2 = len(x_signal)

# Connection to robot
client = UDPClient("192.168.0.3")
client.connect()

resp = client.send_init_pack()
current_rob_data = resp[9:18]

# Create/send initial command pack
data = commandpack([1, 0, 0, current_rob_data])
resp = client.send_command_pack(data)
current_rob_data = resp[9:18]

# Loop for initial motion
for i in range(length1):
            
        rob_data = resp[9:18]
        rob_data[0] = signal1[0][i]
        rob_data[2] = signal1[1][i]

        data = commandpack([resp[2], 0, 0, rob_data])

        resp = client.send_command_pack(data)

        if (i % 50 == 0):
            display_cmd_pack(resp)

# Loop for Subsequent looping motion
cnt = 0

while (cnt < loops):
     
    for i in range(length2):
            
        rob_data = resp[9:18]

        if (cnt % 2 == 0):
            rob_data[0] = x_signal[i]
            rob_data[2] = z_signal[i]
        else:
            rob_data[0] = (-1) * x_signal[i]
            rob_data[2] = z_signal[i]

        data = commandpack([resp[2], 0, 0, rob_data])

        resp = client.send_command_pack(data)

        if (i % 50 == 0):
            display_cmd_pack(resp)

    cnt += 1

# Final command pack
data = commandpack([resp[2], 1, 0, rob_data])
resp = client.send_command_pack(data)

# End Pack
client.send_end_pack()
print("End of Stream")

    
