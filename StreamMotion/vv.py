# Fanuc Stream Motion for a variable velocity for each loop

from src.client import *
from src.utils import *
from src.display import *
import numpy as np

print("Stream Motion with Varying Velocity loops!!!")
print("Do not exceed 75mm/s or 20 deg/s for test purposes")
print("Do not exceed 4 seconds for time")
print("----------------------------")

# Obtain number of loops
loops = int(input("Enter the amount of loops: "))
print("Acceleration/Deceleration time set to 1 second")
print("Steady State Velocity time set to 1 second")
print("----------------------------")

# Loop to obtain the steady state velocity for each iteration
v_list = []  # List of velocities
for i in range(loops):

    velo = int(input(f"Enter the steady state velocity for loop {i + 1}: "))
    velo = velo *8/1000   # Convert velo from mm/s to mm/8ms
    print("----------------------------")

    v_list.append(velo)

print(v_list)

# Signal creation for each loop
s_list = []  # List of Signals
for value in v_list:
    acc = np.linspace(start = 0, stop = value, num = 125)
    ss = np.full(125, value)
    dec = np.flip(acc)

    sig = np.append(acc, ss)
    sig = np.append(sig, dec)

    reverse = (-1) * sig
    sig = np.append(sig, reverse)

    s_list.append(sig)

print(s_list)
# Establish Connection

client = UDPClient("192.168.0.3")
client.connect()

resp = client.send_init_pack()

current_rob_data = resp[9:18]

# Create/send initial command pack
data = commandpack([1, 0, 0, current_rob_data])
resp = client.send_command_pack(data)
current_rob_data = resp[9:18]


cnt = 0
while (cnt < loops):

    length = len(s_list[cnt])
    
    for i in range(length):

        rob_data = current_rob_data

        rob_data[2] += s_list[cnt][i] # Z-axis
        data = commandpack([resp[2], 0, 0, rob_data])

        resp = client.send_command_pack(data)
        current_data = resp[9:18]

    cnt += 1

# Send Final Command Pack   
data = commandpack([resp[2], 1, 0, rob_data])
resp = client.send_command_pack(data)

client.send_end_pack()
print("End of Stream")









