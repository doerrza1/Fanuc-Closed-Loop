# Fancuc Stream Motion test for perpetual loop

from src.utils import *
from src.client import *
from src.radius import *
import numpy as np

loops = int(input("How many loops: "))
#Signal Definition
velocity = int(input("Input a velocity value in deg/s: "))
velocity = velocity*8/1000

acceleration = np.linspace(start = 0, stop = velocity, num= 250)
deceleartion = np.flip(acceleration)

signal = np.append(acceleration, deceleartion)

reverse = signal *(-1)

signal = np.append(signal, reverse)

client = UDPClient("192.168.0.3")
client.connect()

resp = client.send_init_pack()

current_jnt_data = resp[18:27]

# Variable definition
i = 0 #index
cnt = 0 #loop count

# Creates/sends initial command pack
data = commandpack([1, 0, 1, current_jnt_data])
resp = client.send_command_pack(data)
current_jnt_data = resp[18:27]

# Loop for a user specified times
while(cnt < loops):
    
    jnt_data = current_jnt_data
    jnt_data[2] += signal[i]
    data = commandpack([resp[2], 0, 1, jnt_data]) # Send Command Pack

    if(i < len(signal)-1):     #increment index
        i += 1

    elif(i == len(signal)-1):  #reset index, increment loop counter
        i = 0
        cnt += 1
    
    resp = client.send_command_pack(data)
    current_jnt_data = resp[18:27]

# Last commandpack
data = commandpack([resp[2], 1, 1, current_jnt_data])
resp = client.send_command_pack(data)

client.send_end_pack()
print("End of Stream")


