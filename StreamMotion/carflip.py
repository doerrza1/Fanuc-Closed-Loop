# Cartesian Flip Motion for Fanuc LR Mate 200 iD

from src.client import *
from src.utils import *
from src.display import *
import numpy as np

# User inputs

# r = int(input("Enter a radius for motion: "))

v = 50 * 8/1000  #50 mm/s --> mm/8ms
print("velocity: ", v)
# accel/decel time = 0.5 seconds = 63 signals

# Signal definintion

accel = np.linspace(start = 0, stop = v, num=125)
decel = np.flip(accel)

signal = np.append(accel, decel)

reverse = signal*(-1)

signal = np.append(signal, reverse)

client = UDPClient("192.168.0.3")
client.connect()

resp = client.send_init_pack()

current_rob_data = resp[9:18]

# Variable declaration
loops = 2
i = 0
cnt = 0

length = len(signal)
print("length: ", length)

while (cnt < loops):
    
  rob_data = current_rob_data
  if (cnt % 2 == 0):
        
    if (i < length - 1):
      rob_data[0] += signal[i]
      rob_data[2] += signal[i]

      data = commandpack([resp[2], 0, 0, rob_data])
      i += 1

    elif (i == length - 1):
      rob_data[0] += signal[i]
      rob_data[2] += signal[i]

      data = commandpack([resp[2], 0, 0, rob_data])
      i = 0
      cnt += 1
      print(f"Loop Done {cnt}") 
  else:
        
    if (i < length - 1):
      rob_data[0] -= signal[i]
      rob_data[2] -= signal[i]

      data = commandpack([resp[2], 0, 0, rob_data])
      i += 1

    elif (i == length - 1):
      rob_data[0] -= signal[i]
      rob_data[2] -= signal[i]

      data = commandpack([resp[2], 0, 0, rob_data])
      i = 0
      cnt += 1
      print(f"Loop Done {cnt}")

  resp = client.send_command_pack(data)
  current_data = resp[9:18]


data = commandpack([resp[2], 1, 0, rob_data])
resp = client.send_command_pack(data)

client.send_end_pack()
print("End of Stream")
