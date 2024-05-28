# Stream Motion test for Fanuc LR Mate 200iD

import socket
import time
from src.utils import *
from src.client import *
import numpy as np

# signal definition
t_max = 8000 
t = np.linspace(start=0, end=t_max-1, num=t_max)
signal = 1e-4*t


# UDP initialization
client = UDPClient("192.168.0.3", 18735)
client.connect()

# Send Initialization pack, Sets resp to the Status Packet
resp = client.send_init_pack()

#Parses current position data from resp and prints the postitions
current_car_data = resp[9:15] 
print(['%.4f' % jnt for jnt in current_car_data]) 

for i, value in enumerate(signal):
  
  car_data = current_car_data
  car_data[2] += value #increments z-axis by value in signal
  
  if (i == 0):
    data = commandpack([1, 0, 0, car_data])
    print('COMMAND PACK SENT:', ['%.4f' % jnt for jnt in car_data])
    print('Sent Seq No:', [1,0,1])
    
  elif (i < len(signal)-1):
    data = commandpack([resp[2], 0, 0, car_data])
    print('COMMAND PACK SENT:', ['%.4f' % jnt for jnt in car_data])
    print('Sent Seq No:', [resp[2],0,1])
  
  else:
    data = commandpack([resp[2], 1, 0, car_data])
    print('COMMAND PACK SENT:', ['%.4f' % jnt for jnt in car_data])
    print('Sent Seq No:', [resp[2], 1, 1])

# Sends command pack and recieves status packet   
  resp = client.send_command_pack(data)
  
  # print('({:.2f}ms)'.format(1000*(time.time()-start_time)))
  
  resp = explainRobData(resp) 
  
  print('Received Seq No:', resp[2])
  
  current_jnt_data = resp[18:27]

client.send_end_pack()
print("End of stream")








