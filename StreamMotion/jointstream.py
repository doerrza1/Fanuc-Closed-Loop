#Joint Stram Motion Test for Fanuc LR Mate 200iD

import socket
import time
from robot.StreamMotion.src.utils import *
from robot.StreamMotion.src.client import *
import numpy as np

# signal definition
t_max = 2000 
t = np.linspace(start=0, stop=t_max-1, num=t_max)
signal = 1e-4*t


# UDP initialization
client = UDPClient("192.168.0.3")
client.connect()

# Send Initialization pack, Sets resp to the Status Packet
resp = client.send_init_pack()

#Parses current joint position data from resp and prints the postitions
current_jnt_data = resp[18:27] 
print(['%.4f' % jnt for jnt in current_jnt_data]) 

for i, value in enumerate(signal):
  
  jnt_data = current_jnt_data
  jnt_data[3] += value #increments joint # by value in signal
  
  if (i == 0):
    #list going to command pack = [sequence_data, last_command_data, cartesian = 0 or joint = 1, [joint postions]]
    data = commandpack([1, 0, 1, jnt_data])  
    print('COMMAND PACK SENT:', ['%.4f' % jnt for jnt in jnt_data])
    print('Sent Seq No:', [1,0,1])
    
  elif (i < len(signal)-1):
    data = commandpack([resp[2], 0, 1, jnt_data])
    print('COMMAND PACK SENT:', ['%.4f' % jnt for jnt in jnt_data])
    print('Sent Seq No:', [resp[2],0,1])
  
  else:
    data = commandpack([resp[2], 1, 1, jnt_data])
    print('COMMAND PACK SENT:', ['%.4f' % jnt for jnt in jnt_data])
    print('Sent Seq No:', [resp[2], 1, 1])

# Sends command pack and recieves status packet   
  resp = client.send_command_pack(data)
  
  # print('({:.2f}ms)'.format(1000*(time.time()-start_time)))
  
#resp = explainRobData(resp) already called within the send command pack function
  
  print('Received Seq No:', resp[2])
  
  current_jnt_data = resp[18:27]

client.send_end_pack()
print("End of stream")








