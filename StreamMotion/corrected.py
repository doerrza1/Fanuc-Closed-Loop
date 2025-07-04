#Joint Stram Motion Test for Fanuc LR Mate 200iD

# Joint 3 motion for testing of making a circular motion

import socket
import time
from src.utils import *
from src.client import *
import numpy as np
import matplotlib.pyplot as plot

# signal definition for j1  (0 -> 45 -> 0 -> -45 -> 0)

    #using linspace creates an array of numbers evenly spaced between start and stop
    #causes the robot to accelerate (start = 0, stop = 45) or decelerate(start = 45, stop = 0)
    #by incrementing by a higher (acc) or lower (dec)

    #positive increment from center to end
jnt_motion1_acc = np.linspace(start = 0, stop = 1, num = 1000)
jnt_motion1_acc = jnt_motion1_acc *(45/sum(jnt_motion1_acc))

jnt_motion1_dec = np.linspace(start = 1, stop = 0, num = 1000)
jnt_motion1_dec = jnt_motion1_dec * (45/sum(jnt_motion1_dec))

print(sum(jnt_motion1_acc))
print(sum(jnt_motion1_dec))

    #negative increment from end to end
jnt_motion2_acc = np.linspace(start = 0, stop = 1, num = 2000)
jnt_motion2_acc = jnt_motion2_acc * (90/sum(jnt_motion2_acc))

jnt_motion2_dec = np.linspace(start = 45, stop = 0, num = 2000)
jnt_motion2_dec = jnt_motion2_dec * (90/sum(jnt_motion2_dec))

print(sum(jnt_motion2_acc))
print(sum(jnt_motion2_dec))
    #postive increment from end to center

jnt_motion3_acc = jnt_motion1_acc

jnt_motion3_dec = jnt_motion1_dec

jnt_motion1 = np.append(jnt_motion1_acc, jnt_motion1_dec)
jnt_motion2 = np.append(jnt_motion2_acc, jnt_motion2_dec)
jnt_motion3 = np.append(jnt_motion3_acc, jnt_motion3_dec)

signal_1 = np.append(jnt_motion1, jnt_motion2)
signal_1 = np.append(signal_1, jnt_motion3)

signal_1 = signal_1/2*5

print(sum(signal_1))

#signal definition for j3
    #positive increment from bottom to  top
jnt_up_acc = np.linspace(start = 0, stop = 1, num = 2000) #0 -> 90
jnt_up_acc = jnt_up_acc * (45/sum(jnt_up_acc))


jnt_up_dec = np.linspace(start = 45, stop = 0, num = 2000)  #90-> 180
jnt_up_dec = jnt_up_dec * (45/sum(jnt_up_dec))
    #negative increment from top to bottom
jnt_down_acc = jnt_up_acc     #180 -> 270 
jnt_down_dec = jnt_up_dec     #270 -> 360

jnt_up = np.append(jnt_up_acc, jnt_up_dec)
jnt_down = np.append(jnt_down_acc, jnt_down_dec)

print("up =", sum(jnt_up))
print(sum(jnt_down))
signal_3 = np.append(jnt_up, jnt_down)
signal_3 = signal_3/2 *5

print(sum(signal_3))

# UDP initialization
client = UDPClient("192.168.0.3")
client.connect()

# Send Initialization pack, Sets resp to the Status Packet
resp = client.send_init_pack()

#Parses current joint position data from resp and prints the postitions
current_jnt_data = resp[18:27] 
print(['%.4f' % jnt for jnt in current_jnt_data]) 

for i, value in enumerate(zip(signal_1, signal_3)):
  
  jnt_data = current_jnt_data
  
  if (i == 0):
    #list going to command pack = [sequence_data, last_command_data, cartesian = 0 or joint = 1, [joint postions]]
    data = commandpack([1, 0, 1, jnt_data])  
    print('COMMAND PACK SENT:', ['%.4f' % jnt for jnt in jnt_data])
    print('Sent Seq No:', [1,0,1])
    
  elif ((0 < i) and (i <= 2000)):
    jnt_data[0] += value[0] #increments joint 1 by value in signal_1
    jnt_data[2] += value[1] #increments joint 3 by value in signal_3

    data = commandpack([resp[2], 0, 1, jnt_data])
    if ((i % 100 == 0)):
      print('COMMAND PACK SENT:', ['%.4f' % jnt for jnt in jnt_data])
      print('Sent Seq No:', [resp[2],0,1])

  elif ((i > 2000) and (i <= 4000)):
    jnt_data[0] -= value[0] #increments joint 1 by value in signal_1
    jnt_data[2] += value[1] #increments joint 3 by value in signal_3
    
    data = commandpack([resp[2], 0, 1, jnt_data])
    if ((i % 100 == 0)):
      print('COMMAND PACK SENT:', ['%.4f' % jnt for jnt in jnt_data])
      print('Sent Seq No:', [resp[2],0,1])
  
  elif ((i > 4000) and (i <= 6000)):
    jnt_data[0] -= value[0] #increments joint 1 by value in signal_1
    jnt_data[2] -= value[1] #increments joint 3 by value in signal_3
    
    data = commandpack([resp[2], 0, 1, jnt_data])
    if ((i % 100 == 0)):
      print('COMMAND PACK SENT:', ['%.4f' % jnt for jnt in jnt_data])
      print('Sent Seq No:', [resp[2],0,1])
  
  elif ((i > 6000) and (i < 8000)):
    jnt_data[0] += value[0] #increments joint 1 by value in signal_1
    jnt_data[2] -= value[1] #increments joint 3 by value in signal_3
    
    data = commandpack([resp[2], 0, 1, jnt_data])
    if ((i % 100 == 0)):
      print('COMMAND PACK SENT:', ['%.4f' % jnt for jnt in jnt_data])
      print('Sent Seq No:', [resp[2],0,1])

  else:
    data = commandpack([resp[2], 1, 1, jnt_data])
    print('COMMAND PACK SENT:', ['%.4f' % jnt for jnt in jnt_data])
    print('Sent Seq No:', [resp[2], 1, 1])

  # Sends command pack and recieves status packet   
  resp = client.send_command_pack(data)

  print('Received Seq No:', resp[2])
  
  current_jnt_data = resp[18:27]

client.send_end_pack()
print("End of stream")








