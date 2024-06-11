# Cartesian Circle Motion for Fanuc LR Mate 200 iD

import socket
import time
from src.utils import *
from src.client import *
import numpy as np
import math


# signal definition for a circle
radius = int(input("Enter radius for circle: "))
t_max = 4000
start_degrees = 0
end_degrees = 360
signal = np.linspace(start=start_degrees, stop=end_degrees, num=t_max)



# Initialize connection to robot
client = UDPClient("192.168.0.3")
client.connect()

# Send Initialization pack, Sets resp to the Status Packet
resp = client.send_init_pack()

#Parses current cartesian position data from resp and prints the postitions
current_car_data = resp[9:15] 
print(['%.4f' % jnt for jnt in current_car_data]) 

x_init = current_car_data[0]  #initial x position
z_init = current_car_data[2]  #initial z position

x_start = -1*(radius + x_init) #x value for the first radius

# Linear signal definition (moves arm to starting radius)

linear = np.linspace(start = x_init, end = x_start, num = t_max)

for i, value in enumerate(linear):
  
  car_data = current_car_data
  
  if (i == 0):
    data = commandpack([1, 0, 0, car_data])  
    print('COMMAND PACK SENT:', ['%.4f' % jnt for jnt in car_data])
    print('Sent Seq No:', [1,0,1])
  
  else:
    car_data[0] = value
    print(resp[2])
    data = commandpack([resp[2], 0, 0, car_data])
    print('COMMAND PACK SENT:', ['%.4f' % jnt for jnt in car_data])
    print('Sent Seq No:', [resp[2],0,1])

  resp = client.send_command_pack(data)
  
  print('Received Seq No:', resp[2])
  
  current_car_data = resp[9:15]

for i, value in enumerate(signal):
  
  car_data = current_car_data
  car_data[0] = -1*(radius*math.cos(math.radians(value)) + x_init) #increments x-axis by value in signal
  car_data[2] = radius*math.sin(math.radians(value)) + z_init #increments z-axis by value in signal
  
#   if (i == 0):
#     data = commandpack([1, 0, 0, car_data])  
#     print('COMMAND PACK SENT:', ['%.4f' % jnt for jnt in car_data])
#     print('Sent Seq No:', [1,0,1])
    
  if (i < len(signal)-1):
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
  
  # resp = explainRobData(resp) already called within send_command_pack
  
  print('Received Seq No:', resp[2])
  
  current_jnt_data = resp[9:15]

client.send_end_pack()
print("End of stream")

