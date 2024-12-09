# Cartesian Circle Motion for Fanuc LR Mate 200 iD

# Attempt at creating circle using a bell curve for acceleration

import socket
import time
from robot.StreamMotion.src.utils import *
from robot.StreamMotion.src.client import *
import numpy as np
import math


# signal definition for a circle
radius = int(input("Enter radius for circle: "))
# Parameters for the bell curve
mean = 180       # Center of the range [0, 360]
std_dev = 60     # Standard deviation to spread the values
num_points = 2000  # Numbers of points in the array
desired_sum = 8000
desired_area = 360

# Generate the bell curve data with a normal distribution
bell_curve = np.random.normal(loc=mean, scale=std_dev, size=num_points)

# Normalize the bell curve to have an integral of 1
bell_curve -= np.min(bell_curve)  # Shift to positive values
bell_curve /= np.sum(bell_curve)  # Normalize to make the sum equal to 1

# Scale the bell curve to have the desired sum and area
bell_curve *= desired_sum / np.sum(bell_curve)  # Ensure the sum equals 8000
scaling_factor = desired_area / np.trapz(bell_curve, dx=1)
bell_curve *= scaling_factor  # Scale to ensure the area under the curve equals 360



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


for i, value in enumerate(bell_curve):
  
  car_data = current_car_data
  car_data[0] = -1*(radius*math.cos(math.radians(value))-radius) #increments x-axis by value in signal
  car_data[2] = radius*math.sin(math.radians(value)) + z_init #increments z-axis by value in signal
  
  if (i == 0):
    data = commandpack([1, 0, 0, car_data])  
    print('COMMAND PACK SENT:', ['%.4f' % jnt for jnt in car_data])
    print('Sent Seq No:', [1,0,1])
    
  elif (i < len(bell_curve)-1):
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

