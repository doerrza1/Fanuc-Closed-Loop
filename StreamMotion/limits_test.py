# Testing script for obtaining the limits from Robot

import numpy as np
import csv

from src.limit import *
from src.client import *
from src.utils import *

client = UDPClient("192.168.0.3")
client.connect()
print("Connection Established to Robot")


limit_matrices, max_velo = obtain_limits(client) # Obtain the no payload/max payload limit matrices

no_load_vel = limit_matrices[0]
max_load_vel = limit_matrices[1]
no_load_acc = limit_matrices[2]
max_load_acc = limit_matrices[3]
no_load_jerk = limit_matrices[4]
max_load_jerk = limit_matrices[5]

with open('limit_table.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    
    # Header
    writer.writerow(["J1 No Load Vel", "J1 Max Load Vel","J1 No Load Acc", "J1 Max Load Acc", "J1 No Load Jerk", "J1 Max Load Jerk",
                     "J2 No Load Vel", "J2 Max Load Vel","J2 No Load Acc", "J2 Max Load Acc", "J2 No Load Jerk", "J2 Max Load Jerk",
                     "J3 No Load Vel", "J3 Max Load Vel","J3 No Load Acc", "J3 Max Load Acc", "J3 No Load Jerk", "J3 Max Load Jerk",
                     "J4 No Load Vel", "J4 Max Load Vel","J4 No Load Acc", "J4 Max Load Acc", "J4 No Load Jerk", "J4 Max Load Jerk",
                     "J5 No Load Vel", "J5 Max Load Vel","J5 No Load Acc", "J5 Max Load Acc", "J5 No Load Jerk", "J5 Max Load Jerk",
                     "J6 No Load Vel", "J6 Max Load Vel","J6 No Load Acc", "J6 Max Load Acc", "J6 No Load Jerk", "J6 Max Load Jerk",
                     ])
    
    # Write data rows
    for row in zip(no_load_vel[0], max_load_vel[0], no_load_acc[0], max_load_acc[0], no_load_jerk[0], max_load_jerk[0],
                   no_load_vel[1], max_load_vel[1], no_load_acc[1], max_load_acc[1], no_load_jerk[1], max_load_jerk[1],
                   no_load_vel[2], max_load_vel[2], no_load_acc[2], max_load_acc[2], no_load_jerk[2], max_load_jerk[2],
                   no_load_vel[3], max_load_vel[3], no_load_acc[3], max_load_acc[3], no_load_jerk[3], max_load_jerk[3],
                   no_load_vel[4], max_load_vel[4], no_load_acc[4], max_load_acc[4], no_load_jerk[4], max_load_jerk[4],
                   no_load_vel[5], max_load_vel[5], no_load_acc[5], max_load_acc[5], no_load_jerk[5], max_load_jerk[5],
                   ):
        
        writer.writerow(row)

print(len(limit_matrices))
print(limit_matrices)
