# Fanuc Stream Motion Test to move end effector to straight position
# Just change pitch to 
from src.client import *
from src.utils import *
from src.display import *
from src.plot import *
import numpy as np

sig = 250
start1 = 0
end1 = 90

# Signal definition
acc_base = np.linspace(1, sig, sig)**2
acc_base = acc_base / acc_base.sum() * (start1 - end1)
signal = start1 - np.cumsum(acc_base)

client = UDPClient("192.168.0.3")
client.connect()

# Send Initialization pack, Sets resp to the Status Packet
resp = client.send_init_pack()

#Parses current cartesian position data from resp and prints the postitions
current_car_data = resp[9:18] 

for i, value in enumerate(signal):
  
  car_data = current_car_data
  car_data[2] += value #increments z-axis by value in signal
  
  if (i == 0):
    data = commandpack([1, 0, 0, car_data])  
    #print('COMMAND PACK SENT:', ['%.4f' % jnt for jnt in car_data])
    #print('Sent Seq No:', [1,0,1])
    
  elif (i < len(signal)-1):
    data = commandpack([resp[2], 0, 0, car_data])
    #print('COMMAND PACK SENT:', ['%.4f' % jnt for jnt in car_data])
    #print('Sent Seq No:', [resp[2],0,1])
    

  else:
    data = commandpack([resp[2], 1, 0, car_data])
    #print('COMMAND PACK SENT:', ['%.4f' % jnt for jnt in car_data])
    #print('Sent Seq No:', [resp[2], 1, 1])

# Sends command pack and recieves status packet   
  resp = client.send_command_pack(data)
  if (i % 25 == 0):
    display_cmd_pack(resp)

  # print('({:.2f}ms)'.format(1000*(time.time()-start_time)))
  
  # resp = explainRobData(resp) already called within send_command_pack
  
  #print('Received Seq No:', resp[2])
  
  current_car_data = resp[9:18]

client.send_end_pack()
print("End of stream")