from src.limit import *
from src.client import *
from src.utils import *
from src.display import *

client = UDPClient("192.168.0.3")
client.connect()
print("Connection Established to Robot")

for i in range(1,7): # Iterate through each joint
    for j in range(3): # 3 iterations per joint
        if (j == 0):
            resp = client.send_vel_pack(i)

        elif (j == 1):
            resp = client.send_acc_pack(i)

        else:
            resp = client.send_jerk_pack(i)

    display_limit_pack(resp)
        


