# obtain the joint angles
from src.client import *
from src.utils import *
from src.display import *
from src.radius import *
import numpy as np

client = UDPClient("192.168.0.3")
client.connect()

resp = client.send_init_pack()

display_jnt_pack(resp)
